"""
用户管理和认证核心模块。

从 main.py 中提取 UserManager 类，保持所有现有逻辑不变：
- PBKDF2 密码哈希
- In-memory session tokens
- users.json 读写
- 用户目录初始化
"""

from __future__ import annotations

import hashlib
import hmac
import json
import logging
import secrets
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional

from config import SYSTEM_FILE, USER_CATEGORIES, USER_DATA_DIR


logger = logging.getLogger(__name__)


def safe_filename(name: str) -> str:
    """Return the basename only, preventing path traversal."""
    return Path(str(name).replace("\\", "/")).name


def get_user_dir(username: str, category: str) -> Path:
    """Return and create the local data directory for one user/category."""
    from config import USER_CATEGORY_DIRS

    if category not in USER_CATEGORY_DIRS:
        raise ValueError("Invalid category")
    target = USER_DATA_DIR / username / USER_CATEGORY_DIRS[category]
    target.mkdir(parents=True, exist_ok=True)
    return target


def now_text() -> str:
    """Return an ISO-like local timestamp for lightweight user metadata."""
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


class UserManager:
    """Minimal local JSON user store used by the basic no-database deployment."""

    def __init__(self) -> None:
        self.users: Dict[str, Dict[str, Any]] = {}
        self.sessions: Dict[str, str] = {}
        self._load()

    def _load(self) -> None:
        if SYSTEM_FILE.exists():
            try:
                self.users = json.loads(SYSTEM_FILE.read_text(encoding="utf-8"))
            except json.JSONDecodeError:
                logger.warning("users.json is invalid; starting with an empty local user store.")
                self.users = {}
        if "admin" not in self.users:
            self.users["admin"] = self._build_user_record("admin", "admin", role="admin")
            self._save()
        if USER_DATA_DIR.exists():
            changed = False
            for user_dir in USER_DATA_DIR.iterdir():
                if not user_dir.is_dir() or user_dir.name in self.users:
                    continue
                username = safe_filename(user_dir.name).strip()
                if not username:
                    continue
                self.users[username] = self._build_user_record(username, username)
                changed = True
            if changed:
                self._save()
        for username in self.users:
            self.init_user_dirs(username)

    def _save(self) -> None:
        SYSTEM_FILE.write_text(json.dumps(self.users, indent=2, ensure_ascii=False), encoding="utf-8")

    def _hash_password(self, password: str, salt: Optional[str] = None) -> tuple[str, str]:
        salt = salt or secrets.token_hex(16)
        digest = hashlib.pbkdf2_hmac("sha256", password.encode("utf-8"), salt.encode("utf-8"), 120000)
        return salt, digest.hex()

    def _build_user_record(self, username: str, password: str, role: str = "user") -> Dict[str, Any]:
        salt, digest = self._hash_password(password)
        created = now_text()
        return {
            "password_hash": digest,
            "salt": salt,
            "role": role,
            "display_name": username,
            "email": "",
            "phone": "",
            "notes": "",
            "created_at": created,
            "updated_at": created,
            "last_login": None,
        }

    def init_user_dirs(self, username: str) -> None:
        """Create the data folders expected by the frontend."""
        for category in USER_CATEGORIES:
            get_user_dir(username, category)

    def authenticate(self, username: str, password: str) -> Optional[Dict[str, Any]]:
        """Validate username/password against the local JSON store."""
        record = self.users.get(username)
        if not record:
            return None
        _, digest = self._hash_password(password, record.get("salt"))
        if not hmac.compare_digest(record.get("password_hash", ""), digest):
            return None
        record["last_login"] = now_text()
        self._save()
        return record

    def register(self, username: str, password: str) -> bool:
        """Register a normal local user and initialize their data folders."""
        username = safe_filename(username).strip()
        if not username or username in self.users:
            return False
        self.users[username] = self._build_user_record(username, password)
        self.init_user_dirs(username)
        self._save()
        return True

    def create_session(self, username: str) -> str:
        """Create a bearer-like random session token for frontend headers."""
        token = secrets.token_urlsafe(32)
        self.sessions[token] = username
        return token

    def validate_session(self, token: Optional[str]) -> Optional[str]:
        """Return the username for a session token, or None when invalid."""
        if not token:
            return None
        return self.sessions.get(token)

    def rename_user(self, old_username: str, new_username: str) -> tuple[bool, str]:
        """Rename a local user and move their local data directory."""
        new_username = safe_filename(new_username).strip()
        if not new_username:
            return False, "用户名不能为空"
        if new_username == old_username:
            return True, old_username
        if new_username in self.users:
            return False, "用户名已存在"
        old_dir = USER_DATA_DIR / old_username
        new_dir = USER_DATA_DIR / new_username
        if new_dir.exists():
            return False, "目标用户目录已存在"
        record = self.users.pop(old_username, None)
        if not record:
            return False, "用户不存在"
        self.users[new_username] = record
        if old_dir.exists():
            old_dir.rename(new_dir)
        self.init_user_dirs(new_username)
        for token, username in list(self.sessions.items()):
            if username == old_username:
                self.sessions[token] = new_username
        self._save()
        return True, new_username

    def update_profile(
        self,
        username: str,
        display_name: str = "",
        email: str = "",
        phone: str = "",
        notes: str = "",
    ) -> None:
        """Update optional display fields in the local user record."""
        record = self.users.setdefault(username, self._build_user_record(username, secrets.token_urlsafe(12)))
        record["display_name"] = display_name.strip() or username
        record["email"] = email.strip()
        record["phone"] = phone.strip()
        record["notes"] = notes.strip()
        record["updated_at"] = now_text()
        self._save()

    def get_profile(self, username: str, search: Optional[str] = None) -> Dict[str, Any]:
        """Build a frontend-friendly profile and local file inventory."""
        record = self.users.get(username, {})
        files: list[Dict[str, Any]] = []
        for category in USER_CATEGORIES:
            folder = get_user_dir(username, category)
            for file_path in folder.iterdir():
                if file_path.is_file() and not file_path.name.startswith("."):
                    files.append({
                        "name": file_path.name,
                        "owner": username,
                        "category": category,
                        "size": f"{file_path.stat().st_size / 1024 / 1024:.2f} MB",
                        "size_bytes": file_path.stat().st_size,
                        "mtime": file_path.stat().st_mtime,
                    })
        files.sort(key=lambda item: item["mtime"], reverse=True)
        if search:
            keyword = search.lower()
            files = [item for item in files if keyword in item["name"].lower()]
        counts = {category: 0 for category in USER_CATEGORIES}
        for item in files:
            counts[item["category"]] += 1
        total_size_bytes = int(sum(int(item["size_bytes"]) for item in files))
        return {
            "username": username,
            "display_name": record.get("display_name") or username,
            "role": record.get("role", "user"),
            "email": record.get("email") or "",
            "phone": record.get("phone") or "",
            "notes": record.get("notes") or "",
            "last_login": record.get("last_login"),
            "created_at": record.get("created_at"),
            "updated_at": record.get("updated_at"),
            "file_counts": counts,
            "total_files": int(sum(counts.values())),
            "total_size": f"{total_size_bytes / 1024 / 1024:.2f} MB",
            "total_size_bytes": total_size_bytes,
            "files": files,
        }


# 全局单例
user_manager = UserManager()
