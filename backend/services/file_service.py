"""
文件服务：用户文件 CRUD、搜索匹配等。
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Any, Dict, List, Optional

from backend.core.security import get_user_dir, safe_filename, user_manager


def list_user_files(username: str, category: str) -> List[Dict[str, Any]]:
    """列出用户某类别下的所有文件。"""
    files = []
    for file_path in get_user_dir(username, category).iterdir():
        if file_path.is_file() and not file_path.name.startswith("."):
            files.append({
                "name": file_path.name,
                "owner": username,
                "size": f"{file_path.stat().st_size / 1024 / 1024:.2f} MB",
                "mtime": file_path.stat().st_mtime,
            })
    files.sort(key=lambda item: item["mtime"], reverse=True)
    return files


def find_user_file(username: str, category: str, filename: str) -> Optional[Path]:
    """在用户目录中查找一个文件。"""
    candidate = get_user_dir(username, category) / safe_filename(filename)
    return candidate if candidate.exists() else None


def extract_tower_tokens(name: str) -> set[str]:
    """从文件名中提取杆塔编号用于匹配。"""
    stem = Path(name).stem
    hash_tokens = {token.lstrip("0") or "0" for token in re.findall(r"#\s*0*(\d{1,6})", stem)}
    if hash_tokens:
        return hash_tokens
    return {token.lstrip("0") or "0" for token in re.findall(r"(?<![A-Za-z])0*(\d{2,6})(?![A-Za-z])", stem)}


def manual_route_match_score(scene_name: str, route_path: Path) -> int:
    """计算人工航线和点云场景的匹配分数。"""
    if route_path.stem == scene_name:
        return 100
    if route_path.stem.startswith(scene_name):
        return 80
    scene_tokens = extract_tower_tokens(scene_name)
    if not scene_tokens:
        return 0
    route_hash_tokens = {token.lstrip("0") or "0" for token in re.findall(r"#\s*0*(\d{1,6})", route_path.stem)}
    if scene_tokens & route_hash_tokens:
        return 60
    route_tokens = extract_tower_tokens(route_path.stem)
    return 40 if scene_tokens & route_tokens else 0


def find_matching_manual_route(username: str, scene_name: str) -> Optional[Path]:
    """为一个点云场景查找匹配的人工航线。"""
    exact = find_user_file(username, "manual_route", f"{scene_name}.json")
    if exact:
        return exact
    candidates: List[tuple[int, Path]] = []
    manual_dir = get_user_dir(username, "manual_route")
    for route_path in manual_dir.glob("*.json"):
        score = manual_route_match_score(scene_name, route_path)
        if score > 0:
            candidates.append((score, route_path))
    if not candidates:
        return None
    best_score = max(score for score, _ in candidates)
    best = sorted({path for score, path in candidates if score == best_score})
    return best[0] if len(best) == 1 else None


def get_profile(username: str, search: Optional[str] = None) -> Dict[str, Any]:
    """获取用户资料（含文件清单）。"""
    return user_manager.get_profile(username, search=search)


def update_user_profile(username: str, **fields) -> None:
    """更新用户资料字段。"""
    user_manager.update_profile(username, **fields)


def rename_user(old_username: str, new_username: str) -> tuple[bool, str]:
    """重命名用户。"""
    return user_manager.rename_user(old_username, new_username)
