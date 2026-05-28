"""
从 backend/ 模块重建原始单体 main.py。

所有原始代码都保留在 backend/ 中，此脚本将它们重新组装为原始格式。
"""

import re
from pathlib import Path

BACKEND = Path(__file__).resolve().parent.parent / "backend"


def read_module(path: str) -> str:
    """读取模块文件内容，移除 doctstring 和 import 语句。"""
    text = (BACKEND / path).read_text(encoding="utf-8")
    # 移除模块级 docstring
    text = re.sub(r'^"""[^"]*"""\s*', '', text, flags=re.DOTALL)
    text = text.strip()
    # 移除 from __future__ 行
    text = re.sub(r'from __future__ import.*\n', '', text)
    # 移除顶层 import 的 backend.* 路径导入（保留标准库和第三方导入）
    # 注意：局部函数内的 import 要保留
    lines = text.split('\n')
    result = []
    for line in lines:
        stripped = line.strip()
        # 跳过顶层 backend 导入（函数内的 from backend.xxx import 保留）
        if stripped.startswith('from backend.') and not stripped.startswith('from backend.'):
            continue
        if stripped.startswith('import backend.'):
            continue
        result.append(line)
    return '\n'.join(result).strip()


def extract_class(text: str, class_name: str) -> str:
    """提取类定义（包括所有方法）。"""
    pattern = rf'class {class_name}\('
    idx = text.find(pattern)
    if idx == -1:
        return ''
    # 找到类定义开始处
    lines = text[idx:].split('\n')
    class_lines = []
    indent_level = None

    for i, line in enumerate(lines):
        if i == 0:
            class_lines.append(line)
            continue
        if indent_level is None and line and line[0] in (' ', '\t'):
            indent_level = len(line) - len(line.lstrip())

        if line.strip() and not line[0].isspace() and indent_level is not None:
            break  # 退出类定义（顶层语句）

        if not line.strip():
            if class_lines and class_lines[-1].strip():
                class_lines.append(line)
            continue

        class_lines.append(line)

    return '\n'.join(class_lines)


def extract_function(text: str, func_name: str) -> str:
    """提取单个函数定义。"""
    pattern = rf'^(def {func_name}\()'
    found = re.search(pattern, text, re.MULTILINE)
    if not found:
        return ''

    idx = found.start()
    lines = text[idx:].split('\n')
    func_lines = []

    for i, line in enumerate(lines):
        if i > 0 and line.strip() and not line[0].isspace():
            break
        func_lines.append(line)

    return '\n'.join(func_lines)


def main():
    # 读取所有模块
    security = (BACKEND / "core" / "security.py").read_text(encoding="utf-8")
    services_proc = (BACKEND / "services" / "processing_service.py").read_text(encoding="utf-8")
    services_file = (BACKEND / "services" / "file_service.py").read_text(encoding="utf-8")
    services_route = (BACKEND / "services" / "route_service.py").read_text(encoding="utf-8")

    # 路由层（只提取函数体，不提取装饰器）
    routers = {}
    router_dir = BACKEND / "routers"
    for rf in sorted(router_dir.glob("*.py")):
        if rf.name == "__init__.py" or rf.name == "ws.py":
            continue
        routers[rf.stem] = rf.read_text(encoding="utf-8")

    # 构建原始 main.py
    result = []

    # === 第 1 部分：顶层导入 ===
    result.append(
        '''from __future__ import annotations

import hashlib
import hmac
import json
import logging
import re
import secrets
import shutil
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
from fastapi import BackgroundTasks, Body, FastAPI, File, Form, Header, UploadFile
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from starlette.requests import Request

from config import (
    LOG_DIR,
    STATIC_DIR,
    SYSTEM_FILE,
    TEMPLATE_DIR,
    USER_CATEGORIES,
    USER_CATEGORY_DIRS,
    USER_DATA_DIR,
)
from waypoint_planning.algorithms import (
    HRLSolver,
    Pretreatment,
    SemanticSingleLayerRLPlanner,
    SemanticWeightedGreedyPlanner,
    compute_waypoint_metrics,
    evaluate_waypoint_coverage,
    parse_manual_route,
    process_status,
    read_las_for_vis,
)
from waypoint_planning.planning_solvers import run_waypoint_planning
from waypoint_planning.route_planner import plan_route_from_waypoints, validate_route_safety
from waypoint_planning.waypoint_models import WaypointPlanningInput'''
    )

    # === 第 2 部分：日志和 app 初始化 ===
    result.append('''

LOG_DIR.mkdir(exist_ok=True)
USER_DATA_DIR.mkdir(exist_ok=True)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler(LOG_DIR / "app.log", encoding="utf-8"),
    ],
)
logger = logging.getLogger(__name__)

app = FastAPI(title="Power Grid Inspection Basic Deployment")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")
templates = Jinja2Templates(directory=str(TEMPLATE_DIR))''')

    # === 第 3 部分：PLANNER_REGISTRY + 约束构建 ===
    result.append('''

PLANNER_REGISTRY: Dict[str, Dict[str, Any]] = {
    SemanticWeightedGreedyPlanner.planner_name: {
        "name": SemanticWeightedGreedyPlanner.planner_name,
        "solver": SemanticWeightedGreedyPlanner,
        "description": "基线算法：按覆盖增益、安全距离、关键目标和路径代价选择多焦段视点。",
        "parameters": [
            {"key": "safety_distance_m", "label": "安全距离(m)", "type": "number",
             "default": 5.0, "min": 2.5, "max": 10.0, "step": 0.1},
        ],
    },
    SemanticSingleLayerRLPlanner.planner_name: {
        "name": SemanticSingleLayerRLPlanner.planner_name,
        "solver": SemanticSingleLayerRLPlanner,
        "description": "单层强化学习策略：在统一奖励函数下优化覆盖率、航点数和安全距离。",
        "parameters": [
            {"key": "safety_distance_m", "label": "安全距离(m)", "type": "number",
             "default": 5.0, "min": 2.5, "max": 10.0, "step": 0.1},
            {"key": "single_layer_episodes", "label": "训练轮次", "type": "number",
             "default": 10, "min": 1, "max": 50, "step": 1},
        ],
    },
    HRLSolver.planner_name: {
        "name": HRLSolver.planner_name,
        "solver": HRLSolver,
        "description": "分层强化学习策略：高层选择语义关注区域，低层选择安全航点和拍摄动作。",
        "parameters": [
            {"key": "safety_distance_m", "label": "安全距离(m)", "type": "number",
             "default": 5.0, "min": 2.5, "max": 10.0, "step": 0.1},
            {"key": "hierarchical_episodes", "label": "训练轮次", "type": "number",
             "default": 10, "min": 1, "max": 50, "step": 1},
        ],
    },
}''')

    # === build_planner_constraints_payload ===
    result.append('''

def build_planner_constraints_payload(
    safety_distance_m: Optional[float] = None,
    conductor_no_fly_enabled: Optional[bool] = None,
    conductor_no_fly_extent_margin_m: Optional[float] = None,
    conductor_no_fly_min_length_m: Optional[float] = None,
    conductor_no_fly_boundary_tolerance_m: Optional[float] = None,
    max_waypoints: Optional[int] = None,
    max_shots_per_waypoint: Optional[int] = None,
    single_layer_episodes: Optional[int] = None,
    hierarchical_episodes: Optional[int] = None,
    manual_ratio_min: Optional[float] = None,
    manual_ratio_max: Optional[float] = None,
    target_manual_ratio: Optional[float] = None,
) -> dict:
    """Build planner constraints from API form values while keeping UI aliases stable."""
    constraints = {
        key: value
        for key, value in {
            "safety_distance_m": safety_distance_m,
            "conductor_no_fly_enabled": conductor_no_fly_enabled,
            "conductor_no_fly_extent_margin_m": conductor_no_fly_extent_margin_m,
            "conductor_no_fly_min_length_m": conductor_no_fly_min_length_m,
            "conductor_no_fly_boundary_tolerance_m": conductor_no_fly_boundary_tolerance_m,
            "max_waypoints": max_waypoints,
            "max_shots_per_waypoint": max_shots_per_waypoint,
            "single_layer_episodes": single_layer_episodes,
            "hierarchical_episodes": hierarchical_episodes,
            "manual_ratio_min": manual_ratio_min,
            "manual_ratio_max": manual_ratio_max,
        }.items()
        if value is not None
    }
    if target_manual_ratio is not None:
        ratio = max(0.60, min(0.80, float(target_manual_ratio)))
        constraints["manual_ratio_min"] = round(ratio, 2)
        constraints["manual_ratio_max"] = round(ratio, 2)
    return constraints''')

    # === clamp_optional_float + build_route_clearance_payload ===
    result.append('''

def clamp_optional_float(value: Optional[float], default: float, low: float, high: float) -> float:
    """Clamp an optional form float with a stable default."""
    if value is None:
        value = default
    return max(low, min(high, float(value)))


def build_route_clearance_payload(
    safety_distance_m: Optional[float] = None,
    clearance_m: Optional[float] = None,
    wire_clearance_m: Optional[float] = None,
    tower_clearance_m: Optional[float] = None,
    task_tower_clearance_m: Optional[float] = None,
    task_wire_clearance_m: Optional[float] = None,
) -> dict:
    """Normalize route safety input while accepting the waypoint planner's safety_distance_m format."""
    unified = None if safety_distance_m is None else max(0.5, min(40.0, float(safety_distance_m)))
    default_tower = unified if unified is not None else 6.0
    default_wire = unified if unified is not None else 10.0
    default_task_tower = unified if unified is not None else 3.0
    default_task_wire = unified if unified is not None else 5.0
    tower_value = clearance_m if clearance_m is not None else tower_clearance_m
    return {
        "safety_distance_m": unified,
        "clearance_m": clamp_optional_float(tower_value, default_tower, 0.5, 30.0),
        "wire_clearance_m": clamp_optional_float(wire_clearance_m, default_wire, 0.5, 40.0),
        "task_tower_clearance_m": clamp_optional_float(task_tower_clearance_m, default_task_tower, 0.5, 30.0),
        "task_wire_clearance_m": clamp_optional_float(task_wire_clearance_m, default_task_wire, 0.5, 40.0),
    }''')

    # === safe_filename, get_user_dir, now_text ===
    result.append('''

def safe_filename(name: str) -> str:
    """Return the basename only, preventing path traversal through uploads or query params."""
    return Path(str(name).replace("\\\\", "/")).name


def get_user_dir(username: str, category: str) -> Path:
    """Return and create the local data directory for one user/category."""
    if category not in USER_CATEGORY_DIRS:
        raise ValueError("Invalid category")
    target = USER_DATA_DIR / username / USER_CATEGORY_DIRS[category]
    target.mkdir(parents=True, exist_ok=True)
    return target


def now_text() -> str:
    """Return an ISO-like local timestamp for lightweight user metadata."""
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")''')

    # === UserManager 类 ===
    # 从 security.py 提取 UserManager 类（去掉 import 和 from backend 引用）
    # security.py 中的 UserManager 应该是最完整的
    result.append('''

class UserManager:
    """Minimal local JSON user store used by the basic no-deployment database."""

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
            "password_hash": digest, "salt": salt, "role": role,
            "display_name": username, "email": "", "phone": "", "notes": "",
            "created_at": created, "updated_at": created, "last_login": None,
        }

    def init_user_dirs(self, username: str) -> None:
        for category in USER_CATEGORIES:
            get_user_dir(username, category)

    def authenticate(self, username: str, password: str) -> Optional[Dict[str, Any]]:
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
        username = safe_filename(username).strip()
        if not username or username in self.users:
            return False
        self.users[username] = self._build_user_record(username, password)
        self.init_user_dirs(username)
        self._save()
        return True

    def create_session(self, username: str) -> str:
        token = secrets.token_urlsafe(32)
        self.sessions[token] = username
        return token

    def validate_session(self, token: Optional[str]) -> Optional[str]:
        if not token:
            return None
        return self.sessions.get(token)

    def rename_user(self, old_username: str, new_username: str) -> tuple[bool, str]:
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
        self, username: str,
        display_name: str = "", email: str = "", phone: str = "", notes: str = "",
    ) -> None:
        record = self.users.setdefault(username, self._build_user_record(username, secrets.token_urlsafe(12)))
        record["display_name"] = display_name.strip() or username
        record["email"] = email.strip()
        record["phone"] = phone.strip()
        record["notes"] = notes.strip()
        record["updated_at"] = now_text()
        self._save()

    def get_profile(self, username: str, search: Optional[str] = None) -> Dict[str, Any]:
        record = self.users.get(username, {})
        files: List[Dict[str, Any]] = []
        for category in USER_CATEGORIES:
            folder = get_user_dir(username, category)
            for file_path in folder.iterdir():
                if file_path.is_file() and not file_path.name.startswith("."):
                    files.append({
                        "name": file_path.name, "owner": username, "category": category,
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


user_manager = UserManager()''')

    # === require_user, user_task_key, find_user_file ===
    result.append('''

def require_user(x_user_name: Optional[str] = None, x_auth_token: Optional[str] = None) -> Optional[str]:
    """Validate the frontend session headers and return the current username."""
    username = user_manager.validate_session(x_auth_token)
    if not username:
        return None
    if x_user_name and x_user_name != username:
        return None
    return username


def user_task_key(username: str, task: str) -> str:
    return f"{username}:{task}"


def find_user_file(username: str, category: str, filename: str) -> Optional[Path]:
    candidate = get_user_dir(username, category) / safe_filename(filename)
    return candidate if candidate.exists() else None''')

    # === 文件匹配逻辑 ===
    result.append('''

def extract_tower_tokens(name: str) -> set[str]:
    stem = Path(name).stem
    hash_tokens = {token.lstrip("0") or "0" for token in re.findall(r"#\\\\s*0*(\\\\d{1,6})", stem)}
    if hash_tokens:
        return hash_tokens
    return {token.lstrip("0") or "0" for token in re.findall(r"(?<![A-Za-z])0*(\\\\d{2,6})(?![A-Za-z])", stem)}


def manual_route_match_score(scene_name: str, route_path: Path) -> int:
    if route_path.stem == scene_name:
        return 100
    if route_path.stem.startswith(scene_name):
        return 80
    scene_tokens = extract_tower_tokens(scene_name)
    if not scene_tokens:
        return 0
    route_hash_tokens = {token.lstrip("0") or "0" for token in re.findall(r"#\\\\s*0*(\\\\d{1,6})", route_path.stem)}
    if scene_tokens & route_hash_tokens:
        return 60
    route_tokens = extract_tower_tokens(route_path.stem)
    return 40 if scene_tokens & route_tokens else 0


def find_matching_manual_route(username: str, scene_name: str) -> Optional[Path]:
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
    return best[0] if len(best) == 1 else None''')

    # === normalize_waypoints ===
    result.append('''

def normalize_waypoints(raw_waypoints: list) -> list:
    waypoints = []
    for index, waypoint in enumerate(raw_waypoints or []):
        pos = waypoint.get("position", waypoint.get("pos_utm"))
        if not pos:
            continue
        shots = waypoint.get("shots") or []
        primary = shots[0] if shots else waypoint
        waypoints.append({
            "id": index + 1,
            "pos_utm": pos,
            "pitch": primary.get("pitch", waypoint.get("pitch", 0)),
            "yaw": primary.get("yaw", waypoint.get("yaw", 0)),
            "focal_level": primary.get("focal_level", waypoint.get("focal_level")),
            "f_eq_mm": primary.get("f_eq_mm", waypoint.get("f_eq_mm")),
            "shot_count": len(shots) if shots else int(waypoint.get("shot_count", 1) or 1),
            "shots": shots,
            "action": waypoint.get("action", "fly"),
        })
    return waypoints''')

    # === Voxel matching ===
    result.append('''

def matching_voxel_names(route_filename: str) -> List[str]:
    stem = Path(safe_filename(route_filename)).stem
    candidates = [f"{stem}_voxel.npz"]
    for planner in sorted(PLANNER_REGISTRY.keys(), key=len, reverse=True):
        suffix = f"_{planner}"
        if stem.endswith(suffix):
            candidates.append(f"{stem[:-len(suffix)]}_voxel.npz")
    if "_" in stem:
        candidates.append(f"{stem.rsplit('_', 1)[0]}_voxel.npz")
    unique: List[str] = []
    for name in candidates:
        if name not in unique:
            unique.append(name)
    return unique


def find_matching_voxel(username: str, route_filename: str) -> Optional[Path]:
    for name in matching_voxel_names(route_filename):
        voxel_path = find_user_file(username, "voxel", name)
        if voxel_path:
            return voxel_path
    voxels = [
        path for path in get_user_dir(username, "voxel").iterdir()
        if path.is_file() and path.name.endswith("_voxel.npz")
    ]
    return voxels[0] if len(voxels) == 1 else None


def find_voxel_for_route(username: str, category: str, filename: str) -> Optional[Path]:
    if category == "algorithm_route":
        route_path = find_user_file(username, "algorithm_route", filename)
        if route_path:
            try:
                payload = json.loads(route_path.read_text(encoding="utf-8-sig"))
                source = payload.get("route_planning", {}).get("source_waypoint_file")
                if source:
                    voxel = find_matching_voxel(username, source)
                    if voxel:
                        return voxel
            except Exception:
                pass
    return find_matching_voxel(username, filename)''')

    # === attach_coverage_metrics, load_route_for_compare, run_task ===
    result.append('''

def attach_coverage_metrics(username: str, filename: str, waypoints: list, stats: dict) -> dict:
    voxel_path = find_matching_voxel(username, filename)
    if not voxel_path:
        return stats
    coverage = evaluate_waypoint_coverage(waypoints, str(voxel_path))
    merged = {**stats, **coverage}
    if merged.get("count") is None:
        merged["count"] = len(waypoints or [])
    return merged


def load_route_for_compare(username: str, category: str, filename: str) -> Dict[str, Any]:
    if category in {"manual_route", "algorithm_route"}:
        route_path = find_user_file(username, category, filename)
        if not route_path:
            raise FileNotFoundError(filename)
        waypoints = parse_manual_route(str(route_path))
        stats = attach_coverage_metrics(username, route_path.name, waypoints, compute_waypoint_metrics(waypoints))
        method = "算法航线" if category == "algorithm_route" else "人工航点"
        return {"category": category, "filename": route_path.name, "method": method, "method_name": method, "waypoints": waypoints, "stats": stats}
    if category == "waypoint":
        route_path = find_user_file(username, "waypoint", filename)
        if not route_path:
            raise FileNotFoundError(filename)
        result = json.loads(route_path.read_text(encoding="utf-8"))
        waypoints = normalize_waypoints(result.get("waypoints", []))
        raw_stats = result.get("stats", {})
        coverage = raw_stats.get("coverage", raw_stats.get("final_coverage"))
        stats = {**compute_waypoint_metrics(waypoints, coverage=coverage), **raw_stats}
        if stats.get("coverage") is None and stats.get("final_coverage") is not None:
            stats["coverage"] = stats["final_coverage"]
        stats["count"] = stats.get("count") or len(waypoints)
        if any(stats.get(key) is None for key in ("coverage_tower", "coverage_insulator", "C_geo", "C_top", "C_edge", "C_body")):
            stats = attach_coverage_metrics(username, route_path.name, waypoints, stats)
        item = {"category": category, "filename": route_path.name, "method": result.get("method", "算法航点"), "method_name": result.get("method_name", "算法航点"), "waypoints": waypoints, "stats": stats}
        for key in ("planning_input", "route_context", "tower_results"):
            if key in result:
                item[key] = result[key]
        return item
    raise ValueError("Unsupported route category")


def run_task(func: Any, task_key: Optional[str] = None, *args: Any) -> None:
    try:
        func(*args)
    except Exception as exc:
        if task_key:
            process_status[task_key] = {"progress": 100, "status": "error", "message": str(exc)}
        logger.exception("Background task failed: %s", exc)''')

    # === API 路由 ===
    api_endpoints = '''
@app.get("/")
async def index(request: Request):
    return templates.TemplateResponse(request, "index.html")


@app.post("/api/auth/login")
async def login(username: str = Form(...), password: str = Form(...)):
    record = user_manager.authenticate(username, password)
    if not record:
        return {"status": "error", "message": "用户名或密码错误"}
    token = user_manager.create_session(username)
    return {"status": "success", "data": {"name": username, "display_name": record.get("display_name") or username, "role": record.get("role", "user"), "token": token}}


@app.post("/api/auth/register")
async def register(username: str = Form(...), password: str = Form(...)):
    if not user_manager.register(username, password):
        return {"status": "error", "message": "用户名已存在或格式不合法"}
    return {"status": "success"}


@app.get("/api/status")
async def get_status(x_user_name: str = Header(None), x_auth_token: str = Header(None)):
    username = require_user(x_user_name, x_auth_token)
    if not username:
        return {"status": "success", "data": {"voxelize": process_status.get("voxelize"), "rl": process_status.get("rl")}}
    return {"status": "success", "data": {"voxelize": process_status.get(user_task_key(username, "voxelize"), process_status.get("voxelize")), "rl": process_status.get(user_task_key(username, "rl"), process_status.get("rl"))}}


@app.get("/api/planners")
async def get_planners():
    return {"status": "success", "data": [{"key": key, "name": cfg["name"], "description": cfg.get("description", ""), "parameters": cfg.get("parameters", [])} for key, cfg in PLANNER_REGISTRY.items()]}


@app.get("/api/admin/stats")
async def admin_stats():
    users = list(user_manager.users)
    file_counts = {category: 0 for category in USER_CATEGORIES}
    disk_usage = {username: 0.0 for username in users}
    total_size_bytes = 0
    for username in users:
        for category in USER_CATEGORIES:
            for file_path in get_user_dir(username, category).iterdir():
                if file_path.is_file():
                    size = file_path.stat().st_size
                    file_counts[category] += 1
                    disk_usage[username] += size / 1024 / 1024
                    total_size_bytes += size
    route_count = file_counts.get("manual_route", 0) + file_counts.get("algorithm_route", 0)
    return {"status": "success", "data": {"user_count": len(users), "file_count": int(sum(file_counts.values())), "file_counts": {**file_counts, "route": route_count}, "disk_usage": {name: round(size, 2) for name, size in disk_usage.items()}, "total_size": f"{total_size_bytes / 1024 / 1024:.2f} MB", "total_size_mb": round(total_size_bytes / 1024 / 1024, 2)}}


@app.get("/api/list/{category}")
async def list_files(category: str, x_user_name: str = Header(None), x_auth_token: str = Header(None)):
    username = require_user(x_user_name, x_auth_token)
    if not username:
        return {"status": "error", "message": "Login required"}
    files = []
    for file_path in get_user_dir(username, category).iterdir():
        if file_path.is_file() and not file_path.name.startswith("."):
            files.append({"name": file_path.name, "owner": username, "size": f"{file_path.stat().st_size / 1024 / 1024:.2f} MB", "mtime": file_path.stat().st_mtime})
    files.sort(key=lambda item: item["mtime"], reverse=True)
    return {"status": "success", "data": files}


@app.get("/api/user/profile")
async def get_profile(search: str = "", x_user_name: str = Header(None), x_auth_token: str = Header(None)):
    username = require_user(x_user_name, x_auth_token)
    if not username:
        return {"status": "error", "message": "Login required"}
    return {"status": "success", "data": user_manager.get_profile(username, search=search or None)}


@app.post("/api/user/profile/update")
async def update_profile(new_username: str = Form(""), display_name: str = Form(""), email: str = Form(""), phone: str = Form(""), notes: str = Form(""), x_user_name: str = Header(None), x_auth_token: str = Header(None)):
    username = require_user(x_user_name, x_auth_token)
    if not username:
        return {"status": "error", "message": "Login required"}
    current_name = username
    if new_username and new_username.strip():
        ok, result = user_manager.rename_user(username, new_username.strip())
        if not ok:
            return {"status": "error", "message": result}
        current_name = result
    user_manager.update_profile(current_name, display_name=display_name, email=email, phone=phone, notes=notes)
    return {"status": "success", "data": user_manager.get_profile(current_name)}


@app.post("/api/user/profile/rescan")
async def rescan_profile(x_user_name: str = Header(None), x_auth_token: str = Header(None)):
    username = require_user(x_user_name, x_auth_token)
    if not username:
        return {"status": "error", "message": "Login required"}
    return {"status": "success", "data": user_manager.get_profile(username)}


@app.post("/api/upload/{category}")
async def upload_file(category: str, file: UploadFile = File(...), x_user_name: str = Header(None), x_auth_token: str = Header(None)):
    username = require_user(x_user_name, x_auth_token)
    if not username:
        return JSONResponse({"status": "error", "message": "Login required"}, status_code=401)
    try:
        target_dir = get_user_dir(username, category)
        filename = safe_filename(file.filename or "")
        if not filename:
            return {"status": "error", "message": "文件名不能为空"}
        target_path = target_dir / filename
        if target_path.exists():
            return {"status": "error", "message": "文件已存在", "code": "DUPLICATE"}
        with target_path.open("wb") as output:
            shutil.copyfileobj(file.file, output)
        return {"status": "success", "message": "上传成功"}
    except Exception as exc:
        return JSONResponse({"status": "error", "message": str(exc)}, status_code=500)


@app.post("/api/manage/delete")
async def delete_file(category: str = Form(...), filename: str = Form(...), x_user_name: str = Header(None), x_auth_token: str = Header(None)):
    username = require_user(x_user_name, x_auth_token)
    if not username:
        return {"status": "error", "message": "Login required"}
    target = get_user_dir(username, category) / safe_filename(filename)
    if not target.exists():
        return {"status": "error", "message": "文件未找到或无权删除"}
    target.unlink()
    return {"status": "success"}


@app.post("/api/manage/rename")
async def rename_file(category: str = Form(...), old_name: str = Form(...), new_name: str = Form(...), x_user_name: str = Header(None), x_auth_token: str = Header(None)):
    username = require_user(x_user_name, x_auth_token)
    if not username:
        return {"status": "error", "message": "Login required"}
    folder = get_user_dir(username, category)
    old_path = folder / safe_filename(old_name)
    new_path = folder / safe_filename(new_name)
    if not old_path.exists():
        return {"status": "error", "message": "文件未找到"}
    if new_path.exists():
        return {"status": "error", "message": "目标文件名已存在"}
    old_path.rename(new_path)
    return {"status": "success"}


@app.get("/api/visualize/pointcloud")
async def get_pointcloud(filename: str, x_user_name: str = Header(None), x_auth_token: str = Header(None)):
    username = require_user(x_user_name, x_auth_token)
    if not username:
        return {"status": "error", "message": "Login required"}
    point_cloud = find_user_file(username, "point_cloud", filename)
    if not point_cloud:
        return {"status": "error", "message": "File not found"}
    try:
        return {"status": "success", "data": read_las_for_vis(str(point_cloud))}
    except Exception as exc:
        return {"status": "error", "message": str(exc)}''')

    result.append(api_endpoints)

    # === /api/visualize/result ===
    result.append('''
@app.get("/api/visualize/result")
async def get_result(type: str, filename: str, x_user_name: str = Header(None), x_auth_token: str = Header(None)):
    username = require_user(x_user_name, x_auth_token)
    if not username:
        return {"status": "error", "message": "Login required"}
    safe = safe_filename(filename)
    try:
        if type in {"manual_route", "algorithm_route"}:
            route_path = find_user_file(username, type, safe)
            if not route_path:
                return {"status": "error", "message": "File not found"}
            method = "算法航线" if type == "algorithm_route" else "人工航点"
            return {"status": "success", "data": {"waypoints": parse_manual_route(str(route_path)), "method": method, "method_name": method}}
        if type == "voxel":
            voxel_path = find_user_file(username, "voxel", safe)
            if not voxel_path:
                return {"status": "error", "message": "File not found"}
            data = np.load(voxel_path, allow_pickle=True)
            raw_voxels = data["display_voxels"] if "display_voxels" in data.files else data["voxels"]
            meta = {}
            if "meta_json" in data.files:
                try:
                    meta = json.loads(str(data["meta_json"].item()))
                except Exception:
                    meta = {}
            voxels = [{"pos": voxel["coord"], "type": int(voxel.get("type", 1)) if isinstance(voxel, dict) else int(voxel["type"]), "label": int(voxel.get("label", 0)) if isinstance(voxel, dict) else int(voxel["label"]) if "label" in (voxel.dtype.names or ()) else 0, "category": voxel.get("category", "tower") if isinstance(voxel, dict) else str(voxel["category"]) if "category" in (voxel.dtype.names or ()) else "tower"} for voxel in raw_voxels]
            base = voxel_path.stem.replace("_voxel", "")
            candidate_path = voxel_path.parent / f"{base}_candidates.json"
            candidates = []
            if candidate_path.exists():
                candidate_data = json.loads(candidate_path.read_text(encoding="utf-8"))
                seen = set()
                for candidate in candidate_data:
                    point = tuple(candidate["utm_position"])
                    if point not in seen:
                        candidates.append(candidate["utm_position"])
                        seen.add(point)
            return {"status": "success", "data": {"voxels": voxels, "candidates": candidates, "center": data["local_center"].tolist(), "meta": meta}}
        if type == "waypoint":
            route_path = find_user_file(username, "waypoint", safe)
            if not route_path:
                return {"status": "error", "message": "File not found"}
            result_json = json.loads(route_path.read_text(encoding="utf-8"))
            waypoints = normalize_waypoints(result_json.get("waypoints", []))
            raw_stats = result_json.get("stats", {})
            coverage = raw_stats.get("coverage", raw_stats.get("final_coverage"))
            stats = {**compute_waypoint_metrics(waypoints, coverage=coverage), **raw_stats}
            if any(stats.get(key) is None for key in ("coverage_tower", "coverage_insulator", "C_geo", "C_top", "C_edge", "C_body")):
                stats = attach_coverage_metrics(username, route_path.name, waypoints, stats)
            payload = {"waypoints": waypoints, "method": result_json.get("method", "算法航点"), "method_name": result_json.get("method_name", "算法航点"), "stats": stats}
            for key in ("planning_input", "route_context", "tower_results"):
                if key in result_json:
                    payload[key] = result_json[key]
            return {"status": "success", "data": payload}
        return {"status": "error", "message": "Unsupported result type"}
    except Exception as exc:
        return {"status": "error", "message": str(exc)}''')

    # === compare, export, route, process 端点 ===
    result.append('''

@app.post("/api/compare/routes")
async def compare_routes(payload: dict = Body(...), x_user_name: str = Header(None), x_auth_token: str = Header(None)):
    username = require_user(x_user_name, x_auth_token)
    if not username:
        return {"status": "error", "message": "Login required"}
    selections = payload.get("selections", [])
    if not isinstance(selections, list) or not selections:
        return {"status": "error", "message": "请选择至少一组航点"}
    items = []
    for item in selections:
        try:
            category = item.get("category")
            filename = item.get("filename")
            if not category or not filename:
                continue
            items.append(load_route_for_compare(username, category, filename))
        except Exception as exc:
            items.append({"category": item.get("category"), "filename": item.get("filename"), "error": str(exc)})
    valid_items = [item for item in items if "error" not in item]
    manual = next((item for item in valid_items if item.get("category") == "manual_route"), None)
    manual_count = manual.get("stats", {}).get("count") if manual else None
    for item in valid_items:
        stats = item.get("stats", {})
        if item.get("category") != "manual_route" and manual_count:
            stats["waypoint_reduction_vs_manual"] = round((manual_count - stats.get("count", 0)) / manual_count, 4)
    return {"status": "success", "data": {"items": items}}


@app.get("/api/export/waypoint")
async def export_waypoint(filename: str, x_user_name: str = Header(None), x_auth_token: str = Header(None)):
    username = require_user(x_user_name, x_auth_token)
    if not username:
        return {"status": "error", "message": "Login required"}
    route_path = find_user_file(username, "waypoint", filename)
    if not route_path:
        return {"status": "error", "message": "File not found"}
    return FileResponse(path=str(route_path), media_type="application/json", filename=route_path.name)


@app.get("/api/export/route")
async def export_route(filename: str, category: str = "algorithm_route", x_user_name: str = Header(None), x_auth_token: str = Header(None)):
    """Export a manual or algorithm route JSON file."""
    username = require_user(x_user_name, x_auth_token)
    if not username:
        return {"status": "error", "message": "Login required"}
    if category not in {"manual_route", "algorithm_route"}:
        return {"status": "error", "message": "Invalid route category"}
    route_path = find_user_file(username, category, filename)
    if not route_path:
        return {"status": "error", "message": "File not found"}
    return FileResponse(path=str(route_path), media_type="application/json", filename=route_path.name)


@app.post("/api/route/plan")
async def plan_route(waypoint_filename: str = Form(...), safety_distance_m: Optional[float] = Form(None), clearance_m: Optional[float] = Form(None), wire_clearance_m: Optional[float] = Form(None), task_tower_clearance_m: Optional[float] = Form(None), task_wire_clearance_m: Optional[float] = Form(None), entry_distance_m: float = Form(28.0), x_user_name: str = Header(None), x_auth_token: str = Header(None)):
    username = require_user(x_user_name, x_auth_token)
    if not username:
        return {"status": "error", "message": "Login required"}
    waypoint_path = find_user_file(username, "waypoint", waypoint_filename)
    if not waypoint_path:
        return {"status": "error", "message": "算法航点文件不存在"}
    voxel_path = find_matching_voxel(username, waypoint_path.name)
    try:
        clearances = build_route_clearance_payload(safety_distance_m=safety_distance_m, clearance_m=clearance_m, wire_clearance_m=wire_clearance_m, task_tower_clearance_m=task_tower_clearance_m, task_wire_clearance_m=task_wire_clearance_m)
        output_path, output = plan_route_from_waypoints(waypoint_path=waypoint_path, output_dir=get_user_dir(username, "algorithm_route"), voxel_path=voxel_path, safety_distance_m=clearances["safety_distance_m"], clearance_m=clearances["clearance_m"], wire_clearance_m=clearances["wire_clearance_m"], task_tower_clearance_m=clearances["task_tower_clearance_m"], task_wire_clearance_m=clearances["task_wire_clearance_m"], entry_distance_m=max(10.0, min(80.0, float(entry_distance_m))))
        return {"status": "success", "data": {"filename": output_path.name, "source": waypoint_path.name, "voxel": voxel_path.name if voxel_path else None, "route_point_count": len((output.get("towers") or [{}])[0].get("points") or []), "totalLen": output.get("totalLen"), "detour_point_count": output.get("route_planning", {}).get("detour_point_count", 0), "astar_segment_count": output.get("route_planning", {}).get("astar_segment_count", 0), "astar_fallback_count": output.get("route_planning", {}).get("astar_fallback_count", 0), "clearance": output.get("route_planning", {})}}
    except Exception as exc:
        logger.exception("Route planning failed: %s", exc)
        return JSONResponse({"status": "error", "message": str(exc)}, status_code=500)


@app.post("/api/route/validate")
async def validate_route(category: str = Form("algorithm_route"), filename: str = Form(...), safety_distance_m: Optional[float] = Form(None), tower_clearance_m: Optional[float] = Form(None), wire_clearance_m: Optional[float] = Form(None), task_tower_clearance_m: Optional[float] = Form(None), task_wire_clearance_m: Optional[float] = Form(None), x_user_name: str = Header(None), x_auth_token: str = Header(None)):
    username = require_user(x_user_name, x_auth_token)
    if not username:
        return {"status": "error", "message": "Login required"}
    if category not in {"manual_route", "algorithm_route"}:
        return {"status": "error", "message": "只支持人工航线或算法航线安全校验"}
    route_path = find_user_file(username, category, filename)
    if not route_path:
        return {"status": "error", "message": "航线文件不存在"}
    voxel_path = find_voxel_for_route(username, category, filename)
    try:
        clearances = build_route_clearance_payload(safety_distance_m=safety_distance_m, tower_clearance_m=tower_clearance_m, wire_clearance_m=wire_clearance_m, task_tower_clearance_m=task_tower_clearance_m, task_wire_clearance_m=task_wire_clearance_m)
        result_val = validate_route_safety(route_path=route_path, voxel_path=voxel_path, tower_clearance_m=clearances["clearance_m"], wire_clearance_m=clearances["wire_clearance_m"], task_tower_clearance_m=clearances["task_tower_clearance_m"], task_wire_clearance_m=clearances["task_wire_clearance_m"])
        if clearances["safety_distance_m"] is not None:
            result_val["safety_distance_m"] = clearances["safety_distance_m"]
        result_val["filename"] = route_path.name
        result_val["category"] = category
        return {"status": "success", "data": result_val}
    except Exception as exc:
        logger.exception("Route safety validation failed: %s", exc)
        return JSONResponse({"status": "error", "message": str(exc)}, status_code=500)


@app.post("/api/process/voxelize")
async def start_voxel(background_tasks: BackgroundTasks, pc_filename: str = Form(...), x_user_name: str = Header(None), x_auth_token: str = Header(None)):
    username = require_user(x_user_name, x_auth_token)
    if not username:
        return {"status": "error", "message": "Login required"}
    point_cloud = find_user_file(username, "point_cloud", pc_filename)
    if not point_cloud:
        return {"status": "error", "message": "File not found"}
    task_key = user_task_key(username, "voxelize")
    process_status[task_key] = {"progress": 0, "status": "starting"}
    background_tasks.add_task(run_task, Pretreatment(str(point_cloud), str(get_user_dir(username, "voxel")), status_key=task_key, manual_route_path=str(find_matching_manual_route(username, point_cloud.stem) or "")).run, task_key)
    return {"status": "success"}


@app.post("/api/process/rl")
async def start_rl(background_tasks: BackgroundTasks, voxel_filename: str = Form(...), planner: str = Form(SemanticWeightedGreedyPlanner.planner_name), safety_distance_m: Optional[float] = Form(None), conductor_no_fly_enabled: Optional[bool] = Form(None), conductor_no_fly_extent_margin_m: Optional[float] = Form(None), conductor_no_fly_min_length_m: Optional[float] = Form(None), conductor_no_fly_boundary_tolerance_m: Optional[float] = Form(None), max_waypoints: Optional[int] = Form(None), max_shots_per_waypoint: Optional[int] = Form(None), single_layer_episodes: Optional[int] = Form(None), hierarchical_episodes: Optional[int] = Form(None), manual_ratio_min: Optional[float] = Form(None), manual_ratio_max: Optional[float] = Form(None), target_manual_ratio: Optional[float] = Form(None), x_user_name: str = Header(None), x_auth_token: str = Header(None)):
    username = require_user(x_user_name, x_auth_token)
    if not username:
        return {"status": "error", "message": "Login required"}
    voxel_path = find_user_file(username, "voxel", voxel_filename)
    if not voxel_path:
        return {"status": "error", "message": "File not found"}
    base = voxel_path.stem.replace("_voxel", "")
    candidate_path = voxel_path.parent / f"{base}_candidates.json"
    if not candidate_path.exists():
        return {"status": "error", "message": f"候选点文件不存在: {candidate_path.name}"}
    planner = (planner or SemanticWeightedGreedyPlanner.planner_name).strip()
    if planner not in PLANNER_REGISTRY:
        planner = next(iter(PLANNER_REGISTRY.keys()))
    planner_cfg = PLANNER_REGISTRY[planner]
    task_key = user_task_key(username, "rl")
    process_status[task_key] = {"progress": 0, "status": "starting"}
    planning_input = WaypointPlanningInput(voxel_path=voxel_path, candidate_path=candidate_path, output_dir=get_user_dir(username, "waypoint"), planner_key=planner, planner_name=planner_cfg["name"], status_key=task_key, manual_route_path=find_matching_manual_route(username, base), constraints=build_planner_constraints_payload(safety_distance_m=safety_distance_m, conductor_no_fly_enabled=conductor_no_fly_enabled, conductor_no_fly_extent_margin_m=conductor_no_fly_extent_margin_m, conductor_no_fly_min_length_m=conductor_no_fly_min_length_m, conductor_no_fly_boundary_tolerance_m=conductor_no_fly_boundary_tolerance_m, max_waypoints=max_waypoints, max_shots_per_waypoint=max_shots_per_waypoint, single_layer_episodes=single_layer_episodes, hierarchical_episodes=hierarchical_episodes, manual_ratio_min=manual_ratio_min, manual_ratio_max=manual_ratio_max, target_manual_ratio=target_manual_ratio))
    background_tasks.add_task(run_task, run_waypoint_planning, task_key, planning_input, planner_cfg["solver"])
    return {"status": "success"}''')

    # === 入口 ===
    result.append('''

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=8000)''')

    output = '\n'.join(result)
    target = Path(__file__).resolve().parent.parent / "original_backup" / "main.py"
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(output, encoding="utf-8")
    print(f"Written: {target} ({len(output)} bytes, ~{len(output.splitlines())} lines)")


if __name__ == "__main__":
    main()
