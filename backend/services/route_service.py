"""
航线服务：路径规划、安全验证、对比分析。
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Optional

from backend.core.security import get_user_dir, safe_filename


def find_matching_voxel(username: str, route_filename: str) -> Optional[Path]:
    """查找与航线文件匹配的体素文件。"""
    stem = Path(safe_filename(route_filename)).stem
    candidates = [f"{stem}_voxel.npz"]

    # 根据规划器名称推导备选体素文件名
    from backend.services.processing_service import PLANNER_REGISTRY
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

    voxel_dir = get_user_dir(username, "voxel")
    for name in unique:
        voxel_path = voxel_dir / name
        if voxel_path.exists():
            return voxel_path

    voxels = [path for path in voxel_dir.iterdir()
              if path.is_file() and path.name.endswith("_voxel.npz")]
    return voxels[0] if len(voxels) == 1 else None


def find_voxel_for_route(username: str, category: str, filename: str) -> Optional[Path]:
    """查找与航点或生成航线配对的体素文件。"""
    from backend.services.file_service import find_user_file

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
    return find_matching_voxel(username, filename)


def attach_coverage_metrics(username: str, filename: str, waypoints: list, stats: dict) -> dict:
    """将覆盖率指标附加到现有统计中。"""
    from waypoint_planning.planning_core import evaluate_waypoint_coverage

    voxel_path = find_matching_voxel(username, filename)
    if not voxel_path:
        return stats
    coverage = evaluate_waypoint_coverage(waypoints, str(voxel_path))
    merged = {**stats, **coverage}
    if merged.get("count") is None:
        merged["count"] = len(waypoints or [])
    return merged


def normalize_waypoints(raw_waypoints: list) -> list:
    """将生成的航点 JSON 标准化为前端使用的格式。"""
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
    return waypoints
