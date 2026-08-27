"""
可视化路由：GET /api/visualize/pointcloud, /result, /voxel, /route
"""

import json

import numpy as np
from fastapi import APIRouter, Depends, Query

from backend.core.dependencies import get_current_user
from backend.core.security import safe_filename
from backend.services.file_service import find_user_file
from backend.services.route_service import (
    attach_coverage_metrics,
    find_matching_voxel,
    find_voxel_for_route,
    normalize_waypoints,
)
from waypoint_planning.algorithms import (
    compute_waypoint_metrics,
    evaluate_waypoint_coverage,
    parse_manual_route,
    read_las_for_vis,
)

router = APIRouter(tags=["visualization"])


@router.get("/api/visualize/pointcloud")
async def get_pointcloud(filename: str, username: str = Depends(get_current_user)):
    point_cloud = find_user_file(username, "point_cloud", filename)
    if not point_cloud:
        return {"status": "error", "message": "File not found"}
    try:
        from backend.services.weight_service import get_active_profile

        active_profile = get_active_profile(username, filename)
        return {
            "status": "success",
            "data": read_las_for_vis(str(point_cloud), weight_profile=active_profile),
        }
    except Exception as exc:
        return {"status": "error", "message": str(exc)}


@router.get("/api/visualize/result")
async def get_result(
    type: str = Query(...),
    filename: str = Query(...),
    username: str = Depends(get_current_user),
):
    safe = safe_filename(filename)
    try:
        if type in {"manual_route", "algorithm_route"}:
            route_path = find_user_file(username, type, safe)
            if not route_path:
                return {"status": "error", "message": "File not found"}
            method = "算法航线" if type == "algorithm_route" else "人工航点"
            return {
                "status": "success",
                "data": {
                    "waypoints": parse_manual_route(str(route_path)),
                    "method": method,
                    "method_name": method,
                },
            }

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
            voxels = []
            for voxel in raw_voxels:
                if isinstance(voxel, dict):
                    entry = {"pos": voxel["coord"], "type": int(voxel.get("type", 1)),
                             "label": int(voxel.get("label", 0)), "category": voxel.get("category", "tower")}
                else:
                    entry = {"pos": voxel["coord"], "type": int(voxel["type"]),
                             "category": str(voxel["category"]) if "category" in (voxel.dtype.names or ()) else "tower"}
                    if "label" in (voxel.dtype.names or ()):
                        entry["label"] = int(voxel["label"])
                    else:
                        entry["label"] = 0
                voxels.append(entry)

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
            return {
                "status": "success",
                "data": {
                    "voxels": voxels, "candidates": candidates,
                    "center": data["local_center"].tolist(), "meta": meta,
                },
            }

        if type == "waypoint":
            route_path = find_user_file(username, "waypoint", safe)
            if not route_path:
                return {"status": "error", "message": "File not found"}
            result = json.loads(route_path.read_text(encoding="utf-8"))
            waypoints = normalize_waypoints(result.get("waypoints", []))
            raw_stats = result.get("stats", {})
            coverage = raw_stats.get("coverage", raw_stats.get("final_coverage"))
            stats = {**compute_waypoint_metrics(waypoints, coverage=coverage), **raw_stats}
            if any(stats.get(key) is None for key in ("coverage_tower", "coverage_insulator", "C_geo", "C_top", "C_edge", "C_body")):
                stats = attach_coverage_metrics(username, route_path.name, waypoints, stats)
            payload = {
                "waypoints": waypoints,
                "method": result.get("method", "算法航点"),
                "method_name": result.get("method_name", "算法航点"),
                "stats": stats,
            }
            for key in ("planning_input", "route_context", "tower_results"):
                if key in result:
                    payload[key] = result[key]
            return {"status": "success", "data": payload}

        return {"status": "error", "message": "Unsupported result type"}
    except Exception as exc:
        return {"status": "error", "message": str(exc)}
