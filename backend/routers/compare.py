"""
航线对比路由：POST /api/compare/routes
"""

import json

from fastapi import APIRouter, Body, Depends

from backend.core.dependencies import get_current_user
from backend.services.file_service import find_user_file
from backend.services.route_service import (
    attach_coverage_metrics,
    find_matching_voxel,
    normalize_waypoints,
)
from waypoint_planning.algorithms import (
    compute_waypoint_metrics,
    evaluate_waypoint_coverage,
    parse_manual_route,
)

router = APIRouter(tags=["compare"])


@router.post("/api/compare/routes")
async def compare_routes(
    payload: dict = Body(...),
    username: str = Depends(get_current_user),
):
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

            if category in {"manual_route", "algorithm_route"}:
                route_path = find_user_file(username, category, filename)
                if not route_path:
                    items.append({"category": category, "filename": filename, "error": "FileNotFoundError"})
                    continue
                waypoints = parse_manual_route(str(route_path))
                stats = attach_coverage_metrics(username, route_path.name, waypoints, compute_waypoint_metrics(waypoints))
                method = "算法航线" if category == "algorithm_route" else "人工航点"
                items.append({
                    "category": category, "filename": route_path.name, "method": method,
                    "method_name": method, "waypoints": waypoints, "stats": stats,
                })

            elif category == "waypoint":
                route_path = find_user_file(username, "waypoint", filename)
                if not route_path:
                    items.append({"category": category, "filename": filename, "error": "FileNotFoundError"})
                    continue
                result = json.loads(route_path.read_text(encoding="utf-8"))
                waypoints = normalize_waypoints(result.get("waypoints", []))
                raw_stats = result.get("stats", {})
                coverage = raw_stats.get("coverage", raw_stats.get("final_coverage"))
                stats = {**compute_waypoint_metrics(waypoints, coverage=coverage), **raw_stats}
                if any(stats.get(key) is None for key in ("coverage_tower", "coverage_insulator", "C_geo", "C_top", "C_edge", "C_body")):
                    stats = attach_coverage_metrics(username, route_path.name, waypoints, stats)
                item = {
                    "category": category, "filename": route_path.name,
                    "method": result.get("method", "算法航点"),
                    "method_name": result.get("method_name", "算法航点"),
                    "waypoints": waypoints, "stats": stats,
                }
                for key in ("planning_input", "route_context", "tower_results"):
                    if key in result:
                        item[key] = result[key]
                items.append(item)
            else:
                items.append({"category": category, "filename": filename, "error": "Unsupported category"})

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
