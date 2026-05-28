"""
航线规划与验证路由：POST /api/route/plan, /validate
"""

from typing import Optional

from fastapi import APIRouter, Depends, Form
from fastapi.responses import JSONResponse

from backend.core.dependencies import get_current_user
from backend.core.security import get_user_dir
from backend.services.file_service import find_user_file
from backend.services.processing_service import build_route_clearance_payload
from backend.services.route_service import find_matching_voxel, find_voxel_for_route
from waypoint_planning.route_planner import plan_route_from_waypoints, validate_route_safety

router = APIRouter(tags=["routes"])


@router.post("/api/route/plan")
async def plan_route(
    waypoint_filename: str = Form(...),
    safety_distance_m: Optional[float] = Form(None),
    clearance_m: Optional[float] = Form(None),
    wire_clearance_m: Optional[float] = Form(None),
    task_tower_clearance_m: Optional[float] = Form(None),
    task_wire_clearance_m: Optional[float] = Form(None),
    entry_distance_m: float = Form(28.0),
    username: str = Depends(get_current_user),
):
    waypoint_path = find_user_file(username, "waypoint", waypoint_filename)
    if not waypoint_path:
        return {"status": "error", "message": "算法航点文件不存在"}
    voxel_path = find_matching_voxel(username, waypoint_path.name)
    try:
        clearances = build_route_clearance_payload(
            safety_distance_m=safety_distance_m,
            clearance_m=clearance_m,
            wire_clearance_m=wire_clearance_m,
            task_tower_clearance_m=task_tower_clearance_m,
            task_wire_clearance_m=task_wire_clearance_m,
        )
        output_path, output = plan_route_from_waypoints(
            waypoint_path=waypoint_path,
            output_dir=get_user_dir(username, "algorithm_route"),
            voxel_path=voxel_path,
            safety_distance_m=clearances["safety_distance_m"],
            clearance_m=clearances["clearance_m"],
            wire_clearance_m=clearances["wire_clearance_m"],
            task_tower_clearance_m=clearances["task_tower_clearance_m"],
            task_wire_clearance_m=clearances["task_wire_clearance_m"],
            entry_distance_m=max(10.0, min(80.0, float(entry_distance_m))),
        )
        return {
            "status": "success",
            "data": {
                "filename": output_path.name,
                "source": waypoint_path.name,
                "voxel": voxel_path.name if voxel_path else None,
                "route_point_count": len((output.get("towers") or [{}])[0].get("points") or []),
                "totalLen": output.get("totalLen"),
                "detour_point_count": output.get("route_planning", {}).get("detour_point_count", 0),
                "astar_segment_count": output.get("route_planning", {}).get("astar_segment_count", 0),
                "astar_fallback_count": output.get("route_planning", {}).get("astar_fallback_count", 0),
                "clearance": output.get("route_planning", {}),
            },
        }
    except Exception as exc:
        import logging
        logger = logging.getLogger(__name__)
        logger.exception("Route planning failed: %s", exc)
        return JSONResponse({"status": "error", "message": str(exc)}, status_code=500)


@router.post("/api/route/validate")
async def validate_route(
    category: str = Form("algorithm_route"),
    filename: str = Form(...),
    safety_distance_m: Optional[float] = Form(None),
    tower_clearance_m: Optional[float] = Form(None),
    wire_clearance_m: Optional[float] = Form(None),
    task_tower_clearance_m: Optional[float] = Form(None),
    task_wire_clearance_m: Optional[float] = Form(None),
    username: str = Depends(get_current_user),
):
    if category not in {"manual_route", "algorithm_route"}:
        return {"status": "error", "message": "只支持人工航线或算法航线安全校验"}
    route_path = find_user_file(username, category, filename)
    if not route_path:
        return {"status": "error", "message": "航线文件不存在"}
    voxel_path = find_voxel_for_route(username, category, filename)
    try:
        clearances = build_route_clearance_payload(
            safety_distance_m=safety_distance_m,
            tower_clearance_m=tower_clearance_m,
            wire_clearance_m=wire_clearance_m,
            task_tower_clearance_m=task_tower_clearance_m,
            task_wire_clearance_m=task_wire_clearance_m,
        )
        result = validate_route_safety(
            route_path=route_path,
            voxel_path=voxel_path,
            tower_clearance_m=clearances["clearance_m"],
            wire_clearance_m=clearances["wire_clearance_m"],
            task_tower_clearance_m=clearances["task_tower_clearance_m"],
            task_wire_clearance_m=clearances["task_wire_clearance_m"],
        )
        if clearances["safety_distance_m"] is not None:
            result["safety_distance_m"] = clearances["safety_distance_m"]
        result["filename"] = route_path.name
        result["category"] = category
        return {"status": "success", "data": result}
    except Exception as exc:
        import logging
        logger = logging.getLogger(__name__)
        logger.exception("Route safety validation failed: %s", exc)
        return JSONResponse({"status": "error", "message": str(exc)}, status_code=500)
