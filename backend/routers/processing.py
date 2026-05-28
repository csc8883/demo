"""
处理任务路由：POST /api/process/voxelize, /rl, GET /api/status
"""

from typing import Any, Optional

from fastapi import APIRouter, BackgroundTasks, Depends, Form, Header

from backend.core.dependencies import get_current_user
from backend.core.security import get_user_dir
from backend.services.file_service import find_user_file
from backend.services.processing_service import (
    PLANNER_REGISTRY,
    build_planner_constraints_payload,
)
from backend.services.progress_service import progress_broker
from waypoint_planning.algorithms import (
    Pretreatment,
    process_status,
)
from waypoint_planning.planning_solvers import run_waypoint_planning
from waypoint_planning.waypoint_models import WaypointPlanningInput

router = APIRouter(tags=["processing"])


def _task_key(username: str, task: str) -> str:
    return f"{username}:{task}"


async def _broadcast_progress(task_key: str) -> None:
    """从 process_status 读取最新状态并通过 WebSocket 广播。"""
    status = process_status.get(task_key, {})
    await progress_broker.broadcast(task_key, status)


def _run_task(func: Any, task_key: Optional[str] = None, *args: Any) -> None:
    """运行后台任务并在失败时更新状态和广播。"""
    import logging
    logger = logging.getLogger(__name__)
    try:
        func(*args)
    except Exception as exc:
        if task_key:
            process_status[task_key] = {"progress": 100, "status": "error", "message": str(exc)}
        logger.exception("Background task failed: %s", exc)


@router.get("/api/status")
async def get_status(
    x_user_name: str = Header(None),
    x_auth_token: str = Header(None),
):
    from backend.core.security import user_manager
    username = user_manager.validate_session(x_auth_token)
    if not username:
        return {"status": "success", "data": {"voxelize": process_status.get("voxelize"), "rl": process_status.get("rl")}}
    return {
        "status": "success",
        "data": {
            "voxelize": process_status.get(_task_key(username, "voxelize"), process_status.get("voxelize")),
            "rl": process_status.get(_task_key(username, "rl"), process_status.get("rl")),
        },
    }


@router.post("/api/process/voxelize")
async def start_voxel(
    background_tasks: BackgroundTasks,
    pc_filename: str = Form(...),
    username: str = Depends(get_current_user),
):
    point_cloud = find_user_file(username, "point_cloud", pc_filename)
    if not point_cloud:
        return {"status": "error", "message": "File not found"}

    task_key = _task_key(username, "voxelize")
    process_status[task_key] = {"progress": 0, "status": "starting"}

    from backend.services.file_service import find_matching_manual_route
    manual_route_path = str(find_matching_manual_route(username, point_cloud.stem) or "")

    background_tasks.add_task(
        _run_task,
        Pretreatment(
            str(point_cloud),
            str(get_user_dir(username, "voxel")),
            status_key=task_key,
            manual_route_path=manual_route_path,
        ).run,
        task_key,
    )
    return {"status": "success"}


@router.post("/api/process/rl")
async def start_rl(
    background_tasks: BackgroundTasks,
    voxel_filename: str = Form(...),
    planner: str = Form("SemanticWeightedGreedy"),
    safety_distance_m: Optional[float] = Form(None),
    conductor_no_fly_enabled: Optional[bool] = Form(None),
    conductor_no_fly_extent_margin_m: Optional[float] = Form(None),
    conductor_no_fly_min_length_m: Optional[float] = Form(None),
    conductor_no_fly_boundary_tolerance_m: Optional[float] = Form(None),
    max_waypoints: Optional[int] = Form(None),
    max_shots_per_waypoint: Optional[int] = Form(None),
    single_layer_episodes: Optional[int] = Form(None),
    hierarchical_episodes: Optional[int] = Form(None),
    manual_ratio_min: Optional[float] = Form(None),
    manual_ratio_max: Optional[float] = Form(None),
    target_manual_ratio: Optional[float] = Form(None),
    username: str = Depends(get_current_user),
):
    voxel_path = find_user_file(username, "voxel", voxel_filename)
    if not voxel_path:
        return {"status": "error", "message": "File not found"}

    base = voxel_path.stem.replace("_voxel", "")
    candidate_path = voxel_path.parent / f"{base}_candidates.json"
    if not candidate_path.exists():
        return {"status": "error", "message": f"候选点文件不存在: {candidate_path.name}"}

    planner = (planner or "SemanticWeightedGreedy").strip()
    if planner not in PLANNER_REGISTRY:
        planner = next(iter(PLANNER_REGISTRY.keys()))
    planner_cfg = PLANNER_REGISTRY[planner]

    task_key = _task_key(username, "rl")
    process_status[task_key] = {"progress": 0, "status": "starting"}

    from backend.services.file_service import find_matching_manual_route

    planning_input = WaypointPlanningInput(
        voxel_path=voxel_path,
        candidate_path=candidate_path,
        output_dir=get_user_dir(username, "waypoint"),
        planner_key=planner,
        planner_name=planner_cfg["name"],
        status_key=task_key,
        manual_route_path=find_matching_manual_route(username, base),
        constraints=build_planner_constraints_payload(
            safety_distance_m=safety_distance_m,
            conductor_no_fly_enabled=conductor_no_fly_enabled,
            conductor_no_fly_extent_margin_m=conductor_no_fly_extent_margin_m,
            conductor_no_fly_min_length_m=conductor_no_fly_min_length_m,
            conductor_no_fly_boundary_tolerance_m=conductor_no_fly_boundary_tolerance_m,
            max_waypoints=max_waypoints,
            max_shots_per_waypoint=max_shots_per_waypoint,
            single_layer_episodes=single_layer_episodes,
            hierarchical_episodes=hierarchical_episodes,
            manual_ratio_min=manual_ratio_min,
            manual_ratio_max=manual_ratio_max,
            target_manual_ratio=target_manual_ratio,
        ),
    )
    background_tasks.add_task(_run_task, run_waypoint_planning, task_key, planning_input, planner_cfg["solver"])
    return {"status": "success"}
