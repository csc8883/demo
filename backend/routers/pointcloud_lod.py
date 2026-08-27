"""Point-cloud LOD cache API."""

from __future__ import annotations

from typing import Any, Dict

from fastapi import APIRouter, BackgroundTasks, Depends, Query

from backend.core.dependencies import get_current_user
from backend.services.pointcloud_lod_service import lod_status, prepare_lod


router = APIRouter(prefix="/api/pointcloud-lod", tags=["pointcloud-lod"])


def _success(data: Any) -> Dict[str, Any]:
    return {"status": "success", "data": data}


def _error(exc: Exception) -> Dict[str, Any]:
    return {"status": "error", "message": str(exc)}


@router.get("/{point_cloud_name}/status")
async def status(
    point_cloud_name: str,
    variant: str = Query("base"),
    profile_id: str = Query(""),
    username: str = Depends(get_current_user),
):
    try:
        return _success(lod_status(username, point_cloud_name, variant=variant, profile_id=profile_id or None))
    except Exception as exc:
        return _error(exc)


@router.post("/{point_cloud_name}/prepare")
async def prepare(
    point_cloud_name: str,
    background_tasks: BackgroundTasks,
    variant: str = Query("base"),
    profile_id: str = Query(""),
    username: str = Depends(get_current_user),
):
    try:
        return _success(
            prepare_lod(
                username,
                point_cloud_name,
                background_tasks,
                variant=variant,
                profile_id=profile_id or None,
            )
        )
    except Exception as exc:
        return _error(exc)
