"""Custom point-cloud weight profile API."""

from __future__ import annotations

from typing import Any, Dict

from fastapi import APIRouter, Body, Depends, Query

from backend.core.dependencies import get_current_user
from backend.services.weight_service import (
    editable_points,
    preview_profile,
    profile_status,
    restore_original,
    save_profile,
)


router = APIRouter(prefix="/api/weights", tags=["weights"])


def _success(data: Any) -> Dict[str, Any]:
    return {"status": "success", "data": data}


def _error(exc: Exception) -> Dict[str, Any]:
    return {"status": "error", "message": str(exc)}


@router.get("/{point_cloud_name}/editable-points")
async def get_editable_points(
    point_cloud_name: str,
    limit: int = Query(120_000, ge=1_000, le=200_000),
    username: str = Depends(get_current_user),
):
    try:
        return _success(editable_points(username, point_cloud_name, limit=limit))
    except Exception as exc:
        return _error(exc)


@router.post("/{point_cloud_name}/preview")
async def preview(
    point_cloud_name: str,
    payload: Dict[str, Any] = Body(default_factory=dict),
    username: str = Depends(get_current_user),
):
    try:
        return _success(preview_profile(username, point_cloud_name, payload))
    except Exception as exc:
        return _error(exc)


@router.post("/{point_cloud_name}/draft")
async def save_draft(
    point_cloud_name: str,
    payload: Dict[str, Any] = Body(default_factory=dict),
    username: str = Depends(get_current_user),
):
    try:
        return _success(save_profile(username, point_cloud_name, payload, apply=False))
    except Exception as exc:
        return _error(exc)


@router.post("/{point_cloud_name}/apply")
async def apply_profile(
    point_cloud_name: str,
    payload: Dict[str, Any] = Body(default_factory=dict),
    username: str = Depends(get_current_user),
):
    try:
        return _success(save_profile(username, point_cloud_name, payload, apply=True))
    except Exception as exc:
        return _error(exc)


@router.post("/{point_cloud_name}/restore")
async def restore(
    point_cloud_name: str,
    username: str = Depends(get_current_user),
):
    try:
        return _success(restore_original(username, point_cloud_name))
    except Exception as exc:
        return _error(exc)


@router.get("/{point_cloud_name}/status")
async def status(
    point_cloud_name: str,
    username: str = Depends(get_current_user),
):
    try:
        return _success(profile_status(username, point_cloud_name))
    except Exception as exc:
        return _error(exc)
