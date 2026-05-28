"""
导出路由：GET /api/export/waypoint, /route
"""

from fastapi import APIRouter, Depends
from fastapi.responses import FileResponse

from backend.core.dependencies import get_current_user
from backend.services.file_service import find_user_file

router = APIRouter(tags=["export"])


@router.get("/api/export/waypoint")
async def export_waypoint(filename: str, username: str = Depends(get_current_user)):
    route_path = find_user_file(username, "waypoint", filename)
    if not route_path:
        return {"status": "error", "message": "File not found"}
    return FileResponse(path=str(route_path), media_type="application/json", filename=route_path.name)


@router.get("/api/export/route")
async def export_route(
    filename: str,
    category: str = "algorithm_route",
    username: str = Depends(get_current_user),
):
    if category not in {"manual_route", "algorithm_route"}:
        return {"status": "error", "message": "Invalid route category"}
    route_path = find_user_file(username, category, filename)
    if not route_path:
        return {"status": "error", "message": "File not found"}
    return FileResponse(path=str(route_path), media_type="application/json", filename=route_path.name)
