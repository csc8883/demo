"""
管理路由：GET /api/admin/stats
"""

from fastapi import APIRouter, Depends

from backend.core.dependencies import get_current_admin_user
from backend.core.security import get_user_dir, user_manager
from config import USER_CATEGORIES

router = APIRouter(tags=["admin"])


@router.get("/api/admin/stats")
async def admin_stats(username: str = Depends(get_current_admin_user)):
    users = list(user_manager.users)
    file_counts = {category: 0 for category in USER_CATEGORIES}
    disk_usage = {u: 0.0 for u in users}
    total_size_bytes = 0

    for u in users:
        for category in USER_CATEGORIES:
            for file_path in get_user_dir(u, category).iterdir():
                if file_path.is_file():
                    if category == "voxel" and "_weight_profile_" in file_path.name:
                        continue
                    size = file_path.stat().st_size
                    file_counts[category] += 1
                    disk_usage[u] += size / 1024 / 1024
                    total_size_bytes += size

    route_count = file_counts.get("manual_route", 0) + file_counts.get("algorithm_route", 0)
    return {
        "status": "success",
        "data": {
            "user_count": len(users),
            "file_count": int(sum(file_counts.values())),
            "file_counts": {**file_counts, "route": route_count},
            "disk_usage": {name: round(size, 2) for name, size in disk_usage.items()},
            "total_size": f"{total_size_bytes / 1024 / 1024:.2f} MB",
            "total_size_mb": round(total_size_bytes / 1024 / 1024, 2),
        },
    }
