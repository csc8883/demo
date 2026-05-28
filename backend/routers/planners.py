"""
规划器路由：GET /api/planners
"""

from fastapi import APIRouter

from backend.services.processing_service import PLANNER_REGISTRY

router = APIRouter(tags=["planners"])


@router.get("/api/planners")
async def get_planners():
    return {
        "status": "success",
        "data": [
            {
                "key": key,
                "name": cfg["name"],
                "description": cfg.get("description", ""),
                "parameters": cfg.get("parameters", []),
            }
            for key, cfg in PLANNER_REGISTRY.items()
        ],
    }
