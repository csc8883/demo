"""
FastAPI 应用工厂。

将所有 APIRouter 组装成一个 FastAPI 实例，
同时提供唯一前端的静态文件服务和 Jinja2 模板渲染。
"""

import logging

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from starlette.requests import Request

from config import (
    LOG_DIR,
    STATIC_DIR,
    TEMPLATE_DIR,
    USER_DATA_DIR,
)


LOG_DIR.mkdir(exist_ok=True)
USER_DATA_DIR.mkdir(exist_ok=True)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler(LOG_DIR / "app.log", encoding="utf-8"),
    ],
)
logger = logging.getLogger(__name__)


def create_app() -> FastAPI:
    app = FastAPI(title="Power Grid Inspection Basic Deployment (Refactored)")

    # CORS
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # 静态文件
    app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")

    templates = Jinja2Templates(directory=str(TEMPLATE_DIR))

    # 初始化规划器注册表
    from backend.services.processing_service import _init_planner_registry
    _init_planner_registry()

    # 注册所有 APIRouter
    from backend.routers import (
        admin,
        auth,
        compare,
        export,
        files,
        planners,
        processing,
        routes,
        users,
        visualization,
        ws,
    )
    app.include_router(auth.router)
    app.include_router(users.router)
    app.include_router(files.router)
    app.include_router(planners.router)
    app.include_router(visualization.router)
    app.include_router(processing.router)
    app.include_router(routes.router)
    app.include_router(compare.router)
    app.include_router(export.router)
    app.include_router(admin.router)
    app.include_router(ws.router)

    # 唯一前端 — 根路径渲染 index.html
    @app.get("/")
    async def index(request: Request):
        return templates.TemplateResponse(request, "index.html")

    return app
