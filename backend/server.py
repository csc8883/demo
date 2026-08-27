"""
FastAPI 应用工厂。

将所有 APIRouter 组装成一个 FastAPI 实例，
同时提供唯一前端的静态文件服务和 Jinja2 模板渲染。
"""

import logging

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, HTMLResponse
from fastapi.staticfiles import StaticFiles
from starlette.requests import Request

from config import (
    FRONTEND_DIST_DIR,
    LOG_DIR,
    POINTCLOUD_LOD_DIR,
    STATIC_DIR,
    USER_DATA_DIR,
)


LOG_DIR.mkdir(exist_ok=True)
USER_DATA_DIR.mkdir(exist_ok=True)
POINTCLOUD_LOD_DIR.mkdir(parents=True, exist_ok=True)

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
    app.mount("/static/lod", StaticFiles(directory=str(POINTCLOUD_LOD_DIR), check_dir=False), name="pointcloud-lod")
    app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")
    app.mount(
        "/assets",
        StaticFiles(directory=str(FRONTEND_DIST_DIR / "assets"), check_dir=False),
        name="frontend-assets",
    )

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
        pointcloud_lod,
        processing,
        routes,
        users,
        visualization,
        weights,
        ws,
    )
    app.include_router(auth.router)
    app.include_router(users.router)
    app.include_router(files.router)
    app.include_router(planners.router)
    app.include_router(pointcloud_lod.router)
    app.include_router(visualization.router)
    app.include_router(weights.router)
    app.include_router(processing.router)
    app.include_router(routes.router)
    app.include_router(compare.router)
    app.include_router(export.router)
    app.include_router(admin.router)
    app.include_router(ws.router)

    # 唯一前端入口：生产模式只服务 Vite build，避免回退到旧 static/js 页面。
    def frontend_entry_response(request: Request):
        frontend_index = FRONTEND_DIST_DIR / "index.html"
        if frontend_index.exists():
            return FileResponse(
                frontend_index,
                headers={"Cache-Control": "no-store"},
            )
        return HTMLResponse(
            """
            <!doctype html>
            <meta charset="utf-8">
            <title>Frontend build missing</title>
            <body style="font-family:system-ui;padding:32px;line-height:1.6">
              <h1>React/Vite 前端尚未构建</h1>
              <p>请先在 <code>frontend</code> 目录执行 <code>npm install</code> 和 <code>npm run build</code>，然后重新启动 FastAPI。</p>
              <p>开发模式可同时运行 FastAPI 与 <code>npm run dev</code>。</p>
            </body>
            """,
            status_code=503,
        )

    @app.get("/")
    async def index(request: Request):
        return frontend_entry_response(request)

    @app.get("/{full_path:path}")
    async def spa_fallback(request: Request, full_path: str):
        return frontend_entry_response(request)

    return app
