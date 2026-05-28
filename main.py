"""
无人机巡检视点规划系统 — 应用入口。

重构后架构：
- backend/server.py — FastAPI 应用工厂
- backend/core/       — 安全认证、依赖注入
- backend/routers/    — 11 个 APIRouter 模块
- backend/services/   — 业务逻辑服务层

前端入口：index.html 在 / 提供，配套资源由 /static 提供。
"""

from backend.server import create_app

app = create_app()

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=8000)
