"""
WebSocket 路由：/api/ws/progress
"""

from fastapi import APIRouter, WebSocket, WebSocketDisconnect, Query

from backend.services.progress_service import progress_broker

router = APIRouter(tags=["ws"])


@router.websocket("/api/ws/progress")
async def ws_progress(
    websocket: WebSocket,
    task: str = Query(""),
    username: str = Query(""),
):
    task_key = f"{username}:{task}" if username else task
    await progress_broker.connect(websocket, task_key)
    try:
        while True:
            # 保持连接，客户端可随时断开
            await websocket.receive_text()
    except WebSocketDisconnect:
        pass
    except Exception:
        pass
    finally:
        progress_broker.disconnect(websocket, task_key)
