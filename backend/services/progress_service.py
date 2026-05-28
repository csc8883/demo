"""
WebSocket 实时进度推送服务。

ProgressBroker 按 task_key 管理 WebSocket 连接组，
当后台任务更新 process_status 时广播进度给所有订阅者。
"""

from __future__ import annotations

import asyncio
import json
import logging
from typing import Dict, Set

from fastapi import WebSocket

logger = logging.getLogger(__name__)


class ProgressBroker:
    """按 task_key 分组管理 WebSocket 连接的中间人。"""

    def __init__(self) -> None:
        # {task_key: {WebSocket, ...}}
        self._connections: Dict[str, Set[WebSocket]] = {}

    async def connect(self, ws: WebSocket, task_key: str) -> None:
        """接受 WebSocket 连接并注册到指定的 task_key 组。"""
        await ws.accept()
        self._connections.setdefault(task_key, set()).add(ws)
        logger.info("WS connect: task=%s, clients=%d", task_key, len(self._connections[task_key]))

    def disconnect(self, ws: WebSocket, task_key: str) -> None:
        """从组中移除连接，如果组为空则清理组。"""
        group = self._connections.get(task_key)
        if group:
            group.discard(ws)
            if not group:
                del self._connections[task_key]
        logger.info("WS disconnect: task=%s, clients=%d", task_key, len(self._connections.get(task_key, set())))

    async def broadcast(self, task_key: str, message: dict) -> None:
        """向某个 task_key 组的全部连接发送消息。"""
        group = self._connections.get(task_key, set())
        if not group:
            return

        payload = json.dumps(message, ensure_ascii=False)
        dead: list[WebSocket] = []

        for ws in list(group):
            try:
                await ws.send_text(payload)
            except Exception:
                dead.append(ws)

        for ws in dead:
            self.disconnect(ws, task_key)

    async def broadcast_all(self, message: dict) -> None:
        """向所有连接的客户端广播消息。"""
        tasks = [self.broadcast(key, message) for key in list(self._connections.keys())]
        if tasks:
            await asyncio.gather(*tasks)


# 全局单例
progress_broker = ProgressBroker()
