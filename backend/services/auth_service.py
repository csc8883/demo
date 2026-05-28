"""
认证服务：封装 UserManager 操作，供 auth router 调用。
"""

from typing import Any, Dict, Optional

from backend.core.security import user_manager


def authenticate_user(username: str, password: str) -> Optional[Dict[str, Any]]:
    """验证用户凭据，返回用户记录或 None。"""
    return user_manager.authenticate(username, password)


def register_user(username: str, password: str) -> bool:
    """注册新用户。"""
    return user_manager.register(username, password)


def create_session(username: str) -> str:
    """为用户创建会话 token。"""
    return user_manager.create_session(username)


def build_login_response(username: str, record: Dict[str, Any]) -> dict:
    """构建前端期望的登录响应格式。"""
    token = create_session(username)
    return {
        "status": "success",
        "data": {
            "name": username,
            "display_name": record.get("display_name") or username,
            "role": record.get("role", "user"),
            "token": token,
        },
    }
