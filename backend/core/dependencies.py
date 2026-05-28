"""
FastAPI 依赖注入：从请求头提取并验证当前用户。
"""

from typing import Optional

from fastapi import Depends, Header, HTTPException, Request

from .security import user_manager


async def get_current_user(
    request: Request,
    x_user_name: Optional[str] = Header(None),
    x_auth_token: Optional[str] = Header(None),
) -> str:
    """验证会话 token 并返回当前用户名。未认证时抛出 401。"""
    username = user_manager.validate_session(x_auth_token)
    if not username:
        raise HTTPException(status_code=401, detail="登录已过期，请重新登录")
    if x_user_name and x_user_name != username:
        raise HTTPException(status_code=401, detail="用户身份不匹配")
    request.state.username = username
    return username


async def get_current_admin_user(
    username: str = Depends(get_current_user),
) -> str:
    """验证当前用户为管理员。"""
    record = user_manager.users.get(username, {})
    if record.get("role") != "admin":
        raise HTTPException(status_code=403, detail="需要管理员权限")
    return username
