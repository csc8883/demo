"""
认证路由：POST /api/auth/login, /register
"""

from fastapi import APIRouter, Form

from backend.services.auth_service import authenticate_user, build_login_response, register_user

router = APIRouter(tags=["auth"])


@router.post("/api/auth/login")
async def login(username: str = Form(...), password: str = Form(...)):
    record = authenticate_user(username, password)
    if not record:
        return {"status": "error", "message": "用户名或密码错误"}
    return build_login_response(username, record)


@router.post("/api/auth/register")
async def register(username: str = Form(...), password: str = Form(...)):
    if not register_user(username, password):
        return {"status": "error", "message": "用户名已存在或格式不合法"}
    return {"status": "success"}
