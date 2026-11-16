"""
认证路由：POST /api/auth/login, /register
"""

from fastapi import APIRouter, Form, Request, Response, HTTPException
from backend.core.security import user_manager

from backend.services.auth_service import authenticate_user, build_login_response, register_user

router = APIRouter(tags=["auth"])


@router.post("/api/auth/login")
async def login(response: Response, username: str = Form(...), password: str = Form(...)):
    record = authenticate_user(username, password)
    if not record:
        return {"status": "error", "message": "用户名或密码错误"}
    result = build_login_response(username, record)
    response.set_cookie('planning_session', result['data']['token'], max_age=30 * 86400,
                        httponly=True, samesite='lax', path='/')
    return result


@router.get('/api/auth/me')
async def me(request: Request):
    token = request.cookies.get('planning_session')
    username = user_manager.validate_session(token)
    if not username:
        raise HTTPException(status_code=401, detail='登录已过期，请重新登录')
    record = user_manager.users[username]
    return {'status': 'success', 'data': {'name': username,
            'display_name': record.get('display_name') or username,
            'role': record.get('role', 'user'), 'token': token}}


@router.post('/api/auth/logout')
async def logout(request: Request, response: Response):
    user_manager.revoke_session(request.cookies.get('planning_session') or request.headers.get('x-auth-token'))
    response.delete_cookie('planning_session', path='/')
    return {'status': 'success'}


@router.post("/api/auth/register")
async def register(username: str = Form(...), password: str = Form(...)):
    if not register_user(username, password):
        return {"status": "error", "message": "用户名已存在或格式不合法"}
    return {"status": "success"}
