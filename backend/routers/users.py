"""
用户路由：GET/POST /api/user/profile
"""

from fastapi import APIRouter, Depends, Form

from backend.core.dependencies import get_current_user
from backend.services.file_service import get_profile, rename_user, update_user_profile

router = APIRouter(tags=["users"])


@router.get("/api/user/profile")
async def profile(
    search: str = "",
    username: str = Depends(get_current_user),
):
    return {"status": "success", "data": get_profile(username, search=search or None)}


@router.post("/api/user/profile/update")
async def profile_update(
    new_username: str = Form(""),
    display_name: str = Form(""),
    email: str = Form(""),
    phone: str = Form(""),
    notes: str = Form(""),
    username: str = Depends(get_current_user),
):
    current_name = username
    if new_username and new_username.strip():
        ok, result = rename_user(username, new_username.strip())
        if not ok:
            return {"status": "error", "message": result}
        current_name = result
    update_user_profile(current_name, display_name=display_name, email=email, phone=phone, notes=notes)
    return {"status": "success", "data": get_profile(current_name)}


@router.post("/api/user/profile/rescan")
async def profile_rescan(username: str = Depends(get_current_user)):
    return {"status": "success", "data": get_profile(username)}
