"""
文件管理路由：GET /api/list/{category}, POST /api/upload/{category}, /api/manage/*
"""

import shutil
from pathlib import Path

from fastapi import APIRouter, Depends, File, Form, UploadFile
from fastapi.responses import JSONResponse

from backend.core.dependencies import get_current_user
from backend.core.security import get_user_dir, safe_filename
from backend.services.file_service import list_user_files
from backend.services.file_service import find_user_file as _find_user_file

router = APIRouter(tags=["files"])


@router.get("/api/list/{category}")
async def list_files(category: str, username: str = Depends(get_current_user)):
    return {"status": "success", "data": list_user_files(username, category)}


@router.post("/api/upload/{category}")
async def upload_file(
    category: str,
    file: UploadFile = File(...),
    username: str = Depends(get_current_user),
):
    try:
        target_dir = get_user_dir(username, category)
        filename = safe_filename(file.filename or "")
        if not filename:
            return {"status": "error", "message": "文件名不能为空"}
        target_path = target_dir / filename
        if target_path.exists():
            return {"status": "error", "message": "文件已存在", "code": "DUPLICATE"}
        with target_path.open("wb") as output:
            shutil.copyfileobj(file.file, output)
        return {"status": "success", "message": "上传成功"}
    except Exception as exc:
        return JSONResponse({"status": "error", "message": str(exc)}, status_code=500)


@router.post("/api/manage/delete")
async def delete_file(
    category: str = Form(...),
    filename: str = Form(...),
    username: str = Depends(get_current_user),
):
    target = get_user_dir(username, category) / safe_filename(filename)
    if not target.exists():
        return {"status": "error", "message": "文件未找到或无权删除"}
    target.unlink()
    return {"status": "success"}


@router.post("/api/manage/rename")
async def rename_file(
    category: str = Form(...),
    old_name: str = Form(...),
    new_name: str = Form(...),
    username: str = Depends(get_current_user),
):
    folder = get_user_dir(username, category)
    old_path = folder / safe_filename(old_name)
    new_path = folder / safe_filename(new_name)
    if not old_path.exists():
        return {"status": "error", "message": "文件未找到"}
    if new_path.exists():
        return {"status": "error", "message": "目标文件名已存在"}
    old_path.rename(new_path)
    return {"status": "success"}
