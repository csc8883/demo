"""Potree/LOD cache status and preparation helpers."""

from __future__ import annotations

import os
import hashlib
import json
import shutil
import subprocess
from pathlib import Path
from threading import Lock
from typing import Any, Dict, Optional
from urllib.parse import quote

import laspy
import numpy as np

from backend.core.security import safe_filename
from backend.services.file_service import find_user_file
from config import POINTCLOUD_LOD_DIR


_jobs: Dict[str, Dict[str, Any]] = {}
_jobs_lock = Lock()
CLASS_RGB_CACHE_VERSION = "classrgb_v2"
CLASSIFICATION_RGB16: Dict[int, tuple[int, int, int]] = {
    0: (2621, 32768, 11796),
    1: (27525, 31457, 36700),
    2: (31457, 24903, 16384),
    3: (26214, 65535, 39321),
    4: (18350, 40632, 22282),
    5: (11796, 32768, 16384),
    6: (36044, 36700, 38010),
    7: (51118, 31457, 60292),
    8: (6554, 42598, 51118),
    9: (13107, 29491, 55705),
    10: (49807, 36044, 11796),
    11: (47185, 20971, 20971),
    12: (20971, 40632, 47185),
    15: (65535, 34078, 10486),
    16: (65535, 3277, 3277),
    22: (3277, 22937, 65535),
    24: (15073, 33423, 62914),
    25: (62914, 40632, 2621),
    26: (61604, 17694, 17694),
}
DEFAULT_CLASS_RGB16 = (23593, 26214, 30147)


def _safe_user(username: str) -> str:
    return safe_filename(username or "user") or "user"


def _cloud_stem(point_cloud_name: str) -> str:
    return Path(safe_filename(point_cloud_name)).stem or "pointcloud"


def _normalize_variant(variant: Optional[str]) -> str:
    value = str(variant or "base").strip().lower()
    if value in {"active_weight", "weight", "weighted", "weight_profile"}:
        return "active_weight"
    return "base"


def _safe_profile_id(profile_id: Optional[str]) -> str:
    return "".join(ch for ch in str(profile_id or "active") if ch.isalnum() or ch in ("-", "_"))[:64] or "active"


def _profile_digest(profile: Dict[str, Any]) -> str:
    payload = json.dumps(
        {
            "profile_id": profile.get("profile_id"),
            "groups": profile.get("groups") or [],
            "policy": profile.get("policy") or {},
            "updated_at": profile.get("updated_at"),
        },
        ensure_ascii=False,
        sort_keys=True,
        default=str,
    )
    return hashlib.sha1(payload.encode("utf-8")).hexdigest()[:10]


def _cache_dir(username: str, cache_stem: str) -> Path:
    return POINTCLOUD_LOD_DIR / _safe_user(username) / cache_stem


def _job_key(username: str, cache_stem: str) -> str:
    return f"{_safe_user(username)}:{cache_stem}"


def _find_manifest(cache_dir: Path) -> Optional[Path]:
    for candidate in ("metadata.json", "cloud.js"):
        path = cache_dir / candidate
        if path.exists():
            return path
    return None


def _static_lod_url(path: Path) -> str:
    relative = path.relative_to(POINTCLOUD_LOD_DIR).as_posix()
    return f"/static/lod/{quote(relative, safe='/')}"


def _find_converter() -> Optional[str]:
    configured = os.environ.get("POTREE_CONVERTER_PATH")
    if configured and Path(configured).exists():
        return configured
    return shutil.which("PotreeConverter") or shutil.which("PotreeConverter.exe")


def _source_point_cloud(username: str, point_cloud_name: str) -> Path:
    source = find_user_file(username, "point_cloud", safe_filename(point_cloud_name))
    if not source:
        raise FileNotFoundError("Point cloud file not found")
    return source


def _resolve_lod_target(
    username: str,
    point_cloud_name: str,
    variant: Optional[str] = "base",
    profile_id: Optional[str] = None,
) -> Dict[str, Any]:
    source = _source_point_cloud(username, point_cloud_name)
    normalized_variant = _normalize_variant(variant)
    profile: Optional[Dict[str, Any]] = None
    cache_stem = f"{_cloud_stem(point_cloud_name)}_{CLASS_RGB_CACHE_VERSION}"
    visual_source: Optional[Path] = None
    resolved_profile_id: Optional[str] = None
    if normalized_variant == "base":
        visual_source = POINTCLOUD_LOD_DIR / _safe_user(username) / "_classified_sources" / f"{cache_stem}.las"

    if normalized_variant == "active_weight":
        from backend.services.weight_service import get_active_profile, get_profile_by_id

        profile = get_profile_by_id(username, point_cloud_name, profile_id) if profile_id else get_active_profile(username, point_cloud_name)
        if not profile:
            raise FileNotFoundError("Active weight profile not found")
        resolved_profile_id = str(profile.get("profile_id") or profile_id or "active")
        cache_stem = (
            f"{_cloud_stem(point_cloud_name)}_weighted_"
            f"{_safe_profile_id(resolved_profile_id)}_{_profile_digest(profile)}_{CLASS_RGB_CACHE_VERSION}"
        )
        visual_source = POINTCLOUD_LOD_DIR / _safe_user(username) / "_weighted_sources" / f"{cache_stem}.las"

    cache_dir = _cache_dir(username, cache_stem)
    return {
        "variant": normalized_variant,
        "source": source,
        "cache_stem": cache_stem,
        "cache_dir": cache_dir,
        "job_key": _job_key(username, cache_stem),
        "profile": profile,
        "profile_id": resolved_profile_id,
        "visual_source": visual_source,
    }


def lod_status(
    username: str,
    point_cloud_name: str,
    variant: Optional[str] = "base",
    profile_id: Optional[str] = None,
) -> Dict[str, Any]:
    target = _resolve_lod_target(username, point_cloud_name, variant=variant, profile_id=profile_id)
    source = target["source"]
    cache_dir = target["cache_dir"]
    manifest = _find_manifest(cache_dir)
    converter = _find_converter()
    with _jobs_lock:
        job = dict(_jobs.get(target["job_key"]) or {})

    if manifest:
        status = "ready"
        message = "LOD cache is ready"
    elif job.get("status") in {"queued", "running", "failed"}:
        status = str(job.get("status"))
        message = str(job.get("message") or status)
    elif converter:
        status = "not_prepared"
        message = "LOD cache has not been generated"
    else:
        status = "converter_missing"
        message = "PotreeConverter was not found"

    payload: Dict[str, Any] = {
        "renderer": "potree-lod",
        "status": status,
        "message": message,
        "variant": target["variant"],
        "source_filename": source.name,
        "source_size_bytes": source.stat().st_size if source.exists() else 0,
        "cache_key": f"{_safe_user(username)}/{target['cache_stem']}",
        "converter_available": bool(converter),
        "converter_path": converter,
    }
    if target["profile_id"]:
        payload["weight_profile_id"] = target["profile_id"]
    if target["visual_source"]:
        payload["visual_source_available"] = target["visual_source"].exists()
    if manifest:
        payload["manifest_url"] = _static_lod_url(manifest)
        payload["cache_url"] = _static_lod_url(cache_dir)
    if job:
        payload["job"] = job
    return payload


def _run_conversion(username: str, point_cloud_name: str, target: Dict[str, Any], converter: str) -> None:
    key = target["job_key"]
    with _jobs_lock:
        _jobs[key] = {"status": "running", "message": "PotreeConverter is running"}
    cache_dir = target["cache_dir"]
    cache_dir.mkdir(parents=True, exist_ok=True)
    timeout = int(os.environ.get("POTREE_CONVERTER_TIMEOUT_SECONDS", "3600"))
    source = target["source"]
    cmd = [converter, str(source), "-o", str(cache_dir)]
    try:
        if target["variant"] == "active_weight":
            from backend.services.weight_service import write_weighted_visual_las

            write_weighted_visual_las(
                username,
                point_cloud_name,
                target["visual_source"],
                profile_id=target["profile_id"],
            )
            source = target["visual_source"]
            cmd = [converter, str(source), "-o", str(cache_dir)]
        elif target["variant"] == "base" and target.get("visual_source"):
            _write_classified_rgb_las(source, target["visual_source"])
            source = target["visual_source"]
            cmd = [converter, str(source), "-o", str(cache_dir)]
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
        manifest = _find_manifest(cache_dir)
        if result.returncode == 0 and manifest:
            status = {"status": "ready", "message": "LOD cache is ready"}
        else:
            output = (result.stderr or result.stdout or "PotreeConverter did not produce a manifest").strip()
            status = {
                "status": "failed",
                "message": output[-800:],
                "returncode": result.returncode,
            }
    except Exception as exc:
        status = {"status": "failed", "message": str(exc)}
    with _jobs_lock:
        _jobs[key] = status


def _write_classified_rgb_las(source_path: Path, target_path: Path) -> Dict[str, Any]:
    """Create a display-only LAS copy whose RGB channels encode classification colors."""
    with laspy.open(source_path) as source:
        las = source.read()

    if not (hasattr(las, "red") and hasattr(las, "green") and hasattr(las, "blue")):
        target_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source_path, target_path)
        return {"visual_source": str(target_path), "colorized": False, "reason": "source_has_no_rgb"}

    labels = np.asarray(las.classification, dtype=np.int32)
    red = np.full(labels.shape, DEFAULT_CLASS_RGB16[0], dtype=np.uint16)
    green = np.full(labels.shape, DEFAULT_CLASS_RGB16[1], dtype=np.uint16)
    blue = np.full(labels.shape, DEFAULT_CLASS_RGB16[2], dtype=np.uint16)

    for label, (r_value, g_value, b_value) in CLASSIFICATION_RGB16.items():
        mask = labels == int(label)
        if not np.any(mask):
            continue
        red[mask] = r_value
        green[mask] = g_value
        blue[mask] = b_value

    las.red = red
    las.green = green
    las.blue = blue
    _strip_display_vlrs(las)
    target_path.parent.mkdir(parents=True, exist_ok=True)
    if target_path.exists():
        target_path.unlink()
    las.write(target_path)
    return {"visual_source": str(target_path), "colorized": True}


def _strip_display_vlrs(las: laspy.LasData) -> None:
    """Drop optional metadata records that can contain non-ASCII bytes when rewriting display copies."""
    try:
        las.vlrs.clear()
        las.header.vlrs.clear()
        if getattr(las, "evlrs", None):
            las.evlrs.clear()
    except Exception:
        pass


def prepare_lod(
    username: str,
    point_cloud_name: str,
    background_tasks,
    variant: Optional[str] = "base",
    profile_id: Optional[str] = None,
) -> Dict[str, Any]:
    target = _resolve_lod_target(username, point_cloud_name, variant=variant, profile_id=profile_id)
    current = lod_status(username, point_cloud_name, variant=variant, profile_id=profile_id)
    if current["status"] in {"ready", "queued", "running"}:
        return current

    converter = _find_converter()
    if not converter:
        return current

    with _jobs_lock:
        _jobs[target["job_key"]] = {"status": "queued", "message": "LOD conversion has been queued"}
    background_tasks.add_task(_run_conversion, username, point_cloud_name, target, converter)
    return lod_status(username, point_cloud_name, variant=variant, profile_id=profile_id)
