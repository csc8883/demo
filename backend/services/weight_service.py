"""Custom point-cloud weight profile persistence and geometry evaluation."""

from __future__ import annotations

import json
import math
import os
import uuid
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import laspy
import numpy as np

from backend.core.security import get_user_dir, safe_filename
from backend.services.file_service import find_user_file


TOWER_LABEL = 16
INSULATOR_LABEL = 22
CONDUCTOR_LABEL = 0
GROUND_WIRE_LABEL = 3
EDITABLE_LABELS = (TOWER_LABEL, INSULATOR_LABEL)
CONTEXT_LABELS = (CONDUCTOR_LABEL, GROUND_WIRE_LABEL)
LEVEL_PRIORITY = {"normal": 1, "needed": 2, "important": 3}
LEVEL_MULTIPLIER = {"normal": 1.0, "needed": 2.0, "important": 3.0}
LEVEL_DIRECTIONS = {"normal": 1, "needed": 3, "important": 4}
LEVEL_COLORS = {"normal": "#3b82f6", "needed": "#f59e0b", "important": "#ef4444"}
LEVEL_LOD_CLASSIFICATION = {"normal": 24, "needed": 25, "important": 26}
DEFAULT_VOXEL_SIZE = 0.10
CLASSIFICATION_RGB16: Dict[int, Tuple[int, int, int]] = {
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
}
DEFAULT_CLASS_RGB16 = (23593, 26214, 30147)


def _now() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def _profile_id() -> str:
    return uuid.uuid4().hex[:12]


def _point_cloud_base(point_cloud_name: str) -> str:
    return Path(safe_filename(point_cloud_name)).stem


def _hex_to_rgb16(value: Any, fallback: str = "#3b82f6") -> Tuple[int, int, int]:
    text = str(value or fallback).strip().lstrip("#")
    if len(text) != 6 or any(ch not in "0123456789abcdefABCDEF" for ch in text):
        text = fallback.lstrip("#")
    return tuple(int(text[index:index + 2], 16) * 257 for index in (0, 2, 4))


def _profile_pattern(point_cloud_name: str) -> str:
    return f"{_point_cloud_base(point_cloud_name)}_weight_profile_*.json"


def profile_path(username: str, point_cloud_name: str, profile_id: str) -> Path:
    clean_id = "".join(ch for ch in str(profile_id) if ch.isalnum() or ch in ("-", "_"))[:64]
    if not clean_id:
        raise ValueError("profile_id 无效")
    return get_user_dir(username, "voxel") / (
        f"{_point_cloud_base(point_cloud_name)}_weight_profile_{clean_id}.json"
    )


def list_profiles(username: str, point_cloud_name: str) -> List[Dict[str, Any]]:
    profiles: List[Dict[str, Any]] = []
    for path in get_user_dir(username, "voxel").glob(_profile_pattern(point_cloud_name)):
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
            if payload.get("base_point_cloud") == safe_filename(point_cloud_name):
                profiles.append(payload)
        except Exception:
            continue
    profiles.sort(key=lambda item: str(item.get("updated_at") or ""), reverse=True)
    return profiles


def get_profile_by_id(
    username: str,
    point_cloud_name: str,
    profile_id: Optional[str],
) -> Optional[Dict[str, Any]]:
    if not profile_id:
        return None
    path = profile_path(username, point_cloud_name, profile_id)
    if not path.exists():
        return None
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise ValueError(f"权重 profile 无法解析: {exc}") from exc
    if payload.get("base_point_cloud") != safe_filename(point_cloud_name):
        return None
    return payload


def get_active_profile(username: str, point_cloud_name: str) -> Optional[Dict[str, Any]]:
    return next(
        (
            profile
            for profile in list_profiles(username, point_cloud_name)
            if profile.get("status") == "applied" and bool(profile.get("active"))
        ),
        None,
    )


def _atomic_write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    try:
        temp.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        os.replace(temp, path)
    finally:
        if temp.exists():
            temp.unlink(missing_ok=True)


def _load_las_targets(point_cloud_path: Path) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    with laspy.open(point_cloud_path) as source:
        las = source.read()
        labels = np.asarray(las.classification, dtype=int)
        indices = np.flatnonzero(np.isin(labels, EDITABLE_LABELS))
        if len(indices) == 0:
            raise ValueError("点云中没有 classification=16/22 的可赋权目标")
        points = np.column_stack(
            (
                np.asarray(las.x, dtype=float)[indices],
                np.asarray(las.y, dtype=float)[indices],
                np.asarray(las.z, dtype=float)[indices],
            )
        )
        return points, labels[indices], indices.astype(np.int64)


def _load_las_context_points(
    point_cloud_path: Path,
    limit: int,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, Dict[str, int]]:
    """Sample non-editable semantic points for UI-only selection impact hints."""
    with laspy.open(point_cloud_path) as source:
        las = source.read()
        labels = np.asarray(las.classification, dtype=int)
        context_indices = np.flatnonzero(np.isin(labels, CONTEXT_LABELS))
        totals = {
            "conductor_count": int(np.sum(labels == CONDUCTOR_LABEL)),
            "ground_wire_count": int(np.sum(labels == GROUND_WIRE_LABEL)),
        }
        if len(context_indices) == 0:
            return (
                np.empty((0, 3), dtype=float),
                np.empty((0,), dtype=int),
                np.empty((0,), dtype=np.int64),
                totals,
            )
        points = np.column_stack(
            (
                np.asarray(las.x, dtype=float)[context_indices],
                np.asarray(las.y, dtype=float)[context_indices],
                np.asarray(las.z, dtype=float)[context_indices],
            )
        )
        labels = labels[context_indices]
        budget = max(0, min(int(limit), len(points)))
        selected = _spatial_xz_sample_indices(points, budget) if budget else np.array([], dtype=np.int64)
        return (
            points[selected],
            labels[selected],
            context_indices[selected].astype(np.int64),
            totals,
        )


def _spatial_xz_sample_indices(points: np.ndarray, budget: int) -> np.ndarray:
    """Select deterministic representatives with broad XZ screen-space coverage."""
    count = len(points)
    budget = max(0, min(int(budget), count))
    if budget == 0:
        return np.array([], dtype=np.int64)
    if count <= budget:
        return np.arange(count, dtype=np.int64)

    xz = np.asarray(points, dtype=float)[:, (0, 2)]
    minimum = np.min(xz, axis=0)
    span = np.maximum(np.max(xz, axis=0) - minimum, 0.0)
    if span[0] <= 1e-9 and span[1] <= 1e-9:
        return np.linspace(0, count - 1, budget, dtype=np.int64)

    if span[0] <= 1e-9:
        columns, rows = 1, budget
    elif span[1] <= 1e-9:
        columns, rows = budget, 1
    else:
        aspect = float(span[0] / span[1])
        columns = min(budget, max(1, int(round(math.sqrt(budget * aspect)))))
        rows = max(1, budget // columns)

    normalized = (xz - minimum) / np.where(span > 1e-9, span, 1.0)
    grid_x = np.minimum((normalized[:, 0] * columns).astype(np.int64), columns - 1)
    grid_z = np.minimum((normalized[:, 1] * rows).astype(np.int64), rows - 1)
    cell_ids = grid_z * columns + grid_x
    center_x = (grid_x + 0.5) / columns
    center_z = (grid_z + 0.5) / rows
    center_distance = (normalized[:, 0] - center_x) ** 2 + (normalized[:, 1] - center_z) ** 2

    original_order = np.arange(count, dtype=np.int64)
    ordered = np.lexsort((original_order, center_distance, cell_ids))
    ordered_cells = cell_ids[ordered]
    first_in_cell = np.empty(count, dtype=bool)
    first_in_cell[0] = True
    first_in_cell[1:] = ordered_cells[1:] != ordered_cells[:-1]
    representatives = ordered[first_in_cell]

    chosen = np.zeros(count, dtype=bool)
    chosen[representatives] = True
    remaining_budget = budget - len(representatives)
    if remaining_budget > 0:
        remaining = ordered[~chosen[ordered]]
        supplemental_positions = np.linspace(
            0,
            len(remaining) - 1,
            remaining_budget,
            dtype=np.int64,
        )
        representatives = np.concatenate((representatives, remaining[supplemental_positions]))

    return np.sort(representatives[:budget].astype(np.int64))


def _editable_sample_indices(points: np.ndarray, labels: np.ndarray, limit: int) -> np.ndarray:
    count = len(points)
    limit = max(1, min(int(limit), count))
    if count <= limit:
        return np.arange(count, dtype=np.int64)

    tower = np.flatnonzero(labels == TOWER_LABEL)
    insulator = np.flatnonzero(labels == INSULATOR_LABEL)
    insulator_budget = min(len(insulator), max(int(limit * 0.35), 1))
    tower_budget = min(len(tower), limit - insulator_budget)
    remaining = limit - tower_budget - insulator_budget
    if remaining > 0:
        insulator_budget += min(len(insulator) - insulator_budget, remaining)
        remaining = limit - tower_budget - insulator_budget
    if remaining > 0:
        tower_budget += min(len(tower) - tower_budget, remaining)

    tower_local = _spatial_xz_sample_indices(points[tower], tower_budget)
    insulator_local = _spatial_xz_sample_indices(points[insulator], insulator_budget)
    selected = np.concatenate((tower[tower_local], insulator[insulator_local]))
    return np.sort(selected.astype(np.int64))


def _normalize_rect(rect: Any) -> Optional[Tuple[float, float, float, float]]:
    if isinstance(rect, dict):
        values = (
            rect.get("min_x", rect.get("x1")),
            rect.get("max_x", rect.get("x2")),
            rect.get("min_y", rect.get("y1")),
            rect.get("max_y", rect.get("y2")),
        )
    elif isinstance(rect, Sequence) and len(rect) >= 4:
        values = rect[:4]
    else:
        return None
    try:
        x1, x2, y1, y2 = (float(value) for value in values)
    except (TypeError, ValueError):
        return None
    return min(x1, x2), max(x1, x2), min(y1, y2), max(y1, y2)


def _box3d_mask(points: np.ndarray, geometry: Dict[str, Any]) -> np.ndarray:
    rect = _normalize_rect(geometry.get("ndc_rect") or geometry.get("rect"))
    matrix_values = geometry.get("view_projection_matrix") or geometry.get("matrix")
    if rect is None or not isinstance(matrix_values, Sequence) or len(matrix_values) != 16:
        return np.zeros(len(points), dtype=bool)
    try:
        matrix = np.asarray(matrix_values, dtype=float).reshape((4, 4), order="F")
    except (TypeError, ValueError):
        return np.zeros(len(points), dtype=bool)
    homogeneous = np.column_stack((points, np.ones(len(points), dtype=float)))
    clip = homogeneous @ matrix.T
    w = clip[:, 3]
    valid = np.isfinite(w) & (np.abs(w) > 1e-9)
    ndc = np.full((len(points), 3), np.nan, dtype=float)
    ndc[valid] = clip[valid, :3] / w[valid, None]
    min_x, max_x, min_y, max_y = rect
    return (
        valid
        & (ndc[:, 0] >= min_x)
        & (ndc[:, 0] <= max_x)
        & (ndc[:, 1] >= min_y)
        & (ndc[:, 1] <= max_y)
        & (ndc[:, 2] >= -1.0)
        & (ndc[:, 2] <= 1.0)
    )


def _front_xz_mask(points: np.ndarray, geometry: Dict[str, Any]) -> np.ndarray:
    bounds = geometry.get("bounds") or geometry
    try:
        x1 = float(bounds.get("min_x", bounds.get("x1")))
        x2 = float(bounds.get("max_x", bounds.get("x2")))
        z1 = float(bounds.get("min_z", bounds.get("z1")))
        z2 = float(bounds.get("max_z", bounds.get("z2")))
    except (AttributeError, TypeError, ValueError):
        return np.zeros(len(points), dtype=bool)
    min_x, max_x = sorted((x1, x2))
    min_z, max_z = sorted((z1, z2))
    return (
        (points[:, 0] >= min_x)
        & (points[:, 0] <= max_x)
        & (points[:, 2] >= min_z)
        & (points[:, 2] <= max_z)
    )


def selection_mask(
    points: np.ndarray,
    geometry: Dict[str, Any],
    default_tool: str = "box3d",
) -> np.ndarray:
    tool = str(geometry.get("tool") or geometry.get("selection_tool") or default_tool)
    if tool == "front_xz":
        return _front_xz_mask(points, geometry)
    return _box3d_mask(points, geometry)


def evaluate_operations(
    points: np.ndarray,
    operations: Iterable[Dict[str, Any]],
    default_tool: str = "box3d",
) -> np.ndarray:
    current = np.zeros(len(points), dtype=bool)
    for operation in operations or []:
        if not isinstance(operation, dict):
            continue
        mode = str(operation.get("mode") or operation.get("selection_mode") or "new")
        geometry = operation.get("geometry") if isinstance(operation.get("geometry"), dict) else operation
        selected = selection_mask(points, geometry, default_tool=default_tool)
        if mode == "add":
            current |= selected
        elif mode == "subtract":
            current &= ~selected
        elif mode == "invert":
            current = ~current
        else:
            current = selected.copy()
    return current


def group_mask(points: np.ndarray, labels: np.ndarray, group: Dict[str, Any]) -> np.ndarray:
    selection_geometry = group.get("selection_geometry") or {}
    operations = (
        selection_geometry.get("operations", [])
        if isinstance(selection_geometry, dict)
        else []
    )
    mask = evaluate_operations(
        points,
        operations,
        default_tool=str(group.get("selection_tool") or "box3d"),
    )
    semantic_filter = set(group.get("semantic_filter") or ("tower", "insulator"))
    semantic_mask = np.zeros(len(points), dtype=bool)
    if "tower" in semantic_filter:
        semantic_mask |= labels == TOWER_LABEL
    if "insulator" in semantic_filter:
        semantic_mask |= labels == INSULATOR_LABEL
    return mask & semantic_mask


def resolve_group_assignments(
    points: np.ndarray,
    labels: np.ndarray,
    groups: Sequence[Dict[str, Any]],
) -> Tuple[np.ndarray, List[np.ndarray], int]:
    assignments = np.full(len(points), -1, dtype=int)
    winning_priorities = np.full(len(points), -1, dtype=np.int16)
    winning_revisions = np.full(len(points), -1, dtype=np.int64)
    group_masks: List[np.ndarray] = []
    membership_count = np.zeros(len(points), dtype=np.int16)
    for group_index, group in enumerate(groups):
        if not bool(group.get("enabled", True)):
            group_masks.append(np.zeros(len(points), dtype=bool))
            continue
        mask = group_mask(points, labels, group)
        group_masks.append(mask)
        membership_count += mask.astype(np.int16)
        level = str(group.get("level") or "normal")
        revision_seq = max(0, int(group.get("revision_seq") or group_index + 1))
        priority = LEVEL_PRIORITY.get(level, 1)
        wins = mask & (
            (priority > winning_priorities)
            | ((priority == winning_priorities) & (revision_seq >= winning_revisions))
        )
        assignments[wins] = group_index
        winning_priorities[wins] = priority
        winning_revisions[wins] = revision_seq
    overlap_count = int(np.sum(membership_count > 1))
    return assignments, group_masks, overlap_count


def _bbox(points: np.ndarray) -> Dict[str, List[float]]:
    if len(points) == 0:
        return {}
    return {
        "min": np.min(points, axis=0).astype(float).tolist(),
        "max": np.max(points, axis=0).astype(float).tolist(),
    }


def _estimated_voxel_count(points: np.ndarray, voxel_size: float = DEFAULT_VOXEL_SIZE) -> int:
    if len(points) == 0:
        return 0
    origin = np.min(points, axis=0)
    cells = np.floor((points - origin) / max(float(voxel_size), 1e-6)).astype(np.int64)
    return int(len(np.unique(cells, axis=0)))


def _group_stats(
    points: np.ndarray,
    labels: np.ndarray,
    groups: Sequence[Dict[str, Any]],
) -> Tuple[List[Dict[str, Any]], Dict[str, Any], np.ndarray]:
    assignments, group_masks, overlap_count = resolve_group_assignments(points, labels, groups)
    summaries: List[Dict[str, Any]] = []
    for index, group in enumerate(groups):
        final_mask = assignments == index
        selected = points[final_mask]
        tower_count = int(np.sum(labels[final_mask] == TOWER_LABEL))
        insulator_count = int(np.sum(labels[final_mask] == INSULATOR_LABEL))
        point_count = int(len(selected))
        estimated_voxels = _estimated_voxel_count(selected)
        summary = {
            "group_id": group.get("group_id"),
            "point_count": point_count,
            "raw_point_count": int(np.sum(group_masks[index])) if index < len(group_masks) else 0,
            "tower_count": tower_count,
            "insulator_count": insulator_count,
            "tower_ratio": round(tower_count / max(point_count, 1), 6),
            "insulator_ratio": round(insulator_count / max(point_count, 1), 6),
            "estimated_voxel_count": estimated_voxels,
            "estimated_target_cell_count": estimated_voxels,
            "bbox": _bbox(selected),
        }
        summaries.append(summary)

    selected_mask = assignments >= 0
    selected_count = int(np.sum(selected_mask))
    tower_selected = int(np.sum((labels == TOWER_LABEL) & selected_mask))
    insulator_selected = int(np.sum((labels == INSULATOR_LABEL) & selected_mask))
    stats = {
        "important_groups": int(sum(1 for group in groups if group.get("level") == "important")),
        "needed_groups": int(sum(1 for group in groups if group.get("level") == "needed")),
        "normal_groups": int(sum(1 for group in groups if group.get("level") == "normal")),
        "selected_point_count": selected_count,
        "tower_count": tower_selected,
        "insulator_count": insulator_selected,
        "tower_ratio": round(tower_selected / max(selected_count, 1), 6),
        "insulator_ratio": round(insulator_selected / max(selected_count, 1), 6),
        "overlap_count": overlap_count,
        "editable_point_count": int(len(points)),
        "bbox": _bbox(points[selected_mask]),
    }
    return summaries, stats, assignments


def normalize_profile(
    point_cloud_name: str,
    payload: Dict[str, Any],
    existing: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    now = _now()
    profile_id = str(payload.get("profile_id") or (existing or {}).get("profile_id") or _profile_id())
    groups: List[Dict[str, Any]] = []
    for index, raw_group in enumerate(payload.get("groups") or []):
        group = dict(raw_group or {})
        level = str(group.get("level") or "normal")
        if level not in LEVEL_PRIORITY:
            level = "normal"
        group_id = str(group.get("group_id") or uuid.uuid4().hex[:12])
        created_at = group.get("created_at") or now
        revision_seq = max(
            index + 1,
            int(group.get("revision_seq") or 0),
        )
        selection_geometry = group.get("selection_geometry")
        if not isinstance(selection_geometry, dict):
            selection_geometry = {"operations": []}
        operations = selection_geometry.get("operations")
        if not isinstance(operations, list):
            selection_geometry["operations"] = []
        groups.append(
            {
                **group,
                "group_id": group_id,
                "name": str(group.get("name") or f"{level}-{index + 1}"),
                "level": level,
                "color": str(group.get("color") or LEVEL_COLORS[level]),
                "enabled": bool(group.get("enabled", True)),
                "visible": bool(group.get("visible", True)),
                "selection_tool": str(group.get("selection_tool") or "box3d"),
                "selection_mode": str(group.get("selection_mode") or "new"),
                "selection_geometry": selection_geometry,
                "semantic_filter": list(group.get("semantic_filter") or ["tower", "insulator"]),
                "revision_seq": revision_seq,
                "weight_multiplier": LEVEL_MULTIPLIER[level],
                "required_view_directions": LEVEL_DIRECTIONS[level],
                "created_at": created_at,
                "updated_at": now,
                "note": str(group.get("note") or ""),
            }
        )
    history = list(payload.get("revision_history") or (existing or {}).get("revision_history") or [])
    return {
        **(existing or {}),
        **payload,
        "profile_id": profile_id,
        "base_point_cloud": safe_filename(point_cloud_name),
        "name": str(payload.get("name") or f"{_point_cloud_base(point_cloud_name)}_自定义权重"),
        "status": str(payload.get("status") or (existing or {}).get("status") or "draft"),
        "active": bool(payload.get("active", (existing or {}).get("active", False))),
        "created_at": payload.get("created_at") or (existing or {}).get("created_at") or now,
        "updated_at": now,
        "groups": groups,
        "stats": dict(payload.get("stats") or {}),
        "revision_history": history[-50:],
        "policy": {
            "level_priority": ["important", "needed", "normal"],
            "level_multiplier": LEVEL_MULTIPLIER,
            "required_view_directions": LEVEL_DIRECTIONS,
            "same_level_rule": "higher_revision_seq_wins",
        },
    }


def evaluate_profile(
    point_cloud_path: Path,
    profile: Dict[str, Any],
) -> Tuple[Dict[str, Any], np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    points, labels, indices = _load_las_targets(point_cloud_path)
    group_summaries, stats, assignments = _group_stats(points, labels, profile.get("groups") or [])
    for group, summary in zip(profile.get("groups") or [], group_summaries):
        group.update(summary)
    profile["stats"] = stats
    return profile, points, labels, indices, assignments


def preview_profile(username: str, point_cloud_name: str, payload: Dict[str, Any]) -> Dict[str, Any]:
    point_cloud = find_user_file(username, "point_cloud", point_cloud_name)
    if not point_cloud:
        raise FileNotFoundError("点云文件不存在")
    existing = get_profile_by_id(username, point_cloud_name, payload.get("profile_id"))
    profile = normalize_profile(point_cloud_name, payload, existing=existing)
    profile, _, _, _, _ = evaluate_profile(point_cloud, profile)
    return profile


def save_profile(
    username: str,
    point_cloud_name: str,
    payload: Dict[str, Any],
    *,
    apply: bool,
) -> Dict[str, Any]:
    point_cloud = find_user_file(username, "point_cloud", point_cloud_name)
    if not point_cloud:
        raise FileNotFoundError("点云文件不存在")
    existing = get_profile_by_id(username, point_cloud_name, payload.get("profile_id"))
    profile = normalize_profile(point_cloud_name, payload, existing=existing)
    profile["status"] = "applied" if apply else "draft"
    profile["active"] = bool(apply)
    profile, _, _, _, _ = evaluate_profile(point_cloud, profile)
    if apply and profile["stats"].get("selected_point_count", 0) <= 0:
        raise ValueError("当前 profile 未选中任何杆塔或绝缘子点")
    profile["revision_history"] = (
        list(profile.get("revision_history") or [])
        + [{"at": _now(), "action": "apply" if apply else "save_draft"}]
    )[-50:]
    if apply:
        for other in list_profiles(username, point_cloud_name):
            if other.get("profile_id") == profile["profile_id"]:
                continue
            if other.get("active"):
                other["active"] = False
                other["updated_at"] = _now()
                other["revision_history"] = (
                    list(other.get("revision_history") or [])
                    + [{"at": _now(), "action": "deactivated_by_other_profile"}]
                )[-50:]
                _atomic_write_json(
                    profile_path(username, point_cloud_name, str(other["profile_id"])),
                    {key: value for key, value in other.items() if key != "_path"},
                )
    _atomic_write_json(profile_path(username, point_cloud_name, profile["profile_id"]), profile)
    return profile


def write_weighted_visual_las(
    username: str,
    point_cloud_name: str,
    target_path: Path,
    profile_id: Optional[str] = None,
) -> Dict[str, Any]:
    """Create a non-authoritative LAS copy for Potree visualization of weight groups."""
    point_cloud = find_user_file(username, "point_cloud", point_cloud_name)
    if not point_cloud:
        raise FileNotFoundError("Point cloud file not found")
    profile = get_profile_by_id(username, point_cloud_name, profile_id) if profile_id else get_active_profile(username, point_cloud_name)
    if not profile:
        raise FileNotFoundError("Active weight profile not found")

    points, labels, indices = _load_las_targets(point_cloud)
    assignments, _, overlap_count = resolve_group_assignments(points, labels, profile.get("groups") or [])
    groups = profile.get("groups") or []

    with laspy.open(point_cloud) as source:
        las = source.read()

    classifications = np.asarray(las.classification, dtype=np.uint8).copy()
    assigned_count = 0
    for local_index, assignment in enumerate(assignments):
        if assignment < 0 or assignment >= len(groups):
            continue
        group = groups[int(assignment)]
        level = str(group.get("level") or "normal")
        classifications[int(indices[local_index])] = LEVEL_LOD_CLASSIFICATION.get(level, LEVEL_LOD_CLASSIFICATION["normal"])
        assigned_count += 1
    las.classification = classifications

    if hasattr(las, "red") and hasattr(las, "green") and hasattr(las, "blue"):
        all_labels = np.asarray(las.classification, dtype=np.int32)
        red = np.full(all_labels.shape, DEFAULT_CLASS_RGB16[0], dtype=np.uint16)
        green = np.full(all_labels.shape, DEFAULT_CLASS_RGB16[1], dtype=np.uint16)
        blue = np.full(all_labels.shape, DEFAULT_CLASS_RGB16[2], dtype=np.uint16)
        for label, (r_value, g_value, b_value) in CLASSIFICATION_RGB16.items():
            mask = all_labels == int(label)
            if not np.any(mask):
                continue
            red[mask] = r_value
            green[mask] = g_value
            blue[mask] = b_value
        for local_index, assignment in enumerate(assignments):
            if assignment < 0 or assignment >= len(groups):
                continue
            group = groups[int(assignment)]
            r, g, b = _hex_to_rgb16(group.get("color"), LEVEL_COLORS.get(str(group.get("level") or "normal"), "#3b82f6"))
            point_index = int(indices[local_index])
            red[point_index] = r
            green[point_index] = g
            blue[point_index] = b
        las.red = red
        las.green = green
        las.blue = blue

    target_path.parent.mkdir(parents=True, exist_ok=True)
    if target_path.exists():
        target_path.unlink()
    _strip_display_vlrs(las)
    las.write(target_path)
    return {
        "source_filename": point_cloud.name,
        "visual_source": str(target_path),
        "weight_profile_id": profile.get("profile_id"),
        "assigned_point_count": int(assigned_count),
        "overlap_count": int(overlap_count),
        "classification_codes": LEVEL_LOD_CLASSIFICATION,
    }


def _strip_display_vlrs(las: laspy.LasData) -> None:
    """Drop optional metadata records that can contain non-ASCII bytes when rewriting display copies."""
    try:
        las.vlrs.clear()
        las.header.vlrs.clear()
        if getattr(las, "evlrs", None):
            las.evlrs.clear()
    except Exception:
        pass


def restore_original(username: str, point_cloud_name: str) -> Dict[str, Any]:
    restored_ids: List[str] = []
    for profile in list_profiles(username, point_cloud_name):
        if not profile.get("active"):
            continue
        profile["active"] = False
        profile["updated_at"] = _now()
        profile["revision_history"] = (
            list(profile.get("revision_history") or [])
            + [{"at": _now(), "action": "restore_original"}]
        )[-50:]
        restored_ids.append(str(profile.get("profile_id")))
        _atomic_write_json(
            profile_path(username, point_cloud_name, str(profile["profile_id"])),
            {key: value for key, value in profile.items() if key != "_path"},
        )
    return {"restored_profile_ids": restored_ids, "weighted": False}


def expected_output_names(point_cloud_name: str, profile_id: Optional[str]) -> Dict[str, str]:
    base = _point_cloud_base(point_cloud_name)
    if profile_id:
        prefix = f"{base}_weighted_{profile_id}"
    else:
        prefix = base
    return {
        "voxel_filename": f"{prefix}_voxel.npz",
        "candidate_filename": f"{prefix}_candidates.json",
        "output_base": prefix,
    }


def _timestamp_seconds(value: Any) -> Optional[float]:
    if not value:
        return None
    try:
        text = str(value).replace("Z", "+00:00")
        return datetime.fromisoformat(text).timestamp()
    except Exception:
        return None


def _file_sync_state(path: Path, profile_updated_seconds: Optional[float]) -> Dict[str, Any]:
    exists = path.exists()
    modified_seconds = path.stat().st_mtime if exists else None
    stale = bool(
        profile_updated_seconds is not None
        and (
            not exists
            or modified_seconds is None
            or modified_seconds + 1e-6 < profile_updated_seconds
        )
    )
    return {
        "exists": exists,
        "stale": stale,
        "modified_at": datetime.fromtimestamp(modified_seconds).astimezone().isoformat(timespec="seconds")
        if modified_seconds is not None
        else None,
    }


def profile_status(username: str, point_cloud_name: str) -> Dict[str, Any]:
    point_cloud = find_user_file(username, "point_cloud", point_cloud_name)
    if not point_cloud:
        raise FileNotFoundError("点云文件不存在")
    profiles = list_profiles(username, point_cloud_name)
    active = next(
        (
            profile
            for profile in profiles
            if profile.get("status") == "applied" and bool(profile.get("active"))
        ),
        None,
    )
    latest_draft = next((profile for profile in profiles if profile.get("status") == "draft"), None)
    names = expected_output_names(
        point_cloud_name,
        str(active.get("profile_id")) if active else None,
    )
    voxel_dir = get_user_dir(username, "voxel")
    profile_updated_seconds = _timestamp_seconds(active.get("updated_at")) if active else None
    voxel_state = _file_sync_state(voxel_dir / names["voxel_filename"], profile_updated_seconds)
    candidate_state = _file_sync_state(voxel_dir / names["candidate_filename"], profile_updated_seconds)
    model_sync = {
        "weight_applied": active is not None,
        "profile_updated_at": active.get("updated_at") if active else None,
        "voxel": voxel_state,
        "candidate": candidate_state,
        "voxel_exists": voxel_state["exists"],
        "candidate_exists": candidate_state["exists"],
        "voxel_stale": voxel_state["stale"],
        "candidate_stale": candidate_state["stale"],
        "is_stale": bool(active and (voxel_state["stale"] or candidate_state["stale"])),
        "ready_for_waypoints": bool(
            voxel_state["exists"]
            and candidate_state["exists"]
            and not voxel_state["stale"]
            and not candidate_state["stale"]
        ),
    }
    return {
        "weighted": active is not None,
        "active_profile_id": active.get("profile_id") if active else None,
        "active_profile": active,
        "latest_draft": latest_draft,
        "profile_count": len(profiles),
        "model_sync": model_sync,
        **names,
    }


def editable_points(
    username: str,
    point_cloud_name: str,
    limit: int = 120_000,
) -> Dict[str, Any]:
    point_cloud = find_user_file(username, "point_cloud", point_cloud_name)
    if not point_cloud:
        raise FileNotFoundError("点云文件不存在")
    points, labels, indices = _load_las_targets(point_cloud)
    total = len(points)
    total_tower_count = int(np.sum(labels == TOWER_LABEL))
    total_insulator_count = int(np.sum(labels == INSULATOR_LABEL))
    limit = max(1_000, min(int(limit), 200_000))
    context_limit = max(1_000, min(limit // 4, 50_000))
    context_points, context_labels, context_indices, context_totals = _load_las_context_points(
        point_cloud,
        context_limit,
    )
    if total > limit:
        selected = _editable_sample_indices(points, labels, limit)
        points = points[selected]
        labels = labels[selected]
        indices = indices[selected]

    active = get_active_profile(username, point_cloud_name)
    assignments = np.full(len(points), -1, dtype=int)
    if active:
        assignments, _, _ = resolve_group_assignments(points, labels, active.get("groups") or [])
    group_ids: List[Optional[str]] = []
    levels: List[Optional[str]] = []
    colors: List[Optional[str]] = []
    groups = active.get("groups") or [] if active else []
    for assignment in assignments:
        group = groups[int(assignment)] if assignment >= 0 and assignment < len(groups) else None
        group_ids.append(group.get("group_id") if group else None)
        levels.append(group.get("level") if group else None)
        colors.append(group.get("color") if group else None)

    return {
        "points": points.astype(float).tolist(),
        "labels": labels.astype(int).tolist(),
        "point_indices": indices.astype(int).tolist(),
        "context_points": context_points.astype(float).tolist(),
        "context_labels": context_labels.astype(int).tolist(),
        "context_point_indices": context_indices.astype(int).tolist(),
        "group_ids": group_ids,
        "levels": levels,
        "colors": colors,
        "center": np.mean(points, axis=0).astype(float).tolist(),
        "bbox": _bbox(points),
        "sampled_point_count": int(len(points)),
        "editable_point_count": int(total),
        "tower_count": total_tower_count,
        "insulator_count": total_insulator_count,
        "sampled_tower_count": int(np.sum(labels == TOWER_LABEL)),
        "sampled_insulator_count": int(np.sum(labels == INSULATOR_LABEL)),
        "sampled_context_point_count": int(len(context_points)),
        "sampled_conductor_count": int(np.sum(context_labels == CONDUCTOR_LABEL)),
        "sampled_ground_wire_count": int(np.sum(context_labels == GROUND_WIRE_LABEL)),
        **context_totals,
        "active_profile": active,
    }


def annotate_target_points(
    points: np.ndarray,
    labels: np.ndarray,
    profile: Optional[Dict[str, Any]],
) -> Tuple[np.ndarray, List[Optional[Dict[str, Any]]], int]:
    """Resolve profile winners for target points used by the modeling pipeline."""
    if not profile:
        return np.ones(len(points), dtype=bool), [None] * len(points), 0
    assignments, _, overlap_count = resolve_group_assignments(points, labels, profile.get("groups") or [])
    groups = profile.get("groups") or []
    annotations: List[Optional[Dict[str, Any]]] = []
    for assignment in assignments:
        if assignment < 0 or assignment >= len(groups):
            annotations.append(None)
            continue
        group = groups[int(assignment)]
        level = str(group.get("level") or "normal")
        annotations.append(
            {
                "weight_level": level,
                "weight_group_id": group.get("group_id"),
                "weight_group_name": group.get("name"),
                "group_color": group.get("color") or LEVEL_COLORS[level],
                "weight_multiplier": LEVEL_MULTIPLIER[level],
                "required_view_directions": LEVEL_DIRECTIONS[level],
                "revision_seq": int(group.get("revision_seq") or 0),
            }
        )
    return assignments >= 0, annotations, overlap_count
