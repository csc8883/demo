from __future__ import annotations

import heapq
import json
import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
import utm

from waypoint_planning.planning_core import (
    ConductorNoFlyVolume,
    build_conductor_no_fly_volumes_from_point_cloud,
    load_conductor_no_fly_volumes,
)

WIRE_LABELS = {0, 3}
CONDUCTOR_LABEL = 0
GROUND_WIRE_LABEL = 3
TOWER_LABELS = {16, 22}


def _read_json(path: Path) -> Dict[str, Any]:
    raw = Path(path).read_bytes()
    last_error: Optional[Exception] = None
    for encoding in ("utf-8-sig", "utf-8", "gb18030", "gbk"):
        try:
            return json.loads(raw.decode(encoding))
        except Exception as exc:
            last_error = exc
    raise ValueError(f"无法解析 JSON 文件: {last_error}")


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        if value is None:
            return default
        result = float(value)
        return result if math.isfinite(result) else default
    except Exception:
        return default


def _optional_float(value: Any) -> Optional[float]:
    try:
        if value is None:
            return None
        result = float(value)
        return result if math.isfinite(result) else None
    except Exception:
        return None


def _xyz(value: Any) -> Optional[np.ndarray]:
    if value is None:
        return None
    if isinstance(value, str):
        parts = value.replace(";", ",").split(",")
    else:
        parts = value
    try:
        arr = np.asarray([float(parts[0]), float(parts[1]), float(parts[2])], dtype=float)
    except Exception:
        return None
    return arr if np.all(np.isfinite(arr)) else None


def _distance(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.linalg.norm(a - b))


def _utm_to_latlon(point: Sequence[float], zone: int = 50) -> Tuple[float, float]:
    lat, lon = utm.to_latlon(float(point[0]), float(point[1]), int(zone), northern=True)
    return float(lat), float(lon)


def _bearing_deg(src: np.ndarray, dst: np.ndarray) -> float:
    dx = float(dst[0] - src[0])
    dy = float(dst[1] - src[1])
    return round(math.degrees(math.atan2(dx, dy)), 3)


def _pitch_deg(src: np.ndarray, dst: np.ndarray) -> float:
    horizontal = math.hypot(float(dst[0] - src[0]), float(dst[1] - src[1]))
    if horizontal <= 1e-6:
        return -90.0 if dst[2] < src[2] else 90.0
    return round(math.degrees(math.atan2(float(dst[2] - src[2]), horizontal)), 3)


def _segment_length(points: Sequence[np.ndarray]) -> float:
    if len(points) < 2:
        return 0.0
    return float(sum(_distance(points[i - 1], points[i]) for i in range(1, len(points))))


def _densify_path(points: Sequence[np.ndarray], max_segment_m: float) -> List[np.ndarray]:
    if len(points) < 2:
        return list(points)
    densified: List[np.ndarray] = [points[0]]
    for index in range(1, len(points)):
        start = densified[-1]
        end = points[index]
        distance = _distance(start, end)
        steps = max(1, int(math.ceil(distance / max(max_segment_m, 1.0))))
        for step in range(1, steps + 1):
            t = step / steps
            densified.append(start * (1 - t) + end * t)
    return densified


def _focus_from_waypoint(waypoint: Dict[str, Any]) -> str:
    shots = waypoint.get("shots") or []
    if shots and isinstance(shots[0], dict):
        return str(shots[0].get("semantic_focus") or "")
    return str(waypoint.get("semantic_focus") or "")


def _aim_type_for_focus(focus: str) -> str:
    if "insulator" in focus:
        return "绝缘子"
    if "connection" in focus:
        return "挂点"
    if focus == "tower_top":
        return "塔头"
    if focus == "tower_edge":
        return "塔身边缘"
    if focus == "tower_body":
        return "塔身"
    return "塔身"


def _manual_target_candidates(route_context: Dict[str, Any]) -> List[Dict[str, Any]]:
    candidates: List[Dict[str, Any]] = []
    for tower in route_context.get("towers") or []:
        for point in tower.get("points") or []:
            target = _manual_point_target_utm(point)
            if target is not None:
                candidates.append({"aim_type": str(point.get("AimType") or ""), "target": target})
    return candidates


def _manual_point_target_utm(point: Dict[str, Any]) -> Optional[np.ndarray]:
    explicit = _xyz(point.get("target_utm") or point.get("target_center"))
    if explicit is not None:
        return explicit
    for lat_key, lon_key, alt_key in (
        ("latitude_Aim", "longitude_Aim", "altitude_Aim"),
        ("latitude", "longitude", "altitude"),
    ):
        if point.get(lat_key) is None or point.get(lon_key) is None:
            continue
        try:
            easting, northing, _, _ = utm.from_latlon(float(point[lat_key]), float(point[lon_key]))
            return np.asarray([easting, northing, _safe_float(point.get(alt_key))], dtype=float)
        except Exception:
            continue
    return None


def _is_ground_wire_aim(aim_type: Any) -> bool:
    text = str(aim_type or "")
    lower = text.lower()
    return ("地线" in text and "挂点" in text) or ("ground" in lower and "wire" in lower)


def _is_conductor_aim(aim_type: Any) -> bool:
    text = str(aim_type or "")
    lower = text.lower()
    return ("导线" in text and "挂点" in text) or ("conductor" in lower and ("point" in lower or "hang" in lower))


def _target_for_waypoint(
    waypoint: Dict[str, Any],
    tower_center: np.ndarray,
    manual_targets: Sequence[Dict[str, Any]],
) -> np.ndarray:
    explicit = _xyz(waypoint.get("target_utm") or waypoint.get("target_center"))
    if explicit is not None:
        return explicit

    focus = _focus_from_waypoint(waypoint)
    pos = _xyz(waypoint.get("pos_utm") or waypoint.get("position"))
    if pos is not None and manual_targets:
        keywords: List[str] = []
        if "insulator" in focus:
            keywords = ["绝缘子"]
        elif "connection" in focus:
            keywords = ["挂点"]
        elif focus == "tower_top":
            keywords = ["塔头", "地线"]
        elif focus == "tower_edge":
            keywords = ["塔身", "横担"]
        matches = [item for item in manual_targets if any(k in item["aim_type"] for k in keywords)]
        pool = matches or list(manual_targets)
        return min(pool, key=lambda item: _distance(pos, item["target"]))["target"].copy()

    target = tower_center.copy()
    if focus == "tower_top":
        target[2] += 12.0
    elif "insulator" in focus or "connection" in focus:
        target[2] += 4.0
    elif focus == "tower_body":
        target[2] -= 12.0
    return target


def _sample_points(points: np.ndarray, max_points: int = 20000) -> np.ndarray:
    if len(points) > max_points:
        step = max(1, int(math.ceil(len(points) / max_points)))
        return points[::step]
    return points


def _empty_safety() -> Dict[str, Any]:
    empty = np.empty((0, 3), dtype=float)
    return {"all": empty, "wire": empty, "tower": empty, "no_fly": [], "no_fly_source": None}


def _as_point_array(points: Sequence[Any]) -> np.ndarray:
    if points is None:
        return np.empty((0, 3), dtype=float)
    if hasattr(points, "__len__") and len(points) == 0:
        return np.empty((0, 3), dtype=float)
    arr = np.asarray(points, dtype=float).reshape((-1, 3))
    if len(arr) == 0:
        return arr
    return arr[np.all(np.isfinite(arr), axis=1)]


def _load_conductor_no_fly_from_voxel(
    data: Any,
    conductor_points: Sequence[Any],
    ground_wire_points: Sequence[Any],
) -> Tuple[List[ConductorNoFlyVolume], Optional[str]]:
    volumes: List[ConductorNoFlyVolume] = []
    conductor = _as_point_array(conductor_points)
    ground_wire = _as_point_array(ground_wire_points)
    if len(conductor) >= 2 and len(ground_wire) >= 2:
        try:
            if "local_center" in data.files:
                center = np.asarray(data["local_center"], dtype=float)[:3]
            else:
                center = np.mean(np.vstack([conductor, ground_wire]), axis=0)
            records = build_conductor_no_fly_volumes_from_point_cloud(
                center=center,
                wire_points=conductor,
                ground_wire_points=ground_wire,
                source="point_cloud",
            )
            volumes = load_conductor_no_fly_volumes(records)
        except Exception:
            volumes = []
    if not volumes and "conductor_no_fly_volumes" in data.files:
        try:
            volumes = load_conductor_no_fly_volumes(data["conductor_no_fly_volumes"])
        except Exception:
            volumes = []
    source = volumes[0].source if volumes else None
    return volumes, source


def _with_no_fly(
    safety: Dict[str, Any],
    data: Any,
    conductor_points: Sequence[Any],
    ground_wire_points: Sequence[Any],
) -> Dict[str, Any]:
    volumes, source = _load_conductor_no_fly_from_voxel(data, conductor_points, ground_wire_points)
    safety["no_fly"] = volumes
    safety["no_fly_source"] = source
    return safety


def _load_safety_points(voxel_path: Optional[Path]) -> Dict[str, Any]:
    empty = np.empty((0, 3), dtype=float)
    if not voxel_path or not Path(voxel_path).exists():
        return _empty_safety()
    data = np.load(voxel_path, allow_pickle=True)
    if "display_voxels" in data.files:
        all_points: List[Any] = []
        wire_points: List[Any] = []
        conductor_points: List[Any] = []
        ground_wire_points: List[Any] = []
        tower_points: List[Any] = []
        for item in data["display_voxels"]:
            if not isinstance(item, dict) or "coord" not in item:
                continue
            all_points.append(item["coord"])
            category = str(item.get("category") or "").lower()
            semantic = str(item.get("semantic") or "").lower()
            label = int(item.get("label", -1))
            if category == "ground_wire" or semantic == "ground_wire" or label == GROUND_WIRE_LABEL:
                wire_points.append(item["coord"])
                ground_wire_points.append(item["coord"])
            elif category == "wire" or semantic == "wire" or label == CONDUCTOR_LABEL:
                wire_points.append(item["coord"])
                conductor_points.append(item["coord"])
            elif category in {"tower", "insulator"} or label in TOWER_LABELS:
                tower_points.append(item["coord"])
        if "safety_points" in data.files and "safety_labels" in data.files:
            points = np.asarray(data["safety_points"], dtype=float)
            labels = np.asarray(data["safety_labels"], dtype=int)
            if len(points) == len(labels):
                safety_wire = points[np.isin(labels, list(WIRE_LABELS)), :3]
                safety_conductor = points[labels == CONDUCTOR_LABEL, :3]
                safety_ground_wire = points[labels == GROUND_WIRE_LABEL, :3]
                safety_tower = points[np.isin(labels, list(TOWER_LABELS)), :3]
                if len(safety_wire):
                    wire_points.extend(safety_wire.tolist())
                    all_points.extend(safety_wire.tolist())
                if len(safety_conductor):
                    conductor_points.extend(safety_conductor.tolist())
                if len(safety_ground_wire):
                    ground_wire_points.extend(safety_ground_wire.tolist())
                if len(safety_tower):
                    tower_points.extend(safety_tower.tolist())
                    all_points.extend(safety_tower.tolist())
        if wire_points or tower_points:
            return _with_no_fly({
                "all": _sample_points(np.asarray(all_points, dtype=float)) if all_points else empty,
                "wire": _sample_points(np.asarray(wire_points, dtype=float)) if wire_points else empty,
                "tower": _sample_points(np.asarray(tower_points, dtype=float)) if tower_points else empty,
            }, data, conductor_points, ground_wire_points)
    if "safety_points" in data.files:
        points = np.asarray(data["safety_points"], dtype=float)
        labels = np.asarray(data["safety_labels"], dtype=int) if "safety_labels" in data.files else np.zeros(len(points), dtype=int)
        return _with_no_fly({
            "all": _sample_points(points[:, :3]),
            "wire": _sample_points(points[np.isin(labels, list(WIRE_LABELS)), :3]),
            "tower": _sample_points(points[np.isin(labels, list(TOWER_LABELS)), :3]),
        }, data, points[labels == CONDUCTOR_LABEL, :3], points[labels == GROUND_WIRE_LABEL, :3])
    key = "display_voxels" if "display_voxels" in data.files else "voxels"
    all_points: List[Any] = []
    wire_points: List[Any] = []
    conductor_points: List[Any] = []
    ground_wire_points: List[Any] = []
    tower_points: List[Any] = []
    for item in data[key]:
        if isinstance(item, dict) and "coord" in item:
            all_points.append(item["coord"])
            category = str(item.get("category") or "").lower()
            semantic = str(item.get("semantic") or "").lower()
            label = int(item.get("label", -1))
            if category == "ground_wire" or semantic == "ground_wire" or label == GROUND_WIRE_LABEL:
                wire_points.append(item["coord"])
                ground_wire_points.append(item["coord"])
            elif category == "wire" or semantic == "wire" or label == CONDUCTOR_LABEL:
                wire_points.append(item["coord"])
                conductor_points.append(item["coord"])
            elif category in {"tower", "insulator"} or label in TOWER_LABELS:
                tower_points.append(item["coord"])
    return _with_no_fly({
        "all": _sample_points(np.asarray(all_points, dtype=float)) if all_points else empty,
        "wire": _sample_points(np.asarray(wire_points, dtype=float)) if wire_points else empty,
        "tower": _sample_points(np.asarray(tower_points, dtype=float)) if tower_points else empty,
    }, data, conductor_points, ground_wire_points)


def _load_voxel_center(voxel_path: Optional[Path]) -> Optional[np.ndarray]:
    if not voxel_path or not Path(voxel_path).exists():
        return None
    try:
        data = np.load(voxel_path, allow_pickle=True)
        if "local_center" not in data.files:
            return None
        center = np.asarray(data["local_center"], dtype=float)[:3]
        return center if len(center) == 3 and np.all(np.isfinite(center)) else None
    except Exception:
        return None


def _route_context_no_fly_volumes(route_context: Dict[str, Any], tower_center: np.ndarray) -> List[ConductorNoFlyVolume]:
    towers = route_context.get("towers") or []
    first_tower = towers[0] if towers else {}
    center = _xyz(first_tower.get("PlaneCenterPoint")) if isinstance(first_tower, dict) else None
    center = center if center is not None else tower_center
    plane_len = _optional_float(first_tower.get("PlaneLen")) if isinstance(first_tower, dict) else None
    plane_angle = _optional_float(first_tower.get("PlaneAngle")) if isinstance(first_tower, dict) else None
    manual_points: List[Dict[str, Any]] = []
    for tower in towers:
        if isinstance(tower, dict):
            manual_points.extend(point for point in (tower.get("points") or []) if isinstance(point, dict))
    manual_points.extend(point for point in (route_context.get("points") or []) if isinstance(point, dict))

    conductor_points: List[np.ndarray] = []
    conductor_records: List[Tuple[str, np.ndarray]] = []
    ground_wire_points: List[np.ndarray] = []
    for point in manual_points:
        aim_type = point.get("AimType") or point.get("aim_type") or ""
        target = _manual_point_target_utm(point)
        if target is None:
            continue
        if _is_ground_wire_aim(aim_type):
            ground_wire_points.append(target)
        elif _is_conductor_aim(aim_type):
            conductor_points.append(target)
            conductor_records.append((str(aim_type), target))
    if len(conductor_points) < 2 or len(ground_wire_points) < 2:
        return []

    line_axes: List[Optional[np.ndarray]] = []
    grouped_conductors: Dict[str, List[np.ndarray]] = {}
    for aim_type, target in conductor_records:
        grouped_conductors.setdefault(aim_type.rstrip("0123456789"), []).append(target)
    for points in grouped_conductors.values():
        if len(points) < 2:
            continue
        best_pair: Optional[Tuple[np.ndarray, np.ndarray]] = None
        best_distance = 0.0
        for first_index, first in enumerate(points):
            for second in points[first_index + 1 :]:
                distance = float(np.linalg.norm((second - first)[:2]))
                if distance > best_distance:
                    best_distance = distance
                    best_pair = (first, second)
        if best_pair and best_distance > 0.5:
            line_axes.append(best_pair[1] - best_pair[0])
    if plane_angle is not None:
        for offset in (math.pi / 2.0, -math.pi / 2.0, 0.0):
            angle = plane_angle + offset
            line_axes.append(np.asarray([math.cos(angle), math.sin(angle), 0.0], dtype=float))
    line_axes.append(None)

    conductor_array = np.asarray(conductor_points, dtype=float)
    ground_wire_array = np.asarray(ground_wire_points, dtype=float)
    best_volumes: List[ConductorNoFlyVolume] = []
    best_width = -float("inf")
    for line_axis in line_axes:
        try:
            records = build_conductor_no_fly_volumes_from_point_cloud(
                center=center,
                wire_points=conductor_array,
                ground_wire_points=ground_wire_array,
                source="manual_route",
                plane_len=plane_len,
                line_axis=line_axis,
            )
            volumes = load_conductor_no_fly_volumes(records)
        except Exception:
            volumes = []
        if not volumes:
            continue
        width = max(float(volume.v_max - volume.v_min) for volume in volumes)
        if width > best_width:
            best_width = width
            best_volumes = volumes
    return best_volumes


def _apply_route_context_no_fly(
    safety: Dict[str, Any],
    route_context: Dict[str, Any],
    tower_center: np.ndarray,
) -> bool:
    volumes = _route_context_no_fly_volumes(route_context, tower_center)
    if not volumes:
        return False
    safety["no_fly"] = volumes
    safety["no_fly_source"] = volumes[0].source or "manual_route"
    return True


def _apply_embedded_no_fly(safety: Dict[str, Any], route_payload: Dict[str, Any]) -> bool:
    route_meta = route_payload.get("route_planning") or {}
    raw_records = route_meta.get("conductor_no_fly_volumes") or []
    if not raw_records:
        return False
    try:
        volumes = load_conductor_no_fly_volumes(raw_records)
    except Exception:
        volumes = []
    if not volumes:
        return False
    safety["no_fly"] = volumes
    safety["no_fly_source"] = volumes[0].source or route_meta.get("conductor_no_fly_source") or "embedded_route"
    return True


def _line_near_obstacles(a: np.ndarray, b: np.ndarray, obstacles: np.ndarray, clearance: float) -> bool:
    if obstacles.size == 0:
        return False
    lower = np.minimum(a, b) - clearance
    upper = np.maximum(a, b) + clearance
    mask = np.all((obstacles >= lower) & (obstacles <= upper), axis=1)
    pts = obstacles[mask]
    if len(pts) == 0:
        return False
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom <= 1e-9:
        return bool(np.any(np.linalg.norm(pts - a, axis=1) < clearance))
    t = np.clip(((pts - a) @ ab) / denom, 0.0, 1.0)
    closest = a + t[:, None] * ab
    return bool(np.any(np.linalg.norm(pts - closest, axis=1) < clearance))


def _nearest_distance(point: np.ndarray, cloud: np.ndarray) -> Tuple[float, Optional[np.ndarray]]:
    if cloud.size == 0:
        return float("inf"), None
    deltas = cloud - point
    distances = np.linalg.norm(deltas, axis=1)
    index = int(np.argmin(distances))
    return float(distances[index]), cloud[index].copy()


def _adjust_for_clearance(
    point: np.ndarray,
    target: np.ndarray,
    safety: Dict[str, np.ndarray],
    wire_clearance_m: float,
    tower_clearance_m: float,
) -> np.ndarray:
    adjusted = point.copy()
    for _ in range(6):
        changed = False
        for key, clearance in (("wire", wire_clearance_m), ("tower", tower_clearance_m)):
            distance, nearest = _nearest_distance(adjusted, safety.get(key, np.empty((0, 3), dtype=float)))
            if nearest is None:
                continue
            direction = adjusted - nearest
            direction[2] = 0.0
            norm = float(np.linalg.norm(direction))
            effective_distance = min(distance, norm)
            if effective_distance >= clearance or effective_distance <= 1e-6:
                continue
            if norm <= 1e-6:
                direction = adjusted - target
                direction[2] = 0.0
                norm = float(np.linalg.norm(direction))
            if norm <= 1e-6:
                direction = np.asarray([1.0, 0.0, 0.0], dtype=float)
                norm = 1.0
            step = min(clearance - effective_distance + 0.8, max(1.5, clearance * 0.35))
            adjusted[:2] += (direction[:2] / norm) * step
            adjusted[2] = max(float(adjusted[2]), float(nearest[2]) + 2.0)
            changed = True
        if not changed:
            break
    return adjusted


def _no_fly_volumes(safety: Dict[str, Any]) -> List[ConductorNoFlyVolume]:
    return list(safety.get("no_fly") or [])


def _set_no_fly_margin(safety: Dict[str, Any], margin_m: float) -> None:
    safety["no_fly_margin_m"] = max(0.0, float(margin_m))


def _inside_no_fly(point: np.ndarray, safety: Dict[str, Any]) -> bool:
    return any(volume.contains(point) for volume in _no_fly_volumes(safety))


def _no_fly_clearance(point: np.ndarray, safety: Dict[str, Any]) -> float:
    volumes = _no_fly_volumes(safety)
    if not volumes:
        return float("inf")
    return float(min(volume.clearance(point) for volume in volumes))


def _adjust_for_no_fly(
    point: np.ndarray,
    target: np.ndarray,
    safety: Dict[str, Any],
    margin_m: float = 0.8,
) -> np.ndarray:
    adjusted = point.copy()
    margin_m = max(float(margin_m), _no_fly_segment_margin(safety))
    for _ in range(4):
        changed = False
        for volume in _no_fly_volumes(safety):
            if not volume.contains(adjusted) and volume.clearance(adjusted) >= margin_m:
                continue
            u_value, v_value, z_value = volume.local_coordinates(adjusted)
            left_v, right_v = volume.side_bounds_at_z(z_value)
            clamped_v = min(max(v_value, left_v), right_v)
            push_margin_m = margin_m + max(0.25, float(volume.tolerance_m))
            candidates = [
                volume.world_position(u_value, left_v - push_margin_m, z_value),
                volume.world_position(u_value, right_v + push_margin_m, z_value),
                volume.world_position(u_value, clamped_v, volume.upper_z() + push_margin_m),
            ]
            legal_candidates = [
                candidate
                for candidate in candidates
                if not _inside_no_fly(candidate, safety) and _no_fly_clearance(candidate, safety) >= margin_m
            ]
            if not legal_candidates:
                continue
            adjusted = min(
                legal_candidates,
                key=lambda candidate: (
                    _distance(candidate, point),
                    _distance(candidate, target),
                ),
            )
            changed = True
        if not changed:
            break
    return adjusted


def _point_satisfies_route_constraints(
    point: np.ndarray,
    safety: Dict[str, Any],
    wire_clearance_m: float,
    tower_clearance_m: float,
    no_fly_margin_m: float = 0.0,
) -> bool:
    no_fly_margin_m = max(float(no_fly_margin_m), _no_fly_segment_margin(safety))
    if _inside_no_fly(point, safety):
        return False
    if _no_fly_clearance(point, safety) < float(no_fly_margin_m):
        return False
    wire_distance, _ = _nearest_distance(point, safety.get("wire", np.empty((0, 3), dtype=float)))
    tower_distance, _ = _nearest_distance(point, safety.get("tower", np.empty((0, 3), dtype=float)))
    return wire_distance >= float(wire_clearance_m) and tower_distance >= float(tower_clearance_m)


def _unique_horizontal_directions(vectors: Sequence[np.ndarray]) -> List[np.ndarray]:
    directions: List[np.ndarray] = []
    for raw in vectors:
        direction = np.asarray(raw, dtype=float).copy()
        direction[2] = 0.0
        norm = float(np.linalg.norm(direction[:2]))
        if norm <= 1e-6:
            continue
        unit = direction / norm
        if not any(float(np.linalg.norm((unit - existing)[:2])) < 0.12 for existing in directions):
            directions.append(unit)
    return directions


def _search_clear_route_point(
    original: np.ndarray,
    adjusted: np.ndarray,
    target: np.ndarray,
    safety: Dict[str, Any],
    wire_clearance_m: float,
    tower_clearance_m: float,
) -> np.ndarray:
    no_fly_margin_m = max(
        _no_fly_segment_margin(safety),
        min(3.0, max(0.8, max(float(wire_clearance_m), float(tower_clearance_m)) * 0.45)),
    )
    if _point_satisfies_route_constraints(adjusted, safety, wire_clearance_m, tower_clearance_m, no_fly_margin_m):
        return adjusted

    wire_distance, nearest_wire = _nearest_distance(adjusted, safety.get("wire", np.empty((0, 3), dtype=float)))
    tower_distance, nearest_tower = _nearest_distance(adjusted, safety.get("tower", np.empty((0, 3), dtype=float)))
    base_vectors: List[np.ndarray] = [
        original - target,
        adjusted - target,
        original - adjusted,
    ]
    if nearest_wire is not None:
        base_vectors.extend([adjusted - nearest_wire, original - nearest_wire])
    if nearest_tower is not None:
        base_vectors.extend([adjusted - nearest_tower, original - nearest_tower])
    for volume in _no_fly_volumes(safety):
        _, v_value, z_value = volume.local_coordinates(adjusted)
        if volume.v_min - 3.0 <= v_value <= volume.v_max + 3.0 and z_value <= volume.upper_z() + 3.0:
            base_vectors.extend([volume.v_axis, -volume.v_axis])
    for angle in np.linspace(0.0, 2.0 * math.pi, 16, endpoint=False):
        base_vectors.append(np.asarray([math.cos(angle), math.sin(angle), 0.0], dtype=float))
    directions = _unique_horizontal_directions(base_vectors)
    if not directions:
        directions = [np.asarray([1.0, 0.0, 0.0], dtype=float)]

    base_radius = float(np.linalg.norm((original - target)[:2]))
    adjusted_radius = float(np.linalg.norm((adjusted - target)[:2]))
    min_radius = max(float(wire_clearance_m), float(tower_clearance_m), 2.0) + 1.0
    radii = sorted(
        {
            max(base_radius, min_radius),
            max(adjusted_radius, min_radius),
            min_radius + 3.0,
            min_radius + 7.0,
            min_radius + 12.0,
            max(base_radius, adjusted_radius, min_radius) + 6.0,
            max(base_radius, adjusted_radius, min_radius) + 12.0,
        }
    )
    local_top = max(
        _local_cloud_top_around(adjusted, safety.get("wire", np.empty((0, 3), dtype=float)), 35.0),
        _local_cloud_top_around(adjusted, safety.get("tower", np.empty((0, 3), dtype=float)), 35.0),
    )
    clearance = max(float(wire_clearance_m), float(tower_clearance_m), 2.0)
    z_values = sorted(
        {
            float(original[2]),
            float(adjusted[2]),
            float(target[2]) + 2.0,
            float(target[2]) + clearance + 2.0,
            float(adjusted[2]) + 3.0,
            float(adjusted[2]) + 7.0,
            local_top + clearance + 2.0 if math.isfinite(local_top) else float(adjusted[2]),
            _max_no_fly_top(safety) + clearance + 2.0,
        }
    )

    candidates: List[np.ndarray] = [adjusted.copy(), original.copy()]
    for direction in directions:
        for radius in radii:
            for z_value in z_values:
                candidate = target.copy()
                candidate[:2] = target[:2] + direction[:2] * radius
                candidate[2] = z_value
                candidates.append(candidate)
        for offset in (clearance + 1.0, clearance + 4.0, clearance + 8.0, clearance + 12.0):
            candidate = adjusted.copy()
            candidate[:2] = adjusted[:2] + direction[:2] * offset
            candidates.append(candidate)

    valid_candidates = [
        candidate
        for candidate in candidates
        if np.all(np.isfinite(candidate)) and _point_satisfies_route_constraints(candidate, safety, wire_clearance_m, tower_clearance_m, no_fly_margin_m)
    ]
    if not valid_candidates:
        return adjusted
    return min(
        valid_candidates,
        key=lambda candidate: (
            _distance(candidate, original),
            max(0.0, _distance(candidate, target) - max(base_radius, min_radius)),
            abs(float(candidate[2] - original[2])),
        ),
    )


def _adjust_for_route_constraints(
    point: np.ndarray,
    target: np.ndarray,
    safety: Dict[str, Any],
    wire_clearance_m: float,
    tower_clearance_m: float,
) -> np.ndarray:
    original = point.copy()
    adjusted = point.copy()
    no_fly_margin_m = min(3.0, max(0.8, max(float(wire_clearance_m), float(tower_clearance_m)) * 0.45))
    no_fly_margin_m = max(no_fly_margin_m, _no_fly_segment_margin(safety))
    for _ in range(5):
        before = adjusted.copy()
        adjusted = _adjust_for_clearance(adjusted, target, safety, wire_clearance_m, tower_clearance_m)
        adjusted = _adjust_for_no_fly(adjusted, target, safety, margin_m=no_fly_margin_m)
        if (
            _distance(before, adjusted) < 1e-6
            and _point_satisfies_route_constraints(adjusted, safety, wire_clearance_m, tower_clearance_m, no_fly_margin_m)
        ):
            break
    if not _point_satisfies_route_constraints(adjusted, safety, wire_clearance_m, tower_clearance_m, no_fly_margin_m):
        adjusted = _search_clear_route_point(original, adjusted, target, safety, wire_clearance_m, tower_clearance_m)
    return adjusted


def _local_cloud_top_around(center: np.ndarray, cloud: np.ndarray, radius_m: float) -> float:
    if cloud.size == 0:
        return -float("inf")
    deltas = cloud[:, :2] - center[:2]
    mask = np.linalg.norm(deltas, axis=1) <= float(radius_m)
    if not np.any(mask):
        return -float("inf")
    return float(np.max(cloud[mask, 2]))


def _max_no_fly_top(safety: Dict[str, Any]) -> float:
    top = -float("inf")
    for volume in _no_fly_volumes(safety):
        top = max(top, float(volume.upper_z()) + float(volume.tolerance_m))
    return top


def _normalize_horizontal(vector: Sequence[float]) -> Optional[np.ndarray]:
    arr = np.asarray(vector, dtype=float).copy()
    if len(arr) < 3:
        return None
    arr[2] = 0.0
    norm = float(np.linalg.norm(arr[:2]))
    if norm <= 1e-6:
        return None
    return arr / norm


def _route_line_axis(
    route_context: Dict[str, Any],
    safety: Dict[str, Any],
    tower_center: np.ndarray,
    task_positions: Sequence[np.ndarray],
) -> np.ndarray:
    """Infer the transmission-line axis used to place entry/exit overhead points."""
    volumes = _no_fly_volumes(safety)
    if volumes:
        axis = _normalize_horizontal(volumes[0].u_axis)
        if axis is not None:
            return axis

    towers = route_context.get("towers") or []
    first_tower = towers[0] if towers else {}
    plane_angle = _optional_float(first_tower.get("PlaneAngle")) if isinstance(first_tower, dict) else None
    if plane_angle is not None:
        axis = _normalize_horizontal([math.cos(plane_angle), math.sin(plane_angle), 0.0])
        if axis is not None:
            return axis

    points = np.asarray([pos for pos in task_positions if pos is not None], dtype=float)
    if len(points) >= 2:
        xy = points[:, :2] - np.mean(points[:, :2], axis=0)
        if float(np.max(np.linalg.norm(xy, axis=1))) > 1e-6:
            _, _, vh = np.linalg.svd(xy, full_matrices=False)
            axis = _normalize_horizontal([float(vh[0, 0]), float(vh[0, 1]), 0.0])
            if axis is not None:
                return axis

    if len(task_positions):
        fallback = _normalize_horizontal(np.asarray(task_positions[0], dtype=float) - tower_center)
        if fallback is not None:
            return fallback
    return np.asarray([1.0, 0.0, 0.0], dtype=float)


def _route_entry_exit_axes(
    safety: Dict[str, Any],
    tower_center: np.ndarray,
    task_positions: Sequence[np.ndarray],
) -> Tuple[np.ndarray, np.ndarray]:
    """Choose entry/exit axes from split conductor no-fly branches when available."""
    fallback_axis = _route_line_axis({}, safety, tower_center, task_positions)
    if len(task_positions):
        first_delta = np.asarray(task_positions[0], dtype=float) - tower_center
        if float(np.dot(first_delta[:2], fallback_axis[:2])) < 0.0:
            fallback_axis = -fallback_axis

    volume_axes: List[np.ndarray] = []
    for volume in _no_fly_volumes(safety):
        axis = _normalize_horizontal(volume.u_axis)
        if axis is None:
            continue
        if not any(float(np.linalg.norm((axis - existing)[:2])) < 0.12 for existing in volume_axes):
            volume_axes.append(axis)
    if len(volume_axes) < 2 or not len(task_positions):
        return fallback_axis, -fallback_axis

    first_direction = _normalize_horizontal(np.asarray(task_positions[0], dtype=float) - tower_center)
    if first_direction is None:
        first_direction = fallback_axis
    entry_axis = max(volume_axes, key=lambda axis: float(np.dot(axis[:2], first_direction[:2])))
    exit_candidates = [axis for axis in volume_axes if float(np.dot(axis[:2], entry_axis[:2])) < 0.85]
    if exit_candidates:
        exit_axis = min(exit_candidates, key=lambda axis: float(np.dot(axis[:2], entry_axis[:2])))
    else:
        exit_axis = -entry_axis
    return _normalize_or_fallback(entry_axis), _normalize_or_fallback(exit_axis)


def _normalize_or_fallback(axis: Sequence[float]) -> np.ndarray:
    normalized = _normalize_horizontal(axis)
    return normalized if normalized is not None else np.asarray([1.0, 0.0, 0.0], dtype=float)


def _tower_overhead_entry_position(
    tower_center: np.ndarray,
    task_positions: Sequence[np.ndarray],
    safety: Dict[str, Any],
    wire_clearance_m: float,
    tower_clearance_m: float,
) -> np.ndarray:
    position = tower_center.copy()
    task_top = max((float(point[2]) for point in task_positions), default=float(tower_center[2]))
    local_radius = 45.0
    local_structure_top = max(
        _local_cloud_top_around(tower_center, safety.get("wire", np.empty((0, 3), dtype=float)), local_radius),
        _local_cloud_top_around(tower_center, safety.get("tower", np.empty((0, 3), dtype=float)), local_radius),
    )
    clearance = max(float(wire_clearance_m), float(tower_clearance_m), 2.0)
    position[2] = max(
        float(tower_center[2]) + 20.0,
        task_top + 8.0,
        local_structure_top + clearance + 6.0 if math.isfinite(local_structure_top) else -float("inf"),
        _max_no_fly_top(safety) + clearance + 6.0,
    )
    for _ in range(24):
        wire_distance, _ = _nearest_distance(position, safety.get("wire", np.empty((0, 3), dtype=float)))
        tower_distance, _ = _nearest_distance(position, safety.get("tower", np.empty((0, 3), dtype=float)))
        if (
            wire_distance >= float(wire_clearance_m)
            and tower_distance >= float(tower_clearance_m)
            and not _inside_no_fly(position, safety)
        ):
            break
        position[2] += max(2.0, clearance * 0.5)
    return position


def _nearest_segment_obstacle(
    a: np.ndarray,
    b: np.ndarray,
    cloud: np.ndarray,
    sample_count: int = 16,
) -> Tuple[float, Optional[np.ndarray], Optional[np.ndarray], float]:
    if cloud.size == 0:
        return float("inf"), None, None, 0.5
    best_distance = float("inf")
    best_sample: Optional[np.ndarray] = None
    best_obstacle: Optional[np.ndarray] = None
    best_t = 0.5
    for i in range(sample_count + 1):
        t = i / max(sample_count, 1)
        sample = a * (1 - t) + b * t
        distance, obstacle = _nearest_distance(sample, cloud)
        if distance < best_distance:
            best_distance = distance
            best_sample = sample
            best_obstacle = obstacle
            best_t = t
    return best_distance, best_sample, best_obstacle, best_t


def _offset_task_corridor_points(
    prev_pos: np.ndarray,
    cur_pos: np.ndarray,
    obstacle: np.ndarray,
    t: float,
    direction: np.ndarray,
    offset: float,
    min_z: float,
) -> List[np.ndarray]:
    segment = cur_pos - prev_pos
    length = float(np.linalg.norm(segment))
    if length <= 1e-6:
        return []
    window = min(0.24, max(0.10, 7.0 / length))
    t1 = max(0.12, t - window)
    t2 = min(0.88, t + window)
    if t2 <= t1 + 0.05:
        t1 = max(0.10, t - 0.16)
        t2 = min(0.90, t + 0.16)
    points: List[np.ndarray] = []
    for tt in (t1, t2):
        base = prev_pos * (1 - tt) + cur_pos * tt
        detour = base.copy()
        detour[:2] = base[:2] + direction[:2] * offset
        detour[2] = max(float(prev_pos[2]), float(cur_pos[2]), float(base[2]), min_z)
        points.append(detour)
    return points


def _detour_score(
    prev_pos: np.ndarray,
    cur_pos: np.ndarray,
    detours: Sequence[np.ndarray],
    safety: Dict[str, np.ndarray],
    tower_clearance_m: float,
    wire_clearance_m: float,
) -> float:
    chain = [prev_pos] + list(detours) + [cur_pos]
    score = float("inf")
    for point in detours:
        wire_point_dist, _ = _nearest_distance(point, safety.get("wire", np.empty((0, 3))))
        tower_point_dist, _ = _nearest_distance(point, safety.get("tower", np.empty((0, 3))))
        score = min(score, wire_point_dist - wire_clearance_m, tower_point_dist - tower_clearance_m)
        score = min(score, _no_fly_clearance(point, safety))
    for idx in range(1, len(chain)):
        if _segment_crosses_no_fly(chain[idx - 1], chain[idx], safety):
            score = min(score, -1.0)
        wire_dist = _segment_min_distance(chain[idx - 1], chain[idx], safety.get("wire", np.empty((0, 3))), sample_count=10)
        tower_dist = _segment_min_distance(chain[idx - 1], chain[idx], safety.get("tower", np.empty((0, 3))), sample_count=10)
        score = min(score, wire_dist - wire_clearance_m, tower_dist - tower_clearance_m)
    return score


def _local_cloud_top(
    a: np.ndarray,
    b: np.ndarray,
    cloud: np.ndarray,
    padding: float,
) -> float:
    if cloud.size == 0:
        return max(float(a[2]), float(b[2]))
    lower = np.minimum(a, b) - padding
    upper = np.maximum(a, b) + padding
    local = cloud[np.all((cloud >= lower) & (cloud <= upper), axis=1)]
    if len(local):
        return float(np.max(local[:, 2]))
    return max(float(a[2]), float(b[2]))


def _segment_detour_chain(
    prev_pos: np.ndarray,
    cur_pos: np.ndarray,
    obstacle: np.ndarray,
    obstacle_t: float,
    safety: Dict[str, np.ndarray],
    tower_clearance_m: float,
    wire_clearance_m: float,
    risk_key: str,
    tower_center: np.ndarray,
) -> List[np.ndarray]:
    segment = cur_pos - prev_pos
    length = float(np.linalg.norm(segment))
    if length <= 1e-6:
        return []
    local_top = _local_cloud_top(prev_pos, cur_pos, safety.get(risk_key, np.empty((0, 3))), max(wire_clearance_m, tower_clearance_m) * 2.0)
    z_values = sorted(
        {
            max(float(prev_pos[2]), float(cur_pos[2]), local_top + 2.0),
            max(float(prev_pos[2]), float(cur_pos[2]), local_top + wire_clearance_m + 4.0),
            max(float(prev_pos[2]), float(cur_pos[2]), local_top + wire_clearance_m + 10.0),
        }
    )
    base_dirs = [
        obstacle - tower_center,
        tower_center - obstacle,
        np.asarray([-segment[1], segment[0], 0.0], dtype=float),
        np.asarray([segment[1], -segment[0], 0.0], dtype=float),
        prev_pos - obstacle,
        cur_pos - obstacle,
    ]
    dirs: List[np.ndarray] = []
    for raw in base_dirs:
        direction = raw.copy()
        direction[2] = 0.0
        norm = float(np.linalg.norm(direction[:2]))
        if norm > 1e-6:
            unit = direction / norm
            if not any(float(np.linalg.norm((unit - existing)[:2])) < 0.15 for existing in dirs):
                dirs.append(unit)
    if not dirs:
        dirs.append(np.asarray([1.0, 0.0, 0.0], dtype=float))

    t_window = min(0.28, max(0.12, 8.0 / length))
    t1 = max(0.08, obstacle_t - t_window)
    t2 = min(0.92, obstacle_t + t_window)
    bases = [prev_pos * (1 - t1) + cur_pos * t1, prev_pos * (1 - t2) + cur_pos * t2]
    offsets = [max(wire_clearance_m, tower_clearance_m) * scale for scale in (1.6, 2.4, 3.4, 4.5)]

    best_score = -float("inf")
    best_length = float("inf")
    best_chain: List[np.ndarray] = []
    for direction in dirs:
        for offset in offsets:
            for z_value in z_values:
                chain: List[np.ndarray] = []
                for base in bases:
                    point = base.copy()
                    point[:2] = base[:2] + direction[:2] * offset
                    point[2] = z_value
                    chain.append(point)
                score = _detour_score(prev_pos, cur_pos, chain, safety, tower_clearance_m, wire_clearance_m)
                length_score = _segment_length([prev_pos] + chain + [cur_pos])
                if score >= -1e-6 and length_score < best_length:
                    best_score = score
                    best_length = length_score
                    best_chain = chain
                elif not best_chain and score > best_score:
                    best_score = score
                    best_length = length_score
                    best_chain = chain
    return best_chain


def _item_segment_clearance(
    prev_item: Dict[str, Any],
    next_item: Dict[str, Any],
    tower_clearance_m: float,
    wire_clearance_m: float,
    task_tower_clearance_m: float,
    task_wire_clearance_m: float,
    task_segment_limit_m: float = 80.0,
) -> Tuple[float, float]:
    task_kinds = {"capture", "task_detour"}
    touches_task_corridor = prev_item.get("kind") in task_kinds or next_item.get("kind") in task_kinds
    if touches_task_corridor and _distance(prev_item["pos"], next_item["pos"]) <= task_segment_limit_m:
        return task_tower_clearance_m, task_wire_clearance_m
    return tower_clearance_m, wire_clearance_m


def _insert_detours(
    points: List[Dict[str, Any]],
    safety: Dict[str, np.ndarray],
    tower_clearance_m: float,
    wire_clearance_m: float,
    tower_center: np.ndarray,
    task_tower_clearance_m: float,
    task_wire_clearance_m: float,
) -> List[Dict[str, Any]]:
    if len(points) < 2:
        return points
    planned: List[Dict[str, Any]] = [points[0]]
    for current in points[1:]:
        prev_item = planned[-1]
        prev_pos = prev_item["pos"]
        cur_pos = current["pos"]
        task_kinds = {"capture", "task_detour"}
        both_task_corridor = prev_item.get("kind") in task_kinds and current.get("kind") in task_kinds
        both_original_capture = prev_item.get("kind") == "capture" and current.get("kind") == "capture"
        segment_tower_clearance_m, segment_wire_clearance_m = _item_segment_clearance(
            prev_item,
            current,
            tower_clearance_m,
            wire_clearance_m,
            task_tower_clearance_m,
            task_wire_clearance_m,
        )
        wire_dist = _segment_min_distance(prev_pos, cur_pos, safety.get("wire", np.empty((0, 3))), sample_count=10)
        tower_dist = _segment_min_distance(prev_pos, cur_pos, safety.get("tower", np.empty((0, 3))), sample_count=10)
        wire_risk = wire_dist < segment_wire_clearance_m
        tower_risk = tower_dist < segment_tower_clearance_m
        no_fly_risk = _segment_crosses_no_fly(prev_pos, cur_pos, safety)
        if both_task_corridor and not both_original_capture and not (wire_risk or tower_risk or no_fly_risk):
            planned.append(current)
            continue
        if both_original_capture and not (wire_risk or tower_risk or no_fly_risk):
            planned.append(current)
            continue
        if no_fly_risk and not (wire_risk or tower_risk):
            detour_path = _no_fly_detour_path(prev_pos, cur_pos, safety, segment_tower_clearance_m, segment_wire_clearance_m)
            detour_kind = "task_detour" if both_original_capture else "detour"
            if detour_path and detour_kind == "task_detour":
                detour_path = _densify_path(detour_path, 60.0)
            for detour in (detour_path or [])[1:-1]:
                if _distance(planned[-1]["pos"], detour) >= 1.0 and _distance(detour, cur_pos) >= 1.0:
                    planned.append({"kind": detour_kind, "pos": detour, "target": current["target"], "focus": "auxiliary"})
            planned.append(current)
            continue
        if wire_risk or tower_risk:
            risk_key = "wire" if wire_risk and wire_dist <= tower_dist else "tower"
            clearance = segment_wire_clearance_m if risk_key == "wire" else segment_tower_clearance_m
            risk_cloud = safety.get(risk_key, np.empty((0, 3), dtype=float))
            _, sample, obstacle, obstacle_t = _nearest_segment_obstacle(prev_pos, cur_pos, risk_cloud)
            if sample is None:
                sample = (prev_pos + cur_pos) / 2.0
            if obstacle is None:
                obstacle = tower_center
            detours: List[np.ndarray] = []
            detours_are_validated_path = False
            if no_fly_risk:
                no_fly_path = _no_fly_detour_path(prev_pos, cur_pos, safety, segment_tower_clearance_m, segment_wire_clearance_m)
                if no_fly_path:
                    detours = no_fly_path[1:-1]
                    detours_are_validated_path = True
            if not detours:
                detours = _segment_detour_chain(
                    prev_pos,
                    cur_pos,
                    obstacle,
                    obstacle_t,
                    safety,
                    segment_tower_clearance_m,
                    segment_wire_clearance_m,
                    risk_key,
                    tower_center,
                )
            lateral_offset = max(5.0, clearance * 1.5) if both_original_capture else max(2.5, clearance * 0.7)
            segment = cur_pos - prev_pos
            min_detour_z = float(obstacle[2]) + (clearance + 3.0 if risk_key == "wire" else 2.0)
            raw_dirs = [
                sample - obstacle,
                np.asarray([-segment[1], segment[0], 0.0], dtype=float),
                np.asarray([segment[1], -segment[0], 0.0], dtype=float),
                sample - tower_center,
                tower_center - sample,
            ]
            candidates: List[Tuple[float, List[np.ndarray]]] = []
            for raw_dir in raw_dirs:
                direction = raw_dir.copy()
                direction[2] = 0.0
                norm = float(np.linalg.norm(direction[:2]))
                if norm <= 1e-6:
                    continue
                direction_unit = direction / norm
                for scale in (1.0, 1.45):
                    candidate_detours = _offset_task_corridor_points(
                        prev_pos,
                        cur_pos,
                        obstacle,
                        obstacle_t,
                        direction_unit,
                        lateral_offset * scale,
                        min_detour_z,
                    )
                    if candidate_detours:
                        candidates.append(
                            (
                                _detour_score(
                                    prev_pos,
                                    cur_pos,
                                    candidate_detours,
                                    safety,
                                    segment_tower_clearance_m,
                                    segment_wire_clearance_m,
                                ),
                                candidate_detours,
                            )
                        )
            if not detours:
                detours = max(candidates, key=lambda item: item[0])[1] if candidates else []
            if not detours:
                detours = [sample.copy()]
                fallback = sample - tower_center
                fallback[2] = 0.0
                fallback_norm = float(np.linalg.norm(fallback[:2])) or 1.0
                detours[0][:2] = sample[:2] + fallback[:2] / fallback_norm * lateral_offset
                detours[0][2] = max(float(prev_pos[2]), float(cur_pos[2]), float(sample[2]), min_detour_z)
            detour_kind = "task_detour" if both_original_capture else "detour"
            if detour_kind == "task_detour" and detours:
                detours = _densify_path([prev_pos] + list(detours) + [cur_pos], 60.0)[1:-1]
            for detour in detours:
                if not detours_are_validated_path:
                    detour = _adjust_for_route_constraints(detour, current["target"], safety, segment_wire_clearance_m, segment_tower_clearance_m)
                if _distance(planned[-1]["pos"], detour) >= 1.0 and _distance(detour, cur_pos) >= 1.0:
                    planned.append({"kind": detour_kind, "pos": detour, "target": current["target"], "focus": "auxiliary"})
        planned.append(current)
    return planned


def _repair_unsafe_segments(
    route_items: List[Dict[str, Any]],
    safety: Dict[str, np.ndarray],
    tower_clearance_m: float,
    wire_clearance_m: float,
    tower_center: np.ndarray,
    task_tower_clearance_m: float,
    task_wire_clearance_m: float,
    max_rounds: int = 4,
) -> List[Dict[str, Any]]:
    repaired = list(route_items)
    for _ in range(max_rounds):
        next_items = _insert_detours(
            repaired,
            safety,
            tower_clearance_m,
            wire_clearance_m,
            tower_center,
            task_tower_clearance_m,
            task_wire_clearance_m,
        )
        next_items, _ = _repair_segments_with_astar(
            next_items,
            safety,
            tower_clearance_m,
            wire_clearance_m,
            task_tower_clearance_m,
            task_wire_clearance_m,
        )
        next_items = _prune_auxiliary_points(
            next_items,
            safety,
            tower_clearance_m,
            wire_clearance_m,
            task_tower_clearance_m,
            task_wire_clearance_m,
        )
        unsafe = False
        for idx in range(1, len(next_items)):
            segment_tower_clearance_m, segment_wire_clearance_m = _item_segment_clearance(
                next_items[idx - 1],
                next_items[idx],
                tower_clearance_m,
                wire_clearance_m,
                task_tower_clearance_m,
                task_wire_clearance_m,
            )
            if not _segment_is_safe(next_items[idx - 1]["pos"], next_items[idx]["pos"], safety, segment_tower_clearance_m, segment_wire_clearance_m):
                unsafe = True
                break
        repaired = next_items
        if not unsafe:
            break
    return repaired


def _repair_segments_with_astar(
    route_items: List[Dict[str, Any]],
    safety: Dict[str, np.ndarray],
    tower_clearance_m: float,
    wire_clearance_m: float,
    task_tower_clearance_m: float,
    task_wire_clearance_m: float,
) -> Tuple[List[Dict[str, Any]], int]:
    if len(route_items) < 2:
        return route_items, 0
    repaired: List[Dict[str, Any]] = [route_items[0]]
    repaired_count = 0
    for current in route_items[1:]:
        prev_item = repaired[-1]
        segment_tower_clearance_m, segment_wire_clearance_m = _item_segment_clearance(
            prev_item,
            current,
            tower_clearance_m,
            wire_clearance_m,
            task_tower_clearance_m,
            task_wire_clearance_m,
        )
        if _segment_is_safe(prev_item["pos"], current["pos"], safety, segment_tower_clearance_m, segment_wire_clearance_m):
            repaired.append(current)
            continue
        path = _find_local_astar_path(prev_item["pos"], current["pos"], safety, segment_tower_clearance_m, segment_wire_clearance_m)
        if not path or len(path) < 2:
            repaired.append(current)
            continue
        if not all(
            _segment_is_safe(path[idx - 1], path[idx], safety, segment_tower_clearance_m, segment_wire_clearance_m)
            for idx in range(1, len(path))
        ):
            repaired.append(current)
            continue
        repaired_count += 1
        task_kinds = {"capture", "task_detour"}
        aux_kind = "task_detour" if prev_item.get("kind") in task_kinds or current.get("kind") in task_kinds else "detour"
        if aux_kind == "task_detour":
            path = _densify_path(path, 60.0)
        for point in path[1:-1]:
            if _distance(repaired[-1]["pos"], point) < 1.0 or _distance(point, current["pos"]) < 1.0:
                continue
            repaired.append({"kind": aux_kind, "pos": point, "target": current["target"], "focus": "auxiliary"})
        repaired.append(current)
    return repaired, repaired_count


def _segment_crosses_no_fly(
    a: np.ndarray,
    b: np.ndarray,
    safety: Dict[str, Any],
    sample_step_m: float = 1.5,
) -> bool:
    volumes = _no_fly_volumes(safety)
    if not volumes:
        return False
    return any(volume.segment_intersects(a, b) for volume in volumes)


def _segment_min_no_fly_clearance(
    a: np.ndarray,
    b: np.ndarray,
    safety: Dict[str, Any],
    sample_step_m: float = 1.5,
) -> float:
    if not _no_fly_volumes(safety):
        return float("inf")
    length = _distance(a, b)
    sample_count = max(2, int(math.ceil(length / max(sample_step_m, 0.2))))
    best = float("inf")
    for index in range(sample_count + 1):
        t = index / sample_count
        point = a * (1 - t) + b * t
        best = min(best, _no_fly_clearance(point, safety))
    return best


def _no_fly_segment_margin(safety: Dict[str, Any]) -> float:
    volumes = _no_fly_volumes(safety)
    if not volumes:
        return 0.0
    configured = safety.get("no_fly_margin_m")
    if configured is not None:
        return max(float(configured), max(float(volume.tolerance_m) for volume in volumes))
    return max(float(volume.tolerance_m) for volume in volumes) + 0.25


def _segment_is_safe(
    a: np.ndarray,
    b: np.ndarray,
    safety: Dict[str, np.ndarray],
    tower_clearance_m: float,
    wire_clearance_m: float,
) -> bool:
    if _segment_crosses_no_fly(a, b, safety):
        return False
    if _segment_min_no_fly_clearance(a, b, safety) < _no_fly_segment_margin(safety):
        return False
    wire_dist = _segment_min_distance(a, b, safety.get("wire", np.empty((0, 3))), sample_count=10)
    tower_dist = _segment_min_distance(a, b, safety.get("tower", np.empty((0, 3))), sample_count=10)
    return wire_dist >= wire_clearance_m and tower_dist >= tower_clearance_m


def _prune_auxiliary_points(
    route_items: List[Dict[str, Any]],
    safety: Dict[str, np.ndarray],
    tower_clearance_m: float,
    wire_clearance_m: float,
    task_tower_clearance_m: float,
    task_wire_clearance_m: float,
) -> List[Dict[str, Any]]:
    if len(route_items) <= 2:
        return route_items
    pruned = list(route_items)
    changed = True
    while changed:
        changed = False
        next_items: List[Dict[str, Any]] = [pruned[0]]
        for idx in range(1, len(pruned) - 1):
            item = pruned[idx]
            prev_item = next_items[-1]
            next_item = pruned[idx + 1]
            segment_tower_clearance_m, segment_wire_clearance_m = _item_segment_clearance(
                prev_item,
                next_item,
                tower_clearance_m,
                wire_clearance_m,
                task_tower_clearance_m,
                task_wire_clearance_m,
            )
            if item.get("kind") in {"detour", "task_detour"} and _segment_is_safe(prev_item["pos"], next_item["pos"], safety, segment_tower_clearance_m, segment_wire_clearance_m):
                changed = True
                continue
            next_items.append(item)
        next_items.append(pruned[-1])
        pruned = next_items
    return pruned


def _spread_task_points(
    items: List[Dict[str, Any]],
    tower_center: np.ndarray,
    safety: Dict[str, np.ndarray],
    tower_clearance_m: float,
    wire_clearance_m: float,
    min_spacing_m: float = 4.0,
) -> List[Dict[str, Any]]:
    tasks = [item for item in items if item.get("kind") == "capture"]
    if len(tasks) < 2:
        return items
    for _ in range(8):
        moved = False
        for i in range(len(tasks)):
            push = np.zeros(3, dtype=float)
            for j in range(len(tasks)):
                if i == j:
                    continue
                delta = tasks[i]["pos"] - tasks[j]["pos"]
                delta[2] = 0.0
                dist = float(np.linalg.norm(delta))
                if 1e-6 < dist < min_spacing_m:
                    push += (delta / dist) * (min_spacing_m - dist) * 0.45
            if np.linalg.norm(push[:2]) > 1e-6:
                tasks[i]["pos"] = tasks[i]["pos"] + push
                tasks[i]["pos"] = _adjust_for_route_constraints(tasks[i]["pos"], tasks[i]["target"], safety, wire_clearance_m, tower_clearance_m)
                moved = True
        if not moved:
            break
    return items


def _route_items_from_manual(route_payload: Dict[str, Any]) -> List[Dict[str, Any]]:
    points: List[Dict[str, Any]] = []
    if route_payload.get("points"):
        points.extend(route_payload.get("points") or [])
    for tower in route_payload.get("towers") or []:
        points.extend(tower.get("points") or [])
    route_items: List[Dict[str, Any]] = []
    for point in points:
        lat = point.get("latitude")
        lon = point.get("longitude")
        if lat is None or lon is None:
            continue
        try:
            easting, northing, _, _ = utm.from_latlon(float(lat), float(lon))
        except Exception:
            continue
        action = str(point.get("actionName") or point.get("action") or "").lower()
        point_type = str(point.get("point_type") or "").lower()
        is_task_detour = bool(point.get("TaskCorridor") or point.get("task_corridor"))
        is_task = point_type == "task" or action == "photo"
        route_items.append(
            {
                "kind": "capture" if is_task else ("task_detour" if is_task_detour else "auxiliary"),
                "pos": np.asarray([easting, northing, _safe_float(point.get("altitude"))], dtype=float),
                "raw": point,
            }
        )
    return route_items


def _route_positions_from_manual(route_payload: Dict[str, Any]) -> List[np.ndarray]:
    return [item["pos"] for item in _route_items_from_manual(route_payload)]


def _segment_nearby_points(a: np.ndarray, b: np.ndarray, cloud: np.ndarray, margin: float) -> np.ndarray:
    if cloud.size == 0:
        return np.empty((0, 3), dtype=float)
    lower = np.minimum(a, b) - margin
    upper = np.maximum(a, b) + margin
    mask = np.all((cloud >= lower) & (cloud <= upper), axis=1)
    return cloud[mask]


def _point_to_segment_distances(a: np.ndarray, b: np.ndarray, pts: np.ndarray) -> np.ndarray:
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom <= 1e-9:
        return np.linalg.norm(pts - a, axis=1)
    t = np.clip(((pts - a) @ ab) / denom, 0.0, 1.0)
    closest = a + t[:, None] * ab
    return np.linalg.norm(pts - closest, axis=1)


def _segment_min_distance(a: np.ndarray, b: np.ndarray, cloud: np.ndarray, sample_count: int = 8) -> float:
    if cloud.size == 0:
        return float("inf")
    pts = _segment_nearby_points(a, b, cloud, margin=25.0)
    if len(pts) == 0:
        distance_a, _ = _nearest_distance(a, cloud)
        distance_b, _ = _nearest_distance(b, cloud)
        return min(distance_a, distance_b)
    return float(np.min(_point_to_segment_distances(a, b, pts)))


def _segment_min_horizontal_distance(a: np.ndarray, b: np.ndarray, cloud: np.ndarray, margin: float = 25.0) -> float:
    if cloud.size == 0:
        return float("inf")
    pts = _segment_nearby_points(a, b, cloud, margin=margin)
    if len(pts) == 0:
        distance_a, _ = _nearest_distance(a, cloud)
        distance_b, _ = _nearest_distance(b, cloud)
        return min(distance_a, distance_b)
    a2 = np.asarray([a[0], a[1], 0.0], dtype=float)
    b2 = np.asarray([b[0], b[1], 0.0], dtype=float)
    pts2 = np.column_stack((pts[:, 0], pts[:, 1], np.zeros(len(pts), dtype=float)))
    return float(np.min(_point_to_segment_distances(a2, b2, pts2)))


def _wire_points_in_segment_interior(
    a: np.ndarray,
    b: np.ndarray,
    wire_cloud: np.ndarray,
    margin: float,
    endpoint_fraction: float = 0.12,
) -> np.ndarray:
    pts = _segment_nearby_points(a, b, wire_cloud, margin=margin)
    if len(pts) == 0:
        return pts
    a2 = np.asarray([a[0], a[1]], dtype=float)
    b2 = np.asarray([b[0], b[1]], dtype=float)
    ab = b2 - a2
    denom = float(np.dot(ab, ab))
    if denom <= 1e-9:
        return pts
    t = np.clip(((pts[:, :2] - a2) @ ab) / denom, 0.0, 1.0)
    return pts[(t >= endpoint_fraction) & (t <= 1.0 - endpoint_fraction)]


def _segment_min_horizontal_distance_for_points(a: np.ndarray, b: np.ndarray, pts: np.ndarray) -> float:
    if len(pts) == 0:
        return float("inf")
    a2 = np.asarray([a[0], a[1], 0.0], dtype=float)
    b2 = np.asarray([b[0], b[1], 0.0], dtype=float)
    pts2 = np.column_stack((pts[:, 0], pts[:, 1], np.zeros(len(pts), dtype=float)))
    return float(np.min(_point_to_segment_distances(a2, b2, pts2)))


def _wire_segment_clearance_distance(a: np.ndarray, b: np.ndarray, wire_cloud: np.ndarray, wire_clearance_m: float) -> float:
    """Treat mid-segment wire avoidance as horizontal clearance unless the segment overflies it."""
    if wire_cloud.size == 0:
        return float("inf")
    distance_3d = _segment_min_distance(a, b, wire_cloud)
    if float(np.linalg.norm((b - a)[:2])) <= max(2.0, float(wire_clearance_m)):
        return distance_3d
    local_wire = _wire_points_in_segment_interior(a, b, wire_cloud, margin=max(25.0, float(wire_clearance_m) * 3.0))
    if len(local_wire) == 0:
        return distance_3d
    segment_low_z = min(float(a[2]), float(b[2]))
    overflight_z = float(np.max(local_wire[:, 2])) + max(float(wire_clearance_m), 1.0)
    if segment_low_z >= overflight_z:
        return distance_3d
    horizontal_distance = _segment_min_horizontal_distance_for_points(a, b, local_wire)
    return min(distance_3d, horizontal_distance)


def _local_obstacle_points(
    a: np.ndarray,
    b: np.ndarray,
    safety: Dict[str, np.ndarray],
    margin: float,
    extra_top: float,
) -> Dict[str, np.ndarray]:
    lower = np.minimum(a, b) - margin
    upper = np.maximum(a, b) + margin
    upper[2] = max(float(upper[2]), _local_cloud_top(a, b, safety.get("wire", np.empty((0, 3))), margin) + extra_top)
    result: Dict[str, np.ndarray] = {}
    for key in ("wire", "tower"):
        cloud = safety.get(key, np.empty((0, 3), dtype=float))
        if cloud.size == 0:
            result[key] = np.empty((0, 3), dtype=float)
            continue
        mask = np.all((cloud >= lower) & (cloud <= upper), axis=1)
        result[key] = cloud[mask]
    return result


def _build_occupied_cells(
    local_safety: Dict[str, np.ndarray],
    origin: np.ndarray,
    resolution: float,
    tower_clearance_m: float,
    wire_clearance_m: float,
) -> set[Tuple[int, int, int]]:
    occupied: set[Tuple[int, int, int]] = set()
    for key, clearance in (("wire", wire_clearance_m), ("tower", tower_clearance_m)):
        cloud = local_safety.get(key, np.empty((0, 3), dtype=float))
        if cloud.size == 0:
            continue
        radius = max(1, int(math.ceil(clearance / resolution)))
        cell_radius_sq = (clearance / resolution + 0.5) ** 2
        base_cells = np.rint((cloud - origin) / resolution).astype(int)
        for cx, cy, cz in base_cells:
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    for dz in range(-radius, radius + 1):
                        if dx * dx + dy * dy + dz * dz <= cell_radius_sq:
                            occupied.add((int(cx + dx), int(cy + dy), int(cz + dz)))
    return occupied


def _add_no_fly_occupied_cells(
    occupied: set[Tuple[int, int, int]],
    safety: Dict[str, Any],
    origin: np.ndarray,
    resolution: float,
    max_cell: Tuple[int, int, int],
) -> None:
    tolerance = max(float(resolution) * 0.75, 0.3, _no_fly_segment_margin(safety))
    for volume in _no_fly_volumes(safety):
        world_corners: List[np.ndarray] = []
        for u_value in (volume.u_min, volume.u_max):
            for v_value in (volume.v_min, volume.v_max):
                for z_value in (volume.lower_z(), volume.upper_z()):
                    world_corners.append(volume.world_position(u_value, v_value, z_value))
        if not world_corners:
            continue
        corners = np.vstack(world_corners)
        min_cell = np.floor((np.min(corners, axis=0) - origin - tolerance) / resolution).astype(int)
        max_cell_arr = np.ceil((np.max(corners, axis=0) - origin + tolerance) / resolution).astype(int)
        min_cell = np.maximum(min_cell, np.zeros(3, dtype=int))
        max_cell_arr = np.minimum(max_cell_arr, np.asarray(max_cell, dtype=int))
        if np.any(max_cell_arr < min_cell):
            continue
        for cx in range(int(min_cell[0]), int(max_cell_arr[0]) + 1):
            for cy in range(int(min_cell[1]), int(max_cell_arr[1]) + 1):
                for cz in range(int(min_cell[2]), int(max_cell_arr[2]) + 1):
                    point = _cell_to_point((cx, cy, cz), origin, resolution)
                    if volume.contains(point, tolerance_m=tolerance):
                        occupied.add((cx, cy, cz))


def _cell_to_point(cell: Tuple[int, int, int], origin: np.ndarray, resolution: float) -> np.ndarray:
    return origin + np.asarray(cell, dtype=float) * resolution


def _point_to_cell(point: np.ndarray, origin: np.ndarray, resolution: float) -> Tuple[int, int, int]:
    arr = np.rint((point - origin) / resolution).astype(int)
    return int(arr[0]), int(arr[1]), int(arr[2])


def _astar_grid_path(
    start: np.ndarray,
    goal: np.ndarray,
    safety: Dict[str, np.ndarray],
    tower_clearance_m: float,
    wire_clearance_m: float,
    resolution: float = 1.5,
    max_nodes: int = 35000,
    margin_extra_m: float = 0.0,
    top_extra_m: float = 0.0,
) -> Optional[List[np.ndarray]]:
    planning_tower_clearance = tower_clearance_m + 0.75
    planning_wire_clearance = wire_clearance_m + 0.75
    margin = max(10.0, tower_clearance_m, wire_clearance_m) + 8.0 + margin_extra_m
    extra_top = max(12.0, wire_clearance_m + 8.0) + top_extra_m
    lower = np.minimum(start, goal) - margin
    local_safety = _local_obstacle_points(start, goal, safety, margin, extra_top)
    local_top = max(
        _local_cloud_top(start, goal, local_safety.get("wire", np.empty((0, 3))), margin),
        _local_cloud_top(start, goal, local_safety.get("tower", np.empty((0, 3))), margin),
        float(start[2]),
        float(goal[2]),
    )
    upper = np.maximum(start, goal) + margin
    upper[2] = max(float(upper[2]), local_top + extra_top)
    origin = lower.copy()
    max_cell = np.ceil((upper - origin) / resolution).astype(int)
    max_tuple = (int(max_cell[0]), int(max_cell[1]), int(max_cell[2]))
    if max(max_tuple) > 90:
        resolution = max(resolution, max(float(np.max(upper - origin)) / 110.0, 1.5))
        max_cell = np.ceil((upper - origin) / resolution).astype(int)
        max_tuple = (int(max_cell[0]), int(max_cell[1]), int(max_cell[2]))

    occupied = _build_occupied_cells(local_safety, origin, resolution, planning_tower_clearance, planning_wire_clearance)
    _add_no_fly_occupied_cells(occupied, safety, origin, resolution, max_tuple)
    start_cell = _point_to_cell(start, origin, resolution)
    goal_cell = _point_to_cell(goal, origin, resolution)
    occupied.discard(start_cell)
    occupied.discard(goal_cell)

    def in_bounds(cell: Tuple[int, int, int]) -> bool:
        return 0 <= cell[0] <= max_tuple[0] and 0 <= cell[1] <= max_tuple[1] and 0 <= cell[2] <= max_tuple[2]

    def heuristic(cell: Tuple[int, int, int]) -> float:
        return _distance(_cell_to_point(cell, origin, resolution), goal)

    neighbors: List[Tuple[Tuple[int, int, int], float]] = []
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                if dx == 0 and dy == 0 and dz == 0:
                    continue
                step = math.sqrt(dx * dx + dy * dy + dz * dz) * resolution
                vertical_penalty = 1.08 if dz else 1.0
                neighbors.append(((dx, dy, dz), step * vertical_penalty))

    open_heap: List[Tuple[float, float, Tuple[int, int, int]]] = [(heuristic(start_cell), 0.0, start_cell)]
    came_from: Dict[Tuple[int, int, int], Tuple[int, int, int]] = {}
    cost_so_far: Dict[Tuple[int, int, int], float] = {start_cell: 0.0}
    visited = 0
    while open_heap and visited < max_nodes:
        _, current_cost, current = heapq.heappop(open_heap)
        if current == goal_cell:
            cells = [current]
            while cells[-1] != start_cell:
                cells.append(came_from[cells[-1]])
            cells.reverse()
            points = [_cell_to_point(cell, origin, resolution) for cell in cells]
            points[0] = start.copy()
            points[-1] = goal.copy()
            return points
        if current_cost > cost_so_far.get(current, float("inf")) + 1e-9:
            continue
        visited += 1
        for delta, step_cost in neighbors:
            nxt = (current[0] + delta[0], current[1] + delta[1], current[2] + delta[2])
            if not in_bounds(nxt) or nxt in occupied:
                continue
            new_cost = current_cost + step_cost
            if new_cost < cost_so_far.get(nxt, float("inf")):
                cost_so_far[nxt] = new_cost
                came_from[nxt] = current
                heapq.heappush(open_heap, (new_cost + heuristic(nxt), new_cost, nxt))
    return None


def _no_fly_detour_path(
    start: np.ndarray,
    goal: np.ndarray,
    safety: Dict[str, Any],
    tower_clearance_m: float,
    wire_clearance_m: float,
) -> Optional[List[np.ndarray]]:
    if not _segment_crosses_no_fly(start, goal, safety):
        return None
    margin = max(2.0, float(tower_clearance_m), float(wire_clearance_m)) + 1.5
    for volume in _no_fly_volumes(safety):
        if not _segment_crosses_no_fly(start, goal, {"no_fly": [volume]}):
            continue
        start_u, start_v, start_z = volume.local_coordinates(start)
        goal_u, goal_v, goal_z = volume.local_coordinates(goal)
        side_mid = (volume.v_min + volume.v_max) / 2.0

        def outside_v(value: float) -> float:
            return volume.v_min - margin if value <= side_mid else volume.v_max + margin

        def top_for(value: float) -> float:
            return volume.upper_z() + margin

        def bottom_for(value: float) -> float:
            return volume.lower_z() - margin

        route_z = max(float(start_z), float(goal_z), top_for(start_v), top_for(goal_v))
        low_route_z = min(float(start_z), float(goal_z), bottom_for(start_v), bottom_for(goal_v))
        u_options = sorted(
            (volume.u_min - margin, volume.u_max + margin),
            key=lambda u_value: min(abs(u_value - start_u), abs(u_value - goal_u)),
        )
        raw_candidates: List[List[np.ndarray]] = []
        for bypass_u in u_options:
            start_side_v = outside_v(start_v)
            goal_side_v = outside_v(goal_v)
            raw_candidates.append(
                [
                    start,
                    volume.world_position(start_u, start_side_v, route_z),
                    volume.world_position(bypass_u, start_side_v, route_z),
                    volume.world_position(bypass_u, goal_side_v, route_z),
                    volume.world_position(goal_u, goal_side_v, route_z),
                    goal,
                ]
            )
            raw_candidates.append(
                [
                    start,
                    volume.world_position(start_u, start_side_v, low_route_z),
                    volume.world_position(bypass_u, start_side_v, low_route_z),
                    volume.world_position(bypass_u, goal_side_v, low_route_z),
                    volume.world_position(goal_u, goal_side_v, low_route_z),
                    goal,
                ]
            )
        for side_v in (volume.v_min - margin, volume.v_max + margin):
            raw_candidates.append(
                [
                    start,
                    volume.world_position(start_u, side_v, route_z),
                    volume.world_position(goal_u, side_v, route_z),
                    goal,
                ]
            )
            raw_candidates.append(
                [
                    start,
                    volume.world_position(start_u, side_v, low_route_z),
                    volume.world_position(goal_u, side_v, low_route_z),
                    goal,
                ]
            )

        best: Optional[List[np.ndarray]] = None
        best_len = float("inf")
        for candidate in raw_candidates:
            cleaned: List[np.ndarray] = [candidate[0]]
            for point in candidate[1:-1]:
                adjusted = _adjust_for_route_constraints(point, goal, safety, wire_clearance_m, tower_clearance_m)
                if _distance(cleaned[-1], adjusted) >= 0.75:
                    cleaned.append(adjusted)
            if _distance(cleaned[-1], candidate[-1]) >= 0.75:
                cleaned.append(candidate[-1])
            else:
                cleaned[-1] = candidate[-1]
            if len(cleaned) < 2:
                continue
            if all(
                _segment_is_safe(cleaned[idx - 1], cleaned[idx], safety, tower_clearance_m, wire_clearance_m)
                for idx in range(1, len(cleaned))
            ):
                length = _segment_length(cleaned)
                if length < best_len:
                    best = cleaned
                    best_len = length
        if best is not None:
            return best
    return None


def _find_local_astar_path(
    start: np.ndarray,
    goal: np.ndarray,
    safety: Dict[str, np.ndarray],
    tower_clearance_m: float,
    wire_clearance_m: float,
) -> Optional[List[np.ndarray]]:
    attempts = (
        (1.5, 45000, 0.0, 0.0),
        (1.75, 70000, 8.0, 8.0),
        (2.0, 110000, 16.0, 14.0),
        (2.5, 160000, 28.0, 20.0),
    )
    best_path: Optional[List[np.ndarray]] = None
    best_len = float("inf")
    for resolution, max_nodes, margin_extra, top_extra in attempts:
        path = _astar_grid_path(
            start,
            goal,
            safety,
            tower_clearance_m,
            wire_clearance_m,
            resolution=resolution,
            max_nodes=max_nodes,
            margin_extra_m=margin_extra,
            top_extra_m=top_extra,
        )
        if not path:
            continue
        path = _smooth_safe_path(path, safety, tower_clearance_m, wire_clearance_m)
        if len(path) >= 2 and all(
            _segment_is_safe(path[idx - 1], path[idx], safety, tower_clearance_m, wire_clearance_m)
            for idx in range(1, len(path))
        ):
            length = _segment_length(path)
            if length < best_len:
                best_path = path
                best_len = length
    if best_path is not None:
        return best_path
    no_fly_path = _no_fly_detour_path(start, goal, safety, tower_clearance_m, wire_clearance_m)
    if no_fly_path is not None:
        return no_fly_path
    return _overflight_bridge_path(start, goal, safety, tower_clearance_m, wire_clearance_m)


def _overflight_bridge_path(
    start: np.ndarray,
    goal: np.ndarray,
    safety: Dict[str, np.ndarray],
    tower_clearance_m: float,
    wire_clearance_m: float,
) -> Optional[List[np.ndarray]]:
    margin = max(25.0, tower_clearance_m, wire_clearance_m) * 2.0
    local_wire = _segment_nearby_points(start, goal, safety.get("wire", np.empty((0, 3))), margin=margin)
    local_tower = _segment_nearby_points(start, goal, safety.get("tower", np.empty((0, 3))), margin=margin)
    local_top = max(
        float(np.max(local_wire[:, 2])) if len(local_wire) else -float("inf"),
        float(np.max(local_tower[:, 2])) if len(local_tower) else -float("inf"),
        float(start[2]),
        float(goal[2]),
    )
    bridge_z = max(
        float(start[2]) + 8.0,
        float(goal[2]) + 8.0,
        local_top + max(float(wire_clearance_m), float(tower_clearance_m)) + 6.0,
    )
    start_high = start.copy()
    start_high[2] = bridge_z
    goal_high = goal.copy()
    goal_high[2] = bridge_z

    candidates = [
        [start, start_high, goal_high, goal],
    ]

    segment = goal - start
    lateral_dirs = [
        np.asarray([-segment[1], segment[0], 0.0], dtype=float),
        np.asarray([segment[1], -segment[0], 0.0], dtype=float),
    ]
    lateral_offset = max(float(wire_clearance_m), float(tower_clearance_m), 6.0) * 1.8
    for raw_dir in lateral_dirs:
        norm = float(np.linalg.norm(raw_dir[:2]))
        if norm <= 1e-6:
            continue
        direction = raw_dir / norm
        a = start_high.copy()
        b = goal_high.copy()
        a[:2] += direction[:2] * lateral_offset
        b[:2] += direction[:2] * lateral_offset
        candidates.append([start, start_high, a, b, goal_high, goal])

    for candidate in sorted(candidates, key=_segment_length):
        if all(
            _segment_is_safe(candidate[idx - 1], candidate[idx], safety, tower_clearance_m, wire_clearance_m)
            for idx in range(1, len(candidate))
        ):
            return candidate
    return None


def _smooth_safe_path(
    points: Sequence[np.ndarray],
    safety: Dict[str, np.ndarray],
    tower_clearance_m: float,
    wire_clearance_m: float,
) -> List[np.ndarray]:
    if len(points) <= 2:
        return list(points)
    smoothed: List[np.ndarray] = [points[0]]
    cursor = 0
    while cursor < len(points) - 1:
        next_index = len(points) - 1
        while next_index > cursor + 1:
            if _segment_is_safe(points[cursor], points[next_index], safety, tower_clearance_m, wire_clearance_m):
                break
            next_index -= 1
        smoothed.append(points[next_index])
        cursor = next_index
    return smoothed


def _plan_local_astar_segments(
    route_items: List[Dict[str, Any]],
    safety: Dict[str, np.ndarray],
    tower_clearance_m: float,
    wire_clearance_m: float,
    task_tower_clearance_m: float,
    task_wire_clearance_m: float,
) -> Tuple[List[Dict[str, Any]], int, int]:
    if len(route_items) < 2:
        return route_items, 0, 0
    planned: List[Dict[str, Any]] = [route_items[0]]
    astar_segments = 0
    fallback_segments = 0
    for idx in range(1, len(route_items)):
        prev_item = planned[-1]
        current = route_items[idx]
        seg_tower, seg_wire = _item_segment_clearance(
            prev_item,
            current,
            tower_clearance_m,
            wire_clearance_m,
            task_tower_clearance_m,
            task_wire_clearance_m,
        )
        if _segment_is_safe(prev_item["pos"], current["pos"], safety, seg_tower, seg_wire):
            planned.append(current)
            continue
        path = _find_local_astar_path(prev_item["pos"], current["pos"], safety, seg_tower, seg_wire)
        if not path or len(path) < 2:
            fallback_segments += 1
            planned.append(current)
            continue
        astar_segments += 1
        aux_kind = "task_detour" if prev_item.get("kind") in {"capture", "task_detour"} and current.get("kind") == "capture" else "detour"
        if aux_kind == "task_detour":
            path = _densify_path(path, 60.0)
        for point in path[1:-1]:
            if _distance(planned[-1]["pos"], point) < 1.0 or _distance(point, current["pos"]) < 1.0:
                continue
            planned.append({"kind": aux_kind, "pos": point, "target": current["target"], "focus": "auxiliary"})
        planned.append(current)
    return planned, astar_segments, fallback_segments


def validate_route_safety(
    route_path: Path,
    voxel_path: Optional[Path],
    tower_clearance_m: float = 6.0,
    wire_clearance_m: float = 10.0,
    task_tower_clearance_m: float = 3.0,
    task_wire_clearance_m: float = 5.0,
) -> Dict[str, Any]:
    route_payload = _read_json(Path(route_path))
    route_items = _route_items_from_manual(route_payload)
    if not route_items:
        raise ValueError("航线文件中没有可校验的经纬度航点")
    positions = [item["pos"] for item in route_items]
    safety = _load_safety_points(voxel_path)
    towers = route_payload.get("towers") or []
    first_tower = towers[0] if towers else {}
    tower_center = _xyz(first_tower.get("PlaneCenterPoint")) if isinstance(first_tower, dict) else None
    if tower_center is None:
        tower_center = np.mean(np.vstack(positions), axis=0)
    if not _no_fly_volumes(safety) and not _apply_embedded_no_fly(safety, route_payload):
        _apply_route_context_no_fly(safety, route_payload, tower_center)
    _set_no_fly_margin(safety, max(float(tower_clearance_m), float(wire_clearance_m), float(task_tower_clearance_m), float(task_wire_clearance_m)))

    min_wire = float("inf")
    min_tower = float("inf")
    min_no_fly = float("inf")
    task_violations = 0
    auxiliary_violations = 0
    segment_violations = 0
    no_fly_violations = 0
    violations: List[Dict[str, Any]] = []
    for index, item in enumerate(route_items, start=1):
        point = item["pos"]
        is_task_corridor = item.get("kind") in {"capture", "task_detour"}
        point_tower_clearance_m = task_tower_clearance_m if is_task_corridor else tower_clearance_m
        point_wire_clearance_m = task_wire_clearance_m if is_task_corridor else wire_clearance_m
        wire_dist, _ = _nearest_distance(point, safety["wire"])
        tower_dist, _ = _nearest_distance(point, safety["tower"])
        no_fly_clearance = _no_fly_clearance(point, safety)
        min_wire = min(min_wire, wire_dist)
        min_tower = min(min_tower, tower_dist)
        min_no_fly = min(min_no_fly, no_fly_clearance)
        if wire_dist < point_wire_clearance_m:
            violations.append({"type": "point", "point_type": item["kind"], "target": "wire", "index": index, "distance_m": round(wire_dist, 3), "threshold_m": point_wire_clearance_m})
            if is_task_corridor:
                task_violations += 1
            else:
                auxiliary_violations += 1
        if tower_dist < point_tower_clearance_m:
            violations.append({"type": "point", "point_type": item["kind"], "target": "tower", "index": index, "distance_m": round(tower_dist, 3), "threshold_m": point_tower_clearance_m})
            if is_task_corridor:
                task_violations += 1
            else:
                auxiliary_violations += 1
        no_fly_margin = _no_fly_segment_margin(safety)
        if _inside_no_fly(point, safety) or no_fly_clearance < no_fly_margin:
            violations.append({"type": "point", "point_type": item["kind"], "target": "conductor_no_fly", "index": index, "distance_m": round(no_fly_clearance, 3), "threshold_m": round(no_fly_margin, 3)})
            no_fly_violations += 1
            if is_task_corridor:
                task_violations += 1
            else:
                auxiliary_violations += 1

    for index in range(1, len(route_items)):
        a = route_items[index - 1]["pos"]
        b = route_items[index]["pos"]
        segment_tower_clearance_m, segment_wire_clearance_m = _item_segment_clearance(
            route_items[index - 1],
            route_items[index],
            tower_clearance_m,
            wire_clearance_m,
            task_tower_clearance_m,
            task_wire_clearance_m,
        )
        wire_dist = _segment_min_distance(a, b, safety["wire"])
        tower_dist = _segment_min_distance(a, b, safety["tower"])
        no_fly_clearance = _segment_min_no_fly_clearance(a, b, safety)
        min_wire = min(min_wire, wire_dist)
        min_tower = min(min_tower, tower_dist)
        min_no_fly = min(min_no_fly, no_fly_clearance)
        if wire_dist < segment_wire_clearance_m:
            violations.append({"type": "segment", "target": "wire", "from": index, "to": index + 1, "distance_m": round(wire_dist, 3), "threshold_m": segment_wire_clearance_m})
            segment_violations += 1
        if tower_dist < segment_tower_clearance_m:
            violations.append({"type": "segment", "target": "tower", "from": index, "to": index + 1, "distance_m": round(tower_dist, 3), "threshold_m": segment_tower_clearance_m})
            segment_violations += 1
        no_fly_margin = _no_fly_segment_margin(safety)
        if _segment_crosses_no_fly(a, b, safety) or no_fly_clearance < no_fly_margin:
            violations.append({"type": "segment", "target": "conductor_no_fly", "from": index, "to": index + 1, "distance_m": round(no_fly_clearance, 3), "threshold_m": round(no_fly_margin, 3)})
            segment_violations += 1
            no_fly_violations += 1

    return {
        "passed": not violations,
        "point_count": len(positions),
        "task_point_count": sum(1 for item in route_items if item["kind"] == "capture"),
        "auxiliary_point_count": sum(1 for item in route_items if item["kind"] != "capture"),
        "task_corridor_auxiliary_count": sum(1 for item in route_items if item["kind"] == "task_detour"),
        "tower_clearance_m": tower_clearance_m,
        "wire_clearance_m": wire_clearance_m,
        "task_tower_clearance_m": task_tower_clearance_m,
        "task_wire_clearance_m": task_wire_clearance_m,
        "conductor_no_fly_margin_m": _no_fly_segment_margin(safety),
        "min_tower_distance_m": None if math.isinf(min_tower) else round(min_tower, 3),
        "min_wire_distance_m": None if math.isinf(min_wire) else round(min_wire, 3),
        "min_conductor_no_fly_clearance_m": None if math.isinf(min_no_fly) else round(min_no_fly, 3),
        "conductor_no_fly_volume_count": len(_no_fly_volumes(safety)),
        "conductor_no_fly_source": safety.get("no_fly_source"),
        "conductor_no_fly_violation_count": no_fly_violations,
        "violation_count": len(violations),
        "task_violation_count": task_violations,
        "auxiliary_violation_count": auxiliary_violations,
        "segment_violation_count": segment_violations,
        "violations": violations[:80],
        "truncated": len(violations) > 80,
        "voxel_file": Path(voxel_path).name if voxel_path else None,
    }


def _nearest_order(items: List[Dict[str, Any]], start: np.ndarray) -> List[Dict[str, Any]]:
    remaining = list(items)
    ordered: List[Dict[str, Any]] = []
    current = start
    while remaining:
        index = min(range(len(remaining)), key=lambda i: _distance(current, remaining[i]["pos"]))
        item = remaining.pop(index)
        ordered.append(item)
        current = item["pos"]
    return ordered


def _angular_order(items: List[Dict[str, Any]], tower_center: np.ndarray, start: np.ndarray) -> List[Dict[str, Any]]:
    if len(items) <= 2:
        return list(items)

    def angle(item: Dict[str, Any]) -> float:
        delta = item["pos"] - tower_center
        return math.atan2(float(delta[1]), float(delta[0]))

    ordered = sorted(items, key=angle)
    start_angle = math.atan2(float(start[1] - tower_center[1]), float(start[0] - tower_center[0]))
    split = min(
        range(len(ordered)),
        key=lambda i: abs(math.atan2(math.sin(angle(ordered[i]) - start_angle), math.cos(angle(ordered[i]) - start_angle))),
    )
    ccw = ordered[split:] + ordered[:split]
    cw = [ordered[split]] + list(reversed(ordered[:split])) + list(reversed(ordered[split + 1:]))
    ccw_len = _segment_length([start] + [item["pos"] for item in ccw])
    cw_len = _segment_length([start] + [item["pos"] for item in cw])
    return cw if cw_len < ccw_len else ccw


def _route_point(
    index: int,
    pos: np.ndarray,
    target: np.ndarray,
    aim_type: str,
    action: str,
    linename: str,
    zone: int,
    point_type: str = "task",
    source_waypoint_id: Optional[Any] = None,
    task_corridor: bool = False,
) -> Dict[str, Any]:
    lat, lon = _utm_to_latlon(pos, zone)
    aim_lat, aim_lon = _utm_to_latlon(target, zone)
    point = {
        "AimType": aim_type,
        "Distance": round(_distance(pos, target), 6),
        "MatchImg": "",
        "SerialNumber": str(index),
        "actionName": action,
        "altitude": round(float(pos[2]), 6),
        "altitude_Aim": round(float(target[2]), 6),
        "heading": _bearing_deg(pos, target),
        "isLast": 1,
        "latitude": lat,
        "latitude_Aim": aim_lat,
        "linename": linename,
        "longitude": lon,
        "longitude_Aim": aim_lon,
        "pitch": _pitch_deg(pos, target),
        "point_type": point_type,
    }
    if source_waypoint_id is not None:
        point["SourceWaypointId"] = str(source_waypoint_id)
    if task_corridor:
        point["TaskCorridor"] = True
    return point


def plan_route_from_waypoints(
    waypoint_path: Path,
    output_dir: Path,
    voxel_path: Optional[Path] = None,
    safety_distance_m: Optional[float] = None,
    clearance_m: float = 6.0,
    wire_clearance_m: float = 10.0,
    task_tower_clearance_m: float = 3.0,
    task_wire_clearance_m: float = 5.0,
    entry_distance_m: float = 28.0,
) -> Tuple[Path, Dict[str, Any]]:
    waypoint_path = Path(waypoint_path)
    payload = _read_json(waypoint_path)
    raw_waypoints = payload.get("waypoints") or []
    if not raw_waypoints:
        raise ValueError("算法航点文件中没有 waypoints")

    safety = _load_safety_points(voxel_path)
    tower_center = _load_voxel_center(voxel_path)
    positions = [_xyz(wp.get("pos_utm") or wp.get("position")) for wp in raw_waypoints]
    positions = [pos for pos in positions if pos is not None]
    if not positions:
        raise ValueError("算法航点文件中没有有效 UTM 坐标")
    if tower_center is None:
        tower_center = np.mean(np.vstack(positions), axis=0)

    zone = 50
    linename = waypoint_path.stem
    manual_targets: List[Dict[str, Any]] = []
    planning_tower_clearance_m = clearance_m
    planning_wire_clearance_m = wire_clearance_m
    task_tower_clearance_m = max(0.5, min(float(task_tower_clearance_m), planning_tower_clearance_m))
    task_wire_clearance_m = max(0.5, min(float(task_wire_clearance_m), planning_wire_clearance_m))
    _set_no_fly_margin(
        safety,
        max(
            float(planning_tower_clearance_m),
            float(planning_wire_clearance_m),
            float(task_tower_clearance_m),
            float(task_wire_clearance_m),
        ),
    )
    planned_items: List[Dict[str, Any]] = []
    skipped_waypoint_ids: List[str] = []
    for waypoint in raw_waypoints:
        pos = _xyz(waypoint.get("pos_utm") or waypoint.get("position"))
        if pos is None:
            skipped_waypoint_ids.append(str(waypoint.get("id", len(skipped_waypoint_ids) + 1)))
            continue
        target = _target_for_waypoint(waypoint, tower_center, manual_targets)
        pos = _adjust_for_route_constraints(pos, target, safety, task_wire_clearance_m, task_tower_clearance_m)
        planned_items.append(
            {
                "kind": "capture",
                "pos": pos,
                "target": target,
                "focus": _focus_from_waypoint(waypoint),
                "source_waypoint_id": waypoint.get("id"),
            }
        )
    if skipped_waypoint_ids:
        raise ValueError(f"航线规划输入中有 {len(skipped_waypoint_ids)} 个任务点缺少有效UTM坐标，已拒绝生成不完整航线: {', '.join(skipped_waypoint_ids[:8])}")
    if len(planned_items) != len(raw_waypoints):
        raise ValueError(f"航线规划任务点数量不一致: 输入 {len(raw_waypoints)} 个，规划 {len(planned_items)} 个")
    planned_items = _spread_task_points(planned_items, tower_center, safety, task_tower_clearance_m, task_wire_clearance_m, min_spacing_m=3.0)

    entry_axis, exit_axis = _route_entry_exit_axes(safety, tower_center, [item["pos"] for item in planned_items])
    entry_anchor = tower_center + entry_axis * entry_distance_m
    exit_anchor = tower_center + exit_axis * entry_distance_m
    entry_order_ref = entry_anchor.copy()
    entry_pos = _tower_overhead_entry_position(
        entry_anchor,
        [item["pos"] for item in planned_items],
        safety,
        planning_wire_clearance_m,
        planning_tower_clearance_m,
    )
    ordered = _angular_order(planned_items, tower_center, entry_order_ref)
    entry_target = tower_center.copy()
    entry = {"kind": "entry", "pos": entry_pos, "target": entry_target, "focus": "auxiliary"}
    exit_pos = _tower_overhead_entry_position(
        exit_anchor,
        [item["pos"] for item in planned_items],
        safety,
        planning_wire_clearance_m,
        planning_tower_clearance_m,
    )
    exit_pos[2] = max(float(ordered[-1]["pos"][2]), float(entry_pos[2]), float(exit_pos[2]))
    exit_target = ordered[-1]["target"].copy() if ordered else tower_center.copy()
    exit_item = {"kind": "exit", "pos": exit_pos, "target": exit_target, "focus": "auxiliary"}

    base_route_items = [entry] + ordered + [exit_item]
    route_items, astar_segment_count, astar_fallback_count = _plan_local_astar_segments(
        base_route_items,
        safety,
        planning_tower_clearance_m,
        planning_wire_clearance_m,
        task_tower_clearance_m,
        task_wire_clearance_m,
    )
    route_items = _prune_auxiliary_points(
        route_items,
        safety,
        planning_tower_clearance_m,
        planning_wire_clearance_m,
        task_tower_clearance_m,
        task_wire_clearance_m,
    )
    def count_unsafe_segments(items: Sequence[Dict[str, Any]]) -> int:
        count = 0
        for idx in range(1, len(items)):
            seg_tower, seg_wire = _item_segment_clearance(
                items[idx - 1],
                items[idx],
                planning_tower_clearance_m,
                planning_wire_clearance_m,
                task_tower_clearance_m,
                task_wire_clearance_m,
            )
            if not _segment_is_safe(items[idx - 1]["pos"], items[idx]["pos"], safety, seg_tower, seg_wire):
                count += 1
        return count

    def unsafe_segment_details(items: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
        details: List[Dict[str, Any]] = []
        for idx in range(1, len(items)):
            seg_tower, seg_wire = _item_segment_clearance(
                items[idx - 1],
                items[idx],
                planning_tower_clearance_m,
                planning_wire_clearance_m,
                task_tower_clearance_m,
                task_wire_clearance_m,
            )
            wire_dist = _segment_min_distance(items[idx - 1]["pos"], items[idx]["pos"], safety.get("wire", np.empty((0, 3))), sample_count=10)
            tower_dist = _segment_min_distance(items[idx - 1]["pos"], items[idx]["pos"], safety.get("tower", np.empty((0, 3))), sample_count=10)
            no_fly_clearance = _segment_min_no_fly_clearance(items[idx - 1]["pos"], items[idx]["pos"], safety)
            if wire_dist < seg_wire or tower_dist < seg_tower or no_fly_clearance < _no_fly_segment_margin(safety):
                details.append(
                    {
                        "from": idx,
                        "to": idx + 1,
                        "wire": round(wire_dist, 3),
                        "wire_threshold": round(seg_wire, 3),
                        "tower": round(tower_dist, 3),
                        "tower_threshold": round(seg_tower, 3),
                        "no_fly": round(no_fly_clearance, 3) if math.isfinite(no_fly_clearance) else None,
                    }
                )
        return details

    final_unsafe_segments = count_unsafe_segments(route_items)
    if final_unsafe_segments:
        route_items = _repair_unsafe_segments(
            route_items,
            safety,
            planning_tower_clearance_m,
            planning_wire_clearance_m,
            tower_center,
            task_tower_clearance_m,
            task_wire_clearance_m,
            max_rounds=3,
        )
        final_unsafe_segments = count_unsafe_segments(route_items)
    if final_unsafe_segments:
        details = unsafe_segment_details(route_items)[:3]
        raise ValueError(f"final route still has {final_unsafe_segments} unsafe segments after detour repair: {details}")

    planned_source_ids = {str(item.get("source_waypoint_id")) for item in planned_items if item.get("source_waypoint_id") is not None}
    routed_source_ids = {str(item.get("source_waypoint_id")) for item in route_items if item.get("kind") == "capture" and item.get("source_waypoint_id") is not None}
    missing_source_ids = sorted(planned_source_ids - routed_source_ids)
    if len([item for item in route_items if item.get("kind") == "capture"]) != len(planned_items) or missing_source_ids:
        detail = f": {', '.join(missing_source_ids[:8])}" if missing_source_ids else ""
        raise ValueError(f"航线规划未包含全部任务点{detail}")

    route_points: List[Dict[str, Any]] = []
    for index, item in enumerate(route_items, start=1):
        action = "photo" if item["kind"] == "capture" else "none"
        aim_type = "辅助点" if action == "none" else _aim_type_for_focus(str(item.get("focus") or ""))
        route_points.append(
            _route_point(
                index,
                item["pos"],
                item["target"],
                aim_type,
                action,
                linename,
                zone,
                point_type="task" if action == "photo" else "auxiliary",
                source_waypoint_id=item.get("source_waypoint_id"),
                task_corridor=item.get("kind") == "task_detour",
            )
        )

    route_positions = [item["pos"] for item in route_items]
    output: Dict[str, Any] = {}
    output["CoordinateSystem"] = str(output.get("CoordinateSystem") or "0")
    output["FlyType"] = str(output.get("FlyType") or "0")
    output["MinPts"] = output.get("MinPts") or []
    output["UTM"] = str(output.get("UTM") or zone)
    output["linename"] = linename
    output["sCameraType"] = output.get("sCameraType") or "84-24mm"
    output["tasktype"] = output.get("tasktype") or "精细化"
    output["totalLen"] = round(_segment_length(route_positions), 6)
    output["towercount"] = 1
    output["vLevel"] = output.get("vLevel") or ""
    output["version"] = output.get("version") or "V2.5"
    tower: Dict[str, Any] = {}
    tower["towername"] = Path(waypoint_path).stem
    tower["PlaneCenterPoint"] = ",".join(str(round(float(v), 6)) for v in tower_center)
    tower["points"] = route_points
    output["towers"] = [tower]
    output["route_planning"] = {
        "source_waypoint_file": waypoint_path.name,
        "source_voxel_file": Path(voxel_path).name if voxel_path else None,
        "algorithm": "local-voxel-astar-with-line-of-sight-smoothing",
        "safety_distance_m": safety_distance_m,
        "clearance_m": clearance_m,
        "wire_clearance_m": wire_clearance_m,
        "planning_tower_clearance_m": planning_tower_clearance_m,
        "planning_wire_clearance_m": planning_wire_clearance_m,
        "task_tower_clearance_m": task_tower_clearance_m,
        "task_wire_clearance_m": task_wire_clearance_m,
        "conductor_no_fly_margin_m": _no_fly_segment_margin(safety),
        "entry_distance_m": entry_distance_m,
        "entry_line_axis": [round(float(value), 8) for value in entry_axis.tolist()],
        "exit_line_axis": [round(float(value), 8) for value in exit_axis.tolist()],
        "entry_anchor_utm": [round(float(value), 6) for value in entry_anchor.tolist()],
        "exit_anchor_utm": [round(float(value), 6) for value in exit_anchor.tolist()],
        "capture_point_count": len(planned_items),
        "task_point_count": sum(1 for item in route_items if item["kind"] == "capture"),
        "auxiliary_point_count": sum(1 for item in route_items if item["kind"] != "capture"),
        "route_point_count": len(route_points),
        "detour_point_count": sum(1 for item in route_items if item["kind"] in {"detour", "task_detour"}),
        "task_corridor_auxiliary_count": sum(1 for item in route_items if item["kind"] == "task_detour"),
        "astar_segment_count": astar_segment_count,
        "astar_fallback_count": astar_fallback_count,
        "conductor_no_fly_volume_count": len(_no_fly_volumes(safety)),
        "conductor_no_fly_source": safety.get("no_fly_source"),
        "conductor_no_fly_volumes": [volume.to_record() for volume in _no_fly_volumes(safety)],
    }

    output_dir.mkdir(parents=True, exist_ok=True)
    output_name = f"{waypoint_path.stem}_自动航线规划.json"
    output_path = output_dir / output_name
    output_path.write_text(json.dumps(output, ensure_ascii=False, indent=2), encoding="utf-8")
    return output_path, output
