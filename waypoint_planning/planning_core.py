from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np
import utm


TOWER_LABEL = 16
INSULATOR_LABEL = 22
WIRE_LABEL = 0
GROUND_WIRE_LABEL = 3
STRUCTURE_LABELS = (TOWER_LABEL, INSULATOR_LABEL)
SAFETY_EXCLUDED_LABELS: Tuple[int, ...] = ()
CRITICAL_SAFETY_LABELS = (WIRE_LABEL, GROUND_WIRE_LABEL, TOWER_LABEL, INSULATOR_LABEL)

TARGET_SEMANTICS = ("insulator", "tower_top", "tower_edge", "tower_body")
ATTENTION_SEMANTICS = ("conductor_insulator_connection", "insulator_tower_side_connection", "ground_wire_tower_connection", "tower_base_connection")
NON_TARGET_SEMANTICS = ("tower_lower30", "wire", "ground_wire", "background") + ATTENTION_SEMANTICS
ALL_SEMANTICS = TARGET_SEMANTICS + NON_TARGET_SEMANTICS
# Legacy alias: wire_insulator_connection → conductor_insulator_connection
LEGACY_SEMANTIC_ALIASES = {"wire_insulator_connection": "conductor_insulator_connection"}
def _normalize_semantic(semantic: str) -> str:
    return LEGACY_SEMANTIC_ALIASES.get(semantic, semantic)

SEMANTIC_PRIORITY = {
    "insulator": 5,
    "conductor_insulator_connection": 5,
    "wire_insulator_connection": 5,
    "insulator_tower_side_connection": 5,
    "ground_wire_tower_connection": 4,
    "tower_base_connection": 3,
    "tower_top": 4,
    "tower_edge": 3,
    "tower_body": 1,
    "tower_overview": 2,
    "tower_lower30": 1,
}
PATCH_BIN_CONFIG = {
    "insulator": {"azimuth_deg": 30, "height_bins": 6},
    "tower_top": {"azimuth_deg": 45, "height_bins": 2},
    "tower_edge": {"azimuth_deg": 45, "height_bins": 4},
    "tower_body": {"azimuth_deg": 90, "height_bins": 2},
    "tower_lower30": {"azimuth_deg": 180, "height_bins": 1},
}
PATCH_CANDIDATE_CONFIG = {
    "insulator": {
        "radii": (5.5, 7.5),
        "z_offsets": (-0.5, 0.0, 0.5),
        "focals": ("F1", "F2"),
        "yaw_offsets": (0.0, -10.0, 10.0),
    },
    "tower_top": {
        "radii": (7.5, 10.5, 13.0),
        "z_offsets": (1.5, 3.0, 5.0),
        "focals": ("F1", "F2"),
        "yaw_offsets": (0.0, -8.0, 8.0),
    },
    "tower_edge": {
        "radii": (5.5, 7.5, 9.0),
        "z_offsets": (-1.0, 0.0, 1.0),
        "focals": ("F1", "F2"),
        "yaw_offsets": (0.0, -10.0, 10.0),
    },
    "tower_body": {
        "radii": (8.5, 11.5, 14.5),
        "z_offsets": (-1.5, 0.0, 1.5),
        "focals": ("F0", "F1"),
        "yaw_offsets": (0.0, -10.0, 10.0),
    },
    "tower_lower30": {
        "radii": (6.0, 8.0),
        "z_offsets": (-0.5, 0.5),
        "focals": ("F0", "F1", "F2"),
        "yaw_offsets": (0.0, -10.0, 10.0),
    },
    "conductor_insulator_connection": {
        "radii": (5.0, 7.0, 9.0),
        "z_offsets": (-0.4, 0.0, 0.4),
        "focals": ("F1", "F2"),
        "yaw_offsets": (0.0, -8.0, 8.0),
    },
    "wire_insulator_connection": {
        "radii": (5.0, 7.0, 9.0),
        "z_offsets": (-0.4, 0.0, 0.4),
        "focals": ("F1", "F2"),
        "yaw_offsets": (0.0, -8.0, 8.0),
    },
    "insulator_tower_side_connection": {
        "radii": (5.0, 7.0, 9.0),
        "z_offsets": (-0.4, 0.0, 0.4),
        "focals": ("F1", "F2"),
        "yaw_offsets": (0.0, -8.0, 8.0),
    },
    "ground_wire_tower_connection": {
        "radii": (6.0, 8.0, 10.0),
        "z_offsets": (-0.5, 0.0, 0.8),
        "focals": ("F1", "F2"),
        "yaw_offsets": (0.0, -8.0, 8.0),
    },
    "tower_base_connection": {
        "radii": (7.0, 10.0, 13.0),
        "z_ratios": (0.30, 0.35, 0.40),
        "focals": ("F0",),
        "yaw_offsets": (0.0, -10.0, 10.0),
    },
}

# ── AIMTYPE_VIEW_PROFILE: per-AimType shooting strategy ──────────────
# Used by candidate generation to determine position, pitch, focal.
# Distances and pitches are preferred values with acceptable ranges;
# focal_range gives [min, max] for choose_focal_length_eq_mm().
# max_candidates_per_target controls generation caps (final selection
# may be further constrained by the planner).
AIMTYPE_VIEW_PROFILE = {
    "tower_overview": {
        "aim_type": "塔全貌",
        "preferred_distance_m": 50.0,
        "distance_range_m": (40.0, 60.0),
        "preferred_pitch_deg": -33.0,
        "pitch_range_deg": (-45.0, -20.0),
        "focal_range_mm": (24, 35),
        "max_candidates": 6,
        "max_final_waypoints": 3,
        "action_name": "photo",
        "description": "远距离斜视拍摄整塔整体结构",
    },
    "tower_top": {
        "aim_type": "塔头",
        "preferred_distance_m": 15.0,
        "distance_range_m": (10.0, 22.0),
        "preferred_pitch_deg": -14.0,
        "pitch_range_deg": (-25.0, -10.0),
        "focal_range_mm": (35, 84),
        "max_candidates_per_layer_sector": 3,
        "action_name": "photo",
        "description": "中距离斜视塔头上部结构",
    },
    "tower_body": {
        "aim_type": "塔身",
        "preferred_distance_m": 18.0,
        "distance_range_m": (12.0, 28.0),
        "preferred_pitch_deg": -25.0,
        "pitch_range_deg": (-35.0, -15.0),
        "focal_range_mm": (35, 84),
        "max_candidates_per_layer_sector": 3,
        "action_name": "photo",
        "description": "中距离斜视塔身主体结构",
    },
    "tower_edge": {
        "aim_type": "塔边缘结构",
        "preferred_distance_m": 14.0,
        "distance_range_m": (10.0, 22.0),
        "preferred_pitch_deg": None,
        "pitch_range_deg": None,
        "focal_range_mm": (35, 84),
        "max_candidates_per_layer_sector": 3,
        "action_name": "photo",
        "description": "中距离拍摄塔材边缘外轮廓",
    },
    "tower_base": {
        "aim_type": "塔基",
        "preferred_distance_m": 18.0,
        "distance_range_m": (15.0, 28.0),
        "preferred_pitch_deg": -49.0,
        "pitch_range_deg": (-55.0, -40.0),
        "focal_range_mm": (35, 84),
        "max_candidates_per_layer_sector": 2,
        "action_name": "photo",
        "description": "中距离斜俯视塔基底部结构",
    },
    "insulator_string": {
        "aim_type": "绝缘子串",
        "preferred_distance_m": 14.0,
        "distance_range_m": (10.0, 18.0),
        "preferred_pitch_deg": 0.0,
        "pitch_range_deg": (-5.0, 5.0),
        "focal_range_mm": (35, 84),
        "max_candidates_per_instance": 6,
        "max_final_per_instance": 4,
        "action_name": "photo",
        "description": "中距离水平拍摄整串绝缘子",
    },
    "conductor_insulator_connection": {
        "aim_type": "导线端挂点",
        "preferred_distance_m": 13.0,
        "distance_range_m": (8.0, 14.0),
        "preferred_pitch_deg": -20.0,
        "pitch_range_deg": (-30.0, -10.0),
        "focal_range_mm": (60, 84),
        "max_candidates_per_target": 3,
        "max_final_per_target": 3,
        "action_name": "photo",
        "description": "中距离精细拍摄导线端金具连接点",
    },
    # Legacy alias — kept for backward compat with existing data
    "wire_insulator_connection": {
        "aim_type": "导线端挂点(旧)",
        "preferred_distance_m": 13.0,
        "distance_range_m": (8.0, 14.0),
        "preferred_pitch_deg": -20.0,
        "pitch_range_deg": (-30.0, -10.0),
        "focal_range_mm": (60, 84),
        "max_candidates_per_target": 3,
        "max_final_per_target": 3,
        "action_name": "photo",
        "description": "中距离精细拍摄导线端金具连接点(legacy alias)",
    },
    "insulator_tower_side_connection": {
        "aim_type": "绝缘子横担端挂点",
        "preferred_distance_m": 10.0,
        "distance_range_m": (8.0, 16.0),
        "preferred_pitch_deg": 0.0,
        "pitch_range_deg": (-5.0, 5.0),
        "focal_range_mm": (50, 84),
        "max_candidates_per_target": 3,
        "max_final_per_target": 3,
        "action_name": "photo",
        "description": "中距离拍摄横担端连接板金具",
    },
    "ground_wire_tower_connection": {
        "aim_type": "地线挂点",
        "preferred_distance_m": 13.0,
        "distance_range_m": (8.0, 14.0),
        "preferred_pitch_deg": -10.0,
        "pitch_range_deg": (-20.0, -5.0),
        "focal_range_mm": (60, 84),
        "max_candidates_per_target": 3,
        "max_final_per_target": 3,
        "action_name": "photo",
        "description": "中距离轻微俯视地线塔头连接处",
    },
    "grounding_connection": {
        "aim_type": "接地连接",
        "preferred_distance_m": 25.0,
        "distance_range_m": (15.0, 35.0),
        "preferred_pitch_deg": -40.0,
        "pitch_range_deg": (-55.0, -25.0),
        "focal_range_mm": (50, 84),
        "max_candidates_per_target": 3,
        "max_final_per_target": 3,
        "action_name": "photo",
        "description": "中近距离拍摄接地连接点",
    },
    "channel": {
        "aim_type": "通道",
        "preferred_distance_m": 16.0,
        "distance_range_m": (12.0, 20.0),
        "preferred_pitch_deg": 0.0,
        "pitch_range_deg": (-5.0, 5.0),
        "focal_range_mm": (24, 35),
        "max_candidates_per_target": 2,
        "action_name": "photo",
        "description": "水平广角拍摄线路通道走廊",
    },
    "auxiliary": {
        "aim_type": "辅助点",
        "preferred_distance_m": None,
        "distance_range_m": None,
        "preferred_pitch_deg": None,
        "pitch_range_deg": None,
        "focal_range_mm": None,
        "action_name": "none",
        "description": "航线过渡避障姿态切换，不参与覆盖率",
    },
}

FINAL_WAYPOINT_LIMITS = {
    "min_photo_waypoints": 10,
    "max_photo_waypoints": 80,
    "count_auxiliary_separately": True,
    "max_auxiliary_waypoints": 30,
}

SEMANTIC_WEIGHTS = {
    "insulator": 5.0,
    "conductor_insulator_connection": 5.0,
    "wire_insulator_connection": 5.0,
    "insulator_tower_side_connection": 5.0,
    "ground_wire_tower_connection": 4.2,
    "tower_base_connection": 3.0,
    "tower_top": 3.8,
    "tower_edge": 3.0,
    "tower_body": 1.8,
    "tower_overview": 2.0,
    "tower_lower30": 0.05,
    "wire": 0.0,
    "ground_wire": 0.0,
    "background": 0.0,
}
REQUIRED_RESOLUTION = {
    "insulator": 1.5,
    "conductor_insulator_connection": 1.3,
    "wire_insulator_connection": 1.3,
    "insulator_tower_side_connection": 1.3,
    "ground_wire_tower_connection": 1.2,
    "tower_base_connection": 0.9,
    "tower_top": 1.15,
    "tower_edge": 1.0,
    "tower_body": 0.7,
    "tower_lower30": 0.5,
    "tower_overview": 0.12,
}
INCIDENCE_THRESHOLDS = {
    "insulator": 45.0,
    "conductor_insulator_connection": 50.0,
    "wire_insulator_connection": 50.0,
    "insulator_tower_side_connection": 50.0,
    "ground_wire_tower_connection": 55.0,
    "tower_base_connection": 65.0,
    "tower_top": 50.0,
    "tower_edge": 55.0,
    "tower_body": 65.0,
    "tower_lower30": 70.0,
}
COVERAGE_THRESHOLDS = {
    "C_geo": 0.90,
    "C_weighted": 0.98,
    "C_ins": 0.98,
    "C_top": 0.94,
    "C_edge": 0.94,
    "C_body": 0.92,
}
COMPACT_COVERAGE_THRESHOLDS = {
    "C_geo": 0.90,
    "C_weighted": 0.90,
    "C_ins": 0.90,
    "C_top": 0.90,
    "C_edge": 0.90,
}
DEFAULT_LIMITS = {
    "min_waypoints": 20,
    "max_waypoints": 80,
    "max_total_shots": 240,
    "max_shots_per_waypoint": 3,
    "max_visibility_distance_m": 60.0,
    "safety_distance_m": 5.0,
    "non_critical_safety_max_voxels": 60000,
    "conductor_no_fly_extent_margin_m": 10.0,
    "conductor_no_fly_min_length_m": 80.0,
    "conductor_no_fly_exterior_clearance_m": 6.5,
    "conductor_no_fly_boundary_tolerance_m": 0.3,
    "manual_ratio_min": 0.70,
    "manual_ratio_max": 0.70,
    "single_layer_episodes": 10,
    "hierarchical_episodes": 10,
}
REPAIR_PRIORITY = ["insulator", "tower_top", "tower_edge"]
CAMERA_SENSOR_WIDTH_MM = 36.0
CAMERA_SENSOR_HEIGHT_MM = 24.0


def required_no_fly_clearance_m(
    safety_distance_m: float,
    user_clearance_m: Optional[float] = None,
) -> float:
    """Return the single conductor no-fly clearance threshold used by all stages."""
    safety = max(float(safety_distance_m), 0.0)
    if user_clearance_m is not None:
        return max(safety, float(user_clearance_m))
    return max(safety, float(DEFAULT_LIMITS["conductor_no_fly_exterior_clearance_m"]))

POWER_MODEL_CONFIG = {
    "power_model_version": "semantic_geometric_v1",
    "tower_layer_height_m": 1.5,
    "tower_sector_count": 12,
    "tower_body_weight": 2.0,
    "tower_edge_weight": 3.0,
    "tower_top_weight": 3.0,
    "tower_lower_weight": 0.8,
    "tower_safety_radius": 2.0,
    "tower_body_min_dist": 4.0,
    "tower_body_max_dist": 16.0,
    "tower_body_max_view_angle_deg": 70.0,
    "tower_edge_min_dist": 3.5,
    "tower_edge_max_dist": 14.0,
    "tower_edge_max_view_angle_deg": 65.0,
    "tower_top_min_dist": 5.0,
    "tower_top_max_dist": 18.0,
    "tower_top_max_view_angle_deg": 70.0,
    "tower_lower_min_dist": 4.0,
    "tower_lower_max_dist": 16.0,
    "tower_lower_max_view_angle_deg": 75.0,
    "insulator_segments": 6,
    "insulator_around": 6,
    "insulator_weight": 5.0,
    "insulator_min_dist": 2.0,
    "insulator_max_dist": 8.0,
    "insulator_max_view_angle_deg": 55.0,
    "wire_occ_radius": 0.10,
    "wire_safety_radius_from_limits": True,
    "connection_weight": 6.0,
    "ground_wire_connection_weight": 4.5,
    "tower_base_weight": 2.0,
    "use_conductor_no_fly_volume": True,
    "no_fly_required_clearance_mode": "max_safety_and_exterior_clearance",
    "candidate_filter_min_keep_ratio": 0.10,
    "candidate_filter_min_count": 20,
}

INSULATOR_INSTANCE_VIEW_CONFIG = {
    "directions": [
        "outward",
        "outward_plus_line",
        "outward_minus_line",
        "outward_plus_z",
        "outward_minus_z",
    ],
    "distances": [4.5, 6.0, 7.5, 9.0],
    "height_offsets": [0.0],
    "yaw_offsets_deg": [0.0],
    "max_candidates_per_instance": 30,
}

TOWER_CANDIDATE_LIMITS = {
    "tower_body": 2000,
    "tower_edge": 3000,
    "tower_top": 2000,
    "tower_lower30": 1000,
}

INSULATOR_CLUSTER_MIN_POINTS = 80
INSULATOR_CLUSTER_MIN_LENGTH_M = 0.4
INSULATOR_MERGE_MAX_DISTANCE_M = 1.5
INSULATOR_MERGE_MAX_AXIS_ANGLE_DEG = 25.0
INSULATOR_MERGE_MAX_OUTWARD_ANGLE_DEG = 30.0
INSULATOR_CLUSTER_SECOND_STAGE_EPS_M = 1.5
INSULATOR_MAX_CANDIDATE_INSTANCES = 80

MAX_ATTENTION_CANDIDATES = 5000
MAX_CANDIDATES_TOTAL = 50000
ENABLE_OBSERVABILITY_GAP_REPAIR = True
MAX_GAP_REPAIR_CANDIDATES_TOTAL = 2000
MAX_GAP_REPAIR_CANDIDATES_PER_TARGET = 3
MAX_GAP_REPAIR_TARGETS_PER_SEMANTIC = 300
GAP_REPAIR_SEMANTIC_PRIORITY = (
    "conductor_insulator_connection",
    "insulator",
    "tower_top",
    "tower_edge",
    "tower_body",
    "tower_lower30",
)


def build_supported_focals(max_eq_mm: float = 84.0) -> Dict[str, Dict[str, float]]:
    levels = [
        ("F0", 24.0, 1.0),
        ("F1", 48.0, 2.0),
        ("F2", 84.0, 4.0),
    ]
    supported = {}
    for level, focal_mm, min_distance in levels:
        if focal_mm > max_eq_mm:
            continue
        supported[level] = {
            "f_eq_mm": focal_mm,
            "min_distance_m": min_distance,
            "hfov_deg": math.degrees(2.0 * math.atan(CAMERA_SENSOR_WIDTH_MM / (2.0 * focal_mm))),
            "vfov_deg": math.degrees(2.0 * math.atan(CAMERA_SENSOR_HEIGHT_MM / (2.0 * focal_mm))),
        }
    return supported


SUPPORTED_FOCALS = build_supported_focals(84.0)


def estimate_patch_scale(
    display_voxels: Sequence[Dict[str, object]],
    local_center: Sequence[float],
    tower_height: float,
) -> Dict[str, Dict[str, float]]:
    center = np.asarray(local_center, dtype=float)
    height = max(float(tower_height), 1e-6)
    result: Dict[str, Dict[str, float]] = {}
    for semantic, cfg in PATCH_BIN_CONFIG.items():
        coords = np.asarray(
            [voxel["coord"] for voxel in display_voxels if str(voxel.get("semantic")) == semantic],
            dtype=float,
        )
        radii = np.linalg.norm(coords[:, :2] - center[:2], axis=1) if len(coords) else np.zeros(0, dtype=float)
        reference_radius = float(np.percentile(radii, 75)) if len(radii) else 0.0
        arc_length = 2.0 * math.pi * reference_radius * (float(cfg["azimuth_deg"]) / 360.0)
        semantic_height = float(np.max(coords[:, 2]) - np.min(coords[:, 2])) if len(coords) else height
        result[semantic] = {
            "azimuth_deg": float(cfg["azimuth_deg"]),
            "height_bins": float(cfg["height_bins"]),
            "approx_arc_length_m": round(float(arc_length), 3),
            "approx_height_m": round(float(max(semantic_height, height * 0.08) / max(int(cfg["height_bins"]), 1)), 3),
        }
    return result


def _safe_float(value, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return float(default)


def normalize_vector(vec: Sequence[float], fallback: Sequence[float] = (1.0, 0.0, 0.0)) -> np.ndarray:
    arr = np.asarray(vec, dtype=float)
    norm = float(np.linalg.norm(arr))
    if norm <= 1e-9:
        arr = np.asarray(fallback, dtype=float)
        norm = float(np.linalg.norm(arr))
        if norm <= 1e-9:
            return np.array([1.0, 0.0, 0.0], dtype=float)
    return arr / norm


def get_direction_vector(yaw: float, pitch: float) -> np.ndarray:
    yaw_rad = math.radians(90.0 - yaw)
    pitch_rad = math.radians(pitch)
    x = math.cos(pitch_rad) * math.cos(yaw_rad)
    y = math.cos(pitch_rad) * math.sin(yaw_rad)
    z = math.sin(pitch_rad)
    return normalize_vector([x, y, z])


def camera_basis(yaw: float, pitch: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    forward = get_direction_vector(yaw, pitch)
    world_up = np.array([0.0, 0.0, 1.0], dtype=float)
    right = np.cross(forward, world_up)
    if np.linalg.norm(right) <= 1e-8:
        right = np.array([1.0, 0.0, 0.0], dtype=float)
    right = normalize_vector(right)
    up = normalize_vector(np.cross(right, forward))
    return forward, right, up


def yaw_pitch_to_target(position: Sequence[float], target: Sequence[float]) -> Tuple[float, float]:
    vec = np.asarray(target, dtype=float) - np.asarray(position, dtype=float)
    horizontal = math.hypot(float(vec[0]), float(vec[1]))
    yaw = (90.0 - math.degrees(math.atan2(float(vec[1]), float(vec[0])))) % 360.0
    pitch = math.degrees(math.atan2(float(vec[2]), max(horizontal, 1e-6)))
    return yaw, pitch


def compute_view_geometry(position: Sequence[float], look_at: Sequence[float]) -> Dict[str, float]:
    """Compute Distance, heading, pitch from position → look_at.

    heading = azimuth(P→T), identical to yaw from yaw_pitch_to_target.
    pitch  = elevation angle, positive = look up.
    Distance = 3D Euclidean distance in meters.
    """
    p = np.asarray(position, dtype=float)
    t = np.asarray(look_at, dtype=float)
    vec = t - p
    horizontal = math.hypot(float(vec[0]), float(vec[1]))
    return {
        "Distance": float(np.linalg.norm(vec)),
        "heading": (90.0 - math.degrees(math.atan2(float(vec[1]), float(vec[0])))) % 360.0,
        "pitch": math.degrees(math.atan2(float(vec[2]), max(horizontal, 1e-6))),
    }


def choose_focal_length_eq_mm(
    aim_type: Optional[str],
    distance: float,
    target_size: Optional[float] = None,
    required_resolution: Optional[float] = None,
) -> Optional[float]:
    """Select continuous focal length (35mm equiv) to meet resolution requirements.

    Picks a focal in AIMTYPE_VIEW_PROFILE.focal_range_mm that delivers at least
    the required_resolution at the given distance (nadir).  Falls back to a
    distance-based heuristic when aim_type is None.

    Returns None for auxiliary (non-photo) aim types.
    """
    if aim_type is None:
        d = max(float(distance), 1.0)
        # Resolution-aware heuristic: min focal to deliver decent quality
        req_res = required_resolution if required_resolution is not None else 0.8
        min_focal_for_res = req_res * 3.0 * d  # cos(0°) = 1.0
        return round(max(24.0, min(84.0, min_focal_for_res)), 1)

    profile = AIMTYPE_VIEW_PROFILE.get(aim_type)
    if profile is None:
        return choose_focal_length_eq_mm(None, distance, target_size, required_resolution)

    focal_range = profile.get("focal_range_mm")
    if focal_range is None:
        return None  # auxiliary

    f_min, f_max = float(focal_range[0]), float(focal_range[1])
    d = max(float(distance), 1.0)

    _aim_to_sem = {
        "insulator_string": "insulator",
        "conductor_insulator_connection": "conductor_insulator_connection",
        "wire_insulator_connection": "conductor_insulator_connection",
        "insulator_tower_side_connection": "insulator_tower_side_connection",
        "ground_wire_tower_connection": "ground_wire_tower_connection",
        "grounding_connection": "tower_base_connection",
        "tower_overview": "tower_overview",
        "tower_top": "tower_top",
        "tower_body": "tower_body",
        "tower_edge": "tower_edge",
        "tower_base": "tower_base_connection",
        "channel": "tower_body",
    }

    # Determine required resolution for this aim type
    if required_resolution is None:
        # Map aim_type to semantic and look up required resolution
        sem = _aim_to_sem.get(aim_type, "tower_body")
        required_resolution = REQUIRED_RESOLUTION.get(sem, 0.7)

    # Minimum focal to meet required resolution at this distance.
    # Use the incidence threshold (worst-case allowed viewing angle) for
    # a conservative cos estimate, since actual incidence depends on target
    # normal vs camera ray and can be much larger than the geometric pitch.
    inc_max = INCIDENCE_THRESHOLDS.get(_aim_to_sem.get(aim_type, aim_type), 65.0)
    cos_inc = max(0.3, math.cos(math.radians(float(inc_max))))
    pref_pitch = profile.get("preferred_pitch_deg")
    # For close-range connection types, the incidence angle can diverge
    # significantly from the geometric pitch (side view vs target normal).
    # Use the pure incidence-based estimate to guarantee resolution at
    # all allowed viewing angles.
    _connection_aims = ("conductor_insulator_connection", "wire_insulator_connection",
                        "insulator_tower_side_connection",
                        "ground_wire_tower_connection", "grounding_connection")
    if aim_type in _connection_aims:
        cos_est = cos_inc
    elif pref_pitch is not None:
        cos_pitch = max(0.4, math.cos(math.radians(abs(float(pref_pitch)))))
        cos_est = 0.35 * cos_inc + 0.65 * cos_pitch  # weight toward pitch (preferred angle)
    else:
        cos_est = cos_inc
    min_focal_for_res = required_resolution * 3.0 * d / cos_est

    # Clamp to profile's available range, but prefer upper end for close shots
    # where resolution demands it
    pref_dist = profile.get("preferred_distance_m")
    if min_focal_for_res >= f_max:
        # Need more focal than profile allows → use max, resolution check will gate
        chosen = f_max
    elif min_focal_for_res <= f_min:
        chosen = f_min
    elif pref_dist is not None and d <= float(pref_dist) * 1.15:
        # Near preferred distance: choose focal that meets resolution with 15% margin
        chosen = min_focal_for_res * 1.08
    elif d < float(pref_dist or 50.0):
        # Closer → slightly wider than resolution minimum
        chosen = min_focal_for_res * 0.92
    else:
        # Farther → resolution-driven
        chosen = min_focal_for_res * 1.05

    return round(max(f_min, min(f_max, chosen)), 1)


def observation_resolution(f_eq_mm: float, distance_m: float, incidence_angle_deg: float) -> float:
    distance = max(float(distance_m), 0.5)
    cos_factor = max(0.2, math.cos(math.radians(max(0.0, incidence_angle_deg))))
    return (float(f_eq_mm) / 24.0) * (8.0 / distance) * cos_factor


def normalize_focal_level(focal_level: Optional[str] = None, f_eq_mm: Optional[float] = None) -> Tuple[str, Dict[str, float]]:
    if focal_level and focal_level in SUPPORTED_FOCALS:
        return focal_level, SUPPORTED_FOCALS[focal_level]
    if f_eq_mm is not None:
        nearest = min(SUPPORTED_FOCALS.items(), key=lambda item: abs(item[1]["f_eq_mm"] - float(f_eq_mm)))
        return nearest[0], nearest[1]
    first = next(iter(SUPPORTED_FOCALS.items()))
    return first[0], first[1]


def check_occlusion(observer_pos, target_pos, z_max_map, min_bound, cell_size) -> bool:
    x0 = int((observer_pos[0] - min_bound[0]) / cell_size)
    y0 = int((observer_pos[1] - min_bound[1]) / cell_size)
    x1 = int((target_pos[0] - min_bound[0]) / cell_size)
    y1 = int((target_pos[1] - min_bound[1]) / cell_size)

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    curr_x, curr_y = x0, y0
    total_steps = max(dx, dy)
    if total_steps == 0:
        return True

    z_start = observer_pos[2]
    z_end = target_pos[2]

    map_h, map_w = z_max_map.shape
    steps_taken = 0
    while True:
        if 0 <= curr_x < map_w and 0 <= curr_y < map_h:
            ratio = steps_taken / total_steps
            curr_ray_z = z_start + (z_end - z_start) * ratio
            if z_max_map[curr_y, curr_x] > curr_ray_z + 0.2:
                if max(abs(curr_x - x1), abs(curr_y - y1)) > 1:
                    return False

        if curr_x == x1 and curr_y == y1:
            break

        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            curr_x += sx
        if e2 < dx:
            err += dx
            curr_y += sy
        steps_taken += 1
        if steps_taken > total_steps * 1.5:
            break
    return True


def _decode_text(value, default: str = "") -> str:
    if value is None:
        return default
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="ignore")
    return str(value)


def _connected_components(keys: Iterable[Tuple[int, int, int]]) -> List[List[Tuple[int, int, int]]]:
    key_set = set(keys)
    visited = set()
    neighbor_offsets = [
        (dx, dy, dz)
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        for dz in (-1, 0, 1)
        if not (dx == 0 and dy == 0 and dz == 0)
    ]
    components: List[List[Tuple[int, int, int]]] = []

    for key in key_set:
        if key in visited:
            continue
        stack = [key]
        visited.add(key)
        component = []
        while stack:
            current = stack.pop()
            component.append(current)
            for dx, dy, dz in neighbor_offsets:
                nxt = (current[0] + dx, current[1] + dy, current[2] + dz)
                if nxt in key_set and nxt not in visited:
                    visited.add(nxt)
                    stack.append(nxt)
        components.append(component)
    return components


def _voxel_centroids(
    points: np.ndarray,
    labels: np.ndarray,
    voxel_size: float,
    max_voxels: Optional[int] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """Compress point samples into voxel centroids with dominant labels."""
    points = np.asarray(points, dtype=float)
    labels = np.asarray(labels, dtype=int)
    if len(points) == 0:
        return np.empty((0, 3), dtype=float), np.empty((0,), dtype=int)

    min_bound = np.min(points, axis=0)
    voxel_map: Dict[Tuple[int, int, int], Dict[str, object]] = {}
    for point, label in zip(points, labels):
        key = tuple(int(math.floor((float(point[i]) - float(min_bound[i])) / voxel_size)) for i in range(3))
        if key not in voxel_map:
            voxel_map[key] = {"sum": np.zeros(3, dtype=float), "count": 0, "labels": {}}
        cell = voxel_map[key]
        cell["sum"] += point
        cell["count"] += 1
        cell["labels"][int(label)] = cell["labels"].get(int(label), 0) + 1

    rows = []
    for cell in voxel_map.values():
        dominant_label = max(cell["labels"].items(), key=lambda item: item[1])[0]
        centroid = cell["sum"] / max(int(cell["count"]), 1)
        rows.append((int(cell["count"]), centroid, dominant_label))
    rows.sort(key=lambda item: item[0], reverse=True)
    if max_voxels is not None and len(rows) > max_voxels:
        rows = rows[:max_voxels]

    centroids = np.asarray([row[1] for row in rows], dtype=float) if rows else np.empty((0, 3), dtype=float)
    dominant_labels = np.asarray([row[2] for row in rows], dtype=int) if rows else np.empty((0,), dtype=int)
    return centroids, dominant_labels


def build_label_preserving_safety_points(
    points: np.ndarray,
    labels: np.ndarray,
    voxel_size: float,
) -> Tuple[np.ndarray, np.ndarray, Dict[str, object]]:
    """Build safety samples while preserving critical power-grid labels exactly."""
    all_points = np.asarray(points, dtype=float)
    all_labels = np.asarray(labels, dtype=int)
    finite_mask = np.all(np.isfinite(all_points), axis=1)
    if SAFETY_EXCLUDED_LABELS:
        finite_mask &= ~np.isin(all_labels, list(SAFETY_EXCLUDED_LABELS))

    critical_mask = finite_mask & np.isin(all_labels, list(CRITICAL_SAFETY_LABELS))
    critical_points = all_points[critical_mask]
    critical_labels = all_labels[critical_mask]

    non_critical_mask = finite_mask & ~np.isin(all_labels, list(CRITICAL_SAFETY_LABELS))
    non_critical_points, non_critical_labels = _voxel_centroids(
        all_points[non_critical_mask],
        all_labels[non_critical_mask],
        max(voxel_size * 10.0, 1.0),
        max_voxels=int(DEFAULT_LIMITS["non_critical_safety_max_voxels"]),
    )

    if len(critical_points) and len(non_critical_points):
        safety_points = np.vstack([critical_points, non_critical_points])
        safety_labels = np.concatenate([critical_labels, non_critical_labels])
    elif len(critical_points):
        safety_points = np.asarray(critical_points, dtype=float)
        safety_labels = np.asarray(critical_labels, dtype=int)
    else:
        safety_points = np.asarray(non_critical_points, dtype=float)
        safety_labels = np.asarray(non_critical_labels, dtype=int)

    critical_counts = {
        str(int(label)): int(np.sum(critical_labels == int(label)))
        for label in CRITICAL_SAFETY_LABELS
    }
    meta = {
        "critical_safety_point_count": int(len(critical_points)),
        "non_critical_safety_voxel_count": int(len(non_critical_points)),
        "critical_safety_label_counts": critical_counts,
        "safety_sampling": "critical_labels_preserved_non_critical_downsampled",
    }
    return safety_points, safety_labels, meta


def _nearest_distances(query_points: np.ndarray, reference_points: np.ndarray, chunk_size: int = 256) -> np.ndarray:
    """Return nearest-neighbor distances using chunked NumPy broadcasting."""
    query_points = np.asarray(query_points, dtype=float)
    reference_points = np.asarray(reference_points, dtype=float)
    if len(query_points) == 0:
        return np.empty((0,), dtype=float)
    if len(reference_points) == 0:
        return np.full((len(query_points),), np.inf, dtype=float)

    distances = np.empty((len(query_points),), dtype=float)
    for start in range(0, len(query_points), chunk_size):
        chunk = query_points[start : start + chunk_size]
        deltas = chunk[:, None, :] - reference_points[None, :, :]
        distances[start : start + len(chunk)] = np.sqrt(np.min(np.sum(deltas * deltas, axis=2), axis=1))
    return distances


def safe_normalize(vec: Sequence[float], fallback: Optional[Sequence[float]] = None) -> np.ndarray:
    """Normalize with safety fallback — never raise on zero-length."""
    arr = np.asarray(vec, dtype=float)
    norm = float(np.linalg.norm(arr))
    if norm > 1e-9:
        return arr / norm
    if fallback is not None:
        fb = np.asarray(fallback, dtype=float)
        fb_norm = float(np.linalg.norm(fb))
        if fb_norm > 1e-9:
            return fb / fb_norm
    return np.array([1.0, 0.0, 0.0], dtype=float)


def pca_main_direction(points: np.ndarray, fallback: Optional[Sequence[float]] = None) -> np.ndarray:
    """Return PCA principal direction from a set of points. Falls back gracefully."""
    pts = np.asarray(points, dtype=float)
    if len(pts) < 2:
        if fallback is not None:
            return safe_normalize(fallback)
        return np.array([1.0, 0.0, 0.0], dtype=float)
    try:
        centered = pts - np.mean(pts, axis=0)
        cov = np.cov(centered.T)
        eigenvalues, eigenvectors = np.linalg.eigh(cov)
        main_axis = eigenvectors[:, np.argmax(eigenvalues)]
        return safe_normalize(main_axis, fallback)
    except Exception:
        if fallback is not None:
            return safe_normalize(fallback)
        return np.array([1.0, 0.0, 0.0], dtype=float)


def point_to_segment_distance(point: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    """Minimum distance from point to line segment ab."""
    ab = b - a
    ab_len_sq = float(np.dot(ab, ab))
    if ab_len_sq < 1e-12:
        return float(np.linalg.norm(point - a))
    t = max(0.0, min(1.0, float(np.dot(point - a, ab)) / ab_len_sq))
    return float(np.linalg.norm(point - (a + t * ab)))


def point_to_polyline_distance(point: np.ndarray, polyline: Sequence[Sequence[float]]) -> float:
    """Minimum distance from point to a polyline (sequence of 3D points)."""
    pts = [np.asarray(p, dtype=float) for p in polyline]
    if len(pts) < 2:
        return float(np.linalg.norm(point - pts[0])) if pts else float("inf")
    best = float("inf")
    for i in range(len(pts) - 1):
        best = min(best, point_to_segment_distance(point, pts[i], pts[i + 1]))
    return best


def is_point_safe_to_wire_curves(
    point: Sequence[float],
    wire_curves: Sequence[Dict[str, object]],
    safety_radius: Optional[float] = None,
) -> Tuple[bool, float]:
    """Check if a point maintains safety_radius from all wire_curves."""
    pos = np.asarray(point, dtype=float)
    min_dist = float("inf")
    for curve in wire_curves or []:
        radius = safety_radius
        if radius is None:
            radius = float(curve.get("safety_radius", DEFAULT_LIMITS["safety_distance_m"]))
        polyline = curve.get("polyline", [])
        if not polyline or len(polyline) < 1:
            continue
        dist = point_to_polyline_distance(pos, polyline)
        min_dist = min(min_dist, dist)
        if dist < float(radius):
            return False, dist
    return True, min_dist


def ray_blocked_by_wire_curves(
    start: Sequence[float],
    end: Sequence[float],
    wire_curves: Sequence[Dict[str, object]],
    occ_radius: Optional[float] = None,
) -> bool:
    """Check if line-of-sight is blocked by wire_curves (visual occlusion only)."""
    s = np.asarray(start, dtype=float)
    e = np.asarray(end, dtype=float)
    samples = 21
    for curve in wire_curves or []:
        radius = occ_radius
        if radius is None:
            radius = float(curve.get("occ_radius", POWER_MODEL_CONFIG["wire_occ_radius"]))
        polyline = curve.get("polyline", [])
        if not polyline or len(polyline) < 2:
            continue
        for i in range(samples):
            t = i / max(samples - 1, 1)
            point = s + (e - s) * t
            dist = point_to_polyline_distance(point, polyline)
            if dist < float(radius):
                return True
    return False


def check_view_angle(
    viewpoint_pos: Sequence[float],
    target_pos: Sequence[float],
    target_normal: Sequence[float],
    max_angle_deg: float,
) -> bool:
    """Return True when the viewing angle satisfies the max-angle constraint."""
    view_dir = safe_normalize(
        np.asarray(target_pos, dtype=float) - np.asarray(viewpoint_pos, dtype=float)
    )
    normal = safe_normalize(target_normal, fallback=[0.0, 0.0, 1.0])
    cos_angle = float(np.dot(view_dir, normal))
    angle_deg = math.degrees(math.acos(max(-1.0, min(1.0, cos_angle))))
    return angle_deg <= float(max_angle_deg) + 1e-6


def compute_weighted_coverage(
    visibility_matrix: np.ndarray,
    target_patches: Sequence[Dict[str, object]],
) -> Dict[str, float]:
    """Compute weighted coverage from a boolean visibility matrix (views x targets)."""
    if visibility_matrix.size == 0:
        return {"weighted_coverage": 0.0, "geometric_coverage": 0.0}
    weights = np.array([float(p.get("weight", 1.0)) for p in target_patches], dtype=float)
    areas = np.array([float(p.get("area", 1.0)) for p in target_patches], dtype=float)
    covered = np.any(visibility_matrix > 0, axis=0).astype(float)
    weighted_num = float(np.sum(covered * weights * areas))
    weighted_den = float(np.sum(weights * areas))
    return {
        "weighted_coverage": weighted_num / max(weighted_den, 1e-9),
        "geometric_coverage": float(np.mean(covered)),
    }


def estimate_patch_area(
    points_or_count: object,
    voxel_size: float,
    semantic: str = "tower_body",
) -> float:
    """Estimate patch area from point count or coordinate array."""
    if isinstance(points_or_count, (int, float, np.integer, np.floating)):
        count = max(int(points_or_count), 1)
        return float(count) * float(voxel_size) * float(voxel_size)
    pts = np.asarray(points_or_count, dtype=float).reshape((-1, 3))
    if len(pts) < 2:
        return float(voxel_size) * float(voxel_size)
    if len(pts) == 2:
        return float(np.linalg.norm(pts[1] - pts[0])) * float(voxel_size)
    try:
        centered = pts - np.mean(pts, axis=0)
        cov = np.cov(centered.T)
        eigenvalues, _ = np.linalg.eigh(cov)
        positive_vals = eigenvalues[eigenvalues > 0]
        if len(positive_vals) >= 2:
            return float(np.sqrt(positive_vals[-1] * positive_vals[-2])) * 1.5
        return float(np.sqrt(max(positive_vals[-1], 0.01) * 0.01)) * 1.5 if len(positive_vals) >= 1 else float(voxel_size * voxel_size)
    except Exception:
        return float(voxel_size) * float(voxel_size) * max(len(pts), 1)


def serialize_vector(vec: object) -> List[float]:
    """Serialize any vector-like object to a JSON-safe list."""
    if vec is None:
        return [0.0, 0.0, 0.0]
    return [round(float(v), 6) for v in np.asarray(vec, dtype=float).ravel()[:3]]


def serialize_wire_curves(curves: Sequence[Dict[str, object]]) -> List[Dict[str, object]]:
    """Serialize wire_curves for JSON embedding in meta_json."""
    result = []
    for curve in curves or []:
        result.append({
            "id": int(curve.get("id", len(result))),
            "type": str(curve.get("type", "conductor")),
            "polyline": [serialize_vector(p) for p in curve.get("polyline", [])],
            "occ_radius": float(curve.get("occ_radius", POWER_MODEL_CONFIG["wire_occ_radius"])),
            "safety_radius": float(curve.get("safety_radius", DEFAULT_LIMITS["safety_distance_m"])),
            "point_count": int(curve.get("point_count", 0)),
            "source": str(curve.get("source", "point_cloud_pca_polyline")),
            "usage": list(curve.get("usage", ["auxiliary_occlusion_geometry", "fine_distance_geometry"])),
        })
    return result


def serialize_no_fly_limits(limits: Dict[str, object]) -> Dict[str, object]:
    """Extract no-fly-relevant values from a config dict."""
    return {
        "safety_distance_m": float(limits.get("safety_distance_m", DEFAULT_LIMITS["safety_distance_m"])),
        "conductor_no_fly_exterior_clearance_m": float(limits.get(
            "conductor_no_fly_exterior_clearance_m",
            DEFAULT_LIMITS["conductor_no_fly_exterior_clearance_m"],
        )),
        "tower_safety_radius": float(limits.get("tower_safety_radius", POWER_MODEL_CONFIG["tower_safety_radius"])),
    }


def classify_manual_aim_type(aim_type: object) -> Tuple[str, float]:
    """Map a manual-route AimType into planner focus semantic and priority."""
    text = _decode_text(aim_type, "").strip()
    if not text:
        return "tower_body", 0.35
    if any(token in text for token in ("起始", "结束")):
        return "tower_body", 0.0
    if "辅助" in text:
        return "tower_body", 0.15
    if "导线端挂点" in text:
        return "conductor_insulator_connection", 1.0
    if "地线挂点" in text:
        return "ground_wire_tower_connection", 0.95
    if "横担端挂点" in text:
        return "insulator_tower_side_connection", 0.92
    if "绝缘子" in text:
        return "insulator", 0.9
    if "挂点" in text:
        return "tower_edge", 0.82
    if "塔头" in text:
        return "tower_top", 0.78
    if "塔身" in text:
        return "tower_body", 0.55
    if "塔全貌" in text:
        return "tower_body", 0.45
    return "tower_body", 0.35


class VoxelSafetyIndex:
    """Small grid index for nearest distance checks without extra dependencies."""

    def __init__(self, points: np.ndarray, cell_size: float):
        self.points = np.asarray(points, dtype=float)
        self.cell_size = max(float(cell_size), 0.5)
        self.cells: Dict[Tuple[int, int, int], List[int]] = {}
        if len(self.points) == 0:
            self.origin = np.zeros(3, dtype=float)
            return
        self.origin = np.min(self.points, axis=0)
        keys = np.floor((self.points - self.origin) / self.cell_size).astype(int)
        for index, key in enumerate(keys):
            self.cells.setdefault((int(key[0]), int(key[1]), int(key[2])), []).append(index)

    def _cell_key(self, position: Sequence[float]) -> Tuple[int, int, int]:
        key = np.floor((np.asarray(position, dtype=float) - self.origin) / self.cell_size).astype(int)
        return int(key[0]), int(key[1]), int(key[2])

    def min_distance(self, position: Sequence[float], search_radius_m: float) -> float:
        """Return the closest indexed point distance around a position."""
        if len(self.points) == 0:
            return float("inf")
        cx, cy, cz = self._cell_key(position)
        radius_cells = max(1, int(math.ceil(float(search_radius_m) / self.cell_size)))
        candidate_indices: List[int] = []
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                for dz in range(-radius_cells, radius_cells + 1):
                    candidate_indices.extend(self.cells.get((cx + dx, cy + dy, cz + dz), []))
        if not candidate_indices:
            return float("inf")
        refs = self.points[np.asarray(candidate_indices, dtype=int)]
        deltas = refs - np.asarray(position, dtype=float)
        return float(np.sqrt(np.min(np.sum(deltas * deltas, axis=1))))


@dataclass
class ConductorNoFlyVolume:
    """Oriented cuboid-like no-fly volume between conductor and ground-wire planes."""

    source: str
    origin: np.ndarray
    u_axis: np.ndarray
    v_axis: np.ndarray
    u_min: float
    u_max: float
    v_min: float
    v_max: float
    bottom_slope: float
    bottom_intercept: float
    top_slope: float
    top_intercept: float
    tolerance_m: float = DEFAULT_LIMITS["conductor_no_fly_boundary_tolerance_m"]
    z_min: Optional[float] = None
    z_max: Optional[float] = None
    left_side_slope: Optional[float] = None
    left_side_intercept: Optional[float] = None
    right_side_slope: Optional[float] = None
    right_side_intercept: Optional[float] = None
    left_z_min: Optional[float] = None
    left_z_max: Optional[float] = None
    right_z_min: Optional[float] = None
    right_z_max: Optional[float] = None
    top_margin_m: float = 0.0
    bottom_margin_m: float = 0.0

    @classmethod
    def from_record(cls, record: Mapping[str, Any] | Dict[str, Any]) -> "ConductorNoFlyVolume":
        """Build a volume from the dict representation stored in voxel files."""
        return cls(
            source=_decode_text(record.get("source"), "unknown"),
            origin=np.asarray(record.get("origin", [0.0, 0.0, 0.0]), dtype=float),
            u_axis=normalize_vector(record.get("u_axis", [1.0, 0.0, 0.0])),
            v_axis=normalize_vector(record.get("v_axis", [0.0, 1.0, 0.0])),
            u_min=float(record.get("u_min", 0.0)),
            u_max=float(record.get("u_max", 0.0)),
            v_min=float(record.get("v_min", 0.0)),
            v_max=float(record.get("v_max", 0.0)),
            bottom_slope=float(record.get("bottom_slope", 0.0)),
            bottom_intercept=float(record.get("bottom_intercept", 0.0)),
            top_slope=float(record.get("top_slope", 0.0)),
            top_intercept=float(record.get("top_intercept", 0.0)),
            tolerance_m=float(record.get("tolerance_m", DEFAULT_LIMITS["conductor_no_fly_boundary_tolerance_m"])),
            z_min=float(record["z_min"]) if record.get("z_min") is not None else None,
            z_max=float(record["z_max"]) if record.get("z_max") is not None else None,
            left_side_slope=(
                float(record["left_side_slope"])
                if record.get("left_side_slope") is not None
                else None
            ),
            left_side_intercept=(
                float(record["left_side_intercept"])
                if record.get("left_side_intercept") is not None
                else None
            ),
            right_side_slope=(
                float(record["right_side_slope"])
                if record.get("right_side_slope") is not None
                else None
            ),
            right_side_intercept=(
                float(record["right_side_intercept"])
                if record.get("right_side_intercept") is not None
                else None
            ),
            left_z_min=float(record["left_z_min"]) if record.get("left_z_min") is not None else None,
            left_z_max=float(record["left_z_max"]) if record.get("left_z_max") is not None else None,
            right_z_min=float(record["right_z_min"]) if record.get("right_z_min") is not None else None,
            right_z_max=float(record["right_z_max"]) if record.get("right_z_max") is not None else None,
            top_margin_m=float(record.get("top_margin_m", 0.0) or 0.0),
            bottom_margin_m=float(record.get("bottom_margin_m", 0.0) or 0.0),
        )

    def to_record(self) -> Dict[str, object]:
        """Return a JSON/NumPy-serializable no-fly volume record."""
        base_z_min = (
            float(self.z_min)
            if self.z_min is not None
            else float(min(self.bottom_z(self.v_min), self.bottom_z(self.v_max)))
        )
        base_z_max = (
            float(self.z_max)
            if self.z_max is not None
            else float(max(self.top_z(self.v_min), self.top_z(self.v_max)))
        )
        return {
            "source": self.source,
            "origin": [round(float(value), 6) for value in self.origin.tolist()],
            "u_axis": [round(float(value), 8) for value in self.u_axis.tolist()],
            "v_axis": [round(float(value), 8) for value in self.v_axis.tolist()],
            "u_min": round(float(self.u_min), 6),
            "u_max": round(float(self.u_max), 6),
            "v_min": round(float(self.v_min), 6),
            "v_max": round(float(self.v_max), 6),
            "bottom_slope": round(float(self.bottom_slope), 8),
            "bottom_intercept": round(float(self.bottom_intercept), 6),
            "top_slope": round(float(self.top_slope), 8),
            "top_intercept": round(float(self.top_intercept), 6),
            "tolerance_m": round(float(self.tolerance_m), 6),
            "z_min": round(base_z_min, 6),
            "z_max": round(base_z_max, 6),
            "left_side_slope": None if self.left_side_slope is None else round(float(self.left_side_slope), 8),
            "left_side_intercept": None if self.left_side_intercept is None else round(float(self.left_side_intercept), 6),
            "right_side_slope": None if self.right_side_slope is None else round(float(self.right_side_slope), 8),
            "right_side_intercept": None if self.right_side_intercept is None else round(float(self.right_side_intercept), 6),
            "left_z_min": None if self.left_z_min is None else round(float(self.left_z_min), 6),
            "left_z_max": None if self.left_z_max is None else round(float(self.left_z_max), 6),
            "right_z_min": None if self.right_z_min is None else round(float(self.right_z_min), 6),
            "right_z_max": None if self.right_z_max is None else round(float(self.right_z_max), 6),
            "top_margin_m": round(float(self.top_margin_m), 6),
            "bottom_margin_m": round(float(self.bottom_margin_m), 6),
        }

    def local_coordinates(self, position: Sequence[float]) -> Tuple[float, float, float]:
        """Project a world position into the volume's line/lateral/height coordinates."""
        point = np.asarray(position, dtype=float)
        rel = point - self.origin
        return (
            float(np.dot(rel, self.u_axis)),
            float(np.dot(rel, self.v_axis)),
            float(point[2]),
        )

    def world_position(self, u_value: float, v_value: float, z_value: float) -> np.ndarray:
        """Return a world coordinate from local line/lateral/height coordinates."""
        base = self.origin + float(u_value) * self.u_axis + float(v_value) * self.v_axis
        return np.array([float(base[0]), float(base[1]), float(z_value)], dtype=float)

    def bottom_z(self, v_value: float) -> float:
        """Return the lower plane height at a lateral coordinate."""
        return float(self.bottom_slope * float(v_value) + self.bottom_intercept)

    def top_z(self, v_value: float) -> float:
        """Return the upper plane height at a lateral coordinate."""
        return float(self.top_slope * float(v_value) + self.top_intercept)

    def bottom_limit_z(self, v_value: float) -> float:
        """Return the expanded lower no-fly height at a lateral coordinate."""
        return float(self.bottom_z(v_value) - max(float(self.bottom_margin_m), 0.0))

    def top_limit_z(self, v_value: float) -> float:
        """Return the expanded upper no-fly height at a lateral coordinate."""
        return float(self.top_z(v_value) + max(float(self.top_margin_m), 0.0))

    def lower_z(self) -> float:
        """Return the expanded lower height."""
        if self.z_min is not None:
            return float(self.z_min) - max(float(self.bottom_margin_m), 0.0)
        return float(min(self.bottom_z(self.v_min), self.bottom_z(self.v_max)) - max(float(self.bottom_margin_m), 0.0))

    def upper_z(self) -> float:
        """Return the expanded upper height."""
        if self.z_max is not None:
            return float(self.z_max) + max(float(self.top_margin_m), 0.0)
        return float(max(self.top_z(self.v_min), self.top_z(self.v_max)) + max(float(self.top_margin_m), 0.0))

    def left_v(self, z_value: float) -> float:
        """Return the left side plane's lateral boundary at a height."""
        if self.left_side_slope is None or self.left_side_intercept is None:
            return float(self.v_min)
        z_eval = float(z_value)
        if self.left_z_min is not None and self.left_z_max is not None:
            z_eval = min(max(z_eval, float(self.left_z_min)), float(self.left_z_max))
        return float(self.left_side_slope * z_eval + self.left_side_intercept)

    def right_v(self, z_value: float) -> float:
        """Return the right side plane's lateral boundary at a height."""
        if self.right_side_slope is None or self.right_side_intercept is None:
            return float(self.v_max)
        z_eval = float(z_value)
        if self.right_z_min is not None and self.right_z_max is not None:
            z_eval = min(max(z_eval, float(self.right_z_min)), float(self.right_z_max))
        return float(self.right_side_slope * z_eval + self.right_side_intercept)

    def side_bounds_at_z(self, z_value: float) -> Tuple[float, float]:
        """Return lateral no-fly bounds at a height."""
        left = self.left_v(z_value)
        right = self.right_v(z_value)
        if left <= right:
            return left, right
        return right, left

    def with_vertical_margins(self, top_margin_m: float, bottom_margin_m: float) -> "ConductorNoFlyVolume":
        """Return a copy whose vertical no-fly margins are at least the requested margins."""
        record = self.to_record()
        record["top_margin_m"] = max(float(record.get("top_margin_m", 0.0) or 0.0), float(top_margin_m))
        record["bottom_margin_m"] = max(float(record.get("bottom_margin_m", 0.0) or 0.0), float(bottom_margin_m))
        return ConductorNoFlyVolume.from_record(record)

    def with_exact_vertical_margins(self, top_margin_m: float, bottom_margin_m: float) -> "ConductorNoFlyVolume":
        """Return a copy using the requested vertical no-fly margins exactly."""
        record = self.to_record()
        record["top_margin_m"] = max(float(top_margin_m), 0.0)
        record["bottom_margin_m"] = max(float(bottom_margin_m), 0.0)
        return ConductorNoFlyVolume.from_record(record)

    def contains(self, position: Sequence[float], tolerance_m: Optional[float] = None) -> bool:
        """Return True when a waypoint lies inside the no-fly volume."""
        tol = self.tolerance_m if tolerance_m is None else float(tolerance_m)
        u_value, v_value, z_value = self.local_coordinates(position)
        if u_value < self.u_min - tol or u_value > self.u_max + tol:
            return False
        if z_value < self.lower_z() - tol or z_value > self.upper_z() + tol:
            return False
        left_v, right_v = self.side_bounds_at_z(z_value)
        if v_value < left_v - tol or v_value > right_v + tol:
            return False
        return self.bottom_limit_z(v_value) - tol <= z_value <= self.top_limit_z(v_value) + tol

    def clearance(self, position: Sequence[float]) -> float:
        """Return an approximate clearance, negative when the point is inside."""
        u_value, v_value, z_value = self.local_coordinates(position)
        left_v, right_v = self.side_bounds_at_z(z_value)
        clamped_v = min(max(v_value, left_v), right_v)
        bottom = self.bottom_limit_z(clamped_v)
        top = self.top_limit_z(clamped_v)
        outside_u = max(self.u_min - u_value, 0.0, u_value - self.u_max)
        outside_v = max(left_v - v_value, 0.0, v_value - right_v)
        outside_z = max(bottom - z_value, 0.0, z_value - top)
        if outside_u > 0.0 or outside_v > 0.0 or outside_z > 0.0:
            return float(math.sqrt(outside_u * outside_u + outside_v * outside_v + outside_z * outside_z))
        return -min(
            u_value - self.u_min,
            self.u_max - u_value,
            v_value - left_v,
            right_v - v_value,
            z_value - self.bottom_limit_z(v_value),
            self.top_limit_z(v_value) - z_value,
        )

    def segment_intersects(self, start: Sequence[float], end: Sequence[float], tolerance_m: Optional[float] = None) -> bool:
        """Return True when a segment intersects the oriented no-fly prism."""
        start_arr = np.asarray(start, dtype=float)
        end_arr = np.asarray(end, dtype=float)
        sample_count = 65
        for step in range(sample_count):
            t_value = step / max(sample_count - 1, 1)
            point = start_arr + (end_arr - start_arr) * t_value
            if self.contains(point, tolerance_m=tolerance_m):
                return True
        return False


def load_conductor_no_fly_volumes(raw_records: Optional[Iterable[object]]) -> List[ConductorNoFlyVolume]:
    """Load conductor no-fly volumes from object arrays or plain dict records."""
    volumes: List[ConductorNoFlyVolume] = []
    if raw_records is None:
        return volumes
    for raw in raw_records:
        if isinstance(raw, ConductorNoFlyVolume):
            volumes.append(raw)
            continue
        if isinstance(raw, dict):
            record = raw
        elif hasattr(raw, "item"):
            item = raw.item()
            record = item if isinstance(item, dict) else {}
        else:
            names = getattr(getattr(raw, "dtype", None), "names", None) or ()
            record = {name: raw[name].item() if hasattr(raw[name], "item") else raw[name] for name in names}
        if record:
            volumes.append(ConductorNoFlyVolume.from_record(record))
    return volumes


def _load_json_with_fallback(json_path: str | Path) -> Dict[str, Any]:
    """Read route JSON using the encodings found in exported flight routes."""
    raw = Path(json_path).read_bytes()
    last_error: Optional[Exception] = None
    for encoding in ("utf-8-sig", "utf-8", "gb18030", "gbk"):
        try:
            data = json.loads(raw.decode(encoding))
            if isinstance(data, dict):
                return data
        except Exception as exc:
            last_error = exc
    raise ValueError(f"无法解析人工航线 JSON: {last_error}")


def _parse_coord3(value: object) -> Optional[np.ndarray]:
    """Parse a comma-separated or sequence coordinate into a 3D NumPy vector."""
    if value is None:
        return None
    try:
        parts = [part.strip() for part in str(value).split(",")] if isinstance(value, str) else list(value)
        if len(parts) < 3:
            return None
        return np.asarray([float(parts[0]), float(parts[1]), float(parts[2])], dtype=float)
    except Exception:
        return None


def _horizontal_axis_from_angle(angle: Optional[float]) -> Optional[np.ndarray]:
    """Convert a route plane angle into a horizontal unit vector when available."""
    if angle is None or not math.isfinite(float(angle)):
        return None
    return normalize_vector([math.cos(float(angle)), math.sin(float(angle)), 0.0])


def _principal_horizontal_axis(points: np.ndarray) -> Optional[np.ndarray]:
    """Infer the dominant horizontal direction from point samples."""
    points = np.asarray(points, dtype=float)
    if len(points) < 2:
        return None
    xy = points[:, :2]
    centered = xy - np.mean(xy, axis=0)
    if float(np.max(np.linalg.norm(centered, axis=1))) <= 1e-6:
        return None
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    return normalize_vector([float(vh[0, 0]), float(vh[0, 1]), 0.0])


def _manual_line_vectors(records: Sequence[Dict[str, object]]) -> List[np.ndarray]:
    """Infer line-direction hints from manual-route small/large-side and paired conductor points."""
    vectors: List[np.ndarray] = []
    small_points = [
        np.asarray(record["coord"], dtype=float)
        for record in records
        if "小号侧" in _decode_text(record.get("aim_type"), "")
    ]
    large_points = [
        np.asarray(record["coord"], dtype=float)
        for record in records
        if "大号侧" in _decode_text(record.get("aim_type"), "")
    ]
    if small_points and large_points:
        vectors.append(np.mean(large_points, axis=0) - np.mean(small_points, axis=0))

    grouped: Dict[str, List[np.ndarray]] = {}
    for record in records:
        text = _decode_text(record.get("aim_type"), "")
        if "导线端挂点" not in text:
            continue
        base = text.rstrip("0123456789")
        grouped.setdefault(base, []).append(np.asarray(record["coord"], dtype=float))
    for points in grouped.values():
        if len(points) < 2:
            continue
        best_pair: Optional[Tuple[np.ndarray, np.ndarray]] = None
        best_distance = 0.0
        for index, first in enumerate(points):
            for second in points[index + 1 :]:
                distance = float(np.linalg.norm((second - first)[:2]))
                if distance > best_distance:
                    best_distance = distance
                    best_pair = (first, second)
        if best_pair and best_distance > 0.5:
            vectors.append(best_pair[1] - best_pair[0])
    return vectors


def _select_line_axis(
    reference_points: np.ndarray,
    preferred_vectors: Optional[Sequence[np.ndarray]] = None,
    plane_angle: Optional[float] = None,
) -> np.ndarray:
    """Choose the line axis from route hints, plane angle, or point-cloud PCA."""
    vectors = []
    for vector in preferred_vectors or []:
        arr = np.asarray(vector, dtype=float)
        arr[2] = 0.0
        if float(np.linalg.norm(arr[:2])) > 0.5:
            vectors.append(normalize_vector(arr))
    if vectors:
        base = vectors[0]
        aligned = [vector if float(np.dot(vector, base)) >= 0.0 else -vector for vector in vectors]
        return normalize_vector(np.mean(aligned, axis=0))
    angle_axis = _horizontal_axis_from_angle(plane_angle)
    if angle_axis is not None:
        return angle_axis
    pca_axis = _principal_horizontal_axis(reference_points)
    if pca_axis is not None:
        return pca_axis
    return np.array([1.0, 0.0, 0.0], dtype=float)


def _side_anchor(
    points: np.ndarray,
    v_values: np.ndarray,
    side_mid: float,
    side: str,
    mode: str,
) -> Optional[Tuple[float, float]]:
    """Return a lateral/height anchor for one side of a no-fly volume."""
    if len(points) == 0:
        return None
    mask = v_values <= side_mid if side == "left" else v_values > side_mid
    if not np.any(mask):
        index = int(np.argmin(v_values) if side == "left" else np.argmax(v_values))
        mask = np.zeros(len(points), dtype=bool)
        mask[index] = True
    side_points = points[mask]
    side_v = v_values[mask]
    if mode == "top":
        selected_index = int(np.argmax(side_points[:, 2]))
    else:
        selected_index = int(np.argmin(side_points[:, 2]))
    return float(side_v[selected_index]), float(side_points[selected_index, 2])


def _line_plane_coefficients(left: Tuple[float, float], right: Tuple[float, float]) -> Tuple[float, float]:
    """Return z=a*v+b for a plane parallel to the line axis."""
    left_v, left_z = left
    right_v, right_z = right
    if abs(right_v - left_v) <= 1e-6:
        return 0.0, float((left_z + right_z) / 2.0)
    slope = float((right_z - left_z) / (right_v - left_v))
    return slope, float(left_z - slope * left_v)


def _side_plane_coefficients(bottom: Tuple[float, float], top: Tuple[float, float]) -> Tuple[float, float, float, float]:
    """Return v=a*z+b and the source height span for one side plane."""
    bottom_v, bottom_z = bottom
    top_v, top_z = top
    z_min = float(min(bottom_z, top_z))
    z_max = float(max(bottom_z, top_z))
    if abs(top_z - bottom_z) <= 1e-6:
        return 0.0, float((bottom_v + top_v) / 2.0), z_min, z_max
    slope = float((top_v - bottom_v) / (top_z - bottom_z))
    return slope, float(bottom_v - slope * bottom_z), z_min, z_max


def _branch_line_axis(points: np.ndarray, center: Sequence[float], fallback: Sequence[float]) -> np.ndarray:
    """Infer a branch axis oriented away from the tower center."""
    points = np.asarray(points, dtype=float).reshape((-1, 3))
    center_arr = np.asarray(center, dtype=float)
    fallback_axis = normalize_vector([float(fallback[0]), float(fallback[1]), 0.0])
    if len(points) < 2:
        return fallback_axis
    centroid_vec = np.mean(points, axis=0) - center_arr
    centroid_vec[2] = 0.0
    if float(np.linalg.norm(centroid_vec[:2])) <= 1e-6:
        centroid_vec = fallback_axis.copy()
    pca_axis = _principal_horizontal_axis(points)
    if pca_axis is not None:
        centroid_norm = float(np.linalg.norm(centroid_vec[:2]))
        alignment = abs(float(np.dot(pca_axis[:2], centroid_vec[:2]) / max(centroid_norm, 1e-6)))
        if alignment < 0.5:
            return normalize_vector(centroid_vec, fallback_axis)
        if float(np.dot(pca_axis[:2], centroid_vec[:2])) < 0.0:
            pca_axis = -pca_axis
        return normalize_vector(pca_axis)
    return normalize_vector(centroid_vec, fallback_axis)


def _split_conductor_branches(
    center: Sequence[float],
    conductor_points: np.ndarray,
    ground_wire_points: np.ndarray,
    fallback_axis: Optional[Sequence[float]] = None,
) -> List[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """Split wire/ground-wire samples into two tower-side line branches."""
    conductor_points = np.asarray(conductor_points, dtype=float).reshape((-1, 3))
    ground_wire_points = np.asarray(ground_wire_points, dtype=float).reshape((-1, 3))
    if len(conductor_points) < 4 or len(ground_wire_points) < 4:
        return []

    center_arr = np.asarray(center, dtype=float)
    reference_points = np.vstack([conductor_points, ground_wire_points])
    deltas = reference_points[:, :2] - center_arr[:2]
    radii = np.linalg.norm(deltas, axis=1)
    valid = radii > 1.0
    if int(np.sum(valid)) < 4:
        return []

    def build_branches(full_labels: np.ndarray, fallback_centers: np.ndarray) -> List[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
        conductor_count = len(conductor_points)
        conductor_labels = full_labels[:conductor_count]
        ground_labels = full_labels[conductor_count:]
        branches: List[Tuple[np.ndarray, np.ndarray, np.ndarray]] = []
        for cluster_index in (0, 1):
            branch_conductors = conductor_points[conductor_labels == cluster_index]
            branch_ground = ground_wire_points[ground_labels == cluster_index]
            if len(branch_conductors) < 2 or len(branch_ground) < 2:
                return []
            branch_refs = np.vstack([branch_conductors, branch_ground])
            fallback = np.asarray(
                [fallback_centers[cluster_index, 0], fallback_centers[cluster_index, 1], 0.0],
                dtype=float,
            )
            if float(np.linalg.norm(fallback[:2])) <= 1e-6 and fallback_axis is not None:
                fallback = np.asarray(fallback_axis, dtype=float)
            branch_axis = _branch_line_axis(branch_refs, center_arr, fallback)
            branches.append((branch_axis, branch_conductors, branch_ground))
        if len(branches) != 2:
            return []
        axis_dot = float(np.dot(branches[0][0][:2], branches[1][0][:2]))
        if axis_dot > math.cos(math.radians(45.0)):
            return []
        branches.sort(key=lambda item: math.atan2(float(item[0][1]), float(item[0][0])))
        return branches

    pca_axis = _principal_horizontal_axis(reference_points)
    if pca_axis is not None:
        projections = np.dot(reference_points - center_arr, pca_axis)
        if float(np.max(projections)) > 1.0 and float(np.min(projections)) < -1.0:
            full_labels = np.where(projections >= 0.0, 0, 1).astype(int)
            fallback_centers = np.vstack([pca_axis[:2], -pca_axis[:2]])
            branches = build_branches(full_labels, fallback_centers)
            if branches:
                return branches

    units = deltas[valid] / radii[valid, None]
    best_pair: Optional[Tuple[int, int]] = None
    best_dot = 1.0
    for first_index in range(len(units)):
        for second_index in range(first_index + 1, len(units)):
            dot = float(np.dot(units[first_index], units[second_index]))
            if dot < best_dot:
                best_dot = dot
                best_pair = (first_index, second_index)
    if best_pair is None or best_dot > math.cos(math.radians(60.0)):
        return []

    centers = np.vstack([units[best_pair[0]], units[best_pair[1]]])
    labels = np.zeros(len(units), dtype=int)
    for _ in range(10):
        scores = units @ centers.T
        new_labels = np.argmax(scores, axis=1)
        if len(set(int(value) for value in new_labels)) < 2:
            return []
        next_centers = centers.copy()
        for cluster_index in (0, 1):
            cluster_units = units[new_labels == cluster_index]
            mean = np.mean(cluster_units, axis=0)
            norm = float(np.linalg.norm(mean))
            if norm > 1e-6:
                next_centers[cluster_index] = mean / norm
        labels = new_labels
        if np.allclose(next_centers, centers, atol=1e-4):
            break
        centers = next_centers

    full_labels = np.full(len(reference_points), -1, dtype=int)
    full_valid_indices = np.flatnonzero(valid)
    full_labels[full_valid_indices] = labels
    invalid_indices = np.flatnonzero(~valid)
    for index in invalid_indices:
        delta = deltas[index]
        norm = float(np.linalg.norm(delta))
        if norm <= 1e-6:
            full_labels[index] = 0
            continue
        unit = delta / norm
        full_labels[index] = int(np.argmax(unit @ centers.T))
    return build_branches(full_labels, centers)


def _build_conductor_no_fly_volume(
    center: Sequence[float],
    line_axis: Sequence[float],
    conductor_points: np.ndarray,
    ground_wire_points: np.ndarray,
    reference_points: np.ndarray,
    source: str,
    plane_len: Optional[float] = None,
    extent_margin_m: float = DEFAULT_LIMITS["conductor_no_fly_extent_margin_m"],
    min_length_m: float = DEFAULT_LIMITS["conductor_no_fly_min_length_m"],
    tolerance_m: float = DEFAULT_LIMITS["conductor_no_fly_boundary_tolerance_m"],
    top_margin_m: float = 0.0,
    bottom_margin_m: float = 0.0,
    one_sided: bool = False,
) -> Optional[ConductorNoFlyVolume]:
    """Build one conductor no-fly volume from conductor and ground-wire boundary points."""
    conductor_points = np.asarray(conductor_points, dtype=float).reshape((-1, 3))
    ground_wire_points = np.asarray(ground_wire_points, dtype=float).reshape((-1, 3))
    if len(conductor_points) < 2 or len(ground_wire_points) < 2:
        return None
    origin = np.asarray(center, dtype=float)
    if len(origin) < 3:
        return None
    u_axis = normalize_vector([float(line_axis[0]), float(line_axis[1]), 0.0])
    v_axis = normalize_vector([-float(u_axis[1]), float(u_axis[0]), 0.0])
    boundary_points = np.vstack([conductor_points, ground_wire_points])
    v_values = np.dot(boundary_points - origin, v_axis)
    if float(np.max(v_values) - np.min(v_values)) < 2.0:
        return None
    v_min = float(np.min(v_values) - tolerance_m)
    v_max = float(np.max(v_values) + tolerance_m)
    conductor_v = np.dot(conductor_points - origin, v_axis)
    ground_wire_v = np.dot(ground_wire_points - origin, v_axis)
    side_mid = float((np.min(v_values) + np.max(v_values)) / 2.0)
    bottom_left = _side_anchor(conductor_points, conductor_v, side_mid, "left", "bottom")
    bottom_right = _side_anchor(conductor_points, conductor_v, side_mid, "right", "bottom")
    top_left = _side_anchor(ground_wire_points, ground_wire_v, side_mid, "left", "top")
    top_right = _side_anchor(ground_wire_points, ground_wire_v, side_mid, "right", "top")
    if not bottom_left or not bottom_right or not top_left or not top_right:
        return None
    bottom_slope, bottom_intercept = _line_plane_coefficients(bottom_left, bottom_right)
    top_slope, top_intercept = _line_plane_coefficients(top_left, top_right)
    left_side_slope, left_side_intercept, left_z_min, left_z_max = _side_plane_coefficients(bottom_left, top_left)
    right_side_slope, right_side_intercept, right_z_min, right_z_max = _side_plane_coefficients(bottom_right, top_right)
    bottom_values = [bottom_slope * v_min + bottom_intercept, bottom_slope * v_max + bottom_intercept]
    top_values = [top_slope * v_min + top_intercept, top_slope * v_max + top_intercept]
    if min(top - bottom for top, bottom in zip(top_values, bottom_values)) <= 0.5:
        z_min = float(np.min(conductor_points[:, 2]))
        z_max = float(np.max(ground_wire_points[:, 2]))
        if z_max <= z_min + 0.5:
            z_min = float(np.min(boundary_points[:, 2]))
            z_max = float(np.max(boundary_points[:, 2]))
        if z_max <= z_min + 0.5:
            return None
        bottom_slope = 0.0
        bottom_intercept = z_min
        top_slope = 0.0
        top_intercept = z_max
    z_min = float(min(bottom_slope * v_min + bottom_intercept, bottom_slope * v_max + bottom_intercept))
    z_max = float(max(top_slope * v_min + top_intercept, top_slope * v_max + top_intercept))
    refs = np.asarray(reference_points, dtype=float).reshape((-1, 3)) if len(reference_points) else boundary_points
    u_values = np.dot(refs - origin, u_axis)
    plane_half_length = float(plane_len or 0.0) / 2.0
    if one_sided:
        point_forward_length = float(np.max(u_values)) if len(u_values) else 0.0
        forward_length = max(point_forward_length, plane_half_length, float(min_length_m) / 2.0) + float(extent_margin_m)
        rear_margin = max(float(tolerance_m), min(2.0, float(extent_margin_m) * 0.2))
        u_min = -rear_margin
        u_max = forward_length
    else:
        point_half_length = float(np.max(np.abs(u_values))) if len(u_values) else 0.0
        half_length = max(point_half_length, plane_half_length, float(min_length_m) / 2.0) + float(extent_margin_m)
        u_min = -half_length
        u_max = half_length
    return ConductorNoFlyVolume(
        source=source,
        origin=origin,
        u_axis=u_axis,
        v_axis=v_axis,
        u_min=u_min,
        u_max=u_max,
        v_min=v_min,
        v_max=v_max,
        bottom_slope=bottom_slope,
        bottom_intercept=bottom_intercept,
        top_slope=top_slope,
        top_intercept=top_intercept,
        tolerance_m=tolerance_m,
        z_min=z_min,
        z_max=z_max,
        left_side_slope=left_side_slope,
        left_side_intercept=left_side_intercept,
        right_side_slope=right_side_slope,
        right_side_intercept=right_side_intercept,
        left_z_min=left_z_min,
        left_z_max=left_z_max,
        right_z_min=right_z_min,
        right_z_max=right_z_max,
        top_margin_m=max(float(top_margin_m), 0.0),
        bottom_margin_m=max(float(bottom_margin_m), 0.0),
    )


def build_conductor_no_fly_volumes_from_point_cloud(
    center: Sequence[float],
    wire_points: np.ndarray,
    ground_wire_points: np.ndarray,
    source: str = "point_cloud",
    plane_len: Optional[float] = None,
    line_axis: Optional[Sequence[float]] = None,
    extent_margin_m: float = DEFAULT_LIMITS["conductor_no_fly_extent_margin_m"],
    min_length_m: float = DEFAULT_LIMITS["conductor_no_fly_min_length_m"],
    tolerance_m: float = DEFAULT_LIMITS["conductor_no_fly_boundary_tolerance_m"],
    top_margin_m: float = 0.0,
    bottom_margin_m: float = 0.0,
) -> List[Dict[str, object]]:
    """Build no-fly volume records from semantic wire and ground-wire point clouds."""
    wire_points = np.asarray(wire_points, dtype=float).reshape((-1, 3))
    ground_wire_points = np.asarray(ground_wire_points, dtype=float).reshape((-1, 3))
    if len(wire_points) < 2 or len(ground_wire_points) < 2:
        return []
    reference_points = np.vstack([wire_points, ground_wire_points])

    branch_records: List[Dict[str, object]] = []
    branches = _split_conductor_branches(center, wire_points, ground_wire_points, fallback_axis=line_axis)
    if len(branches) == 2:
        for branch_axis, branch_wire_points, branch_ground_wire_points in branches:
            branch_reference_points = np.vstack([branch_wire_points, branch_ground_wire_points])
            volume = _build_conductor_no_fly_volume(
                center=center,
                line_axis=branch_axis,
                conductor_points=branch_wire_points,
                ground_wire_points=branch_ground_wire_points,
                reference_points=branch_reference_points,
                source=source,
                plane_len=plane_len,
                extent_margin_m=extent_margin_m,
                min_length_m=min_length_m,
                tolerance_m=tolerance_m,
                top_margin_m=top_margin_m,
                bottom_margin_m=bottom_margin_m,
                one_sided=True,
            )
            if volume is not None:
                branch_records.append(volume.to_record())
        if len(branch_records) == 2:
            return branch_records

    selected_line_axis = normalize_vector(line_axis) if line_axis is not None else _select_line_axis(reference_points)
    volume = _build_conductor_no_fly_volume(
        center=center,
        line_axis=selected_line_axis,
        conductor_points=wire_points,
        ground_wire_points=ground_wire_points,
        reference_points=reference_points,
        source=source,
        plane_len=plane_len,
        extent_margin_m=extent_margin_m,
        min_length_m=min_length_m,
        tolerance_m=tolerance_m,
        top_margin_m=top_margin_m,
        bottom_margin_m=bottom_margin_m,
    )
    return [volume.to_record()] if volume else []


def _manual_boundary_records(manual_route_path: str | Path) -> Tuple[List[Dict[str, object]], Dict[str, Any]]:
    """Extract conductor and ground-wire target records from a manual route file."""
    raw = _load_json_with_fallback(manual_route_path)
    tower = (raw.get("towers") or [{}])[0] if isinstance(raw.get("towers"), list) else {}
    center = _parse_coord3(tower.get("PlaneCenterPoint"))
    plane_len = tower.get("PlaneLen")
    plane_angle = tower.get("PlaneAngle")
    meta = {
        "center": center,
        "plane_len": _safe_float(plane_len, 0.0) if plane_len is not None else None,
        "plane_angle": _safe_float(plane_angle, 0.0) if plane_angle is not None else None,
    }
    records: List[Dict[str, object]] = []
    for waypoint in parse_manual_route(str(manual_route_path)):
        text = _decode_text(waypoint.get("aim_type"), "")
        if any(token in text for token in ("辅助", "通道", "防鸟")):
            continue
        target = waypoint.get("target_utm")
        if target is None:
            continue
        coord = np.asarray(target, dtype=float)
        if len(coord) < 3 or not np.all(np.isfinite(coord[:3])):
            continue
        if "地线" in text:
            records.append({"kind": "ground_wire", "coord": coord[:3], "aim_type": text})
        elif "导线" in text and "挂点" in text:
            records.append({"kind": "conductor", "coord": coord[:3], "aim_type": text})
    return records, meta


def build_conductor_no_fly_volumes_from_manual_route(
    surface_model: Dict[str, object],
    manual_route_path: str | Path,
    extent_margin_m: float = DEFAULT_LIMITS["conductor_no_fly_extent_margin_m"],
    min_length_m: float = DEFAULT_LIMITS["conductor_no_fly_min_length_m"],
    tolerance_m: float = DEFAULT_LIMITS["conductor_no_fly_boundary_tolerance_m"],
    top_margin_m: float = 0.0,
    bottom_margin_m: float = 0.0,
) -> List[Dict[str, object]]:
    """Build no-fly volume records from manual-route conductor and ground-wire targets."""
    try:
        records, meta = _manual_boundary_records(manual_route_path)
    except Exception:
        return []
    conductor_points = np.asarray([record["coord"] for record in records if record["kind"] == "conductor"], dtype=float)
    ground_wire_points = np.asarray([record["coord"] for record in records if record["kind"] == "ground_wire"], dtype=float)
    if len(conductor_points) < 2 or len(ground_wire_points) < 2:
        return []
    center = meta.get("center")
    if center is None:
        center = np.asarray(surface_model.get("local_center", np.mean(np.vstack([conductor_points, ground_wire_points]), axis=0)), dtype=float)
    reference_points = np.vstack([conductor_points, ground_wire_points])
    line_axis = _select_line_axis(
        reference_points,
        preferred_vectors=_manual_line_vectors(records),
        plane_angle=meta.get("plane_angle"),
    )
    return build_conductor_no_fly_volumes_from_point_cloud(
        center=center,
        wire_points=conductor_points,
        ground_wire_points=ground_wire_points,
        source="manual_route",
        plane_len=meta.get("plane_len"),
        line_axis=line_axis,
        extent_margin_m=extent_margin_m,
        min_length_m=min_length_m,
        tolerance_m=tolerance_m,
        top_margin_m=top_margin_m,
        bottom_margin_m=bottom_margin_m,
    )


def _build_wire_curves_from_points(
    points: np.ndarray,
    wire_type: str,
    start_id: int = 0,
) -> List[Dict[str, object]]:
    """Build wire_curve polylines from a set of conductor/ground_wire points."""
    pts = np.asarray(points, dtype=float)
    if len(pts) < 2:
        return []
    clusters = _cluster_points_spatial(pts, radius=3.0, min_points=2)
    curves: List[Dict[str, object]] = []
    for cluster_pts in clusters:
        if len(cluster_pts) < 2:
            continue
        try:
            axis = pca_main_direction(cluster_pts)
            if np.linalg.norm(axis) < 1e-9:
                axis = np.array([1.0, 0.0, 0.0], dtype=float)
            proj = np.dot(cluster_pts, axis)
            sort_idx = np.argsort(proj)
            sorted_pts = cluster_pts[sort_idx]
            proj_range = float(np.max(proj) - np.min(proj))
            bin_count = max(3, min(80, int(proj_range / 1.5)))
            polyline = []
            for bin_idx in range(bin_count):
                t0 = bin_idx / max(bin_count, 1)
                t1 = (bin_idx + 1) / max(bin_count, 1)
                mask = (proj >= np.min(proj) + t0 * proj_range) & (proj < np.min(proj) + t1 * proj_range)
                if bin_idx == bin_count - 1:
                    mask = (proj >= np.min(proj) + t0 * proj_range)
                if not np.any(mask):
                    continue
                bin_pts = sorted_pts[mask]
                median_pt = np.median(bin_pts, axis=0)
                polyline.append([float(median_pt[0]), float(median_pt[1]), float(median_pt[2])])
            if len(polyline) < 2:
                continue
            occ_radius = float(POWER_MODEL_CONFIG["wire_occ_radius"])
            safety_radius = float(DEFAULT_LIMITS["safety_distance_m"])
            curves.append({
                "id": start_id + len(curves),
                "type": wire_type,
                "polyline": polyline,
                "occ_radius": occ_radius,
                "safety_radius": safety_radius,
                "point_count": int(len(cluster_pts)),
                "source": "point_cloud_pca_polyline",
                "usage": [
                    "auxiliary_occlusion_geometry",
                    "fine_distance_geometry",
                    "fallback_when_no_fly_volume_missing",
                ],
            })
        except Exception:
            continue
    return curves


def _cluster_points_spatial(
    points: np.ndarray,
    radius: float = 3.0,
    min_points: int = 2,
) -> List[np.ndarray]:
    """Simple spatial clustering — lightweight DBSCAN-like without sklearn."""
    pts = np.asarray(points, dtype=float)
    if len(pts) < min_points:
        return [pts] if len(pts) >= 1 else []
    visited = np.zeros(len(pts), dtype=bool)
    clusters: List[np.ndarray] = []
    for i in range(len(pts)):
        if visited[i]:
            continue
        visited[i] = True
        cluster_indices = [i]
        queue = [i]
        while queue:
            cur = queue.pop(0)
            dists = np.linalg.norm(pts - pts[cur], axis=1)
            neighbors = np.where((dists <= radius) & (~visited))[0]
            for n in neighbors:
                visited[n] = True
                cluster_indices.append(int(n))
                queue.append(int(n))
        if len(cluster_indices) >= min_points:
            clusters.append(pts[np.asarray(cluster_indices, dtype=int)])
    if not clusters and len(pts) >= 1:
        clusters.append(pts)
    return clusters


def _merge_insulator_clusters(
    insulator_instances: List[Dict[str, object]],
    insulator_voxel_instance_map: Dict[Tuple[int, int, int], int],
    local_center: np.ndarray,
    line_dir: np.ndarray,
) -> Tuple[List[Dict[str, object]], Dict[Tuple[int, int, int], int], Dict[str, object]]:
    """Merge fragmented insulator clusters and filter noise.

    Returns (merged_instances, updated_voxel_map, clustering_stats).
    """
    raw_count = len(insulator_instances)
    stats: Dict[str, object] = {
        "raw_cluster_count": raw_count,
        "filtered_small_cluster_count": 0,
        "merged_instance_count": 0,
        "candidate_instance_count": 0,
        "avg_points_per_instance": 0.0,
        "warnings": [],
    }

    if raw_count == 0:
        return [], insulator_voxel_instance_map, stats

    # --- First, merge all clusters (including small fragments) to combine into larger instances ---
    all_instances = list(insulator_instances)
    for idx, inst in enumerate(all_instances):
        inst["fragment_type"] = "raw"

    merge_count_0 = 0
    if len(all_instances) > 1:
        all_instances, merge_count_0 = _merge_clusters_by_proximity(
            all_instances, local_center, line_dir
        )

    # --- Filter by quality ---
    # Noise if: very tiny (<10 pts), OR both small AND short
    # Survive if: decent points, OR decent length with adequate pts, OR merged from many fragments
    kept_instances: List[Dict[str, object]] = []
    noise_old_ids: set = set()
    noise_count = 0
    for inst in all_instances:
        pc = int(inst.get("point_count", 0))
        length = float(inst.get("length", 0))
        merged_n = len(inst.get("_merged_from", []))
        # Very tiny = noise
        if pc < 10:
            inst["fragment_type"] = "noise_fragment"
            for oid in inst.get("_merged_from", [int(inst.get("id", -1))]):
                noise_old_ids.add(int(oid))
            noise_old_ids.add(int(inst.get("id", -1)))
            noise_count += 1
        elif pc < INSULATOR_CLUSTER_MIN_POINTS and length < INSULATOR_CLUSTER_MIN_LENGTH_M:
            # Small AND short = noise
            inst["fragment_type"] = "noise_fragment"
            for oid in inst.get("_merged_from", [int(inst.get("id", -1))]):
                noise_old_ids.add(int(oid))
            noise_old_ids.add(int(inst.get("id", -1)))
            noise_count += 1
        else:
            inst["fragment_type"] = "candidate"
            kept_instances.append(inst)

    stats["filtered_small_cluster_count"] = noise_count
    stats["merged_instance_count"] = merge_count_0

    # --- Fallback: if nothing survived, keep top-5 by point_count ---
    if not kept_instances and all_instances:
        sorted_by_size = sorted(all_instances, key=lambda x: int(x.get("point_count", 0)), reverse=True)
        kept_instances = sorted_by_size[:5]
        for inst in kept_instances:
            inst["fragment_type"] = "fallback_kept"
        stats["warnings"].append("all_clusters_filtered_kept_top5_fallback")

    # --- Second-stage clustering on centers if still too many ---
    if len(kept_instances) > INSULATOR_MAX_CANDIDATE_INSTANCES:
        stats["warnings"].append(
            f"too_many_instances_{len(kept_instances)}_gt_{INSULATOR_MAX_CANDIDATE_INSTANCES}"
        )
        # Try second-stage: cluster centers by proximity
        kept_instances, merge_count_1 = _merge_clusters_by_proximity(
            kept_instances, local_center, line_dir
        )
        stats["merged_instance_count"] = merge_count_0 + merge_count_1

    # If still too many, keep top-K for candidates
    if len(kept_instances) > INSULATOR_MAX_CANDIDATE_INSTANCES:
        kept_instances.sort(key=lambda x: int(x.get("point_count", 0)), reverse=True)
        for inst in kept_instances[INSULATOR_MAX_CANDIDATE_INSTANCES:]:
            inst["fragment_type"] = "excluded_from_candidates"
        kept_instances = kept_instances[:INSULATOR_MAX_CANDIDATE_INSTANCES]

    # --- Remap voxel_instance_map to new merged IDs ---
    old_to_new: Dict[int, int] = {}
    for new_id, inst in enumerate(kept_instances):
        for old_id in inst.get("_merged_from", [int(inst.get("id", -1))]):
            old_to_new[int(old_id)] = new_id
    for new_id, inst in enumerate(kept_instances):
        oid = int(inst.get("id", -1))
        if oid not in old_to_new:
            old_to_new[oid] = new_id

    new_voxel_map: Dict[Tuple[int, int, int], int] = {}
    for key, old_id in insulator_voxel_instance_map.items():
        new_id = old_to_new.get(int(old_id), -1)
        if new_id >= 0:
            new_voxel_map[key] = new_id

    # Re-index
    for new_id, inst in enumerate(kept_instances):
        inst["id"] = new_id
        inst["fragment_type"] = inst.get("fragment_type", "merged")

    stats["candidate_instance_count"] = len(kept_instances)
    if kept_instances:
        stats["avg_points_per_instance"] = round(
            float(sum(int(i.get("point_count", 0)) for i in kept_instances)) / len(kept_instances), 1
        )

    return kept_instances, new_voxel_map, stats


def _merge_clusters_by_proximity(
    instances: List[Dict[str, object]],
    local_center: np.ndarray,
    line_dir: np.ndarray,
) -> Tuple[List[Dict[str, object]], int]:
    """Merge clusters that are close in space and have similar axis/outward direction.

    Returns (merged_instances, merge_count).
    """
    n = len(instances)
    if n <= 1:
        return instances, 0

    max_dist = INSULATOR_MERGE_MAX_DISTANCE_M
    max_axis_angle = INSULATOR_MERGE_MAX_AXIS_ANGLE_DEG
    max_outward_angle = INSULATOR_MERGE_MAX_OUTWARD_ANGLE_DEG
    cos_axis_threshold = math.cos(math.radians(max_axis_angle))
    cos_outward_threshold = math.cos(math.radians(max_outward_angle))

    # Build adjacency
    adjacency: Dict[int, List[int]] = {i: [] for i in range(n)}
    for i in range(n):
        ci = np.asarray(instances[i]["center"], dtype=float)
        ax_i = safe_normalize(np.asarray(instances[i].get("axis", [0, 0, 1]), dtype=float))
        out_i = safe_normalize(ci[:2] - local_center[:2])
        for j in range(i + 1, n):
            cj = np.asarray(instances[j]["center"], dtype=float)
            dist = float(np.linalg.norm(ci - cj))
            if dist > max_dist:
                continue
            ax_j = safe_normalize(np.asarray(instances[j].get("axis", [0, 0, 1]), dtype=float))
            if abs(float(np.dot(ax_i, ax_j))) < cos_axis_threshold:
                continue
            out_j = safe_normalize(cj[:2] - local_center[:2])
            if float(np.dot(out_i, out_j)) < cos_outward_threshold:
                continue
            adjacency[i].append(j)
            adjacency[j].append(i)

    # Connected components in adjacency graph
    visited = [False] * n
    components: List[List[int]] = []
    for i in range(n):
        if visited[i]:
            continue
        comp: List[int] = []
        stack = [i]
        while stack:
            node = stack.pop()
            if visited[node]:
                continue
            visited[node] = True
            comp.append(node)
            stack.extend(adjacency[node])
        components.append(comp)

    merge_count = n - len(components)

    merged: List[Dict[str, object]] = []
    for comp in components:
        if len(comp) == 1:
            idx = comp[0]
            inst = dict(instances[idx])
            inst["_merged_from"] = [int(instances[idx].get("id", idx))]
            merged.append(inst)
        else:
            # Merge all instances in this component
            all_coords = []
            merged_from = []
            total_pts = 0
            for idx in comp:
                merged_from.append(int(instances[idx].get("id", idx)))
                total_pts += int(instances[idx].get("point_count", 0))
                ci = np.asarray(instances[idx]["center"], dtype=float)
                # approximate coords from bbox
                all_coords.append(ci)

            merged_center = np.mean(all_coords, axis=0)
            all_axes = [safe_normalize(np.asarray(instances[idx].get("axis", [0, 0, 1]), dtype=float))
                       for idx in comp]
            merged_axis = safe_normalize(np.mean(all_axes, axis=0))

            # Compute merged bbox and length
            bboxes_min = [np.asarray(instances[idx].get("bbox_min", merged_center), dtype=float) for idx in comp]
            bboxes_max = [np.asarray(instances[idx].get("bbox_max", merged_center), dtype=float) for idx in comp]
            merged_bbox_min = np.min(bboxes_min, axis=0)
            merged_bbox_max = np.max(bboxes_max, axis=0)
            merged_length = float(np.linalg.norm(merged_bbox_max - merged_bbox_min))

            # Merged radius: weighted avg of per-instance radii
            merged_radius = 0.0
            for idx in comp:
                merged_radius += float(instances[idx].get("radius", 0.5)) / len(comp)

            merged.append({
                "id": comp[0],
                "center": merged_center.tolist(),
                "axis": merged_axis.tolist(),
                "length": round(float(merged_length), 4),
                "radius": round(float(merged_radius), 4),
                "bbox_min": merged_bbox_min.tolist(),
                "bbox_max": merged_bbox_max.tolist(),
                "point_count": total_pts,
                "fallback_used": False,
                "fragment_type": "merged",
                "_merged_from": merged_from,
            })

    return merged, merge_count


def build_semantic_surface_model(points: np.ndarray, classes: np.ndarray, voxel_size: float) -> Dict[str, object]:
    all_points = np.asarray(points, dtype=float)
    all_classes = np.asarray(classes, dtype=int)
    structure_mask = np.isin(all_classes.astype(int), list(STRUCTURE_LABELS))
    points = np.asarray(all_points[structure_mask], dtype=float)
    classes = np.asarray(all_classes[structure_mask], dtype=int)
    if len(points) == 0:
        raise ValueError("没有可用于体素建模的杆塔/绝缘子点")

    wire_points, _ = _voxel_centroids(
        all_points[all_classes == WIRE_LABEL],
        all_classes[all_classes == WIRE_LABEL],
        max(voxel_size * 5.0, 0.5),
        max_voxels=5000,
    )
    ground_wire_points, _ = _voxel_centroids(
        all_points[all_classes == GROUND_WIRE_LABEL],
        all_classes[all_classes == GROUND_WIRE_LABEL],
        max(voxel_size * 5.0, 0.5),
        max_voxels=5000,
    )
    safety_points, safety_labels, safety_meta = build_label_preserving_safety_points(
        all_points,
        all_classes,
        voxel_size,
    )

    min_bound = np.min(points, axis=0)
    max_bound = np.max(points, axis=0)
    local_center = np.mean(points, axis=0)
    z_min = float(np.min(points[:, 2]))
    z_max = float(np.max(points[:, 2]))
    tower_height = max(z_max - z_min, voxel_size)

    conductor_no_fly_volumes = build_conductor_no_fly_volumes_from_point_cloud(
        center=local_center,
        wire_points=wire_points,
        ground_wire_points=ground_wire_points,
        top_margin_m=0.0,
        bottom_margin_m=float(DEFAULT_LIMITS["safety_distance_m"]) / 4.0,
    )

    no_fly_volumes_objs = load_conductor_no_fly_volumes(conductor_no_fly_volumes)
    z_axis = np.array([0.0, 0.0, 1.0], dtype=float)
    line_direction_source = "default_x_axis"
    if no_fly_volumes_objs:
        x_axis = safe_normalize(no_fly_volumes_objs[0].u_axis)
        line_direction_source = "no_fly_volume_u_axis"
    elif len(wire_points) >= 3:
        try:
            x_axis = safe_normalize(pca_main_direction(wire_points))
            x_axis[2] = 0.0
            x_axis = safe_normalize(x_axis)
            line_direction_source = "conductor_pca"
        except Exception:
            x_axis = None
    else:
        x_axis = None
    if x_axis is None and len(points) >= 3:
        try:
            tower_xy = np.asarray(points[:, :2], dtype=float)
            x_axis_2d = pca_main_direction(tower_xy)
            x_axis = np.array([float(x_axis_2d[0]), float(x_axis_2d[1]), 0.0], dtype=float)
            x_axis = safe_normalize(x_axis)
            line_direction_source = "tower_pca_fallback"
        except Exception:
            x_axis = None
    if x_axis is None:
        x_axis = np.array([1.0, 0.0, 0.0], dtype=float)
        line_direction_source = "default_x_axis"
    y_axis = safe_normalize(np.cross(z_axis, x_axis))
    local_frame = {
        "origin": local_center.tolist(),
        "x_axis": x_axis.tolist(),
        "y_axis": y_axis.tolist(),
        "z_axis": z_axis.tolist(),
        "line_direction_source": line_direction_source,
    }

    tower_layer_height_m = float(POWER_MODEL_CONFIG["tower_layer_height_m"])
    tower_sector_count = int(POWER_MODEL_CONFIG["tower_sector_count"])

    def compute_layer_sector(coord: np.ndarray) -> Tuple[int, int]:
        layer_id = max(0, int(math.floor((float(coord[2]) - z_min) / tower_layer_height_m)))
        dx = float(coord[0]) - float(local_center[0])
        dy = float(coord[1]) - float(local_center[1])
        azimuth = (math.degrees(math.atan2(dy, dx)) + 360.0) % 360.0
        sector_id = int(azimuth // (360.0 / max(tower_sector_count, 1)))
        return layer_id, sector_id

    def tower_weight_for_semantic(semantic: str) -> float:
        return {
            "tower_body": float(POWER_MODEL_CONFIG["tower_body_weight"]),
            "tower_edge": float(POWER_MODEL_CONFIG["tower_edge_weight"]),
            "tower_top": float(POWER_MODEL_CONFIG["tower_top_weight"]),
            "tower_lower30": float(POWER_MODEL_CONFIG["tower_lower_weight"]),
        }.get(semantic, SEMANTIC_WEIGHTS.get(semantic, 1.0))

    def tower_dist_limits(semantic: str) -> Tuple[float, float, float]:
        return {
            "tower_body": (
                float(POWER_MODEL_CONFIG["tower_body_min_dist"]),
                float(POWER_MODEL_CONFIG["tower_body_max_dist"]),
                float(POWER_MODEL_CONFIG["tower_body_max_view_angle_deg"]),
            ),
            "tower_edge": (
                float(POWER_MODEL_CONFIG["tower_edge_min_dist"]),
                float(POWER_MODEL_CONFIG["tower_edge_max_dist"]),
                float(POWER_MODEL_CONFIG["tower_edge_max_view_angle_deg"]),
            ),
            "tower_top": (
                float(POWER_MODEL_CONFIG["tower_top_min_dist"]),
                float(POWER_MODEL_CONFIG["tower_top_max_dist"]),
                float(POWER_MODEL_CONFIG["tower_top_max_view_angle_deg"]),
            ),
            "tower_lower30": (
                float(POWER_MODEL_CONFIG["tower_lower_min_dist"]),
                float(POWER_MODEL_CONFIG["tower_lower_max_dist"]),
                float(POWER_MODEL_CONFIG["tower_lower_max_view_angle_deg"]),
            ),
        }.get(semantic, (4.0, 16.0, 70.0))

    voxel_map: Dict[Tuple[int, int, int], Dict[str, object]] = {}
    for pt, lbl in zip(points, classes):
        key = tuple(int(math.floor((pt[i] - min_bound[i]) / voxel_size)) for i in range(3))
        if key not in voxel_map:
            voxel_map[key] = {
                "sum": np.zeros(3, dtype=float),
                "count": 0,
                "labels": {},
                "grid_index": key,
            }
        cell = voxel_map[key]
        cell["sum"] += pt
        cell["count"] += 1
        cell["labels"][int(lbl)] = cell["labels"].get(int(lbl), 0) + 1

    occupied = set(voxel_map.keys())
    surface_keys = []
    neighbors6 = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
    neighbors26 = [
        (dx, dy, dz)
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        for dz in (-1, 0, 1)
        if not (dx == 0 and dy == 0 and dz == 0)
    ]
    for key, cell in voxel_map.items():
        dominant_label = max(cell["labels"].items(), key=lambda item: item[1])[0]
        cell["coord"] = (cell["sum"] / max(int(cell["count"]), 1)).astype(float)
        cell["label"] = dominant_label
        cell["neighbor_count26"] = sum(
            1 for dx, dy, dz in neighbors26 if (key[0] + dx, key[1] + dy, key[2] + dz) in occupied
        )
        if dominant_label == INSULATOR_LABEL or any((key[0] + dx, key[1] + dy, key[2] + dz) not in occupied for dx, dy, dz in neighbors6):
            surface_keys.append(key)

    surface_voxels = {key: voxel_map[key] for key in surface_keys}
    insulator_keys = [key for key, value in surface_voxels.items() if int(value["label"]) == INSULATOR_LABEL]
    insulator_components = _connected_components(insulator_keys)
    insulator_instances: List[Dict[str, object]] = []
    insulator_clusters = []
    insulator_voxel_instance_map: Dict[Tuple[int, int, int], int] = {}
    for comp_idx, component in enumerate(insulator_components):
        coords = np.array([surface_voxels[key]["coord"] for key in component], dtype=float)
        if len(coords) == 0:
            continue
        center = np.mean(coords, axis=0)
        fallback_used = False
        try:
            axis = pca_main_direction(coords)
            fallback_used = False
        except Exception:
            bbox_size = np.max(coords, axis=0) - np.min(coords, axis=0)
            axis = np.array([1.0, 0.0, 0.0]) if bbox_size[0] >= max(bbox_size[1], bbox_size[2]) else (
                np.array([0.0, 1.0, 0.0]) if bbox_size[1] >= bbox_size[2] else np.array([0.0, 0.0, 1.0])
            )
            fallback_used = True
        if np.linalg.norm(axis) < 1e-9:
            axis = np.array(local_frame["x_axis"], dtype=float)
            fallback_used = True
        axis = safe_normalize(axis)
        proj = np.dot(coords - center, axis)
        length = float(np.max(proj) - np.min(proj)) if len(proj) > 0 else 1.0
        perp_dists = np.linalg.norm((coords - center) - np.outer(proj, axis), axis=1)
        radius = float(np.percentile(perp_dists, 85)) if len(perp_dists) >= 2 else (float(np.max(perp_dists)) if len(perp_dists) else 0.5)
        if not np.isfinite(radius) or radius <= 0.0:
            bbox_dims = np.max(coords, axis=0) - np.min(coords, axis=0)
            radius = float(np.sort(bbox_dims)[-2]) / 2.0 if len(bbox_dims) >= 2 else 0.5
            if not np.isfinite(radius) or radius <= 0.0:
                radius = 0.5
        bbox_min = np.min(coords, axis=0).tolist()
        bbox_max = np.max(coords, axis=0).tolist()
        for key in component:
            insulator_voxel_instance_map[key] = comp_idx
        instance = {
            "id": comp_idx,
            "center": center.tolist(),
            "axis": axis.tolist(),
            "length": round(float(length), 4),
            "radius": round(float(radius), 4),
            "bbox_min": bbox_min,
            "bbox_max": bbox_max,
            "point_count": int(len(coords)),
            "fallback_used": fallback_used,
        }
        insulator_instances.append(instance)
        insulator_clusters.append({
            "center": center.tolist(),
            "size": int(len(coords)),
            "instance_id": comp_idx,
            "axis": axis.tolist(),
            "length": round(float(length), 4),
            "radius": round(float(radius), 4),
        })
    # --- Merge fragmented insulator clusters ---
    clustering_stats: Dict[str, object] = {}
    if insulator_instances:
        insulator_instances, insulator_voxel_instance_map, clustering_stats = _merge_insulator_clusters(
            insulator_instances, insulator_voxel_instance_map,
            np.asarray(local_center, dtype=float),
            safe_normalize(np.asarray(local_frame.get("x_axis", [1.0, 0.0, 0.0]), dtype=float)),
        )
    insulator_instances.sort(key=lambda item: (int(item.get("point_count", 0)), float(np.asarray(item.get("center", [0,0,0]), dtype=float)[2])), reverse=True)
    insulator_clusters = [
        {
            "center": inst.get("center", local_center),
            "size": int(inst.get("point_count", 0)),
            "instance_id": int(inst.get("id", 0)),
            "axis": inst.get("axis", [0, 0, 1]),
            "length": float(inst.get("length", 1)),
            "radius": float(inst.get("radius", 0.5)),
            "fragment_type": inst.get("fragment_type", "unknown"),
        }
        for inst in insulator_instances
    ]
    insulator_clusters.sort(key=lambda item: (item["size"], item["center"][2]), reverse=True)

    wire_curves: List[Dict[str, object]] = []
    for wire_label, wire_type, source_pts in (
        (WIRE_LABEL, "conductor", wire_points),
        (GROUND_WIRE_LABEL, "ground_wire", ground_wire_points),
    ):
        try:
            pts = np.asarray(source_pts, dtype=float)
            if len(pts) < 2:
                continue
            wire_curves.extend(_build_wire_curves_from_points(
                pts, wire_type, start_id=len(wire_curves)
            ))
        except Exception:
            continue

    insulator_surface_points = np.asarray(
        [value["coord"] for value in surface_voxels.values() if int(value["label"]) == INSULATOR_LABEL],
        dtype=float,
    )
    tower_surface_points = np.asarray(
        [value["coord"] for value in surface_voxels.values() if int(value["label"]) == TOWER_LABEL],
        dtype=float,
    )
    tower_top_points = (
        tower_surface_points[tower_surface_points[:, 2] >= float(np.percentile(tower_surface_points[:, 2], 75))]
        if len(tower_surface_points)
        else tower_surface_points
    )
    tower_base_points = (
        tower_surface_points[tower_surface_points[:, 2] <= z_min + tower_height * 0.12]
        if len(tower_surface_points)
        else tower_surface_points
    )
    if len(tower_surface_points) and len(tower_base_points) == 0:
        tower_base_points = tower_surface_points[
            tower_surface_points[:, 2] <= float(np.percentile(tower_surface_points[:, 2], 15))
        ]
    attention_targets: List[Dict[str, object]] = []

    def add_attention_records(
        source_points: np.ndarray,
        reference_points: np.ndarray,
        semantic: str,
        distance_limit: float,
        fallback_count: int,
    ) -> None:
        if len(source_points) == 0 or len(reference_points) == 0:
            return
        semantic = _normalize_semantic(semantic)
        distances = _nearest_distances(source_points, reference_points)
        selected_indices = np.where(distances <= distance_limit)[0].tolist()
        if not selected_indices:
            selected_indices = np.argsort(distances)[:fallback_count].tolist()
        for index in selected_indices[:fallback_count]:
            coord = np.asarray(source_points[index], dtype=float)
            reference_index = int(np.argmin(np.sum((reference_points - coord) ** 2, axis=1)))
            normal_hint = normalize_vector(coord - reference_points[reference_index])
            type_id = {
                "conductor_insulator_connection": 4,
                "wire_insulator_connection": 4,
                "insulator_tower_side_connection": 6,
                "ground_wire_tower_connection": 5,
                "tower_base_connection": 8,
            }.get(semantic, 4)
            label_id = {
                "conductor_insulator_connection": WIRE_LABEL,
                "wire_insulator_connection": WIRE_LABEL,
                "insulator_tower_side_connection": TOWER_LABEL,
                "ground_wire_tower_connection": GROUND_WIRE_LABEL,
                "tower_base_connection": TOWER_LABEL,
            }.get(semantic, TOWER_LABEL)
            att_weight = {
                "conductor_insulator_connection": float(POWER_MODEL_CONFIG["connection_weight"]),
                "wire_insulator_connection": float(POWER_MODEL_CONFIG["connection_weight"]),
                "insulator_tower_side_connection": float(POWER_MODEL_CONFIG["connection_weight"]),
                "ground_wire_tower_connection": float(POWER_MODEL_CONFIG["ground_wire_connection_weight"]),
                "tower_base_connection": float(POWER_MODEL_CONFIG["tower_base_weight"]),
            }.get(semantic, SEMANTIC_WEIGHTS.get(semantic, 4.0))
            att_min_dist: float = 3.0
            att_max_dist: float = 14.0
            att_max_view: float = 55.0
            if semantic in ("conductor_insulator_connection", "wire_insulator_connection"):
                att_min_dist, att_max_dist, att_max_view = 3.0, 14.0, 50.0
            elif semantic == "insulator_tower_side_connection":
                att_min_dist, att_max_dist, att_max_view = 2.5, 16.0, 55.0
            elif semantic == "ground_wire_tower_connection":
                att_min_dist, att_max_dist, att_max_view = 3.5, 14.0, 55.0
            elif semantic == "tower_base_connection":
                att_min_dist, att_max_dist, att_max_view = 4.0, 14.0, 65.0
            attention_targets.append({
                "coord": coord.tolist(),
                "type": type_id,
                "label": label_id,
                "category": "connection_attention",
                "semantic": semantic,
                "is_target": False,
                "is_attention_target": True,
                "is_planning_target": True,
                "weight": att_weight,
                "required_resolution": REQUIRED_RESOLUTION.get(semantic, REQUIRED_RESOLUTION.get("conductor_insulator_connection", 1.3)),
                "incidence_max_deg": INCIDENCE_THRESHOLDS.get(semantic, INCIDENCE_THRESHOLDS.get("conductor_insulator_connection", 50.0)),
                "normal_hint": normal_hint.tolist(),
                "normal": normal_hint.tolist(),
                "source_part": semantic,
                "related_instance_id": None,
                "min_dist": att_min_dist,
                "max_dist": att_max_dist,
                "max_view_angle_deg": att_max_view,
                "required_gsd": None,
                "source": "attention_target",
                "nearest_structure_distance_m": round(float(distances[index]), 3),
            })

    add_attention_records(
        wire_points,
        insulator_surface_points,
        "conductor_insulator_connection",
        distance_limit=3.5,
        fallback_count=36,
    )
    add_attention_records(
        ground_wire_points,
        tower_top_points,
        "ground_wire_tower_connection",
        distance_limit=4.5,
        fallback_count=24,
    )
    # ── Insulator tower-side connections: insulator surface near tower surface ──
    add_attention_records(
        insulator_surface_points,
        tower_surface_points,
        "insulator_tower_side_connection",
        distance_limit=2.5,
        fallback_count=36,
    )

    def add_tower_base_records(base_points: np.ndarray, fallback_count: int = 12) -> None:
        if len(base_points) == 0:
            return
        bins: Dict[int, Tuple[float, np.ndarray]] = {}
        for coord in np.asarray(base_points, dtype=float):
            vec = coord[:2] - local_center[:2]
            azimuth = (math.degrees(math.atan2(float(vec[1]), float(vec[0]))) + 360.0) % 360.0
            bin_id = int(azimuth // 45.0)
            radial = float(np.linalg.norm(vec))
            if bin_id not in bins or radial > bins[bin_id][0]:
                bins[bin_id] = (radial, coord)
        selected = [item[1] for item in sorted(bins.values(), key=lambda item: item[0], reverse=True)]
        if len(selected) < min(fallback_count, len(base_points)):
            existing = {tuple(np.round(coord, 3)) for coord in selected}
            ordered = sorted(
                np.asarray(base_points, dtype=float),
                key=lambda coord: float(np.linalg.norm(coord[:2] - local_center[:2])),
                reverse=True,
            )
            for coord in ordered:
                key = tuple(np.round(coord, 3))
                if key in existing:
                    continue
                selected.append(coord)
                existing.add(key)
                if len(selected) >= fallback_count:
                    break
        for coord in selected[:fallback_count]:
            normal_hint = normalize_vector([coord[0] - local_center[0], coord[1] - local_center[1], 0.0])
            tb_weight = float(POWER_MODEL_CONFIG["tower_base_weight"])
            attention_targets.append({
                "coord": np.asarray(coord, dtype=float).tolist(),
                "type": 8,
                "label": TOWER_LABEL,
                "category": "connection_attention",
                "semantic": "tower_base_connection",
                "is_target": False,
                "is_attention_target": True,
                "is_planning_target": True,
                "weight": tb_weight,
                "required_resolution": REQUIRED_RESOLUTION["tower_base_connection"],
                "incidence_max_deg": INCIDENCE_THRESHOLDS["tower_base_connection"],
                "normal_hint": normal_hint.tolist(),
                "normal": normal_hint.tolist(),
                "source_part": "tower_base_connection",
                "related_instance_id": None,
                "min_dist": 4.0,
                "max_dist": 14.0,
                "max_view_angle_deg": 65.0,
                "required_gsd": None,
                "source": "attention_target",
                "nearest_structure_distance_m": 0.0,
            })

    add_tower_base_records(tower_base_points)

    radial_distances = np.linalg.norm(points[:, :2] - local_center[:2], axis=1)
    edge_radius_threshold = float(np.percentile(radial_distances, 92)) if len(radial_distances) else 0.0

    display_voxels: List[Dict[str, object]] = []
    for key, cell in surface_voxels.items():
        coord = np.asarray(cell["coord"], dtype=float)
        label = int(cell["label"])
        z_ratio = float((coord[2] - z_min) / tower_height) if tower_height > 1e-6 else 0.5
        radial = float(np.linalg.norm(coord[:2] - local_center[:2]))
        radial_normal = normalize_vector([
            coord[0] - local_center[0], coord[1] - local_center[1], 0.0,
        ])
        if float(np.linalg.norm([coord[0] - local_center[0], coord[1] - local_center[1]])) < 1e-6:
            radial_normal = np.array(local_frame["y_axis"], dtype=float)
        category = "insulator" if label == INSULATOR_LABEL else "tower"
        if label == INSULATOR_LABEL:
            semantic = "insulator"
        elif z_ratio < 0.40:
            semantic = "tower_lower30"
        elif z_ratio >= 0.78:
            semantic = "tower_top"
        elif int(cell["neighbor_count26"]) <= 10 or radial >= edge_radius_threshold:
            semantic = "tower_edge"
        else:
            semantic = "tower_body"
        layer_id, sector_id = compute_layer_sector(coord)
        voxel_weight = tower_weight_for_semantic(semantic) if category == "tower" else SEMANTIC_WEIGHTS.get(semantic, 1.0)
        min_dist, max_dist, max_view = tower_dist_limits(semantic) if category == "tower" else (
            float(POWER_MODEL_CONFIG["insulator_min_dist"]),
            float(POWER_MODEL_CONFIG["insulator_max_dist"]),
            float(POWER_MODEL_CONFIG["insulator_max_view_angle_deg"]),
        )
        area = estimate_patch_area(int(cell["count"]), voxel_size, semantic)
        insulator_instance_id = insulator_voxel_instance_map.get(key)
        is_planning = semantic in TARGET_SEMANTICS or bool(insulator_instance_id is not None)

        display_voxels.append({
            "coord": coord.tolist(),
            "type": 3 if semantic == "insulator" else (2 if semantic == "tower_edge" else 1),
            "label": label,
            "category": category,
            "semantic": semantic,
            "is_target": is_planning,
            "is_planning_target": is_planning,
            "weight": voxel_weight,
            "area": round(float(area), 6),
            "required_resolution": REQUIRED_RESOLUTION.get(semantic, 0.7),
            "incidence_max_deg": INCIDENCE_THRESHOLDS.get(semantic, 65.0),
            "normal_hint": radial_normal.tolist(),
            "normal": radial_normal.tolist(),
            "layer_id": layer_id,
            "sector_id": sector_id,
            "min_dist": min_dist,
            "max_dist": max_dist,
            "max_view_angle_deg": max_view,
            "required_gsd": None,
            "insulator_instance_id": insulator_instance_id,
            "instance_id": insulator_instance_id,
            "axial_segment_id": None,
            "around_id": None,
            "source": "tower_layer_sector" if category == "tower" else "insulator",
            "point_count": int(cell["count"]),
            "grid_index": list(key),
        })

    for label, semantic, category, records in (
        (WIRE_LABEL, "wire", "wire", wire_points),
        (GROUND_WIRE_LABEL, "ground_wire", "ground_wire", ground_wire_points),
    ):
        for coord in records[:2000]:
            display_voxels.append({
                "coord": np.asarray(coord, dtype=float).tolist(),
                "type": 6 if label == WIRE_LABEL else 7,
                "label": int(label),
                "category": category,
                "semantic": semantic,
                "is_target": False,
                "weight": 0.0,
                "required_resolution": 0.0,
                "incidence_max_deg": 90.0,
                "normal_hint": [0.0, 0.0, 1.0],
                "point_count": 1,
            })
    display_voxels.extend(attention_targets)

    target_cells: List[Dict[str, object]] = []
    n_insulator_segments = int(POWER_MODEL_CONFIG["insulator_segments"])
    n_insulator_around = int(POWER_MODEL_CONFIG["insulator_around"])
    for instance in insulator_instances:
        try:
            ins_center = np.asarray(instance["center"], dtype=float)
            ins_axis = safe_normalize(instance["axis"])
            ins_length = float(instance["length"])
            ins_radius = float(instance["radius"])
            instance_id = int(instance["id"])
            outward = safe_normalize([ins_center[0] - local_center[0], ins_center[1] - local_center[1], 0.0])
            if np.linalg.norm(outward) < 1e-6:
                outward = np.array(local_frame["y_axis"], dtype=float)
            radial_perp = safe_normalize(np.cross(ins_axis, outward))
            for seg in range(n_insulator_segments):
                t_center = (seg + 0.5) / float(n_insulator_segments)
                seg_center = ins_center + ins_axis * (t_center - 0.5) * ins_length
                for ring in range(n_insulator_around):
                    angle = 2.0 * math.pi * ring / float(n_insulator_around)
                    normal_dir = safe_normalize(
                        math.cos(angle) * outward + math.sin(angle) * radial_perp
                    )
                    cell_center = seg_center + normal_dir * ins_radius
                    target_cells.append({
                        "coord": cell_center.tolist(),
                        "pos": cell_center.tolist(),
                        "label": INSULATOR_LABEL,
                        "category": "insulator",
                        "semantic": "insulator",
                        "is_target": True,
                        "is_planning_target": True,
                        "weight": float(POWER_MODEL_CONFIG["insulator_weight"]),
                        "area": round(float(ins_length * ins_radius * 2.0 * math.pi / max(n_insulator_segments * n_insulator_around, 1)), 6),
                        "normal": normal_dir.tolist(),
                        "normal_hint": normal_dir.tolist(),
                        "incidence_max_deg": float(POWER_MODEL_CONFIG["insulator_max_view_angle_deg"]),
                        "max_view_angle_deg": float(POWER_MODEL_CONFIG["insulator_max_view_angle_deg"]),
                        "required_resolution": REQUIRED_RESOLUTION.get("insulator", 1.5),
                        "required_gsd": None,
                        "min_dist": float(POWER_MODEL_CONFIG["insulator_min_dist"]),
                        "max_dist": float(POWER_MODEL_CONFIG["insulator_max_dist"]),
                        "instance_id": instance_id,
                        "insulator_instance_id": instance_id,
                        "layer_id": None,
                        "sector_id": None,
                        "axial_segment_id": seg,
                        "around_id": ring,
                        "point_count": 1,
                        "source": "insulator_axis_ring",
                    })
        except Exception:
            continue

    safety_warnings: List[str] = []
    if not conductor_no_fly_volumes and not wire_curves:
        safety_warnings.append("no_conductor_no_fly_volumes_and_no_wire_curves: candidate safety fallback limited")

    required_clearance_m = required_no_fly_clearance_m(DEFAULT_LIMITS["safety_distance_m"])

    patch_bins: Dict[Tuple[str, int, int, int, int], List[Dict[str, object]]] = {}
    for voxel in display_voxels:
        semantic = str(voxel["semantic"])
        if semantic in NON_TARGET_SEMANTICS and semantic not in ATTENTION_SEMANTICS:
            continue
        coord = np.asarray(voxel["coord"], dtype=float)
        if semantic == "insulator":
            inst_id = int(voxel.get("insulator_instance_id", -1) or -1)
            axial_seg = int(voxel.get("axial_segment_id", -1) or -1)
            around_id = int(voxel.get("around_id", -1) or -1)
            if inst_id >= 0 and axial_seg is not None and axial_seg >= 0:
                patch_key = (semantic, inst_id, axial_seg, around_id if around_id is not None and around_id >= 0 else 0, 0)
            else:
                vec = coord[:2] - local_center[:2]
                azimuth = (math.degrees(math.atan2(float(vec[1]), float(vec[0]))) + 360.0) % 360.0
                cfg = PATCH_BIN_CONFIG.get(semantic, {"azimuth_deg": 30, "height_bins": 6})
                azimuth_bin = int(azimuth // float(cfg["azimuth_deg"]))
                z_ratio = float((coord[2] - z_min) / tower_height) if tower_height > 1e-6 else 0.5
                h_bins = max(int(cfg.get("height_bins", 6)), 1)
                height_bin = min(h_bins - 1, max(0, int(z_ratio * h_bins)))
                patch_key = (semantic, inst_id if inst_id >= 0 else -1, height_bin, azimuth_bin, 0)
        elif semantic in ATTENTION_SEMANTICS:
            att_idx = {"conductor_insulator_connection": 0, "wire_insulator_connection": 0, "insulator_tower_side_connection": 1, "ground_wire_tower_connection": 2, "tower_base_connection": 3}.get(semantic, 0)
            vec = coord[:2] - local_center[:2]
            azimuth = (math.degrees(math.atan2(float(vec[1]), float(vec[0]))) + 360.0) % 360.0
            azimuth_bin = int(azimuth // 45.0)
            patch_key = (semantic, att_idx, azimuth_bin, 0, 0)
        elif semantic.startswith("tower"):
            layer_id = int(voxel.get("layer_id", 0) or 0)
            sector_id = int(voxel.get("sector_id", 0) or 0)
            patch_key = (semantic, layer_id, sector_id, 0, 0)
        else:
            vec = coord[:2] - local_center[:2]
            azimuth = (math.degrees(math.atan2(float(vec[1]), float(vec[0]))) + 360.0) % 360.0
            azimuth_bin = int(azimuth // 45.0)
            z_ratio = float((coord[2] - z_min) / tower_height) if tower_height > 1e-6 else 0.5
            height_bin = max(0, min(7, int(z_ratio * 8.0)))
            patch_key = (semantic, azimuth_bin, height_bin, 0, 0)
        patch_bins.setdefault(patch_key, []).append(voxel)

    voxels: List[Dict[str, object]] = []
    patch_id_counter = 0
    patch_counts = {semantic: 0 for semantic in ALL_SEMANTICS}
    for patch_key, members in patch_bins.items():
        semantic, k1, k2, k3, k4 = patch_key
        coords = np.asarray([member["coord"] for member in members], dtype=float)
        normals = np.asarray([member.get("normal_hint", member.get("normal", [0.0, 0.0, 1.0])) for member in members], dtype=float)
        mean_normal = safe_normalize(np.mean(normals, axis=0))
        point_count = int(sum(int(member.get("point_count", 1)) for member in members))
        label = INSULATOR_LABEL if semantic == "insulator" else TOWER_LABEL
        category = "insulator" if semantic == "insulator" else "tower"
        weights = [float(member.get("weight", SEMANTIC_WEIGHTS.get(semantic, 1.0))) for member in members]
        avg_weight = float(np.mean(weights)) if weights else SEMANTIC_WEIGHTS.get(semantic, 1.0)
        areas = [float(member.get("area", estimate_patch_area(member.get("point_count", 1), voxel_size, semantic))) for member in members]
        avg_area = float(np.mean(areas)) if areas else estimate_patch_area(point_count, voxel_size, semantic)
        min_dists = [float(member.get("min_dist", 4.0)) for member in members]
        max_dists = [float(member.get("max_dist", 16.0)) for member in members]
        max_views = [float(member.get("max_view_angle_deg", 70.0)) for member in members]
        avg_min_dist = float(np.mean(min_dists)) if min_dists else 4.0
        avg_max_dist = float(np.mean(max_dists)) if max_dists else 16.0
        avg_max_view = float(np.mean(max_views)) if max_views else 70.0
        inst_ids = [member.get("insulator_instance_id", member.get("instance_id")) for member in members]
        inst_id = next((v for v in inst_ids if v is not None), None)
        layer_ids = [member.get("layer_id") for member in members if member.get("layer_id") is not None]
        sector_ids = [member.get("sector_id") for member in members if member.get("sector_id") is not None]
        src = members[0].get("source", "legacy") if members else "legacy"
        patch_id_counter += 1
        voxels.append({
            "id": patch_id_counter,
            "coord": np.mean(coords, axis=0).tolist(),
            "pos": np.mean(coords, axis=0).tolist(),
            "type": 3 if semantic == "insulator" else (2 if semantic == "tower_edge" else 1),
            "label": label,
            "category": category,
            "semantic": semantic,
            "is_target": True,
            "is_planning_target": True,
            "weight": round(float(avg_weight), 4),
            "area": round(float(avg_area), 6),
            "normal": mean_normal.tolist(),
            "normal_hint": mean_normal.tolist(),
            "required_resolution": REQUIRED_RESOLUTION.get(semantic, 0.7),
            "incidence_max_deg": INCIDENCE_THRESHOLDS.get(semantic, 65.0),
            "max_view_angle_deg": round(float(avg_max_view), 2),
            "min_dist": round(float(avg_min_dist), 2),
            "max_dist": round(float(avg_max_dist), 2),
            "required_gsd": None,
            "instance_id": inst_id,
            "insulator_instance_id": inst_id if semantic == "insulator" else None,
            "layer_id": int(layer_ids[0]) if layer_ids else None,
            "sector_id": int(sector_ids[0]) if sector_ids else None,
            "axial_segment_id": int(k2) if semantic == "insulator" and k2 >= 0 and inst_id is not None else None,
            "around_id": int(k3) if semantic == "insulator" and k3 >= 0 and inst_id is not None else None,
            "source": src,
            "point_count": point_count,
            "support_count": int(len(members)),
        })
        patch_counts[semantic] = patch_counts.get(semantic, 0) + 1

    if target_cells:
        for tc in target_cells:
            patch_id_counter += 1
            tc["id"] = patch_id_counter
            tc["support_count"] = 1
            voxels.append(tc)
        patch_counts["insulator"] = patch_counts.get("insulator", 0) + len(target_cells)

    grid_x = int((max_bound[0] - min_bound[0]) / voxel_size) + 1
    grid_y = int((max_bound[1] - min_bound[1]) / voxel_size) + 1
    z_map = np.full((grid_y, grid_x), -np.inf, dtype=float)
    for voxel in display_voxels:
        coord = np.asarray(voxel["coord"], dtype=float)
        ix = int((coord[0] - min_bound[0]) / voxel_size)
        iy = int((coord[1] - min_bound[1]) / voxel_size)
        if 0 <= ix < grid_x and 0 <= iy < grid_y:
            z_map[iy, ix] = max(float(z_map[iy, ix]), float(coord[2]))

    num_tower_layers = int(math.ceil(tower_height / tower_layer_height_m)) if tower_height > 0 else 1

    # ── Topology validation ──
    _topo_att_counts: Dict[str, int] = {}
    for att in attention_targets:
        sem = str(att.get("semantic", ""))
        _topo_att_counts[sem] = _topo_att_counts.get(sem, 0) + 1
    _gw_ins_near = 0
    if len(ground_wire_points) > 0 and len(insulator_surface_points) > 0:
        _gw_ins_dists = _nearest_distances(ground_wire_points, insulator_surface_points)
        _gw_ins_near = int(np.sum(_gw_ins_dists <= 2.5))
    _cond_double = 0
    if len(wire_points) > 0 and len(insulator_surface_points) > 0 and len(ground_wire_points) > 0:
        _cond_ins_dists = _nearest_distances(wire_points, insulator_surface_points)
        _cond_gw_dists = _nearest_distances(wire_points, ground_wire_points)
        _cond_double = int(np.sum((_cond_ins_dists <= 3.5) & (_cond_gw_dists <= 3.5)))
    _topology_validation: Dict[str, object] = {
        "attention_target_counts": _topo_att_counts,
        "ground_wire_near_insulator_count": _gw_ins_near,
        "ground_wire_insulator_misassociation": bool(_gw_ins_near > 0),
        "conductor_near_both_insulator_and_ground_wire_count": _cond_double,
        "three_key_connection_types_present": {
            "conductor_insulator_connection": _topo_att_counts.get("conductor_insulator_connection", 0) > 0,
            "insulator_tower_side_connection": _topo_att_counts.get("insulator_tower_side_connection", 0) > 0,
            "ground_wire_tower_connection": _topo_att_counts.get("ground_wire_tower_connection", 0) > 0,
        },
    }

    return {
        "voxels": voxels,
        "attention_targets": attention_targets,
        "display_voxels": display_voxels,
        "target_cells": target_cells,
        "wire_curves": wire_curves,
        "conductor_no_fly_volumes": conductor_no_fly_volumes,
        "min_bound": min_bound,
        "max_bound": max_bound,
        "z_max_map": z_map,
        "safety_points": safety_points,
        "safety_labels": safety_labels,
        "local_center": local_center,
        "local_frame": local_frame,
        "meta": {
            "power_model_version": str(POWER_MODEL_CONFIG["power_model_version"]),
            "z_min": z_min,
            "z_max": z_max,
            "tower_height": tower_height,
            "local_frame": local_frame,
            "tower_layer_count": num_tower_layers,
            "tower_sector_count": int(tower_sector_count),
            "insulator_clusters": insulator_clusters,
            "insulator_instance_count": int(len(insulator_instances)),
            "insulator_instances": insulator_instances,
            "insulator_clustering": clustering_stats if clustering_stats else {
                "raw_cluster_count": 0,
                "filtered_small_cluster_count": 0,
                "merged_instance_count": 0,
                "candidate_instance_count": 0,
                "avg_points_per_instance": 0.0,
                "warnings": [],
            },
            "patch_counts": patch_counts,
            "target_cell_count": int(len(target_cells)),
            "weighted_target_enabled": True,
            "attention_counts": {
                semantic: int(sum(1 for item in attention_targets if item["semantic"] == semantic))
                for semantic in ATTENTION_SEMANTICS
            },
            "topology_validation": _topology_validation,
            "wire_voxel_count": int(len(wire_points)),
            "ground_wire_voxel_count": int(len(ground_wire_points)),
            "safety_voxel_count": int(len(safety_points)),
            "safety_scope": "all_physical",
            "wire_curve_count": int(len(wire_curves)),
            "wire_curves": serialize_wire_curves(wire_curves),
            "wire_curves_usage": "auxiliary_only_when_no_fly_exists" if conductor_no_fly_volumes else "fallback_when_no_fly_missing",
            "safety_model_priority": [
                "conductor_no_fly_volumes",
                "wire_curves",
                "safety_points",
            ],
            "no_fly_integrated_with_power_model": True,
            "no_fly_volume_count": int(len(conductor_no_fly_volumes)),
            "no_fly_required_clearance_m": round(float(required_clearance_m), 3),
            "candidate_no_fly_filter_enabled": True,
            "conductor_no_fly_source": "point_cloud" if conductor_no_fly_volumes else None,
            "patch_scale": estimate_patch_scale(display_voxels, local_center, tower_height),
            **safety_meta,
            "warnings": safety_warnings,
        },
    }


def generate_candidate_views(surface_model: Dict[str, object], manual_route_path: Optional[str | Path] = None) -> List[Dict[str, object]]:
    target_records = list(surface_model["voxels"]) + list(surface_model.get("attention_targets", []))
    local_center = np.asarray(surface_model["local_center"], dtype=float)
    meta = dict(surface_model.get("meta", {}))
    z_min = float(meta.get("z_min", local_center[2] - 5.0))
    z_max = float(meta.get("z_max", local_center[2] + 5.0))
    tower_height = max(float(meta.get("tower_height", z_max - z_min)), 1.0)

    conductor_no_fly_volumes = load_conductor_no_fly_volumes(surface_model.get("conductor_no_fly_volumes", []))

    candidates: List[Dict[str, object]] = []
    seen = set()
    position_ids: Dict[Tuple[float, float, float], int] = {}
    insulator_cluster_centers = [
        np.asarray(cluster.get("center", local_center), dtype=float)
        for cluster in meta.get("insulator_clusters", [])
        if isinstance(cluster, dict)
    ]

    def get_position_id(position: Sequence[float]) -> int:
        key = tuple(round(float(v), 3) for v in position)
        if key not in position_ids:
            position_ids[key] = len(position_ids) + 1
        return position_ids[key]

    def target_azimuth_deg(target: Sequence[float]) -> float:
        target_arr = np.asarray(target, dtype=float)
        vec = target_arr[:2] - local_center[:2]
        if float(np.linalg.norm(vec)) <= 1e-6:
            return 0.0
        return float((math.degrees(math.atan2(float(vec[1]), float(vec[0]))) + 360.0) % 360.0)

    def target_cluster_id(semantic: str, target: Sequence[float]) -> str:
        target_arr = np.asarray(target, dtype=float)
        azimuth_bin = int(target_azimuth_deg(target_arr) // 45.0)
        z_ratio = float((target_arr[2] - z_min) / tower_height)
        height_bin = max(0, min(7, int(z_ratio * 8.0)))
        if semantic == "insulator" and insulator_cluster_centers:
            nearest_index = min(
                range(len(insulator_cluster_centers)),
                key=lambda index: float(np.linalg.norm(target_arr - insulator_cluster_centers[index])),
            )
            return f"insulator_{nearest_index}"
        if semantic in ATTENTION_SEMANTICS:
            return f"{semantic}_{azimuth_bin}_{height_bin}"
        return f"{semantic}_{azimuth_bin}_{height_bin}"

    wire_curves = list(surface_model.get("wire_curves", []) or [])
    if not wire_curves:
        wire_curves_from_meta = meta.get("wire_curves", [])
        if wire_curves_from_meta:
            wire_curves = [
                {"polyline": w.get("polyline", []), "occ_radius": w.get("occ_radius", POWER_MODEL_CONFIG["wire_occ_radius"]),
                 "safety_radius": w.get("safety_radius", DEFAULT_LIMITS["safety_distance_m"]), "type": w.get("type", "conductor")}
                for w in wire_curves_from_meta if isinstance(w, dict)
            ]
    # Pre-compute bbox for each wire curve (with safety_radius expansion)
    wire_curve_bboxes: List[Tuple[np.ndarray, np.ndarray, float, int]] = []
    for ci, curve in enumerate(wire_curves):
        polyline = curve.get("polyline", [])
        if not polyline or len(polyline) < 2:
            wire_curve_bboxes.append((np.zeros(3), np.zeros(3), 0.0, ci))
            continue
        poly_arr = np.asarray(polyline, dtype=float)
        bbox_min = np.min(poly_arr, axis=0)
        bbox_max = np.max(poly_arr, axis=0)
        safety_rad = float(curve.get("safety_radius", DEFAULT_LIMITS["safety_distance_m"]))
        wire_curve_bboxes.append((bbox_min, bbox_max, safety_rad, ci))

    no_fly_required_clearance_m = required_no_fly_clearance_m(DEFAULT_LIMITS["safety_distance_m"])
    tower_safety_radius = float(POWER_MODEL_CONFIG["tower_safety_radius"])
    local_frame = surface_model.get("local_frame", {})
    line_dir = safe_normalize(np.asarray(local_frame.get("x_axis", [1.0, 0.0, 0.0]), dtype=float))
    safety_points = np.asarray(surface_model.get("safety_points", []), dtype=float)
    if safety_points.ndim != 2 or (safety_points.size and safety_points.shape[1] != 3):
        safety_points = np.empty((0, 3), dtype=float)
    candidate_safety_index = VoxelSafetyIndex(safety_points, max(DEFAULT_LIMITS["safety_distance_m"], 1.0))

    filter_stats = {"raw_candidate_count": 0, "inside_no_fly_rejected": 0, "clearance_rejected": 0,
                    "wire_curve_rejected": 0, "tower_safety_rejected": 0, "final_candidate_count": 0,
                    "fallback_relaxed_clearance_used": False}

    def _no_fly_lateral_push_direction(volume: ConductorNoFlyVolume, cand_pos: np.ndarray) -> np.ndarray:
        """Choose the nearest exterior lateral side, never the no-fly center line."""
        _, v_value, z_value = volume.local_coordinates(cand_pos)
        left_v, right_v = volume.side_bounds_at_z(z_value)
        if v_value < left_v:
            return -volume.v_axis
        if v_value > right_v:
            return volume.v_axis
        if abs(v_value - left_v) <= abs(right_v - v_value):
            return -volume.v_axis
        return volume.v_axis

    def filter_candidate_by_safety(cand_pos: np.ndarray, allow_adjust: bool = False):
        """Returns (result, reject_reason) where result is None or (position, clearance, adjusted, steps, warning)
        and reject_reason is 'ok' or 'inside_no_fly' or 'clearance' or 'wire_curve'."""
        adjusted = False
        steps = 0
        warning = None
        min_clearance = float("inf")

        for volume in conductor_no_fly_volumes:
            contains = volume.contains(cand_pos)
            clearance = volume.clearance(cand_pos)
            min_clearance = min(min_clearance, clearance)
            if contains or clearance + 1e-9 < no_fly_required_clearance_m:
                if not allow_adjust:
                    return None, "inside_no_fly" if contains else "clearance"
                while steps < 12:
                    push_dir = _no_fly_lateral_push_direction(volume, cand_pos)
                    current_clearance = volume.clearance(cand_pos)
                    step_size = max(
                        0.5,
                        min(2.5, no_fly_required_clearance_m - current_clearance + 0.25),
                    )
                    cand_pos = cand_pos + push_dir * step_size
                    steps += 1
                    adjusted = True
                    contains = volume.contains(cand_pos)
                    clearance = volume.clearance(cand_pos)
                    min_clearance = min(min_clearance, clearance)
                    if (not contains) and clearance + 1e-9 >= no_fly_required_clearance_m:
                        break
                contains = volume.contains(cand_pos)
                clearance = volume.clearance(cand_pos)
                min_clearance = min(min_clearance, clearance)
                if contains:
                    return None, "inside_no_fly"
                if clearance + 1e-9 < no_fly_required_clearance_m:
                    return None, "clearance"

        final_min_clearance = float("inf")
        for volume in conductor_no_fly_volumes:
            if volume.contains(cand_pos):
                return None, "inside_no_fly"
            clearance = volume.clearance(cand_pos)
            final_min_clearance = min(final_min_clearance, clearance)
            if clearance + 1e-9 < no_fly_required_clearance_m:
                return None, "clearance"

        for bbox_min, bbox_max, safety_rad, ci in wire_curve_bboxes:
            # Bbox pre-filter: skip if candidate is far from curve
            bbox_near_min = bbox_min - safety_rad
            bbox_near_max = bbox_max + safety_rad
            if (cand_pos[0] < bbox_near_min[0] or cand_pos[1] < bbox_near_min[1] or cand_pos[2] < bbox_near_min[2] or
                cand_pos[0] > bbox_near_max[0] or cand_pos[1] > bbox_near_max[1] or cand_pos[2] > bbox_near_max[2]):
                continue
            polyline = wire_curves[ci].get("polyline", [])
            dist = point_to_polyline_distance(cand_pos, polyline)
            if dist < safety_rad:
                return None, "wire_curve"

        if len(safety_points):
            safety_distance = candidate_safety_index.min_distance(
                cand_pos,
                search_radius_m=max(float(DEFAULT_LIMITS["safety_distance_m"]) * 2.5, 10.0),
            )
            if safety_distance + 1e-9 < float(DEFAULT_LIMITS["safety_distance_m"]):
                return None, "tower_safety"

        return (cand_pos, float(final_min_clearance), adjusted, steps, warning), "ok"

    def add_candidate(
        position: Sequence[float],
        target: Sequence[float],
        semantic_focus: str,
        focal_length_eq_mm: Optional[float] = None,
        yaw_offset: float = 0.0,
        pitch_offset: float = 0.0,
        manual_priority: float = 0.0,
        source: str = "semantic_surface",
        aim_type: Optional[str] = None,
        cluster_id: Optional[str] = None,
        target_id: Optional[int] = None,
        target_weight: Optional[float] = None,
        required_resolution: Optional[float] = None,
        max_view_angle_deg: Optional[float] = None,
        instance_id: Optional[int] = None,
        layer_id: Optional[int] = None,
        sector_id: Optional[int] = None,
        action_name: str = "photo",
        focal_level: Optional[str] = None,
    ):
        position = np.asarray(position, dtype=float)
        target = np.asarray(target, dtype=float)
        original_position = position.copy()
        filter_stats["raw_candidate_count"] += 1

        # ── Backward-compat: if caller passed a string focal level ("F0"/"F1"/"F2") ──
        if isinstance(focal_length_eq_mm, str):
            focal_level = focal_length_eq_mm
            focal_length_eq_mm = None

        # Safety filter (unchanged)
        allow_adjust = semantic_focus in ("insulator", "conductor_insulator_connection", "wire_insulator_connection", "insulator_tower_side_connection", "ground_wire_tower_connection")
        safety_result, reject_reason = filter_candidate_by_safety(position, allow_adjust=allow_adjust)
        if safety_result is None:
            if reject_reason == "wire_curve":
                filter_stats["wire_curve_rejected"] += 1
            elif reject_reason == "inside_no_fly":
                filter_stats["inside_no_fly_rejected"] += 1
            elif reject_reason == "clearance":
                filter_stats["clearance_rejected"] += 1
            elif reject_reason == "tower_safety":
                filter_stats["tower_safety_rejected"] += 1
            else:
                filter_stats["inside_no_fly_rejected"] += 1
            return
        safe_pos, no_fly_clearance, was_adjusted, adjust_steps, safety_warn = safety_result
        if no_fly_clearance + 1e-9 < no_fly_required_clearance_m:
            filter_stats["clearance_rejected"] += 1
            return
        position = safe_pos

        # --- Compute view geometry from position → target ---
        geom = compute_view_geometry(position, target)
        heading = float(geom["heading"])
        pitch = float(geom["pitch"])
        distance = float(geom["Distance"])

        # Apply yaw/pitch offsets
        heading = (heading + yaw_offset) % 360.0
        pitch = max(-85.0, min(25.0, pitch + pitch_offset))
        base_heading = float(geom["heading"])
        yaw_offset_deg = float(yaw_offset)

        # --- Focal length: use continuous value, compute FOV ---
        resolved_aim_type = aim_type or semantic_focus
        resolved_semantic = _normalize_semantic(semantic_focus)
        req_resolution = (
            float(required_resolution)
            if required_resolution is not None
            else float(REQUIRED_RESOLUTION.get(resolved_semantic, REQUIRED_RESOLUTION.get(semantic_focus, 0.7)))
        )
        if action_name == "none":
            focal_length_eq_mm = None
            focal_is_estimated = False
            focal_source = None
            hfov_deg_value = None
            vfov_deg_value = None
            f_eq_mm_value = None
            focal_key = None
        else:
            focal_key = None

            focal_was_explicit = focal_length_eq_mm is not None
            if focal_length_eq_mm is None:
                focal_length_eq_mm = choose_focal_length_eq_mm(
                    aim_type=resolved_aim_type,
                    distance=distance,
                    required_resolution=req_resolution,
                )
            if focal_length_eq_mm is None:
                focal_length_eq_mm = 48.0  # safety fallback
            focal_length_eq_mm = max(24.0, min(84.0, float(focal_length_eq_mm)))
            focal_is_estimated = True
            focal_source = "explicit" if focal_was_explicit else "aimtype_resolution_auto"
            hfov_deg_value = round(math.degrees(2.0 * math.atan(CAMERA_SENSOR_WIDTH_MM / (2.0 * focal_length_eq_mm))), 3)
            vfov_deg_value = round(math.degrees(2.0 * math.atan(CAMERA_SENSOR_HEIGHT_MM / (2.0 * focal_length_eq_mm))), 3)
            f_eq_mm_value = focal_length_eq_mm
            # Backward-compat focal_level: map continuous to closest named level
            best_level = min(SUPPORTED_FOCALS.items(), key=lambda kv: abs(kv[1]["f_eq_mm"] - focal_length_eq_mm))
            focal_key = best_level[0]

        # Distance validation
        if action_name == "photo" and resolved_aim_type in AIMTYPE_VIEW_PROFILE:
            profile = AIMTYPE_VIEW_PROFILE[resolved_aim_type]
            dist_range = profile.get("distance_range_m")
            if dist_range and not (dist_range[0] <= distance <= dist_range[1]):
                # Outside recommended range — allow but mark
                pass

        # Max visibility range check
        if distance > DEFAULT_LIMITS["max_visibility_distance_m"]:
            return

        # Legacy min distance check (backward compat)
        if focal_key and focal_key in SUPPORTED_FOCALS and distance < SUPPORTED_FOCALS[focal_key].get("min_distance_m", 0.0):
            return

        position_z_ratio = float((position[2] - z_min) / tower_height)
        target_z_ratio = float((target[2] - z_min) / tower_height)
        target_azimuth = target_azimuth_deg(target)
        cluster_id = cluster_id or target_cluster_id(semantic_focus, target)
        if semantic_focus == "tower_body" and (
            0.26 <= position_z_ratio <= 0.42 or 0.26 <= target_z_ratio <= 0.36
        ):
            return

        # Dedup key: waypoint position + target point. Focal is target-driven, not a candidate dimension.
        key = (
            round(float(position[0]), 3),
            round(float(position[1]), 3),
            round(float(position[2]), 3),
            round(float(target[0]), 3),
            round(float(target[1]), 3),
            round(float(target[2]), 3),
            semantic_focus,
            None if target_id is None else int(target_id),
        )
        if key in seen:
            return
        seen.add(key)
        filter_stats["final_candidate_count"] += 1

        cand = {
            "id": len(candidates) + 1,
            "position_id": get_position_id(position),
            "utm_position": position.tolist(),
            # ── New geometry fields ──
            "actionName": action_name,
            "AimType": resolved_aim_type if action_name == "photo" else "auxiliary",
            "look_at": target.tolist(),
            "Distance": round(distance, 3),
            "heading": round(heading, 3),
            "pitch": round(float(pitch), 3),
            "base_heading": round(base_heading, 3),
            "yaw_offset_deg": round(yaw_offset_deg, 3),
            # ── New focal fields ──
            "focal_length_eq_mm": focal_length_eq_mm,
            "focal_is_estimated": focal_is_estimated,
            "focal_source": focal_source,
            # ── Backward-compat fields ──
            "yaw": round(heading, 3),               # = heading + yaw_offset
            "focal_level": focal_key,                # nearest F-level or None
            "f_eq_mm": f_eq_mm_value,                # = focal_length_eq_mm
            "hfov_deg": hfov_deg_value,
            "vfov_deg": vfov_deg_value,
            # ── Existing fields ──
            "semantic_focus": semantic_focus,
            "target_center": target.tolist(),
            "position_z_ratio": round(position_z_ratio, 4),
            "target_z_ratio": round(target_z_ratio, 4),
            "target_azimuth_deg": round(target_azimuth, 3),
            "target_cluster_id": cluster_id,
            "manual_priority": round(float(manual_priority), 3),
            "source": source,
            "target_id": target_id,
            "semantic": semantic_focus,
            "instance_id": instance_id,
            "insulator_instance_id": instance_id if semantic_focus == "insulator" else None,
            "layer_id": layer_id,
            "sector_id": sector_id,
            "weight": target_weight if target_weight is not None else SEMANTIC_WEIGHTS.get(semantic_focus, 1.0),
            "required_gsd": None,
            "required_resolution": round(float(req_resolution), 6),
            "max_view_angle_deg": max_view_angle_deg if max_view_angle_deg is not None else INCIDENCE_THRESHOLDS.get(semantic_focus, 65.0),
            "no_fly_checked": True,
            "no_fly_inside": False,
            "no_fly_clearance_m": round(float(no_fly_clearance), 4),
            "no_fly_required_clearance_m": round(float(no_fly_required_clearance_m), 3),
            "safety_filter_source": "conductor_no_fly_volume" if conductor_no_fly_volumes else ("wire_curve_fallback" if wire_curves else "none"),
            "safety_checked": True,
            "adjusted_for_no_fly": was_adjusted,
            "adjustment_steps": adjust_steps,
            "original_position": None,
            "safety_warning": safety_warn,
            "is_coverage_target": action_name == "photo",
        }
        if was_adjusted:
            cand["original_position"] = original_position.tolist()
        if aim_type:
            cand["aim_type"] = aim_type
        candidates.append(cand)

    def add_no_fly_exterior_candidates(target: Sequence[float], semantic: str, cluster_id: str) -> None:
        """Add viewpoints just outside the lateral no-fly boundary for high-voltage targets."""
        if semantic not in ("insulator", "conductor_insulator_connection", "wire_insulator_connection", "insulator_tower_side_connection", "ground_wire_tower_connection"):
            return
        target_arr = np.asarray(target, dtype=float)
        exterior_clearance = no_fly_required_clearance_m
        for volume in conductor_no_fly_volumes:
            u_value, v_value, _ = volume.local_coordinates(target_arr)
            if u_value < volume.u_min - 5.0 or u_value > volume.u_max + 5.0:
                continue
            left_v, right_v = volume.side_bounds_at_z(float(target_arr[2]))
            nearest_left = abs(v_value - left_v) <= abs(v_value - right_v)
            outside_v = left_v - exterior_clearance if nearest_left else right_v + exterior_clearance
            for z_offset in (-0.6, 0.0, 0.6):
                position = volume.world_position(u_value, outside_v, float(target_arr[2]) + z_offset)
                add_candidate(
                    position,
                    target_arr,
                    semantic,
                    cluster_id=cluster_id,
                    source="no_fly_exterior",
                )

    # Initialize per-source raw candidate tracking
    tower_raw_by_source: Dict[str, int] = {
        "tower_body": 0, "tower_edge": 0, "tower_top": 0,
        "tower_lower30": 0, "insulator_instance": 0,
        "insulator_legacy_voxel": 0,
        "conductor_insulator_connection": 0, "insulator_tower_side_connection": 0,
        "ground_wire_tower_connection": 0, "tower_base_connection": 0,
    }

    tower_focus = np.array([local_center[0], local_center[1], z_min + 0.62 * tower_height], dtype=float)
    top_focus = np.array([local_center[0], local_center[1], z_min + 0.94 * tower_height], dtype=float)
    tower_mid_focus = np.array([local_center[0], local_center[1], z_min + 0.50 * tower_height], dtype=float)

    # ── Tower overview (塔全貌): 80-120m ring, 2-3 azimuths, 1-2 heights → 4-6 candidates ──
    overview_profile = AIMTYPE_VIEW_PROFILE["tower_overview"]
    overview_dist = overview_profile["preferred_distance_m"]  # 100m
    overview_pitch_rad = math.radians(overview_profile["preferred_pitch_deg"])  # -33°
    overview_h_dist = overview_dist * math.cos(overview_pitch_rad)
    overview_h_delta = overview_dist * math.sin(overview_pitch_rad)  # negative → P_z above T
    overview_azimuths = [0, 120, 240]  # 3 distinct directions
    overview_heights = [z_min + 0.50 * tower_height, z_min + 0.70 * tower_height]
    overview_count = 0
    for ov_az in overview_azimuths:
        ov_angle = math.radians(ov_az)
        for ov_z_target in overview_heights:
            if overview_count >= overview_profile["max_candidates"]:
                break
            ov_pos = np.array([
                local_center[0] + overview_h_dist * math.cos(ov_angle),
                local_center[1] + overview_h_dist * math.sin(ov_angle),
                ov_z_target - overview_h_delta,  # P_z = T_z - h*d*tan(pitch), pitch<0 → P_z > T_z
            ])
            add_candidate(ov_pos, tower_mid_focus, "tower_overview",
                          aim_type="tower_overview", source="aimtype_overview")
            overview_count += 1
    tower_raw_by_source["tower_overview"] = overview_count

    # ── Fallback overview: if safety filtered too many, add more distant directions ──
    overview_survivors = sum(1 for c in candidates if c.get("AimType") == "tower_overview")
    if overview_survivors < 3:
        fallback_az = [60, 180, 300]
        fallback_dist = 55.0
        fb_h_d = fallback_dist * math.cos(overview_pitch_rad)
        fb_h_delta = fallback_dist * math.sin(overview_pitch_rad)
        for fb_az in fallback_az:
            if overview_count >= overview_profile["max_candidates"]:
                break
            fb_angle = math.radians(fb_az)
            fb_pos = np.array([
                local_center[0] + fb_h_d * math.cos(fb_angle),
                local_center[1] + fb_h_d * math.sin(fb_angle),
                z_min + 0.55 * tower_height - fb_h_delta,
            ])
            add_candidate(fb_pos, tower_mid_focus, "tower_overview",
                          aim_type="tower_overview", source="aimtype_overview_fallback")
            overview_count += 1
        tower_raw_by_source["tower_overview"] = overview_count

    # ── Tower top (塔头): 50-70m mid-distance oblique ──
    top_profile = AIMTYPE_VIEW_PROFILE["tower_top"]
    top_dist = top_profile["preferred_distance_m"]
    top_pitch_rad = math.radians(top_profile["preferred_pitch_deg"])
    top_h_dist = top_dist * math.cos(top_pitch_rad)
    top_h_delta = top_dist * math.sin(top_pitch_rad)
    top_azimuths = range(0, 360, 60)  # 6 directions
    for tp_az in top_azimuths:
        tp_angle = math.radians(tp_az)
        tp_pos = np.array([
            local_center[0] + top_h_dist * math.cos(tp_angle),
            local_center[1] + top_h_dist * math.sin(tp_angle),
            float(top_focus[2]) - top_h_delta,
        ])
        add_candidate(tp_pos, top_focus, "tower_top", aim_type="tower_top", source="aimtype_tower_top")

    # ── Tower body (塔身): lightweight ring ──
    body_profile = AIMTYPE_VIEW_PROFILE["tower_body"]
    body_dist = body_profile["preferred_distance_m"]
    body_pitch_rad = math.radians(body_profile["preferred_pitch_deg"])
    body_h_dist = body_dist * math.cos(body_pitch_rad)
    body_h_delta = body_dist * math.sin(body_pitch_rad)
    body_heights = [z_min + 0.35 * tower_height, z_min + 0.55 * tower_height, z_min + 0.75 * tower_height]
    for bh in body_heights:
        for ba in range(0, 360, 90):  # 4 per height
            ba_rad = math.radians(ba)
            bp = np.array([
                local_center[0] + body_h_dist * math.cos(ba_rad),
                local_center[1] + body_h_dist * math.sin(ba_rad),
                bh - body_h_delta,
            ])
            add_candidate(bp, tower_focus, "tower_body", aim_type="tower_body", source="aimtype_tower_body")

    # --- Determine insulator mode ---
    insulator_instances_list = list(meta.get("insulator_instances", []) or [])
    insulator_mode = "instance_based" if insulator_instances_list else "legacy_voxel_based"
    legacy_insulator_voxel_candidates_skipped = bool(insulator_instances_list)
    insulator_instance_centers = [
        np.asarray(inst.get("center", local_center), dtype=float)
        for inst in insulator_instances_list if isinstance(inst, dict)
    ]
    meta["insulator_candidate_mode"] = insulator_mode
    meta["legacy_insulator_voxel_candidates_skipped"] = legacy_insulator_voxel_candidates_skipped

    # --- Aggregate tower voxels by patch key (retained for coverage targets, not per-cell generation) ---
    tower_patches: Dict[Tuple[str, int, int], Dict[str, object]] = {}
    attention_records: List[Dict[str, object]] = []
    legacy_insulator_records: List[Dict[str, object]] = []
    for record in target_records:
        semantic = _decode_text(record.get("semantic"), "tower_body")
        if semantic in ATTENTION_SEMANTICS:
            attention_records.append(record)
            continue
        if semantic == "insulator":
            if insulator_mode == "instance_based":
                continue
            else:
                legacy_insulator_records.append(record)
                continue
        if semantic not in TARGET_SEMANTICS:
            continue
        layer_id = int(record.get("layer_id", 0) or 0)
        sector_id = int(record.get("sector_id", 0) or 0)
        patch_key = (semantic, layer_id, sector_id)
        if patch_key not in tower_patches:
            tower_patches[patch_key] = {
                "semantic": semantic,
                "layer_id": layer_id,
                "sector_id": sector_id,
                "coords": [],
                "weights": [],
                "required_resolution": [],
                "max_view_angle_deg": [],
                "target_ids": [],
            }
        patch = tower_patches[patch_key]
        patch["coords"].append(np.asarray(record.get("coord", local_center), dtype=float))
        patch["weights"].append(float(record.get("weight", 1.0)))
        patch["required_resolution"].append(float(record.get("required_resolution", REQUIRED_RESOLUTION.get(semantic, 0.7))))
        patch["max_view_angle_deg"].append(float(record.get("max_view_angle_deg",
                                            INCIDENCE_THRESHOLDS.get(semantic, 65.0))))
        patch["target_ids"].append(record.get("id"))

    # --- Tower patch candidates: 1-2 per layer-sector, AIMTYPE_VIEW_PROFILE driven ---
    tower_raw_by_source: Dict[str, int] = {}
    # Map tower semantics to aim_type keys
    _tower_aimtype_map = {
        "tower_top": "tower_top", "tower_edge": "tower_edge",
        "tower_body": "tower_body", "tower_base_connection": "tower_base",
    }
    for patch_key, patch in tower_patches.items():
        semantic = str(patch["semantic"])
        aim_key = _tower_aimtype_map.get(semantic, "tower_body")
        profile = AIMTYPE_VIEW_PROFILE.get(aim_key)
        if profile is None:
            continue
        max_per_patch = int(profile.get("max_candidates_per_layer_sector", 3))
        coords = np.asarray(patch["coords"], dtype=float)
        weights = np.asarray(patch["weights"], dtype=float)
        req_res = float(np.mean(np.asarray(patch["required_resolution"], dtype=float))) if patch.get("required_resolution") else REQUIRED_RESOLUTION.get(semantic, 0.7)
        n_patch_cells = len(coords)
        if n_patch_cells == 0:
            continue
        # Use patch centroid as representative target
        centroid = np.mean(coords, axis=0)
        outward = centroid[:2] - local_center[:2]
        norm = float(np.linalg.norm(outward))
        if norm <= 0.1:
            continue
        outward = outward / norm
        cluster_id = target_cluster_id(semantic, centroid)
        tgt_weight = float(np.mean(weights))
        pref_dist = float(profile.get("preferred_distance_m", 55.0))
        pref_pitch = profile.get("preferred_pitch_deg")

        for ci in range(max_per_patch):
            # Gentle angle variation for multi-candidate patches
            if max_per_patch > 1 and ci > 0:
                angle_shift = [-8.0, 8.0, -16.0, 16.0, -24.0, 24.0]
                rot_deg = angle_shift[min(ci - 1, len(angle_shift) - 1)]
                rot_rad = math.radians(rot_deg)
                rot_out = np.array([
                    outward[0] * math.cos(rot_rad) - outward[1] * math.sin(rot_rad),
                    outward[0] * math.sin(rot_rad) + outward[1] * math.cos(rot_rad),
                ])
            else:
                rot_out = outward.copy()

            if pref_pitch is not None:
                pitch_rad = math.radians(float(pref_pitch))
                h_dist = pref_dist * math.cos(pitch_rad)
                pz = float(centroid[2]) - pref_dist * math.sin(pitch_rad)
            else:
                h_dist = pref_dist
                pz = float(centroid[2])

            pos = np.array([
                centroid[0] + rot_out[0] * h_dist,
                centroid[1] + rot_out[1] * h_dist,
                pz,
            ])
            add_candidate(pos, centroid, semantic,
                          aim_type=aim_key,
                          target_weight=tgt_weight,
                          required_resolution=req_res,
                          layer_id=int(patch["layer_id"]),
                          sector_id=int(patch["sector_id"]),
                          source="tower_patch_aimtype")
        tower_raw_by_source[semantic] = tower_raw_by_source.get(semantic, 0) + 1

    # --- Legacy insulator voxel generation (fallback, only when no insulator_instances) ---
    if legacy_insulator_records and insulator_mode == "legacy_voxel_based":
        max_legacy_ins = min(len(legacy_insulator_records), 200)
        if len(legacy_insulator_records) > max_legacy_ins:
            legacy_insulator_records = legacy_insulator_records[:max_legacy_ins]
        ins_profile = AIMTYPE_VIEW_PROFILE.get("insulator_string", {})
        ins_dist = float(ins_profile.get("preferred_distance_m", 16.0))
        for record in legacy_insulator_records:
            target = np.asarray(record.get("coord", local_center), dtype=float)
            outward = target[:2] - local_center[:2]
            norm = float(np.linalg.norm(outward))
            if norm <= 1e-6:
                continue
            outward = outward / norm
            cluster_id = target_cluster_id("insulator", target)
            pos = np.array([
                target[0] + outward[0] * ins_dist,
                target[1] + outward[1] * ins_dist,
                target[2],
            ])
            add_candidate(pos, target, "insulator", aim_type="insulator_string",
                          cluster_id=cluster_id, target_id=record.get("id"),
                          target_weight=record.get("weight"),
                          required_resolution=record.get("required_resolution"),
                          max_view_angle_deg=record.get("max_view_angle_deg"),
                          instance_id=record.get("insulator_instance_id", record.get("instance_id")),
                          source="legacy_insulator_voxel")
            if conductor_no_fly_volumes:
                add_no_fly_exterior_candidates(target, "insulator", cluster_id)
        tower_raw_by_source["insulator_legacy_voxel"] = len(legacy_insulator_records)

    # --- Attention / connection candidates: ≤3 per target, AIMTYPE_VIEW_PROFILE driven ---
    # Combine former "attention_target" and "connection_angled" into single pass.
    # Map attention semantics to aim_type keys and their multi-angle profiles.
    _attn_aimtype_map = {
        "conductor_insulator_connection": "conductor_insulator_connection",
        "wire_insulator_connection": "conductor_insulator_connection",
        "insulator_tower_side_connection": "insulator_tower_side_connection",
        "ground_wire_tower_connection": "ground_wire_tower_connection",
        "tower_base_connection": "tower_base",
    }
    # Priority order: three key connection types first, then tower_base
    att_priority_order = ["conductor_insulator_connection", "insulator_tower_side_connection", "ground_wire_tower_connection", "tower_base_connection"]
    att_by_semantic: Dict[str, List[Dict[str, object]]] = {s: [] for s in att_priority_order}
    for record in attention_records:
        semantic = _normalize_semantic(_decode_text(record.get("semantic"), ""))
        if semantic in att_by_semantic:
            att_by_semantic[semantic].append(record)

    # Helper: find the local wire direction at a target point for side-view placement
    def _wire_direction_2d(pt: np.ndarray, curves: list, fallback: np.ndarray) -> np.ndarray:
        best_dir = fallback
        best_dist = float('inf')
        for curve in curves:
            polyline = curve.get("polyline", [])
            if not isinstance(polyline, list) or len(polyline) < 2:
                continue
            poly_arr = np.asarray(polyline, dtype=float)
            for i in range(len(poly_arr) - 1):
                seg_start = poly_arr[i]
                seg_end = poly_arr[i + 1]
                seg_vec = seg_end[:2] - seg_start[:2]
                seg_len = float(np.linalg.norm(seg_vec))
                if seg_len < 1e-3:
                    continue
                seg_dir = seg_vec / seg_len
                t = max(0.0, min(1.0, float(np.dot(pt[:2] - seg_start[:2], seg_dir) / seg_len)))
                closest = seg_start[:2] + t * seg_vec
                dist = float(np.linalg.norm(closest - pt[:2]))
                if dist < best_dist:
                    best_dist = dist
                    best_dir = seg_dir
        return best_dir

    for semantic in att_priority_order:
        records = att_by_semantic.get(semantic, [])
        aim_key = _attn_aimtype_map.get(semantic, "tower_body")
        profile = AIMTYPE_VIEW_PROFILE.get(aim_key)
        if profile is None or not records:
            continue
        max_per_target = int(profile.get("max_candidates_per_target", 3))
        pref_dist = float(profile.get("preferred_distance_m", 5.0))
        pref_pitch = profile.get("preferred_pitch_deg")
        if pref_pitch is not None:
            p_rad = math.radians(float(pref_pitch))
            h_dist = pref_dist * math.cos(p_rad)
            pz_offset = -pref_dist * math.sin(p_rad)  # P_z above T for negative pitch
        else:
            h_dist = pref_dist
            pz_offset = 0.0
        # Wire-proximal types: use outward direction with wider angle sweep for
        # incidence alignment (target normal ≈ outward) while finding safe lateral offsets
        _use_angle_sweep = semantic in ("conductor_insulator_connection", "wire_insulator_connection", "insulator_tower_side_connection", "ground_wire_tower_connection")
        att_count = 0
        for record in records:
            target = np.asarray(record.get("coord", local_center), dtype=float)
            outward = target[:2] - local_center[:2]
            out_norm = float(np.linalg.norm(outward))
            if out_norm <= 1e-6:
                continue
            outward = outward / out_norm
            cluster_id = target_cluster_id(semantic, target)

            if _use_angle_sweep:
                # Sweep from outward (best incidence, worst safety) to side (worse incidence, safer)
                angle_offsets = [0.0, -12.0, 12.0, -24.0, 24.0, -36.0, 36.0, -48.0, 48.0]
            else:
                angle_offsets = [0.0]
                if max_per_target >= 2:
                    angle_offsets.extend([-8.0, 8.0])
                if max_per_target >= 4:
                    angle_offsets.extend([-16.0, 16.0])

            candidates_for_this_target = 0
            # --- Pass 1: preferred distance ---
            for aoff in angle_offsets:
                if candidates_for_this_target >= max_per_target:
                    break
                aoff_rad = math.radians(aoff)
                rot_dir = np.array([
                    outward[0] * math.cos(aoff_rad) - outward[1] * math.sin(aoff_rad),
                    outward[0] * math.sin(aoff_rad) + outward[1] * math.cos(aoff_rad),
                ])
                pos = np.array([
                    target[0] + rot_dir[0] * h_dist,
                    target[1] + rot_dir[1] * h_dist,
                    float(target[2]) + pz_offset,
                ])
                count_before = len(candidates)
                add_candidate(pos, target, semantic,
                              aim_type=aim_key,
                              cluster_id=cluster_id,
                              target_id=record.get("id"),
                              target_weight=record.get("weight"),
                              required_resolution=record.get("required_resolution"),
                              max_view_angle_deg=record.get("max_view_angle_deg"),
                              instance_id=record.get("instance_id"),
                              layer_id=record.get("layer_id"),
                              sector_id=record.get("sector_id"),
                              source="connection_aimtype")
                if len(candidates) > count_before:
                    candidates_for_this_target += 1

            # --- Pass 2: larger distance for stubborn targets (more lateral clearance) ---
            if candidates_for_this_target == 0 and _use_angle_sweep:
                dist_range = profile.get("distance_range_m", (pref_dist, pref_dist))
                larger_dist = min(pref_dist * 1.15, float(dist_range[1]))
                if larger_dist > pref_dist + 0.5:
                    l_rad = math.radians(float(pref_pitch or -15.0))
                    l_h_dist = larger_dist * math.cos(l_rad)
                    l_pz = -larger_dist * math.sin(l_rad)
                    for aoff in angle_offsets:
                        if candidates_for_this_target >= max_per_target:
                            break
                        aoff_rad = math.radians(aoff)
                        rot_dir = np.array([
                            outward[0] * math.cos(aoff_rad) - outward[1] * math.sin(aoff_rad),
                            outward[0] * math.sin(aoff_rad) + outward[1] * math.cos(aoff_rad),
                        ])
                        pos2 = np.array([
                            target[0] + rot_dir[0] * l_h_dist,
                            target[1] + rot_dir[1] * l_h_dist,
                            float(target[2]) + l_pz,
                        ])
                        count_before = len(candidates)
                        add_candidate(pos2, target, semantic,
                                      aim_type=aim_key,
                                      cluster_id=cluster_id,
                                      target_id=record.get("id"),
                                      target_weight=record.get("weight"),
                                      required_resolution=record.get("required_resolution"),
                                      max_view_angle_deg=record.get("max_view_angle_deg"),
                                      instance_id=record.get("instance_id"),
                                      layer_id=record.get("layer_id"),
                                      sector_id=record.get("sector_id"),
                                      source="connection_aimtype")
                        if len(candidates) > count_before:
                            candidates_for_this_target += 1

            # Fallback: if no candidates passed safety, try no_fly_exterior
            if candidates_for_this_target == 0 and _use_angle_sweep:
                add_no_fly_exterior_candidates(target, semantic, cluster_id)
            att_count += 1
        tower_raw_by_source[semantic] = att_count

    # --- Insulator instance candidates: ≤6 per instance, AIMTYPE_VIEW_PROFILE driven ---
    if insulator_instances_list and insulator_mode == "instance_based":
        max_per_inst = int(INSULATOR_INSTANCE_VIEW_CONFIG.get("max_candidates_per_instance", 30))
        direction_names = list(INSULATOR_INSTANCE_VIEW_CONFIG.get("directions", [])) or [
            "outward",
            "outward_plus_line",
            "outward_minus_line",
            "outward_plus_z",
            "outward_minus_z",
        ]
        distances = [float(value) for value in INSULATOR_INSTANCE_VIEW_CONFIG.get("distances", [4.5, 6.0, 7.5])]
        height_offsets = [float(value) for value in INSULATOR_INSTANCE_VIEW_CONFIG.get("height_offsets", [0.0])]
        yaw_offsets_deg = [float(value) for value in INSULATOR_INSTANCE_VIEW_CONFIG.get("yaw_offsets_deg", [0.0])]
        inst_count = 0
        for inst_idx, inst in enumerate(insulator_instances_list):
            if not isinstance(inst, dict):
                continue
            frag_type = str(inst.get("fragment_type", ""))
            if frag_type in ("noise_fragment", "excluded_from_candidates"):
                continue
            instance_id = int(inst.get("id", inst_idx) or inst_idx)
            inst_center = np.asarray(inst.get("center", local_center), dtype=float)
            inst_axis = safe_normalize(np.asarray(inst.get("axis", [0.0, 0.0, 1.0]), dtype=float))
            outward_2d = safe_normalize(inst_center[:2] - local_center[:2])

            # Direction vectors (up to 5 angles, capped by max_per_inst)
            dir_map = {
                "outward": np.array([outward_2d[0], outward_2d[1], 0.0]),
                "outward_plus_line": safe_normalize(np.array([
                    outward_2d[0] + line_dir[0] * 0.5,
                    outward_2d[1] + line_dir[1] * 0.5, 0.0])),
                "outward_minus_line": safe_normalize(np.array([
                    outward_2d[0] - line_dir[0] * 0.5,
                    outward_2d[1] - line_dir[1] * 0.5, 0.0])),
                "outward_plus_z": safe_normalize(np.array([outward_2d[0], outward_2d[1], 0.3])),
                "outward_minus_z": safe_normalize(np.array([outward_2d[0], outward_2d[1], -0.3])),
            }
            inst_candidates = 0
            for dir_name in direction_names:
                base_dir = safe_normalize(dir_map.get(dir_name, np.array([outward_2d[0], outward_2d[1], 0.0])))
                for distance in distances:
                    for height_offset in height_offsets:
                        for yaw_offset_deg in yaw_offsets_deg:
                            if inst_candidates >= max_per_inst:
                                break
                            yaw_rad = math.radians(float(yaw_offset_deg))
                            rot_dir = np.array([
                                base_dir[0] * math.cos(yaw_rad) - base_dir[1] * math.sin(yaw_rad),
                                base_dir[0] * math.sin(yaw_rad) + base_dir[1] * math.cos(yaw_rad),
                                base_dir[2],
                            ])
                            rot_dir = safe_normalize(rot_dir)
                            target = inst_center.copy()
                            target[2] += float(height_offset)
                            pos = np.array([
                                target[0] + rot_dir[0] * float(distance),
                                target[1] + rot_dir[1] * float(distance),
                                target[2] + rot_dir[2] * float(distance),
                            ])
                            count_before = len(candidates)
                            add_candidate(pos, target, "insulator",
                                          aim_type="insulator_string",
                                          target_weight=POWER_MODEL_CONFIG["insulator_weight"],
                                          required_resolution=REQUIRED_RESOLUTION.get("insulator", 1.5),
                                          max_view_angle_deg=POWER_MODEL_CONFIG["insulator_max_view_angle_deg"],
                                          instance_id=instance_id,
                                          layer_id=None, sector_id=None,
                                          source="insulator_instance_multi_distance")
                            if len(candidates) > count_before:
                                inst_candidates += 1
                        if inst_candidates >= max_per_inst:
                            break
                    if inst_candidates >= max_per_inst:
                        break
                if inst_candidates >= max_per_inst:
                    break
            inst_count += 1
        tower_raw_by_source["insulator_instance"] = inst_count

    if manual_route_path:
        for waypoint in parse_manual_route(str(manual_route_path)):
            aim_type = _decode_text(waypoint.get("aim_type"), "")
            semantic, priority = classify_manual_aim_type(aim_type)
            if priority < 0.30:
                continue
            raw_position = waypoint.get("pos_utm", waypoint.get("position"))
            if raw_position is None:
                continue
            position = np.asarray(raw_position, dtype=float)
            target = np.asarray(waypoint.get("target_utm", position), dtype=float)
            if len(position) < 3 or len(target) < 3:
                continue
            lateral = normalize_vector([target[1] - position[1], position[0] - target[0], 0.0])
            offsets = (
                np.zeros(3, dtype=float),
                1.2 * lateral,
                -1.2 * lateral,
                np.array([0.0, 0.0, 0.8], dtype=float),
                np.array([0.0, 0.0, -0.8], dtype=float),
            )
            for offset in offsets:
                candidate_position = position + offset
                add_candidate(
                    candidate_position,
                    target,
                    semantic,
                    manual_priority=priority,
                    source="manual_route_prior",
                    aim_type=aim_type,
                )

    # --- Candidate thinning by semantic priority ---
    thinning_applied = False
    thinning_removed = 0
    if len(candidates) > MAX_CANDIDATES_TOTAL:
        thinning_applied = True
        thinning_priority = {
            "conductor_insulator_connection": 0,
            "wire_insulator_connection": 0,
            "insulator_tower_side_connection": 0,
            "ground_wire_tower_connection": 1,
            "tower_base_connection": 2,
            "insulator": 3,
            "tower_top": 4,
            "tower_edge": 5,
            "tower_body": 6,
            "tower_lower30": 7,
        }
        # Sort by priority (lower = keep first)
        candidates.sort(key=lambda c: (
            thinning_priority.get(_normalize_semantic(c.get("semantic_focus", "")), 10),
            -float(c.get("weight", 0.0)),
        ))
        thinning_removed = len(candidates) - MAX_CANDIDATES_TOTAL
        candidates = candidates[:MAX_CANDIDATES_TOTAL]
        filter_stats["thinning_applied"] = True
        filter_stats["thinning_removed"] = thinning_removed
        filter_stats["final_candidate_count"] = len(candidates)
    meta["candidate_thinning"] = {
        "applied": thinning_applied,
        "removed": thinning_removed,
        "max_total": MAX_CANDIDATES_TOTAL,
    }

    candidates.sort(
        key=lambda item: (
            -float(item.get("manual_priority", 0.0)),
            -SEMANTIC_PRIORITY.get(item["semantic_focus"], 0),
            item["position_id"],
            item["f_eq_mm"],
        )
    )
    for idx, candidate in enumerate(candidates, start=1):
        candidate["id"] = idx

    meta["candidate_filter_stats"] = filter_stats

    # Build per-source final counts
    final_by_source: Dict[str, int] = {}
    for c in candidates:
        src = str(c.get("source", "unknown"))
        final_by_source[src] = final_by_source.get(src, 0) + 1
    meta["candidate_generation_stats"] = {
        "raw_by_source": tower_raw_by_source,
        "rejected_by_safety": {
            "inside_no_fly": filter_stats.get("inside_no_fly_rejected", 0),
            "clearance": filter_stats.get("clearance_rejected", 0),
            "wire_curve": filter_stats.get("wire_curve_rejected", 0),
            "tower_safety": filter_stats.get("tower_safety_rejected", 0),
        },
        "final_by_source": final_by_source,
    }
    surface_model["meta"] = meta

    return candidates


def parse_manual_route(json_path: str) -> List[Dict[str, object]]:
    raw = Path(json_path).read_bytes()
    last_error = None
    data = None
    for encoding in ("utf-8-sig", "utf-8", "gb18030", "gbk"):
        try:
            data = json.loads(raw.decode(encoding))
            break
        except Exception as exc:
            last_error = exc
    if data is None:
        raise ValueError(f"无法解析人工航线 JSON: {last_error}")

    points = data.get("points", [])
    if not points and "towers" in data:
        for tower in data.get("towers", []):
            points.extend(tower.get("points", []))

    waypoints = []
    for idx, point in enumerate(points, start=1):
        lat = point.get("latitude", point.get("lat"))
        lon = point.get("longitude", point.get("lon"))
        if lat is None or lon is None:
            continue
        try:
            lat = float(lat)
            lon = float(lon)
            easting, northing, _, _ = utm.from_latlon(lat, lon)
        except Exception:
            continue

        focal_level = point.get("focalLevel") or point.get("focal_level")
        focal_mm = point.get("focalLength") or point.get("focal_length") or point.get("f_eq_mm")
        focal_key, focal_cfg = normalize_focal_level(focal_level, focal_mm)
        target_utm = None
        target_lat = point.get("latitude_Aim", point.get("lat_Aim", point.get("target_latitude")))
        target_lon = point.get("longitude_Aim", point.get("lon_Aim", point.get("target_longitude")))
        if target_lat is not None and target_lon is not None:
            try:
                target_easting, target_northing, _, _ = utm.from_latlon(float(target_lat), float(target_lon))
                target_utm = [
                    target_easting,
                    target_northing,
                    _safe_float(point.get("altitude_Aim", point.get("target_altitude", point.get("altitude", 0.0)))),
                ]
            except Exception:
                target_utm = None
        aim_type = point.get("AimType", point.get("aim_type", ""))
        semantic_focus, manual_priority = classify_manual_aim_type(aim_type)

        waypoint = {
            "id": idx,
            "pos_utm": [easting, northing, _safe_float(point.get("altitude", point.get("height", 0.0)))],
            "pitch": _safe_float(point.get("pitch", point.get("gimbalPitch", 0.0))),
            "yaw": _safe_float(point.get("heading", point.get("yaw", 0.0))),
            "focal_level": focal_key if focal_level or focal_mm is not None else None,
            "f_eq_mm": focal_cfg["f_eq_mm"] if focal_level or focal_mm is not None else None,
            "action": point.get("actionName", "fly"),
            "aim_type": aim_type,
            "semantic_focus": semantic_focus,
            "manual_priority": manual_priority,
            "point_type": point.get("point_type", "task" if point.get("actionName") == "photo" else "auxiliary"),
            "source_waypoint_id": point.get("SourceWaypointId", point.get("source_waypoint_id")),
        }
        if target_utm is not None:
            waypoint["target_utm"] = target_utm
        waypoints.append(waypoint)
    return waypoints


@dataclass
class CandidateViewpoint:
    id: int
    position_id: int
    position: np.ndarray
    pitch: float
    yaw: float
    focal_level: str
    f_eq_mm: float
    hfov_deg: float
    vfov_deg: float
    semantic_focus: str
    target_center: np.ndarray
    base_score: float = 0.0
    manual_priority: float = 0.0
    safety_distance_m: Optional[float] = None
    source: str = "semantic_surface"
    position_z_ratio: Optional[float] = None
    target_z_ratio: Optional[float] = None
    target_azimuth_deg: Optional[float] = None
    target_cluster_id: Optional[str] = None
    # ── New geometry/focal fields ──
    look_at: Optional[np.ndarray] = None
    distance: Optional[float] = None
    heading: Optional[float] = None
    base_heading: Optional[float] = None
    yaw_offset_deg: float = 0.0
    focal_length_eq_mm: Optional[float] = None
    action_name: str = "photo"
    aim_type: Optional[str] = None
    target_id: Optional[int] = None
    instance_id: Optional[int] = None
    layer_id: Optional[int] = None
    sector_id: Optional[int] = None
    weight: Optional[float] = None
    no_fly_checked: bool = False
    safety_checked: bool = False
    no_fly_clearance_m: Optional[float] = None
    no_fly_required_clearance_m: Optional[float] = None
    required_resolution: Optional[float] = None


def load_candidate_views(cand_path: str) -> List[CandidateViewpoint]:
    with open(cand_path, "r", encoding="utf-8", errors="ignore") as file:
        raw_candidates = json.load(file)

    candidates: List[CandidateViewpoint] = []
    for idx, item in enumerate(raw_candidates, start=1):
        focal_level, focal_cfg = normalize_focal_level(item.get("focal_level"), item.get("f_eq_mm"))
        # Read new geometry/focal fields with backward-compat defaults
        look_at_raw = item.get("look_at")
        look_at = np.asarray(look_at_raw, dtype=float) if look_at_raw else None
        heading = float(item.get("heading", item.get("yaw", 0.0)))
        candidates.append(
            CandidateViewpoint(
                id=int(item.get("id", idx)),
                position_id=int(item.get("position_id", idx)),
                position=np.asarray(item.get("utm_position", item.get("position")), dtype=float),
                pitch=float(item.get("pitch", 0.0)),
                yaw=float(item.get("yaw", 0.0)),
                focal_level=focal_level,
                f_eq_mm=float(item.get("f_eq_mm", focal_cfg["f_eq_mm"])),
                hfov_deg=float(item.get("hfov_deg", focal_cfg["hfov_deg"])),
                vfov_deg=float(item.get("vfov_deg", focal_cfg["vfov_deg"])),
                semantic_focus=_decode_text(item.get("semantic_focus"), "tower_body"),
                target_center=np.asarray(item.get("target_center", item.get("utm_position", [0.0, 0.0, 0.0])), dtype=float),
                manual_priority=float(item.get("manual_priority", 0.0) or 0.0),
                safety_distance_m=(
                    float(item["safety_distance_m"])
                    if item.get("safety_distance_m") is not None
                    else None
                ),
                source=_decode_text(item.get("source"), "semantic_surface"),
                position_z_ratio=(
                    float(item["position_z_ratio"])
                    if item.get("position_z_ratio") is not None
                    else None
                ),
                target_z_ratio=(
                    float(item["target_z_ratio"])
                    if item.get("target_z_ratio") is not None
                    else None
                ),
                target_azimuth_deg=(
                    float(item["target_azimuth_deg"])
                    if item.get("target_azimuth_deg") is not None
                    else None
                ),
                target_cluster_id=_decode_text(
                    item.get("target_cluster_id"),
                    f"{_decode_text(item.get('semantic_focus'), 'tower_body')}_default",
                ),
                # ── New fields ──
                look_at=look_at,
                distance=float(item["Distance"]) if item.get("Distance") is not None else None,
                heading=heading,
                base_heading=float(item.get("base_heading", heading)),
                yaw_offset_deg=float(item.get("yaw_offset_deg", 0.0)),
                focal_length_eq_mm=float(item["focal_length_eq_mm"]) if item.get("focal_length_eq_mm") is not None else None,
                action_name=str(item.get("actionName", "photo")),
                aim_type=_decode_text(item.get("AimType"), None),
                target_id=int(item["target_id"]) if item.get("target_id") is not None else None,
                instance_id=(
                    int(item["instance_id"])
                    if item.get("instance_id") is not None
                    else int(item["insulator_instance_id"]) if item.get("insulator_instance_id") is not None else None
                ),
                layer_id=int(item["layer_id"]) if item.get("layer_id") is not None else None,
                sector_id=int(item["sector_id"]) if item.get("sector_id") is not None else None,
                weight=float(item["weight"]) if item.get("weight") is not None else None,
                no_fly_checked=bool(item.get("no_fly_checked", False)),
                safety_checked=bool(item.get("safety_checked", False)),
                no_fly_clearance_m=float(item["no_fly_clearance_m"]) if item.get("no_fly_clearance_m") is not None else None,
                no_fly_required_clearance_m=(
                    float(item["no_fly_required_clearance_m"])
                    if item.get("no_fly_required_clearance_m") is not None
                    else None
                ),
                required_resolution=(
                    float(item["required_resolution"])
                    if item.get("required_resolution") is not None
                    else None
                ),
            )
        )
    return candidates


def _normalize_voxel_record(raw_voxel, local_center: np.ndarray, z_min: float, z_max: float) -> Dict[str, object]:
    if isinstance(raw_voxel, dict):
        record = dict(raw_voxel)
    else:
        names = raw_voxel.dtype.names or ()
        record = {name: raw_voxel[name].item() if hasattr(raw_voxel[name], "item") else raw_voxel[name] for name in names}

    coord = np.asarray(record.get("coord", record.get("pos", [0.0, 0.0, 0.0])), dtype=float)
    label = int(record.get("label", TOWER_LABEL))
    semantic = _decode_text(record.get("semantic"), "").strip().lower()
    if semantic not in ALL_SEMANTICS:
        category = _decode_text(record.get("category"), "tower").strip().lower()
        if category == "insulator" or label == INSULATOR_LABEL:
            semantic = "insulator"
        else:
            z_ratio = float((coord[2] - z_min) / max(z_max - z_min, 1e-6))
            voxel_type = int(record.get("type", 1) or 1)
            if z_ratio < 0.40:
                semantic = "tower_lower30"
            elif z_ratio >= 0.78:
                semantic = "tower_top"
            else:
                semantic = "tower_edge" if voxel_type == 2 else "tower_body"

    normal_hint = record.get("normal_hint")
    if normal_hint is None:
        normal_hint = [coord[0] - local_center[0], coord[1] - local_center[1], 0.0]

    return {
        "coord": coord.tolist(),
        "id": record.get("id"),
        "label": label,
        "category": "insulator" if semantic == "insulator" else "tower",
        "semantic": semantic,
        "is_target": bool(record.get("is_target", semantic in TARGET_SEMANTICS)),
        "is_attention_target": bool(record.get("is_attention_target", semantic in ATTENTION_SEMANTICS)),
        "weight": float(record.get("weight", SEMANTIC_WEIGHTS.get(semantic, 1.0))),
        "required_resolution": float(record.get("required_resolution", REQUIRED_RESOLUTION.get(semantic, 0.7))),
        "incidence_max_deg": float(record.get("incidence_max_deg", INCIDENCE_THRESHOLDS.get(semantic, 65.0))),
        "normal_hint": normalize_vector(normal_hint).tolist(),
        "instance_id": record.get("instance_id", record.get("insulator_instance_id")),
        "insulator_instance_id": record.get("insulator_instance_id", record.get("instance_id")),
        "layer_id": record.get("layer_id"),
        "sector_id": record.get("sector_id"),
    }


class PlanningEnvironment:
    def __init__(self, voxel_path: str):
        data = np.load(voxel_path, allow_pickle=True)
        raw_voxels = data["voxels"]
        coords_preview = []
        for raw_voxel in raw_voxels:
            if isinstance(raw_voxel, dict):
                coords_preview.append(np.asarray(raw_voxel.get("coord", [0.0, 0.0, 0.0]), dtype=float))
            else:
                coords_preview.append(np.asarray(raw_voxel["coord"], dtype=float))
        coords_preview = np.asarray(coords_preview, dtype=float) if coords_preview else np.zeros((0, 3), dtype=float)
        local_center = (
            np.asarray(data["local_center"], dtype=float)
            if "local_center" in data.files
            else (np.mean(coords_preview, axis=0) if len(coords_preview) else np.zeros(3, dtype=float))
        )
        z_min = float(np.min(coords_preview[:, 2])) if len(coords_preview) else 0.0
        z_max = float(np.max(coords_preview[:, 2])) if len(coords_preview) else 0.0

        self.voxel_records = [
            _normalize_voxel_record(raw_voxel, local_center, z_min, z_max)
            for raw_voxel in raw_voxels
        ]
        raw_attention = data["attention_targets"] if "attention_targets" in data.files else []
        self.attention_records = [
            _normalize_voxel_record(raw_voxel, local_center, z_min, z_max)
            for raw_voxel in raw_attention
        ]
        self.target_records = [record for record in self.voxel_records if record["is_target"] and record["semantic"] in TARGET_SEMANTICS]
        self.target_coords = np.array([record["coord"] for record in self.target_records], dtype=float)
        self.semantics = np.array([record["semantic"] for record in self.target_records], dtype="<U32")
        self.weights = np.array([record["weight"] for record in self.target_records], dtype=float)
        self.required_resolution = np.array([record["required_resolution"] for record in self.target_records], dtype=float)
        self.incidence_max = np.array([record["incidence_max_deg"] for record in self.target_records], dtype=float)
        self.normal_hints = np.array([record["normal_hint"] for record in self.target_records], dtype=float)
        self.attention_coords = (
            np.array([record["coord"] for record in self.attention_records], dtype=float)
            if self.attention_records
            else np.empty((0, 3), dtype=float)
        )
        self.attention_semantics = np.array([record["semantic"] for record in self.attention_records], dtype="<U48")
        self.attention_weights = np.array([record["weight"] for record in self.attention_records], dtype=float)
        self.attention_required_resolution = np.array(
            [record["required_resolution"] for record in self.attention_records],
            dtype=float,
        )
        self.attention_incidence_max = np.array([record["incidence_max_deg"] for record in self.attention_records], dtype=float)
        self.attention_normal_hints = np.array([record["normal_hint"] for record in self.attention_records], dtype=float)
        self.z_max_map = np.asarray(data["z_max_map"], dtype=float)
        self.min_bound = np.asarray(data["min_bound"], dtype=float)
        self.cell_size = float(data["voxel_size"])
        self.local_center = np.asarray(data["local_center"], dtype=float) if "local_center" in data.files else (
            np.mean(self.target_coords, axis=0) if len(self.target_coords) else np.zeros(3, dtype=float)
        )
        self.total_weight = float(np.sum(self.weights)) if len(self.weights) else 0.0
        self.semantic_totals = {semantic: int(np.sum(self.semantics == semantic)) for semantic in TARGET_SEMANTICS}
        self.weight_totals = {
            semantic: float(np.sum(self.weights[self.semantics == semantic])) for semantic in TARGET_SEMANTICS
        }
        self.attention_totals = {
            semantic: int(np.sum(self.attention_semantics == semantic)) for semantic in ATTENTION_SEMANTICS
        }
        self.safety_points = np.asarray(data["safety_points"], dtype=float) if "safety_points" in data.files else np.empty((0, 3), dtype=float)
        self.safety_labels = np.asarray(data["safety_labels"], dtype=int) if "safety_labels" in data.files else np.empty((0,), dtype=int)
        self.safety_index = VoxelSafetyIndex(self.safety_points, max(DEFAULT_LIMITS["safety_distance_m"], 1.0))
        raw_no_fly = data["conductor_no_fly_volumes"] if "conductor_no_fly_volumes" in data.files else []
        self.conductor_no_fly_volumes = load_conductor_no_fly_volumes(raw_no_fly)
        no_fly_is_manual = any(
            str(getattr(volume, "source", "") or "").lower() == "manual_route"
            for volume in self.conductor_no_fly_volumes
        )
        if (not self.conductor_no_fly_volumes or no_fly_is_manual) and "display_voxels" in data.files:
            wire_coords: List[np.ndarray] = []
            ground_wire_coords: List[np.ndarray] = []
            for raw_voxel in data["display_voxels"]:
                if isinstance(raw_voxel, dict):
                    label = int(raw_voxel.get("label", -1))
                    coord = raw_voxel.get("coord", raw_voxel.get("pos"))
                else:
                    names = raw_voxel.dtype.names or ()
                    label = int(raw_voxel["label"]) if "label" in names else -1
                    coord = raw_voxel["coord"] if "coord" in names else raw_voxel["pos"] if "pos" in names else None
                if coord is None:
                    continue
                if label == WIRE_LABEL:
                    wire_coords.append(np.asarray(coord, dtype=float))
                elif label == GROUND_WIRE_LABEL:
                    ground_wire_coords.append(np.asarray(coord, dtype=float))
            fallback_records = build_conductor_no_fly_volumes_from_point_cloud(
                center=self.local_center,
                wire_points=np.asarray(wire_coords, dtype=float),
                ground_wire_points=np.asarray(ground_wire_coords, dtype=float),
            )
            fallback_volumes = load_conductor_no_fly_volumes(fallback_records)
            if fallback_volumes:
                self.conductor_no_fly_volumes = fallback_volumes
            elif no_fly_is_manual:
                self.conductor_no_fly_volumes = []
        self.conductor_no_fly_source = (
            self.conductor_no_fly_volumes[0].source if self.conductor_no_fly_volumes else None
        )
        self.raw_target_mask = np.ones(self.target_count, dtype=bool)
        self.observable_target_mask = self.raw_target_mask.copy()
        self.effective_target_mask = self.observable_target_mask.copy()
        self.unobservable_target_mask = self.raw_target_mask & ~self.observable_target_mask
        self.target_observability_reason = np.full(
            self.target_count,
            "observable_by_default",
            dtype="<U32",
        )
        self.target_observability_probe_options: Dict[int, List[Dict[str, object]]] = {}
        self.observability_gap_repair_records: List[Dict[str, object]] = []
        self.observability_gap_repair_stats: Dict[str, object] = {
            "observability_gap_repair_raw_attempts": 0,
            "observability_gap_repair_added": 0,
            "observability_gap_repair_by_semantic": {},
            "observability_gap_repair_covered_targets": 0,
        }
        self.raw_total_weight = float(np.sum(self.weights[self.raw_target_mask])) if self.target_count else 0.0
        self.effective_total_weight = float(np.sum(self.weights[self.effective_target_mask])) if self.target_count else 0.0
        self.effective_semantic_totals = {
            semantic: int(np.sum(np.logical_and(self.semantics == semantic, self.effective_target_mask)))
            for semantic in TARGET_SEMANTICS
        }
        self.effective_weight_totals = {
            semantic: float(np.sum(self.weights[np.logical_and(self.semantics == semantic, self.effective_target_mask)]))
            for semantic in TARGET_SEMANTICS
        }
        self.target_observability_stats = self._build_observability_stats(
            safe_candidate_count=0,
            probe_count=0,
        )

    @property
    def target_count(self) -> int:
        return int(len(self.target_coords))

    @property
    def attention_count(self) -> int:
        return int(len(self.attention_coords))

    def min_safety_distance(self, position: Sequence[float], safety_distance_m: float) -> float:
        """Return candidate distance to environment/wire/ground safety voxels."""
        return self.safety_index.min_distance(
            position,
            search_radius_m=max(float(safety_distance_m) * 2.5, 10.0),
        )

    def inside_conductor_no_fly(self, position: Sequence[float], tolerance_m: Optional[float] = None) -> bool:
        """Return True when a position lies inside any conductor no-fly volume."""
        return any(volume.contains(position, tolerance_m=tolerance_m) for volume in self.conductor_no_fly_volumes)

    def min_conductor_no_fly_clearance(self, position: Sequence[float]) -> float:
        """Return approximate clearance to the nearest conductor no-fly volume."""
        if not self.conductor_no_fly_volumes:
            return float("inf")
        return float(min(volume.clearance(position) for volume in self.conductor_no_fly_volumes))

    def _position_safe_for_observability(
        self,
        position: Sequence[float],
        safety_distance_m: float,
        conductor_no_fly_enabled: bool,
        conductor_no_fly_clearance_m: float,
        conductor_no_fly_boundary_tolerance_m: float,
    ) -> bool:
        if conductor_no_fly_enabled and self.inside_conductor_no_fly(
            position,
            tolerance_m=conductor_no_fly_boundary_tolerance_m,
        ):
            return False
        if conductor_no_fly_enabled and self.min_conductor_no_fly_clearance(position) + 1e-9 < conductor_no_fly_clearance_m:
            return False
        return self.min_safety_distance(position, safety_distance_m) + 1e-9 >= safety_distance_m

    def _diagnostic_probe_distances(self, target_index: int) -> List[float]:
        semantic = str(self.semantics[target_index])
        req_resolution = max(float(self.required_resolution[target_index]), 1e-6)
        focal_cfg = SUPPORTED_FOCALS.get("F2") or next(reversed(SUPPORTED_FOCALS.values()))
        max_distance = min(
            DEFAULT_LIMITS["max_visibility_distance_m"] * 0.9,
            max(float(focal_cfg["min_distance_m"]) + 1.0, float(focal_cfg["f_eq_mm"]) / 24.0 * 8.0 / req_resolution * 0.92),
        )
        if semantic == "insulator":
            preferred = (6.0, 8.0, 12.0, 16.0)
        elif semantic in {"tower_top", "tower_edge"}:
            preferred = (8.0, 12.0, 18.0, 24.0)
        else:
            preferred = (10.0, 16.0, 24.0, 32.0)

        min_distance = float(focal_cfg["min_distance_m"]) + 0.25
        distances: List[float] = []
        for distance in preferred:
            bounded = min(max_distance, max(min_distance, float(distance)))
            if all(abs(bounded - existing) > 0.5 for existing in distances):
                distances.append(bounded)
        return distances[:3]

    def _diagnostic_probe_directions(self, target_index: int) -> List[np.ndarray]:
        normal = normalize_vector(self.normal_hints[target_index])
        world_up = np.array([0.0, 0.0, 1.0], dtype=float)
        tangent = np.cross(normal, world_up)
        if np.linalg.norm(tangent) <= 1e-6:
            tangent = np.array([1.0, 0.0, 0.0], dtype=float)
        tangent = normalize_vector(tangent)
        bitangent = normalize_vector(np.cross(tangent, normal))
        return [
            normal,
            normalize_vector(normal + 0.22 * tangent),
            normalize_vector(normal - 0.22 * tangent),
            normalize_vector(normal + 0.18 * bitangent),
            normalize_vector(normal - 0.18 * bitangent),
        ]

    def _diagnostic_probe_record(
        self,
        target_index: int,
        position: np.ndarray,
        yaw: float,
        pitch: float,
        focal_level: str,
        f_eq_mm: float,
        hfov_deg: float,
        vfov_deg: float,
    ) -> Dict[str, object]:
        target = np.asarray(self.target_coords[target_index], dtype=float)
        distance = float(np.linalg.norm(position - target))
        to_camera = position - target
        to_camera_norm = to_camera / max(float(np.linalg.norm(to_camera)), 1e-6)
        normal_hint = normalize_vector(self.normal_hints[target_index])
        incidence = math.degrees(math.acos(float(np.clip(np.dot(to_camera_norm, normal_hint), -1.0, 1.0))))
        resolution = observation_resolution(f_eq_mm, distance, incidence)
        req_resolution = max(float(self.required_resolution[target_index]), 1e-6)
        max_incidence = max(float(self.incidence_max[target_index]), 1e-6)
        safety_distance = self.min_safety_distance(position, DEFAULT_LIMITS["safety_distance_m"])
        no_fly_clearance = self.min_conductor_no_fly_clearance(position)
        finite_clearance = no_fly_clearance if math.isfinite(float(no_fly_clearance)) else 25.0
        semantic = str(self.semantics[target_index])
        preferred_distance = {
            "insulator": 12.0,
            "tower_top": 18.0,
            "tower_edge": 18.0,
            "tower_body": 24.0,
        }.get(semantic, 18.0)
        score = (
            3.0 * min(float(resolution / req_resolution), 2.5)
            + 2.0 * max(0.0, 1.0 - float(incidence / max_incidence))
            + 1.0 / (1.0 + abs(distance - preferred_distance))
            + 0.35 * min(float(finite_clearance) / 20.0, 1.5)
            + 0.20 * min(float(safety_distance) / max(DEFAULT_LIMITS["safety_distance_m"], 1e-6), 2.0)
        )
        clearance_value = None if not math.isfinite(float(no_fly_clearance)) else round(float(no_fly_clearance), 3)
        target_record = self.target_records[target_index] if target_index < len(self.target_records) else {}
        def optional_int(value) -> Optional[int]:
            if value is None:
                return None
            try:
                return int(value)
            except Exception:
                return None
        return {
            "target_id": int(target_index),
            "semantic": semantic,
            "coord": [round(float(v), 3) for v in target],
            "weight": round(float(self.weights[target_index]), 6),
            "instance_id": optional_int(target_record.get("instance_id", target_record.get("insulator_instance_id"))),
            "layer_id": optional_int(target_record.get("layer_id")),
            "sector_id": optional_int(target_record.get("sector_id")),
            "reason": "candidate_generation_gap",
            "probe_position": [round(float(v), 3) for v in position],
            "probe_heading": round(float(yaw), 3),
            "probe_pitch": round(float(pitch), 3),
            "probe_focal_level": focal_level,
            "probe_f_eq_mm": round(f_eq_mm, 3),
            "probe_hfov_deg": round(float(hfov_deg), 3),
            "probe_vfov_deg": round(float(vfov_deg), 3),
            "probe_distance_m": round(distance, 3),
            "best_probe_position": [round(float(v), 3) for v in position],
            "best_probe_heading": round(float(yaw), 3),
            "best_probe_pitch": round(float(pitch), 3),
            "best_probe_distance": round(distance, 3),
            "best_probe_focal_length_eq_mm": round(f_eq_mm, 3),
            "probe_incidence_deg": round(float(incidence), 3),
            "probe_resolution": round(float(resolution), 6),
            "required_resolution": round(float(req_resolution), 6),
            "no_fly_clearance_m": clearance_value,
            "nearest_no_fly_clearance_m": clearance_value,
            "safety_distance_m": round(float(safety_distance), 3),
            "nearest_safety_distance_m": round(float(safety_distance), 3),
            "score": round(float(score), 6),
        }

    def _target_observable_by_diagnostic_probe(
        self,
        engine: "VisibilityEngine",
        target_index: int,
        safety_distance_m: float,
        conductor_no_fly_enabled: bool,
        conductor_no_fly_clearance_m: float,
        conductor_no_fly_boundary_tolerance_m: float,
    ) -> Tuple[List[Dict[str, object]], int]:
        target = np.asarray(self.target_coords[target_index], dtype=float)
        attempts = 0
        valid_probes: List[Dict[str, object]] = []
        aim_type_map = {
            "insulator": "insulator_string",
            "tower_top": "tower_top",
            "tower_edge": "tower_edge",
            "tower_body": "tower_body",
        }
        semantic = str(self.semantics[target_index])
        req_resolution = float(self.required_resolution[target_index])
        for direction in self._diagnostic_probe_directions(target_index):
            for distance in self._diagnostic_probe_distances(target_index):
                attempts += 1
                position = target + direction * float(distance)
                if not self._position_safe_for_observability(
                    position,
                    safety_distance_m=safety_distance_m,
                    conductor_no_fly_enabled=conductor_no_fly_enabled,
                    conductor_no_fly_clearance_m=conductor_no_fly_clearance_m,
                    conductor_no_fly_boundary_tolerance_m=conductor_no_fly_boundary_tolerance_m,
                ):
                    continue
                yaw, pitch = yaw_pitch_to_target(position, target)
                f_eq_mm = choose_focal_length_eq_mm(
                    aim_type=aim_type_map.get(semantic, semantic),
                    distance=float(distance),
                    required_resolution=req_resolution,
                )
                if f_eq_mm is None:
                    f_eq_mm = 48.0
                f_eq_mm = max(24.0, min(84.0, float(f_eq_mm)))
                focal_level, _ = normalize_focal_level(None, f_eq_mm)
                hfov_deg = math.degrees(2.0 * math.atan(CAMERA_SENSOR_WIDTH_MM / (2.0 * f_eq_mm)))
                vfov_deg = math.degrees(2.0 * math.atan(CAMERA_SENSOR_HEIGHT_MM / (2.0 * f_eq_mm)))
                visible = engine.visible_indices_for_view(
                    position=position,
                    pitch=pitch,
                    yaw=yaw,
                    focal_level=focal_level,
                    f_eq_mm=f_eq_mm,
                    hfov_deg=hfov_deg,
                    vfov_deg=vfov_deg,
                )
                if len(visible) and int(target_index) in set(int(index) for index in visible):
                    valid_probes.append(
                        self._diagnostic_probe_record(
                            int(target_index),
                            position,
                            yaw,
                            pitch,
                            focal_level,
                            f_eq_mm,
                            hfov_deg,
                            vfov_deg,
                        )
                    )
        valid_probes.sort(key=lambda item: float(item.get("score", 0.0)), reverse=True)
        return valid_probes, attempts

    def _build_observability_stats(self, safe_candidate_count: int, probe_count: int) -> Dict[str, object]:
        reason_counts: Dict[str, int] = {}
        for reason in (
            "observable_by_candidate",
            "candidate_generation_gap",
            "safety_unobservable",
            "observable_by_default",
        ):
            reason_counts[reason] = int(np.sum(self.target_observability_reason == reason))
        reason_counts["observable_by_probe"] = int(reason_counts.get("candidate_generation_gap", 0))

        semantic_summary: Dict[str, Dict[str, int]] = {}
        for semantic in TARGET_SEMANTICS:
            semantic_mask = self.semantics == semantic
            semantic_summary[semantic] = {
                "raw": int(np.sum(semantic_mask)),
                "observable": int(np.sum(np.logical_and(semantic_mask, self.observable_target_mask))),
                "effective": int(np.sum(np.logical_and(semantic_mask, self.effective_target_mask))),
                "unobservable": int(np.sum(np.logical_and(semantic_mask, self.unobservable_target_mask))),
                "observable_by_candidate": int(np.sum(np.logical_and(semantic_mask, self.target_observability_reason == "observable_by_candidate"))),
                "candidate_generation_gap": int(np.sum(np.logical_and(semantic_mask, self.target_observability_reason == "candidate_generation_gap"))),
                "safety_unobservable": int(np.sum(np.logical_and(semantic_mask, self.target_observability_reason == "safety_unobservable"))),
            }

        raw_count = int(np.sum(self.raw_target_mask))
        effective_count = int(np.sum(self.effective_target_mask))
        unobservable_count = int(np.sum(self.unobservable_target_mask))
        raw_weight = float(np.sum(self.weights[self.raw_target_mask])) if raw_count else 0.0
        excluded_weight = float(np.sum(self.weights[self.unobservable_target_mask])) if unobservable_count else 0.0
        semantic_rank = {semantic: rank for rank, semantic in enumerate(GAP_REPAIR_SEMANTIC_PRIORITY)}
        gap_targets: List[Dict[str, object]] = []
        for target_index in np.where(self.target_observability_reason == "candidate_generation_gap")[0]:
            probes = self.target_observability_probe_options.get(int(target_index), [])
            best_probe = dict(probes[0]) if probes else {
                "target_id": int(target_index),
                "semantic": str(self.semantics[target_index]),
                "coord": [round(float(v), 3) for v in self.target_coords[target_index]],
                "weight": round(float(self.weights[target_index]), 6),
                "reason": "candidate_generation_gap",
            }
            gap_targets.append(best_probe)
        gap_targets.sort(
            key=lambda item: (
                semantic_rank.get(str(item.get("semantic")), 999),
                -float(item.get("weight") or 0.0),
                int(item.get("target_id") or 0),
            )
        )
        gap_summary: Dict[str, int] = {}
        for item in gap_targets:
            semantic = str(item.get("semantic", "unknown"))
            gap_summary[semantic] = gap_summary.get(semantic, 0) + 1
        repaired_targets = list(getattr(self, "observability_gap_repair_records", []))
        repaired_targets.sort(
            key=lambda item: (
                semantic_rank.get(str(item.get("semantic")), 999),
                -float(item.get("weight") or 0.0),
                int(item.get("target_id") or 0),
                int(item.get("candidate_id") or 0),
            )
        )
        repair_stats = dict(getattr(self, "observability_gap_repair_stats", {}) or {})
        return {
            "coverage_basis": "observable_effective_targets",
            "raw_target_count": raw_count,
            "observable_target_count": int(np.sum(self.observable_target_mask)),
            "effective_target_count": effective_count,
            "unobservable_target_count": unobservable_count,
            "unobservable_ratio": round(float(unobservable_count / raw_count), 6) if raw_count else None,
            "excluded_weight_ratio": round(float(excluded_weight / max(raw_weight, 1e-9)), 6) if raw_count else None,
            "reason_counts": reason_counts,
            "semantic_summary": semantic_summary,
            "candidate_generation_gap_targets": gap_targets,
            "candidate_generation_gap_summary": gap_summary,
            "repaired_candidate_generation_gap_targets": repaired_targets,
            "gap_repair_candidate_count": int(len(repaired_targets)),
            **repair_stats,
            "safe_candidate_count": int(safe_candidate_count),
            "diagnostic_probe_count": int(probe_count),
        }

    def update_observability_from_candidates(
        self,
        engine: "VisibilityEngine",
        candidates: Sequence[CandidateViewpoint],
        safety_distance_m: float,
        conductor_no_fly_enabled: bool = True,
        conductor_no_fly_clearance_m: float = 0.0,
        conductor_no_fly_boundary_tolerance_m: float = DEFAULT_LIMITS["conductor_no_fly_boundary_tolerance_m"],
    ) -> Dict[str, object]:
        if conductor_no_fly_clearance_m <= 0.0:
            conductor_no_fly_clearance_m = required_no_fly_clearance_m(safety_distance_m)
        raw_mask = np.ones(self.target_count, dtype=bool)
        observable_by_candidate = np.zeros(self.target_count, dtype=bool)
        self.target_observability_probe_options = {}
        safe_candidate_count = 0
        for candidate in candidates:
            if str(getattr(candidate, "action_name", "photo") or "photo").lower() != "photo":
                continue
            if not self._position_safe_for_observability(
                candidate.position,
                safety_distance_m=safety_distance_m,
                conductor_no_fly_enabled=conductor_no_fly_enabled,
                conductor_no_fly_clearance_m=conductor_no_fly_clearance_m,
                conductor_no_fly_boundary_tolerance_m=conductor_no_fly_boundary_tolerance_m,
            ):
                continue
            safe_candidate_count += 1
            visible = engine.visible_indices_for_view(
                position=candidate.position,
                pitch=candidate.pitch,
                yaw=candidate.yaw,
                focal_level=candidate.focal_level,
                f_eq_mm=candidate.f_eq_mm,
                hfov_deg=candidate.hfov_deg,
                vfov_deg=candidate.vfov_deg,
            )
            if len(visible):
                observable_by_candidate[visible] = True

        observable_by_probe = np.zeros(self.target_count, dtype=bool)
        probe_count = 0
        for target_index in np.where(np.logical_and(raw_mask, ~observable_by_candidate))[0]:
            probe_options, attempts = self._target_observable_by_diagnostic_probe(
                engine,
                int(target_index),
                safety_distance_m=safety_distance_m,
                conductor_no_fly_enabled=conductor_no_fly_enabled,
                conductor_no_fly_clearance_m=conductor_no_fly_clearance_m,
                conductor_no_fly_boundary_tolerance_m=conductor_no_fly_boundary_tolerance_m,
            )
            probe_count += attempts
            if probe_options:
                observable_by_probe[target_index] = True
                self.target_observability_probe_options[int(target_index)] = probe_options

        self.raw_target_mask = raw_mask
        self.observable_target_mask = np.logical_or(observable_by_candidate, observable_by_probe)
        self.effective_target_mask = self.observable_target_mask.copy()
        self.unobservable_target_mask = self.raw_target_mask & ~self.observable_target_mask
        self.target_observability_reason = np.full(self.target_count, "safety_unobservable", dtype="<U32")
        self.target_observability_reason[observable_by_candidate] = "observable_by_candidate"
        self.target_observability_reason[observable_by_probe] = "candidate_generation_gap"

        self.raw_total_weight = float(np.sum(self.weights[self.raw_target_mask])) if self.target_count else 0.0
        self.effective_total_weight = float(np.sum(self.weights[self.effective_target_mask])) if self.target_count else 0.0
        self.effective_semantic_totals = {
            semantic: int(np.sum(np.logical_and(self.semantics == semantic, self.effective_target_mask)))
            for semantic in TARGET_SEMANTICS
        }
        self.effective_weight_totals = {
            semantic: float(np.sum(self.weights[np.logical_and(self.semantics == semantic, self.effective_target_mask)]))
            for semantic in TARGET_SEMANTICS
        }
        self.target_observability_stats = self._build_observability_stats(
            safe_candidate_count=safe_candidate_count,
            probe_count=probe_count,
        )
        return self.target_observability_stats

    def build_observability_gap_repair_candidates(
        self,
        existing_candidates: Sequence[CandidateViewpoint],
        engine: "VisibilityEngine",
        safety_distance_m: float,
        conductor_no_fly_enabled: bool = True,
        conductor_no_fly_clearance_m: float = 0.0,
        conductor_no_fly_boundary_tolerance_m: float = DEFAULT_LIMITS["conductor_no_fly_boundary_tolerance_m"],
        max_total: int = MAX_GAP_REPAIR_CANDIDATES_TOTAL,
        max_per_target: int = MAX_GAP_REPAIR_CANDIDATES_PER_TARGET,
        max_targets_per_semantic: int = MAX_GAP_REPAIR_TARGETS_PER_SEMANTIC,
    ) -> List[CandidateViewpoint]:
        if not ENABLE_OBSERVABILITY_GAP_REPAIR or self.target_count == 0:
            self.observability_gap_repair_records = []
            self.observability_gap_repair_stats = {
                "observability_gap_repair_raw_attempts": 0,
                "observability_gap_repair_added": 0,
                "observability_gap_repair_by_semantic": {},
                "observability_gap_repair_covered_targets": 0,
            }
            return []
        if conductor_no_fly_clearance_m <= 0.0:
            conductor_no_fly_clearance_m = required_no_fly_clearance_m(safety_distance_m)

        semantic_rank = {semantic: rank for rank, semantic in enumerate(GAP_REPAIR_SEMANTIC_PRIORITY)}
        gap_indices = [
            int(index)
            for index in np.where(self.target_observability_reason == "candidate_generation_gap")[0]
            if self.target_observability_probe_options.get(int(index))
        ]
        gap_indices.sort(
            key=lambda index: (
                semantic_rank.get(str(self.semantics[index]), 999),
                -float(self.weights[index]),
                index,
            )
        )

        existing_keys = set()
        position_ids: Dict[Tuple[float, float, float], int] = {}
        max_candidate_id = 0
        max_position_id = 0
        for candidate in existing_candidates:
            max_candidate_id = max(max_candidate_id, int(candidate.id))
            max_position_id = max(max_position_id, int(candidate.position_id))
            pos_key = tuple(round(float(v), 3) for v in np.asarray(candidate.position, dtype=float))
            target = np.asarray(candidate.look_at if candidate.look_at is not None else candidate.target_center, dtype=float)
            position_ids.setdefault(pos_key, int(candidate.position_id))
            existing_keys.add((
                *pos_key,
                round(float(target[0]), 3),
                round(float(target[1]), 3),
                round(float(target[2]), 3),
                str(candidate.semantic_focus),
                getattr(candidate, "target_id", None),
            ))

        def position_id_for(position: Sequence[float]) -> int:
            nonlocal max_position_id
            key = tuple(round(float(v), 3) for v in position)
            if key not in position_ids:
                max_position_id += 1
                position_ids[key] = max_position_id
            return position_ids[key]

        z_min = float(np.min(self.target_coords[:, 2])) if self.target_count else 0.0
        z_max = float(np.max(self.target_coords[:, 2])) if self.target_count else 0.0
        height = max(z_max - z_min, 1e-6)
        aim_type_map = {
            "insulator": "insulator_string",
            "tower_top": "tower_top",
            "tower_edge": "tower_edge",
            "tower_body": "tower_body",
        }

        promoted: List[CandidateViewpoint] = []
        repair_records: List[Dict[str, object]] = []
        targets_by_semantic: Dict[str, int] = {}
        raw_attempts = 0
        added_by_semantic: Dict[str, int] = {}
        covered_targets: set[int] = set()

        def add_candidate(target_index: int, probe: Dict[str, object]) -> Optional[CandidateViewpoint]:
            nonlocal max_candidate_id, raw_attempts
            raw_attempts += 1
            semantic = str(self.semantics[target_index])
            target = np.asarray(self.target_coords[target_index], dtype=float)
            position = np.asarray(probe.get("probe_position", probe.get("best_probe_position")), dtype=float)
            if not self._position_safe_for_observability(
                position,
                safety_distance_m=safety_distance_m,
                conductor_no_fly_enabled=conductor_no_fly_enabled,
                conductor_no_fly_clearance_m=conductor_no_fly_clearance_m,
                conductor_no_fly_boundary_tolerance_m=conductor_no_fly_boundary_tolerance_m,
            ):
                return None
            geom = compute_view_geometry(position, target)
            yaw = float(geom["heading"])
            pitch = float(geom["pitch"])
            distance = float(geom["Distance"])
            aim_type = aim_type_map.get(semantic, semantic)
            f_eq_mm = choose_focal_length_eq_mm(
                aim_type=aim_type,
                distance=distance,
                required_resolution=float(self.required_resolution[target_index]),
            )
            if f_eq_mm is None:
                f_eq_mm = 48.0
            f_eq_mm = max(24.0, min(84.0, float(f_eq_mm)))
            focal_level, _ = normalize_focal_level(None, f_eq_mm)
            hfov_deg = math.degrees(2.0 * math.atan(CAMERA_SENSOR_WIDTH_MM / (2.0 * f_eq_mm)))
            vfov_deg = math.degrees(2.0 * math.atan(CAMERA_SENSOR_HEIGHT_MM / (2.0 * f_eq_mm)))
            visible = engine.visible_indices_for_view(
                position=position,
                pitch=pitch,
                yaw=yaw,
                focal_level=focal_level,
                f_eq_mm=f_eq_mm,
                hfov_deg=hfov_deg,
                vfov_deg=vfov_deg,
            )
            if len(visible) == 0 or int(target_index) not in set(int(index) for index in visible):
                return None
            key = (
                round(float(position[0]), 3),
                round(float(position[1]), 3),
                round(float(position[2]), 3),
                round(float(target[0]), 3),
                round(float(target[1]), 3),
                round(float(target[2]), 3),
                semantic,
                int(target_index),
            )
            if key in existing_keys:
                return None
            existing_keys.add(key)
            target_z_ratio = float((target[2] - z_min) / height)
            target_vec = target[:2] - self.local_center[:2]
            target_azimuth = (
                float((math.degrees(math.atan2(float(target_vec[1]), float(target_vec[0]))) + 360.0) % 360.0)
                if float(np.linalg.norm(target_vec)) > 1e-6
                else 0.0
            )
            target_record = self.target_records[target_index] if target_index < len(self.target_records) else {}
            max_candidate_id += 1
            no_fly_clearance = self.min_conductor_no_fly_clearance(position)
            candidate = CandidateViewpoint(
                id=max_candidate_id,
                position_id=position_id_for(position),
                position=position,
                pitch=pitch,
                yaw=yaw,
                focal_level=focal_level,
                f_eq_mm=f_eq_mm,
                hfov_deg=float(hfov_deg),
                vfov_deg=float(vfov_deg),
                semantic_focus=semantic,
                target_center=target,
                base_score=0.0,
                manual_priority=min(1.0, max(0.0, float(self.weights[target_index]) / 5.0)),
                safety_distance_m=self.min_safety_distance(position, safety_distance_m),
                source="observability_gap_repair",
                position_z_ratio=float((position[2] - z_min) / height),
                target_z_ratio=target_z_ratio,
                target_azimuth_deg=target_azimuth,
                target_cluster_id=f"{semantic}_gap_{target_index}",
                look_at=target,
                distance=distance,
                heading=yaw,
                base_heading=yaw,
                yaw_offset_deg=0.0,
                focal_length_eq_mm=f_eq_mm,
                action_name="photo",
                aim_type=aim_type,
                target_id=int(target_index),
                instance_id=(
                    int(target_record["instance_id"])
                    if target_record.get("instance_id") is not None
                    else None
                ),
                layer_id=(
                    int(target_record["layer_id"])
                    if target_record.get("layer_id") is not None
                    else None
                ),
                sector_id=(
                    int(target_record["sector_id"])
                    if target_record.get("sector_id") is not None
                    else None
                ),
                weight=float(self.weights[target_index]),
                no_fly_checked=True,
                safety_checked=True,
                no_fly_clearance_m=(
                    float(no_fly_clearance)
                    if math.isfinite(float(no_fly_clearance))
                    else None
                ),
                no_fly_required_clearance_m=float(conductor_no_fly_clearance_m),
            )
            setattr(candidate, "required_resolution", float(self.required_resolution[target_index]))
            setattr(candidate, "probe_score", float(probe.get("score") or 0.0))
            return candidate

        for target_index in gap_indices:
            semantic = str(self.semantics[target_index])
            if targets_by_semantic.get(semantic, 0) >= max_targets_per_semantic:
                continue
            per_target_count = 0
            for probe in self.target_observability_probe_options.get(target_index, [])[:max_per_target]:
                if len(promoted) >= max_total:
                    break
                candidate = add_candidate(target_index, probe)
                if candidate is None:
                    continue
                promoted.append(candidate)

                repair_record = dict(probe)
                repair_record["candidate_id"] = int(candidate.id)
                repair_record["position_id"] = int(candidate.position_id)
                repair_record["source"] = "observability_gap_repair"
                repair_record["best_probe_position"] = [round(float(v), 3) for v in candidate.position]
                repair_record["best_probe_heading"] = round(float(candidate.heading or candidate.yaw), 3)
                repair_record["best_probe_pitch"] = round(float(candidate.pitch), 3)
                repair_record["best_probe_distance"] = round(float(candidate.distance or 0.0), 3)
                repair_record["best_probe_focal_length_eq_mm"] = round(float(candidate.f_eq_mm), 3)
                repair_records.append(repair_record)
                added_by_semantic[semantic] = added_by_semantic.get(semantic, 0) + 1
                covered_targets.add(int(target_index))
                per_target_count += 1
            if per_target_count:
                targets_by_semantic[semantic] = targets_by_semantic.get(semantic, 0) + 1
            if len(promoted) >= max_total:
                break

        self.observability_gap_repair_records = repair_records
        self.observability_gap_repair_stats = {
            "observability_gap_repair_raw_attempts": int(raw_attempts),
            "observability_gap_repair_added": int(len(promoted)),
            "observability_gap_repair_by_semantic": added_by_semantic,
            "observability_gap_repair_covered_targets": int(len(covered_targets)),
        }
        return promoted

    def observability_summary(self) -> Dict[str, object]:
        return dict(self.target_observability_stats)

    def coverage_from_mask(self, covered_mask: Sequence[bool], include_uncovered: bool = True) -> Dict[str, object]:
        mask = np.asarray(covered_mask, dtype=bool)
        if len(mask) != self.target_count:
            resized = np.zeros(self.target_count, dtype=bool)
            resized[: min(len(mask), self.target_count)] = mask[: min(len(mask), self.target_count)]
            mask = resized

        raw_mask = getattr(self, "raw_target_mask", np.ones(self.target_count, dtype=bool))
        effective_mask = getattr(self, "effective_target_mask", raw_mask)
        unobservable_mask = getattr(self, "unobservable_target_mask", raw_mask & ~effective_mask)
        raw_count = int(np.sum(raw_mask))
        effective_count = int(np.sum(effective_mask))
        total_count = raw_count
        if total_count == 0:
            return {
                "coverage": None,
                "coverage_weighted": None,
                "coverage_total": None,
                "coverage_tower": None,
                "coverage_insulator": None,
                "coverage_basis": "observable_effective_targets",
                "C_geo": None,
                "C_weighted": None,
                "C_geo_effective": None,
                "C_weighted_effective": None,
                "C_geo_raw": None,
                "C_weighted_raw": None,
                "C_ins": None,
                "C_top": None,
                "C_edge": None,
                "C_body": None,
                "C_ins_effective": None,
                "C_top_effective": None,
                "C_edge_effective": None,
                "C_body_effective": None,
                "C_ins_raw": None,
                "C_top_raw": None,
                "C_edge_raw": None,
                "C_body_raw": None,
                "target_count": 0,
                "raw_target_count": 0,
                "observable_target_count": 0,
                "effective_target_count": 0,
                "unobservable_target_count": 0,
                "unobservable_ratio": None,
                "excluded_weight_ratio": None,
                "covered_count": 0,
                "covered_effective_count": 0,
                "covered_raw_count": 0,
                "uncovered_summary": {semantic: 0 for semantic in TARGET_SEMANTICS},
                "uncovered_effective_summary": {semantic: 0 for semantic in TARGET_SEMANTICS},
                "uncovered_raw_summary": {semantic: 0 for semantic in TARGET_SEMANTICS},
                "unobservable_summary": {semantic: 0 for semantic in TARGET_SEMANTICS},
                "uncovered_voxels": [],
                "warnings": [],
            }

        covered_effective_mask = np.logical_and(mask, effective_mask)
        covered_raw_mask = np.logical_and(mask, raw_mask)
        covered_count = int(np.sum(covered_effective_mask))
        covered_raw_count = int(np.sum(covered_raw_mask))
        raw_weight_total = float(np.sum(self.weights[raw_mask])) if raw_count else 0.0
        effective_weight_total = float(np.sum(self.weights[effective_mask])) if effective_count else 0.0
        excluded_weight = float(np.sum(self.weights[unobservable_mask])) if raw_count else 0.0
        weighted_effective = (
            float(np.sum(self.weights[covered_effective_mask]) / max(effective_weight_total, 1e-9))
            if effective_count
            else None
        )
        weighted_raw = (
            float(np.sum(self.weights[covered_raw_mask]) / max(raw_weight_total, 1e-9))
            if raw_count
            else None
        )
        geo_effective = float(covered_count / effective_count) if effective_count else None
        geo_raw = float(covered_raw_count / raw_count) if raw_count else None

        result: Dict[str, object] = {
            "coverage_basis": "observable_effective_targets",
            "coverage": round(weighted_effective, 6) if weighted_effective is not None else None,
            "coverage_weighted": round(weighted_effective, 6) if weighted_effective is not None else None,
            "coverage_total": round(geo_effective, 6) if geo_effective is not None else None,
            "C_geo": round(geo_effective, 6) if geo_effective is not None else None,
            "C_weighted": round(weighted_effective, 6) if weighted_effective is not None else None,
            "C_geo_effective": round(geo_effective, 6) if geo_effective is not None else None,
            "C_weighted_effective": round(weighted_effective, 6) if weighted_effective is not None else None,
            "C_geo_raw": round(geo_raw, 6) if geo_raw is not None else None,
            "C_weighted_raw": round(weighted_raw, 6) if weighted_raw is not None else None,
            "target_count": effective_count,
            "raw_target_count": raw_count,
            "observable_target_count": int(np.sum(getattr(self, "observable_target_mask", effective_mask))),
            "effective_target_count": effective_count,
            "unobservable_target_count": int(np.sum(unobservable_mask)),
            "unobservable_ratio": round(float(np.sum(unobservable_mask) / raw_count), 6) if raw_count else None,
            "excluded_weight_ratio": round(float(excluded_weight / max(raw_weight_total, 1e-9)), 6) if raw_count else None,
            "covered_count": covered_count,
            "covered_effective_count": covered_count,
            "covered_raw_count": covered_raw_count,
        }

        tower_mask = np.isin(self.semantics, ["tower_top", "tower_edge", "tower_body"])
        tower_effective_mask = np.logical_and(tower_mask, effective_mask)
        tower_total = int(np.sum(tower_effective_mask))
        tower_covered = int(np.sum(np.logical_and(mask, tower_effective_mask)))
        result["coverage_tower"] = round(float(tower_covered / tower_total), 6) if tower_total else None
        result["tower_target_count"] = tower_total
        result["covered_tower_count"] = tower_covered

        ins_mask = np.logical_and(self.semantics == "insulator", effective_mask)
        ins_total = int(np.sum(ins_mask))
        ins_covered = int(np.sum(np.logical_and(mask, ins_mask)))
        result["coverage_insulator"] = round(float(ins_covered / ins_total), 6) if ins_total else None
        result["insulator_target_count"] = ins_total
        result["covered_insulator_count"] = ins_covered

        uncovered_effective_summary = {}
        uncovered_raw_summary = {}
        unobservable_summary = {}
        uncovered_voxels = []
        for semantic in TARGET_SEMANTICS:
            semantic_raw_mask = np.logical_and(self.semantics == semantic, raw_mask)
            semantic_effective_mask = np.logical_and(self.semantics == semantic, effective_mask)
            semantic_unobservable_mask = np.logical_and(self.semantics == semantic, unobservable_mask)
            raw_total_sem = int(np.sum(semantic_raw_mask))
            raw_covered_sem = int(np.sum(np.logical_and(mask, semantic_raw_mask)))
            effective_total_sem = int(np.sum(semantic_effective_mask))
            effective_covered_sem = int(np.sum(np.logical_and(mask, semantic_effective_mask)))
            uncovered_effective_summary[semantic] = max(effective_total_sem - effective_covered_sem, 0)
            uncovered_raw_summary[semantic] = max(raw_total_sem - raw_covered_sem, 0)
            unobservable_summary[semantic] = int(np.sum(semantic_unobservable_mask))
            coverage_value = float(effective_covered_sem / effective_total_sem) if effective_total_sem else None
            raw_coverage_value = float(raw_covered_sem / raw_total_sem) if raw_total_sem else None
            metric_name = {
                "insulator": "C_ins",
                "tower_top": "C_top",
                "tower_edge": "C_edge",
                "tower_body": "C_body",
            }[semantic]
            result[metric_name] = round(coverage_value, 6) if coverage_value is not None else None
            result[f"{metric_name}_effective"] = round(coverage_value, 6) if coverage_value is not None else None
            result[f"{metric_name}_raw"] = round(raw_coverage_value, 6) if raw_coverage_value is not None else None
            result[f"{semantic}_raw_target_count"] = raw_total_sem
            result[f"{semantic}_effective_target_count"] = effective_total_sem
            result[f"{semantic}_unobservable_target_count"] = int(np.sum(semantic_unobservable_mask))
            if include_uncovered and uncovered_effective_summary[semantic] > 0:
                indices = np.where(np.logical_and(semantic_effective_mask, ~mask))[0]
                for index in indices[:120]:
                    uncovered_voxels.append({
                        "coord": [round(float(v), 3) for v in self.target_coords[index]],
                        "semantic": semantic,
                    })

        warnings = []
        if raw_count and float(effective_count / raw_count) < 0.60:
            warnings.append("有效目标比例过低，需要人工复核禁飞区、候选生成或点云建模逻辑")

        result["uncovered_summary"] = uncovered_effective_summary
        result["uncovered_effective_summary"] = uncovered_effective_summary
        result["uncovered_raw_summary"] = uncovered_raw_summary
        result["unobservable_summary"] = unobservable_summary
        result["uncovered_voxels"] = uncovered_voxels[:400]
        result["warnings"] = warnings
        return result

    def attention_coverage_from_mask(self, covered_mask: Sequence[bool]) -> Dict[str, object]:
        """Build connection-attention metrics that do not affect coverage denominators."""
        mask = np.asarray(covered_mask, dtype=bool)
        if self.attention_count == 0:
            return {
                "C_connection_attention": None,
                "C_conductor_insulator_connection": None,
                "C_insulator_tower_side_connection": None,
                "C_ground_wire_tower_connection": None,
                "C_tower_base_connection": None,
                "connection_attention_count": 0,
                "covered_connection_attention_count": 0,
            }

        total = int(self.attention_count)
        covered = int(np.sum(mask))
        metrics: Dict[str, object] = {
            "C_connection_attention": round(float(covered / total), 6),
            "connection_attention_count": total,
            "covered_connection_attention_count": covered,
        }
        _attn_key_map = {
            "conductor_insulator_connection": "C_conductor_insulator_connection",
            "insulator_tower_side_connection": "C_insulator_tower_side_connection",
            "ground_wire_tower_connection": "C_ground_wire_tower_connection",
            "tower_base_connection": "C_tower_base_connection",
        }
        for semantic in ATTENTION_SEMANTICS:
            sem_mask = self.attention_semantics == semantic
            sem_total = int(np.sum(sem_mask))
            sem_covered = int(np.sum(mask[sem_mask]))
            key = _attn_key_map.get(semantic)
            if key:
                metrics[key] = round(float(sem_covered / sem_total), 6) if sem_total else None
            metrics[f"{semantic}_count"] = sem_total
            metrics[f"covered_{semantic}_count"] = sem_covered
        # Also emit legacy keys for backward compat
        if "C_conductor_insulator_connection" in metrics:
            metrics["C_wire_insulator_connection"] = metrics["C_conductor_insulator_connection"]
        return metrics


def coverage_thresholds_met(metrics: Dict[str, object]) -> bool:
    return coverage_thresholds_met_for(metrics, COVERAGE_THRESHOLDS)


def coverage_thresholds_met_for(metrics: Dict[str, object], thresholds: Mapping[str, float]) -> bool:
    effective_key_map = {
        "C_geo": "C_geo_effective",
        "C_weighted": "C_weighted_effective",
        "C_ins": "C_ins_effective",
        "C_top": "C_top_effective",
        "C_edge": "C_edge_effective",
        "C_body": "C_body_effective",
    }
    for key, threshold in thresholds.items():
        if key == "C_body":
            continue
        value = metrics.get(effective_key_map.get(key, key), metrics.get(key))
        if value is None:
            return False
        if float(value) + 1e-9 < threshold:
            return False
    return True


def compact_coverage_thresholds_met(metrics: Dict[str, object]) -> bool:
    return coverage_thresholds_met_for(metrics, COMPACT_COVERAGE_THRESHOLDS)


class VisibilityEngine:
    def __init__(self, env: PlanningEnvironment):
        self.env = env
        self.cache: Dict[int, np.ndarray] = {}
        self.attention_cache: Dict[int, np.ndarray] = {}

    def candidate_visible_indices(self, candidate: CandidateViewpoint) -> np.ndarray:
        if candidate.id not in self.cache:
            if str(getattr(candidate, "action_name", "photo") or "photo").lower() != "photo":
                self.cache[candidate.id] = np.array([], dtype=int)
            else:
                self.cache[candidate.id] = self.visible_indices_for_view(
                    position=candidate.position,
                    pitch=candidate.pitch,
                    yaw=candidate.yaw,
                    focal_level=candidate.focal_level,
                    f_eq_mm=candidate.f_eq_mm,
                    hfov_deg=candidate.hfov_deg,
                    vfov_deg=candidate.vfov_deg,
                )
        return self.cache[candidate.id]

    def candidate_visible_attention_indices(self, candidate: CandidateViewpoint) -> np.ndarray:
        """Return connection-attention targets visible from a candidate view."""
        if candidate.id not in self.attention_cache:
            self.attention_cache[candidate.id] = self.visible_attention_indices_for_view(
                position=candidate.position,
                pitch=candidate.pitch,
                yaw=candidate.yaw,
                focal_level=candidate.focal_level,
                f_eq_mm=candidate.f_eq_mm,
                hfov_deg=candidate.hfov_deg,
                vfov_deg=candidate.vfov_deg,
            )
        return self.attention_cache[candidate.id]

    def visible_attention_indices_for_view(
        self,
        position: Sequence[float],
        pitch: float,
        yaw: float,
        focal_level: Optional[str] = None,
        f_eq_mm: Optional[float] = None,
        hfov_deg: Optional[float] = None,
        vfov_deg: Optional[float] = None,
    ) -> np.ndarray:
        if self.env.attention_count == 0:
            return np.array([], dtype=int)

        focal_key, focal_cfg = normalize_focal_level(focal_level, f_eq_mm)
        f_eq_mm = float(f_eq_mm or focal_cfg["f_eq_mm"])
        hfov_deg = float(hfov_deg or focal_cfg["hfov_deg"])
        vfov_deg = float(vfov_deg or focal_cfg["vfov_deg"])

        position = np.asarray(position, dtype=float)
        vectors = self.env.attention_coords - position
        distances = np.linalg.norm(vectors, axis=1)
        distance_mask = np.logical_and(
            distances <= DEFAULT_LIMITS["max_visibility_distance_m"],
            distances >= focal_cfg["min_distance_m"],
        )
        if not np.any(distance_mask):
            return np.array([], dtype=int)

        forward, right, up = camera_basis(yaw, pitch)
        x_cam = np.dot(vectors, right)
        y_cam = np.dot(vectors, up)
        z_cam = np.dot(vectors, forward)
        front_mask = z_cam > 0.0
        horizontal_angle = np.degrees(np.arctan2(np.abs(x_cam), np.maximum(z_cam, 1e-6)))
        vertical_angle = np.degrees(np.arctan2(np.abs(y_cam), np.maximum(z_cam, 1e-6)))
        fov_mask = np.logical_and(horizontal_angle <= hfov_deg / 2.0, vertical_angle <= vfov_deg / 2.0)
        preliminary = np.where(np.logical_and.reduce([distance_mask, front_mask, fov_mask]))[0]
        if len(preliminary) == 0:
            return np.array([], dtype=int)

        to_camera = position - self.env.attention_coords[preliminary]
        to_camera_norm = to_camera / np.maximum(np.linalg.norm(to_camera, axis=1, keepdims=True), 1e-6)
        normal_hints = self.env.attention_normal_hints[preliminary]
        incidence = np.degrees(np.arccos(np.clip(np.sum(to_camera_norm * normal_hints, axis=1), -1.0, 1.0)))
        incidence_mask = incidence <= self.env.attention_incidence_max[preliminary]
        if not np.any(incidence_mask):
            return np.array([], dtype=int)

        filtered = preliminary[incidence_mask]
        filtered_distance = distances[filtered]
        filtered_incidence = incidence[incidence_mask]
        resolutions = np.array(
            [observation_resolution(f_eq_mm, dist, inc) for dist, inc in zip(filtered_distance, filtered_incidence)],
            dtype=float,
        )
        precision_mask = resolutions >= self.env.attention_required_resolution[filtered]
        if not np.any(precision_mask):
            return np.array([], dtype=int)
        return np.array(sorted(set(filtered[precision_mask].tolist())), dtype=int)

    def visible_indices_for_view(
        self,
        position: Sequence[float],
        pitch: float,
        yaw: float,
        focal_level: Optional[str] = None,
        f_eq_mm: Optional[float] = None,
        hfov_deg: Optional[float] = None,
        vfov_deg: Optional[float] = None,
        skip_resolution_check: bool = False,
    ) -> np.ndarray:
        if self.env.target_count == 0:
            return np.array([], dtype=int)

        focal_key, focal_cfg = normalize_focal_level(focal_level, f_eq_mm)
        f_eq_mm = float(f_eq_mm or focal_cfg["f_eq_mm"])
        hfov_deg = float(hfov_deg or focal_cfg["hfov_deg"])
        vfov_deg = float(vfov_deg or focal_cfg["vfov_deg"])

        position = np.asarray(position, dtype=float)
        vectors = self.env.target_coords - position
        distances = np.linalg.norm(vectors, axis=1)
        distance_mask = np.logical_and(
            distances <= DEFAULT_LIMITS["max_visibility_distance_m"],
            distances >= focal_cfg["min_distance_m"],
        )
        if not np.any(distance_mask):
            return np.array([], dtype=int)

        forward, right, up = camera_basis(yaw, pitch)
        x_cam = np.dot(vectors, right)
        y_cam = np.dot(vectors, up)
        z_cam = np.dot(vectors, forward)
        front_mask = z_cam > 0.0

        horizontal_angle = np.degrees(np.arctan2(np.abs(x_cam), np.maximum(z_cam, 1e-6)))
        vertical_angle = np.degrees(np.arctan2(np.abs(y_cam), np.maximum(z_cam, 1e-6)))
        fov_mask = np.logical_and(horizontal_angle <= hfov_deg / 2.0, vertical_angle <= vfov_deg / 2.0)

        preliminary = np.where(np.logical_and.reduce([distance_mask, front_mask, fov_mask]))[0]
        if len(preliminary) == 0:
            return np.array([], dtype=int)

        to_camera = position - self.env.target_coords[preliminary]
        to_camera_norm = to_camera / np.maximum(np.linalg.norm(to_camera, axis=1, keepdims=True), 1e-6)
        normal_hints = self.env.normal_hints[preliminary]
        incidence = np.degrees(np.arccos(np.clip(np.sum(to_camera_norm * normal_hints, axis=1), -1.0, 1.0)))
        incidence_mask = incidence <= self.env.incidence_max[preliminary]
        if not np.any(incidence_mask):
            return np.array([], dtype=int)

        filtered = preliminary[incidence_mask]
        if skip_resolution_check:
            return np.array(sorted(set(filtered.tolist())), dtype=int)

        filtered_distance = distances[filtered]
        filtered_incidence = incidence[incidence_mask]
        resolutions = np.array(
            [observation_resolution(f_eq_mm, dist, inc) for dist, inc in zip(filtered_distance, filtered_incidence)],
            dtype=float,
        )
        precision_mask = resolutions >= self.env.required_resolution[filtered]
        if not np.any(precision_mask):
            return np.array([], dtype=int)

        return np.array(sorted(set(filtered[precision_mask].tolist())), dtype=int)


def flatten_waypoint_views(waypoints: Sequence[Dict[str, object]]) -> List[Dict[str, object]]:
    views: List[Dict[str, object]] = []
    for waypoint in waypoints or []:
        position = waypoint.get("position", waypoint.get("pos_utm"))
        if not position:
            continue
        shots = waypoint.get("shots") or []
        if shots:
            for shot in shots:
                focal_level = shot.get("focal_level", waypoint.get("focal_level"))
                f_eq_mm = shot.get("f_eq_mm", waypoint.get("f_eq_mm"))
                views.append({
                    "position": position,
                    "pitch": _safe_float(shot.get("pitch", waypoint.get("pitch", 0.0))),
                    "yaw": _safe_float(shot.get("yaw", waypoint.get("yaw", waypoint.get("heading", 0.0)))),
                    "focal_level": focal_level,
                    "f_eq_mm": f_eq_mm,
                })
        else:
            views.append({
                "position": position,
                "pitch": _safe_float(waypoint.get("pitch", 0.0)),
                "yaw": _safe_float(waypoint.get("yaw", waypoint.get("heading", 0.0))),
                "focal_level": waypoint.get("focal_level"),
                "f_eq_mm": waypoint.get("f_eq_mm"),
            })
    return views


def evaluate_waypoint_coverage(waypoints: Sequence[Dict[str, object]], voxel_path: str) -> Dict[str, object]:
    env = PlanningEnvironment(voxel_path)
    engine = VisibilityEngine(env)
    if env.target_count == 0:
        return env.coverage_from_mask([], include_uncovered=False)

    covered_mask = np.zeros(env.target_count, dtype=bool)
    attention_mask = np.zeros(env.attention_count, dtype=bool)
    for view in flatten_waypoint_views(waypoints):
        position = np.asarray(view["position"], dtype=float)
        pitch = float(view.get("pitch", 0.0))
        yaw = float(view.get("yaw", 0.0))
        focal_level = view.get("focal_level")
        f_eq_mm = view.get("f_eq_mm")

        if not focal_level and f_eq_mm is None:
            best_indices = np.array([], dtype=int)
            best_gain = -1.0
            for key in SUPPORTED_FOCALS:
                indices = engine.visible_indices_for_view(position, pitch, yaw, focal_level=key)
                if len(indices) == 0:
                    continue
                effective_indices = indices[env.effective_target_mask[indices]]
                new_indices = effective_indices[~covered_mask[effective_indices]]
                gain = float(np.sum(env.weights[new_indices])) if len(new_indices) else 0.0
                if gain > best_gain:
                    best_gain = gain
                    best_indices = indices
            indices = best_indices
        else:
            focal_level, focal_cfg = normalize_focal_level(focal_level, f_eq_mm)
            indices = engine.visible_indices_for_view(
                position,
                pitch,
                yaw,
                focal_level=focal_level,
                f_eq_mm=f_eq_mm or focal_cfg["f_eq_mm"],
            )

        if len(indices):
            covered_mask[indices] = True
        attention_indices = engine.visible_attention_indices_for_view(
            position,
            pitch,
            yaw,
            focal_level=focal_level,
            f_eq_mm=f_eq_mm,
        )
        if len(attention_indices):
            attention_mask[attention_indices] = True

    metrics = env.coverage_from_mask(covered_mask)
    metrics.update(env.attention_coverage_from_mask(attention_mask))
    metrics["coverage_status"] = "success" if coverage_thresholds_met(metrics) else "partial"
    return metrics


def compute_waypoint_metrics(
    waypoints: Sequence[Dict[str, object]],
    coverage=None,
    compute_time: Optional[float] = None,
    coverage_details: Optional[Dict[str, object]] = None,
) -> Dict[str, object]:
    waypoint_positions = []
    shot_count = 0
    focal_usage: Dict[str, int] = {}
    for waypoint in waypoints or []:
        position = waypoint.get("position", waypoint.get("pos_utm"))
        if position and len(position) >= 3:
            waypoint_positions.append([_safe_float(v) for v in position[:3]])
        shots = waypoint.get("shots") or []
        if shots:
            shot_count += len(shots)
            for shot in shots:
                focal_level = shot.get("focal_level")
                if focal_level:
                    focal_usage[str(focal_level)] = focal_usage.get(str(focal_level), 0) + 1
        else:
            shot_count += 1
            focal_level = waypoint.get("focal_level")
            if focal_level:
                focal_usage[str(focal_level)] = focal_usage.get(str(focal_level), 0) + 1

    positions = np.asarray(waypoint_positions, dtype=float) if waypoint_positions else np.empty((0, 3), dtype=float)
    waypoint_count = int(len(positions))
    path_length = 0.0
    avg_segment = 0.0
    if waypoint_count >= 2:
        segments = np.diff(positions, axis=0)
        lengths = np.linalg.norm(segments, axis=1)
        path_length = float(np.sum(lengths))
        avg_segment = float(np.mean(lengths))

    details = coverage_details or {}
    if isinstance(coverage, dict):
        details = {**coverage, **details}
        coverage = details.get("coverage")
    coverage_value = None if coverage is None else float(coverage)

    metrics = {
        "coverage": coverage_value,
        "count": waypoint_count,
        "waypoint_count": waypoint_count,
        "shot_count": int(shot_count),
        "path_length": round(path_length, 3),
        "avg_segment_length": round(avg_segment, 3),
        "altitude_avg": round(float(np.mean(positions[:, 2])) if waypoint_count else 0.0, 3),
        "altitude_min": round(float(np.min(positions[:, 2])) if waypoint_count else 0.0, 3),
        "altitude_max": round(float(np.max(positions[:, 2])) if waypoint_count else 0.0, 3),
        "coverage_per_waypoint": round(float(coverage_value / waypoint_count), 6) if coverage_value is not None and waypoint_count else None,
        "coverage_per_shot": round(float(coverage_value / shot_count), 6) if coverage_value is not None and shot_count else None,
        "focal_usage": focal_usage,
    }
    if details:
        metrics.update(details)
        if metrics.get("coverage") is not None and metrics.get("waypoint_count"):
            metrics["coverage_per_waypoint"] = round(float(metrics["coverage"]) / int(metrics["waypoint_count"]), 6)
        if metrics.get("coverage") is not None and metrics.get("shot_count"):
            metrics["coverage_per_shot"] = round(float(metrics["coverage"]) / int(metrics["shot_count"]), 6)
    if compute_time is not None:
        metrics["compute_time"] = round(float(compute_time), 2)
    return metrics
