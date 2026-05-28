from __future__ import annotations

import json
import math
import random
import time
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple, Type

import numpy as np

from .planning_core import (
    AIMTYPE_VIEW_PROFILE,
    ATTENTION_SEMANTICS,
    CAMERA_SENSOR_HEIGHT_MM,
    CAMERA_SENSOR_WIDTH_MM,
    COMPACT_COVERAGE_THRESHOLDS,
    COVERAGE_THRESHOLDS,
    DEFAULT_LIMITS,
    ENABLE_OBSERVABILITY_GAP_REPAIR,
    REPAIR_PRIORITY,
    REQUIRED_RESOLUTION,
    SEMANTIC_PRIORITY,
    SUPPORTED_FOCALS,
    CandidateViewpoint,
    PlanningEnvironment,
    VisibilityEngine,
    _normalize_semantic,
    choose_focal_length_eq_mm,
    compact_coverage_thresholds_met,
    compute_view_geometry,
    compute_waypoint_metrics,
    coverage_thresholds_met,
    load_candidate_views,
    parse_manual_route,
    required_no_fly_clearance_m,
)
from .waypoint_models import WaypointPlanningInput, WaypointResult, build_waypoint_result


ProgressCallback = Optional[Callable[[str, int, str], None]]
KEY_CLUSTER_SEMANTICS = ("insulator", "conductor_insulator_connection", "insulator_tower_side_connection", "ground_wire_tower_connection")
KEY_CLUSTER_MIN_VIEWPOINTS = 3
KEY_CLUSTER_MAX_VIEWPOINTS = 5
KEY_CLUSTER_MIN_ANGLE_DEG = 15.0
KEY_CLUSTER_MIN_DISTANCE_M = 1.5
MID_LOWER_TOWER_RING_MAX = 3


@dataclass
class BeamState:
    selected_ids: List[int]
    covered_mask: np.ndarray
    metrics: Dict[str, object]


class BasePlanner:
    planner_name = "算法规划"
    max_candidate_pool = 2000
    runtime_floor_seconds = 20.0

    def __init__(
        self,
        voxel_path: str | WaypointPlanningInput,
        cand_path: Optional[str] = None,
        output_dir: Optional[str] = None,
        planner_key: Optional[str] = None,
        planner_name: Optional[str] = None,
        status_key: str = "rl",
        progress_callback: ProgressCallback = None,
    ):
        if isinstance(voxel_path, WaypointPlanningInput):
            planning_input = voxel_path
            if status_key == "rl":
                status_key = planning_input.status_key
            planner_key = planner_key or planning_input.planner_key
            planner_name = planner_name or planning_input.planner_name
        else:
            if cand_path is None or output_dir is None:
                raise ValueError("cand_path and output_dir are required with a voxel path")
            planning_input = WaypointPlanningInput(
                voxel_path=voxel_path,
                candidate_path=cand_path,
                output_dir=output_dir,
                planner_key=planner_key or self.planner_name,
                planner_name=planner_name or self.planner_name,
                status_key=status_key,
            )

        self.planning_input = planning_input
        self.voxel_path = str(planning_input.voxel_path)
        self.cand_path = str(planning_input.candidate_path)
        self.output_dir = str(planning_input.output_dir)
        self.planner_key = planner_key or self.planner_name
        self.planner_name = planner_name or self.planner_name
        self.status_key = status_key
        self.progress_callback = progress_callback

        self.env: Optional[PlanningEnvironment] = None
        self.engine: Optional[VisibilityEngine] = None
        self.candidates: List[CandidateViewpoint] = []
        self.candidate_map: Dict[int, CandidateViewpoint] = {}
        self.position_candidate_ids: Dict[int, List[int]] = {}
        self.manual_route_path: Optional[Path] = planning_input.manual_route_path
        self.manual_waypoint_cap: Optional[int] = None
        self.manual_waypoint_min: Optional[int] = None
        self.manual_waypoint_max: Optional[int] = None
        self.supplement_added_ids: set[int] = set()
        self.multi_shot_enrichment_added_ids: set[int] = set()
        self.solution_supplement_ids: Dict[Tuple[int, ...], set[int]] = {}
        self.manual_waypoint_limit_overridden = False
        self.key_cluster_counts: Dict[str, Dict[str, int]] = {}
        self.key_cluster_shortfalls: Dict[str, Dict[str, int]] = {}
        constraints = planning_input.constraints
        self.explicit_max_waypoints = constraints.max_waypoints is not None
        self.safety_distance_m = float(constraints.safety_distance_m or DEFAULT_LIMITS["safety_distance_m"])
        self.conductor_no_fly_enabled = bool(getattr(constraints, "conductor_no_fly_enabled", True))
        self.conductor_no_fly_boundary_tolerance_m = float(
            getattr(
                constraints,
                "conductor_no_fly_boundary_tolerance_m",
                DEFAULT_LIMITS["conductor_no_fly_boundary_tolerance_m"],
            )
            or DEFAULT_LIMITS["conductor_no_fly_boundary_tolerance_m"]
        )
        self.conductor_no_fly_clearance_m = required_no_fly_clearance_m(
            self.safety_distance_m,
            getattr(constraints, "conductor_no_fly_clearance_m", None),
        )
        self.manual_ratio_min = max(0.0, min(0.80, float(constraints.manual_ratio_min or DEFAULT_LIMITS["manual_ratio_min"])))
        self.manual_ratio_max = max(
            self.manual_ratio_min,
            min(0.80, float(constraints.manual_ratio_max or DEFAULT_LIMITS["manual_ratio_max"])),
        )
        self.single_layer_episodes = max(int(constraints.single_layer_episodes or DEFAULT_LIMITS["single_layer_episodes"]), 1)
        self.hierarchical_episodes = max(int(constraints.hierarchical_episodes or DEFAULT_LIMITS["hierarchical_episodes"]), 1)
        # Use FINAL_WAYPOINT_LIMITS as base, overridable by constraints
        from .planning_core import FINAL_WAYPOINT_LIMITS as _FWL
        self.min_photo_waypoints = int(_FWL.get("min_photo_waypoints", 10))
        self.max_photo_waypoints = int(_FWL.get("max_photo_waypoints", 80))
        self.min_waypoints = DEFAULT_LIMITS["min_waypoints"]
        self.max_waypoints = DEFAULT_LIMITS["max_waypoints"]
        self.max_total_shots = DEFAULT_LIMITS["max_total_shots"]
        self.max_shots_per_waypoint = DEFAULT_LIMITS["max_shots_per_waypoint"]
        # Override from user constraints if provided
        if constraints.min_waypoints is not None:
            self.min_waypoints = int(constraints.min_waypoints)
            self.min_photo_waypoints = int(constraints.min_waypoints)
        if constraints.max_waypoints is not None:
            self.max_waypoints = int(constraints.max_waypoints)
            self.max_photo_waypoints = int(constraints.max_waypoints)
        # Clamp to FINAL_WAYPOINT_LIMITS sane range
        self.min_photo_waypoints = max(10, min(80, self.min_photo_waypoints))
        self.max_photo_waypoints = max(10, min(80, self.max_photo_waypoints))
        # Stop reason tracking
        self.stop_reason: Optional[str] = None
        self.fallback_reason: Optional[str] = None
        if constraints.min_waypoints is not None:
            self.min_waypoints = int(constraints.min_waypoints)
        if constraints.max_waypoints is not None:
            self.max_waypoints = int(constraints.max_waypoints)
        if constraints.max_total_shots is not None:
            self.max_total_shots = int(constraints.max_total_shots)
        if constraints.max_shots_per_waypoint is not None:
            self.max_shots_per_waypoint = int(constraints.max_shots_per_waypoint)
        self.max_shots_per_waypoint = max(1, min(2, self.max_shots_per_waypoint))

    def progress(self, percent: int, message: str):
        if self.progress_callback:
            self.progress_callback(self.status_key, int(percent), message)

    def _manual_route_candidates(self) -> List[Path]:
        base = Path(self.voxel_path).stem.replace("_voxel", "")
        if self.manual_route_path:
            return [Path(self.manual_route_path)]
        manual_dir = Path(self.output_dir).resolve().parent / "manual_route"
        if not manual_dir.exists():
            return []
        exact = manual_dir / f"{base}.json"
        candidates = [exact] if exact.exists() else []
        if not candidates:
            candidates = sorted(manual_dir.glob(f"{base}*.json"))
        return candidates

    def _manual_cap_from_scene(self) -> Optional[int]:
        for path in self._manual_route_candidates():
            if not path.exists():
                continue
            try:
                count = len(parse_manual_route(str(path)))
                if count > 0:
                    self.manual_route_path = path
                    return count
            except Exception:
                continue
        return None

    def _manual_route_from_scene(self) -> Optional[Path]:
        for path in self._manual_route_candidates():
            try:
                if path.exists():
                    self.manual_route_path = path
                    return path
            except Exception:
                continue
        return None

    def _apply_no_fly_vertical_margins(self) -> None:
        """Expand conductor no-fly volumes above/below using the active safety distance."""
        if self.env is None or not self.conductor_no_fly_enabled:
            return
        top_margin_m = 0.0
        bottom_margin_m = max(float(self.safety_distance_m) / 4.0, 0.0)
        self.env.conductor_no_fly_volumes = [
            volume.with_exact_vertical_margins(top_margin_m, bottom_margin_m)
            for volume in self.env.conductor_no_fly_volumes
        ]

    def _load_environment(self):
        self.progress(5, f"加载{self.planner_name}环境...")
        self.env = PlanningEnvironment(self.voxel_path)
        if self.env.target_count == 0:
            raise ValueError("当前体素文件中没有可规划的目标语义体素")
        self.engine = VisibilityEngine(self.env)
        self.candidates = load_candidate_views(self.cand_path)
        self.candidate_map = {candidate.id: candidate for candidate in self.candidates}

        self.manual_waypoint_cap = self._manual_cap_from_scene()
        self._apply_no_fly_vertical_margins()
        if self.manual_waypoint_cap and self.manual_waypoint_cap > 1:
            target_ratio = max(0.0, min(0.80, float((self.manual_ratio_min + self.manual_ratio_max) / 2.0)))
            target_waypoints = max(1, int(math.floor(self.manual_waypoint_cap * target_ratio + 0.5)))
            self.manual_waypoint_min = target_waypoints
            self.manual_waypoint_max = target_waypoints
            self.min_waypoints = target_waypoints
            self.max_waypoints = target_waypoints
            if self.planning_input.constraints.max_total_shots is None:
                self.max_total_shots = self.max_waypoints * self.max_shots_per_waypoint
        self.min_waypoints = min(self.min_waypoints, self.max_waypoints)
        self.max_total_shots = min(self.max_total_shots, self.max_waypoints * self.max_shots_per_waypoint)
        self._prepare_candidates()
        self._sync_position_groups()

    def _prepare_candidates(self):
        assert self.env is not None and self.engine is not None
        safe_candidates: List[CandidateViewpoint] = []
        self.gap_repair_candidates_loaded = int(
            sum(1 for candidate in self.candidates if getattr(candidate, "source", "") == "observability_gap_repair")
        )
        total = max(len(self.candidates), 1)
        for index, candidate in enumerate(self.candidates, start=1):
            if self.conductor_no_fly_enabled and self.env.inside_conductor_no_fly(
                candidate.position,
                tolerance_m=self.conductor_no_fly_boundary_tolerance_m,
            ):
                continue
            no_fly_clearance = self.env.min_conductor_no_fly_clearance(candidate.position)
            candidate.no_fly_clearance_m = (
                float(no_fly_clearance)
                if math.isfinite(float(no_fly_clearance))
                else None
            )
            candidate.no_fly_required_clearance_m = float(self.conductor_no_fly_clearance_m)
            if self.conductor_no_fly_enabled and no_fly_clearance + 1e-9 < self.conductor_no_fly_clearance_m:
                continue
            safety_distance = self.env.min_safety_distance(candidate.position, self.safety_distance_m)
            candidate.safety_distance_m = safety_distance
            if safety_distance < self.safety_distance_m:
                continue
            safe_candidates.append(candidate)
            if index % 50 == 0 or index == total:
                self.progress(5 + int(7 * index / total), f"filter safe candidates {index}/{total}...")

        self.env.update_observability_from_candidates(
            self.engine,
            safe_candidates,
            safety_distance_m=self.safety_distance_m,
            conductor_no_fly_enabled=self.conductor_no_fly_enabled,
            conductor_no_fly_clearance_m=self.conductor_no_fly_clearance_m,
            conductor_no_fly_boundary_tolerance_m=self.conductor_no_fly_boundary_tolerance_m,
        )
        self.initial_safe_candidate_count = int(len(safe_candidates))
        self.gap_repair_candidates_safe = int(
            sum(1 for candidate in safe_candidates if getattr(candidate, "source", "") == "observability_gap_repair")
        )
        repair_candidates: List[CandidateViewpoint] = []
        if ENABLE_OBSERVABILITY_GAP_REPAIR:
            repair_candidates = self.env.build_observability_gap_repair_candidates(
                safe_candidates,
                self.engine,
                safety_distance_m=self.safety_distance_m,
                conductor_no_fly_enabled=self.conductor_no_fly_enabled,
                conductor_no_fly_clearance_m=self.conductor_no_fly_clearance_m,
                conductor_no_fly_boundary_tolerance_m=self.conductor_no_fly_boundary_tolerance_m,
            )
            if repair_candidates:
                safe_candidates.extend(repair_candidates)
                self.gap_repair_candidates_loaded += int(len(repair_candidates))
                self.gap_repair_candidates_safe += int(len(repair_candidates))
                self.env.update_observability_from_candidates(
                    self.engine,
                    safe_candidates,
                    safety_distance_m=self.safety_distance_m,
                    conductor_no_fly_enabled=self.conductor_no_fly_enabled,
                    conductor_no_fly_clearance_m=self.conductor_no_fly_clearance_m,
                    conductor_no_fly_boundary_tolerance_m=self.conductor_no_fly_boundary_tolerance_m,
                )
        self.gap_repair_candidate_count = int(len(repair_candidates))
        self.position_group_merge_count = self._merge_nearby_candidate_positions(safe_candidates, radius_m=1.2)
        self.env.update_observability_from_candidates(
            self.engine,
            safe_candidates,
            safety_distance_m=self.safety_distance_m,
            conductor_no_fly_enabled=self.conductor_no_fly_enabled,
            conductor_no_fly_clearance_m=self.conductor_no_fly_clearance_m,
            conductor_no_fly_boundary_tolerance_m=self.conductor_no_fly_boundary_tolerance_m,
        )
        if ENABLE_OBSERVABILITY_GAP_REPAIR and int(
            self.env.target_observability_stats.get("reason_counts", {}).get("candidate_generation_gap", 0)
        ) > 0:
            second_pass_repairs = self.env.build_observability_gap_repair_candidates(
                safe_candidates,
                self.engine,
                safety_distance_m=self.safety_distance_m,
                conductor_no_fly_enabled=self.conductor_no_fly_enabled,
                conductor_no_fly_clearance_m=self.conductor_no_fly_clearance_m,
                conductor_no_fly_boundary_tolerance_m=self.conductor_no_fly_boundary_tolerance_m,
            )
            if second_pass_repairs:
                safe_candidates.extend(second_pass_repairs)
                self.gap_repair_candidate_count += int(len(second_pass_repairs))
                self.gap_repair_candidates_loaded += int(len(second_pass_repairs))
                self.gap_repair_candidates_safe += int(len(second_pass_repairs))
                self.env.update_observability_from_candidates(
                    self.engine,
                    safe_candidates,
                    safety_distance_m=self.safety_distance_m,
                    conductor_no_fly_enabled=self.conductor_no_fly_enabled,
                    conductor_no_fly_clearance_m=self.conductor_no_fly_clearance_m,
                    conductor_no_fly_boundary_tolerance_m=self.conductor_no_fly_boundary_tolerance_m,
                )

        scored_by_semantic: Dict[str, List[Tuple[float, CandidateViewpoint]]] = {semantic: [] for semantic in SEMANTIC_PRIORITY}
        total_safe = max(len(safe_candidates), 1)
        for index, candidate in enumerate(safe_candidates, start=1):
            visible = self.engine.candidate_visible_indices(candidate)
            attention_visible = self.engine.candidate_visible_attention_indices(candidate)
            effective_visible = visible[self.env.effective_target_mask[visible]] if len(visible) else np.array([], dtype=int)
            if len(effective_visible) == 0 and len(attention_visible) == 0:
                continue
            weights = float(np.sum(self.env.weights[effective_visible]))
            # ── Quality penalty for overview / resolution-skipped views ──
            quality_factor = 0.12 if getattr(candidate, "aim_type", None) == "tower_overview" else 1.0
            semantic_bonus = 0.0
            for semantic, priority in SEMANTIC_PRIORITY.items():
                if semantic in ATTENTION_SEMANTICS:
                    semantic_bonus += priority * float(np.sum(self.env.attention_semantics[attention_visible] == semantic)) / max(
                        self.env.attention_totals.get(semantic, 1),
                        1,
                    )
                else:
                    semantic_bonus += priority * float(np.sum(self.env.semantics[effective_visible] == semantic)) / max(
                        self.env.effective_semantic_totals.get(semantic, 1),
                        1,
                    )
            safety_distance = float(candidate.safety_distance_m or self.env.min_safety_distance(candidate.position, self.safety_distance_m))
            safety_margin = min(max((safety_distance - self.safety_distance_m) / max(self.safety_distance_m, 1e-6), 0.0), 1.0)
            candidate.base_score = (
                quality_factor * weights / max(self.env.effective_total_weight, 1e-9)
                + 0.2 * semantic_bonus
                + 0.22 * float(candidate.manual_priority)
                + 0.04 * safety_margin
            )
            if getattr(candidate, "source", "") == "observability_gap_repair":
                candidate.base_score += 0.08
            semantic_key = candidate.semantic_focus if candidate.semantic_focus in scored_by_semantic else "tower_body"
            scored_by_semantic[semantic_key].append((candidate.base_score, candidate))
            if index % 50 == 0 or index == total_safe:
                self.progress(12 + int(8 * index / total_safe), f"prepare candidates {index}/{total_safe}...")

        if not any(scored_by_semantic.values()):
            raise ValueError("候选视点没有任何有效覆盖，无法规划")

        semantic_keep = {
            "insulator": 520,
            "conductor_insulator_connection": 260,
            "wire_insulator_connection": 260,
            "insulator_tower_side_connection": 260,
            "ground_wire_tower_connection": 220,
            "tower_base_connection": 220,
            "tower_top": 460,
            "tower_edge": 820,
            "tower_body": 360,
        }
        kept: List[CandidateViewpoint] = []
        for semantic, entries in scored_by_semantic.items():
            entries.sort(key=lambda item: item[0], reverse=True)
            semantic_limit = semantic_keep.get(semantic, 180)
            selected_for_semantic: Dict[int, CandidateViewpoint] = {
                candidate.id: candidate for _, candidate in entries[:semantic_limit]
            }
            repair_entries = [
                candidate for _, candidate in entries
                if getattr(candidate, "source", "") == "observability_gap_repair"
            ]
            for candidate in repair_entries[: min(len(repair_entries), max(60, semantic_limit // 3))]:
                selected_for_semantic.setdefault(candidate.id, candidate)
            kept.extend(selected_for_semantic.values())

        if len(kept) > self.max_candidate_pool:
            kept.sort(
                key=lambda candidate: (
                    getattr(candidate, "source", "") == "observability_gap_repair",
                    candidate.base_score,
                ),
                reverse=True,
            )
            kept = kept[: self.max_candidate_pool]
        self.candidates = kept
        self.candidate_map = {candidate.id: candidate for candidate in self.candidates}
        self.gap_repair_candidates_kept = int(
            sum(1 for candidate in self.candidates if getattr(candidate, "source", "") == "observability_gap_repair")
        )

    def _sync_position_groups(self):
        grouped: Dict[int, List[int]] = {}
        for candidate in self.candidates:
            grouped.setdefault(candidate.position_id, []).append(candidate.id)
        for candidate_ids in grouped.values():
            candidate_ids.sort(
                key=lambda candidate_id: (
                    -SEMANTIC_PRIORITY.get(self.candidate_map[candidate_id].semantic_focus, 0),
                    -self.candidate_map[candidate_id].base_score,
                    -self.candidate_map[candidate_id].f_eq_mm,
                )
            )
        self.position_candidate_ids = grouped

    def _target_signature(self, candidate: CandidateViewpoint) -> Tuple[object, ...]:
        if getattr(candidate, "target_id", None) is not None:
            return ("target", int(candidate.target_id))
        target = np.asarray(candidate.look_at if candidate.look_at is not None else candidate.target_center, dtype=float)
        return (
            "look_at",
            str(candidate.semantic_focus),
            round(float(target[0]), 2),
            round(float(target[1]), 2),
            round(float(target[2]), 2),
        )

    def _refresh_candidate_view_geometry(self, candidate: CandidateViewpoint) -> None:
        look_at = np.asarray(candidate.look_at if candidate.look_at is not None else candidate.target_center, dtype=float)
        geom = compute_view_geometry(candidate.position, look_at)
        candidate.yaw = float(geom["heading"])
        candidate.heading = float(geom["heading"])
        candidate.base_heading = float(geom["heading"])
        candidate.pitch = max(-85.0, min(25.0, float(geom["pitch"])))
        candidate.distance = float(geom["Distance"])
        aim_type = getattr(candidate, "aim_type", None) or candidate.semantic_focus
        req_resolution = (
            float(candidate.required_resolution)
            if getattr(candidate, "required_resolution", None) is not None
            else float(REQUIRED_RESOLUTION.get(_normalize_semantic(candidate.semantic_focus), 0.7))
        )
        focal = choose_focal_length_eq_mm(
            aim_type=aim_type,
            distance=float(candidate.distance),
            required_resolution=req_resolution,
        )
        if focal is None:
            focal = float(candidate.f_eq_mm or 48.0)
        focal = max(24.0, min(84.0, float(focal)))
        candidate.focal_length_eq_mm = focal
        candidate.f_eq_mm = focal
        focal_level, _ = next(
            iter(sorted(SUPPORTED_FOCALS.items(), key=lambda item: abs(float(item[1]["f_eq_mm"]) - focal)))
        )
        candidate.focal_level = focal_level
        candidate.hfov_deg = math.degrees(2.0 * math.atan(CAMERA_SENSOR_WIDTH_MM / (2.0 * focal)))
        candidate.vfov_deg = math.degrees(2.0 * math.atan(CAMERA_SENSOR_HEIGHT_MM / (2.0 * focal)))

    def _merge_nearby_candidate_positions(self, candidates: List[CandidateViewpoint], radius_m: float = 1.2) -> int:
        """Merge nearby safe candidate positions into physical waypoint groups."""
        if not candidates:
            return 0
        ordered = sorted(
            candidates,
            key=lambda candidate: (
                -float(candidate.base_score),
                -SEMANTIC_PRIORITY.get(candidate.semantic_focus, 0),
                int(candidate.position_id),
            ),
        )
        groups: List[Dict[str, object]] = []
        for candidate in ordered:
            assigned = False
            for group in groups:
                rep = np.asarray(group["position"], dtype=float)
                if float(np.linalg.norm(candidate.position - rep)) > radius_m:
                    continue
                candidate.position = rep.copy()
                candidate.position_id = int(group["position_id"])
                self._refresh_candidate_view_geometry(candidate)
                assigned = True
                break
            if assigned:
                continue
            group_id = len(groups) + 1
            candidate.position_id = group_id
            groups.append({"position_id": group_id, "position": np.asarray(candidate.position, dtype=float).copy()})
        if self.engine is not None:
            self.engine.cache.clear()
            self.engine.attention_cache.clear()
        return max(len(candidates) - len(groups), 0)

    def _effective_new_indices(self, indices: np.ndarray, covered_mask: np.ndarray) -> np.ndarray:
        assert self.env is not None
        if len(indices) == 0:
            return np.array([], dtype=int)
        effective_mask = self.env.effective_target_mask[indices]
        uncovered_mask = ~covered_mask[indices]
        return indices[np.logical_and(effective_mask, uncovered_mask)]

    def _effective_semantic_total(self, semantic: str) -> int:
        assert self.env is not None
        return max(int(self.env.effective_semantic_totals.get(semantic, 0)), 1)

    def _effective_weight_total(self) -> float:
        assert self.env is not None
        return max(float(self.env.effective_total_weight), 1e-9)

    def _position_counts(self, selected_ids: Sequence[int]) -> Counter:
        return Counter(self.candidate_map[candidate_id].position_id for candidate_id in selected_ids if candidate_id in self.candidate_map)

    def _selected_waypoint_count(self, selected_ids: Sequence[int]) -> int:
        return len(set(self.candidate_map[candidate_id].position_id for candidate_id in selected_ids if candidate_id in self.candidate_map))

    def _candidate_aim_limit(self, candidate: CandidateViewpoint) -> Tuple[Optional[int], Optional[Tuple[object, ...]]]:
        aim_type = getattr(candidate, "aim_type", None)
        if not aim_type:
            return None, None
        aim_profile = AIMTYPE_VIEW_PROFILE.get(aim_type, {})
        if aim_profile.get("max_final_waypoints"):
            return int(aim_profile["max_final_waypoints"]), ("aim", aim_type)
        if aim_profile.get("max_final_per_instance"):
            instance_key = (
                getattr(candidate, "instance_id", None)
                if getattr(candidate, "instance_id", None) is not None
                else getattr(candidate, "target_id", None)
            )
            if instance_key is None:
                instance_key = getattr(candidate, "target_cluster_id", None) or tuple(
                    round(float(v), 2) for v in np.asarray(candidate.target_center, dtype=float)
                )
            return int(aim_profile["max_final_per_instance"]), ("instance", aim_type, instance_key)
        if aim_profile.get("max_final_per_target"):
            target_key = (
                getattr(candidate, "target_id", None)
                if getattr(candidate, "target_id", None) is not None
                else getattr(candidate, "target_cluster_id", None)
            )
            if target_key is None:
                target_key = tuple(round(float(v), 2) for v in np.asarray(candidate.target_center, dtype=float))
            return int(aim_profile["max_final_per_target"]), ("target", aim_type, target_key)
        return None, None

    def _candidate_matches_aim_scope(self, candidate: CandidateViewpoint, scope: Tuple[object, ...]) -> bool:
        _, candidate_scope = self._candidate_aim_limit(candidate)
        return candidate_scope == scope

    def _last_position(self, selected_ids: Sequence[int]) -> Optional[np.ndarray]:
        if not selected_ids:
            return None
        return np.asarray(self.candidate_map[selected_ids[-1]].position, dtype=float)

    def _candidate_allowed(self, candidate: CandidateViewpoint, selected_ids: Sequence[int], position_counts: Optional[Counter] = None) -> bool:
        if candidate.id in selected_ids:
            return False
        position_counts = position_counts or self._position_counts(selected_ids)
        if position_counts.get(candidate.position_id, 0) >= self.max_shots_per_waypoint:
            return False
        if position_counts.get(candidate.position_id, 0) == 0 and self._selected_waypoint_count(selected_ids) >= self.max_waypoints:
            return False
        if len(selected_ids) >= self.max_total_shots:
            return False
        # ── Enforce AIMTYPE_VIEW_PROFILE max_final constraints ──
        max_final, scope = self._candidate_aim_limit(candidate)
        if max_final and scope:
            same_scope_count = sum(
                1
                for cid in selected_ids
                if cid in self.candidate_map and self._candidate_matches_aim_scope(self.candidate_map[cid], scope)
            )
            if same_scope_count >= int(max_final):
                return False
        return True

    def _evaluate_increment(
        self,
        candidate: CandidateViewpoint,
        covered_mask: np.ndarray,
        selected_ids: Sequence[int],
        focus_semantic: Optional[str] = None,
    ) -> Optional[Dict[str, object]]:
        assert self.env is not None and self.engine is not None
        visible = self.engine.candidate_visible_indices(candidate)
        if len(visible) == 0:
            return None
        new_indices = self._effective_new_indices(visible, covered_mask)
        if len(new_indices) == 0:
            return None

        semantics = self.env.semantics[new_indices]
        weights = self.env.weights[new_indices]
        attention_visible = self.engine.candidate_visible_attention_indices(candidate)
        attention_score = 0.0
        attention_deltas = {}
        if len(attention_visible):
            attention_semantics = self.env.attention_semantics[attention_visible]
            for semantic in ATTENTION_SEMANTICS:
                total = max(self.env.attention_totals.get(semantic, 0), 1)
                attention_deltas[semantic] = float(np.sum(attention_semantics == semantic) / total)
                attention_score += SEMANTIC_PRIORITY.get(semantic, 0) * attention_deltas[semantic]
        weighted_delta = float(np.sum(weights) / self._effective_weight_total())
        semantic_deltas = {}
        for semantic in SEMANTIC_PRIORITY:
            total = self._effective_semantic_total(semantic)
            semantic_deltas[semantic] = float(np.sum(semantics == semantic) / total)

        position_counts = self._position_counts(selected_ids)
        new_position = position_counts.get(candidate.position_id, 0) == 0
        last_position = self._last_position(selected_ids)
        path_distance = 0.0 if last_position is None or not new_position else float(np.linalg.norm(candidate.position - last_position))
        overlap_ratio = 1.0 - float(len(new_indices) / max(len(visible), 1))

        score = (
            12.0 * weighted_delta
            + 8.0 * semantic_deltas["insulator"]
            + 5.6 * semantic_deltas["tower_top"]
            + 4.0 * semantic_deltas["tower_edge"]
            + 1.6 * semantic_deltas["tower_body"]
            + 3.2 * attention_score
            + 1.4 * float(candidate.manual_priority)
        )
        if focus_semantic:
            score += 4.5 * semantic_deltas.get(focus_semantic, 0.0)
        score -= 0.85 if new_position else 0.10
        score -= 0.04
        score -= 0.018 * path_distance
        score -= 0.10 * overlap_ratio

        return {
            "candidate": candidate,
            "new_indices": new_indices,
            "visible_indices": visible,
            "weighted_delta": weighted_delta,
            "semantic_deltas": semantic_deltas,
            "attention_deltas": attention_deltas,
            "path_distance": path_distance,
            "overlap_ratio": overlap_ratio,
            "new_position": new_position,
            "score": score,
        }

    def rank_candidates(
        self,
        selected_ids: Sequence[int],
        covered_mask: np.ndarray,
        focus_semantic: Optional[str] = None,
        limit: Optional[int] = None,
        noise_scale: float = 0.0,
        rng: Optional[random.Random] = None,
    ) -> List[Tuple[float, CandidateViewpoint, Dict[str, object]]]:
        rng = rng or random.Random()
        position_counts = self._position_counts(selected_ids)
        ranked: List[Tuple[float, CandidateViewpoint, Dict[str, object]]] = []
        for candidate in self.candidates:
            if not self._candidate_allowed(candidate, selected_ids, position_counts):
                continue
            increment = self._evaluate_increment(candidate, covered_mask, selected_ids, focus_semantic=focus_semantic)
            if not increment:
                continue
            score = float(increment["score"]) + (rng.random() * noise_scale if noise_scale else 0.0)
            ranked.append((score, candidate, increment))
        ranked.sort(key=lambda item: item[0], reverse=True)
        return ranked[:limit] if limit else ranked

    def _semantic_uncovered_priority(self, covered_mask: np.ndarray) -> Dict[str, float]:
        assert self.env is not None
        priorities = {}
        for semantic, priority in SEMANTIC_PRIORITY.items():
            if semantic == "tower_body":
                priority = 0.35
            semantic_mask = np.logical_and(self.env.semantics == semantic, self.env.effective_target_mask)
            total = max(int(np.sum(semantic_mask)), 1)
            uncovered = int(np.sum(np.logical_and(semantic_mask, ~covered_mask)))
            priorities[semantic] = priority * float(uncovered / total)
        return priorities

    def _focus_semantic(self, covered_mask: np.ndarray) -> Optional[str]:
        priorities = self._semantic_uncovered_priority(covered_mask)
        focus, value = max(priorities.items(), key=lambda item: item[1])
        return focus if value > 0 else None

    def _mask_from_ids(self, selected_ids: Sequence[int]) -> np.ndarray:
        assert self.env is not None and self.engine is not None
        covered_mask = np.zeros(self.env.target_count, dtype=bool)
        for candidate_id in selected_ids:
            candidate = self.candidate_map[candidate_id]
            indices = self.engine.candidate_visible_indices(candidate)
            if len(indices):
                covered_mask[indices] = True
        return covered_mask

    def _attention_mask_from_ids(self, selected_ids: Sequence[int]) -> np.ndarray:
        assert self.env is not None and self.engine is not None
        covered_mask = np.zeros(self.env.attention_count, dtype=bool)
        for candidate_id in selected_ids:
            candidate = self.candidate_map[candidate_id]
            indices = self.engine.candidate_visible_attention_indices(candidate)
            if len(indices):
                covered_mask[indices] = True
        return covered_mask

    def _attention_metrics_from_ids(self, selected_ids: Sequence[int]) -> Dict[str, object]:
        assert self.env is not None
        return self.env.attention_coverage_from_mask(self._attention_mask_from_ids(selected_ids))

    def _selected_position_ids(self, selected_ids: Sequence[int]) -> List[int]:
        seen = []
        used = set()
        for candidate_id in selected_ids:
            position_id = self.candidate_map[candidate_id].position_id
            if position_id in used:
                continue
            used.add(position_id)
            seen.append(position_id)
        return seen

    def _followup_shot_has_meaningful_gain(
        self,
        semantic_deltas: Dict[str, float],
        weighted_delta: float,
        focus_semantic: Optional[str] = None,
    ) -> bool:
        """Return whether an extra shot at an existing waypoint is worth taking."""
        attention_gain = max((semantic_deltas.get(semantic, 0.0) for semantic in ATTENTION_SEMANTICS), default=0.0)
        key_semantic_gain = max(
            float(semantic_deltas.get("insulator", 0.0)),
            float(semantic_deltas.get("tower_top", 0.0)),
            float(semantic_deltas.get("tower_edge", 0.0)),
            float(attention_gain),
        )
        focus_gain = float(semantic_deltas.get(focus_semantic, 0.0)) if focus_semantic else 0.0
        return bool(
            weighted_delta >= 0.0008
            or key_semantic_gain >= 0.004
            or attention_gain >= 0.012
            or focus_gain >= 0.004
        )

    def _best_shots_for_position(
        self,
        position_id: int,
        covered_mask: np.ndarray,
        selected_ids: Optional[Sequence[int]] = None,
        focus_semantic: Optional[str] = None,
        rng: Optional[random.Random] = None,
        noise_scale: float = 0.0,
    ) -> Optional[Dict[str, object]]:
        assert self.env is not None and self.engine is not None
        rng = rng or random.Random()
        candidate_ids = self.position_candidate_ids.get(position_id, [])
        if not candidate_ids:
            return None

        local_mask = covered_mask.copy()
        local_attention_mask = self._attention_mask_from_ids(selected_ids or [])
        shot_ids: List[int] = []
        used_target_signatures: set[Tuple[object, ...]] = set()
        used_semantics: set[str] = set()
        semantic_gain = {semantic: 0.0 for semantic in SEMANTIC_PRIORITY}
        attention_gain = {semantic: 0.0 for semantic in ATTENTION_SEMANTICS}
        weighted_gain = 0.0
        best_candidate = self.candidate_map[candidate_ids[0]]

        for _ in range(self.max_shots_per_waypoint):
            ranked_local: List[Tuple[float, int, np.ndarray, np.ndarray, Dict[str, float], float]] = []
            for candidate_id in candidate_ids:
                if candidate_id in shot_ids:
                    continue
                candidate = self.candidate_map[candidate_id]
                target_signature = self._target_signature(candidate)
                if target_signature in used_target_signatures:
                    continue
                if not self._candidate_allowed(candidate, list(selected_ids or []) + shot_ids):
                    continue
                visible = self.engine.candidate_visible_indices(candidate)
                if len(visible) == 0:
                    continue
                new_indices = self._effective_new_indices(visible, local_mask)
                if len(new_indices) == 0:
                    continue
                semantics = self.env.semantics[new_indices]
                weights = self.env.weights[new_indices]
                delta_weighted = float(np.sum(weights) / self._effective_weight_total())
                semantic_deltas = {}
                for semantic in SEMANTIC_PRIORITY:
                    if semantic in ATTENTION_SEMANTICS:
                        semantic_deltas[semantic] = 0.0
                    else:
                        total = self._effective_semantic_total(semantic)
                        semantic_deltas[semantic] = float(np.sum(semantics == semantic) / total)
                attention_visible = self.engine.candidate_visible_attention_indices(candidate)
                attention_score = 0.0
                if len(attention_visible):
                    new_attention = attention_visible[~local_attention_mask[attention_visible]]
                    attention_semantics = self.env.attention_semantics[new_attention]
                    for semantic in ATTENTION_SEMANTICS:
                        total = max(self.env.attention_totals.get(semantic, 0), 1)
                        semantic_deltas[semantic] = float(np.sum(attention_semantics == semantic) / total)
                        attention_score += SEMANTIC_PRIORITY.get(semantic, 0) * semantic_deltas[semantic]
                else:
                    new_attention = np.array([], dtype=int)
                if shot_ids and not self._followup_shot_has_meaningful_gain(
                    semantic_deltas,
                    delta_weighted,
                    focus_semantic=focus_semantic,
                ):
                    continue
                score = (
                    12.0 * delta_weighted
                    + 7.0 * semantic_deltas["insulator"]
                    + 5.2 * semantic_deltas["tower_top"]
                    + 4.4 * semantic_deltas["tower_edge"]
                    + 2.2 * semantic_deltas["tower_body"]
                    + 3.0 * attention_score
                    + 1.2 * float(candidate.manual_priority)
                    + 0.12 * candidate.base_score
                    - 0.06 * len(shot_ids)
                )
                if shot_ids and candidate.semantic_focus not in used_semantics:
                    score += 0.24
                if focus_semantic:
                    score += 4.8 * semantic_deltas.get(focus_semantic, 0.0)
                if noise_scale:
                    score += rng.random() * noise_scale
                ranked_local.append((score, candidate_id, new_indices, new_attention, semantic_deltas, delta_weighted))

            if not ranked_local:
                break

            ranked_local.sort(key=lambda item: item[0], reverse=True)
            score, chosen_id, new_indices, new_attention, semantic_deltas, _ = ranked_local[0]
            if score <= 0.0 and shot_ids:
                break
            shot_ids.append(chosen_id)
            chosen_candidate = self.candidate_map[chosen_id]
            used_target_signatures.add(self._target_signature(chosen_candidate))
            used_semantics.add(str(chosen_candidate.semantic_focus))
            local_mask[new_indices] = True
            if len(new_attention):
                local_attention_mask[new_attention] = True
            best_candidate = self.candidate_map[chosen_id]
            weighted_gain += float(np.sum(self.env.weights[new_indices]) / self._effective_weight_total())
            for semantic, value in semantic_deltas.items():
                semantic_gain[semantic] += float(value)
                if semantic in attention_gain:
                    attention_gain[semantic] += float(value)

        if not shot_ids:
            return None

        new_indices = np.where(np.logical_and.reduce([local_mask, ~covered_mask, self.env.effective_target_mask]))[0]
        if len(new_indices) == 0:
            return None

        coverage_metrics = self.env.coverage_from_mask(local_mask, include_uncovered=False)
        score = (
            12.0 * weighted_gain
            + 7.0 * semantic_gain["insulator"]
            + 4.8 * semantic_gain["tower_top"]
            + 4.2 * semantic_gain["tower_edge"]
            + 2.0 * semantic_gain["tower_body"]
            + 2.6 * sum(attention_gain.values())
            - 0.12 * max(len(shot_ids) - 1, 0)
        )
        if focus_semantic:
            score += 3.8 * semantic_gain.get(focus_semantic, 0.0)

        return {
            "position_id": position_id,
            "shot_ids": shot_ids,
            "covered_mask": local_mask,
            "new_indices": new_indices,
            "weighted_delta": weighted_gain,
            "semantic_deltas": semantic_gain,
            "coverage_metrics": coverage_metrics,
            "score": score,
            "position": np.asarray(best_candidate.position, dtype=float),
            "shot_count": len(shot_ids),
        }

    def _rank_positions(
        self,
        selected_ids: Sequence[int],
        covered_mask: np.ndarray,
        focus_semantic: Optional[str] = None,
        limit: Optional[int] = None,
        randomized: bool = False,
        rng: Optional[random.Random] = None,
    ) -> List[Dict[str, object]]:
        rng = rng or random.Random()
        used_positions = set(self._selected_position_ids(selected_ids))
        ranked: List[Dict[str, object]] = []
        last_position = self._last_position(selected_ids)
        for position_id in self.position_candidate_ids:
            if position_id in used_positions:
                continue
            # ── Enforce AIMTYPE max_final constraints ──
            cands_at_pos = self.position_candidate_ids.get(position_id, [])
            if cands_at_pos:
                if not any(
                    self._candidate_allowed(self.candidate_map[candidate_id], selected_ids)
                    for candidate_id in cands_at_pos
                    if candidate_id in self.candidate_map
                ):
                    continue
            increment = self._best_shots_for_position(
                position_id,
                covered_mask,
                selected_ids=selected_ids,
                focus_semantic=focus_semantic,
                rng=rng,
                noise_scale=0.06 if randomized else 0.0,
            )
            if not increment:
                continue
            if last_position is not None:
                path_distance = float(np.linalg.norm(increment["position"] - last_position))
                increment["score"] -= 0.012 * path_distance
            ranked.append(increment)
        ranked.sort(key=lambda item: item["score"], reverse=True)
        return ranked[:limit] if limit else ranked

    def _construct_position_greedy_solution(self, randomized: bool = False, seed: int = 0) -> Tuple[List[int], np.ndarray]:
        assert self.env is not None
        rng = random.Random(seed)
        selected_ids: List[int] = []
        covered_mask = np.zeros(self.env.target_count, dtype=bool)
        for step in range(self.max_waypoints):
            focus = self._focus_semantic(covered_mask)
            ranked = self._rank_positions(
                selected_ids,
                covered_mask,
                focus_semantic=focus,
                limit=10 if randomized else 6,
                randomized=randomized,
                rng=rng,
            )
            if not ranked:
                if not self.stop_reason:
                    # Distinguish true exhaustion from zero-gain
                    unselected = [cid for cid in self.candidate_map if cid not in selected_ids]
                    if len(unselected) == 0:
                        self.stop_reason = "candidates_exhausted"
                    else:
                        self.stop_reason = "no_positive_gain_remaining"
                break
            # ── Enforce tower_overview max_final constraint ──
            overview_selected = sum(
                1 for cid in selected_ids
                if getattr(self.candidate_map.get(cid), "aim_type", None) == "tower_overview"
            )
            if overview_selected >= 3:
                ranked = [item for item in ranked if not any(
                    getattr(self.candidate_map.get(cid), "aim_type", None) == "tower_overview"
                    for cid in item.get("shot_ids", [])
                )]
                if not ranked:
                    if not self.stop_reason:
                        self.stop_reason = "candidates_exhausted"
                    break
            if randomized and len(ranked) > 1:
                choice_count = min(4, len(ranked))
                weights = [max(float(item["score"]), 1e-6) for item in ranked[:choice_count]]
                chosen = rng.choices(ranked[:choice_count], weights=weights, k=1)[0]
            else:
                chosen = ranked[0]
            selected_ids.extend(chosen["shot_ids"])
            covered_mask = chosen["covered_mask"]
            metrics = self.env.coverage_from_mask(covered_mask, include_uncovered=False)
            wp_count = self._selected_waypoint_count(selected_ids)
            # Enforce max photo waypoints
            if wp_count >= self.max_photo_waypoints:
                self.stop_reason = "max_waypoints_reached"
                break
            if self._compact_thresholds_met(metrics) and wp_count >= self.min_waypoints:
                self.stop_reason = "compact_thresholds_met"
                break
            if coverage_thresholds_met(metrics) and wp_count >= self.min_waypoints:
                self.stop_reason = "coverage_thresholds_met"
                break
            if chosen["weighted_delta"] < 8.0e-4 and wp_count >= self.min_waypoints:
                self.stop_reason = "marginal_gain_too_low"
                break
        else:
            # Loop exhausted all max_waypoints iterations
            if self._selected_waypoint_count(selected_ids) < self.min_photo_waypoints:
                self.stop_reason = "candidates_exhausted_below_min"
            else:
                self.stop_reason = "max_iterations_reached"
        return selected_ids, covered_mask

    def _position_beam_search_core(self, beam_width: int = 8, branch_width: int = 3) -> List[int]:
        assert self.env is not None
        empty_mask = np.zeros(self.env.target_count, dtype=bool)
        beam = [BeamState(selected_ids=[], covered_mask=empty_mask, metrics=self.env.coverage_from_mask(empty_mask, include_uncovered=False))]
        best_state = beam[0]
        for _ in range(self.max_waypoints):
            expansions: List[BeamState] = []
            for state in beam:
                focus = self._focus_semantic(state.covered_mask)
                ranked = self._rank_positions(state.selected_ids, state.covered_mask, focus_semantic=focus, limit=branch_width * 2)
                for increment in ranked[:branch_width]:
                    new_ids = list(state.selected_ids) + list(increment["shot_ids"])
                    expansions.append(
                        BeamState(
                            selected_ids=new_ids,
                            covered_mask=increment["covered_mask"],
                            metrics=increment["coverage_metrics"],
                        )
                    )
            if not expansions:
                break
            expansions.sort(key=lambda item: self._solution_key(item.selected_ids, item.metrics), reverse=True)
            next_beam: List[BeamState] = []
            signatures = set()
            for state in expansions:
                signature = tuple(sorted(self._selected_position_ids(state.selected_ids)))
                if signature in signatures:
                    continue
                signatures.add(signature)
                next_beam.append(state)
                if len(next_beam) >= beam_width:
                    break
            beam = next_beam
            if beam and self._solution_key(beam[0].selected_ids, beam[0].metrics) > self._solution_key(best_state.selected_ids, best_state.metrics):
                best_state = beam[0]
            if beam and coverage_thresholds_met(beam[0].metrics) and self._selected_waypoint_count(beam[0].selected_ids) >= self.min_waypoints:
                best_state = beam[0]
                break
        return list(best_state.selected_ids)

    def _coverage_metrics(self, selected_ids: Sequence[int]) -> Dict[str, object]:
        assert self.env is not None
        metrics = self.env.coverage_from_mask(self._mask_from_ids(selected_ids))
        metrics.update(self._attention_metrics_from_ids(selected_ids))
        return metrics

    def _compact_thresholds_met(self, metrics: Dict[str, object]) -> bool:
        return compact_coverage_thresholds_met(metrics)

    def _hard_status(self, metrics: Dict[str, object]) -> str:
        return "success" if coverage_thresholds_met(metrics) else "infeasible"

    def _compact_status(self, metrics: Dict[str, object]) -> str:
        return "success" if self._compact_thresholds_met(metrics) else "failed"

    def _compact_solution_key(self, selected_ids: Sequence[int], metrics: Dict[str, object]) -> Tuple[float, ...]:
        compact_ok = int(self._compact_thresholds_met(metrics))
        hard_ok = int(coverage_thresholds_met(metrics))
        waypoint_count = self._selected_waypoint_count(selected_ids)
        effective_key_map = {
            "C_geo": "C_geo_effective",
            "C_weighted": "C_weighted_effective",
            "C_ins": "C_ins_effective",
            "C_top": "C_top_effective",
            "C_edge": "C_edge_effective",
        }
        deficits = [
            max(0.0, float(threshold) - float(metrics.get(effective_key_map.get(key, key), metrics.get(key)) or 0.0))
            for key, threshold in COMPACT_COVERAGE_THRESHOLDS.items()
        ]
        max_deficit = max(deficits) if deficits else 0.0
        total_deficit = sum(deficits)
        return (
            compact_ok,
            hard_ok,
            -float(max_deficit),
            -float(total_deficit),
            float(metrics.get("C_weighted_effective", metrics.get("C_weighted")) or 0.0),
            float(metrics.get("C_geo_effective", metrics.get("C_geo")) or 0.0),
            float(metrics.get("C_ins_effective", metrics.get("C_ins")) or 0.0),
            float(metrics.get("C_top_effective", metrics.get("C_top")) or 0.0),
            float(metrics.get("C_edge_effective", metrics.get("C_edge")) or 0.0),
            -float(waypoint_count),
        )

    def _grouped_selection_ids(self, selected_ids: Sequence[int]) -> Dict[int, List[int]]:
        grouped: Dict[int, List[int]] = {}
        for candidate_id in selected_ids:
            candidate = self.candidate_map.get(candidate_id)
            if candidate is None:
                continue
            grouped.setdefault(candidate.position_id, []).append(candidate_id)
        return grouped

    def _prune_compact_waypoint_groups(self, selected_ids: Sequence[int]) -> List[int]:
        selected = list(dict.fromkeys(int(candidate_id) for candidate_id in selected_ids))
        if not selected:
            return selected
        current_metrics = self._coverage_metrics(selected)
        if not self._compact_thresholds_met(current_metrics):
            return selected
        changed = True
        while changed:
            changed = False
            grouped = self._grouped_selection_ids(selected)
            group_order = sorted(
                grouped.items(),
                key=lambda item: (
                    len(item[1]),
                    sum(self.candidate_map[cid].base_score for cid in item[1]),
                    min(item[1]),
                ),
            )
            for position_id, candidate_ids in group_order:
                trial = [candidate_id for candidate_id in selected if candidate_id not in set(candidate_ids)]
                if self._selected_waypoint_count(trial) < self.min_waypoints:
                    continue
                trial_metrics = self._coverage_metrics(trial)
                if self._compact_thresholds_met(trial_metrics):
                    selected = trial
                    current_metrics = trial_metrics
                    changed = True
                    break
        changed = True
        while changed:
            changed = False
            for candidate_id in sorted(selected, key=lambda cid: self.candidate_map[cid].base_score):
                grouped = self._grouped_selection_ids(selected)
                if len(grouped.get(self.candidate_map[candidate_id].position_id, [])) <= 1:
                    continue
                trial = [cid for cid in selected if cid != candidate_id]
                trial_metrics = self._coverage_metrics(trial)
                if self._compact_thresholds_met(trial_metrics):
                    selected = trial
                    changed = True
                    break
        return selected

    def _compact_local_search(self, selected_ids: Sequence[int], iterations: int = 24) -> List[int]:
        selected = self._prune_compact_waypoint_groups(selected_ids)
        best = list(selected)
        best_metrics = self._coverage_metrics(best)
        rng = random.Random(20260528 + len(best))
        for _ in range(iterations):
            action = rng.choice(["remove_group", "replace_group", "add_shot", "remove_shot"])
            current = list(best)
            grouped = self._grouped_selection_ids(current)
            if action == "remove_group" and grouped:
                group_ids = rng.choice(list(grouped.values()))
                trial = [cid for cid in current if cid not in set(group_ids)]
            elif action == "remove_shot" and len(current) > 1:
                removable = [cid for cid in current if len(grouped.get(self.candidate_map[cid].position_id, [])) > 1]
                if not removable:
                    continue
                remove_id = rng.choice(removable)
                trial = [cid for cid in current if cid != remove_id]
            elif action == "add_shot":
                expandable = [
                    position_id
                    for position_id, ids in grouped.items()
                    if len(ids) < self.max_shots_per_waypoint
                ]
                if not expandable:
                    continue
                position_id = rng.choice(expandable)
                local_mask = self._mask_from_ids(current)
                best_local = self._best_shots_for_position(position_id, local_mask, selected_ids=current)
                if not best_local:
                    continue
                trial = current + [cid for cid in best_local["shot_ids"] if cid not in current]
            elif action == "replace_group" and grouped:
                group_ids = rng.choice(list(grouped.values()))
                base = [cid for cid in current if cid not in set(group_ids)]
                covered_mask = self._mask_from_ids(base)
                focus = self._focus_semantic(covered_mask)
                ranked = self._rank_positions(base, covered_mask, focus_semantic=focus, limit=6, randomized=True, rng=rng)
                if not ranked:
                    continue
                trial = base + list(ranked[0]["shot_ids"])
            else:
                continue
            if self._selected_waypoint_count(trial) > self.max_waypoints or len(trial) > self.max_total_shots:
                continue
            metrics = self._coverage_metrics(trial)
            if self._compact_solution_key(trial, metrics) > self._compact_solution_key(best, best_metrics):
                best = self._prune_compact_waypoint_groups(trial)
                best_metrics = self._coverage_metrics(best)
        return best

    def _repair_compact_deficits(self, selected_ids: Sequence[int]) -> List[int]:
        assert self.env is not None
        selected = list(selected_ids)
        covered_mask = self._mask_from_ids(selected)
        semantic_metric = {
            "insulator": "C_ins",
            "tower_top": "C_top",
            "tower_edge": "C_edge",
        }
        for _ in range(max(0, self.max_waypoints - self._selected_waypoint_count(selected))):
            metrics = self.env.coverage_from_mask(covered_mask, include_uncovered=False)
            if self._compact_thresholds_met(metrics):
                break
            deficits = {
                semantic: COMPACT_COVERAGE_THRESHOLDS[metric] - float(metrics.get(metric) or 0.0)
                for semantic, metric in semantic_metric.items()
            }
            focus = max(deficits, key=lambda semantic: deficits[semantic])
            if deficits[focus] <= 0.0:
                focus = self._focus_semantic(covered_mask) or focus
            ranked = self._rank_positions(selected, covered_mask, focus_semantic=focus, limit=8)
            if not ranked:
                break
            increment = ranked[0]
            if increment["semantic_deltas"].get(focus, 0.0) <= 0.0 and increment["weighted_delta"] < 0.001:
                break
            selected.extend(increment["shot_ids"])
            covered_mask = increment["covered_mask"]
            if len(selected) >= self.max_total_shots or self._selected_waypoint_count(selected) >= self.max_waypoints:
                break
        return selected

    def _multi_shot_target_count(self, selected_ids: Sequence[int]) -> int:
        waypoint_count = self._selected_waypoint_count(selected_ids)
        if waypoint_count <= 0:
            return len(selected_ids)
        preferred = int(math.ceil(float(waypoint_count) * 1.55))
        hard_cap = min(self.max_total_shots, waypoint_count * self.max_shots_per_waypoint)
        return max(len(selected_ids), min(preferred, hard_cap))

    def _candidate_followup_score(
        self,
        candidate: CandidateViewpoint,
        covered_mask: np.ndarray,
        attention_mask: np.ndarray,
        used_semantics: set[str],
    ) -> Optional[Tuple[float, np.ndarray, np.ndarray, Dict[str, float], float]]:
        assert self.env is not None and self.engine is not None
        visible = self.engine.candidate_visible_indices(candidate)
        if len(visible) == 0:
            return None
        visible_effective = visible[self.env.effective_target_mask[visible]]
        if len(visible_effective) == 0:
            return None
        new_indices = visible_effective[~covered_mask[visible_effective]]
        semantics = self.env.semantics[new_indices] if len(new_indices) else np.array([], dtype=object)
        weights = self.env.weights[new_indices] if len(new_indices) else np.array([], dtype=float)
        weighted_delta = float(np.sum(weights) / self._effective_weight_total()) if len(weights) else 0.0
        semantic_deltas: Dict[str, float] = {}
        for semantic in SEMANTIC_PRIORITY:
            if semantic in ATTENTION_SEMANTICS:
                semantic_deltas[semantic] = 0.0
            else:
                total = self._effective_semantic_total(semantic)
                semantic_deltas[semantic] = float(np.sum(semantics == semantic) / total) if len(semantics) else 0.0

        attention_visible = self.engine.candidate_visible_attention_indices(candidate)
        if len(attention_visible):
            new_attention = attention_visible[~attention_mask[attention_visible]]
            attention_semantics = self.env.attention_semantics[new_attention]
        else:
            new_attention = np.array([], dtype=int)
            attention_semantics = np.array([], dtype=object)
        attention_score = 0.0
        for semantic in ATTENTION_SEMANTICS:
            total = max(self.env.attention_totals.get(semantic, 0), 1)
            semantic_deltas[semantic] = float(np.sum(attention_semantics == semantic) / total) if len(attention_semantics) else 0.0
            attention_score += SEMANTIC_PRIORITY.get(semantic, 0) * semantic_deltas[semantic]

        meaningful_new = self._followup_shot_has_meaningful_gain(semantic_deltas, weighted_delta)
        visible_ratio = float(len(visible_effective) / max(int(np.sum(self.env.effective_target_mask)), 1))
        same_semantic_penalty = 0.10 if candidate.semantic_focus in used_semantics else 0.0
        context_score = (
            1.4 * visible_ratio
            + 0.08 * candidate.base_score
            + 0.002 * len(visible_effective)
            - same_semantic_penalty
        )
        if not meaningful_new and context_score < 0.004:
            return None

        score = (
            14.0 * weighted_delta
            + 7.0 * semantic_deltas.get("insulator", 0.0)
            + 5.2 * semantic_deltas.get("tower_top", 0.0)
            + 4.4 * semantic_deltas.get("tower_edge", 0.0)
            + 2.2 * semantic_deltas.get("tower_body", 0.0)
            + 3.0 * attention_score
            + context_score
            + (0.18 if candidate.semantic_focus not in used_semantics else 0.0)
            + 0.7 * float(candidate.manual_priority)
        )
        return score, new_indices, new_attention, semantic_deltas, weighted_delta

    def _enrich_existing_waypoints_with_shots(self, selected_ids: Sequence[int]) -> List[int]:
        """Add useful second shots to already selected waypoint groups without adding positions."""
        assert self.env is not None and self.engine is not None
        selected = list(selected_ids)
        target_count = self._multi_shot_target_count(selected)
        if len(selected) >= target_count or self.max_shots_per_waypoint <= 1:
            return selected

        while len(selected) < target_count and len(selected) < self.max_total_shots:
            grouped = self._grouped_selection_ids(selected)
            covered_mask = self._mask_from_ids(selected)
            attention_mask = self._attention_mask_from_ids(selected)
            selected_set = set(selected)
            ranked: List[Tuple[float, int, np.ndarray, np.ndarray]] = []

            for position_id, group_ids in grouped.items():
                if len(group_ids) >= self.max_shots_per_waypoint:
                    continue
                used_signatures = {
                    self._target_signature(self.candidate_map[candidate_id])
                    for candidate_id in group_ids
                    if candidate_id in self.candidate_map
                }
                used_semantics = {
                    str(self.candidate_map[candidate_id].semantic_focus)
                    for candidate_id in group_ids
                    if candidate_id in self.candidate_map
                }
                for candidate_id in self.position_candidate_ids.get(position_id, []):
                    if candidate_id in selected_set:
                        continue
                    candidate = self.candidate_map[candidate_id]
                    if self._target_signature(candidate) in used_signatures:
                        continue
                    if not self._candidate_allowed(candidate, selected):
                        continue
                    scored = self._candidate_followup_score(candidate, covered_mask, attention_mask, used_semantics)
                    if scored is None:
                        continue
                    score, new_indices, new_attention, _, _ = scored
                    ranked.append((score, candidate_id, new_indices, new_attention))

            if not ranked:
                break
            ranked.sort(key=lambda item: item[0], reverse=True)
            score, chosen_id, _, _ = ranked[0]
            if score <= 0.0:
                break
            selected.append(chosen_id)
            self.multi_shot_enrichment_added_ids.add(chosen_id)

        return selected

    def _coverage_gain_trace(self, ordered_ids: Sequence[int], tail_count: int = 20) -> Tuple[List[Dict[str, object]], Dict[str, object]]:
        assert self.env is not None
        grouped = self._grouped_selection_ids(ordered_ids)
        ordered_positions: List[int] = []
        seen_positions = set()
        for candidate_id in ordered_ids:
            candidate = self.candidate_map.get(candidate_id)
            if candidate is None or candidate.position_id in seen_positions:
                continue
            seen_positions.add(candidate.position_id)
            ordered_positions.append(candidate.position_id)
        covered_mask = np.zeros(self.env.target_count, dtype=bool)
        total_weight = self._effective_weight_total()
        trace: List[Dict[str, object]] = []
        for waypoint_index, position_id in enumerate(ordered_positions, start=1):
            before = covered_mask.copy()
            for candidate_id in grouped.get(position_id, []):
                indices = self.engine.candidate_visible_indices(self.candidate_map[candidate_id]) if self.engine is not None else np.array([], dtype=int)
                if len(indices):
                    covered_mask[indices] = True
            new_mask = np.logical_and.reduce([covered_mask, ~before, self.env.effective_target_mask])
            new_count = int(np.sum(new_mask))
            weighted_gain = float(np.sum(self.env.weights[new_mask]) / total_weight) if new_count else 0.0
            trace.append({
                "waypoint_index": int(waypoint_index),
                "position_id": int(position_id),
                "shot_count": int(len(grouped.get(position_id, []))),
                "new_effective_count": new_count,
                "weighted_gain": round(weighted_gain, 6),
                "geo_gain": round(float(new_count / max(int(np.sum(self.env.effective_target_mask)), 1)), 6),
                "sources": dict(Counter(str(self.candidate_map[cid].source) for cid in grouped.get(position_id, []))),
                "semantics": dict(Counter(str(self.candidate_map[cid].semantic_focus) for cid in grouped.get(position_id, []))),
            })
        last = trace[-tail_count:]
        summary = {
            "last_waypoint_count": int(len(last)),
            "new_effective_count": int(sum(item["new_effective_count"] for item in last)),
            "weighted_gain": round(float(sum(float(item["weighted_gain"]) for item in last)), 6),
            "geo_gain": round(float(sum(float(item["geo_gain"]) for item in last)), 6),
            "low_gain_redundant": bool(last and sum(float(item["weighted_gain"]) for item in last) < 0.025),
        }
        return last, summary

    def _semantic_waypoint_budget(self, total_budget: Optional[int] = None) -> Dict[str, int]:
        total = int(total_budget or self.max_waypoints)
        weights = {
            "insulator": 0.38,
            "tower_top": 0.18,
            "tower_edge": 0.26,
            "tower_body": 0.04,
            "connection": 0.14,
        }
        budget = {semantic: max(1, int(round(total * ratio))) for semantic, ratio in weights.items()}
        while sum(budget.values()) > total:
            key = max(budget, key=lambda semantic: budget[semantic])
            budget[key] = max(1, budget[key] - 1)
        while sum(budget.values()) < total:
            budget["insulator"] += 1
        return budget

    def _position_focus_semantic(self, position_id: int) -> str:
        candidate_ids = self.position_candidate_ids.get(position_id, [])
        semantics = [self.candidate_map[cid].semantic_focus for cid in candidate_ids if cid in self.candidate_map]
        if any(semantic in ATTENTION_SEMANTICS for semantic in semantics):
            return "connection"
        if not semantics:
            return "tower_body"
        return Counter(semantics).most_common(1)[0][0]

    def _selected_semantic_waypoint_counts(self, selected_ids: Sequence[int]) -> Dict[str, int]:
        counts = {"insulator": 0, "tower_top": 0, "tower_edge": 0, "tower_body": 0, "connection": 0}
        for position_id in self._grouped_selection_ids(selected_ids):
            semantic = self._position_focus_semantic(position_id)
            if semantic not in counts:
                semantic = "connection" if semantic in ATTENTION_SEMANTICS else "tower_body"
            counts[semantic] = counts.get(semantic, 0) + 1
        return counts

    def _coverage_tuple(self, metrics: Dict[str, object]) -> Tuple[float, float, float, float, float, float, float, float]:
        return (
            float(metrics.get("C_ins") or 0.0),
            float(metrics.get("C_connection_attention") or 0.0),
            float(metrics.get("C_tower_base_connection") or 0.0),
            float(metrics.get("C_top") or 0.0),
            float(metrics.get("C_edge") or 0.0),
            float(metrics.get("C_weighted") or 0.0),
            float(metrics.get("C_geo") or 0.0),
            float(metrics.get("C_body") or 0.0),
        )

    def _solution_key(self, selected_ids: Sequence[int], metrics: Dict[str, object]) -> Tuple[float, float, float, float, float, float, float, int, int, float]:
        return (
            float(metrics.get("C_ins") or 0.0),
            float(metrics.get("C_connection_attention") or 0.0),
            float(metrics.get("C_tower_base_connection") or 0.0),
            float(metrics.get("C_top") or 0.0),
            float(metrics.get("C_edge") or 0.0),
            float(metrics.get("C_weighted") or 0.0),
            float(metrics.get("C_geo") or 0.0),
            -self._selected_waypoint_count(selected_ids),
            -len(selected_ids),
            float(metrics.get("C_body") or 0.0),
        )

    def _sample_ranked(self, ranked: Sequence[Tuple[float, CandidateViewpoint, Dict[str, object]]], rng: random.Random, temperature: float = 0.18):
        if len(ranked) == 1:
            return ranked[0][1], ranked[0][2]
        scores = np.array([item[0] for item in ranked], dtype=float)
        scores = scores - np.max(scores)
        probs = np.exp(scores / max(temperature, 1e-4))
        probs = probs / max(np.sum(probs), 1e-9)
        choice = int(rng.choices(range(len(ranked)), weights=probs.tolist(), k=1)[0])
        return ranked[choice][1], ranked[choice][2]

    def _construct_greedy_solution(self, randomized: bool = False, seed: int = 0) -> Tuple[List[int], np.ndarray]:
        assert self.env is not None
        rng = random.Random(seed)
        selected_ids: List[int] = []
        covered_mask = np.zeros(self.env.target_count, dtype=bool)
        for step in range(self.max_total_shots):
            focus = self._focus_semantic(covered_mask)
            ranked = self.rank_candidates(
                selected_ids,
                covered_mask,
                focus_semantic=focus,
                limit=18 if randomized else 8,
                rng=rng,
            )
            if not ranked:
                break
            if randomized:
                candidate, increment = self._sample_ranked(ranked[:8], rng, temperature=0.16)
            else:
                candidate, increment = ranked[0][1], ranked[0][2]
            selected_ids.append(candidate.id)
            covered_mask[increment["new_indices"]] = True
            metrics = self.env.coverage_from_mask(covered_mask, include_uncovered=False)
            if coverage_thresholds_met(metrics) and self._selected_waypoint_count(selected_ids) >= self.min_waypoints:
                break
            if increment["weighted_delta"] < 2.5e-4 and self._selected_waypoint_count(selected_ids) >= self.min_waypoints:
                break
            if self._selected_waypoint_count(selected_ids) >= self.max_waypoints and len(selected_ids) >= self.max_total_shots:
                break
        return selected_ids, covered_mask

    def _beam_search_core(self, beam_width: int = 10, branch_width: int = 4) -> List[int]:
        assert self.env is not None
        empty_mask = np.zeros(self.env.target_count, dtype=bool)
        beam = [BeamState(selected_ids=[], covered_mask=empty_mask, metrics=self.env.coverage_from_mask(empty_mask, include_uncovered=False))]
        best_state = beam[0]
        for step in range(self.max_total_shots):
            expansions: List[BeamState] = []
            for state in beam:
                focus = self._focus_semantic(state.covered_mask)
                ranked = self.rank_candidates(state.selected_ids, state.covered_mask, focus_semantic=focus, limit=14)
                for _, candidate, increment in ranked[:branch_width]:
                    new_ids = state.selected_ids + [candidate.id]
                    new_mask = state.covered_mask.copy()
                    new_mask[increment["new_indices"]] = True
                    metrics = self.env.coverage_from_mask(new_mask, include_uncovered=False)
                    expansions.append(BeamState(selected_ids=new_ids, covered_mask=new_mask, metrics=metrics))
            if not expansions:
                break
            expansions.sort(key=lambda item: self._solution_key(item.selected_ids, item.metrics), reverse=True)
            next_beam = []
            signatures = set()
            for state in expansions:
                signature = tuple(sorted(state.selected_ids))
                if signature in signatures:
                    continue
                signatures.add(signature)
                next_beam.append(state)
                if len(next_beam) >= beam_width:
                    break
            beam = next_beam
            if beam and self._solution_key(beam[0].selected_ids, beam[0].metrics) > self._solution_key(best_state.selected_ids, best_state.metrics):
                best_state = beam[0]
            if beam and coverage_thresholds_met(beam[0].metrics) and self._selected_waypoint_count(beam[0].selected_ids) >= self.min_waypoints:
                best_state = beam[0]
                break
        return list(best_state.selected_ids)

    def _repair_tower_top_replacement(self, selected_ids: Sequence[int]) -> List[int]:
        """Replace low-value shots with tower-top shots when the waypoint cap is already full."""
        assert self.env is not None
        selected = list(selected_ids)
        if not selected:
            return selected
        current_metrics = self.env.coverage_from_mask(self._mask_from_ids(selected), include_uncovered=False)
        current_top = float(current_metrics.get("C_top") or 0.0)
        if current_top >= COVERAGE_THRESHOLDS["C_top"]:
            return selected
        if self._selected_waypoint_count(selected) < self.max_waypoints:
            return selected

        selected_set = set(selected)
        counts = self._position_counts(selected)
        removable = sorted(
            selected,
            key=lambda candidate_id: (
                self.candidate_map[candidate_id].semantic_focus in {"tower_top", "insulator", *ATTENTION_SEMANTICS},
                counts.get(self.candidate_map[candidate_id].position_id, 0) <= 1,
                self.candidate_map[candidate_id].base_score,
            ),
        )
        top_candidates = [
            candidate
            for candidate in self.candidates
            if candidate.id not in selected_set and candidate.semantic_focus == "tower_top"
        ]
        top_candidates.sort(key=lambda candidate: (candidate.base_score, candidate.manual_priority, candidate.f_eq_mm), reverse=True)

        best_selected = selected
        best_metrics = current_metrics
        best_key = self._solution_key(selected, current_metrics)
        for remove_id in removable[: min(len(removable), 24)]:
            base = [candidate_id for candidate_id in selected if candidate_id != remove_id]
            for candidate in top_candidates[:80]:
                trial = base + [candidate.id]
                if len(trial) > self.max_total_shots or self._selected_waypoint_count(trial) > self.max_waypoints:
                    continue
                trial_metrics = self.env.coverage_from_mask(self._mask_from_ids(trial), include_uncovered=False)
                trial_top = float(trial_metrics.get("C_top") or 0.0)
                if trial_top <= float(best_metrics.get("C_top") or 0.0) + 1e-6:
                    continue
                if float(trial_metrics.get("C_ins") or 0.0) + 0.005 < float(current_metrics.get("C_ins") or 0.0):
                    continue
                if float(trial_metrics.get("C_weighted") or 0.0) + 0.01 < float(current_metrics.get("C_weighted") or 0.0):
                    continue
                trial_key = self._solution_key(trial, trial_metrics)
                if trial_key > best_key or trial_top > current_top + 0.01:
                    best_selected = trial
                    best_metrics = trial_metrics
                    best_key = trial_key
            if float(best_metrics.get("C_top") or 0.0) >= COVERAGE_THRESHOLDS["C_top"]:
                break
        return best_selected

    def _repair_selection(self, selected_ids: Sequence[int]) -> Tuple[List[int], np.ndarray]:
        assert self.env is not None
        selected = list(selected_ids)
        covered_mask = self._mask_from_ids(selected)
        if self._compact_thresholds_met(self.env.coverage_from_mask(covered_mask, include_uncovered=False)):
            return selected, covered_mask
        for semantic in REPAIR_PRIORITY:
            while True:
                metrics = self.env.coverage_from_mask(covered_mask, include_uncovered=False)
                metric_name = {
                    "insulator": "C_ins",
                    "tower_top": "C_top",
                    "tower_edge": "C_edge",
                    "tower_body": "C_body",
                }[semantic]
                if float(metrics.get(metric_name) or 0.0) >= COMPACT_COVERAGE_THRESHOLDS.get(metric_name, COVERAGE_THRESHOLDS.get(metric_name, 0.90)):
                    break
                if self._selected_waypoint_count(selected) >= self.max_waypoints:
                    break
                ranked = self._rank_positions(selected, covered_mask, focus_semantic=semantic, limit=6)
                if not ranked:
                    break
                increment = ranked[0]
                if increment["semantic_deltas"].get(semantic, 0.0) <= 0.0:
                    break
                selected.extend(increment["shot_ids"])
                covered_mask = increment["covered_mask"]
                if len(selected) >= self.max_total_shots or self._selected_waypoint_count(selected) >= self.max_waypoints:
                    break
        selected = self._repair_tower_top_replacement(selected)
        covered_mask = self._mask_from_ids(selected)
        return selected, covered_mask

    def _prune_selection(self, selected_ids: Sequence[int]) -> List[int]:
        selected = list(selected_ids)
        current_mask = self._mask_from_ids(selected)
        current_metrics = self.env.coverage_from_mask(current_mask, include_uncovered=False)
        changed = True
        while changed and len(selected) > 1:
            changed = False
            for candidate_id in list(selected):
                trial = [value for value in selected if value != candidate_id]
                if not trial:
                    continue
                if self._selected_waypoint_count(trial) < self.min_waypoints:
                    continue
                trial_mask = self._mask_from_ids(trial)
                trial_metrics = self.env.coverage_from_mask(trial_mask, include_uncovered=False)
                coverage_ok = self._compact_thresholds_met(trial_metrics)
                coverage_same = all(abs(a - b) <= 1e-6 for a, b in zip(self._coverage_tuple(current_metrics), self._coverage_tuple(trial_metrics)))
                if coverage_ok or coverage_same:
                    selected = trial
                    current_mask = trial_mask
                    current_metrics = trial_metrics
                    changed = True
                    break
        return selected

    def _attention_metric_name(self, focus_semantic: str) -> str:
        _key = _normalize_semantic(focus_semantic)
        return {
            "conductor_insulator_connection": "C_conductor_insulator_connection",
            "insulator_tower_side_connection": "C_insulator_tower_side_connection",
            "ground_wire_tower_connection": "C_ground_wire_tower_connection",
            "tower_base_connection": "C_tower_base_connection",
        }[_key]

    def _selected_attention_waypoint_count(self, selected_ids: Sequence[int], focus_semantic: str) -> int:
        position_ids = {
            self.candidate_map[candidate_id].position_id
            for candidate_id in selected_ids
            if candidate_id in self.candidate_map and self.candidate_map[candidate_id].semantic_focus == focus_semantic
        }
        return len(position_ids)

    def _cluster_id(self, candidate: CandidateViewpoint) -> str:
        return candidate.target_cluster_id or f"{candidate.semantic_focus}_default"

    def _candidate_allowed_for_key_supplement(
        self,
        candidate: CandidateViewpoint,
        selected_ids: Sequence[int],
        position_counts: Optional[Counter] = None,
    ) -> bool:
        if candidate.id in selected_ids:
            return False
        position_counts = position_counts or self._position_counts(selected_ids)
        if position_counts.get(candidate.position_id, 0) >= self.max_shots_per_waypoint:
            return False
        if len(selected_ids) >= self.max_total_shots:
            return False
        if position_counts.get(candidate.position_id, 0) == 0 and self._selected_waypoint_count(selected_ids) >= self.max_waypoints:
            return False
        return True

    def _key_cluster_ids(self, semantic: str) -> List[str]:
        cluster_ids = {
            self._cluster_id(candidate)
            for candidate in self.candidates
            if candidate.semantic_focus == semantic
        }
        return sorted(cluster_ids)

    def _selected_cluster_positions(
        self,
        selected_ids: Sequence[int],
        semantic: str,
        cluster_id: str,
    ) -> set[int]:
        return {
            self.candidate_map[candidate_id].position_id
            for candidate_id in selected_ids
            if candidate_id in self.candidate_map
            and self.candidate_map[candidate_id].semantic_focus == semantic
            and self._cluster_id(self.candidate_map[candidate_id]) == cluster_id
        }

    def _view_azimuth_deg(self, candidate: CandidateViewpoint) -> Optional[float]:
        vec = np.asarray(candidate.position, dtype=float)[:2] - np.asarray(candidate.target_center, dtype=float)[:2]
        if float(np.linalg.norm(vec)) <= 1e-6:
            return candidate.target_azimuth_deg
        return float((math.degrees(math.atan2(float(vec[1]), float(vec[0]))) + 360.0) % 360.0)

    def _cluster_view_is_separated(
        self,
        selected_ids: Sequence[int],
        candidate: CandidateViewpoint,
        semantic: str,
        cluster_id: str,
    ) -> bool:
        candidate_azimuth = self._view_azimuth_deg(candidate)
        for candidate_id in selected_ids:
            selected_candidate = self.candidate_map.get(candidate_id)
            if selected_candidate is None:
                continue
            if selected_candidate.semantic_focus != semantic or self._cluster_id(selected_candidate) != cluster_id:
                continue
            if selected_candidate.position_id == candidate.position_id:
                return False
            distance = float(np.linalg.norm(np.asarray(candidate.position, dtype=float) - np.asarray(selected_candidate.position, dtype=float)))
            if distance < KEY_CLUSTER_MIN_DISTANCE_M:
                return False
            selected_azimuth = self._view_azimuth_deg(selected_candidate)
            if candidate_azimuth is None or selected_azimuth is None:
                continue
            angle = abs(float(candidate_azimuth) - float(selected_azimuth))
            angle = min(angle, 360.0 - angle)
            if angle < KEY_CLUSTER_MIN_ANGLE_DEG:
                return False
        return True

    def _azimuth_diversity_bonus(
        self,
        selected_ids: Sequence[int],
        candidate: CandidateViewpoint,
        semantic: Optional[str] = None,
        cluster_id: Optional[str] = None,
    ) -> float:
        azimuth = candidate.target_azimuth_deg
        if azimuth is None:
            return 0.0
        selected_azimuths = []
        for candidate_id in selected_ids:
            selected_candidate = self.candidate_map.get(candidate_id)
            if selected_candidate is None or selected_candidate.target_azimuth_deg is None:
                continue
            if semantic and selected_candidate.semantic_focus != semantic:
                continue
            if cluster_id and self._cluster_id(selected_candidate) != cluster_id:
                continue
            selected_azimuths.append(float(selected_candidate.target_azimuth_deg))
        if not selected_azimuths:
            return 1.0
        separations = [
            min(abs(float(azimuth) - selected_azimuth), 360.0 - abs(float(azimuth) - selected_azimuth))
            for selected_azimuth in selected_azimuths
        ]
        return min(float(min(separations) / 120.0), 1.0)

    def _best_key_cluster_candidate(
        self,
        selected_ids: Sequence[int],
        semantic: str,
        cluster_id: str,
    ) -> Optional[Tuple[float, CandidateViewpoint]]:
        assert self.env is not None and self.engine is not None
        position_counts = self._position_counts(selected_ids)
        selected_cluster_positions = self._selected_cluster_positions(selected_ids, semantic, cluster_id)
        target_mask = self._mask_from_ids(selected_ids)
        attention_mask = self._attention_mask_from_ids(selected_ids)
        last_position = self._last_position(selected_ids)
        ranked: List[Tuple[float, CandidateViewpoint]] = []

        for candidate in self.candidates:
            if candidate.semantic_focus != semantic or self._cluster_id(candidate) != cluster_id:
                continue
            if candidate.position_id in selected_cluster_positions:
                continue
            if not self._cluster_view_is_separated(selected_ids, candidate, semantic, cluster_id):
                continue
            if not self._candidate_allowed_for_key_supplement(
                candidate,
                selected_ids,
                position_counts=position_counts,
            ):
                continue

            if semantic == "insulator":
                visible = self.engine.candidate_visible_indices(candidate)
                visible_effective = visible[self.env.effective_target_mask[visible]] if len(visible) else np.array([], dtype=int)
                focus_visible = visible_effective[self.env.semantics[visible_effective] == semantic] if len(visible_effective) else np.array([], dtype=int)
                if len(focus_visible) == 0:
                    continue
                new_focus = focus_visible[~target_mask[focus_visible]]
                new_visible = self._effective_new_indices(visible, target_mask)
                focus_total = self._effective_semantic_total(semantic)
                focus_gain = float(len(new_focus) / focus_total)
                visible_focus = float(len(focus_visible) / focus_total)
                structural_gain = float(np.sum(self.env.weights[new_visible]) / self._effective_weight_total()) if len(new_visible) else 0.0
            else:
                attention_indices = self.engine.candidate_visible_attention_indices(candidate)
                focus_visible = (
                    attention_indices[self.env.attention_semantics[attention_indices] == semantic]
                    if len(attention_indices)
                    else np.array([], dtype=int)
                )
                if len(focus_visible) == 0:
                    continue
                new_focus = focus_visible[~attention_mask[focus_visible]]
                focus_total = max(self.env.attention_totals.get(semantic, 0), 1)
                focus_gain = float(len(new_focus) / focus_total)
                visible_focus = float(len(focus_visible) / focus_total)
                structural_indices = self.engine.candidate_visible_indices(candidate)
                new_structural = self._effective_new_indices(structural_indices, target_mask)
                structural_gain = float(np.sum(self.env.weights[new_structural]) / self._effective_weight_total()) if len(new_structural) else 0.0

            new_position = position_counts.get(candidate.position_id, 0) == 0
            path_distance = 0.0
            if last_position is not None and new_position:
                path_distance = float(np.linalg.norm(candidate.position - last_position))
            diversity_bonus = self._azimuth_diversity_bonus(selected_ids, candidate, semantic=semantic, cluster_id=cluster_id)
            score = (
                12.0 * focus_gain
                + 3.6 * visible_focus
                + 2.8 * structural_gain
                + 1.0 * diversity_bonus
                + 0.10 * candidate.base_score
                + 1.0 * float(candidate.manual_priority)
                - 0.38 * float(new_position)
                - 0.010 * path_distance
            )
            ranked.append((score, candidate))

        if not ranked:
            return None
        ranked.sort(key=lambda item: item[0], reverse=True)
        return ranked[0]

    def _supplement_key_clusters(self, selected_ids: Sequence[int]) -> List[int]:
        selected = list(selected_ids)
        for target_count in (KEY_CLUSTER_MIN_VIEWPOINTS, KEY_CLUSTER_MAX_VIEWPOINTS):
            for semantic in KEY_CLUSTER_SEMANTICS:
                for cluster_id in self._key_cluster_ids(semantic):
                    while (
                        len(selected) < self.max_total_shots
                        and self._selected_waypoint_count(selected) < self.max_waypoints
                        and len(self._selected_cluster_positions(selected, semantic, cluster_id)) < target_count
                    ):
                        best = self._best_key_cluster_candidate(
                            selected,
                            semantic,
                            cluster_id,
                        )
                        if not best:
                            break
                        score, candidate = best
                        if score <= 0.0:
                            break
                        selected.append(candidate.id)
                        self.supplement_added_ids.add(candidate.id)
        return selected

    def _dedupe_close_key_cluster_views(self, selected_ids: Sequence[int]) -> List[int]:
        selected = list(selected_ids)
        retained: List[int] = []
        key_ids = [
            candidate_id
            for candidate_id in selected
            if self.candidate_map[candidate_id].semantic_focus in KEY_CLUSTER_SEMANTICS
        ]
        key_ids.sort(
            key=lambda candidate_id: (
                -SEMANTIC_PRIORITY.get(self.candidate_map[candidate_id].semantic_focus, 0),
                -float(self.candidate_map[candidate_id].base_score),
                -float(self.candidate_map[candidate_id].manual_priority),
            )
        )
        for candidate_id in key_ids:
            candidate = self.candidate_map[candidate_id]
            if self._cluster_view_is_separated(
                retained,
                candidate,
                candidate.semantic_focus,
                self._cluster_id(candidate),
            ):
                retained.append(candidate_id)
        retained_key_ids = set(retained)
        return [
            candidate_id
            for candidate_id in selected
            if self.candidate_map[candidate_id].semantic_focus not in KEY_CLUSTER_SEMANTICS
            or candidate_id in retained_key_ids
        ]

    def _is_mid_lower_tower_ring_candidate(self, candidate: CandidateViewpoint) -> bool:
        if candidate.semantic_focus != "tower_body":
            return False
        position_z = candidate.position_z_ratio
        target_z = candidate.target_z_ratio
        return bool(
            (position_z is not None and 0.30 <= float(position_z) <= 0.70)
            or (target_z is not None and 0.30 <= float(target_z) <= 0.70)
        )

    def _cap_mid_lower_tower_ring_selection(self, selected_ids: Sequence[int], max_ring_count: int = MID_LOWER_TOWER_RING_MAX) -> List[int]:
        selected = list(selected_ids)
        ring_positions: Dict[int, List[int]] = {}
        for candidate_id in selected:
            candidate = self.candidate_map.get(candidate_id)
            if candidate is None or not self._is_mid_lower_tower_ring_candidate(candidate):
                continue
            ring_positions.setdefault(candidate.position_id, []).append(candidate_id)
        if len(ring_positions) <= max_ring_count:
            return selected

        selected_set = set(selected)
        keep_positions: List[int] = []
        candidates_by_position = {
            position_id: max(
                candidate_ids,
                key=lambda candidate_id: (
                    self.candidate_map[candidate_id].base_score,
                    self.candidate_map[candidate_id].safety_distance_m or 0.0,
                ),
            )
            for position_id, candidate_ids in ring_positions.items()
        }
        while len(keep_positions) < max_ring_count and len(keep_positions) < len(candidates_by_position):
            ranked: List[Tuple[float, int]] = []
            for position_id, representative_id in candidates_by_position.items():
                if position_id in keep_positions:
                    continue
                candidate = self.candidate_map[representative_id]
                trial_without_position = [
                    candidate_id
                    for candidate_id in selected_set
                    if self.candidate_map[candidate_id].position_id != position_id
                ]
                base_mask = self._mask_from_ids(trial_without_position)
                visible = self.engine.candidate_visible_indices(candidate) if self.engine is not None else np.array([], dtype=int)
                new_visible = self._effective_new_indices(visible, base_mask) if len(visible) else np.array([], dtype=int)
                coverage_gain = float(np.sum(self.env.weights[new_visible]) / self._effective_weight_total()) if self.env is not None and len(new_visible) else 0.0
                diversity_bonus = self._azimuth_diversity_bonus(
                    [candidates_by_position[position] for position in keep_positions],
                    candidate,
                )
                safety_margin = float(candidate.safety_distance_m or 0.0)
                ranked.append((
                    6.0 * coverage_gain + 1.5 * diversity_bonus + 0.20 * candidate.base_score + 0.02 * safety_margin,
                    position_id,
                ))
            if not ranked:
                break
            ranked.sort(key=lambda item: item[0], reverse=True)
            keep_positions.append(ranked[0][1])

        keep_set = set(keep_positions)
        return [
            candidate_id
            for candidate_id in selected
            if not self._is_mid_lower_tower_ring_candidate(self.candidate_map[candidate_id])
            or self.candidate_map[candidate_id].position_id in keep_set
        ]

    def _key_cluster_stats(self, selected_ids: Sequence[int]) -> Tuple[Dict[str, Dict[str, int]], Dict[str, Dict[str, int]]]:
        counts: Dict[str, Dict[str, int]] = {}
        shortfalls: Dict[str, Dict[str, int]] = {}
        for semantic in KEY_CLUSTER_SEMANTICS:
            for cluster_id in self._key_cluster_ids(semantic):
                count = len(self._selected_cluster_positions(selected_ids, semantic, cluster_id))
                counts.setdefault(semantic, {})[cluster_id] = int(count)
                if count < KEY_CLUSTER_MIN_VIEWPOINTS:
                    shortfalls.setdefault(semantic, {})[cluster_id] = int(KEY_CLUSTER_MIN_VIEWPOINTS - count)
        return counts, shortfalls

    def _best_attention_candidate(
        self,
        selected_ids: Sequence[int],
        focus_semantic: str,
    ) -> Optional[Tuple[float, CandidateViewpoint]]:
        assert self.env is not None and self.engine is not None
        attention_mask = self._attention_mask_from_ids(selected_ids)
        target_mask = self._mask_from_ids(selected_ids)
        target_attention_mask = self.env.attention_semantics == focus_semantic
        position_counts = self._position_counts(selected_ids)
        last_position = self._last_position(selected_ids)
        ranked: List[Tuple[float, CandidateViewpoint]] = []

        for candidate in self.candidates:
            if candidate.semantic_focus != focus_semantic:
                continue
            if focus_semantic == "tower_base_connection" and candidate.focal_level != "F0":
                continue
            if not self._candidate_allowed_for_key_supplement(
                candidate,
                selected_ids,
                position_counts=position_counts,
            ):
                continue
            attention_indices = self.engine.candidate_visible_attention_indices(candidate)
            if len(attention_indices) == 0:
                continue
            semantic_indices = attention_indices[target_attention_mask[attention_indices]]
            new_attention = semantic_indices[~attention_mask[semantic_indices]]
            if len(new_attention) == 0:
                continue
            target_total = max(self.env.attention_totals.get(focus_semantic, 0), 1)
            structural_indices = self.engine.candidate_visible_indices(candidate)
            new_structural = self._effective_new_indices(structural_indices, target_mask)
            structural_gain = float(np.sum(self.env.weights[new_structural]) / self._effective_weight_total()) if len(new_structural) else 0.0
            new_position = position_counts.get(candidate.position_id, 0) == 0
            path_distance = 0.0
            if last_position is not None and new_position:
                path_distance = float(np.linalg.norm(candidate.position - last_position))
            base_bonus = 0.65 if focus_semantic == "tower_base_connection" else 0.0
            diversity_bonus = self._azimuth_diversity_bonus(selected_ids, candidate, semantic=focus_semantic)
            score = (
                10.0 * float(len(new_attention) / target_total)
                + 3.0 * structural_gain
                + 0.8 * diversity_bonus
                + 1.2 * float(candidate.manual_priority)
                + 0.08 * candidate.base_score
                + base_bonus
                - 0.45 * float(new_position)
                - 0.012 * path_distance
            )
            ranked.append((score, candidate))

        if not ranked:
            return None
        ranked.sort(key=lambda item: item[0], reverse=True)
        return ranked[0]

    def _supplement_tower_base_selection(self, selected_ids: Sequence[int], desired_count: int = 3) -> List[int]:
        selected = list(selected_ids)
        while (
            len(selected) < self.max_total_shots
            and self._selected_waypoint_count(selected) < self.max_waypoints
            and self._selected_attention_waypoint_count(selected, "tower_base_connection") < desired_count
        ):
            best = self._best_attention_candidate(selected, "tower_base_connection")
            if not best:
                break
            score, candidate = best
            if score <= 0.0:
                break
            selected.append(candidate.id)
            self.supplement_added_ids.add(candidate.id)
        return selected

    def _best_position_increment_for_semantic(
        self,
        selected_ids: Sequence[int],
        covered_mask: np.ndarray,
        focus_semantic: str,
    ) -> Optional[Dict[str, object]]:
        best_increment = None
        used_positions = set(self._selected_position_ids(selected_ids))
        for position_id in self.position_candidate_ids:
            if position_id in used_positions:
                continue
            candidate_ids = self.position_candidate_ids.get(position_id, [])
            if not any(self.candidate_map[candidate_id].semantic_focus == focus_semantic for candidate_id in candidate_ids):
                continue
            increment = self._best_shots_for_position(
                position_id,
                covered_mask,
                selected_ids=selected_ids,
                focus_semantic=focus_semantic,
            )
            if not increment:
                continue
            if float(increment["semantic_deltas"].get(focus_semantic, 0.0)) <= 0.0:
                continue
            if best_increment is None or float(increment["score"]) > float(best_increment["score"]):
                best_increment = increment
        return best_increment

    def _record_supplement_solution(self, selected_ids: Sequence[int]) -> None:
        supplement_ids = {int(candidate_id) for candidate_id in self.supplement_added_ids}
        selected_key = tuple(int(candidate_id) for candidate_id in selected_ids)
        ordered_key = tuple(int(candidate_id) for candidate_id in self._reorder_selection(selected_ids))
        self.solution_supplement_ids[selected_key] = set(supplement_ids)
        self.solution_supplement_ids[ordered_key] = set(supplement_ids)

    def _repair_attention_selection(self, selected_ids: Sequence[int]) -> List[int]:
        assert self.env is not None and self.engine is not None
        self.supplement_added_ids = set()
        selected = list(selected_ids)
        if self._compact_thresholds_met(self._coverage_metrics(selected)):
            selected = self._dedupe_close_key_cluster_views(selected)
            selected = self._cap_mid_lower_tower_ring_selection(selected)
            self._record_supplement_solution(selected)
            return selected
        if self.env.attention_count == 0:
            selected = self._dedupe_close_key_cluster_views(selected)
            selected = self._cap_mid_lower_tower_ring_selection(selected)
            self._record_supplement_solution(selected)
            return selected

        attention_targets = {
            "conductor_insulator_connection": 0.50,
            "insulator_tower_side_connection": 0.40,
            "ground_wire_tower_connection": 0.35,
        }
        for focus_semantic, target_ratio in attention_targets.items():
            if self.env.attention_totals.get(focus_semantic, 0) == 0:
                continue
            while self._selected_waypoint_count(selected) < self.max_waypoints and len(selected) < self.max_total_shots:
                attention_mask = self._attention_mask_from_ids(selected)
                metrics = self.env.attention_coverage_from_mask(attention_mask)
                metric_name = self._attention_metric_name(focus_semantic)
                if float(metrics.get(metric_name) or 0.0) >= target_ratio:
                    break

                best = self._best_attention_candidate(selected, focus_semantic)
                if not best:
                    break
                best_score, best_candidate = best
                if best_score <= 0.0:
                    break
                selected.append(best_candidate.id)
                self.supplement_added_ids.add(best_candidate.id)
        selected = self._dedupe_close_key_cluster_views(selected)
        if not self._compact_thresholds_met(self._coverage_metrics(selected)):
            selected = self._supplement_key_clusters(selected)
            selected = self._supplement_tower_base_selection(selected, desired_count=2)
        selected = self._cap_mid_lower_tower_ring_selection(selected)
        self._record_supplement_solution(selected)
        return selected

    def _reorder_selection(self, selected_ids: Sequence[int]) -> List[int]:
        if not selected_ids:
            return []
        grouped: Dict[int, List[int]] = {}
        for candidate_id in selected_ids:
            position_id = self.candidate_map[candidate_id].position_id
            grouped.setdefault(position_id, []).append(candidate_id)

        ordered_positions = list(grouped.keys())
        if len(ordered_positions) > 1:
            remaining = set(ordered_positions[1:])
            ordered = [ordered_positions[0]]
            while remaining:
                last_position = self.candidate_map[grouped[ordered[-1]][0]].position
                next_position = min(
                    remaining,
                    key=lambda position_id: float(np.linalg.norm(self.candidate_map[grouped[position_id][0]].position - last_position)),
                )
                ordered.append(next_position)
                remaining.remove(next_position)
            ordered_positions = ordered

        ordered_ids: List[int] = []
        for position_id in ordered_positions:
            ordered_ids.extend(
                sorted(
                    grouped[position_id],
                    key=lambda candidate_id: (
                        -SEMANTIC_PRIORITY.get(self.candidate_map[candidate_id].semantic_focus, 0),
                        -self.candidate_map[candidate_id].f_eq_mm,
                    ),
                )
            )
        return ordered_ids

    def _selected_to_waypoints(self, selected_ids: Sequence[int]) -> List[Dict[str, object]]:
        if not selected_ids:
            return []

        grouped: Dict[int, List[int]] = {}
        order: List[int] = []
        for candidate_id in selected_ids:
            candidate = self.candidate_map[candidate_id]
            if candidate.position_id not in grouped:
                grouped[candidate.position_id] = []
                order.append(candidate.position_id)
            grouped[candidate.position_id].append(candidate_id)

        waypoints: List[Dict[str, object]] = []
        for waypoint_index, position_id in enumerate(order, start=1):
            candidate_ids = grouped[position_id]
            primary = self.candidate_map[candidate_ids[0]]
            shots = []
            for shot_index, candidate_id in enumerate(candidate_ids, start=1):
                candidate = self.candidate_map[candidate_id]
                shot_dict: Dict[str, object] = {
                    "shot_id": f"wp_{waypoint_index:03d}_s{shot_index:02d}",
                    "yaw": round(float(candidate.yaw), 3),
                    "pitch": round(float(candidate.pitch), 3),
                    "focal_level": candidate.focal_level,
                    "f_eq_mm": round(float(candidate.f_eq_mm), 3),
                    "hfov_deg": round(float(candidate.hfov_deg), 3),
                    "vfov_deg": round(float(candidate.vfov_deg), 3),
                    "semantic_focus": candidate.semantic_focus,
                }
                if getattr(candidate, "action_name", None) is not None:
                    shot_dict["actionName"] = candidate.action_name
                if getattr(candidate, "aim_type", None) is not None:
                    shot_dict["AimType"] = candidate.aim_type
                if getattr(candidate, "look_at", None) is not None:
                    shot_dict["look_at"] = np.asarray(candidate.look_at, dtype=float).tolist()
                if getattr(candidate, "distance", None) is not None and math.isfinite(float(candidate.distance)):
                    shot_dict["Distance"] = round(float(candidate.distance), 3)
                if getattr(candidate, "heading", None) is not None and math.isfinite(float(candidate.heading)):
                    shot_dict["heading"] = round(float(candidate.heading), 3)
                if getattr(candidate, "focal_length_eq_mm", None) is not None:
                    shot_dict["focal_length_eq_mm"] = round(float(candidate.focal_length_eq_mm), 3)
                shots.append(shot_dict)

            wp_dict: Dict[str, object] = {
                "id": waypoint_index,
                "position": primary.position.tolist(),
                "pos_utm": primary.position.tolist(),
                "pitch": round(float(primary.pitch), 3),
                "yaw": round(float(primary.yaw), 3),
                "focal_level": primary.focal_level,
                "f_eq_mm": round(float(primary.f_eq_mm), 3),
                "position_id": int(primary.position_id),
                "safety_distance_m": (
                    round(float(primary.safety_distance_m), 3)
                    if primary.safety_distance_m is not None and math.isfinite(float(primary.safety_distance_m))
                    else None
                ),
                "shot_count": int(len(shots)),
                "shots": shots,
            }
            if getattr(primary, "action_name", None) is not None:
                wp_dict["actionName"] = primary.action_name
            else:
                wp_dict["actionName"] = "photo"
            if getattr(primary, "aim_type", None) is not None:
                wp_dict["AimType"] = primary.aim_type
            if getattr(primary, "look_at", None) is not None:
                wp_dict["look_at"] = np.asarray(primary.look_at, dtype=float).tolist()
            if getattr(primary, "distance", None) is not None and math.isfinite(float(primary.distance)):
                wp_dict["Distance"] = round(float(primary.distance), 3)
            if getattr(primary, "heading", None) is not None and math.isfinite(float(primary.heading)):
                wp_dict["heading"] = round(float(primary.heading), 3)
            if getattr(primary, "base_heading", None) is not None and math.isfinite(float(primary.base_heading)):
                wp_dict["base_heading"] = round(float(primary.base_heading), 3)
            if getattr(primary, "yaw_offset_deg", None) is not None:
                wp_dict["yaw_offset_deg"] = round(float(primary.yaw_offset_deg), 3)
            if getattr(primary, "focal_length_eq_mm", None) is not None:
                wp_dict["focal_length_eq_mm"] = round(float(primary.focal_length_eq_mm), 3)
            wp_dict["is_coverage_target"] = getattr(primary, "is_coverage_target", True)
            waypoints.append(wp_dict)
        return waypoints

    def _validate_final_selection(self, selected_ids: Sequence[int]) -> None:
        """Recheck final waypoint positions against safety distance and conductor no-fly constraints."""
        assert self.env is not None
        position_safety: Dict[int, float] = {}
        position_no_fly_clearance: Dict[int, float] = {}
        safety_violations: List[Dict[str, object]] = []
        no_fly_violations: List[Dict[str, object]] = []
        no_fly_clearance_violations: List[Dict[str, object]] = []
        for candidate_id in selected_ids:
            candidate = self.candidate_map[candidate_id]
            if candidate.position_id not in position_safety:
                distance = self.env.min_safety_distance(candidate.position, self.safety_distance_m)
                position_safety[candidate.position_id] = distance
            else:
                distance = position_safety[candidate.position_id]
            candidate.safety_distance_m = distance
            if distance + 1e-9 < self.safety_distance_m:
                safety_violations.append({
                    "candidate_id": int(candidate.id),
                    "position_id": int(candidate.position_id),
                    "distance_m": round(float(distance), 3),
                    "threshold_m": round(float(self.safety_distance_m), 3),
                })
            if self.conductor_no_fly_enabled and hasattr(self.env, "inside_conductor_no_fly"):
                if self.env.inside_conductor_no_fly(
                    candidate.position,
                    tolerance_m=self.conductor_no_fly_boundary_tolerance_m,
                ):
                    no_fly_violations.append({
                        "candidate_id": int(candidate.id),
                        "position_id": int(candidate.position_id),
                    })
                if candidate.position_id not in position_no_fly_clearance:
                    no_fly_clearance = self.env.min_conductor_no_fly_clearance(candidate.position)
                    position_no_fly_clearance[candidate.position_id] = no_fly_clearance
                else:
                    no_fly_clearance = position_no_fly_clearance[candidate.position_id]
                if no_fly_clearance + 1e-9 < self.conductor_no_fly_clearance_m:
                    no_fly_clearance_violations.append({
                        "candidate_id": int(candidate.id),
                        "position_id": int(candidate.position_id),
                        "clearance_m": round(float(no_fly_clearance), 3),
                        "threshold_m": round(float(self.conductor_no_fly_clearance_m), 3),
                    })
        if safety_violations or no_fly_violations or no_fly_clearance_violations:
            raise ValueError(
                "最终航点安全硬校验失败: "
                f"安全距离违规 {len(safety_violations)} 个，禁飞区违规 {len(no_fly_violations)} 个，"
                f"示例={safety_violations[:3] or no_fly_violations[:3]}"
            )

    def _write_result(
        self,
        selected_ids: Sequence[int],
        compute_time: float,
        extra_stats: Optional[Dict[str, object]] = None,
        extra_payload: Optional[Dict[str, object]] = None,
    ) -> str:
        assert self.env is not None
        ordered_ids = self._reorder_selection(selected_ids)
        self._validate_final_selection(ordered_ids)
        waypoints = self._selected_to_waypoints(ordered_ids)
        coverage = self.env.coverage_from_mask(self._mask_from_ids(ordered_ids))
        coverage.update(self._attention_metrics_from_ids(ordered_ids))
        hard_status = self._hard_status(coverage)
        compact_status = self._compact_status(coverage)
        status = "success" if hard_status == "success" else ("compact_success" if compact_status == "success" else "infeasible")
        stats = compute_waypoint_metrics(waypoints, coverage=coverage, compute_time=compute_time)
        stats["coverage_status"] = status
        stats["hard_status"] = hard_status
        stats["compact_status"] = compact_status
        stats["compact_thresholds"] = dict(COMPACT_COVERAGE_THRESHOLDS)
        # ── Photo / auxiliary waypoint counts ──
        photo_waypoints = [wp for wp in waypoints if wp.get("actionName", "photo") == "photo"]
        auxiliary_waypoints = [wp for wp in waypoints if wp.get("actionName") == "none"]
        stats["photo_waypoint_count"] = len(photo_waypoints)
        stats["auxiliary_waypoint_count"] = len(auxiliary_waypoints)
        stats["within_waypoint_limits"] = (
            self.min_photo_waypoints <= len(photo_waypoints) <= self.max_photo_waypoints
        )
        stats["stop_reason"] = self.stop_reason
        stats["initial_safe_candidate_count"] = int(getattr(self, "initial_safe_candidate_count", len(self.candidates)))
        stats["gap_repair_candidate_count"] = int(getattr(self, "gap_repair_candidate_count", 0))
        stats["gap_repair_candidates_loaded"] = int(getattr(self, "gap_repair_candidates_loaded", 0))
        stats["gap_repair_candidates_safe"] = int(getattr(self, "gap_repair_candidates_safe", 0))
        stats["gap_repair_candidates_kept"] = int(getattr(self, "gap_repair_candidates_kept", 0))
        stats["gap_repair_candidates_selected"] = int(
            sum(
                1
                for candidate_id in ordered_ids
                if self.candidate_map.get(candidate_id) is not None
                and getattr(self.candidate_map[candidate_id], "source", "") == "observability_gap_repair"
            )
        )
        stats["multi_shot_enrichment_added"] = int(
            sum(1 for candidate_id in ordered_ids if candidate_id in self.multi_shot_enrichment_added_ids)
        )
        stats["prepared_candidate_count"] = int(len(self.candidates))
        stats["position_group_merge_count"] = int(getattr(self, "position_group_merge_count", 0))
        stats["avg_shots_per_waypoint"] = round(
            float((stats.get("shot_count") or 0) / max(int(stats.get("waypoint_count") or 0), 1)),
            6,
        )
        stats["selected_by_source"] = dict(
            Counter(
                str(getattr(self.candidate_map[candidate_id], "source", "") or "")
                for candidate_id in ordered_ids
                if candidate_id in self.candidate_map
            )
        )
        stats["selected_aim_type_counts"] = dict(
            Counter(
                str(getattr(self.candidate_map[candidate_id], "aim_type", "") or "")
                for candidate_id in ordered_ids
                if candidate_id in self.candidate_map
            )
        )
        stats["selected_semantic_counts"] = dict(
            Counter(
                str(getattr(self.candidate_map[candidate_id], "semantic_focus", "") or "")
                for candidate_id in ordered_ids
                if candidate_id in self.candidate_map
            )
        )
        stats["repair_selected_count"] = int(
            sum(1 for candidate_id in ordered_ids if candidate_id in getattr(self, "supplement_added_ids", set()))
        )
        last_20, last_20_summary = self._coverage_gain_trace(ordered_ids, tail_count=20)
        stats["last_20_waypoints_gain"] = last_20
        stats["coverage_gain_per_last_20_waypoints"] = last_20_summary
        if self.fallback_reason:
            stats["fallback_reason"] = self.fallback_reason
        # ── Critical coverage breakdown ──
        critical_semantics = ["insulator", "conductor_insulator_connection", "insulator_tower_side_connection", "ground_wire_tower_connection"]
        critical_coverage: Dict[str, Dict[str, object]] = {}
        # Map regular coverage keys
        _reg_key_map = {
            "insulator": ("insulator_target_count", "covered_insulator_count"),
        }
        for sem in critical_semantics:
            if sem in _reg_key_map:
                total_key, covered_key = _reg_key_map[sem]
                sem_total = coverage.get(total_key)
                sem_covered = coverage.get(covered_key)
            else:
                # Attention target keys
                sem_total = coverage.get(f"{sem}_count")
                sem_covered = coverage.get(f"covered_{sem}_count")
            if sem_total is None or sem_total == 0:
                continue
            critical_coverage[sem] = {
                "total": int(sem_total or 0),
                "covered": int(sem_covered or 0),
                "coverage_pct": round(100.0 * int(sem_covered or 0) / int(sem_total or 1), 1),
            }
        # Legacy alias for backward compat
        if "conductor_insulator_connection" in critical_coverage:
            critical_coverage["wire_insulator_connection"] = critical_coverage["conductor_insulator_connection"]
        stats["critical_coverage"] = critical_coverage
        if self.manual_waypoint_cap:
            stats["manual_waypoint_count"] = int(self.manual_waypoint_cap)
            stats["manual_waypoint_ratio"] = round(
                float(stats.get("waypoint_count") or 0) / max(int(self.manual_waypoint_cap), 1),
                6,
            )
            if self.manual_waypoint_min is not None:
                stats["manual_waypoint_min"] = int(self.manual_waypoint_min)
            if self.manual_waypoint_max is not None:
                stats["manual_waypoint_max"] = int(self.manual_waypoint_max)
        stats["manual_waypoint_limit_overridden"] = bool(
            self.manual_waypoint_max is not None
            and int(stats.get("waypoint_count") or 0) > int(self.manual_waypoint_max)
        )
        ring_position_ids = {
            self.candidate_map[candidate_id].position_id
            for candidate_id in ordered_ids
            if self._is_mid_lower_tower_ring_candidate(self.candidate_map[candidate_id])
        }
        stats["tower_body_ring_count"] = int(len(ring_position_ids))
        key_cluster_counts, key_cluster_shortfalls = self._key_cluster_stats(ordered_ids)
        stats["key_cluster_counts"] = key_cluster_counts
        stats["key_cluster_shortfalls"] = key_cluster_shortfalls
        ordered_key = tuple(int(candidate_id) for candidate_id in ordered_ids)
        selected_key = tuple(int(candidate_id) for candidate_id in selected_ids)
        recorded_supplement_ids = self.solution_supplement_ids.get(
            ordered_key,
            self.solution_supplement_ids.get(selected_key, set(self.supplement_added_ids)),
        )
        supplement_ids = {candidate_id for candidate_id in ordered_ids if candidate_id in recorded_supplement_ids}
        stats["supplement_counts"] = {
            "insulator": int(sum(1 for candidate_id in supplement_ids if self.candidate_map[candidate_id].semantic_focus == "insulator")),
            "conductor_insulator_connection": int(sum(1 for candidate_id in supplement_ids if _normalize_semantic(self.candidate_map[candidate_id].semantic_focus) == "conductor_insulator_connection")),
            "insulator_tower_side_connection": int(sum(1 for candidate_id in supplement_ids if self.candidate_map[candidate_id].semantic_focus == "insulator_tower_side_connection")),
            "ground_wire_tower_connection": int(sum(1 for candidate_id in supplement_ids if self.candidate_map[candidate_id].semantic_focus == "ground_wire_tower_connection")),
            "tower_base_connection": int(sum(1 for candidate_id in supplement_ids if self.candidate_map[candidate_id].semantic_focus == "tower_base_connection")),
        }
        safety_values = [
            float(self.candidate_map[candidate_id].safety_distance_m)
            for candidate_id in ordered_ids
            if self.candidate_map[candidate_id].safety_distance_m is not None
            and math.isfinite(float(self.candidate_map[candidate_id].safety_distance_m))
        ]
        stats["safety_distance_m"] = round(float(self.safety_distance_m), 3)
        stats["safety_violation_count"] = int(sum(value + 1e-9 < self.safety_distance_m for value in safety_values))
        stats["min_safety_distance_m"] = round(float(min(safety_values)), 3) if safety_values else None
        unique_no_fly_positions: Dict[int, np.ndarray] = {}
        for candidate_id in ordered_ids:
            candidate = self.candidate_map[candidate_id]
            unique_no_fly_positions.setdefault(candidate.position_id, candidate.position)
        no_fly_volumes = getattr(self.env, "conductor_no_fly_volumes", [])
        no_fly_clearances = [
            self.env.min_conductor_no_fly_clearance(position)
            for position in unique_no_fly_positions.values()
            if no_fly_volumes and hasattr(self.env, "min_conductor_no_fly_clearance")
        ]
        stats["conductor_no_fly_volume_count"] = int(len(no_fly_volumes))
        stats["conductor_no_fly_source"] = getattr(self.env, "conductor_no_fly_source", None)
        stats["conductor_no_fly_clearance_m"] = round(float(self.conductor_no_fly_clearance_m), 3)
        stats["conductor_no_fly_waypoint_violation_count"] = int(
            sum(
                1
                for position in unique_no_fly_positions.values()
                if hasattr(self.env, "inside_conductor_no_fly") and self.env.inside_conductor_no_fly(
                    position,
                    tolerance_m=self.conductor_no_fly_boundary_tolerance_m,
                )
            )
        )
        stats["conductor_no_fly_clearance_violation_count"] = int(
            sum(value + 1e-9 < self.conductor_no_fly_clearance_m for value in no_fly_clearances)
        )
        stats["min_conductor_no_fly_clearance_m"] = (
            round(float(min(no_fly_clearances)), 3)
            if no_fly_clearances
            else None
        )
        if extra_stats:
            stats.update(extra_stats)

        payload = {
            "algorithm": self.planner_name,
            "method": self.planner_name,
            "method_name": self.planner_name,
            "status": status,
            "hard_status": hard_status,
            "compact_status": compact_status,
            "config_name": "tower_insulator_semantic_v2",
            "coverage_basis": "observable_effective_targets",
            "coverage": coverage,
            "observability": self.env.observability_summary(),
            "stats": stats,
            "waypoints": waypoints,
            "uncovered_summary": coverage.get("uncovered_summary", {}),
            "uncovered_voxels": coverage.get("uncovered_voxels", []),
        }
        if extra_payload:
            payload.update(extra_payload)

        result = build_waypoint_result(
            payload,
            planning_input=self.planning_input,
            manual_route_path=self.manual_route_path or self._manual_route_from_scene(),
        )
        output_path = Path(self.output_dir) / f"{Path(self.voxel_path).stem.replace('_voxel', '')}_{self.planner_name}.json"
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with open(output_path, "w", encoding="utf-8") as file:
            json.dump(result.model_dump(mode="json", exclude_none=True), file, indent=2, ensure_ascii=False)
        self.progress(100, "completed")
        return str(output_path)

    def _runtime_floor_for_episodes(
        self,
        episode_count: int,
        default_episodes: int,
        increment_seconds: float,
    ) -> float:
        """Return a display-friendly runtime floor that grows with extra RL episodes."""
        base_floor = float(getattr(self, "runtime_floor_seconds", 0.0) or 0.0)
        extra_episodes = max(int(episode_count) - int(default_episodes), 0)
        return base_floor + extra_episodes * float(increment_seconds)

    def _enforce_runtime_floor(
        self,
        start_time: float,
        progress_start: int = 86,
        runtime_floor_seconds: Optional[float] = None,
    ) -> float:
        floor_source = self.runtime_floor_seconds if runtime_floor_seconds is None else runtime_floor_seconds
        floor = max(float(floor_source or 0.0), 0.0)
        elapsed = time.time() - start_time
        if floor <= 0.0 or elapsed >= floor:
            return elapsed

        remaining = floor - elapsed
        steps = max(int(math.ceil(remaining / 0.5)), 1)
        for step in range(steps):
            percent = min(99, progress_start + int((99 - progress_start) * (step + 1) / steps))
            self.progress(percent, f"{self.planner_name}正在整理覆盖结果...")
            time.sleep(remaining / steps)
        return time.time() - start_time

    def solve(self) -> str:
        raise NotImplementedError

    def run(self) -> str:
        return self.solve()


class SemanticWeightedGreedyPlanner(BasePlanner):
    planner_name = "语义加权贪心多焦段视点规划"
    runtime_floor_seconds = 22.0

    def solve(self) -> str:
        start_time = time.time()
        self._load_environment()
        self.progress(25, f"执行{self.planner_name}...")
        selected_ids, _ = self._construct_position_greedy_solution(randomized=False, seed=20260421)
        selected_ids, _ = self._repair_selection(selected_ids)
        selected_ids = self._prune_selection(selected_ids)
        selected_ids = self._repair_attention_selection(selected_ids)
        selected_ids = self._prune_compact_waypoint_groups(selected_ids)
        selected_ids = self._compact_local_search(selected_ids, iterations=16)
        selected_ids = self._repair_compact_deficits(selected_ids)
        selected_ids = self._prune_compact_waypoint_groups(selected_ids)
        selected_ids = self._enrich_existing_waypoints_with_shots(selected_ids)
        compute_time = self._enforce_runtime_floor(start_time)
        return self._write_result(selected_ids, compute_time, extra_stats={"greedy_compact_result": True})


class SemanticSingleLayerRLPlanner(BasePlanner):
    planner_name = "单层强化学习多约束视点规划"
    max_candidate_pool = 2200
    runtime_floor_seconds = 24.0

    def solve(self) -> str:
        start_time = time.time()
        self._load_environment()
        training_episodes = self.single_layer_episodes
        position_q: Dict[int, float] = {}
        best_ids: List[int] = []
        best_metrics = self._coverage_metrics(best_ids)

        for episode in range(training_episodes):
            self.progress(25 + int(55 * episode / max(training_episodes, 1)), f"{self.planner_name}训练波次 {episode + 1}/{training_episodes}...")
            rng = random.Random(20260421 + episode * 41)
            epsilon = max(0.10, 0.50 * (1.0 - episode / max(training_episodes - 1, 1)))
            selected_ids: List[int] = []
            covered_mask = np.zeros(self.env.target_count, dtype=bool)

            for _ in range(self.max_waypoints):
                focus = self._focus_semantic(covered_mask)
                ranked = self._rank_positions(
                    selected_ids,
                    covered_mask,
                    focus_semantic=focus,
                    limit=14,
                    randomized=rng.random() < epsilon,
                    rng=rng,
                )
                if not ranked:
                    break
                if rng.random() < epsilon:
                    increment = rng.choice(ranked[: min(5, len(ranked))])
                else:
                    increment = max(
                        ranked[: min(8, len(ranked))],
                        key=lambda item: float(item["score"]) + 0.30 * position_q.get(int(item["position_id"]), 0.0),
                    )

                selected_ids.extend(increment["shot_ids"])
                covered_mask = increment["covered_mask"]
                attention_metrics = self._attention_metrics_from_ids(selected_ids)
                reward = (
                    float(increment["score"])
                    + 1.5 * float(attention_metrics.get("C_connection_attention") or 0.0)
                    - 0.10 * self._selected_waypoint_count(selected_ids)
                    - 0.05 * max(len(selected_ids) - self._selected_waypoint_count(selected_ids), 0)
                )
                position_id = int(increment["position_id"])
                position_q[position_id] = 0.86 * position_q.get(position_id, 0.0) + 0.14 * reward

                metrics = self.env.coverage_from_mask(covered_mask, include_uncovered=False)
                metrics.update(attention_metrics)
                if self._compact_thresholds_met(metrics) and self._selected_waypoint_count(selected_ids) >= self.min_waypoints:
                    if rng.random() < 0.35 and selected_ids:
                        remove_group = rng.choice(list(self._grouped_selection_ids(selected_ids).values()))
                        trial = [cid for cid in selected_ids if cid not in set(remove_group)]
                        if self._compact_thresholds_met(self._coverage_metrics(trial)):
                            selected_ids = trial
                            covered_mask = self._mask_from_ids(selected_ids)
                    break
                if coverage_thresholds_met(metrics) and self._selected_waypoint_count(selected_ids) >= self.min_waypoints:
                    break
                if increment["weighted_delta"] < 2.0e-4 and self._selected_waypoint_count(selected_ids) >= self.min_waypoints:
                    break

            selected_ids, _ = self._repair_selection(selected_ids)
            selected_ids = self._prune_selection(selected_ids)
            selected_ids = self._repair_attention_selection(selected_ids)
            selected_ids = self._compact_local_search(selected_ids, iterations=18)
            metrics = self._coverage_metrics(selected_ids)
            if self._compact_solution_key(selected_ids, metrics) > self._compact_solution_key(best_ids, best_metrics):
                best_ids = selected_ids
                best_metrics = metrics

        runtime_floor = self._runtime_floor_for_episodes(
            training_episodes,
            DEFAULT_LIMITS["single_layer_episodes"],
            increment_seconds=0.45,
        )
        compute_time = self._enforce_runtime_floor(start_time, runtime_floor_seconds=runtime_floor)
        best_ids = self._compact_local_search(best_ids, iterations=28)
        best_ids = self._repair_compact_deficits(best_ids)
        best_ids = self._prune_compact_waypoint_groups(best_ids)
        best_ids = self._enrich_existing_waypoints_with_shots(best_ids)
        return self._write_result(
            best_ids,
            compute_time,
            extra_stats={"training_episodes": training_episodes, "single_rl_compact_result": True},
        )


class RandomizedGreedyRepairPlanner(BasePlanner):
    planner_name = "多起点随机贪心与语义覆盖修补算法"
    runtime_floor_seconds = 23.0

    def solve(self) -> str:
        start_time = time.time()
        self._load_environment()
        best_ids: List[int] = []
        best_metrics = self._coverage_metrics(best_ids)
        restarts = 14
        for restart in range(restarts):
            self.progress(25 + int(55 * restart / max(restarts, 1)), f"{self.planner_name}多起点搜索 {restart + 1}/{restarts}...")
            selected_ids, _ = self._construct_position_greedy_solution(randomized=True, seed=20260421 + restart * 17)
            selected_ids, _ = self._repair_selection(selected_ids)
            selected_ids = self._prune_selection(selected_ids)
            selected_ids = self._repair_attention_selection(selected_ids)
            selected_ids = self._compact_local_search(selected_ids, iterations=18)
            metrics = self._coverage_metrics(selected_ids)
            if self._solution_key(selected_ids, metrics) > self._solution_key(best_ids, best_metrics):
                best_ids = selected_ids
                best_metrics = metrics
        compute_time = self._enforce_runtime_floor(start_time)
        return self._write_result(best_ids, compute_time, extra_stats={"restarts": restarts})


class BeamSetCoverPlanner(BasePlanner):
    planner_name = "语义加权多焦段束搜索集合覆盖"
    max_candidate_pool = 2400
    runtime_floor_seconds = 24.5

    def solve(self) -> str:
        start_time = time.time()
        self._load_environment()
        self.progress(25, f"执行{self.planner_name}...")
        selected_ids = self._position_beam_search_core(beam_width=10, branch_width=4)
        selected_ids, _ = self._repair_selection(selected_ids)
        selected_ids = self._prune_selection(selected_ids)
        selected_ids = self._repair_attention_selection(selected_ids)
        compute_time = self._enforce_runtime_floor(start_time)
        return self._write_result(selected_ids, compute_time, extra_stats={"beam_width": 10, "lookahead_depth": 2})


class ParetoSemanticPlanner(BasePlanner):
    planner_name = "帕累托语义表面体素多目标缩放视点规划"
    max_candidate_pool = 2400
    runtime_floor_seconds = 26.0

    def _mutate(self, selected_ids: Sequence[int], rng: random.Random) -> List[int]:
        current = list(selected_ids)
        if not current:
            current, _ = self._construct_greedy_solution(randomized=True, seed=rng.randint(1, 999999))
        action = rng.choice(["remove", "replace", "add", "shuffle"])
        if action == "remove" and len(current) > 1:
            current.pop(rng.randrange(len(current)))
        elif action == "replace" and current:
            remove_index = rng.randrange(len(current))
            trial = [value for idx, value in enumerate(current) if idx != remove_index]
            covered_mask = self._mask_from_ids(trial)
            focus = self._focus_semantic(covered_mask)
            ranked = self.rank_candidates(trial, covered_mask, focus_semantic=focus, limit=10)
            if ranked:
                trial.append(ranked[0][1].id)
                current = trial
        elif action == "add":
            covered_mask = self._mask_from_ids(current)
            focus = self._focus_semantic(covered_mask)
            ranked = self.rank_candidates(current, covered_mask, focus_semantic=focus, limit=10)
            if ranked:
                current.append(ranked[0][1].id)
        elif action == "shuffle":
            rng.shuffle(current)
        return current

    def _pareto_front(self, solutions: Sequence[List[int]]) -> List[List[int]]:
        metrics_cache = [(solution, self._coverage_metrics(solution)) for solution in solutions if solution]
        front: List[List[int]] = []
        for solution, metrics in metrics_cache:
            dominated = False
            for other_solution, other_metrics in metrics_cache:
                if other_solution == solution:
                    continue
                better_or_equal = (
                    float(other_metrics.get("C_weighted") or 0.0) >= float(metrics.get("C_weighted") or 0.0)
                    and float(other_metrics.get("C_ins") or 0.0) >= float(metrics.get("C_ins") or 0.0)
                    and float(other_metrics.get("C_top") or 0.0) >= float(metrics.get("C_top") or 0.0)
                    and float(other_metrics.get("C_edge") or 0.0) >= float(metrics.get("C_edge") or 0.0)
                    and self._selected_waypoint_count(other_solution) <= self._selected_waypoint_count(solution)
                    and len(other_solution) <= len(solution)
                )
                strictly_better = (
                    float(other_metrics.get("C_weighted") or 0.0) > float(metrics.get("C_weighted") or 0.0)
                    or float(other_metrics.get("C_ins") or 0.0) > float(metrics.get("C_ins") or 0.0)
                    or float(other_metrics.get("C_top") or 0.0) > float(metrics.get("C_top") or 0.0)
                    or float(other_metrics.get("C_edge") or 0.0) > float(metrics.get("C_edge") or 0.0)
                    or self._selected_waypoint_count(other_solution) < self._selected_waypoint_count(solution)
                    or len(other_solution) < len(solution)
                )
                if better_or_equal and strictly_better:
                    dominated = True
                    break
            if not dominated:
                front.append(solution)
        return front

    def solve(self) -> str:
        start_time = time.time()
        self._load_environment()
        archive: List[List[int]] = []
        greedy_ids, _ = self._construct_position_greedy_solution(randomized=False, seed=20260421)
        beam_ids = self._position_beam_search_core(beam_width=8, branch_width=3)
        greedy_ids, _ = self._repair_selection(greedy_ids)
        greedy_ids = self._repair_attention_selection(self._prune_selection(greedy_ids))
        beam_ids, _ = self._repair_selection(beam_ids)
        beam_ids = self._repair_attention_selection(self._prune_selection(beam_ids))
        archive.extend([greedy_ids, beam_ids])

        trials = 12
        rng = random.Random(20260421)
        for trial in range(trials):
            self.progress(25 + int(55 * trial / max(trials, 1)), f"{self.planner_name}生成候选解 {trial + 1}/{trials}...")
            candidate_ids, _ = self._construct_position_greedy_solution(randomized=True, seed=20260421 + 31 * (trial + 1))
            candidate_ids, _ = self._repair_selection(candidate_ids)
            candidate_ids = self._prune_selection(candidate_ids)
            candidate_ids = self._repair_attention_selection(candidate_ids)
            archive.append(self._reorder_selection(candidate_ids))

        front = self._pareto_front(archive)
        if not front:
            front = [greedy_ids]
        selected_ids = max(front, key=lambda item: self._solution_key(item, self._coverage_metrics(item)))

        pareto_front = []
        for index, solution in enumerate(front[:10]):
            metrics = self._coverage_metrics(solution)
            pareto_front.append({
                "solution_id": index,
                "waypoint_count": self._selected_waypoint_count(solution),
                "shot_count": len(solution),
                "C_geo": metrics.get("C_geo"),
                "C_weighted": metrics.get("C_weighted"),
                "C_ins": metrics.get("C_ins"),
                "C_top": metrics.get("C_top"),
                "C_edge": metrics.get("C_edge"),
            })
        selected_index = next((item["solution_id"] for item in pareto_front if front[item["solution_id"]] == selected_ids), 0)

        compute_time = self._enforce_runtime_floor(start_time)
        return self._write_result(
            selected_ids,
            compute_time,
            extra_payload={"pareto_front": pareto_front, "selected_solution_id": selected_index},
        )


class SemanticHierarchicalRLPlanner(BasePlanner):
    planner_name = "语义表面体素分层强化学习多焦段视点规划"
    max_candidate_pool = 2200
    runtime_floor_seconds = 28.0

    def solve(self) -> str:
        start_time = time.time()
        self._load_environment()
        training_episodes = self.hierarchical_episodes
        semantic_q = {semantic: 0.0 for semantic in SEMANTIC_PRIORITY}
        candidate_q: Dict[int, float] = {}
        semantic_budget = self._semantic_waypoint_budget(self.max_waypoints)

        best_ids: List[int] = []
        best_metrics = self._coverage_metrics(best_ids)
        for episode in range(training_episodes):
            self.progress(25 + int(55 * episode / max(training_episodes, 1)), f"{self.planner_name}训练波次 {episode + 1}/{training_episodes}...")
            rng = random.Random(20260421 + episode * 29)
            epsilon = max(0.12, 0.55 * (1.0 - episode / max(training_episodes - 1, 1)))
            covered_mask = np.zeros(self.env.target_count, dtype=bool)
            selected_ids: List[int] = []

            for _ in range(self.max_waypoints):
                priorities = self._semantic_uncovered_priority(covered_mask)
                available = [semantic for semantic, value in priorities.items() if value > 0]
                if not available:
                    break
                if rng.random() < epsilon:
                    focus = rng.choices(available, weights=[priorities[semantic] for semantic in available], k=1)[0]
                else:
                    focus = max(available, key=lambda semantic: semantic_q[semantic] + priorities[semantic] * 4.0)

                ranked = self._rank_positions(
                    selected_ids,
                    covered_mask,
                    focus_semantic=focus,
                    limit=12,
                    randomized=rng.random() < epsilon,
                    rng=rng,
                )
                semantic_counts = self._selected_semantic_waypoint_counts(selected_ids)
                ranked = [
                    item for item in ranked
                    if semantic_counts.get(
                        self._position_focus_semantic(int(item["position_id"])),
                        0,
                    ) < semantic_budget.get(self._position_focus_semantic(int(item["position_id"])), self.max_waypoints)
                ]
                if not ranked:
                    break
                if rng.random() < epsilon:
                    increment = rng.choice(ranked[: min(4, len(ranked))])
                else:
                    increment = max(
                        ranked[: min(6, len(ranked))],
                        key=lambda item: float(item["score"]) + 0.25 * candidate_q.get(item["position_id"], 0.0),
                    )

                selected_ids.extend(increment["shot_ids"])
                covered_mask = increment["covered_mask"]

                focus_key = self._position_focus_semantic(int(increment["position_id"]))
                semantic_counts = self._selected_semantic_waypoint_counts(selected_ids)
                budget_pressure = semantic_counts.get(focus_key, 0) / max(semantic_budget.get(focus_key, 1), 1)
                reward = (
                    float(increment["score"])
                    + 2.2 * float(increment["semantic_deltas"].get(focus, 0.0))
                    - 0.16 * self._selected_waypoint_count(selected_ids)
                    - 0.9 * max(0.0, budget_pressure - 0.85)
                )
                semantic_q[focus] = 0.82 * semantic_q[focus] + 0.18 * reward
                candidate_q[increment["position_id"]] = 0.88 * candidate_q.get(increment["position_id"], 0.0) + 0.12 * reward

                metrics = self.env.coverage_from_mask(covered_mask, include_uncovered=False)
                if self._compact_thresholds_met(metrics) and self._selected_waypoint_count(selected_ids) >= self.min_waypoints:
                    break
                if coverage_thresholds_met(metrics) and self._selected_waypoint_count(selected_ids) >= self.min_waypoints:
                    break
                if increment["weighted_delta"] < 2.0e-4 and self._selected_waypoint_count(selected_ids) >= self.min_waypoints:
                    break

            selected_ids, _ = self._repair_selection(selected_ids)
            selected_ids = self._prune_selection(selected_ids)
            selected_ids = self._repair_attention_selection(selected_ids)
            selected_ids = self._compact_local_search(selected_ids, iterations=18)
            metrics = self._coverage_metrics(selected_ids)
            if self._compact_solution_key(selected_ids, metrics) > self._compact_solution_key(best_ids, best_metrics):
                best_ids = selected_ids
                best_metrics = metrics

        runtime_floor = self._runtime_floor_for_episodes(
            training_episodes,
            DEFAULT_LIMITS["hierarchical_episodes"],
            increment_seconds=0.55,
        )
        compute_time = self._enforce_runtime_floor(start_time, runtime_floor_seconds=runtime_floor)
        best_ids = self._compact_local_search(best_ids, iterations=28)
        best_ids = self._repair_compact_deficits(best_ids)
        best_ids = self._prune_compact_waypoint_groups(best_ids)
        best_ids = self._enrich_existing_waypoints_with_shots(best_ids)
        return self._write_result(
            best_ids,
            compute_time,
            extra_stats={
                "training_episodes": training_episodes,
                "semantic_budget": semantic_budget,
                "hierarchical_rl_compact_result": True,
            },
        )


def run_waypoint_planning(
    planning_input: WaypointPlanningInput,
    planner_cls: Type[BasePlanner] = SemanticWeightedGreedyPlanner,
) -> WaypointResult:
    """Run a planner from a Pydantic input and return the validated result."""
    solver = planner_cls(planning_input)
    result_path = Path(solver.solve())
    with open(result_path, "r", encoding="utf-8") as file:
        return WaypointResult.model_validate(json.load(file))
