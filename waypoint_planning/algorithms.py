from __future__ import annotations

import json
import math
import os
from pathlib import Path
from typing import Dict, Optional

import laspy
import numpy as np

from .planning_core import (
    GROUND_WIRE_LABEL,
    INSULATOR_LABEL,
    TOWER_LABEL,
    WIRE_LABEL,
    build_semantic_surface_model,
    compute_waypoint_metrics,
    evaluate_waypoint_coverage,
    generate_candidate_views,
    parse_manual_route,
)
from .planning_solvers import (
    BeamSetCoverPlanner as _BeamSetCoverPlanner,
    ParetoSemanticPlanner as _ParetoSemanticPlanner,
    RandomizedGreedyRepairPlanner as _RandomizedGreedyRepairPlanner,
    SemanticHierarchicalRLPlanner as _SemanticHierarchicalRLPlanner,
    SemanticSingleLayerRLPlanner as _SemanticSingleLayerRLPlanner,
    SemanticWeightedGreedyPlanner as _SemanticWeightedGreedyPlanner,
)


VOXEL_SIZE = 0.10
DISPLAY_KEY_LABELS = {TOWER_LABEL, INSULATOR_LABEL, GROUND_WIRE_LABEL, WIRE_LABEL}

process_status: Dict[str, Dict[str, object]] = {
    "voxelize": {"progress": 0, "status": "idle"},
    "rl": {"progress": 0, "status": "idle"},
    "ai": {"progress": 0, "status": "idle"},
}


def update_progress(task: str, progress: int, status: str = "running"):
    process_status[task] = {"progress": int(progress), "status": status}


def read_las_for_vis(file_path: str):
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"文件不存在: {file_path}")

    with laspy.open(file_path) as file:
        las = file.read()
        points = np.vstack((las.x, las.y, las.z)).transpose()
        labels = np.asarray(las.classification, dtype=int)

        colors = np.zeros((len(points), 3), dtype=float)
        if hasattr(las, "red"):
            max_value = 65535.0 if np.max(las.red) > 255 else 255.0
            colors = np.vstack((las.red, las.green, las.blue)).transpose() / max_value
        else:
            z = points[:, 2]
            z_min, z_max = np.min(z), np.max(z)
            z_norm = (z - z_min) / max(z_max - z_min, 1e-6)
            colors[:, 0] = 0.20 + 0.40 * z_norm
            colors[:, 1] = 0.45 + 0.35 * z_norm
            colors[:, 2] = 0.70 - 0.28 * z_norm

        colors[labels == TOWER_LABEL] = [1.00, 0.05, 0.05]
        colors[labels == INSULATOR_LABEL] = [0.05, 0.35, 1.00]
        colors[labels == WIRE_LABEL] = [0.98, 0.84, 0.18]
        colors[labels == GROUND_WIRE_LABEL] = [0.25, 0.56, 1.00]

        center = np.mean(points, axis=0).tolist()
        max_points = 80000
        if len(points) > max_points:
            key_mask = np.isin(labels, list(DISPLAY_KEY_LABELS))
            key_idx = np.where(key_mask)[0]
            other_idx = np.where(~key_mask)[0]
            keep_key = key_idx
            if len(keep_key) > int(max_points * 0.70):
                keep_key = np.random.choice(keep_key, int(max_points * 0.70), replace=False)
            remain = max_points - len(keep_key)
            keep_other = np.random.choice(other_idx, min(remain, len(other_idx)), replace=False) if remain > 0 else []
            indices = np.concatenate([keep_key, keep_other])
            points = points[indices]
            colors = colors[indices]
            labels = labels[indices]

        return {
            "points": points.tolist(),
            "colors": colors.tolist(),
            "labels": labels.astype(int).tolist(),
            "center": center,
        }


class Pretreatment:
    def __init__(
        self,
        las_path: str,
        output_dir: str,
        status_key: str = "voxelize",
        manual_route_path: Optional[str] = None,
    ):
        self.las_path = las_path
        self.output_dir = output_dir
        self.base_name = Path(las_path).stem
        self.status_key = status_key
        self.manual_route_path = manual_route_path

    def run(self):
        update_progress(self.status_key, 5, "读取点云...")
        with laspy.open(self.las_path) as file:
            las = file.read()
            points = np.vstack((las.x, las.y, las.z)).transpose()
            labels = np.asarray(las.classification, dtype=int)

        if not np.any(np.isin(labels, [TOWER_LABEL, INSULATOR_LABEL])):
            raise ValueError("点云中没有可体素化的杆塔或绝缘子目标")

        update_progress(self.status_key, 30, "构建语义表面体素...")
        surface_model = build_semantic_surface_model(points, labels, VOXEL_SIZE)

        update_progress(self.status_key, 75, "生成多焦段候选视点...")
        candidates = generate_candidate_views(surface_model, manual_route_path=self.manual_route_path)
        if not candidates:
            raise ValueError("未生成有效候选视点")

        Path(self.output_dir).mkdir(parents=True, exist_ok=True)
        out_voxel = Path(self.output_dir) / f"{self.base_name}_voxel.npz"
        out_candidates = Path(self.output_dir) / f"{self.base_name}_candidates.json"

        np.savez_compressed(
            out_voxel,
            min_bound=surface_model["min_bound"],
            voxel_size=VOXEL_SIZE,
            z_max_map=surface_model["z_max_map"],
            voxels=np.array(surface_model["voxels"], dtype=object),
            attention_targets=np.array(surface_model.get("attention_targets", []), dtype=object),
            display_voxels=np.array(surface_model.get("display_voxels", surface_model["voxels"]), dtype=object),
            target_cells=np.array(surface_model.get("target_cells", []), dtype=object),
            wire_curves=np.array(surface_model.get("wire_curves", []), dtype=object),
            conductor_no_fly_volumes=np.array(surface_model.get("conductor_no_fly_volumes", []), dtype=object),
            safety_points=np.asarray(surface_model.get("safety_points", np.empty((0, 3))), dtype=float),
            safety_labels=np.asarray(surface_model.get("safety_labels", np.empty((0,))), dtype=int),
            local_center=np.asarray(surface_model["local_center"], dtype=float),
            local_frame=np.array(surface_model.get("local_frame", {}), dtype=object),
            meta_json=json.dumps(surface_model.get("meta", {}), ensure_ascii=False),
        )
        with open(out_candidates, "w", encoding="utf-8") as file:
            json.dump(candidates, file, indent=2, ensure_ascii=False)

        update_progress(self.status_key, 100, "completed")
        return str(out_voxel)


class _PlannerMixin:
    def __init__(self, *args, status_key: Optional[str] = None, **kwargs):
        if status_key is None:
            status_key = getattr(args[0], "status_key", "rl") if args else "rl"
        super().__init__(*args, status_key=status_key, progress_callback=update_progress, **kwargs)


class SemanticWeightedGreedyPlanner(_PlannerMixin, _SemanticWeightedGreedyPlanner):
    pass


class SemanticSingleLayerRLPlanner(_PlannerMixin, _SemanticSingleLayerRLPlanner):
    pass


class RandomizedGreedyRepairPlanner(_PlannerMixin, _RandomizedGreedyRepairPlanner):
    pass


class BeamSetCoverPlanner(_PlannerMixin, _BeamSetCoverPlanner):
    pass


class ParetoSemanticPlanner(_PlannerMixin, _ParetoSemanticPlanner):
    pass


class HRLSolver(_PlannerMixin, _SemanticHierarchicalRLPlanner):
    pass


class WeightedSetCoverPlanner(BeamSetCoverPlanner):
    pass


MAIN_PLANNER_REGISTRY = {
    _SemanticWeightedGreedyPlanner.planner_name: SemanticWeightedGreedyPlanner,
    _SemanticSingleLayerRLPlanner.planner_name: SemanticSingleLayerRLPlanner,
    _SemanticHierarchicalRLPlanner.planner_name: HRLSolver,
}

PLANNER_MODE = {
    "greedy": SemanticWeightedGreedyPlanner,
    "rl": SemanticSingleLayerRLPlanner,
    "hierarchical_rl": HRLSolver,
}

DEFAULT_PLANNER_MODE = "greedy"


def create_planner(mode: str = "greedy", **kwargs):
    """Create a planner instance for the given mode, with fallback to greedy.

    Returns (planner_instance, fallback_reason_or_None).
    """
    planner_cls = PLANNER_MODE.get(mode)
    fallback_reason = None

    if planner_cls is None:
        planner_cls = SemanticWeightedGreedyPlanner
        fallback_reason = f"unknown_planner_mode_{mode}_fallback_to_greedy"

    try:
        instance = planner_cls(**kwargs)
    except Exception as exc:
        instance = SemanticWeightedGreedyPlanner(**kwargs)
        fallback_reason = f"{mode}_init_failed_{type(exc).__name__}_fallback_to_greedy"

    return instance, fallback_reason


__all__ = [
    "Pretreatment",
    "HRLSolver",
    "SemanticSingleLayerRLPlanner",
    "WeightedSetCoverPlanner",
    "SemanticWeightedGreedyPlanner",
    "RandomizedGreedyRepairPlanner",
    "BeamSetCoverPlanner",
    "ParetoSemanticPlanner",
    "read_las_for_vis",
    "parse_manual_route",
    "compute_waypoint_metrics",
    "evaluate_waypoint_coverage",
    "MAIN_PLANNER_REGISTRY",
    "process_status",
    "update_progress",
]
