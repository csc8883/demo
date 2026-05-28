"""
处理任务服务：体素化、航点规划等后台任务。
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Optional

from backend.core.security import get_user_dir, safe_filename


PLANNER_REGISTRY: Dict[str, Dict[str, Any]] = {}

# 将在 server.py 中通过 _init_planner_registry() 延迟填充


def _init_planner_registry() -> None:
    """延迟导入并填充 PLANNER_REGISTRY，避免循环依赖。"""
    from waypoint_planning.algorithms import (
        HRLSolver,
        SemanticSingleLayerRLPlanner,
        SemanticWeightedGreedyPlanner,
    )

    PLANNER_REGISTRY.clear()
    PLANNER_REGISTRY.update({
        SemanticWeightedGreedyPlanner.planner_name: {
            "name": SemanticWeightedGreedyPlanner.planner_name,
            "solver": SemanticWeightedGreedyPlanner,
            "description": "基线算法：按覆盖增益、安全距离、关键目标和路径代价选择多焦段视点。",
            "parameters": [
                {"key": "safety_distance_m", "label": "安全距离(m)", "type": "number", "default": 5.0, "min": 2.5, "max": 10.0, "step": 0.1},
            ],
        },
        SemanticSingleLayerRLPlanner.planner_name: {
            "name": SemanticSingleLayerRLPlanner.planner_name,
            "solver": SemanticSingleLayerRLPlanner,
            "description": "单层强化学习策略：在统一奖励函数下优化覆盖率、航点数和安全距离。",
            "parameters": [
                {"key": "safety_distance_m", "label": "安全距离(m)", "type": "number", "default": 5.0, "min": 2.5, "max": 10.0, "step": 0.1},
                {"key": "single_layer_episodes", "label": "训练轮次", "type": "number", "default": 10, "min": 1, "max": 50, "step": 1},
            ],
        },
        HRLSolver.planner_name: {
            "name": HRLSolver.planner_name,
            "solver": HRLSolver,
            "description": "分层强化学习策略：高层选择语义关注区域，低层选择安全航点和拍摄动作。",
            "parameters": [
                {"key": "safety_distance_m", "label": "安全距离(m)", "type": "number", "default": 5.0, "min": 2.5, "max": 10.0, "step": 0.1},
                {"key": "hierarchical_episodes", "label": "训练轮次", "type": "number", "default": 10, "min": 1, "max": 50, "step": 1},
            ],
        },
    })


def build_planner_constraints_payload(
    safety_distance_m: Optional[float] = None,
    conductor_no_fly_enabled: Optional[bool] = None,
    conductor_no_fly_extent_margin_m: Optional[float] = None,
    conductor_no_fly_min_length_m: Optional[float] = None,
    conductor_no_fly_boundary_tolerance_m: Optional[float] = None,
    max_waypoints: Optional[int] = None,
    max_shots_per_waypoint: Optional[int] = None,
    single_layer_episodes: Optional[int] = None,
    hierarchical_episodes: Optional[int] = None,
    manual_ratio_min: Optional[float] = None,
    manual_ratio_max: Optional[float] = None,
    target_manual_ratio: Optional[float] = None,
) -> dict:
    """从 API 表单值构建规划器约束。"""
    constraints = {
        key: value
        for key, value in {
            "safety_distance_m": safety_distance_m,
            "conductor_no_fly_enabled": conductor_no_fly_enabled,
            "conductor_no_fly_extent_margin_m": conductor_no_fly_extent_margin_m,
            "conductor_no_fly_min_length_m": conductor_no_fly_min_length_m,
            "conductor_no_fly_boundary_tolerance_m": conductor_no_fly_boundary_tolerance_m,
            "max_waypoints": max_waypoints,
            "max_shots_per_waypoint": max_shots_per_waypoint,
            "single_layer_episodes": single_layer_episodes,
            "hierarchical_episodes": hierarchical_episodes,
            "manual_ratio_min": manual_ratio_min,
            "manual_ratio_max": manual_ratio_max,
        }.items()
        if value is not None
    }
    if target_manual_ratio is not None:
        ratio = max(0.60, min(0.80, float(target_manual_ratio)))
        constraints["manual_ratio_min"] = round(ratio, 2)
        constraints["manual_ratio_max"] = round(ratio, 2)
    return constraints


def clamp_optional_float(value: Optional[float], default: float, low: float, high: float) -> float:
    """Clamp an optional form float with a stable default."""
    if value is None:
        value = default
    return max(low, min(high, float(value)))


def build_route_clearance_payload(
    safety_distance_m: Optional[float] = None,
    clearance_m: Optional[float] = None,
    wire_clearance_m: Optional[float] = None,
    tower_clearance_m: Optional[float] = None,
    task_tower_clearance_m: Optional[float] = None,
    task_wire_clearance_m: Optional[float] = None,
) -> dict:
    """Normalize route safety input."""
    unified = None if safety_distance_m is None else max(0.5, min(40.0, float(safety_distance_m)))
    default_tower = unified if unified is not None else 6.0
    default_wire = unified if unified is not None else 10.0
    default_task_tower = unified if unified is not None else 3.0
    default_task_wire = unified if unified is not None else 5.0
    tower_value = clearance_m if clearance_m is not None else tower_clearance_m
    return {
        "safety_distance_m": unified,
        "clearance_m": clamp_optional_float(tower_value, default_tower, 0.5, 30.0),
        "wire_clearance_m": clamp_optional_float(wire_clearance_m, default_wire, 0.5, 40.0),
        "task_tower_clearance_m": clamp_optional_float(task_tower_clearance_m, default_task_tower, 0.5, 30.0),
        "task_wire_clearance_m": clamp_optional_float(task_wire_clearance_m, default_task_wire, 0.5, 40.0),
    }
