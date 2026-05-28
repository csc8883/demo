from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path
from typing import List, Tuple

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from waypoint_planning.planning_core import (  # noqa: E402
    CRITICAL_SAFETY_LABELS,
    SUPPORTED_FOCALS,
    build_label_preserving_safety_points,
    build_conductor_no_fly_volumes_from_point_cloud,
    generate_candidate_views,
    load_conductor_no_fly_volumes,
    yaw_pitch_to_target,
)
from waypoint_planning.route_planner import _segment_crosses_no_fly, _segment_is_safe, _set_no_fly_margin  # noqa: E402
from waypoint_planning.route_planner import _load_safety_points, _no_fly_volumes  # noqa: E402
from waypoint_planning.route_planner import plan_route_from_waypoints, validate_route_safety  # noqa: E402
from waypoint_planning.planning_solvers import SemanticWeightedGreedyPlanner  # noqa: E402
from waypoint_planning.waypoint_models import WaypointPlanningInput  # noqa: E402


def _sample_no_fly_records() -> List[dict]:
    """Build a small conductor no-fly volume from mock wire and ground-wire points."""
    center = np.array([0.0, 0.0, 10.0], dtype=float)
    wire = np.array(
        [
            [-30.0, -5.0, 12.0],
            [30.0, -5.0, 12.0],
            [-30.0, 5.0, 12.0],
            [30.0, 5.0, 12.0],
        ],
        dtype=float,
    )
    ground_wire = np.array(
        [
            [-30.0, -6.0, 22.0],
            [30.0, -6.0, 22.0],
            [-30.0, 6.0, 22.0],
            [30.0, 6.0, 22.0],
        ],
        dtype=float,
    )
    return build_conductor_no_fly_volumes_from_point_cloud(center, wire, ground_wire)


def _branch_points(
    axis: Tuple[float, float, float],
    distances: Tuple[float, float] = (30.0, 60.0),
    conductor_lateral_m: float = 5.0,
    ground_lateral_m: float = 6.0,
) -> Tuple[np.ndarray, np.ndarray]:
    """Build wire/ground-wire samples for one line branch."""
    unit = np.asarray(axis, dtype=float)
    unit[2] = 0.0
    unit = unit / float(np.linalg.norm(unit[:2]))
    lateral = np.asarray([-unit[1], unit[0], 0.0], dtype=float)
    wire: List[np.ndarray] = []
    ground: List[np.ndarray] = []
    center = np.array([0.0, 0.0, 10.0], dtype=float)
    for distance in distances:
        for offset in (-conductor_lateral_m, conductor_lateral_m):
            point = center + unit * distance + lateral * offset
            point[2] = 12.0
            wire.append(point)
        for offset in (-ground_lateral_m, ground_lateral_m):
            point = center + unit * distance + lateral * offset
            point[2] = 22.0
            ground.append(point)
    return np.asarray(wire, dtype=float), np.asarray(ground, dtype=float)


def _bent_no_fly_records() -> List[dict]:
    """Build two no-fly volumes whose front/back line directions differ."""
    front_wire, front_ground = _branch_points((1.0, 0.0, 0.0))
    back_wire, back_ground = _branch_points((-0.45, -0.89, 0.0))
    return build_conductor_no_fly_volumes_from_point_cloud(
        np.array([0.0, 0.0, 10.0], dtype=float),
        np.vstack([front_wire, back_wire]),
        np.vstack([front_ground, back_ground]),
    )


def test_volume_contains_and_clearance() -> None:
    """Verify inside points are blocked and exterior points are retained."""
    records = _sample_no_fly_records()
    assert records, "expected a no-fly volume"
    volume = load_conductor_no_fly_volumes(records)[0]
    assert volume.contains([0.0, 0.0, 16.0])
    assert not volume.contains([0.0, 12.0, 16.0])
    assert volume.clearance([0.0, 12.0, 16.0]) > 0.0


def test_volume_is_cuboid_and_blocks_crossing_segments() -> None:
    """Verify the conductor no-fly region acts as an oriented cuboid through the route planner."""
    records = _sample_no_fly_records()
    volume = load_conductor_no_fly_volumes(records)[0]
    record = volume.to_record()
    assert record["z_min"] == 12.0
    assert record["z_max"] == 22.0
    assert volume.contains([0.0, 0.0, 21.8])
    assert not volume.contains([0.0, 0.0, 23.0])

    safety = {"no_fly": [volume]}
    assert _segment_crosses_no_fly(np.array([-70.0, 0.0, 16.0]), np.array([70.0, 0.0, 16.0]), safety)
    assert not _segment_crosses_no_fly(np.array([-70.0, 12.0, 16.0]), np.array([70.0, 12.0, 16.0]), safety)
    assert not _segment_crosses_no_fly(np.array([-70.0, 0.0, 35.0]), np.array([70.0, 0.0, 35.0]), safety)


def test_bent_line_builds_two_tower_side_volumes() -> None:
    """Verify front/back line directions become separate no-fly volumes."""
    records = _bent_no_fly_records()
    volumes = load_conductor_no_fly_volumes(records)
    assert len(volumes) == 2

    expected_axes = [
        np.asarray([1.0, 0.0, 0.0], dtype=float),
        np.asarray([-0.45, -0.89, 0.0], dtype=float) / float(np.linalg.norm([-0.45, -0.89])),
    ]
    for expected in expected_axes:
        assert max(float(np.dot(volume.u_axis[:2], expected[:2])) for volume in volumes) > 0.96
    for volume in volumes:
        assert -2.1 <= volume.u_min <= 0.0
        assert volume.u_max >= 50.0


def test_sparse_line_falls_back_to_single_volume() -> None:
    """Verify sparse inputs keep the legacy single-volume safety fallback."""
    center = np.array([0.0, 0.0, 10.0], dtype=float)
    wire = np.array([[-30.0, -5.0, 12.0], [0.0, 5.0, 12.0], [30.0, -5.0, 12.0]], dtype=float)
    ground_wire = np.array([[-30.0, -6.0, 22.0], [0.0, 6.0, 22.0], [30.0, -6.0, 22.0]], dtype=float)
    volumes = load_conductor_no_fly_volumes(
        build_conductor_no_fly_volumes_from_point_cloud(center, wire, ground_wire)
    )
    assert len(volumes) == 1
    assert volumes[0].u_min < -40.0
    assert volumes[0].u_max > 40.0


def test_volume_uses_sloped_top_and_bottom_planes() -> None:
    """Verify conductor and ground-wire side anchors define the vertical bounds."""
    center = np.array([0.0, 0.0, 10.0], dtype=float)
    wire = np.array(
        [
            [-30.0, -5.0, 10.0],
            [30.0, -5.0, 10.0],
            [-30.0, 5.0, 14.0],
            [30.0, 5.0, 14.0],
        ],
        dtype=float,
    )
    ground_wire = np.array(
        [
            [-30.0, -6.0, 20.0],
            [30.0, -6.0, 20.0],
            [-30.0, 6.0, 24.0],
            [30.0, 6.0, 24.0],
        ],
        dtype=float,
    )
    volume = load_conductor_no_fly_volumes(
        build_conductor_no_fly_volumes_from_point_cloud(center, wire, ground_wire)
    )[0]
    assert volume.contains([0.0, 0.0, 13.0])
    assert not volume.contains([0.0, 0.0, 11.0])
    assert not volume.contains([0.0, 0.0, 23.0])
    assert volume.bottom_z(-5.0) < volume.bottom_z(5.0)
    assert volume.top_z(-6.0) < volume.top_z(6.0)


def test_candidate_generation_uses_split_no_fly_volumes() -> None:
    """Verify waypoint candidates honor both front/back no-fly branches."""
    records = _bent_no_fly_records()
    volumes = load_conductor_no_fly_volumes(records)
    front_target = np.asarray([40.0, 0.0, 16.0], dtype=float)
    back_axis = np.asarray([-0.45, -0.89, 0.0], dtype=float)
    back_axis = back_axis / float(np.linalg.norm(back_axis[:2]))
    back_target = np.asarray([0.0, 0.0, 16.0], dtype=float) + back_axis * 40.0
    surface_model = {
        "voxels": [
            {"coord": front_target.tolist(), "semantic": "insulator", "label": 22},
            {"coord": back_target.tolist(), "semantic": "insulator", "label": 22},
        ],
        "attention_targets": [],
        "local_center": np.asarray([0.0, 0.0, 10.0], dtype=float),
        "meta": {"z_min": 0.0, "z_max": 30.0, "tower_height": 30.0, "insulator_clusters": []},
        "conductor_no_fly_volumes": records,
    }
    candidates = generate_candidate_views(surface_model)
    exterior = [candidate for candidate in candidates if candidate.get("source") == "no_fly_exterior"]
    assert exterior
    assert all(
        min(volume.clearance(candidate["utm_position"]) for volume in volumes) >= 5.0
        for candidate in exterior
    )
    assert any(float(np.dot((np.asarray(candidate["utm_position"]) - np.asarray([0.0, 0.0, 10.0]))[:2], [1.0, 0.0])) > 20.0 for candidate in exterior)
    assert any(float(np.dot((np.asarray(candidate["utm_position"]) - np.asarray([0.0, 0.0, 10.0]))[:2], back_axis[:2])) > 20.0 for candidate in exterior)


def test_volume_uses_side_planes_and_vertical_safety_margins() -> None:
    """Verify side boundaries and vertical margins follow conductor/ground-wire anchors."""
    center = np.array([0.0, 0.0, 10.0], dtype=float)
    wire = np.array(
        [
            [-30.0, -5.0, 10.0],
            [30.0, -5.0, 10.0],
            [-30.0, 5.0, 14.0],
            [30.0, 5.0, 14.0],
        ],
        dtype=float,
    )
    ground_wire = np.array(
        [
            [-30.0, -7.0, 20.0],
            [30.0, -7.0, 20.0],
            [-30.0, 7.0, 24.0],
            [30.0, 7.0, 24.0],
        ],
        dtype=float,
    )
    volume = load_conductor_no_fly_volumes(
        build_conductor_no_fly_volumes_from_point_cloud(
            center,
            wire,
            ground_wire,
            top_margin_m=2.0,
            bottom_margin_m=1.0,
        )
    )[0]

    assert volume.contains(volume.world_position(0.0, 0.0, 13.0))
    assert volume.contains(volume.world_position(0.0, 0.0, 11.5))
    assert not volume.contains(volume.world_position(0.0, 0.0, 10.5))
    assert volume.contains(volume.world_position(0.0, 0.0, 23.5))
    assert not volume.contains(volume.world_position(0.0, 0.0, 24.5))

    left_at_15, _ = volume.side_bounds_at_z(15.0)
    _, right_at_19 = volume.side_bounds_at_z(19.0)
    assert volume.contains(volume.world_position(0.0, left_at_15 + 0.1, 15.0))
    assert not volume.contains(volume.world_position(0.0, left_at_15 - 0.5, 15.0))
    assert volume.contains(volume.world_position(0.0, right_at_19 - 0.1, 19.0))
    assert not volume.contains(volume.world_position(0.0, right_at_19 + 0.5, 19.0))


def test_no_fly_margin_blocks_near_boundary_segments() -> None:
    """Verify safety distance expands the conductor no-fly constraint around the cuboid."""
    volume = load_conductor_no_fly_volumes(_sample_no_fly_records())[0]
    safety = {
        "no_fly": [volume],
        "wire": np.empty((0, 3), dtype=float),
        "tower": np.empty((0, 3), dtype=float),
    }
    near_boundary_start = np.array([-70.0, 10.0, 16.0], dtype=float)
    near_boundary_end = np.array([70.0, 10.0, 16.0], dtype=float)
    assert not _segment_crosses_no_fly(near_boundary_start, near_boundary_end, safety)
    assert _segment_is_safe(near_boundary_start, near_boundary_end, safety, 1.0, 1.0)
    _set_no_fly_margin(safety, 5.0)
    assert not _segment_is_safe(near_boundary_start, near_boundary_end, safety, 1.0, 1.0)


def test_label_preserving_safety_points_keep_critical_labels() -> None:
    """Verify wire, ground-wire, tower, and insulator points are not dropped by downsampling."""
    critical_points = np.array(
        [
            [0.0, 0.0, 0.0],
            [0.2, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.2, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [2.2, 0.0, 0.0],
            [3.0, 0.0, 0.0],
            [3.2, 0.0, 0.0],
        ],
        dtype=float,
    )
    critical_labels = np.array([0, 0, 3, 3, 16, 16, 22, 22], dtype=int)
    background_points = np.array([[10.0 + 0.05 * index, 0.0, 0.0] for index in range(40)], dtype=float)
    background_labels = np.full(len(background_points), 4, dtype=int)
    safety_points, safety_labels, meta = build_label_preserving_safety_points(
        np.vstack([critical_points, background_points]),
        np.concatenate([critical_labels, background_labels]),
        voxel_size=0.1,
    )
    for label in CRITICAL_SAFETY_LABELS:
        assert int(np.sum(safety_labels == label)) == int(np.sum(critical_labels == label))
    assert int(meta["critical_safety_point_count"]) == len(critical_points)
    assert len(safety_points) < len(critical_points) + len(background_points)


def test_route_safety_prefers_point_cloud_volume_over_stored_manual_volume() -> None:
    """Verify route planning uses the two-side conductor cuboid when stale manual volumes exist."""
    center = np.array([0.0, 0.0, 10.0], dtype=float)
    display_voxels = [
        {"coord": [-40.0, -5.0, 12.0], "label": 0, "category": "wire", "semantic": "wire"},
        {"coord": [40.0, -5.0, 12.0], "label": 0, "category": "wire", "semantic": "wire"},
        {"coord": [-40.0, 5.0, 12.0], "label": 0, "category": "wire", "semantic": "wire"},
        {"coord": [40.0, 5.0, 12.0], "label": 0, "category": "wire", "semantic": "wire"},
        {"coord": [-40.0, -6.0, 22.0], "label": 3, "category": "ground_wire", "semantic": "ground_wire"},
        {"coord": [40.0, -6.0, 22.0], "label": 3, "category": "ground_wire", "semantic": "ground_wire"},
        {"coord": [-40.0, 6.0, 22.0], "label": 3, "category": "ground_wire", "semantic": "ground_wire"},
        {"coord": [40.0, 6.0, 22.0], "label": 3, "category": "ground_wire", "semantic": "ground_wire"},
    ]
    stale_manual_records = build_conductor_no_fly_volumes_from_point_cloud(
        center=[0.0, 30.0, 10.0],
        wire_points=np.array([[-40.0, 30.0, 12.0], [40.0, 30.0, 12.0], [-40.0, 38.0, 12.0], [40.0, 38.0, 12.0]], dtype=float),
        ground_wire_points=np.array([[-40.0, 30.0, 22.0], [40.0, 30.0, 22.0], [-40.0, 38.0, 22.0], [40.0, 38.0, 22.0]], dtype=float),
        source="manual_route",
    )
    with tempfile.TemporaryDirectory() as tmp:
        voxel_path = Path(tmp) / "scene_voxel.npz"
        np.savez_compressed(
            voxel_path,
            display_voxels=np.array(display_voxels, dtype=object),
            safety_points=np.empty((0, 3), dtype=float),
            safety_labels=np.empty((0,), dtype=int),
            local_center=center,
            conductor_no_fly_volumes=np.array(stale_manual_records, dtype=object),
        )
        safety = _load_safety_points(voxel_path)
    volumes = _no_fly_volumes(safety)
    assert len(volumes) == 2
    assert safety["no_fly_source"] == "point_cloud"
    assert any(volume.contains([0.0, 0.0, 16.0]) for volume in volumes)
    assert all(abs(volume.v_min) <= 7.0 and abs(volume.v_max) <= 7.0 for volume in volumes)


def test_route_planning_ignores_manual_route_context() -> None:
    """Verify algorithm-route generation does not consume embedded manual-route context."""
    waypoint_payload = {
        "waypoints": [
            {
                "id": 1,
                "pos_utm": [652850.0, 3422890.0, 120.0],
                "target_center": [652852.0, 3422892.0, 118.0],
                "semantic_focus": "insulator",
            }
        ],
        "route_context": {
            "UTM": "49",
            "linename": "MANUAL_LINE_SHOULD_NOT_APPEAR",
            "points": [
                {"AimType": "导线挂点", "target_utm": [652800.0, 3422880.0, 100.0]},
                {"AimType": "导线挂点", "target_utm": [652900.0, 3422880.0, 100.0]},
                {"AimType": "地线挂点", "target_utm": [652800.0, 3422900.0, 130.0]},
                {"AimType": "地线挂点", "target_utm": [652900.0, 3422900.0, 130.0]},
            ],
        },
    }
    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = Path(tmp)
        waypoint_path = tmp_path / "algorithm_waypoints.json"
        waypoint_path.write_text(json.dumps(waypoint_payload, ensure_ascii=False), encoding="utf-8")
        _, output = plan_route_from_waypoints(
            waypoint_path=waypoint_path,
            output_dir=tmp_path,
            voxel_path=None,
            clearance_m=1.0,
            wire_clearance_m=1.0,
            task_tower_clearance_m=1.0,
            task_wire_clearance_m=1.0,
        )
    assert "points" not in output
    assert output["linename"] == "algorithm_waypoints"
    assert output["UTM"] == "50"
    assert output["towercount"] == 1
    assert output["route_planning"]["conductor_no_fly_source"] is None
    assert output["route_planning"]["conductor_no_fly_volume_count"] == 0


def test_route_generation_uses_split_no_fly_for_task_points() -> None:
    """Verify front/back task points are adjusted and routed around split no-fly volumes."""
    route_center = np.asarray([652850.0, 3422890.0, 120.0], dtype=float)
    model_center = np.asarray([0.0, 0.0, 10.0], dtype=float)
    offset = route_center - model_center
    front_wire, front_ground = _branch_points((1.0, 0.0, 0.0))
    back_axis = np.asarray([-0.45, -0.89, 0.0], dtype=float)
    back_axis = back_axis / float(np.linalg.norm(back_axis[:2]))
    back_wire, back_ground = _branch_points(tuple(back_axis.tolist()))
    wire = np.vstack([front_wire, back_wire]) + offset
    ground = np.vstack([front_ground, back_ground]) + offset
    display_voxels = []
    for point in wire:
        display_voxels.append({"coord": point.tolist(), "label": 0, "category": "wire", "semantic": "wire"})
    for point in ground:
        display_voxels.append({"coord": point.tolist(), "label": 3, "category": "ground_wire", "semantic": "ground_wire"})

    front_task = route_center + np.asarray([40.0, 0.0, 6.0], dtype=float)
    back_task = route_center + back_axis * 40.0 + np.asarray([0.0, 0.0, 6.0], dtype=float)
    waypoint_payload = {
        "waypoints": [
            {"id": 1, "pos_utm": front_task.tolist(), "target_center": (front_task + np.asarray([2.0, 0.0, 0.0])).tolist(), "semantic_focus": "insulator"},
            {"id": 2, "pos_utm": back_task.tolist(), "target_center": (back_task + back_axis * 2.0).tolist(), "semantic_focus": "insulator"},
        ]
    }
    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = Path(tmp)
        voxel_path = tmp_path / "bent_scene_voxel.npz"
        np.savez_compressed(
            voxel_path,
            display_voxels=np.array(display_voxels, dtype=object),
            safety_points=np.empty((0, 3), dtype=float),
            safety_labels=np.empty((0,), dtype=int),
            local_center=route_center,
        )
        waypoint_path = tmp_path / "bent_waypoints.json"
        waypoint_path.write_text(json.dumps(waypoint_payload, ensure_ascii=False), encoding="utf-8")
        output_path, output = plan_route_from_waypoints(
            waypoint_path=waypoint_path,
            output_dir=tmp_path,
            voxel_path=voxel_path,
            clearance_m=1.0,
            wire_clearance_m=1.0,
            task_tower_clearance_m=1.0,
            task_wire_clearance_m=1.0,
        )
        safety = validate_route_safety(
            output_path,
            voxel_path,
            tower_clearance_m=1.0,
            wire_clearance_m=1.0,
            task_tower_clearance_m=1.0,
            task_wire_clearance_m=1.0,
        )
    assert output["route_planning"]["conductor_no_fly_volume_count"] == 2
    assert output["route_planning"]["task_point_count"] == 2
    assert safety["conductor_no_fly_volume_count"] == 2
    assert safety["conductor_no_fly_violation_count"] == 0


def _candidate(position: Tuple[float, float, float], target: List[float], candidate_id: int) -> dict:
    """Build one candidate JSON record that looks at a target."""
    yaw, pitch = yaw_pitch_to_target(position, target)
    focal = SUPPORTED_FOCALS["F1"]
    return {
        "id": candidate_id,
        "position_id": candidate_id,
        "utm_position": list(position),
        "pitch": pitch,
        "yaw": yaw,
        "focal_level": "F1",
        "f_eq_mm": focal["f_eq_mm"],
        "hfov_deg": focal["hfov_deg"],
        "vfov_deg": focal["vfov_deg"],
        "semantic_focus": "insulator",
        "target_center": target,
    }


def test_solver_filters_legacy_candidate_inside_volume() -> None:
    """Verify solver-side filtering removes old candidate points inside the no-fly volume."""
    target = [10.0, 0.0, 16.0]
    voxel_record = {
        "coord": target,
        "label": 22,
        "category": "insulator",
        "semantic": "insulator",
        "is_target": True,
        "weight": 5.0,
        "required_resolution": 0.1,
        "incidence_max_deg": 80.0,
        "normal_hint": [0.0, 1.0, 0.0],
    }

    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = Path(tmp)
        voxel_path = tmp_path / "scene_voxel.npz"
        np.savez_compressed(
            voxel_path,
            voxels=np.array([voxel_record], dtype=object),
            attention_targets=np.array([], dtype=object),
            z_max_map=np.zeros((3, 3), dtype=float),
            min_bound=np.array([-50.0, -50.0, 0.0], dtype=float),
            voxel_size=1.0,
            safety_points=np.empty((0, 3), dtype=float),
            safety_labels=np.empty((0,), dtype=int),
            local_center=np.array([0.0, 0.0, 10.0], dtype=float),
            conductor_no_fly_volumes=np.array(_sample_no_fly_records(), dtype=object),
        )

        candidate_path = tmp_path / "scene_candidates.json"
        candidate_path.write_text(
            json.dumps(
                [
                    _candidate((10.0, 2.0, 16.0), target, 1),
                    _candidate((10.0, 12.0, 16.0), target, 2),
                ],
                ensure_ascii=False,
            ),
            encoding="utf-8",
        )

        planner = SemanticWeightedGreedyPlanner(
            WaypointPlanningInput(
                voxel_path=voxel_path,
                candidate_path=candidate_path,
                output_dir=tmp_path,
            )
        )
        planner._load_environment()
        kept_positions = [tuple(round(float(value), 3) for value in item.position) for item in planner.candidates]
        assert kept_positions == [(10.0, 12.0, 16.0)]


def test_solver_recomputes_legacy_candidate_safety_distance() -> None:
    """Verify stale candidate safety distances cannot bypass the current safety index."""
    target = [10.0, 0.0, 16.0]
    voxel_record = {
        "coord": target,
        "label": 22,
        "category": "insulator",
        "semantic": "insulator",
        "is_target": True,
        "weight": 5.0,
        "required_resolution": 0.1,
        "incidence_max_deg": 80.0,
        "normal_hint": [0.0, 1.0, 0.0],
    }

    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = Path(tmp)
        voxel_path = tmp_path / "scene_voxel.npz"
        np.savez_compressed(
            voxel_path,
            voxels=np.array([voxel_record], dtype=object),
            attention_targets=np.array([], dtype=object),
            z_max_map=np.zeros((3, 3), dtype=float),
            min_bound=np.array([-50.0, -50.0, 0.0], dtype=float),
            voxel_size=1.0,
            safety_points=np.array([[10.0, 12.0, 16.0]], dtype=float),
            safety_labels=np.array([0], dtype=int),
            local_center=np.array([0.0, 0.0, 10.0], dtype=float),
            conductor_no_fly_volumes=np.array([], dtype=object),
        )

        unsafe = _candidate((10.0, 12.0, 16.0), target, 1)
        unsafe["safety_distance_m"] = 999.0
        safe = _candidate((10.0, 20.0, 16.0), target, 2)
        safe["safety_distance_m"] = 999.0
        candidate_path = tmp_path / "scene_candidates.json"
        candidate_path.write_text(json.dumps([unsafe, safe], ensure_ascii=False), encoding="utf-8")

        planner = SemanticWeightedGreedyPlanner(
            WaypointPlanningInput(
                voxel_path=voxel_path,
                candidate_path=candidate_path,
                output_dir=tmp_path,
                constraints={"safety_distance_m": 5.0},
            )
        )
        planner._load_environment()
        kept_positions = [tuple(round(float(value), 3) for value in item.position) for item in planner.candidates]
    assert kept_positions == [(10.0, 20.0, 16.0)]


def test_solver_filters_candidate_near_no_fly_boundary() -> None:
    """Verify current safety distance also keeps waypoints away from no-fly faces."""
    target = [10.0, 0.0, 16.0]
    voxel_record = {
        "coord": target,
        "label": 22,
        "category": "insulator",
        "semantic": "insulator",
        "is_target": True,
        "weight": 5.0,
        "required_resolution": 0.1,
        "incidence_max_deg": 80.0,
        "normal_hint": [0.0, 1.0, 0.0],
    }

    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = Path(tmp)
        voxel_path = tmp_path / "scene_voxel.npz"
        np.savez_compressed(
            voxel_path,
            voxels=np.array([voxel_record], dtype=object),
            attention_targets=np.array([], dtype=object),
            z_max_map=np.zeros((3, 3), dtype=float),
            min_bound=np.array([-50.0, -50.0, 0.0], dtype=float),
            voxel_size=1.0,
            safety_points=np.empty((0, 3), dtype=float),
            safety_labels=np.empty((0,), dtype=int),
            local_center=np.array([0.0, 0.0, 10.0], dtype=float),
            conductor_no_fly_volumes=np.array(_sample_no_fly_records(), dtype=object),
        )

        candidate_path = tmp_path / "scene_candidates.json"
        candidate_path.write_text(
            json.dumps(
                [
                    _candidate((10.0, 7.0, 16.0), target, 1),
                    _candidate((10.0, 12.0, 16.0), target, 2),
                ],
                ensure_ascii=False,
            ),
            encoding="utf-8",
        )

        planner = SemanticWeightedGreedyPlanner(
            WaypointPlanningInput(
                voxel_path=voxel_path,
                candidate_path=candidate_path,
                output_dir=tmp_path,
                constraints={"safety_distance_m": 5.0},
            )
        )
        planner._load_environment()
        kept_positions = [tuple(round(float(value), 3) for value in item.position) for item in planner.candidates]
        assert kept_positions == [(10.0, 12.0, 16.0)]


def main() -> int:
    """Run the lightweight conductor no-fly regression tests."""
    test_volume_contains_and_clearance()
    test_volume_is_cuboid_and_blocks_crossing_segments()
    test_bent_line_builds_two_tower_side_volumes()
    test_sparse_line_falls_back_to_single_volume()
    test_volume_uses_sloped_top_and_bottom_planes()
    test_candidate_generation_uses_split_no_fly_volumes()
    test_volume_uses_side_planes_and_vertical_safety_margins()
    test_no_fly_margin_blocks_near_boundary_segments()
    test_label_preserving_safety_points_keep_critical_labels()
    test_route_safety_prefers_point_cloud_volume_over_stored_manual_volume()
    test_route_planning_ignores_manual_route_context()
    test_route_generation_uses_split_no_fly_for_task_points()
    test_solver_filters_legacy_candidate_inside_volume()
    test_solver_recomputes_legacy_candidate_safety_distance()
    test_solver_filters_candidate_near_no_fly_boundary()
    print("conductor no-fly volume tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
