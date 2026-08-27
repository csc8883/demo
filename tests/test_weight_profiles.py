from __future__ import annotations

import os
import tempfile
import unittest
from datetime import datetime
from pathlib import Path
from unittest.mock import patch

import laspy
import numpy as np

from backend.services import pointcloud_lod_service, weight_service
from waypoint_planning.planning_core import build_semantic_surface_model


def front_operation(mode: str, x1: float, x2: float, z1: float, z2: float) -> dict:
    return {
        "mode": mode,
        "geometry": {
            "tool": "front_xz",
            "bounds": {
                "min_x": x1,
                "max_x": x2,
                "min_z": z1,
                "max_z": z2,
            },
        },
    }


class VisualizationColorTests(unittest.TestCase):
    def test_lod_cache_version_and_classification_colors_are_current(self) -> None:
        self.assertEqual(pointcloud_lod_service.CLASS_RGB_CACHE_VERSION, "classrgb_v2")
        self.assertEqual(pointcloud_lod_service.CLASSIFICATION_RGB16[0], (2621, 32768, 11796))
        self.assertEqual(pointcloud_lod_service.CLASSIFICATION_RGB16[3], (26214, 65535, 39321))
        self.assertEqual(weight_service.CLASSIFICATION_RGB16[0], pointcloud_lod_service.CLASSIFICATION_RGB16[0])
        self.assertEqual(weight_service.CLASSIFICATION_RGB16[3], pointcloud_lod_service.CLASSIFICATION_RGB16[3])


class WeightGeometryTests(unittest.TestCase):
    def setUp(self) -> None:
        self.points = np.asarray(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 1.0],
                [2.0, 0.0, 2.0],
                [3.0, 0.0, 3.0],
            ],
            dtype=float,
        )

    def test_selection_modes_are_ordered(self) -> None:
        operations = [
            front_operation("new", -0.1, 2.1, -0.1, 2.1),
            front_operation("subtract", 0.9, 1.1, 0.9, 1.1),
            front_operation("add", 2.9, 3.1, 2.9, 3.1),
        ]
        mask = weight_service.evaluate_operations(self.points, operations, "front_xz")
        self.assertEqual(mask.tolist(), [True, False, True, True])

        inverted = weight_service.evaluate_operations(
            self.points,
            operations + [{"mode": "invert", "geometry": {"tool": "front_xz"}}],
            "front_xz",
        )
        self.assertEqual(inverted.tolist(), [False, True, False, False])

    def test_box3d_uses_threejs_column_major_matrix(self) -> None:
        operation = {
            "mode": "new",
            "geometry": {
                "tool": "box3d",
                "view_projection_matrix": np.eye(4).reshape(-1, order="F").tolist(),
                "ndc_rect": {
                    "min_x": -0.1,
                    "max_x": 1.1,
                    "min_y": -0.1,
                    "max_y": 1.1,
                },
            },
        }
        points = np.asarray([[0, 0, 0], [1, 1, 0], [2, 0, 0]], dtype=float)
        mask = weight_service.evaluate_operations(points, [operation])
        self.assertEqual(mask.tolist(), [True, True, False])

    def test_overlap_priority_then_revision(self) -> None:
        labels = np.asarray([16, 16, 22, 22], dtype=int)
        all_points = front_operation("new", -1, 4, -1, 4)
        groups = [
            {
                "group_id": "normal-newer",
                "level": "normal",
                "revision_seq": 99,
                "enabled": True,
                "selection_tool": "front_xz",
                "selection_geometry": {"operations": [all_points]},
            },
            {
                "group_id": "important-older",
                "level": "important",
                "revision_seq": 1,
                "enabled": True,
                "selection_tool": "front_xz",
                "selection_geometry": {"operations": [all_points]},
            },
        ]
        assignments, _, overlap = weight_service.resolve_group_assignments(
            self.points,
            labels,
            groups,
        )
        self.assertEqual(assignments.tolist(), [1, 1, 1, 1])
        self.assertEqual(overlap, 4)

        groups[0]["level"] = "important"
        assignments, _, _ = weight_service.resolve_group_assignments(
            self.points,
            labels,
            groups,
        )
        self.assertEqual(assignments.tolist(), [0, 0, 0, 0])

    def test_priority_cannot_be_overridden_by_large_lower_revision(self) -> None:
        labels = np.asarray([16, 16, 22, 22], dtype=int)
        all_points = front_operation("new", -1, 4, -1, 4)
        levels = (("normal", "needed"), ("needed", "important"), ("normal", "important"))
        for lower, higher in levels:
            groups = [
                {
                    "group_id": "lower",
                    "level": lower,
                    "revision_seq": 10_000_000_000,
                    "enabled": True,
                    "selection_geometry": {"operations": [all_points]},
                },
                {
                    "group_id": "higher",
                    "level": higher,
                    "revision_seq": 1,
                    "enabled": True,
                    "selection_geometry": {"operations": [all_points]},
                },
            ]
            assignments, _, _ = weight_service.resolve_group_assignments(
                self.points,
                labels,
                groups,
            )
            self.assertEqual(assignments.tolist(), [1, 1, 1, 1])

    def test_xz_spatial_sampling_is_deterministic_and_keeps_semantic_quota(self) -> None:
        tower = np.column_stack(
            (
                np.linspace(0.0, 800.0, 800),
                np.zeros(800),
                np.mod(np.arange(800), 40),
            )
        )
        insulator = np.column_stack(
            (
                np.linspace(0.0, 800.0, 800),
                np.ones(800),
                50.0 + np.mod(np.arange(800), 40),
            )
        )
        points = np.vstack((tower, insulator))
        labels = np.asarray([16] * 800 + [22] * 800, dtype=int)

        first = weight_service._editable_sample_indices(points, labels, 1000)
        second = weight_service._editable_sample_indices(points, labels, 1000)

        self.assertEqual(len(first), 1000)
        self.assertTrue(np.array_equal(first, second))
        self.assertEqual(int(np.sum(labels[first] == 16)), 650)
        self.assertEqual(int(np.sum(labels[first] == 22)), 350)
        self.assertLessEqual(float(np.min(points[first, 0])), 2.0)
        self.assertGreaterEqual(float(np.max(points[first, 0])), 798.0)


class WeightPersistenceTests(unittest.TestCase):
    def _make_las(self, path: Path) -> None:
        header = laspy.LasHeader(point_format=3, version="1.2")
        las = laspy.LasData(header)
        las.x = np.asarray([0.0, 1.0, 2.0, 3.0])
        las.y = np.asarray([0.0, 0.0, 0.0, 0.0])
        las.z = np.asarray([0.0, 1.0, 2.0, 3.0])
        las.classification = np.asarray([16, 16, 22, 22], dtype=np.uint8)
        las.write(path)

    def test_draft_apply_and_restore_are_persistent(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            point_cloud = root / "tower.las"
            voxel_dir = root / "voxel"
            voxel_dir.mkdir()
            self._make_las(point_cloud)
            payload = {
                "name": "test",
                "groups": [
                    {
                        "group_id": "g1",
                        "name": "重要",
                        "level": "important",
                        "enabled": True,
                        "selection_tool": "front_xz",
                        "selection_geometry": {
                            "operations": [front_operation("new", -1, 4, -1, 4)]
                        },
                    }
                ],
            }

            with (
                patch.object(weight_service, "find_user_file", return_value=point_cloud),
                patch.object(weight_service, "get_user_dir", return_value=voxel_dir),
            ):
                draft = weight_service.save_profile(
                    "user",
                    point_cloud.name,
                    payload,
                    apply=False,
                )
                self.assertEqual(draft["status"], "draft")
                self.assertFalse(draft["active"])
                self.assertEqual(draft["stats"]["selected_point_count"], 4)

                applied = weight_service.save_profile(
                    "user",
                    point_cloud.name,
                    draft,
                    apply=True,
                )
                self.assertEqual(applied["status"], "applied")
                self.assertTrue(applied["active"])
                self.assertEqual(
                    weight_service.get_active_profile("user", point_cloud.name)["profile_id"],
                    applied["profile_id"],
                )

                restored = weight_service.restore_original("user", point_cloud.name)
                self.assertIn(applied["profile_id"], restored["restored_profile_ids"])
                self.assertIsNone(weight_service.get_active_profile("user", point_cloud.name))

    def test_editable_points_include_context_samples_without_changing_targets(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            point_cloud = root / "context.las"
            voxel_dir = root / "voxel"
            voxel_dir.mkdir()
            header = laspy.LasHeader(point_format=3, version="1.2")
            las = laspy.LasData(header)
            las.x = np.asarray([0, 1, 2, 3, 4, 5], dtype=float)
            las.y = np.zeros(6, dtype=float)
            las.z = np.asarray([0, 1, 2, 3, 4, 5], dtype=float)
            las.classification = np.asarray([16, 22, 0, 3, 0, 3], dtype=np.uint8)
            las.write(point_cloud)

            with (
                patch.object(weight_service, "find_user_file", return_value=point_cloud),
                patch.object(weight_service, "get_user_dir", return_value=voxel_dir),
            ):
                payload = weight_service.editable_points("user", point_cloud.name, limit=1000)

            self.assertEqual(payload["editable_point_count"], 2)
            self.assertEqual(payload["sampled_point_count"], 2)
            self.assertEqual(payload["conductor_count"], 2)
            self.assertEqual(payload["ground_wire_count"], 2)
            self.assertEqual(payload["sampled_conductor_count"], 2)
            self.assertEqual(payload["sampled_ground_wire_count"], 2)
            self.assertEqual(payload["context_labels"], [0, 3, 0, 3])

    def test_profile_status_reports_model_sync_readiness(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            point_cloud = root / "tower.las"
            voxel_dir = root / "voxel"
            voxel_dir.mkdir()
            self._make_las(point_cloud)
            payload = {
                "name": "sync",
                "groups": [
                    {
                        "group_id": "g1",
                        "level": "important",
                        "enabled": True,
                        "selection_tool": "front_xz",
                        "selection_geometry": {
                            "operations": [front_operation("new", -1, 4, -1, 4)]
                        },
                    }
                ],
            }

            with (
                patch.object(weight_service, "find_user_file", return_value=point_cloud),
                patch.object(weight_service, "get_user_dir", return_value=voxel_dir),
            ):
                applied = weight_service.save_profile("user", point_cloud.name, payload, apply=True)
                status = weight_service.profile_status("user", point_cloud.name)
                self.assertTrue(status["weighted"])
                self.assertFalse(status["model_sync"]["ready_for_waypoints"])
                self.assertTrue(status["model_sync"]["voxel_stale"])

                names = weight_service.expected_output_names(point_cloud.name, applied["profile_id"])
                voxel_path = voxel_dir / names["voxel_filename"]
                candidate_path = voxel_dir / names["candidate_filename"]
                voxel_path.write_bytes(b"voxel")
                candidate_path.write_text("[]", encoding="utf-8")
                profile_time = datetime.fromisoformat(applied["updated_at"]).timestamp()
                os.utime(voxel_path, (profile_time + 5, profile_time + 5))
                os.utime(candidate_path, (profile_time + 5, profile_time + 5))

                synced = weight_service.profile_status("user", point_cloud.name)
                self.assertTrue(synced["model_sync"]["ready_for_waypoints"])
                self.assertFalse(synced["model_sync"]["is_stale"])

    def test_weighted_visual_las_uses_weight_classification_codes(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            point_cloud = root / "tower.las"
            voxel_dir = root / "voxel"
            output_las = root / "weighted_visual.las"
            voxel_dir.mkdir()
            self._make_las(point_cloud)
            payload = {
                "name": "visual",
                "groups": [
                    {
                        "group_id": "g-important",
                        "name": "important area",
                        "level": "important",
                        "color": "#ef4444",
                        "enabled": True,
                        "selection_tool": "front_xz",
                        "selection_geometry": {
                            "operations": [front_operation("new", -1, 2.1, -1, 2.1)]
                        },
                    }
                ],
            }

            with (
                patch.object(weight_service, "find_user_file", return_value=point_cloud),
                patch.object(weight_service, "get_user_dir", return_value=voxel_dir),
            ):
                applied = weight_service.save_profile("user", point_cloud.name, payload, apply=True)
                result = weight_service.write_weighted_visual_las(
                    "user",
                    point_cloud.name,
                    output_las,
                    profile_id=applied["profile_id"],
                )

            with laspy.open(output_las) as source:
                visual = source.read()
                classes = np.asarray(visual.classification, dtype=int)

            self.assertEqual(result["assigned_point_count"], 3)
            self.assertEqual(classes.tolist(), [26, 26, 26, 22])


class WeightedModelTests(unittest.TestCase):
    def test_profile_filters_targets_but_preserves_full_safety_points(self) -> None:
        tower = []
        for x in np.linspace(-2.0, 2.0, 9):
            for y in np.linspace(-2.0, 2.0, 9):
                for z in np.linspace(0.0, 8.0, 17):
                    if abs(x) > 1.5 or abs(y) > 1.5:
                        tower.append([x, y, z])
        insulator = [[2.5 + index * 0.08, 0.0, 5.0] for index in range(30)]
        wire = [[x, 4.0, 8.0] for x in np.linspace(-10.0, 10.0, 60)]
        points = np.asarray(tower + insulator + wire, dtype=float)
        labels = np.asarray(
            [16] * len(tower) + [22] * len(insulator) + [0] * len(wire),
            dtype=int,
        )
        profile = {
            "profile_id": "profile123456",
            "name": "half tower",
            "status": "applied",
            "groups": [
                {
                    "group_id": "g1",
                    "name": "right",
                    "level": "important",
                    "color": "#ef4444",
                    "enabled": True,
                    "revision_seq": 1,
                    "selection_tool": "front_xz",
                    "selection_geometry": {
                        "operations": [front_operation("new", 0.0, 5.0, -1.0, 10.0)]
                    },
                }
            ],
            "stats": {},
            "policy": {},
        }

        original = build_semantic_surface_model(points, labels, 0.5)
        weighted = build_semantic_surface_model(points, labels, 0.5, weight_profile=profile)

        self.assertLess(len(weighted["voxels"]), len(original["voxels"]))
        self.assertEqual(len(weighted["safety_points"]), len(original["safety_points"]))
        self.assertEqual(
            weighted["meta"]["active_weight_profile"]["profile_id"],
            profile["profile_id"],
        )
        self.assertTrue(weighted["meta"]["weight_profile_enabled"])
        self.assertTrue(all(record.get("weight_group_id") == "g1" for record in weighted["voxels"]))
        self.assertTrue(
            any(
                not record.get("is_planning_target", True)
                for record in weighted["display_voxels"]
                if record.get("category") in {"tower", "insulator"}
            )
        )


if __name__ == "__main__":
    unittest.main()
