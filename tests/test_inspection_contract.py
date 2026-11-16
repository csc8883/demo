import unittest
from types import SimpleNamespace
import numpy as np
from backend.services.route_service import normalize_waypoints
from waypoint_planning.planning_core import camera_basis, compute_view_geometry, view_fov, VisibilityEngine


class InspectionContractTests(unittest.TestCase):
    def test_continuous_focal_projection_matches_preview_and_controls_visibility(self):
        h, v = view_fov(35)
        self.assertAlmostEqual(h, 54.4322231146)
        self.assertAlmostEqual(v, 37.8492888321)
        env = SimpleNamespace(target_count=1, target_coords=np.array([[6., 10., 0.]]),
                              normal_hints=np.array([[-6., -10., 0.]]) / np.sqrt(136),
                              incidence_max=np.array([90.]), required_resolution=np.array([0.]))
        engine = VisibilityEngine(env)
        wide = engine.visible_indices_for_view([0, 0, 0], 0, 0, focal_level='F0', f_eq_mm=24)
        tele = engine.visible_indices_for_view([0, 0, 0], 0, 0, focal_level='F0', f_eq_mm=84)
        self.assertEqual(wide.tolist(), [0])
        self.assertEqual(tele.tolist(), [])

    def test_normalization_preserves_identity_target_and_camera(self):
        raw = [{"id": 42, "position": [652510, 3423012, 100], "actionName": "photo",
                "shots": [{"shot_id": "s7", "heading": 90, "pitch": -30,
                           "focal_length_eq_mm": 84, "look_at": [652520, 3423012, 90],
                           "instance_id": 3, "target_id": 9, "component_id": "insulator:instance:3"}]}]
        out = normalize_waypoints(raw)[0]
        self.assertEqual(out['id'], 42)
        self.assertEqual(out['waypoint_id'], '42')
        self.assertEqual(out['action'], 'photo')
        self.assertEqual(out['point_type'], 'task')
        self.assertEqual(out['shots'][0]['look_at'], raw[0]['shots'][0]['look_at'])
        self.assertEqual(out['shots'][0]['component_id'], 'insulator:instance:3')
        self.assertEqual(out['f_eq_mm'], 84)
        self.assertNotIn('yaw', raw[0]['shots'][0])

    def test_known_world_targets_match_camera_forward(self):
        p = np.array([652510.25, 3423012.5, 100])
        for delta in ([0, 10, 0], [10, 0, 0], [0, -10, 0], [-10, 0, 0], [0, 0, -10]):
            target = p + delta
            g = compute_view_geometry(p, target)
            forward, right, up = camera_basis(g['heading'], g['pitch'])
            np.testing.assert_allclose(forward * g['Distance'], delta, atol=1e-7)
            np.testing.assert_allclose(np.cross(right, forward), up, atol=1e-7)
        forward, right, up = camera_basis(90, -90)
        np.testing.assert_allclose(right, [0, -1, 0], atol=1e-7)


if __name__ == '__main__':
    unittest.main()
