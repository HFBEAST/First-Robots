from __future__ import annotations

import importlib.util
import json
import sys
import unittest
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
AVAILABLE = importlib.util.find_spec("mujoco") is not None and importlib.util.find_spec("numpy") is not None
if AVAILABLE:
    import mujoco
    import numpy as np
    from first_robots.depth import CameraCalibration, DepthFrame, MujocoDepthFrameSource, add_fixed_cameras, calibration_from_model, project_world_points, unproject_pixels
    from first_robots.grasping import build_cylinder_scene


@unittest.skipUnless(AVAILABLE, "Install requirements-simulation.txt to verify depth contracts")
class DepthTests(unittest.TestCase):
    def setUp(self) -> None:
        self.calibration = CameraCalibration("depth", "v1", 3, 3, 1, 1, 1, 1, np.array([10, 20, 30]), np.eye(3))
        self.frame = DepthFrame("depth", "v1", "synthetic_test", np.full((3, 3), 2, dtype=np.float32))

    def test_off_axis_depth_is_forward_distance_not_normalized_ray_length(self) -> None:
        points = unproject_pixels(self.frame, self.calibration, np.array([[2, 1], [1, 1], [1, 2]]))
        np.testing.assert_array_equal(points, [[12, 20, 32], [10, 20, 32], [10, 22, 32]])

    def test_round_trip_uses_world_translation_rotation_and_pixel_centers(self) -> None:
        calibration = replace(self.calibration, world_from_optical=np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]))
        pixels = np.array([[2, 1], [1, 2]])
        world = unproject_pixels(self.frame, calibration, pixels)
        np.testing.assert_array_equal(world, [[10, 22, 32], [8, 20, 32]])
        recovered, depth = project_world_points(world, calibration)
        np.testing.assert_array_equal(recovered, pixels)
        np.testing.assert_array_equal(depth, [2, 2])

    def test_wrong_frame_version_units_dimensions_dtype_and_invalid_samples_reject(self) -> None:
        alternatives = [replace(self.frame, frame_id="other"), replace(self.frame, scene_version="v0"), replace(self.frame, encoding="uint16_mm"), replace(self.frame, depth_m=np.ones((2, 2), dtype=np.float32)), replace(self.frame, depth_m=np.ones((3, 3), dtype=np.uint16))]
        for value in (0, float("nan"), float("inf"), -1):
            alternatives.append(replace(self.frame, depth_m=np.full((3, 3), value, dtype=np.float32)))
        for frame in alternatives:
            with self.assertRaises(ValueError):
                unproject_pixels(frame, self.calibration, np.array([[1, 1]]))
        with self.assertRaises(ValueError):
            replace(self.calibration, world_from_optical=np.diag([1, 1, -1])).validate()
        with self.assertRaises(ValueError):
            unproject_pixels(self.frame, self.calibration, np.array([[3, 0]]))

    def test_actual_plane_render_validates_depth_semantics_independently(self) -> None:
        configuration = json.loads((ROOT / "config/sim_depth_observation.json").read_text())
        spec = mujoco.MjSpec.from_string('<mujoco><worldbody><geom type="plane" size="5 5 .1"/></worldbody></mujoco>')
        add_fixed_cameras(spec, configuration)
        model = spec.compile()
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        source = MujocoDepthFrameSource(model, data, configuration)
        for name in configuration["cameras"]:
            frame = source.capture(name)
            calibration = calibration_from_model(model, data, name, configuration)
            pixels = np.asarray(configuration["plane_check_pixels_xy"])
            world = unproject_pixels(frame, calibration, pixels)
            budget = 32 * np.finfo(np.float32).eps * max(1, float(np.max(frame.depth_m[pixels[:, 1], pixels[:, 0]])))
            self.assertLessEqual(float(np.max(np.abs(world[:, 2]))), budget)
        with self.assertRaises(ValueError):
            source.capture("unregistered")
        model.vis.quality.offsamples = 4
        with self.assertRaises(ValueError):
            MujocoDepthFrameSource(model, data, configuration)

    def test_adding_fixed_cameras_does_not_change_settled_contact_dynamics(self) -> None:
        observation = json.loads((ROOT / "config/sim_depth_observation.json").read_text())
        task = json.loads((ROOT / observation["task_configuration"]).read_text())
        path = ROOT / task["model"]
        base_model, base_data = build_cylinder_scene(path, task)
        spec = mujoco.MjSpec.from_file(str(path))
        add_fixed_cameras(spec, observation)
        model, data = build_cylinder_scene(path, task, scene_spec=spec)
        for _ in range(400):
            mujoco.mj_step(base_model, base_data)
            mujoco.mj_step(model, data)
        np.testing.assert_array_equal(data.qpos, base_data.qpos)
        np.testing.assert_array_equal(data.qvel, base_data.qvel)


if __name__ == "__main__":
    unittest.main()
