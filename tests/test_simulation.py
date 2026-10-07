from __future__ import annotations

import importlib.util
import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
SIM_AVAILABLE = importlib.util.find_spec("mujoco") is not None and importlib.util.find_spec("numpy") is not None
if SIM_AVAILABLE:
    import mujoco
    import numpy as np
    from first_robots.simulation import command_limits, load_model, penetrating_contacts, simulate_reach, solve_position
    from first_robots.grasping import build_cylinder_scene, robot_limits, solve_side_pose


@unittest.skipUnless(SIM_AVAILABLE, "Install requirements-simulation.txt to verify virtual motion")
class SimulationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.configuration = json.loads((ROOT / "config/sim_reach.json").read_text(encoding="utf-8"))
        self.model_path = ROOT / self.configuration["model"]
        self.model = load_model(self.model_path)

    def test_fixed_target_is_reached_by_integrated_actuator_motion(self) -> None:
        report, trace = simulate_reach(self.model_path, self.configuration)
        self.assertEqual(report["reason"], "motion_completed")
        self.assertTrue(report["ik"]["converged"])
        self.assertGreater(len(trace["time_s"]), 100)
        self.assertGreater(np.linalg.norm(trace["qpos_rad"][-1] - trace["qpos_rad"][0]), 0.1)
        self.assertLess(report["final_position_error_m"], report["initial_position_error_m"] / 10)
        # Actuator command and actual physics state are separately recorded.
        self.assertGreater(float(np.max(np.abs(trace["ctrl_rad"] - trace["qpos_rad"]))), 1e-5)
        limits = command_limits(self.model)
        self.assertTrue(np.all(trace["qpos_rad"] >= limits[:, 0]))
        self.assertTrue(np.all(trace["qpos_rad"] <= limits[:, 1]))

    def test_unreachable_target_exhausts_bounded_solver_without_execution(self) -> None:
        self.configuration["target_world_m"] = [10, 0, 10]
        report, trace = simulate_reach(self.model_path, self.configuration)
        self.assertFalse(report["ik"]["converged"])
        self.assertFalse(report["execution_applied"])
        self.assertLessEqual(report["ik"]["iterations"], self.configuration["ik"]["max_iterations"])
        self.assertEqual(trace, {})

    def test_illegal_initial_joint_and_nan_target_are_rejected(self) -> None:
        for target, initial in (([float("nan"), 0, 0], self.configuration["initial_qpos_rad"]), ([0.25, 0, 0.2], [100, 0, 0, 0, 0, 1])):
            with self.assertRaises(ValueError):
                solve_position(self.model, "gripperframe", target, initial, self.configuration["ik"])

    def test_collision_check_detects_moving_body_floor_penetration(self) -> None:
        model = mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body pos="0 0 .05"><freejoint/><geom name="moving_sphere" type="sphere" size=".1" mass="1"/></body></worldbody></mujoco>')
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        collisions = penetrating_contacts(model, data)
        self.assertEqual(len(collisions), 1)
        self.assertLess(collisions[0]["distance_m"], 0)

    def test_free_cylinder_falls_under_gravity_and_makes_floor_contact(self) -> None:
        configuration = json.loads((ROOT / "config/sim_cylinder_fixture.json").read_text(encoding="utf-8"))
        model, data = build_cylinder_scene(self.model_path, configuration)
        body = model.body("task_cylinder").id
        initial_height = float(data.xpos[body, 2])
        for _ in range(400):
            mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
        self.assertLess(data.xpos[body, 2], initial_height)
        self.assertTrue(any({contact.geom1, contact.geom2} == {model.geom("floor").id, model.geom("task_cylinder_geom").id} for contact in data.contact))
        self.assertAlmostEqual(model.body_mass[body], 0.05)

    def test_side_pose_respects_horizontal_approach_and_joint_limits(self) -> None:
        configuration = json.loads((ROOT / "config/sim_cylinder_fixture.json").read_text(encoding="utf-8"))
        model, _ = build_cylinder_scene(self.model_path, configuration)
        for name in ("pregrasp_site_world_m", "grasp_site_world_m"):
            pose = solve_side_pose(model, configuration[name], configuration["initial_qpos_rad"], configuration["side_pose_ik"])
            self.assertTrue(pose["converged"])
            self.assertLessEqual(pose["weighted_pose_residual_m"], configuration["side_pose_ik"]["numerical_tolerance"])
            limits = robot_limits(model)
            self.assertTrue(np.all(np.asarray(pose["qpos_rad"]) >= limits[:, 0]))
            self.assertTrue(np.all(np.asarray(pose["qpos_rad"]) <= limits[:, 1]))


if __name__ == "__main__":
    unittest.main()
