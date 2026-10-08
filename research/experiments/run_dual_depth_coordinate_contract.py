"""Verify current-scene depth coordinates without center estimation or execution."""
from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import sys
from dataclasses import asdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

import mujoco
import numpy as np
from PIL import Image
from experiment_management.run import RunRecorder, sha256_bytes, strict_json_bytes, write_json_pair
from first_robots.depth import MujocoDepthFrameSource, add_fixed_cameras, calibration_from_model, project_world_points, unproject_pixels
from first_robots.grasping import build_cylinder_scene, cylinder_contacts


def input_file(path: Path) -> dict:
    return {"kind": "frozen_input", "path": path.relative_to(ROOT).as_posix(), "sha256": "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    experiment_path = ROOT / parser.parse_args().config
    experiment = json.loads(experiment_path.read_text(encoding="utf-8"))
    observation_path = ROOT / experiment["observation_configuration"]
    observation = json.loads(observation_path.read_text(encoding="utf-8"))
    task_path = ROOT / observation["task_configuration"]
    task = json.loads(task_path.read_text(encoding="utf-8"))
    predecessor_path = ROOT / experiment["predecessor"]
    predecessor = json.loads(predecessor_path.read_text(encoding="utf-8"))
    if predecessor["status"] != "passed_pending_human_review" or not predecessor["observations"][0]["repeat_trace_comparison"]["identical"]:
        raise RuntimeError("Repeated contact-task prerequisite has not passed")
    rid = experiment["experiment_id"]
    output = ROOT / "research/artifacts" / rid
    result_path = ROOT / "research/reports" / f"{rid}.result.json"
    record_path = ROOT / "research/experiments" / f"{rid}.execution.record.json"
    if any(path.exists() for path in (output, result_path, record_path, ROOT / "research/runs" / f"{rid}.json")):
        raise FileExistsError("Refusing to overwrite coordinate-contract evidence")
    model_path = ROOT / task["model"]
    inputs = [input_file(path) for path in (experiment_path, observation_path, task_path, ROOT / experiment["plan"], predecessor_path, Path(__file__), ROOT / "src/first_robots/depth.py", ROOT / "src/first_robots/grasping.py", ROOT / "src/first_robots/simulation.py", ROOT / "requirements-simulation.txt")]
    inputs.extend(input_file(path) for path in sorted(model_path.parent.rglob("*")) if path.is_file())
    with RunRecorder(run_id=rid, objective="Verify two fixed virtual metric-depth sources and optical-to-world coordinates before estimating a cylinder center.", script=Path(__file__), repo_root=ROOT, output_root=ROOT / "research/runs", parameters={"task": task, "observation": observation}, random_seed=None, inputs=inputs, baseline={"kind": "repeated_contact_task", "reference": predecessor["run_id"]}, level="formal") as recorder:
        recorder.manifest["runtime"]["packages"] = {name: importlib.metadata.version(name) for name in ("mujoco", "numpy", "Pillow", "experiment-management")}
        output.mkdir(parents=True)
        (output / "config.json").write_bytes(strict_json_bytes({"task": task, "observation": observation}))
        base_model, base_data = build_cylinder_scene(model_path, task)
        spec = mujoco.MjSpec.from_file(str(model_path))
        add_fixed_cameras(spec, observation)
        model, data = build_cylinder_scene(model_path, task, scene_spec=spec)
        for _ in range(int(np.ceil(task["fixture_settle_s"] / model.opt.timestep))):
            mujoco.mj_step(base_model, base_data)
            mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
        physics_identical = bool(np.array_equal(data.qpos, base_data.qpos) and np.array_equal(data.qvel, base_data.qvel))
        np.savez_compressed(output / "settled-state.npz", qpos=data.qpos, qvel=data.qvel, base_qpos=base_data.qpos, base_qvel=base_data.qvel)
        plane_spec = mujoco.MjSpec.from_string('<mujoco><worldbody><geom name="calibration_plane" type="plane" size="5 5 .1"/></worldbody></mujoco>')
        add_fixed_cameras(plane_spec, observation)
        plane_model = plane_spec.compile()
        plane_data = mujoco.MjData(plane_model)
        mujoco.mj_forward(plane_model, plane_data)
        source = MujocoDepthFrameSource(model, data, observation)
        plane_source = MujocoDepthFrameSource(plane_model, plane_data, observation)
        cameras = {}
        pixels = np.asarray(observation["plane_check_pixels_xy"])
        for name, settings in observation["cameras"].items():
            frame = source.capture(name)
            calibration = calibration_from_model(model, data, name, observation)
            np.save(output / f"{name}-depth.npy", frame.depth_m)
            with mujoco.Renderer(model, height=observation["height"], width=observation["width"]) as renderer:
                renderer.update_scene(data, camera=name)
                Image.fromarray(renderer.render()).save(output / f"{name}-context-rgb.png")
            serialized = {key: value.tolist() if isinstance(value, np.ndarray) else value for key, value in asdict(calibration).items()}
            serialized.update(optical_axes="x_right_y_down_z_forward", depth_semantics="forward_distance_m", source="compiled_nominal_virtual_camera")
            (output / f"{name}-calibration.json").write_bytes(strict_json_bytes(serialized))
            plane_frame = plane_source.capture(name)
            plane_calibration = calibration_from_model(plane_model, plane_data, name, observation)
            np.save(output / f"{name}-plane-depth.npy", plane_frame.depth_m)
            points = unproject_pixels(plane_frame, plane_calibration, pixels)
            recovered_pixels, recovered_depth = project_world_points(points, plane_calibration)
            sampled_depth = plane_frame.depth_m[pixels[:, 1], pixels[:, 0]].astype(float)
            budget = float(32 * np.finfo(np.float32).eps * max(1, float(np.max(sampled_depth))))
            plane_residual = float(np.max(np.abs(points[:, 2])))
            round_trip_valid = bool(np.allclose(recovered_pixels, pixels, rtol=0, atol=1e-12) and np.allclose(recovered_depth, sampled_depth, rtol=0, atol=1e-12))
            frame_valid = bool(frame.depth_m.shape == (observation["height"], observation["width"]) and frame.depth_m.dtype == np.float32 and np.isfinite(frame.depth_m).all() and np.all(frame.depth_m > 0) and np.ptp(frame.depth_m) > 0)
            camera_position_valid = bool(np.array_equal(calibration.position_world_m, settings["position_world_m"]))
            forward = calibration.world_from_optical[:, 2]
            target_direction = np.asarray(settings["target_world_m"]) - calibration.position_world_m
            orientation_valid = bool(np.allclose(forward, target_direction / np.linalg.norm(target_direction), rtol=0, atol=1e-12))
            cameras[name] = {"frame_valid": frame_valid, "compiled_position_matches_configuration": camera_position_valid, "compiled_orientation_matches_configuration": orientation_valid, "minimum_depth_m": float(np.min(frame.depth_m)), "maximum_depth_m": float(np.max(frame.depth_m)), "plane_pixels_xy": pixels.tolist(), "plane_sampled_depth_m": sampled_depth.tolist(), "plane_points_world_m": points.tolist(), "plane_maximum_height_residual_m": plane_residual, "plane_numerical_budget_m": budget, "plane_check_passed": plane_residual <= budget, "round_trip_valid": round_trip_valid}
        contacts = cylinder_contacts(model, data)
        fixture_valid = bool(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all() and contacts["floor_support_force_n"] > 0 and not contacts["forbidden_penetrations"])
        passed = bool(physics_identical and fixture_valid and len(cameras) == 2 and all(all(item[key] for key in ("frame_valid", "compiled_position_matches_configuration", "compiled_orientation_matches_configuration", "plane_check_passed", "round_trip_valid")) for item in cameras.values()))
        observations = {"settled_physics_identical_to_base": physics_identical, "fixture_valid": fixture_valid, "offsamples": int(model.vis.quality.offsamples), "cylinder_center_world_m_evaluation_only": data.xpos[model.body("task_cylinder").id].tolist(), "contacts": contacts, "cameras": cameras, "center_estimation": "not_evaluated", "vision_driven_execution": "not_evaluated", "wrist_rgb": "not_evaluated"}
        result = {"schema_version": "first-robots/research-result/v1", "run_id": rid, "status": "passed_pending_human_review" if passed else "blocked_pending_human_review", "observations": observations, "review": {"status": "pending_human_review", "adopted": False}}
        record = {"schema_version": "first-robots/experiment-record/v1", "run_id": rid, "kind": "formal_dual_depth_coordinate_contract", "result": {"path": result_path.relative_to(ROOT).as_posix(), "sha256": sha256_bytes(strict_json_bytes(result))}, "review": {"status": "pending_human_review", "adopted": False}}
        write_json_pair(first_path=record_path, first_content=record, second_path=result_path, second_content=result)
        for path in sorted(output.iterdir()):
            recorder.add_artifact(path)
        recorder.add_artifact(record_path)
        recorder.add_artifact(result_path)
        recorder.add_metric("dual_depth_coordinate_prerequisites_valid", passed, "boolean")
        recorder.complete(passed=passed, criteria="Both frame contracts, compiled poses, independent plane depth reconstruction, round trips and unchanged settled dynamics pass.", note="Coordinate prerequisite only; no object detection, center fit, camera-driven grasp, hardware or capability adoption. Null seed deterministic exception documented in plan.")
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
