"""Record the free-cylinder physical fixture and bounded side-pose prerequisites."""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

import mujoco
import numpy as np
from PIL import Image
from experiment_management.run import RunRecorder, sha256_bytes, strict_json_bytes, write_json_pair
from first_robots.grasping import build_cylinder_scene, solve_side_pose


def input_file(path: Path) -> dict:
    return {"kind": "frozen_input", "path": path.relative_to(ROOT).as_posix(), "sha256": "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    experiment_path = ROOT / parser.parse_args().config
    experiment = json.loads(experiment_path.read_text(encoding="utf-8"))
    runtime_path = ROOT / experiment["runtime_configuration"]
    configuration = json.loads(runtime_path.read_text(encoding="utf-8"))
    predecessor = ROOT / experiment["predecessor"]
    if json.loads(predecessor.read_text(encoding="utf-8"))["status"] != "passed_pending_human_review":
        raise RuntimeError("Motion baseline has not passed its technical criteria")
    rid = experiment["experiment_id"]
    output = ROOT / "research/artifacts" / rid
    result_path = ROOT / "research/reports" / f"{rid}.result.json"
    record_path = ROOT / "research/experiments" / f"{rid}.execution.record.json"
    if any(path.exists() for path in (output, result_path, record_path, ROOT / "research/runs" / f"{rid}.json")):
        raise FileExistsError("Refusing to overwrite formal fixture run")
    model_path = ROOT / configuration["model"]
    inputs = [input_file(path) for path in (experiment_path, runtime_path, Path(__file__), predecessor, ROOT / "src/first_robots/grasping.py", ROOT / "src/first_robots/simulation.py")]
    inputs.extend(input_file(path) for path in sorted(model_path.parent.rglob("*")) if path.is_file())
    with RunRecorder(run_id=rid, objective="Verify a gravity/contact free-cylinder fixture and side-grasp pose constraints before grasp execution.", script=Path(__file__), repo_root=ROOT, output_root=ROOT / "research/runs", parameters=configuration, random_seed=None, inputs=inputs, baseline={"kind": "committed_single_arm_motion", "reference": "EXP-20261007-003-single-arm-reach-committed-baseline"}, level="formal") as recorder:
        output.mkdir(parents=True)
        (output / "config.json").write_bytes(strict_json_bytes(configuration))
        model, data = build_cylinder_scene(model_path, configuration)
        cylinder_body = model.body("task_cylinder").id
        cylinder_geom = model.geom("task_cylinder_geom").id
        floor_geom = model.geom("floor").id
        cylinder_joint = model.joint("cylinder_free").id
        initial_center = data.xpos[cylinder_body].copy()
        trace = {name: [] for name in ("time_s", "qpos", "qvel", "cylinder_center_m")}
        for step in range(int(np.ceil(configuration["fixture_settle_s"] / model.opt.timestep)) + 1):
            mujoco.mj_forward(model, data)
            trace["time_s"].append(float(data.time))
            trace["qpos"].append(data.qpos.copy())
            trace["qvel"].append(data.qvel.copy())
            trace["cylinder_center_m"].append(data.xpos[cylinder_body].copy())
            if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
                break
            if step < int(np.ceil(configuration["fixture_settle_s"] / model.opt.timestep)):
                mujoco.mj_step(model, data)
        normal_support_force = 0.0
        for index, contact in enumerate(data.contact):
            if {contact.geom1, contact.geom2} == {cylinder_geom, floor_geom}:
                force = np.zeros(6)
                mujoco.mj_contactForce(model, data, index, force)
                normal_support_force += float(force[0])
        side_poses = {name: solve_side_pose(model, configuration[name], configuration["initial_qpos_rad"], configuration["side_pose_ik"]) for name in ("pregrasp_site_world_m", "grasp_site_world_m")}
        actual_duration = float(data.time)
        final_center = data.xpos[cylinder_body].copy()
        trace_arrays = {name: np.asarray(values) for name, values in trace.items()}
        np.savez_compressed(output / "fixture-trajectory.npz", **trace_arrays)
        free_body_valid = bool(model.jnt_type[cylinder_joint] == mujoco.mjtJoint.mjJNT_FREE and model.body_mass[cylinder_body] > 0 and model.geom_contype[cylinder_geom] != 0 and model.geom_conaffinity[cylinder_geom] != 0)
        finite = bool(all(np.isfinite(value).all() for value in trace_arrays.values()))
        poses_converged = bool(all(pose["converged"] for pose in side_poses.values()))
        observation = {"free_cylinder_positive_mass_contact_enabled": free_body_valid, "initial_center_world_m": initial_center.tolist(), "final_center_world_m": final_center.tolist(), "final_free_joint_velocity": data.qvel[6:].tolist(), "final_normal_support_force_n": normal_support_force, "finite_fixture_trajectory": finite, "actual_duration_s": actual_duration, "side_poses": side_poses, "grasp_execution": "not_evaluated"}
        camera = mujoco.MjvCamera()
        camera.lookat[:] = [0.2, 0, 0.15]
        camera.distance = 0.8
        camera.azimuth = 135
        camera.elevation = -25
        if poses_converged:
            data.qpos[:6] = side_poses["grasp_site_world_m"]["qpos_rad"]
            mujoco.mj_forward(model, data)
        with mujoco.Renderer(model, height=360, width=480) as renderer:
            renderer.update_scene(data, camera)
            Image.fromarray(renderer.render()).save(output / "side-pose-preview.png")
        passed = bool(free_body_valid and finite and final_center[2] < initial_center[2] and normal_support_force > 0 and poses_converged and actual_duration >= configuration["fixture_settle_s"] - model.opt.timestep)
        result = {"schema_version": "first-robots/research-result/v1", "run_id": rid, "status": "passed_pending_human_review" if passed else "blocked_pending_human_review", "observations": observation, "review": {"status": "pending_human_review", "adopted": False}}
        record = {"schema_version": "first-robots/experiment-record/v1", "run_id": rid, "kind": "formal_cylinder_fixture_side_pose", "result": {"path": result_path.relative_to(ROOT).as_posix(), "sha256": sha256_bytes(strict_json_bytes(result))}, "review": {"status": "pending_human_review", "adopted": False}}
        write_json_pair(first_path=record_path, first_content=record, second_path=result_path, second_content=result)
        for path in sorted(output.iterdir()):
            recorder.add_artifact(path)
        recorder.add_artifact(record_path)
        recorder.add_artifact(result_path)
        recorder.add_metric("free_cylinder_side_pose_prerequisites_valid", passed, "boolean")
        recorder.complete(passed=passed, criteria="Finite free-cylinder dynamics lower the initially elevated cylinder onto positive-force support; both bounded side poses converge.", note="Idealized virtual fixture and pose prerequisites only; no grasp execution or physical capability adoption. Preview is a kinematic illustration of the solved pose, not a motion replay.")
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
