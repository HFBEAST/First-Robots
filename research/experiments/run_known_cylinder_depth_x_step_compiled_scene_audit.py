"""Audit the compiled virtual conditions omitted from the 030 pass predicate."""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import mujoco
import numpy as np
from experiment_management.run import RunRecorder, sha256_bytes, strict_json_bytes, write_json_pair


RID = "EXP-20260923-031-known-cylinder-depth-x-step-compiled-scene-audit"
CFG = ROOT / "research/configs" / f"{RID}.json"
SCENE = ROOT / "research/sources/mujoco_menagerie_robotstudio_so101/robotstudio_so101/scene.xml"
OUT = ROOT / "research/artifacts" / RID
RESULT = ROOT / "research/reports" / f"{RID}.result.json"
EXECUTION_RECORD = ROOT / "research/experiments" / f"{RID}.execution.record.json"


def sha256_file(path: Path) -> str:
    return "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()


def look_at_quaternion(position: np.ndarray, target: np.ndarray) -> list[float]:
    z = -(target - position)
    z /= np.linalg.norm(z)
    x = np.cross(np.array([0.0, 0.0, 1.0]), z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    output = np.empty(4)
    mujoco.mju_mat2Quat(output, np.column_stack((x, y, z)).reshape(9))
    return output.tolist()


def compile_condition(scenario: dict, cup_center: list[float]) -> tuple[mujoco.MjModel, mujoco.MjData]:
    spec = mujoco.MjSpec.from_file(str(SCENE))
    table = spec.worldbody.add_geom()
    table.name = "vlab_table"
    table.type = mujoco.mjtGeom.mjGEOM_BOX
    table.pos = scenario["table"]["center_m"]
    table.size = scenario["table"]["half_extents_m"]
    cup = spec.worldbody.add_geom()
    cup.name = "vlab_cup_b"
    cup.type = mujoco.mjtGeom.mjGEOM_CYLINDER
    cup.pos = cup_center
    cup.size = [scenario["cup"]["radius_m"], scenario["cup"]["half_height_m"], 0]
    for name, camera in scenario["fixed_cameras"].items():
        view = spec.worldbody.add_camera()
        view.name = name
        view.pos = camera["position_m"]
        view.quat = look_at_quaternion(np.asarray(camera["position_m"]), np.asarray(camera["target_m"]))
        view.resolution = scenario["camera_resolution_px"]
        view.fovy = scenario["camera_fovy_degrees"]
    model = spec.compile()
    data = mujoco.MjData(model)
    data.qpos[:] = scenario["robot_qpos_rad"]
    mujoco.mj_forward(model, data)
    return model, data


def main() -> None:
    if OUT.exists() or RESULT.exists() or EXECUTION_RECORD.exists():
        raise FileExistsError("refusing to overwrite formal artifacts, result, or execution record")
    config = json.loads(CFG.read_text(encoding="utf-8"))
    scenario = config["scenario"]
    predecessor = config["blocking_predecessor"]
    predecessor_result = ROOT / predecessor["result_path"]
    if sha256_file(predecessor_result) != predecessor["result_sha256"] or sha256_file(SCENE) != config["model"]["scene_sha256"]:
        raise RuntimeError("frozen predecessor or model hash mismatch")
    predecessor_payload = json.loads(predecessor_result.read_text(encoding="utf-8"))
    recorder = RunRecorder(
        run_id=RID,
        objective="Repair the 030 omission by making its compiled virtual geometry and frozen controls explicit pass-predicate conditions.",
        script=Path(__file__), repo_root=ROOT, output_root=ROOT / "research/runs", parameters=scenario, random_seed=None,
        inputs=[
            {"kind": "configuration", "path": CFG.relative_to(ROOT).as_posix(), "sha256": sha256_file(CFG)},
            {"kind": "predecessor_result", "path": predecessor_result.relative_to(ROOT).as_posix(), "sha256": sha256_file(predecessor_result)},
            {"kind": "candidate_model", "path": SCENE.relative_to(ROOT).as_posix(), "sha256": sha256_file(SCENE)},
        ],
        baseline={"kind": "x_step_agent_chain_missing_compiled_gate", "reference": predecessor["run_id"]}, level="formal",
    )
    with recorder:
        OUT.mkdir(parents=True)
        (OUT / "config.json").write_bytes(strict_json_bytes(config))
        observations = []
        predecessor_conditions = predecessor_payload["observations"]["conditions"]
        for x, predecessor_condition in zip(scenario["x_values_m"], predecessor_conditions, strict=True):
            expected_cup = np.array([x, scenario["frozen_y_m"], scenario["frozen_center_z_m"]], dtype=float)
            model, data = compile_condition(scenario, expected_cup.tolist())
            table = model.geom("vlab_table")
            cup = model.geom("vlab_cup_b")
            cameras = {}
            for name, camera in scenario["fixed_cameras"].items():
                expected_quat = np.asarray(look_at_quaternion(np.asarray(camera["position_m"]), np.asarray(camera["target_m"])))
                compiled = model.camera(name)
                cameras[name] = {
                    "position_matches": bool(np.allclose(model.cam_pos[compiled.id], camera["position_m"], rtol=0, atol=1e-12)),
                    "orientation_matches": bool(np.allclose(model.cam_quat[compiled.id], expected_quat, rtol=0, atol=1e-12)),
                    "fovy_matches": bool(np.isclose(model.cam_fovy[compiled.id], scenario["camera_fovy_degrees"], rtol=0, atol=1e-12)),
                }
            observations.append({
                "declared_x_m": x,
                "compiled_cup_center_world_m": data.geom_xpos[cup.id].tolist(),
                "cup_center_matches": bool(np.allclose(data.geom_xpos[cup.id], expected_cup, rtol=0, atol=1e-12)),
                "table_center_matches": bool(np.allclose(data.geom_xpos[table.id], scenario["table"]["center_m"], rtol=0, atol=1e-12)),
                "table_half_extents_matches": bool(np.allclose(model.geom_size[table.id, :3], scenario["table"]["half_extents_m"], rtol=0, atol=1e-12)),
                "robot_qpos_matches": bool(np.allclose(data.qpos, scenario["robot_qpos_rad"], rtol=0, atol=1e-12)),
                "barrier_absent": all(model.geom(index).name != "vlab_barrier" for index in range(model.ngeom)),
                "cameras": cameras,
                "predecessor_recorded_compiled_cup_match": predecessor_condition["compiled_cup_center_matches_configuration"],
            })
        compiled_centers = np.asarray([item["compiled_cup_center_world_m"] for item in observations])
        only_x_varies = bool(np.allclose(compiled_centers[:, 0], scenario["x_values_m"], rtol=0, atol=1e-12) and np.allclose(compiled_centers[:, 1], scenario["frozen_y_m"], rtol=0, atol=1e-12) and np.allclose(compiled_centers[:, 2], scenario["frozen_center_z_m"], rtol=0, atol=1e-12))
        all_conditions_valid = bool(all(item["cup_center_matches"] and item["table_center_matches"] and item["table_half_extents_matches"] and item["robot_qpos_matches"] and item["barrier_absent"] and item["predecessor_recorded_compiled_cup_match"] and all(value for camera in item["cameras"].values() for value in camera.values()) for item in observations))
        payload = {"source_run": predecessor["run_id"], "only_x_varies_in_compiled_scene": only_x_varies, "all_compiled_conditions_valid": all_conditions_valid, "conditions": observations}
        (OUT / "compiled-scene-audit-observations.json").write_bytes(strict_json_bytes(payload))
        passed = bool(only_x_varies and all_conditions_valid)
        result = {"schema_version": "first-robots/research-result/v1", "run_id": RID, "status": "passed_pending_human_review" if passed else "blocked_pending_human_review", "observations": payload, "limits": config["non_claims"], "review": {"status": "pending_human_review", "adopted": False}}
        record = {"schema_version": "first-robots/experiment-record/v1", "run_id": RID, "kind": "formal_known_cylinder_depth_x_step_compiled_scene_audit", "result": {"path": RESULT.relative_to(ROOT).as_posix(), "sha256": sha256_bytes(strict_json_bytes(result))}, "review": {"status": "pending_human_review", "adopted": False}}
        write_json_pair(first_path=EXECUTION_RECORD, first_content=record, second_path=RESULT, second_content=result)
        for artifact in sorted(OUT.iterdir()):
            recorder.add_artifact(artifact)
        recorder.add_artifact(EXECUTION_RECORD)
        recorder.add_artifact(RESULT)
        recorder.add_metric("compiled_x_step_conditions_valid", passed, "boolean")
        recorder.complete(passed=passed, criteria="Every declared X condition compiles with the frozen table, cameras, robot state and cup geometry, and compiled cup centers vary only in the declared X sequence.", review_status="pending_human_review", note="Configuration audit only; it repairs the 030 pass-predicate omission without rerunning or broadening perception, agent-chain, or robot capability claims.")


if __name__ == "__main__":
    main()
