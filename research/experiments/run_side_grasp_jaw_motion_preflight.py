"""Measure two static candidate-model gripper states; never performs IK or hardware I/O."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
from experiment_management.run import RunRecorder, sha256_bytes, strict_json_bytes, write_json_pair


RID = "EXP-20260921-011-side-grasp-jaw-motion-preflight"
ROOT = Path(__file__).resolve().parents[2]
CFG = ROOT / "research/configs" / f"{RID}.json"
SCENE = ROOT / "research/sources/mujoco_menagerie_robotstudio_so101/robotstudio_so101/scene.xml"
OUT = ROOT / "research/artifacts" / RID
RESULT = ROOT / "research/reports" / f"{RID}.result.json"
EXECUTION_RECORD = ROOT / "research/experiments" / f"{RID}.execution.record.json"


def sha256_file(path: Path) -> str:
    return "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    if OUT.exists() or RESULT.exists() or EXECUTION_RECORD.exists():
        raise FileExistsError("refusing to overwrite formal artifacts, result, or execution record")
    config = json.loads(CFG.read_text(encoding="utf-8"))
    predecessor = ROOT / config["blocking_predecessor"]["result_path"]
    if sha256_file(predecessor) != config["blocking_predecessor"]["result_sha256"]:
        raise RuntimeError("010 predecessor hash mismatch")
    if sha256_file(SCENE) != config["model"]["scene_sha256"]:
        raise RuntimeError("candidate model hash mismatch")
    static = config["static_conditions"]
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    gripper_joint_id = model.joint(static["gripper_joint"]).id
    gripper_qpos_address = model.jnt_qposadr[gripper_joint_id]
    if not np.array_equal(model.jnt_range[gripper_joint_id], static["gripper_joint_limits_rad"]):
        raise RuntimeError("configured gripper limits do not match candidate model")
    if model.nq != len(static["arm_joint_qpos_rad"]) + 1:
        raise RuntimeError("candidate model qpos layout is not the expected six joints")
    fixed_geom_id = model.geom(static["fixed_tip_geom"]).id
    moving_geom_id = model.geom(static["moving_tip_geom"]).id
    recorder = RunRecorder(run_id=RID, objective="Establish a static candidate-model jaw-motion convention before side-grasp target design.", script=Path(__file__), repo_root=ROOT, output_root=ROOT / "research/runs", parameters=static, random_seed=None, inputs=[{"kind": "configuration", "path": CFG.relative_to(ROOT).as_posix(), "sha256": sha256_file(CFG)}, {"kind": "predecessor_result", "path": predecessor.relative_to(ROOT).as_posix(), "sha256": sha256_file(predecessor)}, {"kind": "candidate_model", "path": SCENE.relative_to(ROOT).as_posix(), "sha256": sha256_file(SCENE)}], baseline={"kind": "static_side_grasp_preflight", "reference": config["blocking_predecessor"]["run_id"]}, level="formal")
    with recorder:
        OUT.mkdir(parents=True)
        (OUT / "config.json").write_bytes(strict_json_bytes(config))
        observations = {}
        for state, gripper_qpos in zip(("limit_lower", "limit_upper"), static["gripper_joint_limits_rad"], strict=True):
            data = mujoco.MjData(model)
            data.qpos[:5] = static["arm_joint_qpos_rad"]
            data.qpos[gripper_qpos_address] = gripper_qpos
            mujoco.mj_forward(model, data)
            fixed_position = data.geom_xpos[fixed_geom_id].copy()
            moving_position = data.geom_xpos[moving_geom_id].copy()
            observations[state] = {
                "gripper_qpos_rad": float(gripper_qpos),
                "arm_qpos_rad": data.qpos[:5].tolist(),
                "fixed_tip_world_position_m": fixed_position.tolist(),
                "moving_tip_world_position_m": moving_position.tolist(),
                "representative_tip_separation_m": float(np.linalg.norm(moving_position - fixed_position)),
                "all_named_positions_finite": bool(np.isfinite(np.concatenate((fixed_position, moving_position))).all()),
            }
        lower = observations["limit_lower"]
        upper = observations["limit_upper"]
        moving_tip_displacement = np.subtract(upper["moving_tip_world_position_m"], lower["moving_tip_world_position_m"])
        candidate_open_state = "limit_lower" if lower["representative_tip_separation_m"] > upper["representative_tip_separation_m"] else "limit_upper"
        candidate_closed_state = "limit_upper" if candidate_open_state == "limit_lower" else "limit_lower"
        passed = all(item["arm_qpos_rad"] == static["arm_joint_qpos_rad"] and item["all_named_positions_finite"] for item in observations.values()) and lower["representative_tip_separation_m"] != upper["representative_tip_separation_m"]
        assessment = {"static_gripper_states": observations, "moving_tip_displacement_lower_to_upper_m": moving_tip_displacement.tolist(), "candidate_open_state_by_representative_separation": candidate_open_state, "candidate_closed_state_by_representative_separation": candidate_closed_state, "side_grasp_target_pose": "not_defined", "grasp_reachability_assessment": "not_evaluated_no_deterministic_ik_collision_protocol"}
        (OUT / "terminal-assessment.json").write_bytes(strict_json_bytes(assessment))
        result = {"schema_version": "first-robots/research-result/v1", "run_id": RID, "status": "passed_pending_human_review" if passed else "blocked_pending_human_review", "observations": assessment, "limits": config["non_claims"], "review": {"status": "pending_human_review", "adopted": False}}
        execution_record = {"schema_version": "first-robots/experiment-record/v1", "run_id": RID, "kind": "formal_side_grasp_jaw_motion_preflight", "result": {"path": RESULT.relative_to(ROOT).as_posix(), "sha256": sha256_bytes(strict_json_bytes(result))}, "review": {"status": "pending_human_review", "adopted": False}}
        write_json_pair(first_path=EXECUTION_RECORD, first_content=execution_record, second_path=RESULT, second_content=result)
        for artifact in sorted(OUT.iterdir()):
            recorder.add_artifact(artifact)
        recorder.add_artifact(EXECUTION_RECORD)
        recorder.add_artifact(RESULT)
        recorder.add_metric("representative_tip_separation_lower", lower["representative_tip_separation_m"], "m")
        recorder.add_metric("representative_tip_separation_upper", upper["representative_tip_separation_m"], "m")
        recorder.complete(passed=passed, criteria="Arm state is fixed, named tip positions are finite, and representative tip separation changes across gripper limits.", review_status="pending_human_review", note="Static jaw-motion convention only; no target pose, IK, collision, grasp, control, or hardware evidence.")


if __name__ == "__main__":
    main()
