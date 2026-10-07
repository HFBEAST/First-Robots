"""Run the known-cylinder virtual depth coordinate-to-mock-execution golden chain."""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import numpy as np
from experiment_management.run import RunRecorder, sha256_bytes, strict_json_bytes, write_json_pair
from research.protocols.coordinate_transfer_v2 import mock_execute, table_contains_cup
from research.protocols.depth_coordinate_agent import coordinate_message, coordinator_accepts


RID = "EXP-20260923-029-known-cylinder-depth-agent-chain-golden-case"
CFG = ROOT / "research/configs" / f"{RID}.json"
OUT = ROOT / "research/artifacts" / RID
RESULT = ROOT / "research/reports" / f"{RID}.result.json"
EXECUTION_RECORD = ROOT / "research/experiments" / f"{RID}.execution.record.json"


def sha256_file(path: Path) -> str:
    return "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    if OUT.exists() or RESULT.exists() or EXECUTION_RECORD.exists():
        raise FileExistsError("refusing to overwrite formal artifacts, result, or execution record")
    config = json.loads(CFG.read_text(encoding="utf-8"))
    scenario = config["scenario"]
    predecessor = config["blocking_predecessor"]
    predecessor_result = ROOT / predecessor["result_path"]
    if sha256_file(predecessor_result) != predecessor["result_sha256"]:
        raise RuntimeError("frozen 028 result hash mismatch")
    source_result = json.loads(predecessor_result.read_text(encoding="utf-8"))
    fitted_center = np.asarray(source_result["observations"]["fit"]["fitted_center_world_m"], dtype=float)
    required_frames = scenario["required_depth_frames"]
    required_frame_set = set(required_frames)
    scenario_version = scenario["scenario_version"]
    object_model_version = scenario["known_object_model_version"]
    recorder = RunRecorder(
        run_id=RID,
        objective="Run one known-cylinder virtual dual-depth coordinate message through structural coordination, table containment, mock execution, and review.",
        script=Path(__file__),
        repo_root=ROOT,
        output_root=ROOT / "research/runs",
        parameters=scenario,
        random_seed=None,
        inputs=[
            {"kind": "configuration", "path": CFG.relative_to(ROOT).as_posix(), "sha256": sha256_file(CFG)},
            {"kind": "predecessor_result", "path": predecessor_result.relative_to(ROOT).as_posix(), "sha256": sha256_file(predecessor_result)},
        ],
        baseline={"kind": "known_cylinder_depth_center_fit", "reference": predecessor["run_id"]},
        level="formal",
    )
    with recorder:
        OUT.mkdir(parents=True)
        (OUT / "config.json").write_bytes(strict_json_bytes(config))
        source_fit_is_finite = bool(np.isfinite(fitted_center).all())
        positive_message = coordinate_message(fitted_center.tolist(), required_frames, scenario_version, object_model_version)
        coordinator_accepted, coordinator_reason, coordinator_center = coordinator_accepts(
            positive_message, required_frame_set, scenario_version, object_model_version
        )
        table = scenario["table"]
        cup = scenario["cup"]
        capability_accepted = bool(
            coordinator_accepted
            and table_contains_cup(
                coordinator_center,
                np.asarray(table["center_m"], dtype=float),
                np.asarray(table["half_extents_m"], dtype=float),
                cup["radius_m"],
                cup["half_height_m"],
            )
        )
        positive_state = {"cup_center_m": scenario["initial_cup_a_world_m"][:]}
        executor_applied, executor_reason = mock_execute(positive_state, capability_accepted, coordinator_center)
        reviewer_equal = bool(executor_applied and np.allclose(positive_state["cup_center_m"], coordinator_center, rtol=0, atol=1e-12))

        negative_message = coordinate_message(fitted_center.tolist(), required_frames[:1], scenario_version, object_model_version)
        negative_accepted, negative_reason, negative_center = coordinator_accepts(
            negative_message, required_frame_set, scenario_version, object_model_version
        )
        negative_state = {"cup_center_m": scenario["initial_cup_a_world_m"][:]}
        negative_before = negative_state["cup_center_m"][:]
        negative_executor_applied, negative_executor_reason = mock_execute(
            negative_state, bool(negative_accepted), fitted_center if negative_center is None else negative_center
        )
        negative_state_unchanged = negative_state["cup_center_m"] == negative_before
        evaluation_b = np.asarray(scenario["evaluation_only_b_world_m"], dtype=float)
        observations = {
            "source_run": predecessor["run_id"],
            "source_fit_is_finite": source_fit_is_finite,
            "positive_chain": {
                "coordinate_message": positive_message,
                "coordinator": {"accepted": coordinator_accepted, "reason": coordinator_reason},
                "capability_table_containment_only": capability_accepted,
                "executor": {"applied": executor_applied, "reason": executor_reason, "state_after_m": positive_state["cup_center_m"]},
                "reviewer_state_equals_candidate": reviewer_equal,
                "evaluation_only_delta_from_b_m": (fitted_center - evaluation_b).tolist(),
                "evaluation_only_center_error_m": float(np.linalg.norm(fitted_center - evaluation_b)),
            },
            "negative_missing_depth_frame": {
                "coordinate_message": negative_message,
                "coordinator": {"accepted": negative_accepted, "reason": negative_reason},
                "executor": {"applied": negative_executor_applied, "reason": negative_executor_reason},
                "state_unchanged": negative_state_unchanged,
            },
        }
        (OUT / "agent-chain-observations.json").write_bytes(strict_json_bytes(observations))
        passed = bool(
            source_fit_is_finite
            and coordinator_accepted
            and capability_accepted
            and executor_applied
            and reviewer_equal
            and not negative_accepted
            and negative_reason == "required_depth_frames_incomplete"
            and not negative_executor_applied
            and negative_state_unchanged
        )
        result = {
            "schema_version": "first-robots/research-result/v1",
            "run_id": RID,
            "status": "passed_pending_human_review" if passed else "blocked_pending_human_review",
            "observations": observations,
            "limits": config["non_claims"],
            "review": {"status": "pending_human_review", "adopted": False},
        }
        record = {
            "schema_version": "first-robots/experiment-record/v1",
            "run_id": RID,
            "kind": "formal_known_cylinder_depth_agent_chain_golden_case",
            "result": {"path": RESULT.relative_to(ROOT).as_posix(), "sha256": sha256_bytes(strict_json_bytes(result))},
            "review": {"status": "pending_human_review", "adopted": False},
        }
        write_json_pair(first_path=EXECUTION_RECORD, first_content=record, second_path=RESULT, second_content=result)
        for artifact in sorted(OUT.iterdir()):
            recorder.add_artifact(artifact)
        recorder.add_artifact(EXECUTION_RECORD)
        recorder.add_artifact(RESULT)
        recorder.add_metric("known_cylinder_depth_agent_chain_valid", passed, "boolean")
        recorder.complete(
            passed=passed,
            criteria="Frozen known-cylinder fit produces a complete dual-depth message, table-only capability acceptance, mock state transition, reviewer equality, and safe missing-frame rejection.",
            review_status="pending_human_review",
            note="Virtual known-cylinder message and in-memory mock state chain only; no robot action, planning, collision, grasping, safety, or hardware capability claim.",
        )


if __name__ == "__main__":
    main()
