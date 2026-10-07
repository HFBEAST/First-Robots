"""Record a fixed-target motion experiment through the public simulation API."""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

import numpy as np
from PIL import Image, ImageDraw
from experiment_management.run import RunRecorder, sha256_bytes, strict_json_bytes, write_json_pair
from first_robots.simulation import render_reach_replay, simulate_reach


def file_input(path: Path) -> dict:
    return {"kind": "frozen_input", "path": path.relative_to(ROOT).as_posix(), "sha256": "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    args = parser.parse_args()
    experiment_path = ROOT / args.config
    experiment = json.loads(experiment_path.read_text(encoding="utf-8"))
    runtime_path = ROOT / experiment["runtime_configuration"]
    configuration = json.loads(runtime_path.read_text(encoding="utf-8"))
    rid = experiment["experiment_id"]
    output = ROOT / "research/artifacts" / rid
    result_path = ROOT / "research/reports" / f"{rid}.result.json"
    record_path = ROOT / "research/experiments" / f"{rid}.execution.record.json"
    if any(path.exists() for path in (output, result_path, record_path, ROOT / "research/runs" / f"{rid}.json")):
        raise FileExistsError("refusing to overwrite formal run or artifacts")
    model_path = ROOT / configuration["model"]
    inputs = [file_input(experiment_path), file_input(runtime_path), file_input(Path(__file__)), file_input(ROOT / "src/first_robots/simulation.py")]
    inputs.extend(file_input(path) for path in sorted(model_path.parent.rglob("*")) if path.is_file())
    repeat = experiment.get("repeat_of")
    if repeat:
        for name in ("result", "trajectory"):
            frozen_input = file_input(ROOT / repeat[name])
            if frozen_input["sha256"] != repeat[f"{name}_sha256"]:
                raise RuntimeError("Process-repeat predecessor hash mismatch")
            inputs.append(frozen_input)
    with RunRecorder(run_id=rid, objective="Verify fixed-target actuator-driven virtual SO-101 motion before object grasping.", script=Path(__file__), repo_root=ROOT, output_root=ROOT / "research/runs", parameters=configuration, random_seed=None, inputs=inputs, baseline={"kind": "known_target_motion_baseline", "reference": "initial versus final position error"}, level="formal") as recorder:
        output.mkdir(parents=True)
        (output / "config.json").write_bytes(strict_json_bytes(configuration))
        report, trace = simulate_reach(model_path, configuration)
        repeat_equal = True
        if repeat:
            with np.load(ROOT / repeat["trajectory"]) as predecessor_trace:
                repeat_equal = bool(set(trace) == set(predecessor_trace.files) and all(np.array_equal(trace[name], predecessor_trace[name]) for name in trace))
            report["fresh_process_trajectory_equals_predecessor"] = repeat_equal
        if trace:
            np.savez_compressed(output / "trajectory.npz", **trace)
            frames = render_reach_replay(model_path, configuration, trace)
            images = []
            for frame_index, frame in enumerate(frames):
                image = Image.fromarray(frame)
                draw = ImageDraw.Draw(image)
                draw.rectangle((0, 0, 480, 34), fill="white")
                draw.text((8, 8), f"SO-101 motion baseline | frame {frame_index + 1}/{len(frames)}", fill="black")
                images.append(image)
            images[0].save(output / "motion.gif", save_all=True, append_images=images[1:], duration=50, loop=0)
            images[-1].save(output / "final.png")
        passed = bool(repeat_equal and report["ik"]["converged"] and report["execution_applied"] and report["reason"] == "motion_completed" and report.get("finite_trajectory", False) and report.get("actual_joint_limits_satisfied", False) and report.get("position_error_reduced", False))
        result = {"schema_version": "first-robots/research-result/v1", "run_id": rid, "status": "passed_pending_human_review" if passed else "blocked_pending_human_review", "observations": report, "review": {"status": "pending_human_review", "adopted": False}}
        execution_record = {"schema_version": "first-robots/experiment-record/v1", "run_id": rid, "kind": "formal_single_arm_reach", "result": {"path": result_path.relative_to(ROOT).as_posix(), "sha256": sha256_bytes(strict_json_bytes(result))}, "review": {"status": "pending_human_review", "adopted": False}}
        write_json_pair(first_path=record_path, first_content=execution_record, second_path=result_path, second_content=result)
        for path in sorted(output.iterdir()):
            recorder.add_artifact(path)
        recorder.add_artifact(record_path)
        recorder.add_artifact(result_path)
        recorder.add_metric("fixed_target_motion_valid", passed, "boolean")
        if "final_position_error_m" in report:
            recorder.add_metric("final_position_error_m", report["final_position_error_m"], "m")
        recorder.complete(passed=passed, criteria="Bounded IK converges; actual position-actuator trajectory completes finite, within limits, without checked moving-body penetrations, and reduces target error.", note="Fixed virtual position-only motion; numerical settings and results are not adopted real-arm capability or grasp thresholds.")
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
