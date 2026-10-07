"""Record bounded development tuning or a frozen physical pick-and-place run."""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

import numpy as np
from PIL import Image, ImageDraw
from experiment_management.run import RunRecorder, sha256_bytes, strict_json_bytes, write_json_pair
from first_robots.grasping import render_pick_place_replay, simulate_pick_place


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
        raise RuntimeError("Fixture and side-pose prerequisites have not passed")
    rid = experiment["experiment_id"]
    level = experiment.get("level", "formal")
    commands = experiment.get("closed_gripper_commands_rad", [configuration["closed_gripper_command_rad"]])
    if level == "formal" and commands != [configuration["closed_gripper_command_rad"]]:
        raise ValueError("Formal run must use one frozen runtime configuration")
    output = ROOT / "research/artifacts" / rid
    result_path = ROOT / "research/reports" / f"{rid}.result.json"
    record_path = ROOT / "research/experiments" / f"{rid}.execution.record.json"
    if any(path.exists() for path in (output, result_path, record_path, ROOT / "research/runs" / f"{rid}.json")):
        raise FileExistsError("Refusing to overwrite recorded pick-and-place run")
    model_path = ROOT / configuration["model"]
    inputs = [input_file(path) for path in (experiment_path, runtime_path, ROOT / experiment["plan"], Path(__file__), predecessor, ROOT / "src/first_robots/grasping.py", ROOT / "src/first_robots/simulation.py")]
    if experiment.get("repeat_reference"):
        inputs.append(input_file(ROOT / experiment["repeat_reference"]))
    inputs.extend(input_file(path) for path in sorted(model_path.parent.rglob("*")) if path.is_file())
    observations = []
    with RunRecorder(run_id=rid, objective="Verify contact-driven known-coordinate cylinder pick-and-place before adding camera or language inputs.", script=Path(__file__), repo_root=ROOT, output_root=ROOT / "research/runs", parameters={"configuration": configuration, "closed_gripper_commands_rad": commands}, random_seed=None, inputs=inputs, baseline={"kind": "passed_contact_task_prerequisite", "reference": json.loads(predecessor.read_text(encoding="utf-8"))["run_id"]}, level=level) as recorder:
        output.mkdir(parents=True)
        for index, command in enumerate(commands):
            candidate = copy.deepcopy(configuration)
            candidate["closed_gripper_command_rad"] = command
            attempt = output / f"attempt-{index + 1:02d}"
            attempt.mkdir()
            (attempt / "config.json").write_bytes(strict_json_bytes(candidate))
            report, trace = simulate_pick_place(model_path, candidate)
            np.savez_compressed(attempt / "trajectory.npz", **trace)
            report["closed_gripper_command_rad"] = command
            report["actual_duration_s"] = float(trace["time_s"][-1])
            report["recorded_states"] = len(trace["time_s"])
            report["maximum_jaw_normal_force_n"] = {name: float(np.max(trace[name])) for name in ("fixed_jaw_force_n", "moving_jaw_force_n")}
            if experiment.get("repeat_reference"):
                with np.load(ROOT / experiment["repeat_reference"], allow_pickle=False) as reference:
                    comparisons = {name: bool(name in reference and np.array_equal(value, reference[name])) for name, value in trace.items()}
                    report["repeat_trace_comparison"] = {"arrays": comparisons, "identical": bool(set(reference.files) == set(trace) and all(comparisons.values()))}
            (attempt / "observations.json").write_bytes(strict_json_bytes(report))
            observations.append(report)
            if level == "formal":
                frames = render_pick_place_replay(model_path, candidate, trace)
                images = []
                indices = list(range(0, len(trace["qpos"]), 20))
                if indices[-1] != len(trace["qpos"]) - 1:
                    indices.append(len(trace["qpos"]) - 1)
                snapshots = {}
                for frame, state_index in zip(frames, indices):
                    image = Image.fromarray(frame)
                    phase = str(trace["phase"][state_index])
                    ImageDraw.Draw(image).text((8, 8), f"{phase}  t={trace['time_s'][state_index]:.2f}s", fill="white")
                    images.append(image)
                    snapshots[phase] = image
                for phase, image in snapshots.items():
                    image.save(attempt / f"phase-{phase}.png")
                images[0].save(attempt / "motion.gif", save_all=True, append_images=images[1:], duration=100, loop=0)
                images[-1].save(attempt / "final.png")
            recorder.add_metric(f"attempt_{index + 1}_completed", report["completed"], "boolean")
        passed = bool(all(item["completed"] and item.get("repeat_trace_comparison", {"identical": True})["identical"] for item in observations)) if level == "formal" else True
        result = {"schema_version": "first-robots/research-result/v1", "run_id": rid, "status": "passed_pending_human_review" if passed else "blocked_pending_human_review", "level": level, "observations": observations, "review": {"status": "pending_human_review", "adopted": False}}
        record = {"schema_version": "first-robots/experiment-record/v1", "run_id": rid, "kind": "contact_pick_place" if level == "formal" else "exploratory_gripper_command_comparison", "result": {"path": result_path.relative_to(ROOT).as_posix(), "sha256": sha256_bytes(strict_json_bytes(result))}, "review": {"status": "pending_human_review", "adopted": False}}
        write_json_pair(first_path=record_path, first_content=record, second_path=result_path, second_content=result)
        for path in sorted(output.rglob("*")):
            if path.is_file():
                recorder.add_artifact(path)
        recorder.add_artifact(record_path)
        recorder.add_artifact(result_path)
        recorder.complete(passed=passed, criteria="Formal: actual gated contact lift/transfer and released supported placement in declared B. Exploratory: all planned conditions recorded, including failed attempts.", note="Fixed synthetic task only; no hardware force/precision calibration, camera grounding or language task. Deterministic no-random-input protocol; null seed exception recorded in plan.")
    print(json.dumps({"run_id": rid, "status": result["status"], "attempts": [{"command_rad": item["closed_gripper_command_rad"], "reason": item["reason"], "center_error_to_b_m": item["evaluation_only_radial_error_to_b_m"], "maximum_jaw_normal_force_n": item["maximum_jaw_normal_force_n"]} for item in observations]}, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
