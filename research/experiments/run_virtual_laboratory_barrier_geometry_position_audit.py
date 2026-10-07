"""Audit frozen barrier geometry equivalence and a position-only virtual reach screen."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
from experiment_management.run import RunRecorder, sha256_bytes, strict_json_bytes, write_json_pair


RID = "EXP-20260921-009-virtual-laboratory-barrier-geometry-position-audit"
ROOT = Path(__file__).resolve().parents[2]
CFG = ROOT / "research/configs" / f"{RID}.json"
PROBE_CONFIG = ROOT / "research/configs/EXP-20260921-008-virtual-laboratory-occlusion-rgb-color-probe.json"
SCENE = ROOT / "research/sources/mujoco_menagerie_robotstudio_so101/robotstudio_so101/scene.xml"
OUT = ROOT / "research/artifacts" / RID
RESULT = ROOT / "research/reports" / f"{RID}.result.json"
EXECUTION_RECORD = ROOT / "research/experiments" / f"{RID}.execution.record.json"


def sha256_file(path: Path) -> str:
    return "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()


def normalized_reference_scenario(config: dict) -> dict:
    scenario = json.loads(json.dumps(config["scenario"]))
    scenario["barrier"].pop("changed_field_from_005", None)
    scenario["barrier"].pop("purpose", None)
    return scenario


def normalized_probe_geometry(config: dict) -> dict:
    scenario = json.loads(json.dumps(config["scenario"]))
    scenario["cup"].pop("rgba")
    scenario["barrier"].pop("rgba")
    return scenario


def main() -> None:
    if OUT.exists() or RESULT.exists() or EXECUTION_RECORD.exists():
        raise FileExistsError("refusing to overwrite formal artifacts, result, or execution record")
    config = json.loads(CFG.read_text(encoding="utf-8"))
    predecessor = ROOT / config["blocking_predecessor"]["result_path"]
    source_run = ROOT / config["position_screen_provenance"]["source_run_path"]
    if sha256_file(predecessor) != config["blocking_predecessor"]["result_sha256"]:
        raise RuntimeError("008 result hash mismatch")
    if sha256_file(source_run) != config["position_screen_provenance"]["source_run_sha256"]:
        raise RuntimeError("position-screen source-run hash mismatch")
    if sha256_file(SCENE) != config["model"]["scene_sha256"]:
        raise RuntimeError("candidate model hash mismatch")
    reference = json.loads((ROOT / config["reference_barrier_config"]["path"]).read_text(encoding="utf-8"))
    probe = json.loads(PROBE_CONFIG.read_text(encoding="utf-8"))
    geometry_equivalent = normalized_reference_scenario(reference) == normalized_probe_geometry(probe)
    if not geometry_equivalent:
        raise RuntimeError("008 geometry differs from frozen 006 geometry")
    scenario = probe["scenario"]
    recorder = RunRecorder(run_id=RID, objective="Verify 008 geometry equivalence and its frozen position-only reach screen.", script=Path(__file__), repo_root=ROOT, output_root=ROOT / "research/runs", parameters={"reference": reference["experiment_id"], "probe": probe["experiment_id"]}, random_seed=None, inputs=[{"kind": "configuration", "path": CFG.relative_to(ROOT).as_posix(), "sha256": sha256_file(CFG)}, {"kind": "rgb_probe_result", "path": predecessor.relative_to(ROOT).as_posix(), "sha256": sha256_file(predecessor)}, {"kind": "reference_barrier_config", "path": config["reference_barrier_config"]["path"], "sha256": sha256_file(ROOT / config["reference_barrier_config"]["path"])}, {"kind": "rgb_probe_config", "path": PROBE_CONFIG.relative_to(ROOT).as_posix(), "sha256": sha256_file(PROBE_CONFIG)}, {"kind": "position_screen_source_run", "path": source_run.relative_to(ROOT).as_posix(), "sha256": sha256_file(source_run)}, {"kind": "candidate_model", "path": SCENE.relative_to(ROOT).as_posix(), "sha256": sha256_file(SCENE)}], baseline={"kind": "configuration_equivalence", "reference": reference["experiment_id"]}, level="formal")
    with recorder:
        OUT.mkdir(parents=True)
        (OUT / "config.json").write_bytes(strict_json_bytes(config))
        spec = mujoco.MjSpec.from_file(str(SCENE))
        model = spec.compile()
        data = mujoco.MjData(model)
        data.qpos[:] = scenario["qpos"]
        mujoco.mj_forward(model, data)
        base_position = data.site_xpos[model.site("baseframe").id].copy()
        cup_position = np.asarray(scenario["cup"]["center_m"], dtype=float)
        cup_xy_radius_m = float(np.linalg.norm(cup_position[:2] - base_position[:2]))
        radius_m = config["position_screen_provenance"]["virtual_half_reach_radius_m"]
        inside_screen = cup_xy_radius_m <= radius_m
        assessment = {"geometry_equivalent_to_006": geometry_equivalent, "position_screen": {"assessment": "inside_virtual_half_reach_screen" if inside_screen else "outside_virtual_half_reach_screen", "baseframe_world_position_m": base_position.tolist(), "cup_xy_radius_from_base_m": cup_xy_radius_m, "virtual_half_reach_radius_m": radius_m, "source_run_id": config["position_screen_provenance"]["source_run_id"]}, "grasp_reachability_assessment": "not_evaluated_no_deterministic_ik_collision_protocol"}
        (OUT / "terminal-assessment.json").write_bytes(strict_json_bytes(assessment))
        passed = geometry_equivalent and inside_screen
        result = {"schema_version": "first-robots/research-result/v1", "run_id": RID, "status": "passed_pending_human_review" if passed else "blocked_pending_human_review", "observations": assessment, "limits": config["non_claims"], "review": {"status": "pending_human_review", "adopted": False}}
        execution_record = {"schema_version": "first-robots/experiment-record/v1", "run_id": RID, "kind": "formal_virtual_laboratory_barrier_geometry_position_audit", "result": {"path": RESULT.relative_to(ROOT).as_posix(), "sha256": sha256_bytes(strict_json_bytes(result))}, "review": {"status": "pending_human_review", "adopted": False}}
        write_json_pair(first_path=EXECUTION_RECORD, first_content=execution_record, second_path=RESULT, second_content=result)
        for artifact in sorted(OUT.iterdir()):
            recorder.add_artifact(artifact)
        recorder.add_artifact(EXECUTION_RECORD)
        recorder.add_artifact(RESULT)
        recorder.add_metric("geometry_equivalent_to_006", geometry_equivalent, "boolean")
        recorder.add_metric("inside_virtual_half_reach_screen", inside_screen, "boolean")
        recorder.complete(passed=passed, criteria="008 physical geometry equals 006 and cup position remains inside frozen virtual screen.", review_status="pending_human_review", note="No render, movement, or hardware I/O; grasp reachability intentionally not evaluated.")


if __name__ == "__main__":
    main()
