"""Compare compiled virtual scenes and screen one cup position; never render or use hardware."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
from experiment_management.run import RunRecorder, sha256_bytes, strict_json_bytes, write_json_pair


RID = "EXP-20260921-010-virtual-laboratory-compiled-geometry-position-audit"
ROOT = Path(__file__).resolve().parents[2]
CFG = ROOT / "research/configs" / f"{RID}.json"
SCENE = ROOT / "research/sources/mujoco_menagerie_robotstudio_so101/robotstudio_so101/scene.xml"
OUT = ROOT / "research/artifacts" / RID
RESULT = ROOT / "research/reports" / f"{RID}.result.json"
EXECUTION_RECORD = ROOT / "research/experiments" / f"{RID}.execution.record.json"


def sha256_file(path: Path) -> str:
    return "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()


def look_at_quaternion(position: np.ndarray, target: np.ndarray) -> list[float]:
    z_axis = -(target - position)
    z_axis /= np.linalg.norm(z_axis)
    x_axis = np.cross(np.array([0.0, 0.0, 1.0]), z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    output = np.empty(4)
    mujoco.mju_mat2Quat(output, np.column_stack((x_axis, y_axis, z_axis)).reshape(9))
    return output.tolist()


def add_box(spec: mujoco.MjSpec, name: str, size: list[float], position: list[float], rgba: list[float] | None = None) -> None:
    geom = spec.worldbody.add_geom()
    geom.name = name
    geom.type = mujoco.mjtGeom.mjGEOM_BOX
    geom.size = size
    geom.pos = position
    if rgba is not None:
        geom.rgba = rgba


def compile_scene(scenario: dict, colored: bool) -> tuple[mujoco.MjModel, mujoco.MjData]:
    spec = mujoco.MjSpec.from_file(str(SCENE))
    add_box(spec, "vlab_table", scenario["table"]["half_extents_m"], scenario["table"]["center_m"])
    cup = spec.worldbody.add_geom()
    cup.name = "vlab_cup"
    cup.type = mujoco.mjtGeom.mjGEOM_CYLINDER
    cup.size = [scenario["cup"]["radius_m"], scenario["cup"]["half_height_m"], 0]
    cup.pos = scenario["cup"]["center_m"]
    if colored:
        cup.rgba = scenario["cup"]["rgba"]
    add_box(spec, "vlab_barrier", scenario["barrier"]["half_extents_m"], scenario["barrier"]["center_m"], scenario["barrier"].get("rgba"))
    for name, camera in scenario["fixed_cameras"].items():
        view = spec.worldbody.add_camera()
        view.name = name
        view.pos = camera["position_m"]
        view.quat = look_at_quaternion(np.asarray(camera["position_m"]), np.asarray(camera["target_m"]))
        view.resolution = [scenario["renderer"]["width"], scenario["renderer"]["height"]]
        view.fovy = 60
    model = spec.compile()
    data = mujoco.MjData(model)
    data.qpos[:] = scenario["qpos"]
    mujoco.mj_forward(model, data)
    return model, data


def main() -> None:
    if OUT.exists() or RESULT.exists() or EXECUTION_RECORD.exists():
        raise FileExistsError("refusing to overwrite formal artifacts, result, or execution record")
    config = json.loads(CFG.read_text(encoding="utf-8"))
    predecessor = ROOT / config["blocking_predecessor"]["result_path"]
    reference_path = ROOT / config["reference_config_path"]
    probe_path = ROOT / config["probe_config_path"]
    source_run = ROOT / config["position_screen_provenance"]["source_run_path"]
    if sha256_file(predecessor) != config["blocking_predecessor"]["result_sha256"] or sha256_file(source_run) != config["position_screen_provenance"]["source_run_sha256"] or sha256_file(SCENE) != config["model"]["scene_sha256"]:
        raise RuntimeError("frozen input hash mismatch")
    reference = json.loads(reference_path.read_text(encoding="utf-8"))["scenario"]
    probe = json.loads(probe_path.read_text(encoding="utf-8"))["scenario"]
    reference_model, reference_data = compile_scene(reference, colored=False)
    probe_model, probe_data = compile_scene(probe, colored=True)
    geometry_equal = all(reference_model.geom(name).type[0] == probe_model.geom(name).type[0] and np.array_equal(reference_model.geom_size[reference_model.geom(name).id], probe_model.geom_size[probe_model.geom(name).id]) and np.array_equal(reference_data.geom_xpos[reference_model.geom(name).id], probe_data.geom_xpos[probe_model.geom(name).id]) for name in ("vlab_table", "vlab_cup", "vlab_barrier"))
    cameras_equal = all(np.array_equal(reference_model.cam_pos[reference_model.camera(name).id], probe_model.cam_pos[probe_model.camera(name).id]) and np.array_equal(reference_model.cam_quat[reference_model.camera(name).id], probe_model.cam_quat[probe_model.camera(name).id]) and reference_model.cam_fovy[reference_model.camera(name).id] == probe_model.cam_fovy[probe_model.camera(name).id] and np.array_equal(reference_model.cam_resolution[reference_model.camera(name).id], probe_model.cam_resolution[probe_model.camera(name).id]) for name in ("fixed_depth_01", "fixed_depth_02"))
    qpos_equal = np.array_equal(reference_data.qpos, probe_data.qpos)
    base_position = probe_data.site_xpos[probe_model.site("baseframe").id].copy()
    cup_position = probe_data.geom_xpos[probe_model.geom("vlab_cup").id].copy()
    cup_xy_radius_m = float(np.linalg.norm(cup_position[:2] - base_position[:2]))
    radius_m = config["position_screen_provenance"]["virtual_half_reach_radius_m"]
    inside_screen = cup_xy_radius_m <= radius_m
    recorder = RunRecorder(run_id=RID, objective="Compare 006 and 008 compiled geometry, then apply a frozen position-only screen.", script=Path(__file__), repo_root=ROOT, output_root=ROOT / "research/runs", parameters={"reference": config["reference_config_path"], "probe": config["probe_config_path"]}, random_seed=None, inputs=[{"kind": "configuration", "path": CFG.relative_to(ROOT).as_posix(), "sha256": sha256_file(CFG)}, {"kind": "rgb_probe_result", "path": predecessor.relative_to(ROOT).as_posix(), "sha256": sha256_file(predecessor)}, {"kind": "reference_config", "path": reference_path.relative_to(ROOT).as_posix(), "sha256": sha256_file(reference_path)}, {"kind": "probe_config", "path": probe_path.relative_to(ROOT).as_posix(), "sha256": sha256_file(probe_path)}, {"kind": "position_screen_source_run", "path": source_run.relative_to(ROOT).as_posix(), "sha256": sha256_file(source_run)}, {"kind": "candidate_model", "path": SCENE.relative_to(ROOT).as_posix(), "sha256": sha256_file(SCENE)}], baseline={"kind": "compiled_scene_equivalence", "reference": "EXP-20260921-006-virtual-laboratory-barrier-width-isolation"}, level="formal")
    with recorder:
        OUT.mkdir(parents=True)
        (OUT / "config.json").write_bytes(strict_json_bytes(config))
        assessment = {"compiled_geometry_equal_to_006": geometry_equal, "compiled_fixed_cameras_equal_to_006": cameras_equal, "qpos_equal_to_006": qpos_equal, "position_screen": {"assessment": "inside_virtual_half_reach_screen" if inside_screen else "outside_virtual_half_reach_screen", "baseframe_world_position_m": base_position.tolist(), "cup_xy_radius_from_base_m": cup_xy_radius_m, "virtual_half_reach_radius_m": radius_m, "source_run_id": config["position_screen_provenance"]["source_run_id"]}, "grasp_reachability_assessment": "not_evaluated_no_deterministic_ik_collision_protocol"}
        (OUT / "terminal-assessment.json").write_bytes(strict_json_bytes(assessment))
        passed = geometry_equal and cameras_equal and qpos_equal and inside_screen
        result = {"schema_version": "first-robots/research-result/v1", "run_id": RID, "status": "passed_pending_human_review" if passed else "blocked_pending_human_review", "observations": assessment, "limits": config["non_claims"], "review": {"status": "pending_human_review", "adopted": False}}
        execution_record = {"schema_version": "first-robots/experiment-record/v1", "run_id": RID, "kind": "formal_virtual_laboratory_compiled_geometry_position_audit", "result": {"path": RESULT.relative_to(ROOT).as_posix(), "sha256": sha256_bytes(strict_json_bytes(result))}, "review": {"status": "pending_human_review", "adopted": False}}
        write_json_pair(first_path=EXECUTION_RECORD, first_content=execution_record, second_path=RESULT, second_content=result)
        for artifact in sorted(OUT.iterdir()):
            recorder.add_artifact(artifact)
        recorder.add_artifact(EXECUTION_RECORD)
        recorder.add_artifact(RESULT)
        recorder.add_metric("compiled_geometry_equal_to_006", geometry_equal, "boolean")
        recorder.add_metric("compiled_fixed_cameras_equal_to_006", cameras_equal, "boolean")
        recorder.add_metric("inside_virtual_half_reach_screen", inside_screen, "boolean")
        recorder.complete(passed=passed, criteria="Compiled geometry/cameras/qpos match and cup stays inside the frozen virtual screen.", review_status="pending_human_review", note="No rendering, movement, or hardware I/O; grasp reachability intentionally not evaluated.")


if __name__ == "__main__":
    main()
