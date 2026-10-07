"""Measure a predeclared artificial cup-color signature in one frozen virtual barrier scene."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image
from experiment_management.run import RunRecorder, sha256_bytes, strict_json_bytes, write_json_pair


RID = "EXP-20260921-008-virtual-laboratory-occlusion-rgb-color-probe"
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


def render_rgb(renderer: mujoco.Renderer, data: mujoco.MjData, camera: str, output: Path) -> np.ndarray:
    renderer.update_scene(data, camera=camera)
    rgb = renderer.render()
    Image.fromarray(rgb).save(output / f"{camera}-rgb.png")
    return rgb.copy()


def cup_color_signature_pixel_count(rgb: np.ndarray) -> int:
    return int(np.count_nonzero((rgb[:, :, 0] > rgb[:, :, 1]) & (rgb[:, :, 2] > rgb[:, :, 1])))


def main() -> None:
    if OUT.exists() or RESULT.exists() or EXECUTION_RECORD.exists():
        raise FileExistsError("refusing to overwrite formal artifacts, result, or execution record")
    config = json.loads(CFG.read_text(encoding="utf-8"))
    predecessor = ROOT / config["blocking_predecessor"]["result_path"]
    if sha256_file(predecessor) != config["blocking_predecessor"]["result_sha256"]:
        raise RuntimeError("007 result hash mismatch")
    if sha256_file(SCENE) != config["model"]["scene_sha256"]:
        raise RuntimeError("candidate model hash mismatch")
    scenario = config["scenario"]
    recorder = RunRecorder(run_id=RID, objective="Measure only the frozen virtual cup RGB color signature through the central barrier.", script=Path(__file__), repo_root=ROOT, output_root=ROOT / "research/runs", parameters=scenario, random_seed=None, inputs=[{"kind": "configuration", "path": CFG.relative_to(ROOT).as_posix(), "sha256": sha256_file(CFG)}, {"kind": "renderer_association_predecessor", "path": predecessor.relative_to(ROOT).as_posix(), "sha256": sha256_file(predecessor)}, {"kind": "candidate_model", "path": SCENE.relative_to(ROOT).as_posix(), "sha256": sha256_file(SCENE)}], baseline={"kind": "frozen_barrier_geometry", "reference": "EXP-20260921-006-virtual-laboratory-barrier-width-isolation"}, level="formal")
    with recorder:
        OUT.mkdir(parents=True)
        (OUT / "config.json").write_bytes(strict_json_bytes(config))
        spec = mujoco.MjSpec.from_file(str(SCENE))
        add_box(spec, "vlab_table", scenario["table"]["half_extents_m"], scenario["table"]["center_m"])
        cup = spec.worldbody.add_geom()
        cup.name = "vlab_cup"
        cup.type = mujoco.mjtGeom.mjGEOM_CYLINDER
        cup.size = [scenario["cup"]["radius_m"], scenario["cup"]["half_height_m"], 0]
        cup.pos = scenario["cup"]["center_m"]
        cup.rgba = scenario["cup"]["rgba"]
        add_box(spec, "vlab_barrier", scenario["barrier"]["half_extents_m"], scenario["barrier"]["center_m"], scenario["barrier"]["rgba"])
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
        renderer = mujoco.Renderer(model, height=scenario["renderer"]["height"], width=scenario["renderer"]["width"])
        try:
            observations = {camera: {"cup_color_signature_pixels": cup_color_signature_pixel_count(render_rgb(renderer, data, camera, OUT))} for camera in ("fixed_depth_01", "fixed_depth_02")}
        finally:
            renderer.close()
        absent = all(value["cup_color_signature_pixels"] == 0 for value in observations.values())
        assessment = {"terminal_assessment": "not_observable_by_nominal_fixed_rgb_color_probe" if absent else "rgb_color_probe_candidate_remains_observable", "camera_observations": observations, "geometry_changed_from_006": False, "measurement": config["measurement"]}
        (OUT / "terminal-assessment.json").write_bytes(strict_json_bytes(assessment))
        result = {"schema_version": "first-robots/research-result/v1", "run_id": RID, "status": "passed_pending_human_review" if absent else "blocked_pending_human_review", "observations": assessment, "limits": config["non_claims"], "review": {"status": "pending_human_review", "adopted": False}}
        execution_record = {"schema_version": "first-robots/experiment-record/v1", "run_id": RID, "kind": "formal_virtual_laboratory_occlusion_rgb_color_probe", "result": {"path": RESULT.relative_to(ROOT).as_posix(), "sha256": sha256_bytes(strict_json_bytes(result))}, "review": {"status": "pending_human_review", "adopted": False}}
        write_json_pair(first_path=EXECUTION_RECORD, first_content=execution_record, second_path=RESULT, second_content=result)
        for artifact in sorted(OUT.iterdir()):
            recorder.add_artifact(artifact)
        recorder.add_artifact(EXECUTION_RECORD)
        recorder.add_artifact(RESULT)
        recorder.add_metric("fixed_cameras_cup_color_signature_absent", absent, "boolean")
        recorder.complete(passed=absent, criteria="Both fixed RGB images have zero predeclared cup-color signature pixels.", review_status="pending_human_review", note="Artificial-color static RGB measurement; no reachability or physical interpretation.")


if __name__ == "__main__":
    main()
