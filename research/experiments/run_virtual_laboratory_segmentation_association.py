"""Verify renderer segmentation association without changing virtual geometry or hardware I/O."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image
from experiment_management.run import RunRecorder, sha256_bytes, strict_json_bytes, write_json_pair


RID = "EXP-20260921-007-virtual-laboratory-segmentation-association"
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


def render(renderer: mujoco.Renderer, data: mujoco.MjData, camera: str, output: Path) -> tuple[np.ndarray, np.ndarray]:
    renderer.update_scene(data, camera=camera)
    rgb = renderer.render()
    Image.fromarray(rgb).save(output / f"{camera}-rgb.png")
    renderer.enable_segmentation_rendering()
    renderer.update_scene(data, camera=camera)
    segmentation = renderer.render()
    renderer.disable_segmentation_rendering()
    np.save(output / f"{camera}-segmentation.npy", segmentation)
    return rgb, segmentation


def object_color_check(rgb: np.ndarray, segmentation: np.ndarray, geom_id: int, dominant_channel: int, other_channel: int) -> dict:
    mask = (segmentation[:, :, 0] == geom_id) & (segmentation[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM))
    colors = rgb[mask]
    return {
        "pixel_count": int(mask.sum()),
        "all_pixels_have_expected_channel_order": bool(colors.size and np.all(colors[:, dominant_channel] > colors[:, other_channel])),
        "median_rgb": np.median(colors, axis=0).tolist() if colors.size else None,
    }


def main() -> None:
    if OUT.exists() or RESULT.exists() or EXECUTION_RECORD.exists():
        raise FileExistsError("refusing to overwrite formal artifacts, result, or execution record")
    config = json.loads(CFG.read_text(encoding="utf-8"))
    predecessor = ROOT / config["blocking_predecessor"]["result_path"]
    if sha256_file(predecessor) != config["blocking_predecessor"]["result_sha256"]:
        raise RuntimeError("006 result hash mismatch")
    if sha256_file(SCENE) != config["model"]["scene_sha256"]:
        raise RuntimeError("candidate model hash mismatch")
    geometry = config["frozen_geometry"]
    recorder = RunRecorder(run_id=RID, objective="Verify renderer label association using display-only colors.", script=Path(__file__), repo_root=ROOT, output_root=ROOT / "research/runs", parameters=geometry, random_seed=None, inputs=[{"kind": "configuration", "path": CFG.relative_to(ROOT).as_posix(), "sha256": sha256_file(CFG)}, {"kind": "blocked_predecessor_result", "path": predecessor.relative_to(ROOT).as_posix(), "sha256": sha256_file(predecessor)}, {"kind": "candidate_model", "path": SCENE.relative_to(ROOT).as_posix(), "sha256": sha256_file(SCENE)}], baseline={"kind": "display_only_association", "reference": config["blocking_predecessor"]["run_id"]}, level="formal")
    with recorder:
        OUT.mkdir(parents=True)
        (OUT / "config.json").write_bytes(strict_json_bytes(config))
        spec = mujoco.MjSpec.from_file(str(SCENE))
        add_box(spec, "vlab_table", geometry["table"]["half_extents_m"], geometry["table"]["center_m"])
        cup = spec.worldbody.add_geom()
        cup.name = "vlab_cup"
        cup.type = mujoco.mjtGeom.mjGEOM_CYLINDER
        cup.size = [geometry["cup"]["radius_m"], geometry["cup"]["half_height_m"], 0]
        cup.pos = geometry["cup"]["center_m"]
        cup.rgba = geometry["cup"]["rgba"]
        add_box(spec, "vlab_barrier", geometry["barrier"]["half_extents_m"], geometry["barrier"]["center_m"], geometry["barrier"]["rgba"])
        for name, camera in geometry["fixed_cameras"].items():
            view = spec.worldbody.add_camera()
            view.name = name
            view.pos = camera["position_m"]
            view.quat = look_at_quaternion(np.asarray(camera["position_m"]), np.asarray(camera["target_m"]))
            view.resolution = [geometry["renderer"]["width"], geometry["renderer"]["height"]]
            view.fovy = 60
        model = spec.compile()
        data = mujoco.MjData(model)
        data.qpos[:] = geometry["qpos"]
        mujoco.mj_forward(model, data)
        cup_id = model.geom("vlab_cup").id
        barrier_id = model.geom("vlab_barrier").id
        observations = {}
        renderer = mujoco.Renderer(model, height=geometry["renderer"]["height"], width=geometry["renderer"]["width"])
        try:
            for camera in ("fixed_depth_01", "fixed_depth_02"):
                rgb, segmentation = render(renderer, data, camera, OUT)
                observations[camera] = {
                    "cup": object_color_check(rgb, segmentation, cup_id, dominant_channel=0, other_channel=2),
                    "barrier": object_color_check(rgb, segmentation, barrier_id, dominant_channel=2, other_channel=0),
                }
        finally:
            renderer.close()
        passed = all(item["cup"]["all_pixels_have_expected_channel_order"] and item["barrier"]["all_pixels_have_expected_channel_order"] for item in observations.values())
        assessment = {"association_verified": passed, "camera_observations": observations, "geometry_changed_from_006": False, "change": "display rgba only"}
        (OUT / "terminal-assessment.json").write_bytes(strict_json_bytes(assessment))
        result = {"schema_version": "first-robots/research-result/v1", "run_id": RID, "status": "passed_pending_human_review" if passed else "blocked_pending_human_review", "observations": assessment, "limits": config["non_claims"], "review": {"status": "pending_human_review", "adopted": False}}
        execution_record = {"schema_version": "first-robots/experiment-record/v1", "run_id": RID, "kind": "formal_virtual_laboratory_segmentation_association", "result": {"path": RESULT.relative_to(ROOT).as_posix(), "sha256": sha256_bytes(strict_json_bytes(result))}, "review": {"status": "pending_human_review", "adopted": False}}
        write_json_pair(first_path=EXECUTION_RECORD, first_content=execution_record, second_path=RESULT, second_content=result)
        for artifact in sorted(OUT.iterdir()):
            recorder.add_artifact(artifact)
        recorder.add_artifact(EXECUTION_RECORD)
        recorder.add_artifact(RESULT)
        recorder.add_metric("segmentation_association_verified", passed, "boolean")
        recorder.complete(passed=passed, criteria="Each fixed camera's cup and barrier segmentation pixels retain the configured red/blue channel ordering.", review_status="pending_human_review", note="Display-only renderer-label association check.")


if __name__ == "__main__":
    main()
