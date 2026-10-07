"""Freeze and render a reviewable virtual world-coordinate contract; never uses hardware."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image
from experiment_management.run import RunRecorder, sha256_bytes, strict_json_bytes, write_json_pair


RID = "EXP-20260921-012-world-coordinate-contract-review"
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


def add_box(spec: mujoco.MjSpec, name: str, size: list[float], position: list[float], rgba: list[float]) -> None:
    geom = spec.worldbody.add_geom()
    geom.name = name
    geom.type = mujoco.mjtGeom.mjGEOM_BOX
    geom.size = size
    geom.pos = position
    geom.rgba = rgba
    geom.contype = 0
    geom.conaffinity = 0


def add_camera(spec: mujoco.MjSpec, name: str, camera: dict, renderer: dict) -> None:
    view = spec.worldbody.add_camera()
    view.name = name
    view.pos = camera["position_m"]
    view.quat = look_at_quaternion(np.asarray(camera["position_m"]), np.asarray(camera["target_m"]))
    view.resolution = [renderer["width"], renderer["height"]]
    view.fovy = 60


def main() -> None:
    if OUT.exists() or RESULT.exists() or EXECUTION_RECORD.exists():
        raise FileExistsError("refusing to overwrite formal artifacts, result, or execution record")
    config = json.loads(CFG.read_text(encoding="utf-8"))
    predecessor = ROOT / config["blocking_predecessor"]["result_path"]
    if sha256_file(predecessor) != config["blocking_predecessor"]["result_sha256"]:
        raise RuntimeError("011 predecessor hash mismatch")
    if sha256_file(SCENE) != config["model"]["scene_sha256"]:
        raise RuntimeError("candidate model hash mismatch")
    contract = config["world_coordinate_contract"]
    recorder = RunRecorder(run_id=RID, objective="Freeze all current virtual object world coordinates and produce review-only layout images.", script=Path(__file__), repo_root=ROOT, output_root=ROOT / "research/runs", parameters=contract, random_seed=None, inputs=[{"kind": "configuration", "path": CFG.relative_to(ROOT).as_posix(), "sha256": sha256_file(CFG)}, {"kind": "predecessor_result", "path": predecessor.relative_to(ROOT).as_posix(), "sha256": sha256_file(predecessor)}, {"kind": "candidate_model", "path": SCENE.relative_to(ROOT).as_posix(), "sha256": sha256_file(SCENE)}], baseline={"kind": "world_coordinate_contract", "reference": config["blocking_predecessor"]["run_id"]}, level="formal")
    with recorder:
        OUT.mkdir(parents=True)
        (OUT / "config.json").write_bytes(strict_json_bytes(config))
        spec = mujoco.MjSpec.from_file(str(SCENE))
        table = contract["table"]
        add_box(spec, table["name"], table["half_extents_m"], table["center_m"], table["rgba"])
        cup = spec.worldbody.add_geom()
        cup.name = contract["cup"]["name"]
        cup.type = mujoco.mjtGeom.mjGEOM_CYLINDER
        cup.size = [contract["cup"]["radius_m"], contract["cup"]["half_height_m"], 0]
        cup.pos = contract["cup"]["center_m"]
        cup.rgba = contract["cup"]["rgba"]
        cup.contype = 0
        cup.conaffinity = 0
        barrier = contract["barrier"]
        add_box(spec, barrier["name"], barrier["half_extents_m"], barrier["center_m"], barrier["rgba"])
        for name, axis in contract["visual_world_axes"].items():
            add_box(spec, name, axis["half_extents_m"], axis["center_m"], axis["rgba"])
        for name, camera in contract["fixed_cameras"].items():
            add_camera(spec, name, camera, contract["renderer"])
        for name, camera in contract["review_cameras"].items():
            add_camera(spec, name, camera, contract["renderer"])
        model = spec.compile()
        data = mujoco.MjData(model)
        data.qpos[:] = contract["robot"]["qpos_rad"]
        mujoco.mj_forward(model, data)
        coordinate_observations = {"baseframe": data.site_xpos[model.site("baseframe").id].tolist()}
        for name in (table["name"], contract["cup"]["name"], barrier["name"], *contract["visual_world_axes"].keys()):
            coordinate_observations[name] = data.geom_xpos[model.geom(name).id].tolist()
        expected_positions = {"baseframe": contract["robot"]["baseframe_world_position_m"], table["name"]: table["center_m"], contract["cup"]["name"]: contract["cup"]["center_m"], barrier["name"]: barrier["center_m"]}
        expected_positions.update({name: axis["center_m"] for name, axis in contract["visual_world_axes"].items()})
        positions_match = all(np.allclose(coordinate_observations[name], expected, rtol=0, atol=1e-12) for name, expected in expected_positions.items())
        table_top_matches = table["center_m"][2] + table["half_extents_m"][2] == table["top_z_m"] == 0
        compiled_camera_positions = {name: model.cam_pos[model.camera(name).id].tolist() for name in (*contract["fixed_cameras"].keys(), *contract["review_cameras"].keys())}
        camera_positions_match = all(np.allclose(compiled_camera_positions[name], item["position_m"], rtol=0, atol=1e-12) for group in (contract["fixed_cameras"], contract["review_cameras"]) for name, item in group.items())
        review_images = {}
        renderer = mujoco.Renderer(model, height=contract["renderer"]["height"], width=contract["renderer"]["width"])
        try:
            for name in contract["review_cameras"]:
                renderer.update_scene(data, camera=name)
                image = renderer.render()
                Image.fromarray(image).save(OUT / f"{name}-rgb.png")
                review_images[name] = {"path": (OUT / f"{name}-rgb.png").relative_to(ROOT).as_posix(), "nonuniform_pixels": bool(np.any(image != image[0, 0]))}
        finally:
            renderer.close()
        images_valid = all(item["nonuniform_pixels"] for item in review_images.values())
        assessment = {"coordinate_contract": contract["definition"], "world_positions_m": coordinate_observations, "compiled_camera_positions_m": compiled_camera_positions, "table_top_matches_world_z_zero": table_top_matches, "positions_match_configuration": positions_match, "camera_positions_match_configuration": camera_positions_match, "review_images": review_images, "side_grasp_target_pose": "not_defined", "grasp_reachability_assessment": "not_evaluated_no_deterministic_ik_collision_protocol"}
        (OUT / "coordinate-state.json").write_bytes(strict_json_bytes(assessment))
        passed = positions_match and table_top_matches and camera_positions_match and images_valid
        result = {"schema_version": "first-robots/research-result/v1", "run_id": RID, "status": "passed_pending_human_review" if passed else "blocked_pending_human_review", "observations": assessment, "limits": config["non_claims"], "review": {"status": "pending_human_review", "adopted": False}}
        execution_record = {"schema_version": "first-robots/experiment-record/v1", "run_id": RID, "kind": "formal_world_coordinate_contract_review", "result": {"path": RESULT.relative_to(ROOT).as_posix(), "sha256": sha256_bytes(strict_json_bytes(result))}, "review": {"status": "pending_human_review", "adopted": False}}
        write_json_pair(first_path=EXECUTION_RECORD, first_content=execution_record, second_path=RESULT, second_content=result)
        for artifact in sorted(OUT.iterdir()):
            recorder.add_artifact(artifact)
        recorder.add_artifact(EXECUTION_RECORD)
        recorder.add_artifact(RESULT)
        recorder.add_metric("world_coordinate_contract_valid", passed, "boolean")
        recorder.complete(passed=passed, criteria="All frozen world coordinates and camera positions match; both review images are nonuniform.", review_status="pending_human_review", note="Visualization-only coordinate contract; no side-grasp target, IK, collision, or hardware conclusion.")


if __name__ == "__main__":
    main()
