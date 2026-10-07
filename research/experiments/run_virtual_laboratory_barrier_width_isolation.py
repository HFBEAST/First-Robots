"""Run one single-variable virtual barrier-width isolation; never controls hardware."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image
from experiment_management.run import RunRecorder, sha256_bytes, strict_json_bytes, write_json_pair


RID = "EXP-20260921-006-virtual-laboratory-barrier-width-isolation"
ROOT = Path(__file__).resolve().parents[2]
CFG = ROOT / "research/configs" / f"{RID}.json"
SCENE = ROOT / "research/sources/mujoco_menagerie_robotstudio_so101/robotstudio_so101/scene.xml"
OUT = ROOT / "research/artifacts" / RID
RESULT = ROOT / "research/reports" / f"{RID}.result.json"
EXECUTION_RECORD = ROOT / "research/experiments" / f"{RID}.execution.record.json"


def sha256_file(path: Path) -> str:
    return "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()


def load_frozen_json(config: dict, key: str) -> dict:
    item = config[key]
    path = ROOT / item["result_path"]
    if sha256_file(path) != item["result_sha256"]:
        raise RuntimeError(f"{key} hash mismatch")
    return json.loads(path.read_text(encoding="utf-8"))


def look_at_quaternion(position: np.ndarray, target: np.ndarray) -> list[float]:
    z_axis = -(target - position)
    z_axis /= np.linalg.norm(z_axis)
    x_axis = np.cross(np.array([0.0, 0.0, 1.0]), z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    quaternion = np.empty(4)
    mujoco.mju_mat2Quat(quaternion, np.column_stack((x_axis, y_axis, z_axis)).reshape(9))
    return quaternion.tolist()


def add_box(spec: mujoco.MjSpec, name: str, half_extents: list[float], center: list[float]) -> None:
    geom = spec.worldbody.add_geom()
    geom.name = name
    geom.type = mujoco.mjtGeom.mjGEOM_BOX
    geom.size = half_extents
    geom.pos = center


def render_camera(renderer: mujoco.Renderer, data: mujoco.MjData, camera: str, cup_geom_id: int, output: Path) -> dict:
    renderer.update_scene(data, camera=camera)
    Image.fromarray(renderer.render()).save(output / f"{camera}-rgb.png")
    renderer.enable_depth_rendering()
    renderer.update_scene(data, camera=camera)
    depth = renderer.render()
    renderer.disable_depth_rendering()
    np.save(output / f"{camera}-depth.npy", depth)
    renderer.enable_segmentation_rendering()
    renderer.update_scene(data, camera=camera)
    segmentation = renderer.render()
    renderer.disable_segmentation_rendering()
    np.save(output / f"{camera}-segmentation.npy", segmentation)
    cup_mask = (segmentation[:, :, 0] == cup_geom_id) & (
        segmentation[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM)
    )
    return {"cup_segmentation_pixels": int(np.count_nonzero(cup_mask)), "finite_depth_fraction": float(np.isfinite(depth).mean())}


def assert_only_barrier_width_changed(config: dict) -> None:
    prior_config_path = ROOT / "research/configs/EXP-20260921-005-virtual-laboratory-barrier-record-lifecycle-repair.json"
    prior = json.loads(prior_config_path.read_text(encoding="utf-8"))["scenario"]
    current = config["scenario"]
    if prior["barrier"]["half_extents_m"] != [0.01, 0.35, 0.16]:
        raise RuntimeError("frozen 005 barrier baseline is unexpected")
    if current["barrier"]["half_extents_m"] != [0.01, 0.46, 0.16]:
        raise RuntimeError("planned barrier width is unexpected")
    normalized_prior = json.loads(json.dumps(prior))
    normalized_current = json.loads(json.dumps(current))
    normalized_prior["barrier"]["half_extents_m"][1] = normalized_current["barrier"]["half_extents_m"][1]
    normalized_prior["barrier"]["purpose"] = normalized_current["barrier"]["purpose"]
    normalized_prior["barrier"].pop("changed_field_from_005", None)
    normalized_current["barrier"].pop("changed_field_from_005", None)
    if normalized_prior != normalized_current:
        raise RuntimeError("scenario changed outside barrier half-width")


def main() -> None:
    if OUT.exists() or RESULT.exists() or EXECUTION_RECORD.exists():
        raise FileExistsError("refusing to overwrite formal artifacts, result, or execution record")
    config = json.loads(CFG.read_text(encoding="utf-8"))
    v0_result = load_frozen_json(config, "blocking_predecessor")
    isolation_result = load_frozen_json(config, "isolation_predecessor")
    if v0_result["status"] != "passed_pending_human_review":
        raise RuntimeError("v0 predecessor status is not frozen for this run")
    if isolation_result["observations"]["observation_terminal_assessment"] != config["isolation_predecessor"]["observed_terminal"]:
        raise RuntimeError("isolation predecessor terminal mismatch")
    screen_source = ROOT / config["position_screen_provenance"]["source_run_path"]
    if sha256_file(screen_source) != config["position_screen_provenance"]["source_run_sha256"]:
        raise RuntimeError("position-screen source-run hash mismatch")
    if sha256_file(SCENE) != config["model"]["scene_sha256"]:
        raise RuntimeError("candidate model hash mismatch")
    assert_only_barrier_width_changed(config)

    scenario = config["scenario"]
    recorder = RunRecorder(
        run_id=RID,
        objective="Change only virtual barrier lateral half-width after a preserved blocked candidate.",
        script=Path(__file__),
        repo_root=ROOT,
        output_root=ROOT / "research/runs",
        parameters=scenario,
        random_seed=None,
        inputs=[
            {"kind": "configuration", "path": CFG.relative_to(ROOT).as_posix(), "sha256": sha256_file(CFG)},
            {"kind": "predecessor_result", "path": config["blocking_predecessor"]["result_path"], "sha256": sha256_file(ROOT / config["blocking_predecessor"]["result_path"])},
            {"kind": "isolation_predecessor_result", "path": config["isolation_predecessor"]["result_path"], "sha256": sha256_file(ROOT / config["isolation_predecessor"]["result_path"])},
            {"kind": "position_screen_source_run", "path": screen_source.relative_to(ROOT).as_posix(), "sha256": sha256_file(screen_source)},
            {"kind": "candidate_model", "path": SCENE.relative_to(ROOT).as_posix(), "sha256": sha256_file(SCENE)},
        ],
        baseline={"kind": "barrier_width_isolation", "reference": config["isolation_predecessor"]["run_id"]},
        level="formal",
    )
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
        add_box(spec, scenario["barrier"]["name"], scenario["barrier"]["half_extents_m"], scenario["barrier"]["center_m"])
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
        cup_id = model.geom("vlab_cup").id
        barrier_id = model.geom(scenario["barrier"]["name"]).id
        cup_position_matches = bool(np.allclose(data.geom_xpos[cup_id], scenario["cup"]["center_m"], rtol=0, atol=1e-12))
        barrier_position_matches = bool(np.allclose(data.geom_xpos[barrier_id], scenario["barrier"]["center_m"], rtol=0, atol=1e-12))
        cup_inside_table = bool(abs(scenario["cup"]["center_m"][0]) + scenario["cup"]["radius_m"] <= scenario["table"]["half_extents_m"][0] and abs(scenario["cup"]["center_m"][1]) + scenario["cup"]["radius_m"] <= scenario["table"]["half_extents_m"][1])
        barrier_is_camera_side_of_cup = bool(scenario["barrier"]["center_m"][0] - scenario["barrier"]["half_extents_m"][0] > scenario["cup"]["center_m"][0])
        if not all((cup_position_matches, barrier_position_matches, cup_inside_table, barrier_is_camera_side_of_cup)):
            raise RuntimeError("barrier/cup coordinate or table-bound invariant failed")
        camera_observations = {}
        renderer = mujoco.Renderer(model, height=scenario["renderer"]["height"], width=scenario["renderer"]["width"])
        try:
            for camera_name in ("wrist_cam", "fixed_depth_01", "fixed_depth_02"):
                camera_observations[camera_name] = render_camera(renderer, data, camera_name, cup_id, OUT)
        finally:
            renderer.close()
        finite_depth = all(item["finite_depth_fraction"] == 1.0 for item in camera_observations.values())
        fixed_cameras_absent = all(camera_observations[name]["cup_segmentation_pixels"] == 0 for name in ("fixed_depth_01", "fixed_depth_02"))
        base_position = data.site_xpos[model.site("baseframe").id].copy()
        cup_xy_radius_from_base_m = float(np.linalg.norm(data.geom_xpos[cup_id][:2] - base_position[:2]))
        half_reach_radius_m = config["position_screen_provenance"]["virtual_half_reach_radius_m"]
        inside_position_screen = cup_xy_radius_from_base_m <= half_reach_radius_m
        assessment = {
            "observation_terminal_assessment": "not_observable_by_nominal_fixed_depth" if fixed_cameras_absent else "barrier_width_candidate_remains_observable_by_nominal_fixed_depth",
            "cup_position_invariant": cup_position_matches,
            "barrier_position_invariant": barrier_position_matches,
            "cup_within_table_bounds": cup_inside_table,
            "barrier_is_camera_side_of_cup": barrier_is_camera_side_of_cup,
            "camera_observations": camera_observations,
            "all_depth_values_finite": finite_depth,
            "position_screen": {"assessment": "inside_virtual_half_reach_screen" if inside_position_screen else "outside_virtual_half_reach_screen", "baseframe_world_position_m": base_position.tolist(), "cup_xy_radius_from_base_m": cup_xy_radius_from_base_m, "virtual_half_reach_radius_m": half_reach_radius_m, "source_run_id": config["position_screen_provenance"]["source_run_id"]},
            "grasp_reachability_assessment": "not_evaluated_no_deterministic_ik_collision_protocol",
        }
        (OUT / "terminal-assessment.json").write_bytes(strict_json_bytes(assessment))
        passed = fixed_cameras_absent and finite_depth and inside_position_screen
        result = {"schema_version": "first-robots/research-result/v1", "run_id": RID, "status": "passed_pending_human_review" if passed else "blocked_pending_human_review", "observations": assessment, "limits": config["non_claims"], "review": {"status": "pending_human_review", "adopted": False}}
        execution_record = {"schema_version": "first-robots/experiment-record/v1", "run_id": RID, "kind": "formal_virtual_laboratory_barrier_width_isolation", "result": {"path": RESULT.relative_to(ROOT).as_posix(), "sha256": sha256_bytes(strict_json_bytes(result))}, "review": {"status": "pending_human_review", "adopted": False}}
        write_json_pair(first_path=EXECUTION_RECORD, first_content=execution_record, second_path=RESULT, second_content=result)
        for artifact in sorted(OUT.iterdir()):
            recorder.add_artifact(artifact)
        recorder.add_artifact(EXECUTION_RECORD)
        recorder.add_artifact(RESULT)
        recorder.add_metric("fixed_cameras_cup_absent", fixed_cameras_absent, "boolean")
        recorder.add_metric("cup_xy_radius_from_base", cup_xy_radius_from_base_m, "m")
        recorder.add_metric("inside_virtual_half_reach_screen", inside_position_screen, "boolean")
        recorder.complete(passed=passed, criteria="Both fixed cameras emit no cup segmentation pixels; outputs are finite; cup stays inside the frozen virtual position screen.", review_status="pending_human_review", note="Single-variable barrier-width isolation; grasp reachability intentionally not evaluated.")


if __name__ == "__main__":
    main()
