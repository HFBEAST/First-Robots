"""Run a fresh virtual dual-depth known-cylinder X step through the mock agent chain."""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import mujoco
import numpy as np
from PIL import Image
from experiment_management.run import RunRecorder, sha256_bytes, strict_json_bytes, write_json_pair
from research.protocols.coordinate_transfer_v2 import mock_execute, table_contains_cup
from research.protocols.depth_background_detection import positive_depth_foreground_components
from research.protocols.depth_coordinate_agent import coordinate_message, coordinator_accepts
from research.protocols.known_cylinder_depth_fit import fit_xy_circle, foreground_pixels_to_world_points
from research.protocols.mujoco_depth_frame_source import MujocoDepthFrameSource


RID = "EXP-20260923-030-known-cylinder-depth-x-step-agent-chain"
CFG = ROOT / "research/configs" / f"{RID}.json"
SCENE = ROOT / "research/sources/mujoco_menagerie_robotstudio_so101/robotstudio_so101/scene.xml"
OUT = ROOT / "research/artifacts" / RID
RESULT = ROOT / "research/reports" / f"{RID}.result.json"
EXECUTION_RECORD = ROOT / "research/experiments" / f"{RID}.execution.record.json"


def sha256_file(path: Path) -> str:
    return "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()


def look_at_quaternion(position: np.ndarray, target: np.ndarray) -> list[float]:
    z = -(target - position)
    z /= np.linalg.norm(z)
    x = np.cross(np.array([0.0, 0.0, 1.0]), z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    output = np.empty(4)
    mujoco.mju_mat2Quat(output, np.column_stack((x, y, z)).reshape(9))
    return output.tolist()


def world_from_camera(position: np.ndarray, target: np.ndarray) -> np.ndarray:
    z = -(target - position)
    z /= np.linalg.norm(z)
    x = np.cross(np.array([0.0, 0.0, 1.0]), z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack((x, y, z))


def build_model(scenario: dict, cup_center_m: list[float] | None) -> tuple[mujoco.MjModel, mujoco.MjData]:
    spec = mujoco.MjSpec.from_file(str(SCENE))
    table = spec.worldbody.add_geom()
    table.name = "vlab_table"
    table.type = mujoco.mjtGeom.mjGEOM_BOX
    table.pos = scenario["table"]["center_m"]
    table.size = scenario["table"]["half_extents_m"]
    table.rgba = scenario["table"]["rgba"]
    table.contype = 0
    table.conaffinity = 0
    if cup_center_m is not None:
        cup = spec.worldbody.add_geom()
        cup.name = "vlab_cup_b"
        cup.type = mujoco.mjtGeom.mjGEOM_CYLINDER
        cup.pos = cup_center_m
        cup.size = [scenario["cup"]["radius_m"], scenario["cup"]["half_height_m"], 0]
        cup.rgba = scenario["cup"]["rgba"]
        cup.contype = 0
        cup.conaffinity = 0
    contract = scenario["frame_contract"]
    for name, camera in scenario["fixed_cameras"].items():
        view = spec.worldbody.add_camera()
        view.name = name
        view.pos = camera["position_m"]
        view.quat = look_at_quaternion(np.asarray(camera["position_m"]), np.asarray(camera["target_m"]))
        view.resolution = [contract["width"], contract["height"]]
        view.fovy = scenario["camera_fovy_degrees"]
    model = spec.compile()
    data = mujoco.MjData(model)
    data.qpos[:] = scenario["robot_qpos_rad"]
    mujoco.mj_forward(model, data)
    return model, data


def run_condition(scenario: dict, center_truth: np.ndarray, output: Path) -> tuple[dict, bool]:
    reference_model, reference_data = build_model(scenario, None)
    current_model, current_data = build_model(scenario, center_truth.tolist())
    contract = scenario["frame_contract"]
    reference_source = MujocoDepthFrameSource(reference_model, reference_data, contract["width"], contract["height"], contract["scenario_version"])
    current_source = MujocoDepthFrameSource(current_model, current_data, contract["width"], contract["height"], contract["scenario_version"])
    per_camera, all_points = {}, []
    for name, camera in scenario["fixed_cameras"].items():
        reference = reference_source.capture(name)
        current = current_source.capture(name)
        mask, components = positive_depth_foreground_components(reference, current, scenario["detector"]["virtual_numerical_epsilon_m"])
        prefix = f"x-{center_truth[0]:.3f}-{name}"
        reference_path = output / f"{prefix}-reference-depth.npy"
        current_path = output / f"{prefix}-current-depth.npy"
        mask_path = output / f"{prefix}-foreground-mask.png"
        np.save(reference_path, reference.depth)
        np.save(current_path, current.depth)
        Image.fromarray(mask.astype(np.uint8) * 255).save(mask_path)
        detection_valid = bool(
            reference.metadata() == current.metadata()
            and reference.metadata()["source_kind"] == contract["source_kind"]
            and reference.metadata()["pixel_encoding"] == contract["pixel_encoding"]
            and len(components) == 1
            and mask.any()
        )
        points = np.empty((0, 3))
        if detection_valid:
            points = foreground_pixels_to_world_points(
                current.depth,
                mask,
                contract["width"],
                contract["height"],
                scenario["camera_fovy_degrees"],
                np.asarray(camera["position_m"]),
                world_from_camera(np.asarray(camera["position_m"]), np.asarray(camera["target_m"])),
            )
            all_points.append(points)
        per_camera[name] = {
            "reference_frame": reference.metadata(),
            "current_frame": current.metadata(),
            "component_count": len(components),
            "candidate_pixel_count": int(mask.sum()),
            "finite_world_points": bool(len(points) > 0 and np.isfinite(points).all()),
            "reference_depth": reference_path.relative_to(ROOT).as_posix(),
            "current_depth": current_path.relative_to(ROOT).as_posix(),
            "foreground_mask": mask_path.relative_to(ROOT).as_posix(),
        }
    detection_valid = bool(all(item["component_count"] == 1 and item["candidate_pixel_count"] > 0 and item["finite_world_points"] for item in per_camera.values()))
    fit = None
    candidate_center = None
    if detection_valid:
        combined = np.vstack(all_points)
        side = combined[combined[:, 2] < np.median(combined[:, 2])]
        if len(side) >= 3:
            center_xy, fitted_radius, residual = fit_xy_circle(side)
            candidate_center = np.array([center_xy[0], center_xy[1], scenario["cup"]["half_height_m"]])
            fit = {
                "combined_point_count": int(len(combined)),
                "side_point_count": int(len(side)),
                "fitted_center_world_m": candidate_center.tolist(),
                "fitted_radius_m": fitted_radius,
                "circle_radial_rms_residual_m": residual,
                "evaluation_only_delta_from_configured_center_m": (candidate_center - center_truth).tolist(),
                "evaluation_only_center_error_m": float(np.linalg.norm(candidate_center - center_truth)),
            }
    fitted_center_is_finite = bool(candidate_center is not None and np.isfinite(candidate_center).all())
    message = coordinate_message(candidate_center.tolist() if fitted_center_is_finite else [], scenario["required_depth_frames"], scenario["scenario_version"], scenario["known_object_model_version"])
    coordinator_accepted, coordinator_reason, coordinator_center = coordinator_accepts(message, set(scenario["required_depth_frames"]), scenario["scenario_version"], scenario["known_object_model_version"])
    capability_accepted = bool(
        coordinator_accepted
        and table_contains_cup(
            coordinator_center,
            np.asarray(scenario["table"]["center_m"]),
            np.asarray(scenario["table"]["half_extents_m"]),
            scenario["cup"]["radius_m"],
            scenario["cup"]["half_height_m"],
        )
    )
    state = {"cup_center_m": scenario["initial_cup_a_world_m"][:]}
    executor_applied, executor_reason = mock_execute(state, capability_accepted, candidate_center if fitted_center_is_finite else center_truth)
    reviewer_equal = bool(executor_applied and coordinator_center is not None and np.allclose(state["cup_center_m"], coordinator_center, rtol=0, atol=1e-12))
    valid = bool(detection_valid and fitted_center_is_finite and coordinator_accepted and capability_accepted and executor_applied and reviewer_equal)
    return {
        "configured_cylinder_center_world_m": center_truth.tolist(),
        "compiled_cup_center_matches_configuration": bool(np.allclose(current_data.geom_xpos[current_model.geom("vlab_cup_b").id], center_truth, rtol=0, atol=1e-12)),
        "per_camera": per_camera,
        "fit": fit,
        "coordinate_message": message,
        "coordinator": {"accepted": coordinator_accepted, "reason": coordinator_reason},
        "capability_table_containment_only": capability_accepted,
        "executor": {"applied": executor_applied, "reason": executor_reason, "state_after_m": state["cup_center_m"]},
        "reviewer_state_equals_candidate": reviewer_equal,
    }, valid


def main() -> None:
    if OUT.exists() or RESULT.exists() or EXECUTION_RECORD.exists():
        raise FileExistsError("refusing to overwrite formal artifacts, result, or execution record")
    config = json.loads(CFG.read_text(encoding="utf-8"))
    scenario = config["scenario"]
    predecessor = config["blocking_predecessor"]
    predecessor_result = ROOT / predecessor["result_path"]
    if sha256_file(predecessor_result) != predecessor["result_sha256"] or sha256_file(SCENE) != config["model"]["scene_sha256"]:
        raise RuntimeError("frozen predecessor or model hash mismatch")
    recorder = RunRecorder(
        run_id=RID,
        objective="Change only the known virtual cylinder world X across three small conditions and run fresh dual-depth detection-to-mock-agent chains.",
        script=Path(__file__), repo_root=ROOT, output_root=ROOT / "research/runs", parameters=scenario, random_seed=None,
        inputs=[
            {"kind": "configuration", "path": CFG.relative_to(ROOT).as_posix(), "sha256": sha256_file(CFG)},
            {"kind": "predecessor_result", "path": predecessor_result.relative_to(ROOT).as_posix(), "sha256": sha256_file(predecessor_result)},
            {"kind": "candidate_model", "path": SCENE.relative_to(ROOT).as_posix(), "sha256": sha256_file(SCENE)},
        ],
        baseline={"kind": "known_cylinder_depth_agent_chain_golden_case", "reference": predecessor["run_id"]}, level="formal",
    )
    with recorder:
        OUT.mkdir(parents=True)
        (OUT / "config.json").write_bytes(strict_json_bytes(config))
        observations, condition_validity = [], []
        for x in scenario["x_values_m"]:
            truth = np.array([x, scenario["frozen_y_m"], scenario["frozen_center_z_m"]], dtype=float)
            condition, valid = run_condition(scenario, truth, OUT)
            observations.append(condition)
            condition_validity.append(valid)
        configured_centers = [item["configured_cylinder_center_world_m"] for item in observations]
        only_x_varies = bool(all(center[1] == scenario["frozen_y_m"] and center[2] == scenario["frozen_center_z_m"] for center in configured_centers))
        payload = {"source_run": predecessor["run_id"], "changed_independent_variable": scenario["changed_independent_variable"], "only_x_varies": only_x_varies, "conditions": observations}
        (OUT / "x-step-agent-chain-observations.json").write_bytes(strict_json_bytes(payload))
        passed = bool(only_x_varies and all(condition_validity))
        result = {"schema_version": "first-robots/research-result/v1", "run_id": RID, "status": "passed_pending_human_review" if passed else "blocked_pending_human_review", "observations": payload, "limits": config["non_claims"], "review": {"status": "pending_human_review", "adopted": False}}
        record = {"schema_version": "first-robots/experiment-record/v1", "run_id": RID, "kind": "formal_known_cylinder_depth_x_step_agent_chain", "result": {"path": RESULT.relative_to(ROOT).as_posix(), "sha256": sha256_bytes(strict_json_bytes(result))}, "review": {"status": "pending_human_review", "adopted": False}}
        write_json_pair(first_path=EXECUTION_RECORD, first_content=record, second_path=RESULT, second_content=result)
        for artifact in sorted(OUT.iterdir()):
            recorder.add_artifact(artifact)
        recorder.add_artifact(EXECUTION_RECORD)
        recorder.add_artifact(RESULT)
        recorder.add_metric("known_cylinder_depth_x_step_agent_chain_valid", passed, "boolean")
        recorder.add_metric("x_condition_count", len(observations), "count")
        recorder.complete(passed=passed, criteria="For every declared X condition, freshly rendered paired depth frames support one-component detection, finite known-cylinder fit, complete coordination, table-only gate, mock execution, and reviewer equality.", review_status="pending_human_review", note="Small virtual X-only known-cylinder step; no real sensing, calibrated accuracy, reachability, collision, grasping, planning, safety, robot action, or hardware capability claim.")


if __name__ == "__main__":
    main()
