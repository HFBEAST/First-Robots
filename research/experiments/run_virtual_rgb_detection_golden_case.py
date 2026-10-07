"""Capture virtual RGB frames through the interchangeable frame contract; no hardware is used."""
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
from research.protocols.mujoco_rgb_frame_source import MujocoRgbFrameSource
from research.protocols.rgb_frame_contract import candidate_magenta_detection

RID = "EXP-20260921-019-virtual-rgb-detection-golden-case"
CFG = ROOT / "research/configs" / f"{RID}.json"
SCENE = ROOT / "research/sources/mujoco_menagerie_robotstudio_so101/robotstudio_so101/scene.xml"
OUT = ROOT / "research/artifacts" / RID
RESULT = ROOT / "research/reports" / f"{RID}.result.json"
EXECUTION_RECORD = ROOT / "research/experiments" / f"{RID}.execution.record.json"


def sha256_file(path: Path) -> str:
    return "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()


def look_at_quaternion(position: np.ndarray, target: np.ndarray) -> list[float]:
    z_axis = -(target - position); z_axis /= np.linalg.norm(z_axis)
    x_axis = np.cross(np.array([0.0, 0.0, 1.0]), z_axis); x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis); output = np.empty(4)
    mujoco.mju_mat2Quat(output, np.column_stack((x_axis, y_axis, z_axis)).reshape(9))
    return output.tolist()


def main() -> None:
    if OUT.exists() or RESULT.exists() or EXECUTION_RECORD.exists(): raise FileExistsError("refusing to overwrite formal artifacts, result, or execution record")
    config = json.loads(CFG.read_text(encoding="utf-8")); scenario = config["scenario"]; predecessor = ROOT / config["blocking_predecessor"]["result_path"]
    if sha256_file(predecessor) != config["blocking_predecessor"]["result_sha256"] or sha256_file(SCENE) != config["model"]["scene_sha256"]: raise RuntimeError("frozen input hash mismatch")
    recorder = RunRecorder(run_id=RID, objective="Verify a virtual RGB frame adapter and RGB-only artificial-color detector in one fixed golden case.", script=Path(__file__), repo_root=ROOT, output_root=ROOT / "research/runs", parameters=scenario, random_seed=None, inputs=[{"kind":"configuration","path":CFG.relative_to(ROOT).as_posix(),"sha256":sha256_file(CFG)},{"kind":"predecessor_result","path":predecessor.relative_to(ROOT).as_posix(),"sha256":sha256_file(predecessor)},{"kind":"candidate_model","path":SCENE.relative_to(ROOT).as_posix(),"sha256":sha256_file(SCENE)}], baseline={"kind":"no_barrier_coordinate_agent_protocol","reference":config["blocking_predecessor"]["run_id"]}, level="formal")
    with recorder:
        OUT.mkdir(parents=True); (OUT / "config.json").write_bytes(strict_json_bytes(config)); spec = mujoco.MjSpec.from_file(str(SCENE))
        table = spec.worldbody.add_geom(); table.name = "vlab_table"; table.type = mujoco.mjtGeom.mjGEOM_BOX; table.pos = scenario["table"]["center_m"]; table.size = scenario["table"]["half_extents_m"]; table.rgba = scenario["table"]["rgba"]; table.contype = 0; table.conaffinity = 0
        cup = spec.worldbody.add_geom(); cup.name = "vlab_cup_b"; cup.type = mujoco.mjtGeom.mjGEOM_CYLINDER; cup.pos = scenario["cup_b"]["center_m"]; cup.size = [scenario["cup_b"]["radius_m"],scenario["cup_b"]["half_height_m"],0]; cup.rgba = scenario["cup_b"]["rgba"]; cup.contype = 0; cup.conaffinity = 0
        contract = scenario["frame_contract"]
        for name, camera in scenario["fixed_cameras"].items():
            view = spec.worldbody.add_camera(); view.name = name; view.pos = camera["position_m"]; view.quat = look_at_quaternion(np.asarray(camera["position_m"]),np.asarray(camera["target_m"])); view.resolution = [contract["width"],contract["height"]]; view.fovy = 60
        model = spec.compile(); data = mujoco.MjData(model); data.qpos[:] = scenario["robot_qpos_rad"]; mujoco.mj_forward(model,data)
        barrier_absent = all(model.geom(index).name != "vlab_barrier" for index in range(model.ngeom)); cup_position_matches = bool(np.allclose(data.geom_xpos[model.geom("vlab_cup_b").id],scenario["cup_b"]["center_m"],rtol=0,atol=1e-12))
        source = MujocoRgbFrameSource(model,data,contract["width"],contract["height"],contract["scenario_version"]); detector = scenario["detector"]; detections = {}
        for camera_name in scenario["fixed_cameras"]:
            frame = source.capture(camera_name); mask, detection = candidate_magenta_detection(frame,detector["red_min"],detector["green_max"],detector["blue_min"])
            raw_path = OUT / f"{camera_name}-rgb.png"; mask_path = OUT / f"{camera_name}-mask.png"; overlay_path = OUT / f"{camera_name}-overlay.png"; Image.fromarray(frame.rgb).save(raw_path); Image.fromarray((mask.astype(np.uint8)*255)).save(mask_path)
            overlay = frame.rgb.copy(); overlay[mask] = np.array([255,255,255],dtype=np.uint8); Image.fromarray(overlay).save(overlay_path)
            detection["frame"] = frame.metadata(); detection["raw_rgb"] = raw_path.relative_to(ROOT).as_posix(); detection["mask"] = mask_path.relative_to(ROOT).as_posix(); detection["overlay"] = overlay_path.relative_to(ROOT).as_posix(); detection["frame_nonuniform"] = bool(np.any(frame.rgb != frame.rgb[0,0])); detections[camera_name] = detection
        observations = {"barrier_absent":barrier_absent,"cup_b_world_m_evaluation_only":scenario["cup_b"]["center_m"],"cup_b_position_matches_configuration":cup_position_matches,"frame_source_interface":"RgbFrameSource.capture(camera_name) -> RgbFrame","detector_input":"RgbFrame only; no world coordinate, camera pose, MuJoCo model, or MuJoCo data argument","detections":detections}
        (OUT / "rgb-detection-observations.json").write_bytes(strict_json_bytes(observations))
        frames_valid = all(item["frame"]["source_kind"] == contract["source_kind"] and item["frame"]["scenario_version"] == contract["scenario_version"] and item["frame"]["pixel_encoding"] == contract["pixel_encoding"] and item["frame"]["width"] == contract["width"] and item["frame"]["height"] == contract["height"] and item["frame_nonuniform"] for item in detections.values())
        detected = all(item["pixel_count"] > 0 and item["centroid_pixel_xy"] is not None for item in detections.values()); passed = barrier_absent and cup_position_matches and frames_valid and detected
        result = {"schema_version":"first-robots/research-result/v1","run_id":RID,"status":"passed_pending_human_review" if passed else "blocked_pending_human_review","observations":observations,"limits":config["non_claims"],"review":{"status":"pending_human_review","adopted":False}}; record = {"schema_version":"first-robots/experiment-record/v1","run_id":RID,"kind":"formal_virtual_rgb_detection_golden_case","result":{"path":RESULT.relative_to(ROOT).as_posix(),"sha256":sha256_bytes(strict_json_bytes(result))},"review":{"status":"pending_human_review","adopted":False}}
        write_json_pair(first_path=EXECUTION_RECORD,first_content=record,second_path=RESULT,second_content=result)
        for artifact in sorted(OUT.iterdir()): recorder.add_artifact(artifact)
        recorder.add_artifact(EXECUTION_RECORD); recorder.add_artifact(RESULT); recorder.add_metric("rgb_detection_golden_case_valid",passed,"boolean"); recorder.add_metric("camera_count",len(detections),"count"); recorder.complete(passed=passed,criteria="Both virtual RGB frames conform and contain positive artificial signature detections.",review_status="pending_human_review",note="Virtual RGB fixture and adapter contract only; no RGB-to-world pose or hardware implication.")


if __name__ == "__main__": main()
