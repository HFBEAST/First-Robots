"""Correct the formal table-height gate and re-run the virtual message contract only."""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import mujoco
import numpy as np
from PIL import Image, ImageDraw
from experiment_management.run import RunRecorder, sha256_bytes, strict_json_bytes, write_json_pair
from research.protocols.coordinate_transfer_v2 import (CameraMessage, camera_to_world,
    coordinate_agent_accepts, mock_execute, table_contains_cup, world_to_camera)

RID = "EXP-20260921-017-no-barrier-multirange-coordinate-agent-protocol-repair"
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
    y_axis = np.cross(z_axis, x_axis)
    output = np.empty(4); mujoco.mju_mat2Quat(output, np.column_stack((x_axis, y_axis, z_axis)).reshape(9))
    return output.tolist()


def camera_message(name: str, target_world: np.ndarray, model: mujoco.MjModel, version: str) -> CameraMessage:
    camera_id = model.camera(name).id; world_from_camera = model.cam_mat0[camera_id].reshape(3, 3)
    return CameraMessage(agent="detection", source="simulator_truth", frame_id=name, scenario_version=version, point_camera_m=tuple(world_to_camera(target_world, model.cam_pos[camera_id], world_from_camera)))


def coordinate_coverage_map(path: Path, scenario: dict, targets: list[np.ndarray]) -> None:
    size, margin, half = 720, 80, scenario["table"]["half_extents_m"][0]
    image = Image.new("RGB", (size, size), "#102235"); draw = ImageDraw.Draw(image)
    def pixel(point: np.ndarray) -> tuple[float, float]: return size / 2 + point[0] / half * (size / 2 - margin), size / 2 - point[1] / half * (size / 2 - margin)
    draw.rectangle((margin, margin, size-margin, size-margin), outline="#d5b97c", width=3)
    for value in scenario["target_b_grid_m"]["x"]:
        x, _ = pixel(np.array([value, 0, 0])); _, y = pixel(np.array([0, value, 0])); draw.line((x,margin,x,size-margin),fill="#314a60"); draw.line((margin,y,size-margin,y),fill="#314a60")
    base = pixel(np.asarray(scenario["baseframe_world_position_m"])); draw.ellipse((base[0]-7,base[1]-7,base[0]+7,base[1]+7),fill="#f4d35e"); draw.text((base[0]+10,base[1]+10),"base (0,0)",fill="white")
    for target in targets:
        x, y = pixel(target); draw.ellipse((x-7,y-7,x+7,y+7),fill="#3ad6d0")
    source = pixel(np.asarray(scenario["cup_a_center_m"])); draw.ellipse((source[0]-10,source[1]-10,source[0]+10,source[1]+10),fill="#e450a9"); draw.text((source[0]+12,source[1]+10),"A",fill="white")
    draw.text((margin, 30), "Virtual coordinate coverage: 25 B points; no reachability claim", fill="white"); image.save(path)


def main() -> None:
    if OUT.exists() or RESULT.exists() or EXECUTION_RECORD.exists(): raise FileExistsError("refusing to overwrite formal artifacts, result, or execution record")
    config = json.loads(CFG.read_text(encoding="utf-8")); scenario = config["scenario"]
    predecessor = ROOT / config["blocking_predecessor"]["result_path"]; failed = ROOT / config["repair_of"]["result_path"]
    if any(sha256_file(path) != expected for path, expected in ((predecessor, config["blocking_predecessor"]["result_sha256"]), (failed, config["repair_of"]["result_sha256"]), (SCENE, config["model"]["scene_sha256"]))): raise RuntimeError("frozen input hash mismatch")
    recorder = RunRecorder(run_id=RID, objective="Repair one table-height predicate and retest virtual multi-Agent coordinates plus mock state transitions.", script=Path(__file__), repo_root=ROOT, output_root=ROOT / "research/runs", parameters=scenario, random_seed=None, inputs=[{"kind":"configuration","path":CFG.relative_to(ROOT).as_posix(),"sha256":sha256_file(CFG)}, {"kind":"predecessor_result","path":predecessor.relative_to(ROOT).as_posix(),"sha256":sha256_file(predecessor)}, {"kind":"failed_result","path":failed.relative_to(ROOT).as_posix(),"sha256":sha256_file(failed)}, {"kind":"candidate_model","path":SCENE.relative_to(ROOT).as_posix(),"sha256":sha256_file(SCENE)}], baseline={"kind":"no_barrier_coordinate_baseline","reference":config["blocking_predecessor"]["run_id"]}, level="formal")
    with recorder:
        OUT.mkdir(parents=True); (OUT / "config.json").write_bytes(strict_json_bytes(config)); spec = mujoco.MjSpec.from_file(str(SCENE))
        for name, camera in scenario["fixed_cameras"].items():
            view = spec.worldbody.add_camera(); view.name = name; view.pos = camera["position_m"]; view.quat = look_at_quaternion(np.asarray(camera["position_m"]), np.asarray(camera["target_m"]))
        model = spec.compile(); data = mujoco.MjData(model); data.qpos[:] = scenario["robot_qpos_rad"]; mujoco.mj_forward(model, data)
        barrier_absent = all(model.geom(index).name != "vlab_barrier" for index in range(model.ngeom)); version = f"{RID}:coordinate-contract-v2"; frames = set(scenario["fixed_cameras"])
        targets = [np.array([x, y, scenario["target_b_grid_m"]["z"]], dtype=float) for x in scenario["target_b_grid_m"]["x"] for y in scenario["target_b_grid_m"]["y"]]
        table_center = np.asarray(scenario["table"]["center_m"]); table_half = np.asarray(scenario["table"]["half_extents_m"]); positive = []
        for index, target in enumerate(targets, 1):
            messages = [camera_message(name, target, model, version) for name in scenario["fixed_cameras"]]
            coordinates = [camera_to_world(message, model.cam_pos[model.camera(message.frame_id).id], model.cam_mat0[model.camera(message.frame_id).id].reshape(3,3)) for message in messages]
            coordinated, coordination_reason, recovered = coordinate_agent_accepts(coordinates, frames, version)
            capability = bool(coordinated and table_contains_cup(recovered, table_center, table_half, scenario["cup_radius_m"], scenario["cup_half_height_m"]))
            state = {"cup_center_m":scenario["cup_a_center_m"][:]}; executed, execution_reason = mock_execute(state, capability, recovered if recovered is not None else target); reviewed = bool(executed and np.allclose(state["cup_center_m"], target, rtol=0, atol=1e-12))
            positive.append({"case_id":f"B-{index:02d}","target_b_world_m":target.tolist(),"coordinate_agent":{"accepted":coordinated,"reason":coordination_reason},"capability_agent":{"table_contained":capability},"execution_agent":{"executed":executed,"reason":execution_reason},"evidence_reviewer":{"virtual_state_matches_b":reviewed}})
        source = camera_message("fixed_depth_01", targets[0], model, version); wrong = {"frame_id":scenario["negative_cases"]["wrong_frame"]["frame_id"],"scenario_version":version,"point_world_m":targets[0].tolist()}; wrong_ok, wrong_reason, _ = coordinate_agent_accepts([wrong], frames, version); wrong_state = {"cup_center_m":scenario["cup_a_center_m"][:]}; wrong_executed, wrong_execution_reason = mock_execute(wrong_state, wrong_ok, targets[0])
        outside = np.asarray(scenario["negative_cases"]["outside_table_b_m"],dtype=float); outside_messages = [camera_message(name,outside,model,version) for name in scenario["fixed_cameras"]]; outside_coordinates = [camera_to_world(message,model.cam_pos[model.camera(message.frame_id).id],model.cam_mat0[model.camera(message.frame_id).id].reshape(3,3)) for message in outside_messages]; outside_ok,outside_reason,outside_recovered = coordinate_agent_accepts(outside_coordinates,frames,version); outside_capability = bool(outside_ok and table_contains_cup(outside_recovered,table_center,table_half,scenario["cup_radius_m"],scenario["cup_half_height_m"])); outside_state={"cup_center_m":scenario["cup_a_center_m"][:]}; outside_executed,outside_execution_reason=mock_execute(outside_state,outside_capability,outside)
        negative={"wrong_frame":{"coordinator_accepted":wrong_ok,"reason":wrong_reason,"execution_agent":{"executed":wrong_executed,"reason":wrong_execution_reason},"state_unchanged":wrong_state["cup_center_m"]==scenario["cup_a_center_m"]},"outside_table":{"coordinator_accepted":outside_ok,"coordinator_reason":outside_reason,"capability_table_contained":outside_capability,"execution_agent":{"executed":outside_executed,"reason":outside_execution_reason},"state_unchanged":outside_state["cup_center_m"]==scenario["cup_a_center_m"]}}
        coordinate_coverage_map(OUT / "target-b-coordinate-coverage.png", scenario, targets)
        observations={"barrier_absent":barrier_absent,"scenario_version":version,"roles":{"detection":"simulator-truth camera-coordinate emitter","coordinate":"compiled-extrinsic inverse transform","coordinator":"frame/version/pair agreement gate","capability":"table containment only","execution":"mock in-memory state transition only","reviewer":"final virtual state equality"},"positive_cases":positive,"negative_cases":negative,"coverage":{"target_count":len(targets),"x_m":scenario["target_b_grid_m"]["x"],"y_m":scenario["target_b_grid_m"]["y"],"z_m":scenario["target_b_grid_m"]["z"],"map":"research/artifacts/"+RID+"/target-b-coordinate-coverage.png"}}
        (OUT / "coordinate-agent-protocol-observations.json").write_bytes(strict_json_bytes(observations)); positives_pass=all(item["coordinate_agent"]["accepted"] and item["capability_agent"]["table_contained"] and item["execution_agent"]["executed"] and item["evidence_reviewer"]["virtual_state_matches_b"] for item in positive); negatives_pass=not wrong_ok and not wrong_executed and negative["wrong_frame"]["state_unchanged"] and outside_ok and not outside_capability and not outside_executed and negative["outside_table"]["state_unchanged"]; passed=barrier_absent and positives_pass and negatives_pass
        result={"schema_version":"first-robots/research-result/v1","run_id":RID,"status":"passed_pending_human_review" if passed else "blocked_pending_human_review","observations":observations,"limits":config["non_claims"],"review":{"status":"pending_human_review","adopted":False}}; execution_record={"schema_version":"first-robots/experiment-record/v1","run_id":RID,"kind":"formal_no_barrier_multirange_coordinate_agent_protocol_repair","result":{"path":RESULT.relative_to(ROOT).as_posix(),"sha256":sha256_bytes(strict_json_bytes(result))},"review":{"status":"pending_human_review","adopted":False}}
        write_json_pair(first_path=EXECUTION_RECORD,first_content=execution_record,second_path=RESULT,second_content=result)
        for artifact in sorted(OUT.iterdir()): recorder.add_artifact(artifact)
        recorder.add_artifact(EXECUTION_RECORD); recorder.add_artifact(RESULT); recorder.add_metric("positive_target_count",len(targets),"count"); recorder.add_metric("protocol_contract_valid",passed,"boolean"); recorder.complete(passed=passed,criteria="All fixed B messages round trip, gate, mock transition, and review; both negative cases reject without state change.",review_status="pending_human_review",note="Virtual coordinate-message protocol only; no perception, planning, IK, collision, grasp, robot action, or hardware control.")


if __name__ == "__main__": main()
