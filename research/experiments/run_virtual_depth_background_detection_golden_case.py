"""Detect one virtual cup through fixed depth-background subtraction; no pose or execution."""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))

import mujoco
import numpy as np
from PIL import Image
from experiment_management.run import RunRecorder, sha256_bytes, strict_json_bytes, write_json_pair
from research.protocols.depth_background_detection import positive_depth_foreground_components
from research.protocols.mujoco_depth_frame_source import MujocoDepthFrameSource

RID="EXP-20260923-025-virtual-depth-background-detection-golden-case";CFG=ROOT/"research/configs"/f"{RID}.json";SCENE=ROOT/"research/sources/mujoco_menagerie_robotstudio_so101/robotstudio_so101/scene.xml";OUT=ROOT/"research/artifacts"/RID;RESULT=ROOT/"research/reports"/f"{RID}.result.json";EXECUTION_RECORD=ROOT/"research/experiments"/f"{RID}.execution.record.json"


def sha256_file(path:Path)->str:return "sha256:"+hashlib.sha256(path.read_bytes()).hexdigest()


def look_at_quaternion(position:np.ndarray,target:np.ndarray)->list[float]:
    z=-(target-position);z/=np.linalg.norm(z);x=np.cross(np.array([0.,0.,1.]),z);x/=np.linalg.norm(x);y=np.cross(z,x);output=np.empty(4);mujoco.mju_mat2Quat(output,np.column_stack((x,y,z)).reshape(9));return output.tolist()


def build_model(scenario:dict, include_cup:bool)->tuple[mujoco.MjModel,mujoco.MjData]:
    spec=mujoco.MjSpec.from_file(str(SCENE));table=spec.worldbody.add_geom();table.name="vlab_table";table.type=mujoco.mjtGeom.mjGEOM_BOX;table.pos=scenario["table"]["center_m"];table.size=scenario["table"]["half_extents_m"];table.rgba=scenario["table"]["rgba"];table.contype=0;table.conaffinity=0
    if include_cup:
        cup=spec.worldbody.add_geom();cup.name="vlab_cup_b";cup.type=mujoco.mjtGeom.mjGEOM_CYLINDER;cup.pos=scenario["cup_b"]["center_m"];cup.size=[scenario["cup_b"]["radius_m"],scenario["cup_b"]["half_height_m"],0];cup.rgba=scenario["cup_b"]["rgba"];cup.contype=0;cup.conaffinity=0
    contract=scenario["frame_contract"]
    for name,camera in scenario["fixed_cameras"].items():
        view=spec.worldbody.add_camera();view.name=name;view.pos=camera["position_m"];view.quat=look_at_quaternion(np.asarray(camera["position_m"]),np.asarray(camera["target_m"]));view.resolution=[contract["width"],contract["height"]];view.fovy=60
    model=spec.compile();data=mujoco.MjData(model);data.qpos[:]=scenario["robot_qpos_rad"];mujoco.mj_forward(model,data);return model,data


def main()->None:
    if OUT.exists() or RESULT.exists() or EXECUTION_RECORD.exists():raise FileExistsError("refusing to overwrite formal artifacts, result, or execution record")
    config=json.loads(CFG.read_text(encoding="utf-8"));scenario=config["scenario"];predecessor=ROOT/config["blocking_predecessor"]["result_path"]
    if sha256_file(predecessor)!=config["blocking_predecessor"]["result_sha256"] or sha256_file(SCENE)!=config["model"]["scene_sha256"]:raise RuntimeError("frozen input hash mismatch")
    recorder=RunRecorder(run_id=RID,objective="Verify virtual fixed-depth background subtraction detects one cup before pose or fusion work.",script=Path(__file__),repo_root=ROOT,output_root=ROOT/"research/runs",parameters=scenario,random_seed=None,inputs=[{"kind":"configuration","path":CFG.relative_to(ROOT).as_posix(),"sha256":sha256_file(CFG)},{"kind":"predecessor_result","path":predecessor.relative_to(ROOT).as_posix(),"sha256":sha256_file(predecessor)},{"kind":"candidate_model","path":SCENE.relative_to(ROOT).as_posix(),"sha256":sha256_file(SCENE)}],baseline={"kind":"virtual_depth_frame_contract","reference":config["blocking_predecessor"]["run_id"]},level="formal")
    with recorder:
        OUT.mkdir(parents=True);(OUT/"config.json").write_bytes(strict_json_bytes(config));reference_model,reference_data=build_model(scenario,False);current_model,current_data=build_model(scenario,True);contract=scenario["frame_contract"];barrier_absent=all(model.geom(i).name!="vlab_barrier" for model in (reference_model,current_model) for i in range(model.ngeom));cup_matches=bool(np.allclose(current_data.geom_xpos[current_model.geom("vlab_cup_b").id],scenario["cup_b"]["center_m"],rtol=0,atol=1e-12));camera_positions_match=all(np.allclose(reference_model.cam_pos[reference_model.camera(name).id],current_model.cam_pos[current_model.camera(name).id],rtol=0,atol=1e-12) for name in scenario["fixed_cameras"]);reference_source=MujocoDepthFrameSource(reference_model,reference_data,contract["width"],contract["height"],contract["scenario_version"]);current_source=MujocoDepthFrameSource(current_model,current_data,contract["width"],contract["height"],contract["scenario_version"]);detections={}
        for name in scenario["fixed_cameras"]:
            reference=reference_source.capture(name);current=current_source.capture(name);mask,components=positive_depth_foreground_components(reference,current,scenario["detector"]["virtual_numerical_epsilon_m"]);reference_path=OUT/f"{name}-reference-depth.npy";current_path=OUT/f"{name}-current-depth.npy";mask_path=OUT/f"{name}-foreground-mask.png";np.save(reference_path,reference.depth);np.save(current_path,current.depth);Image.fromarray(mask.astype(np.uint8)*255).save(mask_path);detections[name]={"reference_frame":reference.metadata(),"current_frame":current.metadata(),"component_count":len(components),"components":components,"candidate_pixel_count":int(mask.sum()),"reference_depth":reference_path.relative_to(ROOT).as_posix(),"current_depth":current_path.relative_to(ROOT).as_posix(),"foreground_mask":mask_path.relative_to(ROOT).as_posix()}
        observations={"barrier_absent":barrier_absent,"cup_b_position_matches_configuration":cup_matches,"camera_positions_match_between_reference_and_current":camera_positions_match,"frame_source_interface":"DepthFrameSource.capture(camera_name) -> DepthFrame","detector_input":"paired DepthFrame arrays only; no world coordinate, RGB, camera pose, MuJoCo model, or MuJoCo data argument","detections":detections,"depth_pose_or_fusion":"not_evaluated"};(OUT/"depth-background-detection-observations.json").write_bytes(strict_json_bytes(observations));valid=all(item["reference_frame"]==item["current_frame"] and item["reference_frame"]["source_kind"]==contract["source_kind"] and item["component_count"]==1 and item["candidate_pixel_count"]>0 for item in detections.values());passed=barrier_absent and cup_matches and camera_positions_match and valid
        result={"schema_version":"first-robots/research-result/v1","run_id":RID,"status":"passed_pending_human_review" if passed else "blocked_pending_human_review","observations":observations,"limits":config["non_claims"],"review":{"status":"pending_human_review","adopted":False}};record={"schema_version":"first-robots/experiment-record/v1","run_id":RID,"kind":"formal_virtual_depth_background_detection_golden_case","result":{"path":RESULT.relative_to(ROOT).as_posix(),"sha256":sha256_bytes(strict_json_bytes(result))},"review":{"status":"pending_human_review","adopted":False}};write_json_pair(first_path=EXECUTION_RECORD,first_content=record,second_path=RESULT,second_content=result)
        for artifact in sorted(OUT.iterdir()):recorder.add_artifact(artifact)
        recorder.add_artifact(EXECUTION_RECORD);recorder.add_artifact(RESULT);recorder.add_metric("depth_background_detection_valid",passed,"boolean");recorder.add_metric("camera_count",len(detections),"count");recorder.complete(passed=passed,criteria="Both paired virtual depth frames produce one positive foreground component.",review_status="pending_human_review",note="Virtual background-difference detection only; no real capture policy, pose, fusion, or execution claim.")


if __name__=="__main__":main()
