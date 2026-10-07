"""Measure virtual RGB centroid-to-plane projections; does not set an acceptance tolerance or execute."""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]; sys.path.insert(0,str(ROOT))

import mujoco
import numpy as np
from PIL import Image
from experiment_management.run import RunRecorder, sha256_bytes, strict_json_bytes, write_json_pair
from research.protocols.mujoco_rgb_frame_source import MujocoRgbFrameSource
from research.protocols.pixel_plane_projection import pixel_centroid_to_world_plane
from research.protocols.rgb_frame_contract_v2 import candidate_magenta_components

RID="EXP-20260921-021-virtual-rgb-centroid-plane-projection"; CFG=ROOT/"research/configs"/f"{RID}.json"; SCENE=ROOT/"research/sources/mujoco_menagerie_robotstudio_so101/robotstudio_so101/scene.xml"; OUT=ROOT/"research/artifacts"/RID; RESULT=ROOT/"research/reports"/f"{RID}.result.json"; EXECUTION_RECORD=ROOT/"research/experiments"/f"{RID}.execution.record.json"


def sha256_file(path:Path)->str:return "sha256:"+hashlib.sha256(path.read_bytes()).hexdigest()


def look_at_quaternion(position:np.ndarray,target:np.ndarray)->list[float]:
    z=-(target-position);z/=np.linalg.norm(z);x=np.cross(np.array([0.,0.,1.]),z);x/=np.linalg.norm(x);y=np.cross(z,x);output=np.empty(4);mujoco.mju_mat2Quat(output,np.column_stack((x,y,z)).reshape(9));return output.tolist()


def main()->None:
    if OUT.exists() or RESULT.exists() or EXECUTION_RECORD.exists():raise FileExistsError("refusing to overwrite formal artifacts, result, or execution record")
    config=json.loads(CFG.read_text(encoding="utf-8"));scenario=config["scenario"];predecessor=ROOT/config["blocking_predecessor"]["result_path"]
    if sha256_file(predecessor)!=config["blocking_predecessor"]["result_sha256"] or sha256_file(SCENE)!=config["model"]["scene_sha256"]:raise RuntimeError("frozen input hash mismatch")
    recorder=RunRecorder(run_id=RID,objective="Record RGB component centroid rays intersecting a known-height virtual plane without an accuracy gate or execution.",script=Path(__file__),repo_root=ROOT,output_root=ROOT/"research/runs",parameters=scenario,random_seed=None,inputs=[{"kind":"configuration","path":CFG.relative_to(ROOT).as_posix(),"sha256":sha256_file(CFG)},{"kind":"predecessor_result","path":predecessor.relative_to(ROOT).as_posix(),"sha256":sha256_file(predecessor)},{"kind":"candidate_model","path":SCENE.relative_to(ROOT).as_posix(),"sha256":sha256_file(SCENE)}],baseline={"kind":"virtual_rgb_detection_isolation","reference":config["blocking_predecessor"]["run_id"]},level="formal")
    with recorder:
        OUT.mkdir(parents=True);(OUT/"config.json").write_bytes(strict_json_bytes(config));spec=mujoco.MjSpec.from_file(str(SCENE));table=spec.worldbody.add_geom();table.name="vlab_table";table.type=mujoco.mjtGeom.mjGEOM_BOX;table.pos=scenario["table"]["center_m"];table.size=scenario["table"]["half_extents_m"];table.rgba=scenario["table"]["rgba"];table.contype=0;table.conaffinity=0;cup=spec.worldbody.add_geom();cup.name="vlab_cup_b";cup.type=mujoco.mjtGeom.mjGEOM_CYLINDER;cup.pos=scenario["cup_b"]["center_m"];cup.size=[scenario["cup_b"]["radius_m"],scenario["cup_b"]["half_height_m"],0];cup.rgba=scenario["cup_b"]["rgba"];cup.contype=0;cup.conaffinity=0;contract=scenario["frame_contract"]
        for name,camera in scenario["fixed_cameras"].items():
            view=spec.worldbody.add_camera();view.name=name;view.pos=camera["position_m"];view.quat=look_at_quaternion(np.asarray(camera["position_m"]),np.asarray(camera["target_m"]));view.resolution=[contract["width"],contract["height"]];view.fovy=scenario["projection"]["camera_fovy_degrees"]
        model=spec.compile();data=mujoco.MjData(model);data.qpos[:]=scenario["robot_qpos_rad"];mujoco.mj_forward(model,data);barrier_absent=all(model.geom(i).name!="vlab_barrier" for i in range(model.ngeom));cup_matches=bool(np.allclose(data.geom_xpos[model.geom("vlab_cup_b").id],scenario["cup_b"]["center_m"],rtol=0,atol=1e-12));source=MujocoRgbFrameSource(model,data,contract["width"],contract["height"],contract["scenario_version"]);measurements={};truth=np.asarray(scenario["cup_b"]["center_m"])
        for name in scenario["fixed_cameras"]:
            frame=source.capture(name);mask,components=candidate_magenta_components(frame,scenario["detector"]);raw=OUT/f"{name}-rgb.png";mask_path=OUT/f"{name}-mask.png";Image.fromarray(frame.rgb).save(raw);Image.fromarray(mask.astype(np.uint8)*255).save(mask_path);camera_id=model.camera(name).id;estimate,distance=pixel_centroid_to_world_plane(components[0]["centroid_pixel_xy"],contract["width"],contract["height"],scenario["projection"]["camera_fovy_degrees"],model.cam_pos[camera_id],model.cam_mat0[camera_id].reshape(3,3),scenario["projection"]["assumed_cup_center_plane_z_m"]);delta=estimate-truth;measurements[name]={"frame":frame.metadata(),"component_count":len(components),"component":components[0] if len(components)==1 else None,"projected_world_m":estimate.tolist(),"camera_forward_distance_m":distance,"evaluation_only_delta_from_b_m":delta.tolist(),"evaluation_only_euclidean_error_m":float(np.linalg.norm(delta)),"raw_rgb":raw.relative_to(ROOT).as_posix(),"mask":mask_path.relative_to(ROOT).as_posix()}
        observations={"barrier_absent":barrier_absent,"cup_b_world_m_evaluation_only":truth.tolist(),"cup_b_position_matches_configuration":cup_matches,"projection_assumption":{"plane_z_m":scenario["projection"]["assumed_cup_center_plane_z_m"]},"measurements":measurements,"coordinator_or_execution":"not_evaluated"};(OUT/"centroid-plane-projection-observations.json").write_bytes(strict_json_bytes(observations));valid=all(item["component_count"]==1 and np.isfinite(item["camera_forward_distance_m"]) and item["camera_forward_distance_m"]>0 and all(np.isfinite(item["projected_world_m"])) for item in measurements.values());passed=barrier_absent and cup_matches and valid
        result={"schema_version":"first-robots/research-result/v1","run_id":RID,"status":"passed_pending_human_review" if passed else "blocked_pending_human_review","observations":observations,"limits":config["non_claims"],"review":{"status":"pending_human_review","adopted":False}};record={"schema_version":"first-robots/experiment-record/v1","run_id":RID,"kind":"formal_virtual_rgb_centroid_plane_projection","result":{"path":RESULT.relative_to(ROOT).as_posix(),"sha256":sha256_bytes(strict_json_bytes(result))},"review":{"status":"pending_human_review","adopted":False}};write_json_pair(first_path=EXECUTION_RECORD,first_content=record,second_path=RESULT,second_content=result)
        for artifact in sorted(OUT.iterdir()):recorder.add_artifact(artifact)
        recorder.add_artifact(EXECUTION_RECORD);recorder.add_artifact(RESULT);recorder.add_metric("centroid_plane_projection_valid",passed,"boolean");recorder.add_metric("camera_count",len(measurements),"count");recorder.complete(passed=passed,criteria="Each isolated RGB centroid produces a finite, forward virtual plane intersection; deviations are recorded without a tolerance.",review_status="pending_human_review",note="Virtual geometric measurement only; not a calibrated pose, fusion, acceptance gate, or robot action.")


if __name__=="__main__":main()
