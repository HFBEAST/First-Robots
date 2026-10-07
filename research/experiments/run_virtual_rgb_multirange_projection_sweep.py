"""Sweep frozen virtual B coordinates through RGB-only detection and plane projection; never fuse or execute."""
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
from research.protocols.mujoco_rgb_frame_source import MujocoRgbFrameSource
from research.protocols.pixel_plane_projection import pixel_centroid_to_world_plane
from research.protocols.rgb_frame_contract_v2 import candidate_magenta_components

RID="EXP-20260921-022-virtual-rgb-multirange-projection-sweep";CFG=ROOT/"research/configs"/f"{RID}.json";SCENE=ROOT/"research/sources/mujoco_menagerie_robotstudio_so101/robotstudio_so101/scene.xml";OUT=ROOT/"research/artifacts"/RID;RESULT=ROOT/"research/reports"/f"{RID}.result.json";EXECUTION_RECORD=ROOT/"research/experiments"/f"{RID}.execution.record.json"


def sha256_file(path:Path)->str:return "sha256:"+hashlib.sha256(path.read_bytes()).hexdigest()


def look_at_quaternion(position:np.ndarray,target:np.ndarray)->list[float]:
    z=-(target-position);z/=np.linalg.norm(z);x=np.cross(np.array([0.,0.,1.]),z);x/=np.linalg.norm(x);y=np.cross(z,x);output=np.empty(4);mujoco.mju_mat2Quat(output,np.column_stack((x,y,z)).reshape(9));return output.tolist()


def main()->None:
    if OUT.exists() or RESULT.exists() or EXECUTION_RECORD.exists():raise FileExistsError("refusing to overwrite formal artifacts, result, or execution record")
    config=json.loads(CFG.read_text(encoding="utf-8"));scenario=config["scenario"];predecessor=ROOT/config["blocking_predecessor"]["result_path"]
    if sha256_file(predecessor)!=config["blocking_predecessor"]["result_sha256"] or sha256_file(SCENE)!=config["model"]["scene_sha256"]:raise RuntimeError("frozen input hash mismatch")
    recorder=RunRecorder(run_id=RID,objective="Measure range-wide virtual RGB centroid plane projections before choosing a virtual fusion rule.",script=Path(__file__),repo_root=ROOT,output_root=ROOT/"research/runs",parameters=scenario,random_seed=None,inputs=[{"kind":"configuration","path":CFG.relative_to(ROOT).as_posix(),"sha256":sha256_file(CFG)},{"kind":"predecessor_result","path":predecessor.relative_to(ROOT).as_posix(),"sha256":sha256_file(predecessor)},{"kind":"candidate_model","path":SCENE.relative_to(ROOT).as_posix(),"sha256":sha256_file(SCENE)}],baseline={"kind":"virtual_rgb_centroid_plane_projection","reference":config["blocking_predecessor"]["run_id"]},level="formal")
    with recorder:
        OUT.mkdir(parents=True);(OUT/"config.json").write_bytes(strict_json_bytes(config));spec=mujoco.MjSpec.from_file(str(SCENE));table=spec.worldbody.add_geom();table.name="vlab_table";table.type=mujoco.mjtGeom.mjGEOM_BOX;table.pos=scenario["table"]["center_m"];table.size=scenario["table"]["half_extents_m"];table.rgba=scenario["table"]["rgba"];table.contype=0;table.conaffinity=0;cup=spec.worldbody.add_geom();cup.name="vlab_cup";cup.type=mujoco.mjtGeom.mjGEOM_CYLINDER;cup.pos=scenario["cup"]["initial_center_m"];cup.size=[scenario["cup"]["radius_m"],scenario["cup"]["half_height_m"],0];cup.rgba=scenario["cup"]["rgba"];cup.contype=0;cup.conaffinity=0;contract=scenario["frame_contract"]
        for name,camera in scenario["fixed_cameras"].items():
            view=spec.worldbody.add_camera();view.name=name;view.pos=camera["position_m"];view.quat=look_at_quaternion(np.asarray(camera["position_m"]),np.asarray(camera["target_m"]));view.resolution=[contract["width"],contract["height"]];view.fovy=scenario["projection"]["camera_fovy_degrees"]
        model=spec.compile();data=mujoco.MjData(model);data.qpos[:]=scenario["robot_qpos_rad"];barrier_absent=all(model.geom(i).name!="vlab_barrier" for i in range(model.ngeom));cup_id=model.geom("vlab_cup").id;source=MujocoRgbFrameSource(model,data,contract["width"],contract["height"],contract["scenario_version"]);targets=[np.array([x,y,scenario["target_b_grid_m"]["z"]],dtype=float) for x in scenario["target_b_grid_m"]["x"] for y in scenario["target_b_grid_m"]["y"]];cases=[]
        for index,target in enumerate(targets,1):
            model.geom_pos[cup_id]=target;mujoco.mj_forward(model,data);per_camera={}
            for camera_name in scenario["fixed_cameras"]:
                frame=source.capture(camera_name);mask,components=candidate_magenta_components(frame,scenario["detector"]);stem=f"B-{index:02d}-{camera_name}";raw=OUT/f"{stem}-rgb.png";mask_path=OUT/f"{stem}-mask.png";Image.fromarray(frame.rgb).save(raw);Image.fromarray(mask.astype(np.uint8)*255).save(mask_path);measurement={"frame":frame.metadata(),"component_count":len(components),"components":components,"raw_rgb":raw.relative_to(ROOT).as_posix(),"mask":mask_path.relative_to(ROOT).as_posix()}
                if len(components)==1:
                    camera_id=model.camera(camera_name).id;estimate,distance=pixel_centroid_to_world_plane(components[0]["centroid_pixel_xy"],contract["width"],contract["height"],scenario["projection"]["camera_fovy_degrees"],model.cam_pos[camera_id],model.cam_mat0[camera_id].reshape(3,3),scenario["projection"]["assumed_cup_center_plane_z_m"]);delta=estimate-target;measurement.update({"projected_world_m":estimate.tolist(),"camera_forward_distance_m":distance,"evaluation_only_delta_from_b_m":delta.tolist(),"evaluation_only_euclidean_error_m":float(np.linalg.norm(delta))})
                per_camera[camera_name]=measurement
            estimates=[np.asarray(item["projected_world_m"]) for item in per_camera.values() if "projected_world_m" in item];cases.append({"case_id":f"B-{index:02d}","target_b_world_m":target.tolist(),"cup_position_matches_configuration":bool(np.allclose(data.geom_xpos[cup_id],target,rtol=0,atol=1e-12)),"per_camera":per_camera,"two_camera_disagreement_m":float(np.linalg.norm(estimates[0]-estimates[1])) if len(estimates)==2 else None})
        valid_cases=[case for case in cases if len([item for item in case["per_camera"].values() if item["component_count"]==1 and "projected_world_m" in item])==2 and case["cup_position_matches_configuration"]];errors=[item["evaluation_only_euclidean_error_m"] for case in valid_cases for item in case["per_camera"].values()];disagreements=[case["two_camera_disagreement_m"] for case in valid_cases];observations={"barrier_absent":barrier_absent,"target_count":len(targets),"valid_two_camera_case_count":len(valid_cases),"cases":cases,"summary_evaluation_only":{"max_single_camera_error_m":max(errors) if errors else None,"max_two_camera_disagreement_m":max(disagreements) if disagreements else None,"mean_two_camera_disagreement_m":float(np.mean(disagreements)) if disagreements else None},"fusion_or_execution":"not_evaluated"};(OUT/"multirange-projection-observations.json").write_bytes(strict_json_bytes(observations));passed=barrier_absent and len(valid_cases)==len(targets)
        result={"schema_version":"first-robots/research-result/v1","run_id":RID,"status":"passed_pending_human_review" if passed else "blocked_pending_human_review","observations":observations,"limits":config["non_claims"],"review":{"status":"pending_human_review","adopted":False}};record={"schema_version":"first-robots/experiment-record/v1","run_id":RID,"kind":"formal_virtual_rgb_multirange_projection_sweep","result":{"path":RESULT.relative_to(ROOT).as_posix(),"sha256":sha256_bytes(strict_json_bytes(result))},"review":{"status":"pending_human_review","adopted":False}};write_json_pair(first_path=EXECUTION_RECORD,first_content=record,second_path=RESULT,second_content=result)
        for artifact in sorted(OUT.iterdir()):recorder.add_artifact(artifact)
        recorder.add_artifact(EXECUTION_RECORD);recorder.add_artifact(RESULT);recorder.add_metric("valid_two_camera_case_count",len(valid_cases),"count");recorder.add_metric("multirange_projection_valid",passed,"boolean");recorder.complete(passed=passed,criteria="All 25 targets yield one RGB component and finite forward plane projections in both cameras; errors are recorded only.",review_status="pending_human_review",note="Synthetic multirange measurement only; no tolerance, fusion, Coordinator, execution, or hardware claim.")


if __name__=="__main__":main()
