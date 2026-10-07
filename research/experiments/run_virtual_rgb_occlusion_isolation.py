"""Isolate virtual robot-render occlusion for one failed range-sweep case; no fusion or execution."""
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
from research.protocols.rgb_frame_contract_v2 import candidate_magenta_components

RID="EXP-20260921-023-virtual-rgb-occlusion-isolation";CFG=ROOT/"research/configs"/f"{RID}.json";SCENE=ROOT/"research/sources/mujoco_menagerie_robotstudio_so101/robotstudio_so101/scene.xml";OUT=ROOT/"research/artifacts"/RID;RESULT=ROOT/"research/reports"/f"{RID}.result.json";EXECUTION_RECORD=ROOT/"research/experiments"/f"{RID}.execution.record.json"


def sha256_file(path:Path)->str:return "sha256:"+hashlib.sha256(path.read_bytes()).hexdigest()


def look_at_quaternion(position:np.ndarray,target:np.ndarray)->list[float]:
    z=-(target-position);z/=np.linalg.norm(z);x=np.cross(np.array([0.,0.,1.]),z);x/=np.linalg.norm(x);y=np.cross(z,x);output=np.empty(4);mujoco.mju_mat2Quat(output,np.column_stack((x,y,z)).reshape(9));return output.tolist()


def main()->None:
    if OUT.exists() or RESULT.exists() or EXECUTION_RECORD.exists():raise FileExistsError("refusing to overwrite formal artifacts, result, or execution record")
    config=json.loads(CFG.read_text(encoding="utf-8"));scenario=config["scenario"];predecessor=ROOT/config["blocking_predecessor"]["result_path"]
    if sha256_file(predecessor)!=config["blocking_predecessor"]["result_sha256"]:raise RuntimeError("frozen predecessor hash mismatch")
    recorder=RunRecorder(run_id=RID,objective="Isolate whether virtual robot render occlusion explains the B-10 RGB fixture incompleteness.",script=Path(__file__),repo_root=ROOT,output_root=ROOT/"research/runs",parameters=scenario,random_seed=None,inputs=[{"kind":"configuration","path":CFG.relative_to(ROOT).as_posix(),"sha256":sha256_file(CFG)},{"kind":"failed_sweep_result","path":predecessor.relative_to(ROOT).as_posix(),"sha256":sha256_file(predecessor)},{"kind":"candidate_model","path":SCENE.relative_to(ROOT).as_posix(),"sha256":sha256_file(SCENE)}],baseline={"kind":"failed_virtual_rgb_range_sweep","reference":config["blocking_predecessor"]["run_id"]},level="formal")
    with recorder:
        OUT.mkdir(parents=True);(OUT/"config.json").write_bytes(strict_json_bytes(config));spec=mujoco.MjSpec.from_file(str(SCENE));table=spec.worldbody.add_geom();table.name="vlab_table";table.type=mujoco.mjtGeom.mjGEOM_BOX;table.pos=scenario["table"]["center_m"];table.size=scenario["table"]["half_extents_m"];table.rgba=scenario["table"]["rgba"];table.contype=0;table.conaffinity=0;cup=spec.worldbody.add_geom();cup.name="vlab_cup";cup.type=mujoco.mjtGeom.mjGEOM_CYLINDER;cup.pos=scenario["target_case"]["cup_center_m"];cup.size=[scenario["cup"]["radius_m"],scenario["cup"]["half_height_m"],0];cup.rgba=scenario["cup"]["rgba"];cup.contype=0;cup.conaffinity=0;contract=scenario["frame_contract"]
        for name,camera in scenario["fixed_cameras"].items():
            view=spec.worldbody.add_camera();view.name=name;view.pos=camera["position_m"];view.quat=look_at_quaternion(np.asarray(camera["position_m"]),np.asarray(camera["target_m"]));view.resolution=[contract["width"],contract["height"]];view.fovy=60
        model=spec.compile();data=mujoco.MjData(model);data.qpos[:]=scenario["robot_qpos_rad"];mujoco.mj_forward(model,data);table_id=model.geom("vlab_table").id;cup_id=model.geom("vlab_cup").id;robot_geom_ids=[index for index in range(model.ngeom) if index not in {table_id,cup_id}];barrier_absent=all(model.geom(index).name!="vlab_barrier" for index in range(model.ngeom));source=MujocoRgbFrameSource(model,data,contract["width"],contract["height"],contract["scenario_version"]);observations={"barrier_absent":barrier_absent,"target_case":scenario["target_case"],"robot_render_disabled_geom_count":len(robot_geom_ids),"cameras":{}}
        for variant in scenario["render_variants"]:
            if variant=="robot_render_disabled": model.geom_rgba[robot_geom_ids,3]=0
            for camera_name in scenario["fixed_cameras"]:
                frame=source.capture(camera_name);mask,components=candidate_magenta_components(frame,scenario["detector"]);stem=f"{camera_name}-{variant}";raw=OUT/f"{stem}-rgb.png";mask_path=OUT/f"{stem}-mask.png";Image.fromarray(frame.rgb).save(raw);Image.fromarray(mask.astype(np.uint8)*255).save(mask_path);observations["cameras"].setdefault(camera_name,{})[variant]={"frame":frame.metadata(),"component_count":len(components),"components":components,"candidate_pixel_count":int(mask.sum()),"raw_rgb":raw.relative_to(ROOT).as_posix(),"mask":mask_path.relative_to(ROOT).as_posix()}
        for camera in observations["cameras"].values():
            visible=camera["robot_visible"]["candidate_pixel_count"];unoccluded=camera["robot_render_disabled"]["candidate_pixel_count"];camera["visible_to_render_disabled_pixel_ratio"]=visible/unoccluded if unoccluded else None
        (OUT/"occlusion-isolation-observations.json").write_bytes(strict_json_bytes(observations));valid=all(camera["robot_visible"]["component_count"]==1 and camera["robot_render_disabled"]["component_count"]==1 and camera["robot_render_disabled"]["candidate_pixel_count"]>0 and camera["robot_visible"]["candidate_pixel_count"]<=camera["robot_render_disabled"]["candidate_pixel_count"] for camera in observations["cameras"].values());passed=barrier_absent and valid
        result={"schema_version":"first-robots/research-result/v1","run_id":RID,"status":"passed_pending_human_review" if passed else "blocked_pending_human_review","observations":observations,"limits":config["non_claims"],"review":{"status":"pending_human_review","adopted":False}};record={"schema_version":"first-robots/experiment-record/v1","run_id":RID,"kind":"formal_virtual_rgb_occlusion_isolation","result":{"path":RESULT.relative_to(ROOT).as_posix(),"sha256":sha256_bytes(strict_json_bytes(result))},"review":{"status":"pending_human_review","adopted":False}};write_json_pair(first_path=EXECUTION_RECORD,first_content=record,second_path=RESULT,second_content=result)
        for artifact in sorted(OUT.iterdir()):recorder.add_artifact(artifact)
        recorder.add_artifact(EXECUTION_RECORD);recorder.add_artifact(RESULT);recorder.add_metric("occlusion_isolation_valid",passed,"boolean");recorder.complete(passed=passed,criteria="Each B-10 camera comparison yields one component and visible candidate pixels do not exceed robot-render-disabled pixels.",review_status="pending_human_review",note="Single virtual render-appearance isolation only; no physical occlusion, fusion, or execution claim.")


if __name__=="__main__":main()
