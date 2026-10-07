"""Repair only MuJoCo planar-depth projection; surface candidates are not object centers."""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))

import numpy as np
from PIL import Image
from experiment_management.run import RunRecorder, sha256_bytes, strict_json_bytes, write_json_pair
from research.protocols.depth_plane_projection_v2 import depth_pixel_to_world_surface_from_camera_plane

RID="EXP-20260923-027-virtual-depth-plane-projection-repair";CFG=ROOT/"research/configs"/f"{RID}.json";OUT=ROOT/"research/artifacts"/RID;RESULT=ROOT/"research/reports"/f"{RID}.result.json";EXECUTION_RECORD=ROOT/"research/experiments"/f"{RID}.execution.record.json"


def sha256_file(path:Path)->str:return "sha256:"+hashlib.sha256(path.read_bytes()).hexdigest()
def world_from_camera(position:np.ndarray,target:np.ndarray)->np.ndarray:
    z=-(target-position);z/=np.linalg.norm(z);x=np.cross(np.array([0.,0.,1.]),z);x/=np.linalg.norm(x);y=np.cross(z,x);return np.column_stack((x,y,z))


def main()->None:
    if OUT.exists() or RESULT.exists() or EXECUTION_RECORD.exists():raise FileExistsError("refusing to overwrite formal artifacts, result, or execution record")
    config=json.loads(CFG.read_text(encoding="utf-8"));scenario=config["scenario"];prior=config["blocking_predecessor"];failed=config["repair_of"];result_path=ROOT/prior["result_path"];observations_path=ROOT/prior["observations_path"];failed_path=ROOT/failed["result_path"]
    if any(sha256_file(path)!=expected for path,expected in ((result_path,prior["result_sha256"]),(observations_path,prior["observations_sha256"]),(failed_path,failed["result_sha256"]))):raise RuntimeError("frozen input hash mismatch")
    recorder=RunRecorder(run_id=RID,objective="Correct virtual MuJoCo planar-depth projection while retaining the same frozen depth foreground inputs.",script=Path(__file__),repo_root=ROOT,output_root=ROOT/"research/runs",parameters=scenario,random_seed=None,inputs=[{"kind":"configuration","path":CFG.relative_to(ROOT).as_posix(),"sha256":sha256_file(CFG)},{"kind":"predecessor_result","path":result_path.relative_to(ROOT).as_posix(),"sha256":sha256_file(result_path)},{"kind":"predecessor_observations","path":observations_path.relative_to(ROOT).as_posix(),"sha256":sha256_file(observations_path)},{"kind":"failed_projection_result","path":failed_path.relative_to(ROOT).as_posix(),"sha256":sha256_file(failed_path)}],baseline={"kind":"virtual_depth_background_detection","reference":prior["run_id"]},level="formal")
    with recorder:
        OUT.mkdir(parents=True);(OUT/"config.json").write_bytes(strict_json_bytes(config));prior_observations=json.loads(observations_path.read_text(encoding="utf-8"));truth=np.asarray(scenario["cup_b_world_m_evaluation_only"]);contract=scenario["frame_contract"];measurements={}
        for name,camera in scenario["fixed_cameras"].items():
            detection=prior_observations["detections"][name];frame=detection["current_frame"];depth=np.load(ROOT/detection["current_depth"]);mask=np.asarray(Image.open(ROOT/detection["foreground_mask"]))>0;component=detection["components"][0] if detection["component_count"]==1 else None;samples=depth[mask];median=float(np.median(samples)) if len(samples) else float("nan");point,camera_local=depth_pixel_to_world_surface_from_camera_plane(component["centroid_pixel_xy"],median,contract["width"],contract["height"],scenario["camera_fovy_degrees"],np.asarray(camera["position_m"]),world_from_camera(np.asarray(camera["position_m"]),np.asarray(camera["target_m"])));delta=point-truth;measurements[name]={"frame":frame,"component_count":detection["component_count"],"component":component,"depth_sample_count":int(len(samples)),"foreground_planar_depth_median_m":median,"surface_point_world_m":point.tolist(),"camera_local_surface_point_m":camera_local.tolist(),"evaluation_only_delta_from_cup_center_m":delta.tolist(),"evaluation_only_distance_from_cup_center_m":float(np.linalg.norm(delta))}
        observations={"source_run":prior["run_id"],"repaired_semantics":"depth is distance from camera plane","cup_b_world_m_evaluation_only":truth.tolist(),"measurements":measurements,"center_fit_or_fusion":"not_evaluated"};(OUT/"depth-plane-projection-observations.json").write_bytes(strict_json_bytes(observations));valid=all(item["frame"]["source_kind"]==contract["source_kind"] and item["frame"]["scenario_version"]==contract["scenario_version"] and item["frame"]["pixel_encoding"]==contract["pixel_encoding"] and item["component_count"]==1 and item["depth_sample_count"]>0 and np.isfinite(item["foreground_planar_depth_median_m"]) and all(np.isfinite(item["surface_point_world_m"])) for item in measurements.values());passed=valid
        result={"schema_version":"first-robots/research-result/v1","run_id":RID,"status":"passed_pending_human_review" if passed else "blocked_pending_human_review","observations":observations,"limits":config["non_claims"],"review":{"status":"pending_human_review","adopted":False}};record={"schema_version":"first-robots/experiment-record/v1","run_id":RID,"kind":"formal_virtual_depth_plane_projection_repair","result":{"path":RESULT.relative_to(ROOT).as_posix(),"sha256":sha256_bytes(strict_json_bytes(result))},"review":{"status":"pending_human_review","adopted":False}};write_json_pair(first_path=EXECUTION_RECORD,first_content=record,second_path=RESULT,second_content=result)
        for artifact in sorted(OUT.iterdir()):recorder.add_artifact(artifact)
        recorder.add_artifact(EXECUTION_RECORD);recorder.add_artifact(RESULT);recorder.add_metric("depth_plane_projection_valid",passed,"boolean");recorder.complete(passed=passed,criteria="Both frozen foreground components produce finite planar-depth surface candidates; center offsets are recorded only.",review_status="pending_human_review",note="Corrected virtual depth surface points only; no center fit, fusion, Coordinator, execution, or hardware claim.")


if __name__=="__main__":main()
