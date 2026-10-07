"""Fit one known virtual upright-cylinder center from frozen dual-depth foreground points."""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))

import numpy as np
from PIL import Image
from experiment_management.run import RunRecorder, sha256_bytes, strict_json_bytes, write_json_pair
from research.protocols.known_cylinder_depth_fit import fit_xy_circle, foreground_pixels_to_world_points

RID="EXP-20260923-028-known-cylinder-depth-center-fit-golden-case";CFG=ROOT/"research/configs"/f"{RID}.json";OUT=ROOT/"research/artifacts"/RID;RESULT=ROOT/"research/reports"/f"{RID}.result.json";EXECUTION_RECORD=ROOT/"research/experiments"/f"{RID}.execution.record.json"


def sha256_file(path:Path)->str:return "sha256:"+hashlib.sha256(path.read_bytes()).hexdigest()
def world_from_camera(position:np.ndarray,target:np.ndarray)->np.ndarray:
    z=-(target-position);z/=np.linalg.norm(z);x=np.cross(np.array([0.,0.,1.]),z);x/=np.linalg.norm(x);y=np.cross(z,x);return np.column_stack((x,y,z))


def main()->None:
    if OUT.exists() or RESULT.exists() or EXECUTION_RECORD.exists():raise FileExistsError("refusing to overwrite formal artifacts, result, or execution record")
    config=json.loads(CFG.read_text(encoding="utf-8"));scenario=config["scenario"];prior=config["blocking_predecessor"];prior_result=ROOT/prior["result_path"];foreground=ROOT/prior["foreground_observations_path"]
    if sha256_file(prior_result)!=prior["result_sha256"] or sha256_file(foreground)!=prior["foreground_observations_sha256"]:raise RuntimeError("frozen input hash mismatch")
    recorder=RunRecorder(run_id=RID,objective="Fit a known virtual upright cylinder center from frozen dual-depth foreground points before coordinator integration.",script=Path(__file__),repo_root=ROOT,output_root=ROOT/"research/runs",parameters=scenario,random_seed=None,inputs=[{"kind":"configuration","path":CFG.relative_to(ROOT).as_posix(),"sha256":sha256_file(CFG)},{"kind":"predecessor_result","path":prior_result.relative_to(ROOT).as_posix(),"sha256":sha256_file(prior_result)},{"kind":"foreground_observations","path":foreground.relative_to(ROOT).as_posix(),"sha256":sha256_file(foreground)}],baseline={"kind":"corrected_virtual_depth_surface_projection","reference":prior["run_id"]},level="formal")
    with recorder:
        OUT.mkdir(parents=True);(OUT/"config.json").write_bytes(strict_json_bytes(config));source=json.loads(foreground.read_text(encoding="utf-8"));contract=scenario["frame_contract"];per_camera={};all_points=[]
        for name,camera in scenario["fixed_cameras"].items():
            detection=source["detections"][name];depth=np.load(ROOT/detection["current_depth"]);mask=np.asarray(Image.open(ROOT/detection["foreground_mask"]))>0;points=foreground_pixels_to_world_points(depth,mask,contract["width"],contract["height"],scenario["camera_fovy_degrees"],np.asarray(camera["position_m"]),world_from_camera(np.asarray(camera["position_m"]),np.asarray(camera["target_m"])));per_camera[name]={"frame":detection["current_frame"],"foreground_point_count":int(len(points)),"finite":bool(np.isfinite(points).all()),"world_z_min_m":float(points[:,2].min()),"world_z_max_m":float(points[:,2].max())};all_points.append(points)
        points=np.vstack(all_points);median_z=float(np.median(points[:,2]));side=points[points[:,2]<median_z];center_xy,fitted_radius,residual=fit_xy_circle(side);known=scenario["known_cylinder"];center=np.array([center_xy[0],center_xy[1],known["support_plane_z_m"]+known["half_height_m"]]);truth=np.asarray(scenario["cup_b_world_m_evaluation_only"]);delta=center-truth;np.save(OUT/"combined-foreground-points.npy",points);np.save(OUT/"side-fit-points.npy",side);fit={"combined_point_count":int(len(points)),"combined_z_median_m":median_z,"side_point_count":int(len(side)),"fitted_center_world_m":center.tolist(),"fitted_radius_m":fitted_radius,"known_radius_m":known["radius_m"],"circle_radial_rms_residual_m":residual,"evaluation_only_delta_from_b_m":delta.tolist(),"evaluation_only_center_error_m":float(np.linalg.norm(delta))};observations={"source_run":prior["run_id"],"known_cylinder":known,"per_camera":per_camera,"fit":fit,"coordinator_or_execution":"not_evaluated"};(OUT/"known-cylinder-fit-observations.json").write_bytes(strict_json_bytes(observations));valid=all(item["frame"]["source_kind"]==contract["source_kind"] and item["frame"]["scenario_version"]==contract["scenario_version"] and item["frame"]["pixel_encoding"]==contract["pixel_encoding"] and item["finite"] and item["foreground_point_count"]>0 for item in per_camera.values()) and len(side)>=3 and np.isfinite(center).all() and fitted_radius>0 and np.isfinite(residual);passed=bool(valid)
        result={"schema_version":"first-robots/research-result/v1","run_id":RID,"status":"passed_pending_human_review" if passed else "blocked_pending_human_review","observations":observations,"limits":config["non_claims"],"review":{"status":"pending_human_review","adopted":False}};record={"schema_version":"first-robots/experiment-record/v1","run_id":RID,"kind":"formal_known_cylinder_depth_center_fit_golden_case","result":{"path":RESULT.relative_to(ROOT).as_posix(),"sha256":sha256_bytes(strict_json_bytes(result))},"review":{"status":"pending_human_review","adopted":False}};write_json_pair(first_path=EXECUTION_RECORD,first_content=record,second_path=RESULT,second_content=result)
        for artifact in sorted(OUT.iterdir()):recorder.add_artifact(artifact)
        recorder.add_artifact(EXECUTION_RECORD);recorder.add_artifact(RESULT);recorder.add_metric("known_cylinder_center_fit_valid",passed,"boolean");recorder.complete(passed=passed,criteria="Dual frozen depth foreground point clouds yield finite known-cylinder circle fit; evaluation error is recorded only.",review_status="pending_human_review",note="Virtual known-cylinder center fit only; no tolerance, Coordinator, execution, or hardware capability claim.")


if __name__=="__main__":main()
