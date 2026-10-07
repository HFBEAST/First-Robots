"""Test one static virtual not-observable candidate; never controls hardware."""
from __future__ import annotations
import hashlib,json
from pathlib import Path
import mujoco
import numpy as np
from PIL import Image
from experiment_management.run import RunRecorder,sha256_bytes,strict_json_bytes,write_json_pair
RID="EXP-20260921-003-virtual-laboratory-not-observable-candidate";ROOT=Path(__file__).resolve().parents[2];CFG=ROOT/"research/configs"/f"{RID}.json";SCENE=ROOT/"research/sources/mujoco_menagerie_robotstudio_so101/robotstudio_so101/scene.xml";OUT=ROOT/"research/artifacts"/RID;RESULT=ROOT/"research/reports"/f"{RID}.result.json";RECORD=ROOT/"research/experiments"/f"{RID}.record.json"
def h(p:Path)->str:return "sha256:"+hashlib.sha256(p.read_bytes()).hexdigest()
def quat(pos,target):
 z=-(target-pos);z/=np.linalg.norm(z);x=np.cross(np.array([0.,0.,1.]),z);x/=np.linalg.norm(x);y=np.cross(z,x);q=np.empty(4);mujoco.mju_mat2Quat(q,np.column_stack((x,y,z)).reshape(9));return q.tolist()
def main():
 if OUT.exists() or RESULT.exists() or RECORD.exists():raise FileExistsError("refusing overwrite")
 c=json.loads(CFG.read_text());prior=ROOT/c["blocking_predecessor"]["result_path"]
 if h(prior)!=c["blocking_predecessor"]["result_sha256"] or json.loads(prior.read_text())["status"]!="passed_pending_human_review":raise RuntimeError("v0 predecessor is not frozen")
 if h(SCENE)!=c["model"]["scene_sha256"]:raise RuntimeError("model hash mismatch")
 s=c["scenario"];rec=RunRecorder(run_id=RID,objective="Test one table-contained virtual cup position as a not-observable candidate.",script=Path(__file__),repo_root=ROOT,output_root=ROOT/"research/runs",parameters=s,random_seed=None,inputs=[{"kind":"configuration","path":CFG.relative_to(ROOT).as_posix(),"sha256":h(CFG)},{"kind":"predecessor_result","path":prior.relative_to(ROOT).as_posix(),"sha256":h(prior)},{"kind":"model","path":SCENE.relative_to(ROOT).as_posix(),"sha256":h(SCENE)}],baseline={"kind":"virtual_lab_v0","reference":c["blocking_predecessor"]["run_id"]},level="formal")
 with rec:
  OUT.mkdir(parents=True);(OUT/"config.json").write_bytes(strict_json_bytes(c));spec=mujoco.MjSpec.from_file(str(SCENE));t=spec.worldbody.add_geom();t.name="vlab_table";t.type=mujoco.mjtGeom.mjGEOM_BOX;t.size=s["table"]["half_extents_m"];t.pos=s["table"]["center_m"];cup=spec.worldbody.add_geom();cup.name="vlab_cup";cup.type=mujoco.mjtGeom.mjGEOM_CYLINDER;cup.size=[s["cup_radius_m"],s["cup_half_height_m"],0];cup.pos=s["cup_center_m"]
  for n,d in s["fixed_cameras"].items():cam=spec.worldbody.add_camera();cam.name=n;cam.pos=d["position_m"];cam.quat=quat(np.asarray(d["position_m"]),np.asarray(d["target_m"]));cam.resolution=[320,240];cam.fovy=60
  m=spec.compile();d=mujoco.MjData(m);d.qpos[:]=s["qpos"];mujoco.mj_forward(m,d);cid=m.geom("vlab_cup").id;cup_ok=bool(np.allclose(d.geom_xpos[cid],s["cup_center_m"],rtol=0,atol=1e-12));inside=abs(s["cup_center_m"][0])+s["cup_radius_m"]<=s["table"]["half_extents_m"][0] and abs(s["cup_center_m"][1])+s["cup_radius_m"]<=s["table"]["half_extents_m"][1]
  if not cup_ok or not inside:raise RuntimeError("cup coordinate or table-bound invariant failed")
  r=mujoco.Renderer(m,height=240,width=320);obs={}
  try:
   for n in ["wrist_cam","fixed_depth_01","fixed_depth_02"]:
    r.update_scene(d,camera=n);Image.fromarray(r.render()).save(OUT/f"{n}-rgb.png");r.enable_depth_rendering();r.update_scene(d,camera=n);depth=r.render();r.disable_depth_rendering();np.save(OUT/f"{n}-depth.npy",depth);r.enable_segmentation_rendering();r.update_scene(d,camera=n);seg=r.render();r.disable_segmentation_rendering();np.save(OUT/f"{n}-segmentation.npy",seg)
    mask=(seg[:,:,0]==cid)&(seg[:,:,1]==int(mujoco.mjtObj.mjOBJ_GEOM));obs[n]={"cup_segmentation_pixels":int(np.count_nonzero(mask)),"finite_depth_fraction":float(np.isfinite(depth).mean())}
  finally:r.close()
  absent=all(obs[n]["cup_segmentation_pixels"]==0 for n in ["fixed_depth_01","fixed_depth_02"]);assessment={"terminal_assessment":"not_observable_by_nominal_fixed_depth" if absent else "candidate_remains_observable_by_nominal_fixed_depth","cup_position_invariant":cup_ok,"cup_within_table_bounds":inside,"camera_observations":obs};(OUT/"terminal-assessment.json").write_bytes(strict_json_bytes(assessment));result={"schema_version":"first-robots/research-result/v1","run_id":RID,"status":"passed_pending_human_review" if absent else "blocked_pending_human_review","observations":assessment,"limits":c["non_claims"],"review":{"status":"pending_human_review","adopted":False}};record={"schema_version":"first-robots/experiment-record/v1","run_id":RID,"kind":"formal_virtual_laboratory_not_observable_candidate","result":{"path":RESULT.relative_to(ROOT).as_posix(),"sha256":sha256_bytes(strict_json_bytes(result))},"review":{"status":"pending_human_review"}};write_json_pair(first_path=RECORD,first_content=record,second_path=RESULT,second_content=result)
  for p in sorted(OUT.iterdir()):rec.add_artifact(p)
  rec.add_artifact(RESULT);rec.add_artifact(RECORD);rec.add_metric("fixed_cameras_cup_absent",absent,"boolean");rec.complete(passed=absent,criteria="Cup remains table-contained and both fixed cameras emit no cup segmentation pixels.",review_status="pending_human_review",note="One candidate static failure scene; no retry or cause attribution.")
if __name__=="__main__":main()
