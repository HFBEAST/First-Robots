"""Build and verify a static research-only virtual laboratory; never controls hardware."""
from __future__ import annotations
import hashlib, json
from pathlib import Path
import mujoco
import numpy as np
from PIL import Image
from experiment_management.run import RunRecorder, sha256_bytes, strict_json_bytes, write_json_pair

RID="EXP-20260921-001-virtual-laboratory-v0-preflight"; ROOT=Path(__file__).resolve().parents[2]; CFG=ROOT/"research/configs"/f"{RID}.json"; SCENE=ROOT/"research/sources/mujoco_menagerie_robotstudio_so101/robotstudio_so101/scene.xml"; OUT=ROOT/"research/artifacts"/RID; RESULT=ROOT/"research/reports"/f"{RID}.result.json"; RECORD=ROOT/"research/experiments"/f"{RID}.record.json"
def file_hash(path:Path)->str:return "sha256:"+hashlib.sha256(path.read_bytes()).hexdigest()
def look_at_quat(position:np.ndarray,target:np.ndarray)->list[float]:
 forward=target-position;forward/=np.linalg.norm(forward);camera_z=-forward;camera_x=np.cross(np.array([0.,0.,1.]),camera_z);camera_x/=np.linalg.norm(camera_x);camera_y=np.cross(camera_z,camera_x);matrix=np.column_stack((camera_x,camera_y,camera_z)).reshape(9);quat=np.empty(4);mujoco.mju_mat2Quat(quat,matrix);return quat.tolist()
def main()->None:
 if OUT.exists() or RESULT.exists() or RECORD.exists():raise FileExistsError("refusing overwrite")
 cfg=json.loads(CFG.read_text());lab=cfg["laboratory"]
 if file_hash(SCENE)!=cfg["model"]["scene_sha256"]:raise RuntimeError("source scene SHA-256 does not match frozen configuration")
 rec=RunRecorder(run_id=RID,objective="Build one static named virtual laboratory v0 before task or control research.",script=Path(__file__),repo_root=ROOT,output_root=ROOT/"research/runs",parameters=lab,random_seed=None,inputs=[{"kind":"configuration","path":CFG.relative_to(ROOT).as_posix(),"sha256":file_hash(CFG)},{"kind":"candidate_model","path":SCENE.relative_to(ROOT).as_posix(),"sha256":file_hash(SCENE),"revision":cfg["model"]["repository_revision"]}],baseline={"kind":"virtual_laboratory","reference":"No physical-workspace equivalence is assumed."},level="formal")
 with rec:
  OUT.mkdir(parents=True);(OUT/"config.json").write_bytes(strict_json_bytes(cfg));spec=mujoco.MjSpec.from_file(str(SCENE));table=spec.worldbody.add_geom();table.name=lab["table"]["name"];table.type=mujoco.mjtGeom.mjGEOM_BOX;table.size=lab["table"]["half_extents_m"];table.pos=lab["table"]["center_m"];table.rgba=[.45,.32,.2,1.]
  cup=spec.worldbody.add_geom();cup.name=lab["cup"]["name"];cup.type=mujoco.mjtGeom.mjGEOM_CYLINDER;cup.size=[lab["cup"]["radius_m"],lab["cup"]["half_height_m"],0];cup.pos=lab["cup"]["center_m"];cup.rgba=[.1,.5,.9,1.]
  for name,definition in lab["cameras"].items():
   if name=="wrist_cam":continue
   camera=spec.worldbody.add_camera();camera.name=name;camera.pos=definition["position_m"];camera.quat=look_at_quat(np.asarray(definition["position_m"]),np.asarray(definition["target_m"]));camera.resolution=[lab["renderer"]["width"],lab["renderer"]["height"]];camera.fovy=60
  model=spec.compile();data=mujoco.MjData(model);data.qpos[:]=lab["qpos"];mujoco.mj_forward(model,data);cup_id=model.geom(lab["cup"]["name"]).id;table_id=model.geom(lab["table"]["name"]).id;cup_center=data.geom_xpos[cup_id].copy();table_top=float(model.geom_pos[table_id][2]+model.geom_size[table_id][2]);cup_ok=bool(np.allclose(cup_center,lab["cup"]["center_m"],rtol=0,atol=1e-12));table_ok=bool(np.isclose(table_top,lab["table"]["top_z_m"],rtol=0,atol=1e-12))
  if not cup_ok or not table_ok:raise RuntimeError("virtual laboratory coordinate contract failed")
  cameras=list(lab["cameras"]);renderer=mujoco.Renderer(model,height=lab["renderer"]["height"],width=lab["renderer"]["width"]);camera_observations={}
  try:
   for name in cameras:
    renderer.update_scene(data,camera=name);rgb=renderer.render();Image.fromarray(rgb).save(OUT/f"{name}-rgb.png")
    renderer.enable_depth_rendering();renderer.update_scene(data,camera=name);depth=renderer.render();renderer.disable_depth_rendering();np.save(OUT/f"{name}-depth.npy",depth)
    renderer.enable_segmentation_rendering();renderer.update_scene(data,camera=name);seg=renderer.render();renderer.disable_segmentation_rendering();np.save(OUT/f"{name}-segmentation.npy",seg)
    target_mask=(seg[:,:,0]==cup_id)&(seg[:,:,1]==int(mujoco.mjtObj.mjOBJ_GEOM))
    pixels=int(np.count_nonzero(target_mask));camera_observations[name]={"cup_segmentation_pixels":pixels,"finite_depth_fraction":float(np.isfinite(depth).mean())}
  finally:renderer.close()
  state={"coordinate_contract":lab["coordinate_contract"],"cup_input_center_m":lab["cup"]["center_m"],"cup_world_center_m":cup_center.tolist(),"table_top_z_m":table_top,"cameras":camera_observations};(OUT/"state.json").write_bytes(strict_json_bytes(state));(OUT/"derived-scene.xml").write_text(spec.to_xml(),encoding="utf-8")
  fixed_visible=all(camera_observations[name]["cup_segmentation_pixels"]>0 for name in ("fixed_depth_01","fixed_depth_02"));result={"schema_version":"first-robots/research-result/v1","run_id":RID,"status":"passed_pending_human_review" if fixed_visible else "blocked_pending_human_review","observations":{"cup_position_invariant":cup_ok,"table_top_invariant":table_ok,"camera_observations":camera_observations,"fixed_cameras_cup_visible":fixed_visible},"limits":cfg["non_claims"],"review":{"status":"pending_human_review","adopted":False}};record={"schema_version":"first-robots/experiment-record/v1","run_id":RID,"kind":"formal_virtual_laboratory_v0_preflight","result":{"path":RESULT.relative_to(ROOT).as_posix(),"sha256":sha256_bytes(strict_json_bytes(result))},"review":{"status":"pending_human_review"}};write_json_pair(first_path=RECORD,first_content=record,second_path=RESULT,second_content=result)
  for path in sorted(OUT.iterdir()):rec.add_artifact(path)
  rec.add_artifact(RESULT);rec.add_artifact(RECORD);rec.add_metric("fixed_cameras_cup_visible",fixed_visible,"boolean");rec.add_metric("wrist_cup_segmentation_pixels",camera_observations["wrist_cam"]["cup_segmentation_pixels"],"pixels");rec.complete(passed=fixed_visible,criteria="The named static laboratory must satisfy coordinate invariants, emit three-view artifacts, and show the cup in both nominal fixed cameras.",review_status="pending_human_review",note="Static virtual-laboratory initialization only.")
if __name__=="__main__":main()
