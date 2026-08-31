"""Research-only static virtual wrist-camera coverage scan; never controls hardware."""
from __future__ import annotations
import hashlib, json, math, platform
from pathlib import Path
import mujoco, numpy as np
from PIL import Image
from experiment_management.run import RunRecorder, sha256_bytes, strict_json_bytes, write_json_pair

RUN_ID='EXP-20260901-001-virtual-wrist-coverage'
ROOT=Path(__file__).resolve().parents[2]
CFG=ROOT/'research/configs/EXP-20260901-001-virtual-wrist-coverage.json'
SCENE=ROOT/'research/sources/mujoco_menagerie_robotstudio_so101/robotstudio_so101/scene.xml'
OUT=ROOT/'research/artifacts'/RUN_ID; RESULT=ROOT/'research/reports'/f'{RUN_ID}.result.json'; RECORD=ROOT/'research/experiments'/f'{RUN_ID}.record.json'
def h(p): return 'sha256:'+hashlib.sha256(p.read_bytes()).hexdigest()
def visible(m,d,cam,cup):
 o=d.cam_xpos[m.camera(cam).id].copy(); v=d.geom_xpos[cup]-o; v/=np.linalg.norm(v); g=np.array([-1],dtype=np.int32); mujoco.mj_ray(m,d,o,v,None,True,-1,g); return int(g[0])==cup
def main():
 if OUT.exists() or RESULT.exists() or RECORD.exists(): raise FileExistsError('refusing overwrite')
 cfg=json.loads(CFG.read_text()); rec=RunRecorder(run_id=RUN_ID,objective='Measure sampled virtual wrist-camera geometric coverage without claiming hardware visibility.',script=Path(__file__),repo_root=ROOT,output_root=ROOT/'research/runs',parameters=cfg['protocol'],random_seed=11,inputs=[{'kind':'configuration','path':CFG.relative_to(ROOT).as_posix(),'sha256':h(CFG)},{'kind':'model','path':SCENE.relative_to(ROOT).as_posix(),'sha256':h(SCENE),'revision':'da76818e269b82289eba39808e2fb91d679d6994'}],baseline={'kind':'comparison','reference':'Default pose result from EXP-20260831-004 was 0/10.'},level='formal'); rec.__enter__()
 OUT.mkdir(parents=True); (OUT/'config.json').write_bytes(strict_json_bytes(cfg))
 spec=mujoco.MjSpec.from_file(str(SCENE)); cup=spec.worldbody.add_geom(); cup.name='virtual_cup'; cup.type=mujoco.mjtGeom.mjGEOM_CYLINDER; cup.size=[.035,.055,0]; cup.pos=[0,0,.055]; cup.rgba=[.1,.5,.9,1]
 m=spec.compile(); d=mujoco.MjData(m); cid=m.geom('virtual_cup').id; radius=cfg['protocol']['workspace_radius']; cups=[]
 for seed in cfg['protocol']['cup_seeds']:
  r=np.random.default_rng(seed); rho=radius*math.sqrt(float(r.random())); a=float(r.uniform(-math.pi,math.pi)); cups.append([rho*math.cos(a),rho*math.sin(a),.055])
 rng=np.random.default_rng(11); poses=np.vstack([np.zeros(m.nq),rng.uniform(m.jnt_range[:,0],m.jnt_range[:,1],size=(256,m.nq))]); rows=[]
 for i,q in enumerate(poses):
  d.qpos[:]=q; count=0
  for p in cups: m.geom_pos[cid]=p; mujoco.mj_forward(m,d); count+=visible(m,d,'wrist_cam',cid)
  rows.append({'sample_index':i,'qpos':q.tolist(),'visible_cups':count})
 best=max(rows,key=lambda x:x['visible_cups']); d.qpos[:]=best['qpos']; m.geom_pos[cid]=cups[0]; mujoco.mj_forward(m,d); renderer=mujoco.Renderer(m,height=240,width=320); renderer.update_scene(d,camera='wrist_cam'); Image.fromarray(renderer.render()).save(OUT/'best-pose-seed-00-wrist-rgb.png'); renderer.close()
 meta={'run_id':RUN_ID,'mujoco':mujoco.__version__,'cups':cups,'pose_results':rows,'best_first_maximum':best}; (OUT/'coverage.json').write_bytes(strict_json_bytes(meta))
 result={'schema_version':'first-robots/research-result/v1','run_id':RUN_ID,'status':'observed_pending_human_review','observations':{'default_visible_cups':rows[0]['visible_cups'],'best_sample_visible_cups':best['visible_cups'],'best_sample_index':best['sample_index'],'pose_samples':len(rows),'cup_positions':len(cups)},'limits':cfg['non_claims'],'review':{'status':'pending_human_review','adopted':False}}
 record={'schema_version':'first-robots/experiment-record/v1','run_id':RUN_ID,'kind':'formal_virtual_static_coverage_scan','result':{'path':RESULT.relative_to(ROOT).as_posix(),'sha256':sha256_bytes(strict_json_bytes(result))},'review':{'status':'pending_human_review'}}; write_json_pair(first_path=RECORD,first_content=record,second_path=RESULT,second_content=result)
 for p in sorted(OUT.iterdir()): rec.add_artifact(p)
 rec.add_artifact(RESULT); rec.add_artifact(RECORD); rec.add_metric('default_visible_cups',rows[0]['visible_cups'],'cups'); rec.add_metric('best_sample_visible_cups',best['visible_cups'],'cups'); rec.add_metric('pose_samples',len(rows),'poses'); rec.complete(passed=True,criteria='All deterministic samples were evaluated and coverage artifacts written; no capability criterion.',review_status='pending_human_review',note='Virtual-only evidence.')
if __name__=='__main__': main()
