"""Static MuJoCo contact gate for virtual wrist-camera evidence; no hardware I/O."""
from __future__ import annotations
import hashlib,json
from pathlib import Path
import mujoco,numpy as np
from experiment_management.run import RunRecorder,sha256_bytes,strict_json_bytes,write_json_pair
RID='EXP-20260901-002-virtual-wrist-contact-gate'; ROOT=Path(__file__).resolve().parents[2]
CFG=ROOT/'research/configs/EXP-20260901-002-virtual-wrist-contact-gate.json'; SCENE=ROOT/'research/sources/mujoco_menagerie_robotstudio_so101/robotstudio_so101/scene.xml'; PRIOR=ROOT/'research/artifacts/EXP-20260901-001-virtual-wrist-coverage/coverage.json'; OUT=ROOT/'research/artifacts'/RID; RES=ROOT/'research/reports'/f'{RID}.result.json'; REC=ROOT/'research/experiments'/f'{RID}.record.json'
def h(p): return 'sha256:'+hashlib.sha256(p.read_bytes()).hexdigest()
def vis(m,d,cup):
 o=d.cam_xpos[m.camera('wrist_cam').id].copy(); v=d.geom_xpos[cup]-o; v/=np.linalg.norm(v); g=np.array([-1],dtype=np.int32); mujoco.mj_ray(m,d,o,v,None,True,-1,g); return int(g[0])==cup
def main():
 if OUT.exists() or RES.exists() or REC.exists(): raise FileExistsError('refusing overwrite')
 cfg=json.loads(CFG.read_text()); prior=json.loads(PRIOR.read_text()); rec=RunRecorder(run_id=RID,objective='Record candidate-model static contacts alongside virtual wrist visibility without asserting physical safety.',script=Path(__file__),repo_root=ROOT,output_root=ROOT/'research/runs',parameters=cfg['protocol'],random_seed=11,inputs=[{'kind':'configuration','path':CFG.relative_to(ROOT).as_posix(),'sha256':h(CFG)},{'kind':'prior_pose_table','path':PRIOR.relative_to(ROOT).as_posix(),'sha256':h(PRIOR)},{'kind':'model','path':SCENE.relative_to(ROOT).as_posix(),'sha256':h(SCENE),'revision':'da76818e269b82289eba39808e2fb91d679d6994'}],baseline={'kind':'comparison','reference':'EXP-20260901-001 records visibility without a contact gate.'},level='formal'); rec.__enter__(); OUT.mkdir(parents=True); (OUT/'config.json').write_bytes(strict_json_bytes(cfg))
 spec=mujoco.MjSpec.from_file(str(SCENE)); cup=spec.worldbody.add_geom(); cup.name='virtual_cup'; cup.type=mujoco.mjtGeom.mjGEOM_CYLINDER; cup.size=[.035,.055,0]; cup.pos=[0,0,.055]; m=spec.compile(); d=mujoco.MjData(m); cid=m.geom('virtual_cup').id; rows=[]
 for source in prior['pose_results']:
  d.qpos[:]=source['qpos']; count=0; contacts=[]
  for pos in prior['cups']:
   m.geom_pos[cid]=pos; mujoco.mj_forward(m,d); contacts.append(int(d.ncon)); count+=vis(m,d,cid)
  rows.append({'sample_index':source['sample_index'],'qpos':source['qpos'],'visible_cups':count,'contact_counts':contacts,'max_contacts':max(contacts),'zero_contact_all_cups':max(contacts)==0})
 gated=[r for r in rows if r['zero_contact_all_cups']]; best=max(gated,key=lambda r:r['visible_cups']) if gated else None
 meta={'run_id':RID,'mujoco':mujoco.__version__,'rows':rows,'zero_contact_all_cups_count':len(gated),'first_maximum_zero_contact':best}; (OUT/'contact-coverage.json').write_bytes(strict_json_bytes(meta))
 result={'schema_version':'first-robots/research-result/v1','run_id':RID,'status':'observed_pending_human_review','observations':{'samples':len(rows),'zero_contact_all_cups_samples':len(gated),'best_zero_contact_visible_cups':None if best is None else best['visible_cups'],'best_zero_contact_sample_index':None if best is None else best['sample_index']},'limits':cfg['non_claims'],'review':{'status':'pending_human_review','adopted':False}}
 record={'schema_version':'first-robots/experiment-record/v1','run_id':RID,'kind':'formal_virtual_static_contact_gate','result':{'path':RES.relative_to(ROOT).as_posix(),'sha256':sha256_bytes(strict_json_bytes(result))},'review':{'status':'pending_human_review'}}; write_json_pair(first_path=REC,first_content=record,second_path=RES,second_content=result)
 for p in OUT.iterdir(): rec.add_artifact(p)
 rec.add_artifact(RES); rec.add_artifact(REC); rec.add_metric('samples',len(rows),'poses'); rec.add_metric('zero_contact_all_cups_samples',len(gated),'poses'); rec.add_metric('best_zero_contact_visible_cups',-1 if best is None else best['visible_cups'],'cups'); rec.complete(passed=True,criteria='All source poses and cup positions were recorded with MuJoCo static contacts and virtual visibility; no safety claim.',review_status='pending_human_review',note='Candidate-model static gate only.')
if __name__=='__main__': main()
