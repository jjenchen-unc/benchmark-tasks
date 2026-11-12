"""EVALUATOR ONLY: component/foundation/negative probes. NOT robot feasibility.
Object poses and externally applied loads below are deliberately excluded from
reference.py. Every report labels its interventions and records actual channels.
"""
from c1_launch import bootstrap
bootstrap(__file__)
import argparse,json,hashlib,os,shutil,time,sys
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
from c1_contract import *

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--out',required=True,type=Path);ap.add_argument('--suite',choices=['foundation','friction','wet','guard','reset','targeted','selected-force'],default='foundation');a=ap.parse_args();__import__('c1_launch').output_directory(a.out)
 os.environ['C1_TRACE']=str(a.out/'measurements.jsonl');os.environ['C1_TARGET']='C1-T2' if a.suite=='wet' else 'C1-T1'
 pass
 from inspect_robots import Scene
 from inspect_robots.types import Action
 from yam_arena.inspect_embodiment import yam_arena_embodiment
 from scipy.spatial.transform import Rotation
 from c1_kit import arr
 import c1_kit
 emb=yam_arena_embodiment(task=str(HERE/'task.py'),device='cpu',object_jitter=0,control_hz=20,cam_width=320,cam_height=176)
 report={'kind':'EVALUATOR_ONLY component/negative/foundation validation; never feasibility/model evidence','suite':a.suite,'model_calls':0,'checks':{},'episodes':[],'interventions':[],'submission_ready':False,'all_checks_passed':False};actions=[];started=time.monotonic()
 class Checkpoints(dict):
  def __setitem__(self,key,value):
   value=bool(value);super().__setitem__(key,value)
   with (a.out/'checkpoints.jsonl').open('a') as f:f.write(json.dumps({'case':key,'passed':value,'elapsed_s':time.monotonic()-started})+'\n')
   (a.out/'partial-result.json').write_text(json.dumps(report,indent=2)+'\n')
   if not value and a.suite not in ['targeted','selected-force']:raise RuntimeError('Foundation checkpoint failed: '+key)
 report['checks']=Checkpoints()
 reset_count=0
 def reset(seed=42):
  nonlocal reset_count
  previous=c1_kit.ENGINE
  if previous is not None and previous.active:
   if previous.trace:previous.trace.flush()
   if hasattr(previous,'diagnostic_trace'):
    previous.diagnostic_trace.close();del previous.diagnostic_trace
   for filename in ['measurements.jsonl','guard-1ms.jsonl']:
    file=a.out/filename
    if file.exists():shutil.copy2(file,a.out/('episode-'+str(reset_count)+'-'+filename))
  reset_count+=1
  obs=emb.reset(Scene(id='C1-EVALUATOR-ONLY-'+a.suite,instruction='Evaluator diagnostic; component states do not count as a robot demonstration.'),seed=seed)
  return obs,np.asarray(obs.state['joint_pos'],float).copy()
 def step(q,n,phase,allow_done=False):
  last=None
  for _ in range(n):
   actions.append({'episode':len(report['episodes']),'phase':phase,'data':q.tolist()});last=emb.step(Action(data=q.copy()))
   if last.terminated:
    if allow_done:break
    raise RuntimeError('Unexpected termination '+str((phase,last.termination_reason)))
  return last
 def pose(name,p,R):
  # All pose writes are evaluator setup and explicitly logged here.
  import torch
  obj=emb._base.scene[name];origin=arr(emb._base.scene.env_origins)[0];q=Rotation.from_matrix(R).as_quat();data=np.r_[np.asarray(p)+origin,q].astype(np.float32)
  obj.write_root_pose_to_sim_index(root_pose=torch.as_tensor(data,device=emb._base.device).reshape(1,7));obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1,6),device=emb._base.device))
  report['interventions'].append({'kind':'EVALUATOR_ONLY_root_pose','object':name,'position_world_m':np.asarray(p).tolist(),'quaternion_xyzw':q.tolist()})
 def record(label):
  e=c1_kit.ENGINE;row={'label':label,'status':e.status(),'contact_examples':e.examples,'cup_pos':[e.state('cup_'+str(i))[0].tolist() for i in [1,2]],'liner_pos':[e.state('liner_'+str(i))[0].tolist() for i in range(1,7)]};report['episodes'].append(row);snapshot(e,a.out/('readback-'+str(len(report['episodes']))+'.json'));return row
 try:
  obs,q=reset();e=c1_kit.ENGINE
  from PIL import Image
  for name,img in obs.images.items():Image.fromarray(img).save(a.out/('initial-'+name+'.png'))
  report['checks']['three_station_rgb']=set(obs.images)=={'top_cam','left_cam','right_cam'} and all(x.std()>5 for x in obs.images.values())
  report['checks']['14D20Hz']=q.shape==(14,) and float(emb.info.control_hz)==20 and abs(float(emb._base.step_dt)-.05)<1e-8
  report['checks']['1ms_substeps']=abs(float(emb._base.physics_dt)-.001)<1e-8
  report['checks']['all_credit_gated_unverified']=e.status()['score']==0 and bool(e.status()['missing'])
  from targeted_readback import snapshot, force_basis
  snapshot(e,a.out/'actual-initial-inventory.json') # before any action
  initial_inventory=json.loads((a.out/'actual-initial-inventory.json').read_text())
  report['checks']['actual_pair_filter_matrix']=initial_inventory['pair_filter_matrix_matches']
  report['checks']['actual_12_pair_only_contacts']=sum(x['collision_allowed'] for x in initial_inventory['actual_cup_liner_filter_matrix'])==12
  step(q,10,'neutral');report['checks']['actual_contacts_and_substeps']=e.history.samples==500 and e.error is None and bool(e.history.last['contacts'])
  report['force_approximation']={'kind':'conservative Coulomb upper bound, not measured per-contact vector norm','effective_mu_upper_bound':e.contacts.mu_bound,'actual_stage_material_coefficients':e.contacts.material_coefficients,'threshold_n':50.,'window_s':.020}
  report['initial_actual_positions']={name:e.state(name)[0].tolist() for name in ['liner_1','cup_1','tray']}
  report['checks']['rack20degrees']=abs(np.degrees(np.arccos(np.clip(N_RACK[2],-1,1)))-20)<1e-8
  from yam_arena import ik
  from yam_arena.constants import TCP_OFFSET
  report['measured_mount_FK']=[]
  for side in ['left','right']:
   robot=emb._base.scene[side+'_robot'];origin=arr(emb._base.scene.env_origins)[0];rootpos=arr(robot.data.body_pos_w)[0,0]-origin;B=rotation_xyzw(arr(robot.data.body_quat_w)[0,0]);i=robot.body_names.index('gripper');G=rotation_xyzw(arr(robot.data.body_quat_w)[0,i]);tcp=arr(robot.data.body_pos_w)[0,i]-origin+G@np.array(TCP_OFFSET);jointids=robot.find_joints(['joint'+str(i) for i in range(1,7)],preserve_order=True)[0];fp,fR=ik.tcp_pose(arr(robot.data.joint_pos)[0,jointids]);pe=float(np.linalg.norm(rootpos+B@fp-tcp));re=float(np.linalg.norm(ik._rot_error(B@fR,G)))
   report['measured_mount_FK'].append({'arm':side,'actual_base':rootpos.tolist(),'base_R':B.tolist(),'FK_position_error_m':pe,'FK_rotation_error_rad':re})
   yaw=ROBOT_YAW[side];expected_B=np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1.]])
   report['checks'][side+'_declared_mount_and_FK']=np.linalg.norm(rootpos-np.array(ROBOT_BASES[side]))<1e-5 and np.linalg.norm(B-expected_B)<1e-5 and pe<.002 and re<.01
  caps={}
  for side in ['left','right']:
   robot=emb._base.scene[side+'_robot'];caps[side]={'joints':list(robot.joint_names),'effort_limit':arr(robot.root_view.get_dof_max_forces()).tolist(),'stiffness':arr(robot.root_view.get_dof_stiffnesses()).tolist(),'drive_ceiling_API':'root_view.get_dof_max_forces'}
   ids=robot.find_joints(['joint'+str(i) for i in range(1,7)],preserve_order=True)[0];fids=robot.find_joints(['joint7','joint8'],preserve_order=True)[0];lim=np.array(caps[side]['effort_limit'])[0]
   report['checks'][side+'_27_7Nm_40N_actual_caps']=bool(np.allclose(lim[ids],[27]*3+[7]*3) and np.allclose(lim[fids],[40,40]))
  report['actual_robot_limits']=caps;record('foundation')
  if a.suite=='foundation':
   moved=q.copy();moved[0]+=.08;moved[7]-=.08;res=step(moved,30,'real_joint_commands');measured=np.asarray(res.observation.state['joint_pos']);report['checks']['both_arms_respond']=bool(abs(measured[0]-q[0])>.04 and abs(measured[7]-q[7])>.04);step(q,30,'return');record('physical_bimanual_commands')
  if a.suite in ['targeted','selected-force']:
   force_basis(emb,e,q,step,report,a.out)
  if a.suite in ['friction','targeted']:
   p=parameters(42);choices=[('slip',int(np.argmin(p['mu_dry']))),('hold',int(np.argmax(p['mu_dry'])))];R=A@Rotation.from_euler('x',ANGLE).as_matrix();s=slot_point(1)
   for label,i in choices:
    obs,q=reset();e=c1_kit.ENGINE;pose(f'liner_{i+1}',s+N_RACK*.003,R);pose('cup_1',s+N_RACK*(.006+.035+.0005),R);step(q,12,'EVALUATOR_ONLY_placed_settle');p0=e.state('cup_1')[0].copy();snapshot(e,a.out/('placed-'+label+'-before-hold.json'));report['checks']['physical_'+label+'_cup_liner_contact']=bool((10,20+i) in set(map(tuple,e.history.last['contacts'])));res=step(q,420,'EVALUATOR_ONLY_placed_21s',allow_done=True);drift=float(np.linalg.norm(e.state('cup_1')[0]-p0));row=record('EVALUATOR_ONLY_'+label);row.update(mu=p['mu_dry'][i],displacement_after_settle_m=drift)
    report['checks']['physical_'+label]=bool((e.history.cup_fell or drift>.005) if label=='slip' else e.history.score()['score']==1.)
  if a.suite in ['wet','targeted']:
   obs,q=reset(seed=44);e=c1_kit.ENGINE;i=int(np.argmax(e.params['mu_dry']));p=world([.05,.035,.812]);pose(f'liner_{i+1}',p,A);step(q,3,'EVALUATOR_ONLY_actual_water_entry')
   from pxr import UsdPhysics
   material=UsdPhysics.MaterialAPI(e.stage.GetPrimAtPath(e.env+f'/liner_{i+1}/pair_material'));actual=float(material.GetDynamicFrictionAttr().Get())
   report['checks']['passive_water_entry_changes_material']=bool(e.wet[i] and abs(actual-e.params['mu_wet'][i])<1e-6);record('EVALUATOR_ONLY_wet_transition')
   snapshot(e,a.out/'actual-wet-inventory.json')
   # Removing the liner from water cannot dry it within the episode.
   pose(f'liner_{i+1}',LINER_START[i],A);step(q,3,'EVALUATOR_ONLY_leave_water');report['checks']['wet_is_irreversible']=bool(e.wet[i]);record('wet_persists')
  if a.suite=='targeted':
   snapshot(e,a.out/'actual-pre-abort-inventory.json')
  if a.suite in ['guard','targeted']:
   obs,q=reset();e=c1_kit.ENGINE
   if a.suite=='targeted':
    i=int(np.argmax(e.params['mu_dry']));R=A@Rotation.from_euler('x',ANGLE).as_matrix();s=slot_point(1)
    pose(f'liner_{i+1}',s+N_RACK*.003,R);pose('cup_1',s+N_RACK*(.006+.035+.0005),R)
    step(q,12,'EVALUATOR_ONLY_positive_credit_before_abort')
    report['pre_abort_raw']=e.history.score()
    report['checks']['abort_has_positive_credit_baseline']=report['pre_abort_raw']['raw_before_veto']>0.
    snapshot(e,a.out/'actual-positive-pre-abort-inventory.json')
   import torch
   robot=emb._base.scene['right_robot'];force=torch.tensor([[[0.,0.,-120.]]],device=emb._base.device);robot.permanent_wrench_composer.reset();robot.permanent_wrench_composer.add_forces_and_torques(force,torch.zeros_like(force),body_ids=robot.find_bodies('gripper')[0],is_global=True)
   report['interventions'].append({'kind':'EVALUATOR_ONLY_known_load','body':'RightArm/gripper','force_world_n':[0,0,-120]});res=step(q,10,'EVALUATOR_ONLY120N_guard',allow_done=True);robot.permanent_wrench_composer.reset();report['checks']['actual80N100ms_abort']=res.terminated and res.termination_reason=='wrist_force_limit';report['checks']['hard_guard_vetoes_raw_credit']=e.history.score()['score']==0.;
   if a.suite=='targeted':report['checks']['abort_preserves_positive_counterfactual']=e.history.score()['raw_before_veto']>0.
   record('EVALUATOR_ONLY_guard');snapshot(e,a.out/'actual-post-abort-inventory.json')
  if a.suite=='reset':
   before={name:e.state(name)[0].copy() for name in ['liner_1','cup_1','tray']};pose('liner_1',world([.05,.035,.812]),A);step(q,3,'EVALUATOR_ONLY_wet_before_reset');obs,q=reset();e=c1_kit.ENGINE
   report['checks']['reset_restores_dry_material_and_time']=not any(e.wet) and e.history.samples==0 and e.controls==0
   report['checks']['reset_restores_root_poses']=all(np.linalg.norm(e.state(n)[0]-p)<.002 for n,p in before.items());record('fresh_reset')
  report['all_checks_passed']=all(bool(v) for v in report['checks'].values())
 except BaseException as exc:
  report['error']=type(exc).__name__+': '+str(exc);(a.out/'error.json').write_text(json.dumps(report,indent=2)+'\n');raise
 finally:
  report['wall_s']=time.monotonic()-started;(a.out/'result.json').write_text(json.dumps(report,indent=2)+'\n');(a.out/'actions.json').write_text(json.dumps(actions)+'\n');emb.close()
if __name__=='__main__':main()
