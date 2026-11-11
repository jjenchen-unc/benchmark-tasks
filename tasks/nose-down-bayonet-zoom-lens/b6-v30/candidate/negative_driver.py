"""EVALUATOR ONLY: explicitly placed unlocked/locked/out-of-scale/drop negatives.
Never used as a robot demonstration; state writes are confined to this file.
"""
import json,math
from pathlib import Path
import numpy as np
from kit import arr

def run(out,case='unlocked'):
 if case not in ['unlocked','wrong-family','drop','creeping','holding','torsion-positive','torsion-negative','transverse-positive','transverse-negative','torsion-overload-positive']:raise ValueError('unknown evaluator case')
 from inspect_robots import Scene
 from inspect_robots.types import Action
 from yam_arena.inspect_embodiment import yam_arena_embodiment
 import kit,torch
 from constraint_probe import install
 install(out)
 emb=yam_arena_embodiment(task=str(Path(__file__).parent/'task.py'),device='cpu',object_jitter=0,control_hz=20,cam_width=320,cam_height=176)
 actions=[]; samples=[]
 try:
  obs=emb.reset(Scene(id='B6-evaluator-negative-'+case,instruction='Evaluator placed-state component diagnostic only.'),seed=872994)
  E=kit.ENGINE;q=np.asarray(obs.state['joint_pos'],float).copy();
  def primitive(x):return json.loads(json.dumps(x,allow_nan=False))
  baseline=dict(public14=q.tolist(),bases={k:list(v) for k,v in emb.rig.bases.items()},robots={side:dict(joint_names=list(emb._base.scene[side+'_robot'].joint_names),raw=arr(emb._base.scene[side+'_robot'].data.joint_pos).tolist(),body_names=list(emb._base.scene[side+'_robot'].body_names),positions=arr(emb._base.scene[side+'_robot'].data.body_pos_w).tolist(),quaternions=arr(emb._base.scene[side+'_robot'].data.body_quat_w).tolist()) for side in ['left','right']})
  (out/'before-placement.json').write_text(json.dumps(baseline,indent=2,allow_nan=False))
  o='lens_orange' if case=='wrong-family' else 'lens_purple' if case=='creeping' else 'lens_blue';a=E.assets[o]
  pose=a.data.default_root_pose.torch.clone();pose[:,:3]+=E.base.scene.env_origins
  if case=='drop':pose[0,2]+=.15
  else:
   p,R=E.pose('body_right','body_right');pose[0,:3]=torch.tensor(p+R[:,2]*.0006,device=E.base.device);pose[0,3:7]=torch.tensor([1.,0,0,0],device=E.base.device) # xyzw180degX
  a.write_root_pose_to_sim_index(root_pose=pose);a.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(a.data.default_root_vel.torch))
  if case in ['holding','creeping'] or case.startswith(('torsion-','transverse-')):
   # Evaluator arrangement: physically enabled capture/pin at source lock angle;
   # all following retention/creep comes from the simulator, never fake history.
   m=E.assets['body_right'];th=-(E.f['parameters']['lock_angle_R']+.8)*math.pi/180;mp=m.data.joint_pos.torch.clone();mv=torch.zeros_like(mp);mp[0,m.joint_names.index('body_right_turn')]=th;m.write_joint_state_to_sim(mp,mv)
   zp=a.data.joint_pos.torch.clone();zp[0,a.joint_names.index(o+'_zoom')]=(50-18)/37*math.pi/2;zp[0,a.joint_names.index(o+'_ext')]=(50-18)/37*.02;a.write_joint_state_to_sim(zp,torch.zeros_like(zp))
   E.activate_current(E.constraints['body_right',o],'body_right','body_right_ring',o,o);E.cap['body_right']=o;E.click['body_right']=True;E.set_pin('body_right',True)
  load_nm=None
  if case.startswith(('torsion-','transverse-')):
   # EVALUATOR ONLY: independent known local-body torque, no policy writes.
   load_nm=[0.,0.,0.];load_nm[2 if case.startswith('torsion-') else 0]=(2.0 if case=='torsion-overload-positive' else .05) if case.endswith('positive') else -.05
   force=torch.zeros((1,len(a.body_names),3),device=E.base.device);torque=force.clone();torque[0,a.body_names.index(o)]=torch.tensor(load_nm,device=E.base.device)
   a.set_external_force_and_torque(force,torque,is_global=False)
  result=None
  for i in range(8):
   actions.append(q.tolist());result=emb.step(Action(data=q))
   samples.append(dict(current_frames={n:dict(body_names=list(E.assets[n].body_names),pos_w=arr(E.assets[n].data.body_pos_w).tolist(),quat_xyzw=arr(E.assets[n].data.body_quat_w).tolist(),joint_names=list(E.assets[n].joint_names),q=arr(E.assets[n].data.joint_pos).tolist(),limits=arr(E.assets[n].root_view.get_dof_limits()).tolist()) for n in ['body_right',o]},pin_angles=dict(E.pin_angles),control=i+1,simulation_s=E.history.t,evaluator_local_torque_nm=load_nm,status=E.status(),measurement=E.measurement(),zoom=dict(theta=E.joint(o,o+'_zoom')[0],extension_m=E.joint(o,o+'_ext')[0]),last=primitive(E.last),mount_raw_wrench={m:arr(E.assets[m].root_view.get_link_incoming_joint_force()).tolist() for m in E.cap},constraint_prims={str(j.GetPath()):dict(enabled=bool(j.GetJointEnabledAttr().Get()),localPos0=list(j.GetLocalPos0Attr().Get()),localPos1=list(j.GetLocalPos1Attr().Get()),localRot0=str(j.GetLocalRot0Attr().Get()),localRot1=str(j.GetLocalRot1Attr().Get())) for j in list(E.constraints.values())+list(E.pin_constraints.values()) if j.GetJointEnabledAttr().Get()}))
   (out/'component-journal.jsonl').open('a').write(json.dumps(samples[-1],allow_nan=False)+'\n')
   if result.terminated:break
  status=E.status();measurement=E.measurement();termination=result.termination_reason
  release_checks={}
  if E.pin['body_right']:
   m=E.assets['body_right'];pos=m.data.joint_pos.torch.clone();vel=torch.zeros_like(pos);pos[0,m.joint_names.index('body_right_button_slide')]=.003
   m.write_joint_state_to_sim(pos,vel) # EVALUATOR ONLY button placement
   held,F,push=E.read_contacts(.002);E.rules(.002,held,F,push)
   release_checks['original_button_predicate_releases']=not E.pin['body_right']
   release_checks['release_restores_exact_limits']=bool(np.array_equal(arr(m.root_view.get_dof_limits()),E.native_pin_limits['body_right']))
   pos[0,m.joint_names.index('body_right_button_slide')]=0.;m.write_joint_state_to_sim(pos,vel) # EVALUATOR ONLY release button reset
   held,F,push=E.read_contacts(.002);E.rules(.002,held,F,push)
   release_checks['relatches_at_current_angle']=bool(E.pin['body_right'])
  before_reset=dict(captured=dict(E.cap),pin=dict(E.pin))
  emb.reset(Scene(id='B6-loaded-reset',instruction='Evaluator reset after loaded capture/pin, no robot demonstration.'),seed=98712)
  checks=dict(**release_checks,no_measurement_error=status['measurement_error'] is None,credit_stays_disabled=status['score']==0 and not status['credit_channels_activated'],reset_loaded_capture_pin=not any(E.cap.values()) and not any(E.pin.values()) and not E.history.samples and not E.done,reset_native_limits=all(np.array_equal(arr(E.assets[m].root_view.get_dof_limits()),v) for m,v in E.native_pin_limits.items()),reset_task_joints=all(abs(E.joint(n,j)[0])<1e-5 for n,a in E.assets.items() if hasattr(a,'joint_names') for j in a.joint_names))
  if case=='torsion-overload-positive':
   checks['native_overload_abort']=status['failure']=='mount_overload' and status['history']['overload']
   checks['overload_no_credit']=status['score']==0
  if case in ['holding','creeping']:
   checks['short_lock_retention']=bool(measurement['captured'] and measurement['pin_engaged'])
   checks['short_zoom_measurement_finite']=bool(math.isfinite(measurement['focal_mm']))
  (out/'component-evidence.json').write_text(json.dumps(dict(checks=checks,before_reset=before_reset,final_before_reset=status,sample_count=len(samples),scope=__doc__),indent=2,allow_nan=False))
  return dict(state='diagnostic_complete',evaluator_only=True,case=case,intervention='evaluator placements, signed loads and button placement; no history/score writes',checks=checks,actual_status=status,final_measurement=measurement,termination=termination,physics_verified=False,full_goal_verified=False)
 finally:(out/'actions.json').write_text(json.dumps(actions,allow_nan=False));emb.close()
