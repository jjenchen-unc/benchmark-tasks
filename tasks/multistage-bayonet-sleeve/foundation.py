"""ISOLATED EVALUATOR ONLY. Prepared object poses / known forces diagnose channels.
Never imported by the robot reference; no evaluator phase is a robot demonstration.
"""
import json,math
import numeric
import numpy as np
from scoring import History

def evaluate(emb,obs,out):
 import torch,kit
 from contacts import arr
 from inspect_robots import Scene
 from inspect_robots.types import Action
 numeric.preflight(emb,obs,out)
 report=dict(kind='EVALUATOR_ONLY physical foundation and construct negatives',model_calls=0,full_goal_demonstrated=False,calibration_promoted=False,checks={},phases=[]);trace=open(out/'foundation.jsonl','w',buffering=1);completed=0;q=np.asarray(obs.state['joint_pos'],float).copy()
 def checkpoint(label):
  (out/'foundation-progress.json').write_text(numeric.dumps(dict(phase=label,controls=completed,checks=report['checks']),indent=2)+'\n');(out/'foundation-partial.json').write_text(numeric.dumps(report,indent=2)+'\n')
 def tick(label,count=1,allow_stop=False):
  nonlocal completed
  rows=[];reason=None;checkpoint(label);print('B7 EVALUATOR phase',label,'controls',completed,flush=True)
  for _ in range(count):
   r=emb.step(Action(data=q.copy()));completed+=1;row=kit._OBSERVER.last
   trace.write(numeric.dumps(dict(phase=label,control=completed,action=q.tolist(),substeps=kit._OBSERVER.rows,status=kit.current_status()))+'\n');rows.append(row)
   if r.terminated:
    reason=r.termination_reason
    if not allow_stop:raise RuntimeError(str(reason))
    break
  report['phases'].append(dict(phase=label,controls=len(rows),first=rows[0],last=rows[-1],termination=reason));checkpoint(label+'_complete');return rows,reason
 def reset(label):
  nonlocal q
  # Clear evaluator load BEFORE standard reset; no load leaks across phases.
  if kit._OBSERVER is not None:
   for name in ('coupling_sleeve','coupling_fixture','left_robot','right_robot'):emb._base.scene[name].permanent_wrench_composer.reset()
  ob=emb.reset(Scene(id='b7-evaluator-'+label,instruction='Evaluator-only diagnostic, not a robot demonstration.'),seed=42);q=np.asarray(ob.state['joint_pos'],float).copy();return ob
 def place(label,p,yaw=0.):
  reset(label);a=emb._base.scene['coupling_sleeve'];pose=a.data.default_root_pose.torch.clone();pose[:,:3]=torch.tensor([p],device=pose.device,dtype=pose.dtype)+emb._base.scene.env_origins;pose[:,3:]=torch.tensor([[0.,0.,math.sin(yaw/2),math.cos(yaw/2)]],device=pose.device,dtype=pose.dtype)
  a.write_root_pose_to_sim_index(root_pose=pose);a.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(a.data.default_root_vel.torch));kit._OBSERVER.history=History();return a
 def load(asset,force):
  f=torch.tensor([[force]],device=emb._base.device,dtype=torch.float32)
  asset.permanent_wrench_composer.reset();asset.permanent_wrench_composer.add_forces_and_torques(f,torch.zeros_like(f),is_global=True)
 try:
  rows,_=tick('neutral',20);ob=kit._OBSERVER
  report['contact_selections']=ob.selections;report['motor_tables']=ob.motor_tables;report['runtime_source_hashes']=__import__('observer').source_hashes()
  report['checks']['25_substeps']=ob.history.samples==500
  report['checks']['action14']=q.shape==(14,)
  report['checks']['actual27_7Nm_40N_drives']=len(ob.motor_tables)==2 and all(rows[-1]['motor_limits_checked'])
  report['checks']['reset_no_credit']=kit.current_status()['diagnostic_score']==0
  report['checks']['initial_post_contact']=any(r['post_contact'] for r in rows)
  report['checks']['initial_no_robot_or_floor_contact']=not any(r['robot_contact'] or r['floor_contact'] for r in rows)
  report['checks']['spring_initial_q']=abs(rows[-1]['plunger_q']+.015)<.005
  report['checks']['three_RGB_views']=set(obs.images)=={'top_cam','left_cam','right_cam'} and all(float(im.std())>5 for im in obs.images.values())
  # Known upward load must not bypass the seated pin/track pocket.
  a=emb._base.scene['coupling_sleeve'];z0=rows[-1]['pos'][2];load(a,[0.,0.,20.]);rows,_=tick('EVALUATOR_seated_straight20N_pull',30);a.permanent_wrench_composer.reset()
  report['checks']['seated_straight_pull_blocked']=max(r['pos'][2] for r in rows)-z0<.008 and all(r['post_contact'] for r in rows) and kit.current_status()['diagnostic_score']==0
  # Wrong first branch and false-seat: source C dead end -90deg, notch18mm.
  p=np.array(kit.FIX['sleeve_world']);p[2]-=.002+.012;a=place('wrong_branch',p,math.pi/2);load(a,[0.,0.,12.]);rows,_=tick('EVALUATOR_wrong_branch_false_seat12N',40);a.permanent_wrench_composer.reset()
  report['checks']['wrong_branch_cannot_release']=max(r['pos'][2] for r in rows)<kit.FIX['sleeve_world'][2]+.035 and kit.current_status()['diagnostic_score']==0
  # Separate actual spring load diagnostics at source -15mm and -27mm.
  spring=[]
  for value in (-.015,-.027):
   reset('spring_'+str(value));a=emb._base.scene['coupling_fixture'];pos=a.data.joint_pos.torch.clone();pos[:]=value;a.write_joint_state_to_sim(pos,torch.zeros_like(pos));kit._OBSERVER.apply_passive();effort=-kit.FIX['parameters']['plunger_k']*value;spring.append(dict(q=value,predicted_spring_N=effort))
   rows,_=tick('EVALUATOR_spring_release_'+str(value),10);spring[-1]['measured_initial_q']=rows[0]['plunger_q'];spring[-1]['measured_final_q']=rows[-1]['plunger_q']
  report['spring_diagnostics']=spring
  # Prepared tray placement verifies sensor support/containment/history ONLY.
  tray=np.array([.05,.26,.812])+np.array(kit.FIX['translation']);place('tray',tray+[0,0,.0002]);rows,_=tick('EVALUATOR_prepared_tray21s',430)
  report['checks']['prepared_tray_floor_contact']=all(r['floor_contact'] for r in rows[-100:])
  report['checks']['prepared_tray_clear_no_robot_contact']=not any(r['post_contact'] or r['robot_contact'] for r in rows[-100:])
  report['checks']['prepared_unbroken20s']=kit.current_status()['source_binary_success'] is True
  report['checks']['unverified_score_gated']=kit.current_score()==0 and kit.current_status()['calibration_verified'] is False
  place('outside',tray+[.10,0,.0002]);rows,_=tick('EVALUATOR_over_wall_negative',30);report['checks']['outside_zero']=kit.current_status()['diagnostic_score']==0
  # Genuine force-sensor abort. Use actual measured durations as independent oracle.
  reset('wrist');tick('wrist_neutral',10);robot=emb._base.scene['right_robot'];f=torch.tensor([[[0.,0.,-120.]]],device=emb._base.device,dtype=torch.float32);robot.permanent_wrench_composer.add_forces_and_torques(f,torch.zeros_like(f),body_ids=robot.find_bodies('gripper')[0],is_global=True)
  samples=[];original=kit._OBSERVER.sample
  def sample(dt):
   original(dt)
   if kit._OBSERVER.last:samples.append(dict(dt=dt,force=kit._OBSERVER.last['wrist_force']))
  kit._OBSERVER.sample=sample;rows,reason=tick('EVALUATOR120N_wrist_abort',20,True);robot.permanent_wrench_composer.reset();streak=np.zeros(2);first=None
  for i,r in enumerate(samples):
   streak=np.where(np.array(r['force'])>80,streak+r['dt'],0.)
   if first is None and max(streak)>=.1-1e-10:first=i+1
  report['wrist_abort_measurements']=dict(samples=samples,first100ms_sample=first,termination=reason)
  report['checks']['actual_wrist80N100ms_abort']=first is not None and reason=='wrist_force_abort'
  before=kit._OBSERVER.history.samples
  try:emb.step(Action(data=q.copy()));report['checks']['step_after_termination_blocked']=False
  except __import__('step_guard').EpisodeStopped:report['checks']['step_after_termination_blocked']=kit._OBSERVER.history.samples==before
  report['checks']={k:bool(v) for k,v in report['checks'].items()};report['all_checks_passed']=all(report['checks'].values());report['controls']=completed;checkpoint('complete');return report
 finally:trace.close()
