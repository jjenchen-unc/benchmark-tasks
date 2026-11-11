"""EVALUATOR ONLY: known loads, sensors, clock/reset and termination guards.
No task/full-goal or policy success claimed. Does not activate credit channels.
"""
import json,time
import numpy as np
from kit import arr

def run(out,case=None):
 from inspect_robots import Scene
 from inspect_robots.types import Action
 from yam_arena.inspect_embodiment import yam_arena_embodiment
 import kit,torch
 emb=yam_arena_embodiment(task=str(__import__('pathlib').Path(__file__).parent/'task.py'),device='cpu',control_hz=20,object_jitter=0,cam_width=320,cam_height=176)
 checks={};cases={};actions=[]
 def reset(seed,label):
  obs=emb.reset(Scene(id='B6-evaluator-'+label,instruction='Evaluator diagnostic; no robot feasibility or model claim.'),seed=seed)
  E=kit.ENGINE
  if E.error:raise RuntimeError(E.error)
  return obs,E
 def step(q,n):
  last=None
  for _ in range(n):
   actions.append(q.tolist());last=emb.step(Action(data=q.copy()))
   if last.terminated:break
  return last
 try:
  obs,E=reset(42,'neutral');q=np.asarray(obs.state['joint_pos'],float);result=step(q,4);base=emb._base
  checks['finite14D_public_state']=q.shape==(14,) and bool(np.isfinite(q).all());checks['neutral_alive']=not result.terminated
  checks['clock_20Hz']=abs(E.history.t-E.controls*.05)<1e-9;checks['samples_500Hz']=E.history.samples==E.controls*25;checks['actual_clock_matches']=abs(E.clock()-E.started_sim_time-E.history.t)<1e-7
  checks['motor_caps_actual']=all(np.allclose(arr(base.scene[s+'_robot'].root_view.get_dof_max_forces())[0],[27,27,27,7,7,7,40,40]) for s in ['left','right']);checks['credit_gated']=E.status()['score']==0 and not E.status()['credit_channels_activated']
  cases['neutral']=dict(status=E.status(),last=E.last,link_paths=E.paths,robot_paths=E.robots,motor_caps={s:arr(base.scene[s+'_robot'].root_view.get_dof_max_forces()).tolist() for s in ['left','right']})
  # Actual harness seeds do not change the declared seed42 physical case.
  ref={n:arr(a.data.root_pos_w).tolist() for n,a in E.assets.items()}
  obs,E=reset(18394675,'derived-seed');q=np.asarray(obs.state['joint_pos'],float)
  checks['derived_seed_fixed_layout']=all(np.allclose(arr(a.data.root_pos_w),ref[n],atol=1e-5) for n,a in E.assets.items());checks['reset_clears_history']=E.controls==0 and E.history.samples==0 and not any(E.cap.values()) and not E.done
  for side in ['left','right']:
   obs,E=reset(1000003,'known-wrist-'+side);q=np.asarray(obs.state['joint_pos'],float);step(q,2);robot=base.scene[side+'_robot'];f=torch.zeros((1,len(robot.body_names),3),device=base.device);f[0,robot.body_names.index('gripper'),1]=120.
   robot.set_external_force_and_torque(f,torch.zeros_like(f),is_global=True)
   result=step(q,8);cases['wrist_'+side]=dict(status=E.status(),termination=result.termination_reason,external_evaluator_load_n=[0,120,0]);checks['wrist_abort_'+side]=result.terminated and result.termination_reason=='wrist_force_80N_100ms'
   t=E.history.t
   try:emb.step(Action(data=q));checks['step_after_termination_'+side]=False
   except RuntimeError:checks['step_after_termination_'+side]=E.history.t==t
   robot.set_external_force_and_torque(torch.zeros_like(f),torch.zeros_like(f),is_global=True)
  obs,E=reset(99,'final-reset');checks['final_reset_clean']=E.history.samples==0 and not E.done and not E.history.failure
  return dict(state='pass' if all(checks.values()) else 'failed_or_uncovered',scope=__doc__,checks=checks,cases=cases,physics_verified=bool(all(checks.values())),full_goal_verified=False,channels_activated=[],remaining_foundation=['actual capture positive/edge/wrong-family negatives','lock reaction known torque in both orientations','zoom helix/friction gravity creep vs hold','reset after captured and engaged pins','robot geometry/support/clearance and loaded reference'])
 finally:
  (out/'actions.json').write_text(json.dumps(actions));emb.close()
