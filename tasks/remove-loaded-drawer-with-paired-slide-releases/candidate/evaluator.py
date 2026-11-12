"""Evaluator-only foundation loads/joint injection. Never used by reference policy."""
import numpy as np
from drawers_math import arr

def run(emb,obs,record):
 from inspect_robots import Scene
 from inspect_robots.types import Action
 import drawers_kit as kit
 import torch
 q=np.r_[kit.HOME,1.,kit.HOME,1.];results={'scope':'foundation diagnostics only; never robot feasibility or task score'}
 def step(label,n=1):
  nonlocal obs
  for _ in range(n):
   r=emb.step(Action(data=q.copy()));obs=r.observation;record(label,obs,r)
   if r.terminated:raise RuntimeError('foundation termination: '+str(r.termination_reason))
  return kit.ENGINE.last
 step('neutral',4)
 results['initial_status']=kit.ENGINE.status()
 if kit.ENGINE.error:raise RuntimeError(kit.ENGINE.error)
 results['body_paths']={'sensors':kit.ENGINE.sensor_paths,'filters':kit.ENGINE.filters}
 results['robots']={}
 for side in ('left','right'):
  robot=emb._base.scene[side+'_robot'];results['robots'][side]={'names':robot.joint_names,'caps':arr(robot.root_view.get_dof_max_forces()).tolist(),'gains':arr(robot.root_view.get_dof_stiffnesses()).tolist()}
 # Reset accepts harness-derived episode IDs/seeds and restores the SAME physical case.
 before={n:s[0].tolist() for n,s in kit.ENGINE.poses().items()};obs=emb.reset(Scene(id='drawers-foundation-reset',instruction='Foundation neutral reset'),seed=910842);q=np.r_[kit.HOME,1.,kit.HOME,1.];step('harness_seed_reset',4)
 results['reset_pose_deltas']={n:float(np.linalg.norm(s[0]-before[n])) for n,s in kit.ENGINE.poses().items()};results['clock_after_reset']=kit.ENGINE.status()
 # Actual paddle state injection is evaluator-only: checks measured release and
 # collision attr transitions, NOT that the stop physically blocks the bump.
 asset=emb._base.scene['drawer_U'];zeros=torch.zeros_like(asset.data.joint_pos.torch);pressed=zeros+.005
 asset.write_joint_state_to_sim(pressed,zeros);step('EVALUATOR_ONLY_paddle_injection',1)
 results['release_observed_stop_masks']=dict(kit.ENGINE.stop_on)
 obs=emb.reset(Scene(id='drawers-after-evaluator-injection',instruction='Foundation reset'),seed=42);q=np.r_[kit.HOME,1.,kit.HOME,1.];step('clean_reset',4)
 # Known wrist loads use the verified composer API. No load survives this
 # diagnostic; these logs cannot be replayed as a robot demonstration.
 results['wrist_loads']=[]
 for idx,side in enumerate(('left','right')):
  robot=emb._base.scene[side+'_robot'];ids=robot.find_bodies('gripper')[0];baseline=kit.ENGINE.last['wrist'][idx]
  f=torch.tensor([[[0.,0.,-10.]]],device=robot.device,dtype=torch.float32)
  try:
   robot.permanent_wrench_composer.reset();robot.permanent_wrench_composer.add_forces_and_torques(f,torch.zeros_like(f),body_ids=ids,is_global=True)
   row=step('EVALUATOR_ONLY_known_'+side+'_10N_load',10);results['wrist_loads'].append({'side':side,'baseline_norm':baseline,'loaded_norm':row['wrist'][idx],'warning':'norm difference alone does not calibrate complete wrench axes'})
  finally:robot.permanent_wrench_composer.reset()
 obs=emb.reset(Scene(id='drawers-final-clean-foundation',instruction='Foundation neutral'),seed=1234567);q=np.r_[kit.HOME,1.,kit.HOME,1.];step('final_clean_reset',4)
 results['final_status']=kit.ENGINE.status();results['passed_channels']=[];results['still_required']=['actual bump-stop blocking/release/re-arm contacts','jar pad/non-pad identity and closing-normal-speed calibration','compliant ring foot load/impact fidelity','80N100ms abort and subsequent-step rejection','total effort including gravity loaded checks','source40N finger cap measured','physical fullgoal','Dream-1 and matching media']
 return results
