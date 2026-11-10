"""Evaluator-only injected force negative. NEVER a policy/demo/feasibility driver."""
import json,numpy as np
from inspect_robots.types import Action

def run(emb,obs,out):
 import torch
 from s5_kit import ENGINE,arr
 q=np.asarray(obs.state['joint_pos'],float);robot=emb._base.scene['right_robot'];force=torch.tensor([[[0.,0.,-120.]]],device=emb._base.device)
 robot.permanent_wrench_composer.reset();robot.permanent_wrench_composer.add_forces_and_torques(force,torch.zeros_like(force),body_ids=robot.find_bodies('gripper')[0],is_global=True)
 rows=[];terminated=False
 try:
  for _ in range(10):
   r=emb.step(Action(data=q));rows.append(dict(reason=r.termination_reason,status=ENGINE.status()))
   if r.terminated:terminated=True;break
 finally:robot.permanent_wrench_composer.reset()
 before=ENGINE.hist.time;refused=False
 if terminated:
  try:emb.step(Action(data=q))
  except RuntimeError:refused=True
 after=ENGINE.hist.time
 return dict(evaluator_only=True,force_applied_n=120,rows=rows,terminated=terminated,posttermination_refused=refused,no_extra_physics=before==after,score_claim=None)
