"""EVALUATOR ONLY paired known-load readings. No reference-controller import.
Loads robot bodies only. Does not place task objects or advance native timers.
Normal physics sampling/clock continues; this is never robot feasibility proof.
"""
import numpy as np

def run(emb,q,record):
 import torch,kit
 from kit import arr
 from inspect_robots.types import Action
 receipts=[]
 def tick(label,n):
  rows=[]
  for _ in range(n):
   result=emb.step(Action(data=q));record('EVALUATOR_ONLY_'+label,q,result)
   if result.terminated:raise RuntimeError('calibration terminated: '+str(result.termination_reason))
   rows.append(kit.ENGINE.last)
  return rows
 def snapshot():
  engine=kit.ENGINE
  value={'bar_raw':{},'contact_selections':{},'contacts':{}}
  for c in 'ABC':
   view=emb._base.scene['bar_'+c].root_view
   value['bar_raw'][c]={'transforms':arr(view.get_transforms()).tolist(),'velocities':arr(view.get_velocities()).tolist()}
  for c,collector,labels in engine.pairs:
   view=collector.view;normal,points,normals,separation,count,start=view.get_contact_data(.002)
   value['contact_selections'][c]={'selection':collector.selection,'labels':labels,'capacity':collector.capacity}
   value['contacts'][c]={'count':arr(count).tolist(),'start':arr(start).tolist(),'normal':arr(normal).tolist(),'points':arr(points).tolist(),'normals':arr(normals).tolist(),'separation':arr(separation).tolist()}
  collector=engine.link_contacts;view=collector.view
  normal,points,normals,separation,count,start=view.get_contact_data(.002)
  friction,fpoints,fcount,fstart=view.get_friction_data(.002)
  value['nonfinger']={'selection':collector.selection,'capacity':collector.capacity,'normal':arr(normal).tolist(),'count':arr(count).tolist(),'start':arr(start).tolist(),'friction':arr(friction).tolist(),'friction_count':arr(fcount).tolist(),'friction_start':arr(fstart).tolist()}
  return value
 tick('neutral_settle',40);baseline=tick('baseline',20)[-1]
 receipts.append({'kind':'baseline','raw':snapshot(),'row':baseline})
 # Both grippers, signed orthogonal global forces. Force magnitude is invariant
 # under the installed sensor's orthonormal child-side joint transform.
 for side in ('left','right'):
  robot=emb._base.scene[side+'_robot'];bodyid=robot.find_bodies('gripper')[0]
  for axis in range(3):
   for sign in (-1,1):
    value=np.zeros(3);value[axis]=sign*10.
    force=torch.tensor(value.reshape(1,1,3),device=emb._base.device,dtype=torch.float32)
    robot.permanent_wrench_composer.reset()
    try:
     robot.permanent_wrench_composer.add_forces_and_torques(force,torch.zeros_like(force),body_ids=bodyid,is_global=True)
     rows=tick(side+'_axis'+str(axis)+'_'+str(sign),20)
     receipts.append({'kind':'wrist_known_load','side':side,'applied_force_world_n':value.tolist(),'row':rows[-1],'raw':snapshot()})
    finally:robot.permanent_wrench_composer.reset()
    tick('unloaded',20)
 # Actual 120 N abort is a separate fresh-launcher existing wrist_abort variant;
 # no positive nonfinger calibration is inferred from its zero-contact samples.
 return {'evaluator_only':True,'credit_promoted':False,'readings':receipts,'unresolved':'No known nonfinger contact load is claimed; root should first inspect source-bound zero-contact proof rather than require a blind whole-run retry.'}
