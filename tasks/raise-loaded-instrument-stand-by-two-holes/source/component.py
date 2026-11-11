"""Evaluator-only applied loads; no placement, flags, credit activation or robot demo."""
import numpy as np
import os
from numeric_json import dumps
from kit import arr

def evaluate(emb,obs,actions):
 import kit,torch
 from inspect_robots import Scene
 from inspect_robots.types import Action
 E=kit.ENGINE;out={'scope':__doc__,'probes':[]}
 def reset(label):
  o=emb.reset(Scene(id='pins-component-'+label,instruction='Evaluator calibration only'),seed=19358472)
  return np.asarray(o.state['joint_pos'],float)
 def load(asset,body,force):
  z=torch.zeros((1,len(asset.body_names),3),device=emb._base.device)
  f=z.clone();f[0,asset.body_names.index(body)]=torch.tensor(force,device=z.device)
  asset.set_external_force_and_torque(f,z,is_global=True)
 def sample(q,n,label):
  rows=[]
  for _ in range(n):
   r=emb.step(Action(data=q.copy()));actions.append(q.tolist())
   rows.append({'status':E.status(),'raw':E.history.last,'contact_pairs':E.contacts.last_contacts})
   if r.terminated:break
  out['probes'].append({'label':label,'rows':rows})
  with open(os.environ['PINS_COMPONENT_TRACE'],'a') as f:f.write(dumps(out['probes'][-1])+'\n')
  return r
 # Both wrists: applied20N is a safe response probe,100N tests latched80N/0.1s abort.
 for side in ['left','right']:
  for force in [20.,100.]:
   q=reset(side+str(force));rob=emb._base.scene[side+'_robot']
   try:
    load(rob,'gripper',[0,force,0]);r=sample(q,10,side+'_world_y_'+str(force)+'N')
    out['probes'][-1]['termination_reason']=r.termination_reason
    if r.terminated:
     try:emb.step(Action(data=q.copy()));out['probes'][-1]['after_stop_rejected']=False
     except RuntimeError:out['probes'][-1]['after_stop_rejected']=True
   finally:load(rob,'gripper',[0,0,0])
 # Fixed-load mechanism probe:30N outward pin pull under native seated load.
 # A failure to withdraw is useful binding evidence, never a software unlock.
 for leg in 'LR':
  q=reset('pin'+leg)
  try:
   load(E.asset,'pin'+leg,[0,30.,0]);sample(q,20,'loaded_pin'+leg+'_30N_outward')
   load(E.asset,'pin'+leg,[0,0,0])
   if not emb._pins_done:sample(q,20,'pin'+leg+'_spring_return')
  finally:load(E.asset,'pin'+leg,[0,0,0])
 # Instrument impulse response measured using real COM velocities at2ms.
 # Constrained stand: response is not assumed to equal F/m.
 q=reset('instrument')
 try:
  load(E.asset,'instrument',[5.,0,0]);sample(q,2,'instrument_5N_x_100ms')
  load(E.asset,'instrument',[0,0,0])
  if not emb._pins_done:sample(q,4,'instrument_release')
 finally:load(E.asset,'instrument',[0,0,0])
 out['credit_channels_activated']=[];out['full_goal_verified']=False
 return out
