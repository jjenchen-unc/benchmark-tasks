"""Passive environment mechanisms and read-only substep telemetry; no policy object writes."""
import json,os
from numeric_json import dumps
from pathlib import Path
import numpy as np
from kit import arr,rotation,initial,F,JOINTS
from contract import History,Mechanism
from contacts import Contacts
class Physics:
 def __init__(self,emb):
  self.emb=emb;self.base=emb._base;self.asset=self.base.scene['pins'];self.settling=True;self.history=History();self.mechanism=Mechanism();self.previous_velocity=None;self.time=0.;self.error=None;self.trace=None;self.episode=0
  from isaaclab_physx.physics import PhysxManager
  sim=PhysxManager.get_physics_sim_view();robot=[]
  for s in ['left','right']:robot+=list(self.base.scene[s+'_robot'].root_view.link_paths[0])
  # Static hole walls do not count as touched stand in source; use source stand prefix bodies.
  paths=list(self.asset.root_view.link_paths[0]);stand=[p for p in paths if p.split('/')[-1] in ['plate','slotR','instrument','legL','legR','pinL','pinR','knobbL','knobbR']]
  self.contacts=Contacts(sim,stand,robot)
  self.jids={n:i for i,n in enumerate(self.asset.joint_names)}
  if set(self.jids)!={j['name'] for j in JOINTS}:raise RuntimeError('compiled joint names differ from source')
  if os.environ.get('PINS_TRACE'):self.trace=open(os.environ['PINS_TRACE'],'w',buffering=1)
 def reset(self):
  self.episode+=1
  import torch
  q=torch.tensor([[initial()[n] for n in self.asset.joint_names]],device=self.base.device,dtype=self.asset.data.joint_pos.torch.dtype);z=torch.zeros_like(q)
  self.asset.write_joint_state_to_sim(q,z);self.asset.set_joint_effort_target(z);self.history=History();self.mechanism=Mechanism();self.previous_velocity=None;self.time=0.;self.error=None
  self.pre()
 def begin(self):
  self.settling=False;self.history=History();self.mechanism=Mechanism();self.previous_velocity=self.instrument_velocity();self.time=0.
 def instrument_velocity(self):
  i=self.asset.body_names.index('instrument');b=arr(self.asset.data.body_link_state_w)[0,i];R=rotation(b[3:7]);com=next(b for b in F['bodies'] if b['name']=='instrument')['com'];return b[7:10]+np.cross(b[10:13],R@com)
 def joints(self):
  q=dict(zip(self.asset.joint_names,arr(self.asset.data.joint_pos)[0]));v=dict(zip(self.asset.joint_names,arr(self.asset.data.joint_vel)[0]));return q,v
 def pre(self):
  import torch
  q,v=self.joints();limits=self.mechanism.limits(self.time,[q['leg'+l+'_z'] for l in 'LR'],[q['pin'+l+'_x'] for l in 'LR'],[q['knob'+l+'_rot'] for l in 'LR'])
  L=arr(self.asset.data.joint_pos_limits).copy()
  for n,lr in limits.items():L[0,self.jids[n]]=lr
  self.asset.write_joint_position_limit_to_sim_index(limits=torch.tensor(L,device=self.base.device,dtype=self.asset.data.joint_pos.torch.dtype),warn_limit_violation=False)
  target=torch.tensor([[next(j['ref'] for j in JOINTS if j['name']==n) for n in self.asset.joint_names]],device=self.base.device,dtype=self.asset.data.joint_pos.torch.dtype);self.asset.set_joint_position_target(target)
 def post(self,dt):
  if abs(dt-.002)>1e-8 or abs(float(self.base.sim.get_physics_dt())-dt)>1e-8:raise RuntimeError('actual physical clock differs from source 2ms')
  self.time+=dt
  if self.settling:return
  try:
   q,v=self.joints();velocity=self.instrument_velocity()
   acc=0. if self.previous_velocity is None else float(np.linalg.norm((velocity-self.previous_velocity)/dt)/9.81);self.previous_velocity=velocity.copy()
   hit,force,count=self.contacts.read(dt);wr=[]
   for s in ['left','right']:
    sensor=self.base.scene[s+'_pins_wrench'];wr.append(float(np.linalg.norm(arr(sensor.data.force)[0,sensor.body_names.index('gripper')])))
   row=dict(leg=[float(q['leg'+l+'_z']) for l in 'LR'],pin=[float(q['pin'+l+'_x']) for l in 'LR'],angle=[float(q['knob'+l+'_rot']) for l in 'LR'],leg_velocity=[float(v['leg'+l+'_z']) for l in 'LR'],robot_contact=hit,contact_count=count,wrist=wr,acceleration_g=acc,actual_dt=dt,instrument_com_velocity=velocity.tolist(),mechanism_limits=arr(self.asset.data.joint_pos_limits).tolist(),robot_contact_pairs=self.contacts.last_contacts)
   self.history.update(dt,row)
   if self.trace:self.trace.write(dumps(dict(episode=self.episode,time=self.time,measurement=row,status=self.status()))+'\n')
  except Exception as e:
   self.error=type(e).__name__+': '+str(e);self.history.failure=self.history.failure or 'measurement_error';raise
 def status(self):return dict(self.history.result(),measurement_error=self.error)
 def close(self):
  if self.trace:self.trace.close();self.trace=None
