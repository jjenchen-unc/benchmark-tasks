# Evaluator channels: true contact presence and sustained declared wrist force.
from __future__ import annotations
import math

HOOKS=tuple(f"strap_{i}_{e}" for i in range(1,5) for e in ("a","b"))
class GuardState:
 def __init__(self):self.reset()
 def reset(self):
  self.over={"left":0.,"right":0.};self.forces={};self.failure=None
  self.wrench_measured=False;self.contacts_measured=False
  self.interval_contacts={n:False for n in (*HOOKS,"lid")};self.substeps=0
 def sample(self,forces,contacts,dt):
  self.substeps+=1
  valid=(isinstance(forces,dict) and set(forces)=={"left","right"} and all(math.isfinite(x) and x>=0 for x in forces.values()))
  self.wrench_measured=valid
  if not valid:self.failure=self.failure or "missing_wrist_measurement"
  else:
   self.forces=dict(forces)
   for side,f in forces.items():
    self.over[side]=self.over[side]+dt if f>80. else 0.
    if self.over[side]>=.1-1e-9:self.failure=self.failure or "wrist_force_abort"
  valid_contacts=(isinstance(contacts,dict) and set(contacts)==set(self.interval_contacts) and all(isinstance(v,bool) for v in contacts.values()))
  self.contacts_measured=valid_contacts
  if not valid_contacts:self.failure=self.failure or "missing_contact_measurement"
  else:
   for name,value in contacts.items():self.interval_contacts[name]|=value
 def hooks_off(self,name):return self.contacts_measured and not self.interval_contacts[name]
 def fields(self):
  return dict(wrist_force_measured=self.wrench_measured,wrist_forces_n=dict(self.forces),
              wrist_over_s=dict(self.over),robot_contacts_measured=self.contacts_measured,
              robot_contacts=dict(self.interval_contacts),guard_failure=self.failure)
 def finish_control(self):
  self.interval_contacts={n:False for n in self.interval_contacts}

def sensor_channels(base,arr):
 # Use contact positions, not net force: opposing finger forces must not cancel a grip.
 forces={};contacts={}
 for side in ("left","right"):
  sensor=base.scene.sensors.get(side+"_d1r_wrench")
  if sensor is None:return None,None
  import numpy as np
  f=arr(sensor.data.force)[0,sensor.body_names.index("gripper")]
  forces[side]=float(np.linalg.norm(f))
 for name in (*HOOKS,"lid"):
  sensor=base.scene.sensors.get("d1r_contact_"+name)
  if sensor is None or sensor.data.contact_pos_w is None:return forces,None
  points=arr(sensor.data.contact_pos_w)
  if points.shape[-1]!=3:return forces,None
  # Finite contact position denotes a manifold even when its net force is zero.
  finite=np.isfinite(points)
  if np.any(np.any(finite,axis=-1)&~np.all(finite,axis=-1)):return forces,None
  contacts[name]=bool(np.any(np.all(finite,axis=-1)))
 return forces,contacts
