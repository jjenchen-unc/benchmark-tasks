"""Pure evaluator math; no physics/static evidence enables credit."""
import math
from collections import deque
import numpy as np
REQUIRED=frozenset({'contact_pairs','bar_pose_velocity','wrist_force','physics_clock','motor_limits','nonfinger_link_force','workspace_contact'})
from validation_receipt import validated_channels
def rotation(q):
 x,y,z,w=q
 return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
class History:
 def __init__(self,order='ABC'):
  self.order=order;self.hold=np.zeros(len(order));self.maximum=self.hold.copy();self.time=0.;self.samples=0;self.failure=None;self.force_duration=np.zeros(2);self.guard_duration=np.zeros(2);self.guard=False;self.last=None;self.link_window=deque();self.link_guard=False
 def update(self,row,dt):
  if not math.isfinite(dt) or not 0<dt<=.00200001:raise ValueError('unresolved physics clock')
  F=np.asarray(row['wrist_force_n'],float)
  if F.shape!=(2,) or not np.isfinite(F).all():raise ValueError('invalid wrist force')
  self.force_duration=np.where(F>80,self.force_duration+dt,0.)
  self.guard_duration=np.where(F>30,self.guard_duration+dt,0.)
  self.link_window.append((self.time+dt,dict(row['link_force_n']),dt))
  while self.link_window and self.link_window[0][0]<self.time+dt-.020+dt/2:self.link_window.popleft()
  totals={}
  for _,sample,duration in self.link_window:
   for body,force in sample.items():
    if not math.isfinite(force) or force<0:raise ValueError('invalid nonfinger force')
    totals[body]=totals.get(body,0.)+force*duration/.020
  self.link_guard=max(totals.values(),default=0.)>50.
  self.guard=bool(np.any(self.guard_duration>=.02-1e-10)) or self.link_guard
  if np.any(self.force_duration>=.1-1e-10):self.failure='wrist_force_abort'
  if row.get('outside_workspace_robot_bar_contact',False):self.failure='workspace_contact_violation'
  self.time+=dt;self.samples+=1;self.last=row
  pairs={frozenset(p) for p in row['contacts']}
  robot=any('robot' in p and any(b in p for b in 'ABC') for p in pairs)
  good=[]
  for s in range(len(self.order)):
   ok=not robot
   for j,c in enumerate(self.order[:s+1]):
    below='support' if j==0 else self.order[j-1]
    allowed={below}|({self.order[j+1]} if j<s else set())
    touches=set().union(*(p-{c} for p in pairs if c in p)) if any(c in p for p in pairs) else set()
    b=row['bars'][c];R=rotation(b['quat_xyzw']);v=np.r_[b['lin_vel'],b['ang_vel']]
    if not np.isfinite(np.r_[R.ravel(),v]).all():raise ValueError('nonfinite bar state')
    ok=ok and below in touches and not(touches-allowed) and abs(R[2,0])<=math.sin(.06) and max(abs(R[2,1]),abs(R[2,2]))>=math.cos(.06) and abs(R[1,0])<=math.sin(math.radians(10)) and np.linalg.norm(v)<.025
   good.append(ok)
  self.hold=np.where(good,self.hold+dt,0.);self.maximum=np.maximum(self.maximum,self.hold)
 def result(self,channels=()):
  k=max([i+1 for i,v in enumerate(self.hold) if v>=20-1e-9],default=0)
  measured=k/len(self.order) if not self.failure else 0.
  missing=sorted(REQUIRED-(set(channels)&validated_channels()))
  return {'score':measured if not missing else 0.,'diagnostic_ungated_score':measured,'missing_runtime_validation':missing,'failure':self.failure,'hold_s':self.hold.tolist(),'max_hold_s':self.maximum.tolist(),'samples':self.samples,'time_s':self.time,'force_guard':self.guard}
