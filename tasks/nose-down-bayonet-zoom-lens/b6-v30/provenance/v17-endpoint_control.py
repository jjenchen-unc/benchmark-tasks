"""Bounded robot-target endpoint correction; no physics/state/force writes."""
import numpy as np
from yam_arena import ik
class LoadedEndpoint:
 def __init__(self,nominal,command,measured,base):
  self.nominal=self.valid(nominal);self.command=self.valid(command);self.previous=self.valid(measured);self.base=np.asarray(base,float).copy()
  self.limit=np.minimum(.2,np.maximum(.004,2*np.abs(self.nominal-self.previous)))
  self.offset=np.zeros(6)
 @staticmethod
 def valid(value):
  v=np.asarray(value,float)
  if v.shape!=(6,) or not np.isfinite(v).all():raise ValueError('finite six robot joints required')
  return v.copy()
 def update(self,measured):
  q=self.valid(measured)
  offset=np.clip(self.offset+.04*(self.nominal-q)-1.0*(q-self.previous),-self.limit,self.limit)
  desired=np.clip(self.nominal+offset,ik.LOWER,ik.UPPER);delta=desired-self.command
  fraction=min(1.,.003/max(float(np.max(np.abs(delta))),1e-12));p,R=ik.tcp_pose(self.command,self.base)
  for _ in range(60):
   command=self.command+fraction*delta;pp,RR=ik.tcp_pose(command,self.base)
   if np.linalg.norm(pp-p)<=.002+1e-12 and np.linalg.norm(ik._rot_error(RR,R))<=.02+1e-12:break
   fraction*=.5
  else:raise RuntimeError('loaded endpoint command slew rejected')
  self.offset=offset;self.previous=q;self.command=command.copy();return command
