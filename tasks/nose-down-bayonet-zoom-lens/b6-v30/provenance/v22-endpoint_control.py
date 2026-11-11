"""Bounded robot-target endpoint correction; no physics/state/force writes."""
import numpy as np
from yam_arena import ik
class LoadedEndpoint:
 def __init__(self,nominal,command,measured,base):
  self.nominal=self.valid(nominal);self.command=self.valid(command);self.previous=self.valid(measured);self.base=np.asarray(base,float).copy()
  initial_offset=self.command-self.nominal
  if np.any(np.abs(initial_offset)>.2+1e-12):raise ValueError('previous loaded command exceeds endpoint offset envelope')
  self.limit=np.minimum(.2,np.maximum(np.maximum(.004,2*np.abs(self.nominal-self.previous)),2*np.abs(initial_offset)))
  self.offset=initial_offset.copy()
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

class MovingLoad:
 """Same damped integral around a moving legal IK reference.

 Damping uses measured motion minus intended motion, so it does not oppose
 the planned path. The issued correction offset has anti-windup and bounded
 joint/Cartesian increments; task forces, drives and poses are never written.
 """
 def __init__(self,nominal,command,measured,base):
  self.nominal=LoadedEndpoint.valid(nominal);self.command=LoadedEndpoint.valid(command)
  self.previous=LoadedEndpoint.valid(measured);self.base=np.asarray(base,float).copy()
  self.offset=self.command-self.nominal
  if np.any(np.abs(self.offset)>.2+1e-12):raise ValueError('previous held command exceeds moving offset envelope')
 def update(self,nominal,measured):
  n=LoadedEndpoint.valid(nominal);q=LoadedEndpoint.valid(measured)
  error=n-q;motion_error=(q-self.previous)-(n-self.nominal)
  wanted=np.clip(self.offset+.04*error-1.0*motion_error,-.2,.2)
  delta=wanted-self.offset;fraction=min(1.,.003/max(float(np.max(np.abs(delta))),1e-12))
  # Bound the additional correction relative to the unmodified IK path step.
  baseline=n+self.offset;p,R=ik.tcp_pose(baseline,self.base)
  for _ in range(60):
   issued=self.offset+fraction*delta;command=n+issued;pp,RR=ik.tcp_pose(command,self.base)
   if np.linalg.norm(pp-p)<=.002+1e-12 and np.linalg.norm(ik._rot_error(RR,R))<=.02+1e-12:break
   fraction*=.5
  else:raise RuntimeError('moving load correction slew rejected')
  if np.any(command<ik.LOWER) or np.any(command>ik.UPPER):raise RuntimeError('moving loaded command outside public joint limits')
  self.offset=issued.copy();self.previous=q;self.nominal=n;self.command=command.copy();return command
