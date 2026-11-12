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
 def update(self,nominal,measured,target_p=None,target_R=None):
  requested=LoadedEndpoint.valid(nominal);q=LoadedEndpoint.valid(measured)
  if np.any(requested<ik.LOWER) or np.any(requested>ik.UPPER):raise ValueError('moving IK nominal outside public joint limits')
  if np.any(self.nominal<ik.LOWER) or np.any(self.nominal>ik.UPPER):raise ValueError('previous accepted nominal outside public joint limits')
  if np.any(self.command<ik.LOWER) or np.any(self.command>ik.UPPER):raise ValueError('previous moving command outside public joint limits')
  if target_p is None and target_R is None:target_p,target_R=ik.tcp_pose(requested,self.base)
  elif target_p is None or target_R is None:raise ValueError('complete requested TCP pose required')
  target_p=np.asarray(target_p,float);target_R=np.asarray(target_R,float)
  if target_p.shape!=(3,) or target_R.shape!=(3,3) or not np.isfinite(target_p).all() or not np.isfinite(target_R).all():raise ValueError('finite requested TCP pose required')
  for advance in (1.,.5,.25,.125,.0625,.03125,.015625,.0078125,0.):
   n=requested if advance==1. else (self.nominal.copy() if advance==0. else self.nominal+advance*(requested-self.nominal))
   if np.any(n<ik.LOWER) or np.any(n>ik.UPPER):continue
   np_,nR=ik.tcp_pose(n,self.base)
   if np.linalg.norm(np_-target_p)>.003 or np.linalg.norm(ik._rot_error(target_R,nR))>.03:continue
   # Every trial uses the SAME last accepted nominal and measured state.
   error=n-q;motion_error=(q-self.previous)-(n-self.nominal)
   wanted=np.clip(self.offset+.04*error-1.0*motion_error,-.2,.2)
   delta=wanted-self.offset;fraction=min(1.,.003/max(float(np.max(np.abs(delta))),1e-12))
   baseline=n+self.offset;p,R=ik.tcp_pose(baseline,self.base)
   # Necessary joint correction to any legal command; projection cannot evade cap.
   distance=np.maximum(np.maximum(ik.LOWER-baseline,baseline-ik.UPPER),0.)
   if np.max(distance)>.003+1e-12:continue
   for _ in range(60):
    issued=self.offset+fraction*delta;command=n+issued
    legal=np.clip(command,ik.LOWER,ik.UPPER)
    if not np.array_equal(legal,command):command=legal;issued=command-n
    pp,RR=ik.tcp_pose(command,self.base)
    if (np.max(np.abs(issued-self.offset))<=.003+1e-12 and np.max(np.abs(issued))<=.2+1e-12 and np.linalg.norm(pp-p)<=.002+1e-12 and np.linalg.norm(ik._rot_error(RR,R))<=.02+1e-12):
     self.offset=issued.copy();self.previous=q;self.nominal=n;self.command=command.copy();self.last_nominal_fraction=advance;self.last_correction_fraction=fraction;return command
    fraction*=.5
  raise RuntimeError('no legal moving nominal/command satisfies original correction and IK guards')
