"""Disclosed state-aware teacher. All task changes arise from14D20Hz robot actions.
Reads actual valve/load state and source fixed hydraulic parameters. Never sets
poses, task joints, forces, latches, history, or score. Untested robot feasibility.
"""
import math
import numpy as np
from yam_arena import ik,station
import mount
from inspect_robots.types import Action
class Completed(Exception):pass
class Driver:
 def __init__(self,emb,obs,record):
  self.emb,self.obs,self.record=emb,obs,record;self.q=np.asarray(obs.state['joint_pos'],float).copy();self.yaw=math.pi;self.calls=0
 def step(self,q):
  q=np.asarray(q,float)
  if q.shape!=(14,) or not np.all(np.isfinite(q)):raise ValueError('finite 14D action required')
  for offset in (0,7):
   if np.any(q[offset:offset+6]<ik.LOWER-1e-6) or np.any(q[offset:offset+6]>ik.UPPER+1e-6) or not 0<=q[offset+6]<=1:raise ValueError('action outside original SDK limits')
  if np.max(abs(q-self.q))>.025:raise RuntimeError('controller joint increment exceeds .025')
  if self.calls>=3210:raise RuntimeError('teacher action budget exhausted')
  self.emb._hj_pre_action(self.obs,np.asarray(q,float))
  r=self.emb.step(Action(data=np.asarray(q,float)));self.calls+=1;self.obs=r.observation;self.q=np.asarray(q,float).copy();self.record(r,self.q)
  if r.terminated:
   if r.termination_reason=='success' and r.info.get('score')==1.:raise Completed('actual gated task success')
   raise RuntimeError('episode terminated: '+str(r.termination_reason))
  return r
 def move(self,pos,yaw=None,grip=None,duration=1.):
  from yam_arena.runtime import tcp_world
  start=tcp_world(self.emb._base.scene['right_robot']);_,rotation0=mount.tcp(np.asarray(self.obs.state['joint_pos'],float)[7:13]);pos=np.asarray(pos,float);yaw=self.yaw if yaw is None else float(yaw);y0=self.yaw
  n=max(round(duration*20),math.ceil(np.linalg.norm(pos-start)/.004),math.ceil(abs(yaw-y0)/.015),1)
  g0=self.q[13];goalgrip=g0 if grip is None else grip
  for k in range(1,n+1):
   s=k/n;p=start+(pos-start)*s;y=y0+(yaw-y0)*s;c,t=math.cos(y),math.sin(y);R=np.array([[-c,-t,0],[-t,c,0],[0,0,-1.]])
   R=ik.slerp_matrix(rotation0,R,s)
   q,e,r=mount.solve(p,R,self.q[7:13],iters=120)
   if e>.003 or r>.03:raise RuntimeError(f'IK rejected waypoint: {e}m {r}rad')
   action=self.q.copy();action[7:13]=q;action[13]=g0+(goalgrip-g0)*s;
   # Subdivide genuine public commands, including jaw closure, within existing cap.
   previous=self.q.copy();parts=max(1,math.ceil(float(np.max(abs(action-previous)))/.02))
   for j in range(1,parts+1):self.step(previous+(action-previous)*(j/parts))
  self.yaw=yaw
 def valve(self):
  import kit
  a=kit.ENGINE.jacks['S'];return float(kit.arr(a.data.joint_pos)[0,a.joint_names.index('valve_S')])
 def turn(self,target,point,rate=.20):
  for _ in range(600):
   error=target-self.valve()
   if abs(error)<math.radians(.3):return
   self.move(point,self.yaw+float(np.clip(error,-rate*.05,rate*.05)),duration=.05)
  raise RuntimeError('valve did not track robot grip')
 def run(self):
  import kit
  p=kit.F['parameters'];point=np.array([.26,.15,.8785])+kit.F['translation'];high=point+[0,0,.07]
  self.move(high,grip=1.,duration=3.);self.move(point,duration=2.);self.move(point,grip=0.,duration=1.)
  opening=math.radians(p['phi0_S']);weight=p['cabinet_weight'];delta=math.radians(10./((p['v30_S']/30.)*math.sqrt(weight/50.)))
  self.turn(opening+delta,point,rate=.15)
  while not kit.ENGINE.last['on_rails']:
   row=kit.ENGINE.last
   # State-aware slow descent feedback; no object-state writes.
   if row['vz']<-.015:delta*=.9;self.turn(opening+delta,point,rate=.10)
   elif row['vz']>-.004:delta*=1.1;self.turn(opening+delta,point,rate=.10)
   for _ in range(4):self.step(self.q)
  self.turn(max(0.,opening-math.radians(4)),point,rate=.20)
  self.move(point,grip=1.,duration=.6);self.move(high,duration=2.)
  # Both hands open, still; hold sampled by physics history, not teacher timers.
  for _ in range(80):
   q=self.q.copy();q[6]=q[13]=1.;self.step(q)
