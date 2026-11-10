"""Source repaired per-physics-step predicates, including mobile bracing closure.
Input is measured poses/contacts, never policy writes. No simulator dependency.
"""
import math
import numpy as np
from c1_contract import N_RACK,RACK_LOW_Z,TARGETS,supported
class History:
 def __init__(self,target='C1-T1'):
  self.need,self.need_wet=TARGETS[target];self.wet=[False]*6;self.samples=0;self.seconds=0.;self.failure=None
  self.ok_t=[0.]*2;self.ref=[None]*2;self.void=[False]*2;self.hold=[0.]*2;self.on_liner=[-1]*2;self.was_on_rack=[False]*2;self.release_z=[None]*2
  self.cup_fell=False;self.fell_reason=None;self.force_streak=np.zeros(2);self.last=None
 def update(self,row,dt):
  if not(0<dt<=.001001):raise ValueError('C1 must sample every <=1ms physical step')
  T=set(map(tuple,row['contacts']));T|={(b,a) for a,b in T};self.last=dict(row,contacts=T);self.wet=list(row['wet']);self.samples+=1;self.seconds+=dt
  F=np.asarray(row['wrist_force_n'],float)
  if F.shape!=(2,) or not np.isfinite(F).all():raise ValueError('Invalid actual wrist force')
  self.force_streak=np.where(F>80.,self.force_streak+dt,0.)
  if np.any(self.force_streak>=.1-1e-10):self.failure='wrist_force_limit'
  sup=supported(T);lsup=supported(T,False)
  for c in range(2):
   cc=10+c;p=np.asarray(row['cup_pos'][c]);R=np.asarray(row['cup_R'][c]);held=cc in sup
   lin=[i for i in range(6) if (cc,20+i) in T];on=[i for i in lin if (20+i,30) in T and 20+i not in lsup]
   if held:self.was_on_rack[c]=False;self.release_z[c]=float(p[2]);self.void[c]=False
   elif (cc,30) in T or on:self.was_on_rack[c]=True
   up=float(R[:,2]@N_RACK)>math.cos(math.radians(15))
   ok=not held and len(on)==1 and (cc,30) not in T and (cc,31) not in T and up
   if ok and self.ok_t[c]>0 and self.on_liner[c]!=on[0]:ok=False
   if self.void[c]:ok=False
   if ok:
    self.ok_t[c]+=dt;self.on_liner[c]=on[0]
    if self.ref[c] is None and self.ok_t[c]>=.5:self.ref[c]=p.copy()
    if self.ref[c] is not None:
     if np.linalg.norm(p-self.ref[c])>.005:ok=False
     else:self.hold[c]=self.ok_t[c]-.5
   if not ok:
    if self.ref[c] is not None and not held:self.void[c]=True
    self.ok_t[c]=0.;self.ref[c]=None;self.hold[c]=0.;self.on_liner[c]=-1
   if self.was_on_rack[c] and not held and (p[2]<RACK_LOW_Z-.01 or R[2,2]<.5):
    self.cup_fell=True;self.fell_reason=self.fell_reason or 'slid_off_or_tipped_from_rack'
   if not held:
    if R[2,2]<math.cos(math.radians(60)):
     self.cup_fell=True;self.fell_reason=self.fell_reason or 'tipped_over_unheld'
    elif self.release_z[c] is not None and self.release_z[c]-p[2]>.030 and (cc,31) in T:
     self.cup_fell=True;self.fell_reason=self.fell_reason or 'dropped_unheld'
 def score(self):
  if self.last is None:return {'score':0.,'raw_before_veto':0.,'items':[],'required':self.need}
  T=self.last['contacts'];sup=supported(T,False)
  liners=[i for i in range(6) if (20+i,30) in T and (20+i,31) not in T and 20+i not in sup and np.asarray(self.last['liner_R'][i])[:,2]@N_RACK>math.cos(math.radians(15)) and (not self.need_wet or self.wet[i])]
  items=[]
  for c in range(2):
   i=self.on_liner[c];h=self.hold[c]
   if self.ok_t[c]>0 and i in liners:items.append((1. if h>=20. else .75 if h>=10. else .5,i,c,h))
  items.sort(reverse=True);used=set();per=[]
  for v,i,c,h in items:
   if i not in used and len(per)<self.need:used.add(i);per.append({'value':v,'liner':i+1,'glass':c+1,'hold_s':round(h,3)})
  for i in liners:
   if len(per)>=self.need:break
   if i not in used:used.add(i);per.append({'value':.25,'liner':i+1,'glass':None,'hold_s':0.})
  raw=sum(x['value'] for x in per)/self.need
  return {'score':0. if self.cup_fell or self.failure else float(raw),'raw_before_veto':float(raw),'items':per,'required':self.need,'qualifying_liners':[i+1 for i in liners],'cup_fell':self.cup_fell,'fell_reason':self.fell_reason,'failure':self.failure,'physics_samples':self.samples,'physics_s':self.seconds}
