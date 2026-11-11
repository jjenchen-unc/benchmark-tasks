"""Numeric terminal/history scorer. Pure evaluator, no simulation writes."""
import math
REQUIRED=frozenset(['clock','joint_state','robot_contacts','wrist_force','instrument_acceleration','pin_mechanism','total_motor_limits','reset'])
VALIDATED=frozenset() # Root runtime evidence required; native/static claims do not enable credit.
class History:
 def __init__(self):
  self.time=0.;self.controls=0;self.hold=0.;self.partial=0.;self.ema=0.;self.peak_g=0.;self.peak_drop=0.;self.over=[0.,0.];self.failure=None;self.last=None
 def update(self,dt,row):
  if not math.isfinite(dt) or dt<=0 or abs(dt-.002)>1e-8:raise ValueError('pins requires actual 2 ms substeps')
  vals=[*row['leg'],*row['pin'],*row['wrist'],row['acceleration_g'],*row['leg_velocity']]
  if not all(math.isfinite(v) for v in vals):self.failure='nonfinite';return
  self.time+=dt;self.last=row
  self.ema+=dt/(.005+dt)*(row['acceleration_g']-self.ema);self.peak_g=max(self.peak_g,self.ema)
  drop=max(0.,-min(row['leg_velocity']));self.peak_drop=max(self.peak_drop,drop)
  if self.ema>3:self.failure=self.failure or 'instrument_jolt'
  if drop>.2:self.failure=self.failure or 'plate_drop'
  for a,w in enumerate(row['wrist']):
   self.over[a]=self.over[a]+dt if w>80 else 0.
   if self.over[a]>=.1-1e-9:self.failure=self.failure or 'wrist_abort'
  safe=not self.failure and not row['robot_contact'] and all(v>-.003 for v in row['pin']) and abs(row['leg'][0]-row['leg'][1])<=.003
  full=safe and all(abs(v-.075)<=.0015 for v in row['leg']);half=safe and all(abs(v-.05)<=.0015 for v in row['leg'])
  self.hold=self.hold+dt if full else 0.;self.partial=self.partial+dt if half else 0.
 def result(self):
  raw=0. if self.failure else 1. if self.hold>=1-1e-9 else .5 if self.partial>=1-1e-9 else 0.
  return dict(score=raw if REQUIRED<=VALIDATED else 0.,diagnostic_score=raw,gate=None if REQUIRED<=VALIDATED else 'runtime_channels_unverified',missing=sorted(REQUIRED-VALIDATED),failure=self.failure,time_s=self.time,controls=self.controls,hold_s=self.hold,partial_hold_s=self.partial,peak_g=self.peak_g,peak_drop_m_s=self.peak_drop)
class Mechanism:
 """Passive conditional joint stops, translated verbatim from the source rules."""
 def __init__(self):
  self.prev=[0.,0.];self.turned=[False,False];self.rest=[-.0106,-.0106];self.lo=[0,None];self.clear=[-1.,-1.];self.hist=[[],[]];self.tick=0
 def limits(self,t,leg,pin,angle):
  out={}
  for i,l in enumerate('LR'):
   a=angle[i];qp=pin[i]
   if self.turned[i]:
    if a<.15:self.turned[i]=False
   elif a>=.15>self.prev[i] and qp<-.0102:self.turned[i]=True;self.rest[i]=max(-.0106,qp)
   out['pin'+l+'_x']=(-.012,self.rest[i] if self.turned[i] else 0.)
   out['knob'+l+'_rot']=(0.,1.6 if self.turned[i] or qp<-.0102 else .15);self.prev[i]=a
  # Representative fixed case: only L has a catch; gas assist is R.
  i=0;q=leg[i];h=self.hist[i]
  if self.tick%10==0:
   h.append((t,q));h[:]=[x for x in h if t-x[0]<=1.]
  if q-min([x[1] for x in h]+[q])>=.005:self.clear[i]=t+1.
  pw=self.lo[i]
  if t<self.clear[i]:pw=None
  elif pw is None:pw=math.floor((q-.004)/.025)
  elif q>.025*(pw+1)+.004:pw+=1
  self.lo[i]=pw;out['legL_z']=(-.05 if pw is None else max(-.05,.025*pw-.0006),.13);out['legR_z']=(-.05,.13)
  self.tick+=1;return out
