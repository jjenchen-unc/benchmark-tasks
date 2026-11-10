"""Pure passive mechanism and numeric history. No simulator imports or policy writes."""
import math
DEG=math.pi/180;RK=.136;MAPS={'L/R':{1:'L',2:'R'},'R/L':{1:'R',2:'L'},'LR/R':{1:'LR',2:'R'},'R/LR':{1:'R',2:'LR'}}
# No S5 runtime credit channel is validated by authoring/static/native evidence.
VALIDATED_CHANNELS=frozenset()
REQUIRED=frozenset(['geometry','mechanism','clock','contacts','wrist_force','motor_caps','reset'])
def torque(theta,f2,two,latch,armed):
 tau=0.
 if theta<12*DEG:tau+=3*RK*(12*DEG-theta)/(12*DEG)
 if theta<1.2*DEG:tau+=f2*RK*(1.2*DEG-theta)/(1.2*DEG)
 if two and armed and latch=='open' and abs(theta-2.1*DEG)<.7*DEG:tau+=8*RK*math.sin(math.pi*(theta-2.1*DEG)/(.7*DEG))
 return tau
class Latches:
 def __init__(self,cabinets):
  self.cabinets=cabinets;self.latch={n+s:'stage2' for n in cabinets for s in 'LR'};self.armed={k:False for k in self.latch};self.events=[]
 def update(self,q,t):
  held=set()
  for n,c in self.cabinets.items():
   for i in [1,2]:
    if q[f'{n}_paddle{i}_slide']>=.009:held.update(n+s for s in MAPS[c['map']][i])
  for key,ls in list(self.latch.items()):
   th=q[f'{key[0]}_hinge_{key[1]}'];c=self.cabinets[key[0]]
   if key in held:
    if ls!='open':self.latch[key]='open';self.armed[key]=False;self.events.append([t,key,'release'])
    continue
   if ls=='open' and not self.armed[key]:
    if th>2.8*DEG:self.armed[key]=True
    continue
   if ls=='open' and c['two'+key[1]] and th<1.35*DEG:self.latch[key]=ls='stage1';self.events.append([t,key,'stage1'])
   if ls in ['open','stage1'] and th<.10*DEG:self.latch[key]='stage2';self.events.append([t,key,'stage2'])
 def upper(self,key):return {'open':140*DEG,'stage1':1.35*DEG,'stage2':.15*DEG}[self.latch[key]]
class History:
 def __init__(self):self.time=0.;self.hold=0.;self.over=[0.,0.];self.failure=None;self.samples=0;self.prev={};self.peak_force=[0.,0.];self.max_impact=0.;self.last=None
 def add(self,row,dt):
  if not 0<dt<=.05:raise ValueError('invalid physics dt')
  self.samples+=1;self.time+=dt
  for i,f in enumerate(row['wrist_force_n']):
   if not math.isfinite(f):raise ValueError('nonfinite force')
   self.peak_force[i]=max(self.peak_force[i],f);self.over[i]=self.over[i]+dt if f>80 else 0.
   if self.over[i]>=.1-1e-10:self.failure='wrist_overload'
  for key,th in row['door_q'].items():
   prev=self.prev.get(key,th)
   if th<=-.13*DEG<prev:
    speed=max(abs(row['door_v'][key]),(prev-th)/dt)*RK;self.max_impact=max(self.max_impact,speed)
    if speed>.25:self.failure='door_slam'
   self.prev[key]=th
  # Full source predicate plus measured physical table support. Off-table and
  # dropped boxes cannot gain credit merely from centre height or world velocity.
  box=row['box'];ok=abs(box['z']-(.7475+.03))<=.004 and box['upright_cos']>=math.cos(15*DEG) and box['spatial_speed']<=.05 and box['outside_cabinet'] and box['inside_table'] and box['table_contact'] and not box['robot_contact']
  self.hold=self.hold+dt if ok and self.failure is None else 0.;self.last=dict(row,final_predicate=bool(ok))
 def result(self):
  raw=0.
  if not self.failure and self.last:
   if self.last['released'] and self.last['door_q']['AR']>=7*DEG:raw=.25
   if self.last['final_predicate']:raw=.5
   if self.hold>=20-1e-8:raw=1.
  missing=sorted(REQUIRED-VALIDATED_CHANNELS)
  return dict(score=0. if missing else raw,diagnostic_score=raw,missing_runtime_validation=missing,hold_s=self.hold,time_s=self.time,samples=self.samples,failure=self.failure,max_force_n=self.peak_force,max_impact_m_s=self.max_impact)
