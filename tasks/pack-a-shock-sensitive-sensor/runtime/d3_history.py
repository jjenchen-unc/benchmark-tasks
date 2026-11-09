"""D3 read-only measured geometry/history. No physics or object-state writes.
A real backend must supply every physical substep and separately verified channels.
Synthetic checks do not establish runtime feasibility or submission readiness.
"""
import math
import numpy as np
from d3_force_guard import DurationGuard
from d3_rubric import evaluate
KNOWN_ITEMS=frozenset(['A','B']+[f'carton{i}' for i in range(1,5)])
CHANNELS=('clock','shock','contact','wrist','package_force','latch','drop')
def rotation(q):
 q=np.asarray(q,dtype=float)
 if q.shape!=(4,) or not np.isfinite(q).all() or abs(float(q@q)-1.)>.002:raise ValueError('Invalid measured quaternion')
 x,y,z,w=q
 return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def state(value):
 s=np.asarray(value,dtype=float)
 if s.shape!=(13,) or not np.isfinite(s).all():raise ValueError('Missing/nonfinite measured state')
 rotation(s[3:7]);return s
def angle_axis(R0,R1):
 return math.degrees(math.acos(min(1.,abs(float(R0[:,2]@R1[:,2])))))
def geometry(box,carton,rest_z):
 b,c=state(box),state(carton['state']);B,C=rotation(b[3:7]),rotation(c[3:7]);local=C.T@(b[:3]-c[:3]);q=carton['lid_q']
 if not isinstance(q,(int,float)) or isinstance(q,bool) or not math.isfinite(q) or type(carton['latched']) is not bool:raise ValueError('Missing physical lid state')
 inside=bool(abs(local[0])<.085 and abs(local[1])<.065 and 0.<local[2]<.080)
 flat=angle_axis(B,C);tilt=angle_axis(C,np.eye(3));low=float(local[2]-.025);rise=float(c[2]-rest_z)
 packed=inside and flat<=10. and 0.<low<=.035 and q<=.012 and carton['latched'] and tilt<=5. and abs(rise)<=.004
 return {'inside':inside,'packed':packed,'local_m':local.tolist(),'low_face_m':low,'box_flat_deg':flat,'carton_tilt_deg':tilt,'rise_m':rise}
class MeasuredHistory:
 def __init__(self,rest_z,required=('A',)):
  self.rest_z=dict(rest_z)
  if set(self.rest_z)!=set(range(1,5)) or any(not math.isfinite(float(v)) for v in self.rest_z.values()):raise ValueError('Four measured rest heights required')
  self.required=tuple(required);self.validated={k:False for k in CHANNELS};self.invalid_reason=None;self.samples=0;self.seconds=0.
  self.wrist=[DurationGuard(),DurationGuard()];self.package=[DurationGuard(),DurationGuard()]
  self.peak_any={n:0. for n in 'AB'};self.sensor_fresh=False;self.diverged=False;self.source_contact_veto=False;self.independent_contact_veto=False
  self.drops=[];self.trackers={i:{'phase':'rest'} for i in range(1,5)};self.source_drops=[];self.source_open=[]
  self.final={n:{'carton':None,'hold_s':0.,'packed_now':False,'hands_off':False} for n in self.required};self.hold_ref={n:None for n in self.required}
  self.last_geometry=None;self.last_touched=set()
 def add(self,dt,cartons,boxes,touched,direct,wrist_force_n,package_force_n,shock,source_drops=(),source_open=(),source_contact_veto=False):
  if self.invalid_reason:raise ValueError(self.invalid_reason)
  try:
   if not isinstance(dt,(int,float)) or isinstance(dt,bool) or not math.isfinite(dt) or not 0<dt<=.002:raise ValueError('Underresolved/missing physics clock')
   if set(cartons)!=set(range(1,5)) or set(boxes)!=set('AB'):raise ValueError('Incomplete physical body inventory')
   touched,direct=set(touched),set(direct)
   if not touched<=KNOWN_ITEMS or not direct<=touched:raise ValueError('Unresolved contact graph')
   if len(wrist_force_n)!=2 or len(package_force_n)!=2:raise ValueError('Per-arm force measurements required')
   for i in range(2):self.wrist[i].add(float(wrist_force_n[i]),dt);self.package[i].add(float(package_force_n[i]),dt)
   for box in 'AB':
    state(boxes[box]);s=shock[box]
    for k in ['mean_abs_g','norm_mean_g','peak_abs_g','peak_vector_g']:
     if not isinstance(s[k],(int,float)) or isinstance(s[k],bool) or not math.isfinite(s[k]) or s[k]<0:raise ValueError('Nonfinite shock measurement')
    if type(s['valid']) is not bool:raise ValueError('Missing shock validity')
    self.peak_any[box]=max(self.peak_any[box],s['peak_abs_g'],s['peak_vector_g'])
   geoms={n:{i:geometry(boxes[n],cartons[i],self.rest_z[i]) for i in range(1,5)} for n in 'AB'}
   self.samples+=1;self.seconds+=dt;self.sensor_fresh=all(shock[n]['valid'] for n in 'AB')
   self.last_geometry=geoms;self.last_touched=touched
   if type(source_contact_veto) is not bool:raise ValueError('Missing source interference state')
   self.source_contact_veto=self.source_contact_veto or source_contact_veto
   self.source_drops=list(source_drops);self.source_open=list(source_open)
   for i in range(1,5):
    c=state(cartons[i]['state']);h=self.trackers[i];rise=float(c[2]-self.rest_z[i])
    if h['phase']=='rest':
     if rise>.02:
      payload=[n for n in 'AB' if geoms[n][i]['inside']]
      h.update(phase='up',carton=i,boxes=payload,rise_m=rise,peak_g={n:0. for n in payload},robot_contact=False,released=False,completed=False,t_lift=self.seconds)
     continue
    h['rise_m']=max(h['rise_m'],rise)
    if touched&({f'carton{i}'}|set(h['boxes'])):h['robot_contact']=True
    if h['rise_m']>=.25 and h['robot_contact']:self.independent_contact_veto=True
    for n in h['boxes']:h['peak_g'][n]=max(h['peak_g'][n],shock[n]['norm_mean_g'])
    if h['rise_m']>=.25 and c[9]<-.05:h['released']=True
    if h['phase']=='up' and rise<.005:
     if h['rise_m']>=.25:h.update(phase='window',t_land=self.seconds)
     else:self.trackers[i]={'phase':'rest'}
    elif h['phase']=='window' and self.seconds-h['t_land']>=2.:
     h.update(completed=True,postlanding_s=self.seconds-h['t_land'])
     self.drops.append({k:v for k,v in h.items() if k!='phase'});self.trackers[i]={'phase':'rest'}
   for n in self.required:
    c=next((i for i in range(1,5) if geoms[n][i]['packed'] and not touched&{n,f'carton{i}'}),None);old=self.final[n];p=state(boxes[n])[:3]
    if c is None or touched:
     self.final[n]={'carton':None,'hold_s':0.,'packed_now':False,'hands_off':not bool(touched)};self.hold_ref[n]=None
    elif old['carton']!=c or self.hold_ref[n] is None or np.linalg.norm(p-self.hold_ref[n])>.002:
     self.final[n]={'carton':c,'hold_s':0.,'packed_now':True,'hands_off':True};self.hold_ref[n]=p.copy()
    else:old.update(hold_s=old['hold_s']+dt,packed_now=True,hands_off=True)
  except (KeyError,TypeError,ValueError,OverflowError,IndexError) as e:
   self.invalid_reason=type(e).__name__+': '+str(e);raise ValueError(self.invalid_reason) from e
 def packet(self,controls):
  open_records=list(self.source_open)+[{k:v for k,v in h.items() if k!='phase'} for h in self.trackers.values() if h['phase']!='rest']
  return {'required':list(self.required),'valid':self.invalid_reason is None,'fresh':self.sensor_fresh,'samples':self.samples,'verified_channels':dict(self.validated),'controls':controls,'physics_s':self.seconds,'peak_any_g':dict(self.peak_any),'diverged':self.diverged,'damage_veto':False,'wrist_veto':any(x.tripped for x in self.wrist),'crush_veto':any(x.tripped for x in self.package),'drop_contact_veto':self.source_contact_veto or self.independent_contact_veto,'source_drops':self.source_drops,'independent_drops':list(self.drops),'open_drop_records':open_records,'final':self.final}
 def score(self,controls):return evaluate(self.packet(controls))
