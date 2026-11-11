"""Pure physical laws and numeric episode history. No policy state mutation."""
import math
import numpy as np
from numeric_score import validate_measurement,tier_now,score,SchemaError
H=.020/(math.pi/2)
REQUIRED=frozenset(['clock','robot_contacts','seat_push','mount_reaction','wrist_force','lock_capture','zoom_helix','reset','motor_limits'])
# Empty until root's actual known-load/positive/negative checks pass for these bytes.
VALIDATED=frozenset()
def rotation(q):
 x,y,z,w=q;return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def seat_measure(mount_pos,mount_R,lens_pos,lens_R):
 rel=mount_R.T@(lens_pos-mount_pos);r=mount_R.T@lens_R
 return dict(axial_gap_m=float(rel[2]),radial_m=float(np.hypot(*rel[:2])),tilt_deg=float(np.degrees(np.arccos(np.clip(r[2,2],-1,1)))),yaw_deg=float(np.degrees(math.atan2(r[1,0],r[0,0]))))
def detent(theta,d,lock_deg,click_torque,clicked):
 th=math.degrees(theta)*d
 return -d*click_torque*float(np.clip((th-(lock_deg-12))/12,.05,1)) if not clicked and 0<th<lock_deg else 0.
def helix(theta,x,omega,v):
 # U=K/2*(x-h*theta)^2. Paired generalized efforts conserve power;
 # damping removes energy. Finite stiffness is disclosed vs source equality.
 error=x-H*theta;speed=v-H*omega;force=-20000.*error-20.*speed
 return force,-H*force
class History:
 def __init__(self):
  self.t=0.;self.samples=0;self.failure=None;self.overload=False;self.dropped=False;self.invalid=False;self.hands=0.;self.full=0.;self.locked=0.;self.wrist=[0.,0.];self.over={};self.measurement={'lens_on_mount':None};self.peak_wrist=[0.,0.]
 def update(self,row,dt):
  if not math.isfinite(dt) or dt<=0 or dt>.01:raise ValueError('invalid physics dt')
  self.t=round(self.t+dt,12);self.samples+=1
  try:
   forces=np.asarray(row['wrist_n'],float);torques=row['stop_nm'];vel=row['drop_velocity_z'];touch=row['robot_touch']
   if forces.shape!=(2,) or not np.isfinite(forces).all() or not isinstance(touch,bool):raise ValueError('invalid sensors')
   for i,f in enumerate(forces):
    self.peak_wrist[i]=max(self.peak_wrist[i],float(f));self.wrist[i]=self.wrist[i]+dt if f>80 else 0.
    if self.wrist[i]>=.1-1e-9:self.failure='wrist_force_80N_100ms'
   for n,tau in torques.items():
    if not math.isfinite(tau):raise ValueError('nonfinite reaction')
    self.over[n]=self.over.get(n,0.)+dt if abs(tau)>1.5 else 0.
    if self.over[n]>=.05-1e-9:self.overload=True
   for n,vz in vel.items():
    if not math.isfinite(vz):raise ValueError('nonfinite velocity')
    if vz<-.6 and not row['held'][n] and not row['captured'][n]:self.dropped=True
   self.measurement=validate_measurement(row['measurement']);s,_,_=tier_now(self.measurement)
   self.hands=round(self.hands+dt,12) if not touch else 0.
   self.locked=round(self.locked+dt,12) if s>=.5 and not touch else 0.
   self.full=round(self.full+dt,12) if s==1 and not touch else 0.
  except (ValueError,KeyError,TypeError,SchemaError) as e:
   self.invalid=True;self.failure='measurement_invalid: '+str(e);self.full=self.locked=self.hands=0.
  if self.overload:self.failure='mount_overload'
  if self.dropped:self.failure='lens_dropped'
 def result(self,channels=()):
  history=dict(overload=self.overload,dropped=self.dropped,invalid_measurement_seen=self.invalid,hands_off_end_s=self.hands,hold_full_s=self.full,hold_locked_s=self.locked)
  raw=score(self.measurement,history)
  if self.failure:raw=dict(score=0.,tier='veto',reasons=[self.failure])
  missing=sorted(REQUIRED-set(channels))
  return dict(score=0. if missing else raw['score'],measured_rubric=raw,missing_runtime_validation=missing,history=history,sim_elapsed_s=self.t,samples=self.samples,failure=self.failure,peak_wrist_n=self.peak_wrist)
