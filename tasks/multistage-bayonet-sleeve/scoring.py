"""Externally measurable numeric scorer. Passive history never drives the scene.
Native final predicate is unchanged. 0.5 is added for the same resting/clear,
hands-off state before the full uninterrupted20s; source has binary final score.
"""
import math
import numpy as np
from source_geometry import N
DT=.002
CONTROL_LIMIT=6000
REQUIRED=frozenset({'pose_velocity','sleeve_robot_contact','sleeve_post_contact','tray_floor_contact','wrist_force','motor_limits','substeps','release_track','plunger_spring'})
class History:
 def __init__(self):
  self.hold=0.;self.over=[0.,0.];self.failure=None;self.samples=0;self.eligible=False;self.peak_wrist=[0.,0.];self.last=None;self.max_hold=0.
 def update(self,row,dt):
  if not math.isclose(dt,DT,abs_tol=1e-9):self.failure='physics_dt_mismatch';return
  self.samples+=1;self.last=row
  numbers=np.r_[row['pos'],np.asarray(row['R']).ravel(),row['velocity'],row['wrist_force']]
  if not np.isfinite(numbers).all():self.failure='nonfinite_measurement';return
  for i,f in enumerate(row['wrist_force']):
   self.peak_wrist[i]=max(self.peak_wrist[i],f);self.over[i]=self.over[i]+dt if f>80 else 0.
   if self.over[i]>=.1-1e-10:self.failure='wrist_force_abort'
  # Transform only the global scene translation back to the exact native scorer.
  inside=N['sleeve_inside_tray'](np.asarray(row['pos'])-row['translation'],np.asarray(row['R']))
  slow=np.linalg.norm(row['velocity'][:3])<.02 and np.linalg.norm(row['velocity'][3:])<.10
  self.eligible=bool(inside and row['floor_contact'] and not row['post_contact'] and not row['robot_contact'] and slow and not self.failure)
  self.hold=self.hold+dt if self.eligible else 0.;self.max_hold=max(self.max_hold,self.hold)
 def result(self,channels=frozenset()):
  verified=REQUIRED<=channels;candidate=(1. if self.hold>=20-1e-9 else .5 if self.eligible else 0.) if not self.failure else 0.
  return dict(score=candidate if verified else 0.,diagnostic_score=candidate,source_binary_success=self.hold>=20-1e-9 and not self.failure,hold_s=self.hold,peak_hold_s=self.max_hold,eligible=self.eligible,failure=self.failure,physics_samples=self.samples,peak_wrist_force_N=self.peak_wrist,damage={},calibration_verified=verified,unverified_channels=sorted(REQUIRED-channels))
