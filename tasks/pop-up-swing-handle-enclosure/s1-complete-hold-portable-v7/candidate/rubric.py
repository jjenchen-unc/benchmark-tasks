"""Numeric native-goal history; actual backend channel verification gates credit."""
import math
D=math.pi/180
REQUIRED={'joint_state','wrist_force','robot_all_box_contacts','passive_mechanisms','clock','motor_total_limits'}
class History:
 def __init__(self,target):self.target=target;self.hold=0.;self.max_open=0.;self.over=0.;self.over_max=0.;self.failure=None;self.elapsed=0.;self.last=None
 def sample(self,row,dt):
  if not math.isfinite(dt) or dt<=0:raise ValueError('invalid physics dt')
  if not all(math.isfinite(float(x)) for x in [row['door'],row['swing'],row['pop'],*row['wrist_force_n']]):raise ValueError('nonfinite measurement')
  self.elapsed+=dt;self.last=row;self.max_open=max(self.max_open,row['door']);self.over=self.over+dt if max(row['wrist_force_n'])>80 else 0.;self.over_max=max(self.over_max,self.over)
  if self.over_max>=.1-1e-10:self.failure='wrist_overload'
  opened=self.max_open>60*D
  locked=row['engaged'] and row['swing']<=5.5*D and self.gap()<.001
  stowed=row['pop']<4*D and (row['order']!='A' or row['pop_catch'])
  goal=row['door']>60*D if self.target['goal']=='open' else opened and locked and stowed
  self.hold=self.hold+dt if goal and not row['robot_contact'] and not self.failure else 0.
 def gap(self):return .2*self.last['door']+.004
 def raw_score(self):
  if self.failure or not self.last:return 0.
  if self.hold>=20.-1e-9:return 1.
  r=self.last
  if self.target['goal']=='open':return .5 if r['door']>60*D else .25 if r['released'] and r['door']>10*D else 0.
  if self.max_open<=60*D:return 0.
  if r['engaged'] and self.gap()<.001 and r['swing']<=5.5*D:return .75
  return .5 if self.gap()<.010 else .25
 def status(self,verified=()):
  missing=sorted(REQUIRED-set(verified));raw=self.raw_score()
  return dict(score=0. if missing else raw,measured_candidate_score=raw,missing_verified_channels=missing,hold_s=self.hold,elapsed_s=self.elapsed,max_open_deg=self.max_open/D,wrist_over_s_max=self.over_max,failure=self.failure)
