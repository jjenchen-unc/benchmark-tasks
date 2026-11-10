"""Numeric physics-history scorer; no robot or scene writes. Credit defaults gated off."""
import math
REQUIRED=frozenset(('clock','pad_force','rail_contact','robot_load_contact','glass_force','wrist_force','load_pose','drive_limits','hydraulics'))
class History:
 def __init__(self):
  self.t=0.;self.samples=0;self.hold=0.;self.failure=None;self.prev=None;self.landed=False;self.jack_landing=False;self.landing_v=None;self.max_descent=0.;self.peak_glass=0.;self.wrist_high=0.;self.last=None
 def update(self,r,dt):
  self.samples+=1;self.t+=dt;self.last=r
  vals=[dt,r['height'],r['vz'],r['tilt'],r['glass_force'],*r['wrist_force'],r['flow_lowered'],r['speed'],r['spin']]
  if not all(map(math.isfinite,vals)) or not 0<dt<=.01:self.failure='nonfinite_or_clock';return
  self.peak_glass=max(self.peak_glass,r['glass_force'])
  if self.peak_glass>15:self.failure='glass_damage'
  if r['tilt']>math.radians(10):self.failure='cabinet_tipped'
  if r['height']<-.04:self.failure='load_drop'
  self.wrist_high=self.wrist_high+dt if max(r['wrist_force'])>80 else 0.
  if self.wrist_high>=.1-1e-9:self.failure='wrist_abort'
  # Default 30N soft stop is terminal for this controller, so no retry overrides.
  if max(r['wrist_force'])>30 and self.failure is None:self.failure='wrist_guard'
  if self.prev and r['on_rails'] and not self.prev['on_rails']:
   self.landing_v=max(0.,-self.prev['vz']);self.landed=True
   self.jack_landing=bool(self.prev['on_pad'] and not self.prev['robot_contact'] and not r['robot_contact'] and r['flow_lowered']>=.045)
   if self.landing_v>.025+1e-9:self.failure='hard_landing'
  if r['on_pad'] and not r['robot_contact'] and not r['on_rails']:
   # Oil flow and actual carried descent both required; direct hand lowering cannot earn partial credit.
   self.max_descent=max(self.max_descent,min(max(0.,.05-r['height']),r['flow_lowered']))
  final=r['on_rails'] and self.jack_landing and self.landing_v is not None and self.landing_v<=.025 and not r['robot_contact']
  self.hold=self.hold+dt if final and r['speed']<=.001 and r['spin']<=math.radians(1.) and self.failure is None else 0.
  self.prev=dict(r)
 def result(self,verified=frozenset()):
  raw=0.
  if self.failure is None:
   if self.max_descent>=.01:raw=.25
   if self.max_descent>=.03:raw=.5
   if self.jack_landing and self.last and self.last['on_rails'] and not self.last['robot_contact']:raw=max(raw,.75)
   if self.hold>=3.-1e-9:raw=1.
  missing=sorted(REQUIRED-set(verified))
  return dict(score=raw if not missing else 0.,diagnostic_candidate_score=raw,gate='unvalidated_channels' if missing else None,missing_channels=missing,failure=self.failure,hold_s=self.hold,landing_speed_m_s=self.landing_v,max_jack_descent_m=self.max_descent,physics_s=self.t,physics_samples=self.samples,peak_glass_N=self.peak_glass)
