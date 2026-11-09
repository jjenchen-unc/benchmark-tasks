"""Untared continuous force exposure; actual calibrated force inputs are still required."""
import math
class DurationGuard:
 def __init__(self,threshold_n=80.,duration_s=.1):
  self.threshold=threshold_n;self.duration=duration_s;self.exposure=0.;self.peak=0.;self.tripped=False;self.invalid=False
 def add(self,magnitude_n,dt):
  if self.invalid:raise ValueError('Force channel already invalid')
  if not math.isfinite(magnitude_n) or magnitude_n<0 or not math.isfinite(dt) or not 0<dt<=.002:
   self.invalid=True;raise ValueError('Missing/nonfinite/unresolved force sample')
  self.peak=max(self.peak,float(magnitude_n));self.exposure=self.exposure+dt if magnitude_n>self.threshold else 0.
  self.tripped=self.tripped or self.exposure>=self.duration-1e-12
  return self.tripped
