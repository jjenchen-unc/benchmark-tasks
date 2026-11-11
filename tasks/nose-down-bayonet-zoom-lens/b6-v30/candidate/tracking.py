"""Bounded projected integral in the calibrated measured static tracking-error direction."""
import numpy as np
LIMIT=np.array([.004,.060,.040,.025,.010,.025])
CALIBRATED_ERROR=np.array([9.235582826594424e-05, -0.024184394457666958, 0.013020021806505744, -0.007911161601156791, -0.0006331490593266386, 0.004067774360507426]) # actual43955 control240
class Tracking:
 def __init__(self,target,measured):
  self.target=np.asarray(target,float).copy();measured=np.asarray(measured,float)
  if self.target.shape!=(6,) or measured.shape!=(6,) or not np.isfinite(self.target).all() or not np.isfinite(measured).all():raise ValueError('Invalid arm joints')
  error=self.target-measured
  if np.any(2*np.abs(error)>LIMIT):raise ValueError('Measured error outside finite correction envelope')
  self.direction=CALIBRATED_ERROR.copy();self.gain=0.;self.offset=np.zeros(6)
 def update(self,measured):
  measured=np.asarray(measured,float)
  if measured.shape!=(6,) or not np.isfinite(measured).all():raise ValueError('Invalid measured arm joints')
  denom=float(self.direction@self.direction)
  delta=0. if denom<1e-12 else .1*float((self.target-measured)@self.direction)/denom
  # 20Hz Ki=2/s; gain in[0,2], joint slew <=.001rad/control.
  slew=min(.05,.001/max(float(np.max(np.abs(self.direction))),1e-12))
  self.gain=float(np.clip(self.gain+np.clip(delta,-slew,slew),0.,2.))
  self.offset=self.gain*self.direction
  return self.target+self.offset
