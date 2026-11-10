"""Passive conditional hydraulic law, adapted from immutable source; never policy code.
Oil target follows measured physical pump/valve motion and pad forces only.
"""
import math
class Hydraulic:
 def __init__(self,p,tag):
  self.p,self.tag=p,tag;self.V=.055 if tag=='S' else 0.;self.low=.3;self.pumped=self.lowered=0.
 def update(self,theta,omega,phi,force,dt):
  if not all(map(math.isfinite,(theta,omega,phi,force,dt))) or dt<=0:raise ValueError('invalid hydraulic sample')
  closed=math.degrees(phi)<=self.p['phi0_'+self.tag];stroke=self.p['stroke_'+self.tag]
  if theta<self.low:
   if closed and self.V<stroke-1e-5:
    dv=min(.023*(self.low-theta),stroke-self.V);self.V+=dv;self.pumped+=dv
   self.low=theta
  elif theta>self.low+.005:self.low=theta-.005
  opening=min(90.,max(0.,math.degrees(phi)-self.p['phi0_'+self.tag]))
  v=self.p['v30_'+self.tag]/30.*opening*math.sqrt(max(force,0.)/50.)
  if opening>0 and force<2:v=max(v,30.)
  dv=min(v*.001*dt,self.V);self.V-=dv
  if force>=2:self.lowered+=dv
  resistance=(force*.023+.15) if closed and self.V<stroke-1e-5 else .15
  # Smooth Coulomb approximation around the source's 0.004 rad/s stall band.
  # No powered lever target: only its source return spring and resisting torque.
  effort=-(resistance if omega<=0 else .045)*max(-1.,min(1.,omega/.004))
  return self.V,effort,closed
