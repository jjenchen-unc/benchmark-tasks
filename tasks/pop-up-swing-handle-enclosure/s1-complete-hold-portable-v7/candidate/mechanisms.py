"""Passive environment constitutive mechanism; robot policies never write here.
Finite compliant approximation of source equality cams/catches. Needs physics calibration.
All force pairs use conjugate coordinates, including swing reaction to cam load.
"""
import math
D=math.pi/180
class Mechanism:
 def __init__(self,spec):
  self.spec=spec;self.engaged=True;self.thrown_open=False;self.released=False;self.pop_catch=spec['order']=='A';self.swing_lock=True;self.pop_anchor=0.;self.swing_anchor=0.
 def cam(self,th):
  if th>self.spec['angle']-10*D:return None
  if th<=5*D:return -.02,0.
  if th<=self.spec['comp']:
   slope=.02/(self.spec['comp']-5*D);return -.02-slope*5*D,slope
  return 0.,0.
 def force(self,q,v):
  door,th,ph=q['door'],q['swing'],q['pop'];cap=q.get('cap',0.);push=q.get('push',0.)
  if self.pop_catch and cap>.0024:self.pop_catch=False
  elif not self.pop_catch and self.spec['order']=='A' and ph<1.5*D and cap<.001:self.pop_catch=True;self.pop_anchor=ph
  lifted=ph>22*D and (self.spec['order']!='C' or push>.0035)
  if self.swing_lock and lifted:self.swing_lock=False
  elif not self.swing_lock and th<2*D and ph<4*D:self.swing_lock=True;self.swing_anchor=th
  seg=self.cam(th)
  if self.engaged:
   if seg is None:self.engaged=False;self.released=True
  else:
   if seg is not None and not self.thrown_open:
    if door<=seg[0]+seg[1]*th+.0075:self.engaged=True
    else:self.thrown_open=True
   elif seg is None and self.thrown_open:self.thrown_open=False
  f={n:0. for n in q}
  # Gasket tendon is extension-only at negative lid angle; spring torque in rad.
  f['door']+=self.spec['gasket']*.2*.2*max(-door,0.)
  if self.pop_catch:f['pop']+=-200.*(ph-self.pop_anchor)-1.*v['pop']
  if self.swing_lock:f['swing']+=-2000.*(th-self.swing_anchor)-5.*v['swing']
  if self.engaged and seg is not None:
   a,b=seg;load=-10000.*(door-a-b*th)-20.*(v['door']-b*v['swing']);f['door']+=load;f['swing']-=b*load
  if self.thrown_open and door<.015:f['door']+=10000.*(.015-door)-20.*min(v['door'],0.)
  return f
