"""Read-only state-aware robot planner; no task state or collision writes."""
import numpy as np
from drawers_math import arr,native,A

def measured_paddles(emb):
 asset=emb._base.scene['drawer_U']
 values=dict(zip(asset.joint_names,arr(asset.data.joint_pos)[0]))
 q=np.array([values['drawer_U_paddle_'+s+'_j'] for s in ('left','right')],float)
 if q.shape!=(2,) or not np.isfinite(q).all():raise ValueError('invalid measured paddle joints')
 return q

def native_stop_passed(p,R,engine):
 # Exact native disarm geometry, plus actual engine state. Read only.
 p=np.asarray(p,float);R=np.asarray(R,float)
 if p.shape!=(3,) or R.shape!=(3,3) or not np.isfinite(p).all() or not np.isfinite(R).all():raise ValueError('invalid measured drawer frame')
 bx=float(native(p)[0]+(A@R)[0,0]*.13)
 return bx+.003 < .12+.109-.001 and not engine.armed['slot_U'] and not engine.stop_on['slot_U']

class ReleasePlanner:
 def __init__(self):self.z=np.zeros(2);self.x=0.;self.stable=0
 def released(self,q):
  q=np.asarray(q,float)
  if q.shape!=(2,) or not np.isfinite(q).all():raise ValueError('invalid paddle feedback')
  self.stable=self.stable+1 if np.all(np.abs(q)>.004) else 0
  return self.stable>=3
 def press(self,q):
  q=np.asarray(q,float)
  if q.shape!=(2,) or not np.isfinite(q).all():raise ValueError('invalid paddle feedback')
  self.z=np.clip(self.z+np.clip(.005-q,-.00025,.00025),0.,.016)
  return self.z.copy()
 def pull(self,q,passed):
  z=self.press(q)
  # Loss of either native release pauses outward command before disarm.
  if passed or np.all(np.abs(np.asarray(q))>.004):self.x+=.0005
  if self.x>.09500001:raise RuntimeError('retained pull command bound95mm exceeded')
  return self.x,z
