"""Observable support scoring. No hidden fold-angle or magnet-state success gate."""
from collections import deque
import itertools,math
import numpy as np
from scipy.spatial.transform import Rotation
from mechanism import TABLET_SIZE
TOP=.7475
CORNERS=np.array(list(itertools.product(*[[-h,h] for h in TABLET_SIZE/2])))
CONFIG=dict(table_height_m=TOP,table_bounds_xy_m=[[-.19,.91],[-.65,.65]],tablet_size_m=TABLET_SIZE.tolist(),screen_normal_local=[0,0,1],portrait_up_local=[0,1,0],user_axis_world=[-1,0,0],full_tilt_deg=[50.,70.],partial_tilt_deg=[25.,50.],portrait_roll_max_deg=15.,facing_error_max_deg=30.,hold_s=5.,max_position_span_m=.005,max_rotation_span_deg=3.,hand_clearance_min_m=.03)
def geometry(pos,quat):
 r=Rotation.from_quat(quat).as_matrix();n=r[:,2];u=r[:,1]
 tilt=math.degrees(math.acos(float(np.clip(n[2],-1,1))))
 vertical=np.array([0.,0.,1.])-n*n[2];vertical/=max(np.linalg.norm(vertical),1e-12)
 roll=math.degrees(math.acos(float(np.clip(u@vertical,-1,1))))
 h=n.copy();h[2]=0;h/=max(np.linalg.norm(h),1e-12)
 facing=math.degrees(math.acos(float(np.clip(h@np.array([-1.,0.,0.]),-1,1))))
 corners=CORNERS@r.T+pos
 return dict(tilt_deg=tilt,portrait_roll_deg=roll,facing_error_deg=facing,lowest_corner_m=float(corners[:,2].min()-TOP))
class Scorer:
 def __init__(self):self.reset()
 def reset(self):self.steps=0;self.score=0.;self.full_s=0.;self.partial_s=0.;self.history=deque(maxlen=100)
 def update(self,m,dt=.05):
  self.steps+=1
  support=(m['tablet_table_contact'] and m['stand_table_contact'] and not m['robot_contact'] and not m['illegal_support_contact'] and m['hand_clearance_m']>=.03 and m['all_body_on_table'] and m['portrait_roll_deg']<=15 and m['facing_error_deg']<=30)
  self.history.append((np.asarray(m['tablet_pos']),np.asarray(m['tablet_quat'])))
  stable=True
  if len(self.history)>1:
   pp=np.array([v[0] for v in self.history]);qq=Rotation.from_quat(np.array([v[1] for v in self.history]));rel=qq[0].inv()*qq
   stable=bool(np.max(np.linalg.norm(pp-pp[0],axis=1))<=.005 and np.max(np.degrees(rel.magnitude()))<=3.)
  full=support and 50<=m['tilt_deg']<=70
  partial=support and 25<=m['tilt_deg']<50
  self.full_s=self.full_s+dt if full else 0.;self.partial_s=self.partial_s+dt if partial else 0.
  self.score=1. if self.full_s>=5.-1e-8 and stable else .5 if self.partial_s>=5.-1e-8 and stable else 0.
  return self.score
