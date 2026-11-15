"""Estimated two-panel portrait topology; not a calibrated Qek W90 replica."""
import math
import numpy as np
from scipy.spatial.transform import Rotation
TABLET_SIZE=np.array([.180,.240,.007]); TABLET_MASS=.48
PANEL_SIZE=np.array([.134,.150,.0013]); PANEL_MASS=.035
ROOT=np.array([0,.030,-.006]); HINGE2_OFFSET_Z=.0016
SECOND_OFFSET=np.array([0,-.075,HINGE2_OFFSET_Z])
PANEL2_HINGE_OFFSET=np.array([0,-.150,HINGE2_OFFSET_Z])
SIGMA=.006; ENERGY=1.5*SIGMA*math.exp(.5)

def initial_poses(pos,quat,q1=0.,q2=-math.pi):
 r=Rotation.from_quat(quat).as_matrix()
 r1=Rotation.from_rotvec([q1,0,0]).as_matrix();r2=Rotation.from_rotvec([q1+q2,0,0]).as_matrix()
 p1=ROOT+r1@np.array([0,-.075,0]);p2=ROOT+r1@PANEL2_HINGE_OFFSET+r2@np.array([0,-.075,0])
 return {'tablet':(np.array(pos),np.array(quat)), 'panel1':(pos+r@p1,Rotation.from_matrix(r@r1).as_quat()),
         'panel2':(pos+r@p2,Rotation.from_matrix(r@r2).as_quat())}

def folded_rest_height():
 # Exact authored geometry touches the table; no invented initial spacer/gap.
 poses=initial_poses(np.zeros(3),[0.,0.,0.,1.])
 low=[]
 for n,(p,q) in poses.items():
  r=Rotation.from_quat(q).as_matrix();half=(TABLET_SIZE if n=='tablet' else PANEL_SIZE)/2
  low.append(p[2]-np.abs(r[2])@half)
 return -float(min(low))

def wrenches(poses,scale=1.):
 forces={n:np.zeros(3) for n in poses};torques={n:np.zeros(3) for n in poses}
 for x in [-.045,.045]:
  pi,ri=poses['tablet'];pp,rp=poses['panel2']
  a=pp+rp@np.array([x,-.075,0]);b=pi+ri@np.array([x,-.120,-.006])
  delta=b-a;f=scale*ENERGY/SIGMA**2*math.exp(-float(delta@delta)/(2*SIGMA**2))*delta
  forces['panel2']+=f;torques['panel2']+=np.cross(a-pp,f)
  forces['tablet']-=f;torques['tablet']+=np.cross(b-pi,-f)
 return forces,torques
