"""Pure SDK FK/IK planning; imports no Kit or physics. No scene writes."""
import json, math
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from yam_arena import ik
BASE=np.array([0.,.305,.90])
SHIFT=np.array([.23,.14,-.0525])
def orientation(yaw):
 c,s=np.cos(yaw),np.sin(yaw)
 return np.array([[c,s,0],[s,-c,0],[0,0,-1.]])
def solve(p,R,q0=None):
 def err(q):
  a,b=ik.tcp_pose(q,BASE);return np.r_[a-p,.25*ik._rot_error(R,b)]
 best=None
 for q in [q0 if q0 is not None else np.array([0,1,1,-.6,0,0])]+list(np.random.default_rng(42).uniform(ik.LOWER,ik.UPPER,(5,6))):
  fit=least_squares(err,np.clip(q,ik.LOWER+1e-8,ik.UPPER-1e-8),bounds=(ik.LOWER,ik.UPPER),max_nfev=100)
  e=err(fit.x);val=np.linalg.norm(e)
  if best is None or val<best[0]:best=(val,fit.x,e)
  if np.linalg.norm(e[:3])<.0002 and np.linalg.norm(e[3:])<.0005:break
 return best[1],float(np.linalg.norm(best[2][:3])),float(np.linalg.norm(best[2][3:])/.25)
def waypoints():
 # Source seat mouth z=.032 above stand, nominal channel centre 2mm lower.
 seat=np.array([.05,-.12,.816+.032-.002])+SHIFT;out=[]
 for name,x,t in [('approach',.12,0),('grasp',.002,0),('push',-.012,0),('turn1',-.012,30),('pull1',.024,30),('turn2',.024,-30),('out',.10,-30)]:
  out.append((name,seat+[0,0,.052+x],-math.pi/2-math.radians(t)))
 tray=np.array([.05,.26,.812])+SHIFT
 out.extend([('carry_up',seat+[0,0,.202],-math.pi/2+math.pi/6),('tray_above',tray+[0,0,.202],-math.pi/2),('tray_place',tray+[0,0,.053],-math.pi/2),('retreat',tray+[0,0,.24],-math.pi/2)])
 return out
if __name__=='__main__':
 rows=[];q=None
 for name,p,y in waypoints():
  q,ep,er=solve(p,orientation(y),q);rows.append(dict(name=name,target=p.tolist(),yaw=y,q=q.tolist(),position_error_m=ep,rotation_error_rad=er,reachable=ep<.001 and er<.01))
 print(json.dumps(dict(kind='pure actual SDK IK endpoints only; not collision/dynamic proof',base=BASE.tolist(),translation=SHIFT.tolist(),passed=all(r['reachable'] for r in rows),checks=rows),indent=2))
