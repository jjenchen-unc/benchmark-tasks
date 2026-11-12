"""Pure actual SDK URDF IK; waypoint reach is not contact feasibility."""
import json,math,sys
from pathlib import Path
import numpy as np
from yam_arena import ik
from yam_arena.station import BASES
from yam_arena.constants import READY,PAD_TIP_PAST_TCP
HERE=Path(__file__).resolve().parent
R=np.array([[0,1,0],[-1,0,0],[0,0,1.]])
SHIFT=np.array([.28,0.,-.0525])
def world(p):return SHIFT+R@np.asarray(p)
def rx(t):return np.array([[1,0,0],[0,np.cos(t),-np.sin(t)],[0,np.sin(t),np.cos(t)]])
def run():
 rows=[]
 # Native representative's actual contact-centred positions. TCP backs off pad tip.
 for label,p,tilt in [('lid_initial_touch',[-.10,.21,1.19],0.),('lid_close',[-.10,.01,1.09],0.),('hook0',[-.165,-.06,1.065],math.pi/2),('hook1',[-.035,-.06,1.065],math.pi/2),('lever0',[-.165,-.12,1.00],math.pi/2),('lever1',[-.035,-.12,1.00],math.pi/2),('catch0',[-.131,-.05,.96],math.pi/2),('catch1',[-.001,-.05,.96],math.pi/2)]:
  rot=R@rx(tilt)@np.diag([1.,-1.,-1.]);pos=world(p)-PAD_TIP_PAST_TCP*rot[:,2]
  for side in ['left','right']:
   q,pe,re=ik.solve(pos,ik.matrix_to_quat_xyzw(rot),np.asarray(READY),base_pos=BASES[side],iters=250)
   rows.append(dict(label=label,side=side,pos=pos.tolist(),rot_xyzw=ik.matrix_to_quat_xyzw(rot).tolist(),q=q.tolist(),position_error_m=float(pe),rotation_error_rad=float(re),reachable=pe<.003 and re<.05))
 return {'kind':__doc__,'transform':{'rotation':R.tolist(),'translation':SHIFT.tolist()},'rows':rows}
if __name__=='__main__':
 data=run();Path(sys.argv[1]).write_text(json.dumps(data,indent=2)+'\n');print(json.dumps([dict(label=r['label'],side=r['side'],mm=round(1000*r['position_error_m'],2),deg=round(math.degrees(r['rotation_error_rad']),2)) for r in data['rows']]))
