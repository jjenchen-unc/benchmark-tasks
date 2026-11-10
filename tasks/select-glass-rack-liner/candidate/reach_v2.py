"""Complete nominal reference endpoint/leg IK, actual SDK; no physics."""
import json,sys,hashlib
from pathlib import Path
import numpy as np
from c1_contract import *
from yam_arena import ik,YAM_URDF
from yam_arena.constants import READY
from yam_arena.demo import grip_rotation
from scipy.optimize import least_squares

def main(out):
 rng=np.random.default_rng(101);rows=[];targets=[]
 for i,p in enumerate(LINER_START):
  for label,z in [('hover',TABLE_Z+.22),('pinch',TABLE_Z+.015)]:targets.append((f'liner{i+1}_{label}',np.r_[p[:2],z],np.pi))
 for i,p in enumerate(CUP_START):
  targets.extend([(f'cup{i+1}_hover',np.r_[p[:2],TABLE_Z+.22],np.pi),(f'cup{i+1}_grasp',p+[0,0,.028],np.pi)])
 for slot in [0,1]:
  s=slot_point(slot);lt=np.r_[s[:2],s[2]+.075*np.tan(ANGLE)+.018];ct=np.r_[s[:2],(s+N_RACK*.006)[2]+.037*np.tan(ANGLE)+.0645]
  targets.extend([(f'slot{slot}_hover',np.r_[s[:2],TABLE_Z+.22],0.),(f'slot{slot}_liner_lower',lt,0.),(f'slot{slot}_liner_retreat',lt+[0,0,.08],0.),(f'slot{slot}_cup_lower',ct,0.),(f'slot{slot}_cup_clear',ct+[0,0,.02],0.),(f'slot{slot}_cup_retreat',ct+[-.10,-.08,.06],0.)])
 targets.extend([('pan_hover',np.r_[PAN_C[:2],TABLE_Z+.22],np.pi),('pan_dip',np.r_[PAN_C[:2],TABLE_Z+.024],np.pi),('park_left',PARK_POINTS[0],0.),('park_right',PARK_POINTS[1],0.)])
 for name,p,yaw in targets:
  arms=[];R=grip_rotation(yaw)
  for side in ['left','right']:
   a=ROBOT_YAW[side];B=np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1.]]);base=np.array(ROBOT_BASES[side]);best=None
   def fun(q):
    pp,RR=ik.tcp_pose(q);return np.r_[base+B@pp-p,.25*ik._rot_error(R,B@RR)]
   for seed in [np.array(READY)]+[rng.uniform(ik.LOWER,ik.UPPER) for _ in range(8)]:
    fit=least_squares(fun,np.clip(seed,ik.LOWER+1e-8,ik.UPPER-1e-8),bounds=(ik.LOWER,ik.UPPER),max_nfev=160);pp,RR=ik.tcp_pose(fit.x);pe=float(np.linalg.norm(base+B@pp-p));re=float(np.linalg.norm(ik._rot_error(R,B@RR)))
    if best is None or pe+.25*re<best[0]:best=(pe+.25*re,pe,re,fit.x)
    if pe<.0003 and re<.005:break
   arms.append({'arm':side,'position_error_m':best[1],'rotation_error_rad':best[2],'q':best[3].tolist(),'joint_margin_rad':float(np.min(np.r_[best[3]-ik.LOWER,ik.UPPER-best[3]])),'reachable':best[1]<=.003 and best[2]<=.02})
  rows.append({'target':name,'position_m':p.tolist(),'downward_yaw_rad':yaw,'arms':arms})
 r={'kind':'PURE_ACTUAL_SDK_COMPLETE_REFERENCE_ENDPOINTS_NO_PHYSICS','mounts':ROBOT_BASES,'mount_yaw':ROBOT_YAW,'joint_lower':ik.LOWER.tolist(),'joint_upper':ik.UPPER.tolist(),'sdk_urdf_sha256':hashlib.sha256(Path(YAM_URDF).read_bytes()).hexdigest(),'rows':rows,'all_points_some_arm':all(any(a['reachable'] for a in x['arms']) for x in rows),'remaining':'Endpoint IK does not prove intervening orientation transitions, collision-free sweeps, measured mount/FK agreement or force-limited execution.'};Path(out).write_text(json.dumps(r,indent=2)+'\n');print('reachable endpoints',r['all_points_some_arm']);print([(x['target'],[(a['arm'],round(a['position_error_m'],4),round(a['rotation_error_rad'],3)) for a in x['arms']]) for x in rows if not any(a['reachable'] for a in x['arms'])])
if __name__=='__main__':main(sys.argv[1])
