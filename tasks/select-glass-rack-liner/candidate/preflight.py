"""Pure kinematic/source preflight; no physics or simulator imports.
Run in root's CPU allocation with the existing pinned YAM package.
"""
import argparse,json,sys
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
from c1_contract import *
def main():
 from yam_arena import ik
 from yam_arena.station import BASES
 from yam_arena.constants import READY,PAD_TIP_PAST_TCP
 from yam_arena.demo import grip_rotation
 from scipy.optimize import least_squares
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();R=grip_rotation(0.);rng=np.random.default_rng(42);rows=[]
 targets=[]
 for i,p in enumerate(LINER_START):targets.append((f'liner{i+1}_pinch',np.r_[p[:2],TABLE_Z+.002+PAD_TIP_PAST_TCP]))
 for i,p in enumerate(CUP_START):targets.append((f'cup{i+1}_grasp',p+[0,0,.040-.012]))
 for slot in [0,1]:
  s=slot_point(slot);targets.extend([(f'slot{slot}_hover',np.r_[s[:2],TABLE_Z+.22]),(f'slot{slot}_liner_release',np.r_[s[:2],s[2]+.075*np.tan(ANGLE)+.003+.002+PAD_TIP_PAST_TCP]),(f'slot{slot}_glass_release',np.r_[s[:2],(s+N_RACK*.006)[2]+.037*np.tan(ANGLE)+.0015+.035+.040-.012])])
 targets.append(('dip',np.r_[PAN_C[:2],TABLE_Z+.004+.007+PAD_TIP_PAST_TCP]))
 for name,p in targets:
  arms=[]
  for side in ['left','right']:
   base=np.asarray(BASES[side]);best=None
   def fun(q):
    pp,RR=ik.tcp_pose(q,base);return np.r_[pp-p,.25*ik._rot_error(R,RR)]
   for seed in [np.array(READY)]+[rng.uniform(ik.LOWER,ik.UPPER) for _ in range(5)]:
    fit=least_squares(fun,np.clip(seed,ik.LOWER+1e-8,ik.UPPER-1e-8),bounds=(ik.LOWER,ik.UPPER),max_nfev=180);pp,RR=ik.tcp_pose(fit.x,base);pe=float(np.linalg.norm(pp-p));re=float(np.linalg.norm(ik._rot_error(R,RR)));v=pe+.25*re
    if best is None or v<best[0]:best=(v,fit.x,pe,re)
    if pe<.0003 and re<.005:break
   arms.append({'arm':side,'q':best[1].tolist(),'position_error_m':best[2],'rotation_error_rad':best[3],'reachable':best[2]<=.003 and best[3]<=.02})
  rows.append({'target':name,'position_m':p.tolist(),'arms':arms,'at_least_one_reachable':any(x['reachable'] for x in arms)})
 report={'kind':'PURE_URDF_IK_NO_PHYSICS_NOT_ROBOT_EVIDENCE','rows':rows,'all_points_reachable_by_some_arm':all(x['at_least_one_reachable'] for x in rows),'note':'Individual reachability does not prove continuous collision-safe paths or under-load tracking; root actual validation remains necessary.'};a.out.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({'all_points_reachable_by_some_arm':report['all_points_reachable_by_some_arm']}))
if __name__=='__main__':main()
