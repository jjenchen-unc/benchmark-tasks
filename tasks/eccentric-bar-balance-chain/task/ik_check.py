"""Pure actual SDK URDF path preflight, no simulation or collision proof."""
import json,numpy as np
from yam_arena import ik
from yam_arena.constants import READY
from yam_arena.demo import grip_rotation
from reference import planned_waypoints
from fixture import ROBOT_BASES
q=np.array(READY);current=None;records=[]
for label,goal,jaw,n in planned_waypoints():
 start=goal if current is None else current.copy()
 for i in range(1,7):
  p=start+(goal-start)*i/6
  q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(grip_rotation(0)),q,base_pos=ROBOT_BASES['right'],iters=250)
  records.append({'label':label,'sample':i,'pos':p.tolist(),'q':q.tolist(),'position_error_m':pe,'rotation_error_rad':re,'pass':bool(pe<.003 and re<.03)})
 current=goal.copy()
print(json.dumps({'kind':'six samples per Cartesian segment, endpoints included; initial joint-space approach not swept collision checked','all_pass':all(r['pass'] for r in records),'records':records},indent=2))
if not all(r['pass'] for r in records):raise SystemExit(1)
