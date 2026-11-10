"""Actual SDK IK nominal point screen; no physics, meshes or SDK mutation."""
import json,sys,math
from pathlib import Path
import numpy as np
from yam_arena import ik
from yam_arena.demo import grip_rotation
F=json.loads((Path(__file__).parent/'fixture.json').read_text())
# Keep this pure: reference module imports Inspect Action only in runtime environment.
points={'paddle_front':[.15,-.01,1.1975],'paddle_side':[.36,-.01,1.1975],'paddle':[.36,-.08,1.1975],'paddle_pull':[.36,-.045,1.1975],'box_front':[.16,-.378,.788],'box_tab':[.265,-.378,.788],'box_out':[.13,-.378,.796],'box_setdown':[.05,-.10,.7795]}
for deg in [0,12,25,50,75,100]:
 a=math.radians(deg);points['door_'+str(deg)]=[.244-math.cos(a)*.028-math.sin(a)*.136,-.468-math.sin(a)*.028+math.cos(a)*.136,1.2025]
rows=[]
for name,pos in points.items():
 yaw=math.pi/2 if name.startswith('paddle') else math.radians(int(name.split('_')[1])) if name.startswith('door_') else 0.
 if name.startswith('door_') and int(name.split('_')[1])>=75:yaw-=math.pi
 for base in [F['robot_bases']['right'],[0,-.305,.75]]:
  q,e,r=ik.solve(pos,ik.matrix_to_quat_xyzw(grip_rotation(yaw)),base_pos=base,iters=200)
  rows.append(dict(point=name,target=pos,base=base,yaw=yaw,q=q.tolist(),position_error_m=e,angle_error_rad=r,pass_2mm_03rad=bool(e<.002 and r<.03)))
print(json.dumps(dict(backend='actual SDK yam_arena.ik.solve, static kinematic screen only',rows=rows,clearance_verified=False,torque_verified=False),indent=2))
