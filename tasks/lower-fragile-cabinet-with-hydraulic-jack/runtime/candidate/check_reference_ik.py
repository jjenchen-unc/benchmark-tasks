"""Actual SDK IK for the representative approach and state-aware valve rotation."""
import json,math
from pathlib import Path
import numpy as np
from yam_arena import ik,station
from yam_arena.constants import READY
F=json.loads(Path(__file__).with_name('fixture.json').read_text());p=F['parameters'];target=math.radians(p['phi0_S']+10/((p['v30_S']/30)*math.sqrt(p['cabinet_weight']/50)));point=np.array([.26,.15,.875])+F['translation'];rows=[];q=READY
for phase,pos,yaw in [('approach',point+[0,0,.07],0),('pinch',point,0)]+[('open',point,y) for y in np.linspace(0,target,20)]+[('close',point,y) for y in np.linspace(target,0,20)]:
 c,s=math.cos(yaw),math.sin(yaw);R=np.array([[-c,-s,0],[-s,c,0],[0,0,-1]])
 q,e,r=ik.solve(pos,ik.matrix_to_quat_xyzw(R),q_init=q,base_pos=station.BASES['right'],iters=300)
 rows.append(dict(phase=phase,target_pos=pos.tolist(),yaw=yaw,q=q.tolist(),position_error_m=e,rotation_error_rad=r,pass_nominal=e<.003 and r<.03))
print(json.dumps({'kind':'SDK reference-waypoint IK only; no clearance/contact/torque claim','rows':rows,'all_pass':all(r['pass_nominal'] for r in rows)},indent=2))
