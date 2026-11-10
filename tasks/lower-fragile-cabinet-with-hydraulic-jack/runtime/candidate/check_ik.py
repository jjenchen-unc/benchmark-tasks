"""Actual installed SDK URDF IK, NumPy only; no simulator or model."""
import json,math
from pathlib import Path
import numpy as np
from yam_arena import ik,station
from yam_arena.constants import READY
f=json.loads(Path(__file__).with_name('fixture.json').read_text());delta=np.array(f['translation']);rows=[]
for arm,points in {'left':[(.015-.18*math.cos(t),y,.875+.18*math.sin(t)) for y in (.15,.45) for t in (0.,.28,.3)],'right':[(.26,y,.872+z) for y in (.15,.45) for z in (0.,.07)]}.items():
 for p in points:
  target=np.array(p)+delta;q,e,r=ik.solve(target,[0.,1.,0.,0.],base_pos=station.BASES[arm],q_init=READY,iters=400)
  rows.append(dict(arm=arm,target=target.tolist(),q=q.tolist(),position_error_m=e,rotation_error_rad=r,within_tolerance=e<.002 and r<.03))
print(json.dumps({'kind':'SDK nominal IK only; no collision or torque feasibility proof','rows':rows,'all_pass':all(x['within_tolerance'] for x in rows)},indent=2))
