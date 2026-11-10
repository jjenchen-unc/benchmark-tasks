"""New supported opposite-side mount; pure URDF/IK, no simulator imports."""
import sys,json,time,math
from pathlib import Path
R=Path(__file__).resolve().parent;sys.path.insert(0,str(R))
import geometry as G
import numpy as np
from yam_arena import ik,station
F=json.loads((R/'candidate/fixture.json').read_text());start=time.monotonic();rows=[]
P=np.array([.38,-.15,.8225]);rot=np.diag([-1.,1.,-1.]);Z=np.diag([-1.,-1.,1.]);original=G.fk
for x,z in ((.68,.75),(.73,.75),(.78,.75),(.68,.85),(.73,.85),(.78,.85)):
 base=np.array([x,-.15,z]);q=np.array([0,1,1,-.6,0,0.])
 def fk(raw,side='right'):
  t=original(raw,side)
  for a in t.values():a[:3,3]=Z@(a[:3,3]-station.BASES[side])+base;a[:3,:3]=Z@a[:3,:3]
  return t
 G.fk=fk;path=[]
 for y in np.linspace(0,math.radians(115),61):
  c,s=math.cos(y),math.sin(y);rz=np.array([[c,-s,0],[s,c,0],[0,0,1]])
  q,e,r=ik.solve(Z.T@(P-base),ik.matrix_to_quat_xyzw(Z.T@rz@rot),q_init=q,iters=160)
  a=G.audit(np.r_[q,[-.04695]*2],F)
  path.append(dict(yaw=float(y),q=q.tolist(),e=float(e),r=float(r),worst=a[:1]))
 row=dict(base=base.tolist(),yaw_rad=math.pi,path=path,max_e=max(a['e'] for a in path),max_r=max(a['r'] for a in path),min_gap=min(a['worst'][0][0] if a['worst'] else .02 for a in path))
 rows.append(row);print({k:v for k,v in row.items() if k!='path'},flush=True)
G.fk=original
(R/'mount-analysis.json').write_text(json.dumps(dict(rows=rows,wall_s=time.monotonic()-start,evidence='CPU conditional cabinet/rail mesh geometry only'),indent=2,allow_nan=False)+'\n')
