"""CPU conditional full nominal route on NEW reset contract. No physics initialization."""
import sys,json,time,math
from pathlib import Path
R=Path(__file__).resolve().parent;sys.path.insert(0,str(R));sys.path.insert(0,str(R/'candidate'))
import geometry as G,mount
import numpy as np
from yam_arena import ik,station
from yam_arena.urdf import quat_xyzw_to_matrix
F=json.loads((R/'candidate/fixture.json').read_text());oldfk=G.fk;start=time.monotonic();rows=[]
def fk(raw,side='right'):
 t=oldfk(raw,side)
 if side=='right':
  for a in t.values():a[:3,3]=mount.ROT@(a[:3,3]-station.BASES[side])+mount.BASE;a[:3,:3]=mount.ROT@a[:3,:3]
 return t
G.fk=fk
# Source primitives enclosed conservatively by oriented local boxes. No finite cylinder omitted.
obs=G.boxes(F);obs.extend([('table',np.array(station.TABLE_CENTER)-np.array(station.TABLE_SIZE)/2,np.array(station.TABLE_CENTER)+np.array(station.TABLE_SIZE)/2),('mount_plate',np.array([.62,-.21,.7475]),np.array([.74,-.09,.75]))])
for b in F['bodies']:
 if b['name'] in ('world','cabinet'):continue
 for g in b['geoms']:
  if not g['collision']:continue
  rot=quat_xyzw_to_matrix(np.array(b['quat_wxyz'])[list((1,2,3,0))]);gr=quat_xyzw_to_matrix(np.array(g['quat_wxyz'])[list((1,2,3,0))]);sz=np.array(g['size']);kind=g['kind']
  if kind in ('cylinder','capsule'):sz=np.array([sz[0],sz[0],sz[1]+(sz[0] if kind=='capsule' else 0)])
  p=rot@np.array(g['pos'])+np.array(b['world_pos'])+F['translation']
  if b['name']=='ram_S':p[2]+=.055
  extent=abs(rot@gr)@sz;obs.append((b['name']+'/'+g['name'],p-extent,p+extent))
left=fk(np.r_[[0,1,1,-.6,0,0],[-.04695]*2],'left')
leftbounds=[(n,(v@left[n][:3,:3].T+left[n][:3,3]).min(0),(v@left[n][:3,:3].T+left[n][:3,3]).max(0)) for n,i,v,nn,ee in G.M]
def audit(q,g,phase):
 t=fk(np.r_[q,[-.04695*g]*2]);worst=[1.,None,None];inter=1.
 for name,i,v,n,e in G.M:
  T=t[name];rot=T[:3,:3];w=v@rot.T+T[:3,3]
  for bn,lo,hi in obs:
   if bn=='mount_plate' and name in ('base','link1'):continue
   if name.startswith('tip_') and ('valve' in bn or 'grip_' in bn):continue
   d=float(np.max(np.maximum(lo-w.max(0),w.min(0)-hi)))
   if d<.02:d=G.gap(w,n@rot.T,e@rot.T,lo,hi)
   if d<worst[0]:worst=[d,name,bn]
  for ln,lo,hi in leftbounds:inter=min(inter,float(np.max(np.maximum(lo-w.max(0),w.min(0)-hi))))
 return worst,inter
q=np.array([0,1,1,-.6,0,0.]);yaw=math.pi;g=1.
def record(phase,e=0,r=0):
 a,b=audit(q,g,phase);rows.append(dict(phase=phase,q=q.tolist(),grip=float(g),e=float(e),r=float(r),worst=a,interarm_aabb_gap=b))
record('new_reset')
P=np.array([.38,-.15,.8225])
for phase,point,goal_y,goal_g in [('approach',P+[0,0,.07],math.pi,1),('descend',P,math.pi,1),('grip',P,math.pi,0),('open_turn',P,math.pi+math.radians(112),0),('close_turn',P,math.pi+math.radians(105.651),0),('release',P,math.pi+math.radians(105.651),1),('retreat',P+[0,0,.07],math.pi+math.radians(105.651),1)]:
 p0,r0=mount.tcp(q);g0=g;y0=yaw;n=max(60,math.ceil(np.linalg.norm(point-p0)/.002),math.ceil(abs(goal_y-y0)/.015))
 for k in range(1,n+1):
  s=k/n;yaw=y0+s*(goal_y-y0);c,t=math.cos(yaw),math.sin(yaw);rot=np.array([[-c,-t,0],[-t,c,0],[0,0,-1.]])
  q,e,r=mount.solve(p0+(point-p0)*s,ik.slerp_matrix(r0,rot,s),q);g=g0+(goal_g-g0)*s;record(phase,e,r)
  if e>.003 or r>.03 or rows[-1]['worst'][0]<-.00001:break
 print(phase,rows[-1],flush=True)
 if e>.003 or r>.03 or rows[-1]['worst'][0]<-.00001:break
out=dict(rows=rows,wall_s=time.monotonic()-start,passed=all(x['e']<.003 and x['r']<.03 and x['worst'][0]>=0 and x['interarm_aabb_gap']>0 for x in rows),coverage='all URDF collisions versus source cabinet/rails/jack primitive enclosing AABBs/table/plate and left robot AABBs; self collision and swept cabinet lowering not yet certified',evidence='CPU conditional synthetic nominal path; no physical score')
(R/'full-route.json').write_text(json.dumps(out,indent=2,allow_nan=False)+'\n')
