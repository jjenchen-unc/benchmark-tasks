"""CPU conditional full nominal route on NEW reset contract. No physics initialization."""
import sys,json,time,math
from pathlib import Path
R=Path(__file__).resolve().parent;sys.path.insert(0,str(R));sys.path.insert(0,str(R/'candidate'))
import geometry as G,mount
import numpy as np
from yam_arena import ik,station
from yam_arena.urdf import quat_xyzw_to_matrix
# Retain exact convex support vertices; axis deduplication provides valid SAT witnesses.
from scipy.spatial import ConvexHull
def axes(a):
 l=np.linalg.norm(a,axis=1);a=a[l>1e-9]/l[l>1e-9,None];return np.unique(np.round(a,7),axis=0)
G.M=[(n,i,v[ConvexHull(v).vertices],axes(norm),axes(edge)) for n,i,v,norm,edge in G.M]
F=json.loads((R/'candidate/fixture.json').read_text());oldfk=G.fk;start=time.monotonic();rows=[]
def fk(raw,side='right'):
 t=oldfk(raw,side)
 if side=='right':
  for a in t.values():a[:3,3]=mount.ROT@(a[:3,3]-station.BASES[side])+mount.BASE;a[:3,:3]=mount.ROT@a[:3,:3]
 return t
G.fk=fk
# Source primitives enclosed conservatively by oriented local boxes. No finite cylinder omitted.
obs=G.boxes(F)
cabinet_names={g['name'] for b in F['bodies'] if b['name']=='cabinet' for g in b['geoms']}
obs=[(n,lo-np.array([0,0,.055]) if n in cabinet_names else lo,hi) for n,lo,hi in obs]
obs.extend([('table',np.array(station.TABLE_CENTER)-np.array(station.TABLE_SIZE)/2,np.array(station.TABLE_CENTER)+np.array(station.TABLE_SIZE)/2),('mount_plate',np.array([.62,-.21,.7475]),np.array([.74,-.09,.75]))])
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
def witness(v,n,e,lo,hi):
 # Any axis with disjoint support intervals certifies separation. Try a small
 # deterministic subset first; retain original complete SAT fallback on ambiguity.
 axes0=np.concatenate((n[::max(1,len(n)//16)],np.cross(e[::max(1,len(e)//16),None,:],np.eye(3)[None,:,:]).reshape(-1,3)))
 lengths=np.linalg.norm(axes0,axis=1);a=axes0[lengths>1e-9]/lengths[lengths>1e-9,None];center=(hi+lo)/2;radius=(abs(a)*(hi-lo)/2).sum(1);pv=v@a.T;pc=center@a.T
 d=float(np.max(np.maximum(pc-radius-pv.max(0),pv.min(0)-pc-radius)))
 if d>0:return d
 return G.gap(v,n,e,lo,hi)
def audit(q,g,phase):
 t=fk(np.r_[q,[-.04695*g]*2],'left');worst=[1.,None,None];inter=1.
 for name,i,v,n,e in G.M:
  T=t[name];rot=T[:3,:3];w=v@rot.T+T[:3,3]
  for bn,lo,hi in obs:
   if bn=='mount_plate' and name in ('base','link1'):continue
   if name.startswith('tip_') and ('valve' in bn or 'grip_' in bn):continue
   d=float(np.max(np.maximum(lo-w.max(0),w.min(0)-hi)))
   if d<=0:d=witness(w,n@rot.T,e@rot.T,lo,hi)
   if d<worst[0]:worst=[d,name,bn]
  for ln,lo,hi in leftbounds:inter=min(inter,float(np.max(np.maximum(lo-w.max(0),w.min(0)-hi))))
 return worst,inter


a,b=audit(np.array([0,1,1,-.6,0,0.]),1.,'parked_left');out={'passed':a[0]>=0,'worst':a,'wall_s':time.monotonic()-start,'coverage':'parked unchanged left robot all URDF colliders versus same fixtures/swept-cabinet/table/plate'};(R/'left-fixture-audit.json').write_text(json.dumps(out,indent=2,allow_nan=False)+'\n');print(json.dumps(out),flush=True)
