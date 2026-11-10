"""Post-path finite convex robot audit; uses frozen route, no IK retries."""
import sys,json,time
from pathlib import Path
R=Path(__file__).resolve().parent;sys.path.insert(0,str(R));sys.path.insert(0,str(R/'candidate'))
import geometry as G,mount
import numpy as np
from yam_arena import station

def dirs(a):
 l=np.linalg.norm(a,axis=1);a=a[l>1e-9]/l[l>1e-9,None];return np.unique(np.round(a,7),axis=0)
# Subset of SAT axes: every positive separation is valid; unresolved intersections reject.
from scipy.spatial import ConvexHull
M=[(n,i,v[ConvexHull(v).vertices],dirs(norm),dirs(edge)) for n,i,v,norm,edge in G.M]
order={'base':0,'link1':1,'link2':2,'link3':3,'link4':4,'link5':5,'gripper':6,'tip_left':7,'tip_right':7}
def world(q,g,side):
 t=G.fk(np.r_[q,[-.04695*g]*2],side);out=[]
 for n,i,v,norm,e in M:
  a=t[n];rot=a[:3,:3];w=v@rot.T+a[:3,3]
  if side=='right':w=(w-station.BASES[side])@mount.ROT.T+mount.BASE;rot=mount.ROT@rot
  out.append((n,i,w,norm@rot.T,e@rot.T))
 return out
CACHE={}
def separation(a,b):
 _,_,v,n,e=a;_,_,w,m,f=b
 key=(a[0],a[1],b[0],b[1]);cached=CACHE.get(key)
 if cached is not None:
  av=v@cached;bv=w@cached;d=max(av.min()-bv.max(),bv.min()-av.max())
  if d>0:return float(d)
 d=float(np.max(np.maximum(v.min(0)-w.max(0),w.min(0)-v.max(0))))
 if d>0:return d
 axes=np.concatenate((n[::max(1,len(n)//16)],m[::max(1,len(m)//16)]));av=v@axes.T;bv=w@axes.T
 d=float(np.max(np.maximum(av.min(0)-bv.max(0),bv.min(0)-av.max(0))))
 if d>0:
  gaps=np.maximum(av.min(0)-bv.max(0),bv.min(0)-av.max(0));CACHE[key]=axes[int(np.argmax(gaps))];return d
 axes=np.concatenate((n,m,np.cross(e[:,None,:],f[None,:,:]).reshape(-1,3)));axes=dirs(axes);av=v@axes.T;bv=w@axes.T
 gaps=np.maximum(av.min(0)-bv.max(0),bv.min(0)-av.max(0));i=int(np.argmax(gaps));CACHE[key]=axes[i];return float(gaps[i])
commands=json.loads((R/'controller-contract.json').read_text())['actions_14d'];route={'rows':[{'q':q[7:13],'grip':q[13],'phase':'actual_driver'} for q in [[0,1,1,-.6,0,0,1]*2]+commands]};start=time.monotonic();fails=[];minimum=1.;count=0
left=world([0,1,1,-.6,0,0],1,'left')
for idx,row in enumerate(route['rows']):
 right=world(row['q'],row['grip'],'right')
 for side,parts in [('right',right),('left',left if idx==0 else [])]:
  lows=np.array([a[2].min(0) for a in parts]);highs=np.array([a[2].max(0) for a in parts])
  if not parts:continue
  for i,a in enumerate(parts):
   js=np.array([j for j,b in enumerate(parts) if j>i and abs(order[a[0]]-order[b[0]])>1],dtype=int)
   if not len(js):continue
   ds=np.max(np.maximum(lows[js]-highs[i],lows[i]-highs[js]),axis=1);count+=len(js)
   for k in np.flatnonzero(ds<=0):ds[k]=separation(a,parts[js[k]])
   minimum=min(minimum,float(ds.min()))
   for k in np.flatnonzero(ds<0):fails.append(dict(index=idx,phase=row['phase'],side=side,pair=[a[0],parts[js[k]][0]],gap_m=float(ds[k])))
 # A union AABB witness proves all finite interarm mesh pairs clear at once.
 lr=np.min([a[2].min(0) for a in right],axis=0);hr=np.max([a[2].max(0) for a in right],axis=0);ll=np.min([a[2].min(0) for a in left],axis=0);hl=np.max([a[2].max(0) for a in left],axis=0)
 d=float(np.max(np.maximum(lr-hl,ll-hr)));count+=len(right)*len(left);minimum=min(minimum,d)
 if d<=0:
  for a in right:
   for b in left:
    d=separation(a,b)
    if d<0:fails.append(dict(index=idx,phase=row['phase'],side='interarm',pair=[a[0],b[0]],gap_m=d))
 if fails:break
out=dict(passed=not fails,failures=fails[:30],failure_count=len(fails),min_separation_m=minimum,pairs=count,wall_s=time.monotonic()-start,coverage='finite all URDF collision convex hulls; adjacent bodies excluded at mechanical joints, original SDK self collisions unchanged')
(R/'self-audit.json').write_text(json.dumps(out,indent=2,allow_nan=False)+'\n');print(json.dumps(out),flush=True)
