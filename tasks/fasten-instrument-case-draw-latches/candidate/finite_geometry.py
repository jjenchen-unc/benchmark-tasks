"""CPU finite convex collision envelopes. Positive projected gaps certify clearance.
Combined collision mesh hulls overapproximate links; a nonpositive gap is unresolved,
never a certificate of actual mesh collision. Task boxes exact; cylinders circumscribed.
"""
import itertools,xml.etree.ElementTree as ET
from functools import lru_cache
import numpy as np
from scipy.spatial import ConvexHull
from yam_arena import YAM_URDF
from yam_arena.constants import ARM_JOINTS,FINGER_JOINTS,FINGER_OPEN
from yam_arena.station import BASES,TABLE_TOP,TABLE_SIZE,TABLE_CENTER,camera_stand,top_camera
from yam_arena.urdf import forward_kinematics,rpy_matrix,quat_xyzw_to_matrix

def hull(v):
 v=np.unique(np.asarray(v),axis=0);h=ConvexHull(v);return (v[h.vertices],np.unique(h.equations[:,:3].round(9),axis=0))
def box(half):return hull(np.array(list(itertools.product([-1.,1.],repeat=3)))*half)
def moved(h,R,p):return (h[0]@R.T+p,h[1]@R.T)
def gap(a,b,_bounds=None,_broad=None,_projections=None,_certificate=False):
 av,bv=a[0],b[0]
 if _broad is not None:
  broad=_broad
 elif _bounds is None:
  broad=float(np.maximum(bv.min(0)-av.max(0),av.min(0)-bv.max(0)).max())
 else:
  alo,ahi=_bounds(a);blo,bhi=_bounds(b)
  broad=float(np.maximum(blo-ahi,alo-bhi).max())
 if broad>.002:return broad
 axes=np.vstack([a[1],b[1]])
 # Only check's acceptance decision may consume a partial-axis certificate.
 # exact_gap and all unsuccessful certificates retain the original full matrix.
 if _certificate and av.dtype==np.float64 and bv.dtype==np.float64 and axes.dtype==np.float64 and len(axes)>8:
  # Bounded actual workspace; unknown/nonfinite inputs take the original path.
  # 3-term dot absolute error <= gamma_3 * 3*16*2 (<6.4e-14).
  # Two projections and subtraction, including backend reassociation, are
  # conservatively enclosed by 1e-12; return the downward guarded bound.
  if max(np.abs(av).max(),np.abs(bv).max())<=16 and np.abs(axes).max()<=2:
   # Bound speculative certificate work; unsuccessful searches still use ALL
   # original axes and the unchanged full dot products below.
   for start in range(0,min(len(axes),512),8):
    part=axes[start:start+8];pa=np.ascontiguousarray((av@part.T).T);pb=np.ascontiguousarray((bv@part.T).T)
    lower=float(np.maximum(pb.min(1)-pa.max(1),pa.min(1)-pb.max(1)).max())-1e-12
    if lower>=.0001:return lower
 pa=np.ascontiguousarray((av@axes.T).T);pb=np.ascontiguousarray((bv@axes.T).T)
 return float(np.maximum(pb.min(1)-pa.max(1),pa.min(1)-pb.max(1)).max())

@lru_cache(None)
def robot_local():
 import trimesh
 result={}
 for link in ET.parse(YAM_URDF).getroot().findall('link'):
  vertices=[]
  for c in link.findall('collision'):
   origin=c.find('origin');xyz=np.fromstring(origin.get('xyz','0 0 0'),sep=' ') if origin is not None else np.zeros(3);R=rpy_matrix(np.fromstring(origin.get('rpy','0 0 0'),sep=' ')) if origin is not None else np.eye(3)
   mesh=c.find('geometry/mesh');assert mesh is not None
   path=YAM_URDF.parent/mesh.get('filename');scale=np.fromstring(mesh.get('scale','1 1 1'),sep=' ')
   v=np.asarray(trimesh.load_mesh(path,process=False).vertices)*scale
   vertices.extend(v@R.T+xyz)
  if vertices:result[link.get('name')]=hull(vertices)
 return result

def robot(side,q,jaw,raw=None):
 joints=dict(zip(ARM_JOINTS,map(float,q),strict=True));joints.update({n:float(jaw)*FINGER_OPEN for n in FINGER_JOINTS})
 if raw is not None:
  assert set(raw)==set(ARM_JOINTS+FINGER_JOINTS)
  assert all(np.isfinite(v) and not isinstance(v,(bool,np.bool_)) for v in raw.values())
  joints={n:float(v) for n,v in raw.items()}
 fk=forward_kinematics(YAM_URDF,joints)
 return {n:moved(h,fk[n][:3,:3],fk[n][:3,3]+BASES[side]) for n,h in robot_local().items()}

@lru_cache(None)
def primitive(kind,size):
 if kind=='box':return box(np.array(size))
 if kind=='cylinder':
  radius,height=size[:2];theta=np.arange(32)*2*np.pi/32;radius/=np.cos(np.pi/32)
  return hull([[radius*np.cos(t),radius*np.sin(t),z] for t in theta for z in [-height,height]])
 raise ValueError(kind)

def scene(fixture,bodies):
 result={}
 for b in fixture['bodies']:
  p,R=bodies[b['name']]
  for g in b['geoms']:
   # Same collision-enabled physical shapes used by build_usd.py.
   if not (g.get('contype',1) or g.get('conaffinity',1)):continue
   h=primitive(g['kind'],tuple(g['size']))
   w,x,y,z=g['quat'];r=quat_xyzw_to_matrix(np.array([x,y,z,w]));result[b['name']+'/'+g['name']]=moved(h,R@r,p+R@np.array(g['pos']))
 result['plinth']=moved(primitive('box',(.1,.3,.03)),np.eye(3),np.array([.35,-.04,.7775]))
 for n,(kind,size,p,q) in camera_stand(top_camera()).items():
  half=np.array(size)/2 if kind=='box' else np.array([size[0],size[0],size[1]/2+size[0]])
  result[n]=moved(primitive('box',tuple(half)),quat_xyzw_to_matrix(np.array(q)),np.array(p))
 result['table']=moved(primitive('box',tuple(np.array(TABLE_SIZE)/2)),np.eye(3),np.asarray(TABLE_CENTER))
 return result

ADJACENT={frozenset((j.find('parent').get('link'),j.find('child').get('link'))) for j in ET.parse(YAM_URDF).getroot().findall('joint')}
def check(q,objects,allowed=(),raw_joint=None):
 arms={s:robot(s,q[i:i+6],q[i+6],None if raw_joint is None else raw_joint[s]) for s,i in [('left',0),('right',7)]};rows=[]
 # Enumerate identically, then batch only the exact broad AABB expression.
 # Everything here is query-local; current transforms/raw8 are rebuilt above.
 pairs=[]
 def add(kind,pair,a,b):pairs.append((kind,pair,a,b))
 for side,links in arms.items():
  for ln,a in links.items():
   for n,b in objects.items():
    if (side,ln,n) not in allowed:add('task', (side+'/'+ln,n),a,b)
  for (na,a),(nb,b) in itertools.combinations(links.items(),2):
   if frozenset((na,nb)) not in ADJACENT:add('self',(side+'/'+na,side+'/'+nb),a,b)
 for na,a in arms['left'].items():
  for nb,b in arms['right'].items():add('interarm',('left/'+na,'right/'+nb),a,b)
 bounds={}
 for _,_,a,b in pairs:
  for h in (a,b):
   if id(h) not in bounds:bounds[id(h)]=(len(bounds),h[0].min(0),h[0].max(0))
 lo=np.array([v[1] for v in bounds.values()]);hi=np.array([v[2] for v in bounds.values()])
 ai=np.array([bounds[id(a)][0] for _,_,a,b in pairs],dtype=np.intp)
 bi=np.array([bounds[id(b)][0] for _,_,a,b in pairs],dtype=np.intp)
 axis_bounds=np.maximum(lo[bi]-hi[ai],lo[ai]-hi[bi])
 broad=axis_bounds.max(1)
 # Only use AABB axes already present EXACTLY among original face normals.
 # This preserves the old face-SAT decision, not merely physical separation.
 certified={}
 axis_masks={}
 for k in np.flatnonzero((broad>.0001+1e-12)&(broad<=.002)):
  a,b=pairs[k][2:]
  if not all(h[0].dtype==np.float64 and np.abs(h[0]).max()<=16 for h in (a,b)):continue
  for h in (a,b):
   if id(h) not in axis_masks:
    normals=np.abs(h[1]);axis_masks[id(h)]=np.array([np.any(np.all(normals==axis,axis=1)) for axis in np.eye(3)])
  eligible=axis_masks[id(a)]|axis_masks[id(b)]
  if eligible.any():certified[k]=float(axis_bounds[k,eligible].max())-1e-12

 for k,((kind,pair,a,b),g) in enumerate(zip(pairs,broad,strict=True)):
  # Original far-pair result byte exact, avoiding hundreds of thousands of calls.
  if g>.002:value=float(g)
  elif certified.get(k,-np.inf)>=.0001:value=certified[k]
  else:value=gap(a,b,_broad=float(g),_certificate=True)
  rows.append({'kind':kind,'pair':list(pair),'separation_lower_m':value})
 fine={}
 for row in rows:
  if row['separation_lower_m']>=.0001:continue
  def components(n):
   if n in objects:return [objects[n]]
   side,link=n.split('/')
   if side not in fine:
    i=0 if side=='left' else 7
    fine[side]=pieces(side,q[i:i+6],q[i+6],None if raw_joint is None else raw_joint[side])
   return fine[side][link]
  a,b=row['pair'];row['separation_lower_m']=union_gap(components(a),components(b),exact_gap);row['narrow_phase']='individual URDF collision convex parts, full SAT when required'
 return {'pairs':len(rows),'minimum':min(rows,key=lambda r:r['separation_lower_m']),'unresolved':[r for r in rows if r['separation_lower_m']<.0001]}

@lru_cache(None)
def robot_pieces():
 import trimesh
 result={}
 for link in ET.parse(YAM_URDF).getroot().findall('link'):
  pieces=[]
  for c in link.findall('collision'):
   origin=c.find('origin');xyz=np.fromstring(origin.get('xyz','0 0 0'),sep=' ') if origin is not None else np.zeros(3);R=rpy_matrix(np.fromstring(origin.get('rpy','0 0 0'),sep=' ')) if origin is not None else np.eye(3)
   mesh=c.find('geometry/mesh');path=YAM_URDF.parent/mesh.get('filename');scale=np.fromstring(mesh.get('scale','1 1 1'),sep=' ')
   pieces.append(hull((np.asarray(trimesh.load_mesh(path,process=False).vertices)*scale)@R.T+xyz))
  result[link.get('name')]=pieces
 return result

def pieces(side,q,jaw,raw=None):
 joints=dict(zip(ARM_JOINTS,map(float,q),strict=True));joints.update({n:float(jaw)*FINGER_OPEN for n in FINGER_JOINTS})
 if raw is not None:joints.update(raw)
 fk=forward_kinematics(YAM_URDF,joints)
 return {n:[moved(h,fk[n][:3,:3],fk[n][:3,3]+BASES[side]) for h in hs] for n,hs in robot_pieces().items()}

def exact_gap(a,b):
 g=gap(a,b)
 if g>1e-8:return g
 def edges(v):
  h=ConvexHull(v);indices=set()
  for tri in h.simplices:
   for i,j in itertools.combinations(tri,2):indices.add(tuple(sorted((int(i),int(j)))))
  d=np.array([v[j]-v[i] for i,j in indices]);d/=np.linalg.norm(d,axis=1)[:,None]
  d*=np.where(d[np.arange(len(d)),np.argmax(abs(d),axis=1)]<0,-1,1)[:,None]
  return np.unique(d.round(9),axis=0)
 eb=edges(b[0])
 for e in edges(a[0]):
  axes=np.cross(e,eb);norm=np.linalg.norm(axes,axis=1);axes=axes[norm>1e-8]/norm[norm>1e-8,None]
  if not len(axes):continue
  pa=np.ascontiguousarray((a[0]@axes.T).T);pb=np.ascontiguousarray((b[0]@axes.T).T)
  g=max(g,float(np.maximum(pb.min(1)-pa.max(1),pa.min(1)-pb.max(1)).max()))
  if g>1e-8:break
 return g


def union_gap(aa,bb,exact_gap):
 alo=np.array([h[0].min(0) for h in aa]);ahi=np.array([h[0].max(0) for h in aa]);blo=np.array([h[0].min(0) for h in bb]);bhi=np.array([h[0].max(0) for h in bb])
 broad=np.maximum(blo[None,:,:]-ahi[:,None,:],alo[:,None,:]-bhi[None,:,:]).max(2)
 near=broad<.0001;minimum=float(broad[~near].min()) if (~near).any() else float('inf')
 for i,j in np.argwhere(near):minimum=min(minimum,exact_gap(aa[i],bb[j]))
 return minimum
