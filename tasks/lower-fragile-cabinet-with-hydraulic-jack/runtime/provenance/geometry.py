"""CPU collision analysis. Per-collider finite meshes; conservative separating-axis bounds."""
import json,struct,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull
from yam_arena import YAM_URDF,station
from yam_arena.urdf import forward_kinematics,rpy_matrix,quat_xyzw_to_matrix
U=Path(YAM_URDF); INPUTS={str(U)}; M=[]
for link in ET.parse(U).getroot().findall('link'):
 for i,c in enumerate(link.findall('collision')):
  mesh=c.find('geometry/mesh');p=U.parent/mesh.get('filename');INPUTS.add(str(p));b=p.read_bytes();n=struct.unpack_from('<I',b,80)[0]
  v=np.frombuffer(b,dtype=np.dtype([('normal','<f4',3),('v','<f4',(3,3)),('attr','<u2')]),offset=84,count=n)['v'].reshape(-1,3).astype(float)
  o=c.find('origin');R=rpy_matrix(np.fromstring(o.get('rpy','0 0 0'),sep=' '));v=(v*np.fromstring(mesh.get('scale','1 1 1'),sep=' '))@R.T+np.fromstring(o.get('xyz','0 0 0'),sep=' ')
  v=np.unique(v,axis=0);h=ConvexHull(v);norm=h.equations[:,:3];edges=np.concatenate([v[h.simplices[:,j]]-v[h.simplices[:,(j+1)%3]] for j in range(3)])
  M.append((link.get('name'),i,v,norm,edges))
def fk(raw,side='right'):
 t=forward_kinematics(U,dict(zip(['joint'+str(i) for i in range(1,9)],raw)))
 for a in t.values():a[:3,3]+=station.BASES[side]
 return t
SIGNS=np.array([[a,b,c] for a in (-1,1) for b in (-1,1) for c in (-1,1)])
def gap(v,n,e,lo,hi):
 axes=np.concatenate((np.eye(3),n,np.cross(e[:,None,:],np.eye(3)[None,:,:]).reshape(-1,3)))
 lengths=np.linalg.norm(axes,axis=1);axes=axes[lengths>1e-9]/lengths[lengths>1e-9,None]
 box=(lo+hi)/2+SIGNS*(hi-lo)/2;av=v@axes.T;bv=box@axes.T
 return float(np.max(np.maximum(bv.min(0)-av.max(0),av.min(0)-bv.max(0))))
def boxes(F,drop=0):
 out=[];D=np.array(F['translation'])
 for b in F['bodies']:
  if b['name'] not in ('world','cabinet'):continue
  for g in b['geoms']:
   if not g['collision']:continue
   p=np.array(g['pos'])+D
   if b['name']=='cabinet':p+=np.array(b['world_pos'])-[0,0,drop]
   s=np.array(g['size']);out.append((g['name'],p-s,p+s))
 return out
def audit(raw,F,side='right',drop=0):
 t=fk(raw,side);out=[]
 for name,i,v,n,e in M:
  a=t[name];R=a[:3,:3];w=v@R.T+a[:3,3]
  for bn,lo,hi in boxes(F,drop):
   broad=np.max(np.maximum(lo-w.max(0),w.min(0)-hi))
   if broad>.02:continue
   d=gap(w,n@R.T,e@R.T,lo,hi)
   out.append((d,name,i,bn))
 return sorted(out)
# Freeze all Python/binary modules consumed by the CPU geometry path as well as meshes.
def _freeze_consumed(output=Path(__file__).with_name('geometry-consumed-SHA256.json')):
 import sys,hashlib
 paths=set(INPUTS)|{getattr(m,'__file__','') for m in list(sys.modules.values())}
 out={p:hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in sorted(p for p in paths if p and Path(p).is_file())}
 output.write_text(json.dumps(out,indent=2,allow_nan=False)+'\n')
import atexit
atexit.register(_freeze_consumed)
