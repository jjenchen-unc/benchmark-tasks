"""Triangle/OBB SAT for diagnostic tool descent; no scene mutation."""
import numpy as np

def triangle_box_hits(triangles,centre,R,half,padding=0.):
 t=(np.asarray(triangles,float)-np.asarray(centre))@np.asarray(R);h=np.asarray(half)+padding
 if t.ndim!=3 or t.shape[1:]!=(3,3):raise ValueError('triangles must be Nx3x3')
 if not np.isfinite(t).all() or not np.isfinite(h).all() or np.any(h<=0):raise ValueError('invalid collision geometry')
 separated=np.any((t.min(axis=1)>h+1e-10)|(t.max(axis=1)<-h-1e-10),axis=1)
 edges=np.stack((t[:,1]-t[:,0],t[:,2]-t[:,1],t[:,0]-t[:,2]),axis=1)
 axes=[np.cross(edges[:,0],edges[:,1])]
 for i in range(3):
  for basis in np.eye(3):axes.append(np.cross(edges[:,i],basis))
 for a in axes:
  p=np.einsum('ntd,nd->nt',t,a);radius=np.sum(abs(a)*h,axis=1);nonzero=np.sum(a*a,axis=1)>1e-24
  separated|=nonzero&((p.min(axis=1)>radius+1e-10)|(p.max(axis=1)<-radius-1e-10))
 return ~separated

def swept_translation_hits(triangles,translation,centre,R,half,padding=.002):
 # Minkowski union of boxes opposite the tool translation. Bounding OBB is
 # conservative for arbitrary translations, exact along a box principal axis.
 delta=np.asarray(translation,float);RR=np.asarray(R)
 return triangle_box_hits(triangles,np.asarray(centre)-delta/2,RR,np.asarray(half)+abs(RR.T@delta)/2,padding)

def verify():
 cases=[(np.array([[[1.,0,0],[1.,.2,0],[1.,0,.2]]]),np.ones(3),True),(np.array([[[1.001,0,0],[1.001,.2,0],[1.001,0,.2]]]),np.ones(3),False),(np.array([[[-2.,0,0],[2.,0,0],[0,2.,0]]]),np.ones(3),True),(np.array([[[2.,2,0],[2,0,2],[0,2,2.]]]),np.full(3,.5),False)]
 for t,h,want in cases:assert bool(triangle_box_hits(t,np.zeros(3),np.eye(3),h)[0]) is want
 return 4
