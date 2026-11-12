"""Geometric embodiment interface: whole distal finger volume, never normals.
No physical state writes. Runtime independent tip poses include actual jaw motion.
"""
from pathlib import Path
import numpy as np
from runtime_credit import load
DATA=load(Path(__file__).with_name('pad-region.json'))
SOLIDS={n:[np.asarray(s['planes'],dtype=float) for s in a['solids']] for n,a in DATA['actors'].items()}
def inside_local(name,point):
 if name not in SOLIDS:return False
 p=np.asarray(point,dtype=float)
 if p.shape!=(3,) or not np.isfinite(p).all():raise ValueError('invalid pad contact point')
 tol=DATA['boundary_tolerance_m']
 if p[2]>DATA['actors'][name]['cut_z_max_m']+tol:return False
 return any(bool(np.all(q[:,:3]@p+q[:,3]<=tol)) for q in SOLIDS[name])
def contact_is_pad(body_a,body_b,point_w,body_poses):
 """Either actor order; body names and poses keyed by exact resolved path.
 body_poses[path]=(tip name, world position, world rotation). No normal filter.
 """
 for body in (body_a,body_b):
  if body in body_poses:
   name,p,R=body_poses[body]
   if inside_local(name,np.asarray(R).T@(np.asarray(point_w)-np.asarray(p))):return True
 return False
