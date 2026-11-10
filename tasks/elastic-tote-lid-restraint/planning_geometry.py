"""Pure validated planning geometry; no simulator APIs or historical-file reads."""
import numpy as np
from scipy.spatial import ConvexHull
from scipy.optimize import minimize

def complete_sat(vertices,c,R,half):
 v=(vertices-c)@R
 hull=ConvexHull(v)
 edges=set()
 for face in hull.simplices:
  for i,j in ((0,1),(0,2),(1,2)):edges.add(tuple(sorted((int(face[i]),int(face[j])))))
 directions=np.array([v[j]-v[i] for i,j in edges])
 axes=np.vstack([np.eye(3),hull.equations[:,:3],np.cross(directions[:,None,:],np.eye(3)[None,:,:]).reshape(-1,3)])
 norm=np.linalg.norm(axes,axis=1);axes=axes[norm>1e-9]/norm[norm>1e-9,None]
 projections=v@axes.T;radius=np.abs(axes)@half
 gap=np.maximum(projections.min(0)-radius,-radius-projections.max(0))
 return float(gap.max())

def segment_inside(P,Q,hull,margin=0.):
 lo,hi=0.,1.;v=Q-P
 for plane in hull.planes:
  n=plane[:3];d=plane[3]+margin
  a=float(n@v);b=float(d-n@P)
  if abs(a)<1e-12:
   if b<0:return False
  elif a>0:hi=min(hi,b/a)
  else:lo=max(lo,b/a)
  if lo>hi:return False
 return hi-lo>1e-7

def sat_hulls(a,b):
 av,bv=a.vertices,b.vertices
 broad=np.maximum(bv.min(0)-av.max(0),av.min(0)-bv.max(0)).max()
 if broad>.02:return float(broad)
 def directions(v):
  hull=ConvexHull(v);edges=set()
  for face in hull.simplices:
   for i,j in ((0,1),(0,2),(1,2)):edges.add(tuple(sorted((int(face[i]),int(face[j])))))
  d=np.array([v[j]-v[i] for i,j in edges]);n=np.linalg.norm(d,axis=1);d=d[n>1e-10]/n[n>1e-10,None]
  d=np.where((d[np.arange(len(d)),np.argmax(np.abs(d),axis=1)]<0)[:,None],-d,d)
  return np.unique(d.round(7),axis=0)
 # Face axes first; any positive projection gap proves separation.
 def project_axes(axes):
  n=np.linalg.norm(axes,axis=1);axes=axes[n>1e-8]/n[n>1e-8,None]
  best=-float('inf')
  for at in range(0,len(axes),256):
   ax=axes[at:at+256];pa,pb=av@ax.T,bv@ax.T
   gap=np.maximum(pb.min(0)-pa.max(0),pa.min(0)-pb.max(0));best=max(best,float(gap.max()))
   if best>1e-8:return best
  return best
 faces=np.unique(np.vstack([a.planes[:,:3],b.planes[:,:3]]).round(8),axis=0)
 best=project_axes(faces)
 if best>1e-8:return best
 # Stream edge cross-products to bound memory; exact same complete SAT axis family.
 da,db=directions(av),directions(bv)
 for direction in da:
  candidate=project_axes(np.cross(direction,db))
  best=max(best,candidate)
  if best>1e-8:return best
 return best

def exact_segment_hull(P,Q,h):
 if segment_inside(P,Q,h):return dict(centreline_inside=True,distance_upper_m=0.,distance_lower_m=0.,certificate_gap_m=0.,solver_success=True)
 eq=ConvexHull(h.vertices).equations;planes=np.unique(eq.round(11),axis=0);normals=planes[:,:3];offsets=-planes[:,3];v=Q-P
 centroid=h.vertices.mean(0);t=float(np.clip((centroid-P)@v/max(v@v,1e-20),0,1));initial=np.r_[centroid,t]
 def fun(z):
  d=z[:3]-P-z[3]*v;return float(d@d)
 def jac(z):
  d=z[:3]-P-z[3]*v;return np.r_[2*d,-2*(v@d)]
 def cons(z):return offsets-normals@z[:3]
 def cjac(z):return np.c_[-normals,np.zeros(len(normals))]
 fit=minimize(fun,initial,jac=jac,bounds=[(None,None)]*3+[(0,1)],constraints=[dict(type='ineq',fun=cons,jac=cjac)],method='SLSQP',options=dict(ftol=1e-16,maxiter=300))
 x=fit.x[:3];y=P+fit.x[3]*v;distance=float(np.linalg.norm(y-x));n=(y-x)/max(distance,1e-20)
 # This separating projection certifies a Euclidean lower bound independently of solver convergence.
 sep=max(float(min(P@n,Q@n)-np.max(h.vertices@n)),float(np.min(h.vertices@n)-max(P@n,Q@n)),0.)
 return dict(centreline_inside=False,distance_upper_m=distance,distance_lower_m=sep,certificate_gap_m=distance-sep,solver_success=bool(fit.success),constraint_violation_m=float(max(0.,-cons(fit.x).min())))

def sepbox(ca,Ra,ha,cb,Rb,hb):
 axes=np.vstack([Ra.T,Rb.T,np.cross(Ra.T[:,None,:],Rb.T[None,:,:]).reshape(-1,3)]);norm=np.linalg.norm(axes,axis=1);axes=axes[norm>1e-10]/norm[norm>1e-10,None]
 return float(np.max(np.abs(axes@(ca-cb))-(np.abs(axes@Ra)@ha+np.abs(axes@Rb)@hb)))

def hull_set_clearance(aa,bb):
 if not aa or not bb:return 1.,None
 alo=np.array([h.vertices.min(0) for _,h in aa]);ahi=np.array([h.vertices.max(0) for _,h in aa])
 blo=np.array([h.vertices.min(0) for _,h in bb]);bhi=np.array([h.vertices.max(0) for _,h in bb])
 gaps=np.maximum(blo[None,:,:]-ahi[:,None,:],alo[:,None,:]-bhi[None,:,:]).max(2)
 far=gaps>.02;minimum=min(1.,float(gaps[far].min())) if far.any() else 1.;near=None
 for i,j in np.argwhere(~far):
  d=sat_hulls(aa[i][1],bb[j][1])
  if d<minimum:minimum=d;near=(aa[i][0],bb[j][0])
 return minimum,near

_face_cache={}
def sat(vertices,c,R,half):
    entry=_face_cache.get(id(vertices))
    if entry is None:
        if len(_face_cache)>4096:_face_cache.clear()
        entry=(vertices,np.unique(ConvexHull(vertices).equations[:,:3].round(10),axis=0))
        _face_cache[id(vertices)]=entry
    axes=np.vstack([R.T,entry[1]]);norm=np.linalg.norm(axes,axis=1)
    axes=axes[norm>1e-10]/norm[norm>1e-10,None]
    projected=(vertices-c)@axes.T;radius=np.abs(axes@R)@half
    gap=float(np.maximum(projected.min(0)-radius,-radius-projected.max(0)).max())
    return gap if gap>1e-8 else complete_sat(vertices,c,R,half)
