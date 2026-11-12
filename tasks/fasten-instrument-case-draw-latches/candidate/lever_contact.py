"""Finite convex closest-feature tab contact, without modifying task geometry.
A six-variable convex distance solve supplies the direction; no endpoint grid.
"""
import numpy as np
from scipy.optimize import minimize,LinearConstraint
from scipy.spatial import ConvexHull
from finite_geometry import pieces,box,moved,gap
from yam_arena.urdf import axis_angle_matrix,quat_xyzw_to_matrix
from yam_arena.constants import PAD_TIP_PAST_TCP

def closest(a,b):
 ha,hb=ConvexHull(a[0]),ConvexHull(b[0]);A=np.zeros((len(ha.equations)+len(hb.equations),6));A[:len(ha.equations),:3]=ha.equations[:,:3];A[len(ha.equations):,3:]=hb.equations[:,:3]
 upper=-np.r_[ha.equations[:,3],hb.equations[:,3]]
 def fun(x):d=x[:3]-x[3:];return .5*np.dot(d,d)
 def jac(x):d=x[:3]-x[3:];return np.r_[d,-d]
 fit=minimize(fun,np.r_[a[0].mean(0),b[0].mean(0)],jac=jac,constraints=[LinearConstraint(A,-np.inf,upper)],method='SLSQP',options={'ftol':1e-13,'maxiter':150})
 if not fit.success or float(np.max(A@fit.x-upper))>1e-7:raise RuntimeError('Convex tab distance solve failed: '+fit.message)
 return fit.x[:3],fit.x[3:]

def target(controller,j,theta,deflection=.0005):
 n=f'A{j}';bodies,joints,_=controller.state();import kit
 meta=next(b for b in kit.FIXTURE['bodies'] if b['name']==n+'_lever');g=next(g for g in meta['geoms'] if g['name']==n+'_lever_tab')
 pivot,lr=bodies[n+'_lever'];Rlever=lr@axis_angle_matrix(np.array(meta['joints'][0]['axis']),theta-joints[n+'_lever']);w,x,y,z=g['quat'];Rg=Rlever@quat_xyzw_to_matrix(np.array([x,y,z,w]));center=pivot+Rlever@np.array(g['pos']);tab=moved(box(np.array(g['size'])),Rg,center)
 R=controller.rot(np.pi/4);nominal=controller.leverpoint(j,theta,.009,-.05)+[0,0,.025]
 q=controller.solve('left',nominal-PAD_TIP_PAST_TCP*R[:,2],R);pp=pieces('left',q,.08);best=None
 for finger in ('tip_left','tip_right'):
  for h in pp[finger]:
   if best is not None and gap(h,tab)>best[0]:continue
   a,b=closest(h,tab);distance=float(np.linalg.norm(b-a))
   if best is None or distance<best[0]:best=(distance,a,b,finger)
 distance,a,b,finger=best
 if distance<1e-6:raise RuntimeError('Tab seed already intersects; no reliable contact normal')
 direction=(b-a)/distance;torque=float(np.dot(Rlever@np.array(meta['joints'][0]['axis']),np.cross(b-pivot,direction)))
 if torque>=0:raise RuntimeError('Closest tab feature would rotate lever away from closed')
 return nominal+(distance+deflection)*direction,R,{'distance_m':distance,'contact_direction':direction.tolist(),'torque_per_N_m':torque,'finger':finger,'deflection_m':deflection}
