"""Experimental global-compliance pad: rigid top face on a passive six-DOF elastic support.
Matches native declared whole-face and centred-probe K, while retaining tilt/shear and momentum reaction.
It is a constitutive approximation, not a tetrahedral mesh or robot controller.
"""
from dataclasses import dataclass
import math
import numpy as np

def skew(v):
 x,y,z=np.asarray(v,dtype=float)
 return np.array([[0,-z,y],[z,0,-x],[-y,x,0]],dtype=float)

def rotation_xyzw(q):
 q=np.asarray(q,dtype=float)
 if q.shape!=(4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>1e-3:
  raise ValueError('Finite unit xyzw quaternion required')
 q=q/np.linalg.norm(q);x,y,z,w=q
 return np.eye(3)+2*w*skew(q[:3])+2*skew(q[:3])@skew(q[:3])

def dense_elastic(K,stroke,compression):
 u=(compression-stroke)/.0015
 if u<=0:return 0.,0.
 uc=min(u,4.);e=math.exp(uc)
 force=10*K*.0015*((e-1)+e*(u-uc))
 return min(force,500.),min(10*K*e*.004,200.)

@dataclass(frozen=True)
class GlobalPad:
 stiffness:float
 stroke:float
 base_mass:float
 face_mass:float
 base_inertia:object
 face_inertia:object
 rest_offset:tuple=(0.,0.,.0305)
 half_x:float=.0805
 half_y:float=.0605
 poisson:float=.2
 tau:float=.004

 def __post_init__(self):
  if not all(math.isfinite(x) and x>0 for x in [self.stiffness,self.stroke,self.base_mass,self.face_mass,self.half_x,self.half_y,self.tau]):
   raise ValueError('Positive finite material, mass and geometry required')
  for raw in [self.base_inertia,self.face_inertia]:
   i=np.asarray(raw,dtype=float)
   if i.shape!=(3,3) or not np.isfinite(i).all() or not np.allclose(i,i.T) or np.linalg.eigvalsh(i).min()<=0:raise ValueError('Physical inertia matrix required')

 def wrench(self,base,face,dt):
  """States are (worldposition,xyzw,linearvelocity,angularvelocity), measured from actual bodies.
  Force acts at face COM. Base reaction includes the lever torque, conserving total world momentum.
  Damping uses a local backward-Euler impulse; elastic forces retain exact static K.
  """
  if not math.isfinite(dt) or not 0<dt<=.0005:raise ValueError('Resolved positive material step required')
  bp,bq,bv,bw=[np.asarray(x,dtype=float) for x in base];fp,fq,fv,fw=[np.asarray(x,dtype=float) for x in face]
  if any(x.shape!=(3,) or not np.isfinite(x).all() for x in [bp,bv,bw,fp,fv,fw]):raise ValueError('Finite body state required')
  B=rotation_xyzw(bq);F=rotation_xyzw(fq);r=fp-bp;relative=B.T@r;delta=relative-np.asarray(self.rest_offset)
  young=self.stiffness/.34
  shear=young/(2*(1+self.poisson))*(4*self.half_x*self.half_y)/.026
  kt=np.array([shear,shear,self.stiffness]);kr=np.array([self.stiffness*self.half_y**2,self.stiffness*self.half_x**2,shear*(self.half_x**2+self.half_y**2)])
  coeff=np.array([kr[1]+kr[2]-kr[0],kr[0]+kr[2]-kr[1],kr[0]+kr[1]-kr[2]])*.5
  if coeff.min()<0:raise ValueError('Nonpassive orientation stiffness')
  elastic_torque=sum((coeff[i]*np.cross(F[:,i],B[:,i]) for i in range(3)),np.zeros(3))
  fel,dense_c=dense_elastic(self.stiffness,self.stroke,-delta[2])
  elastic=np.r_[-kt*delta,B.T@elastic_torque];elastic[2]+=fel
  velocity=np.r_[B.T@(fv-bv-np.cross(bw,r)),B.T@(fw-bw)]
  cbase=np.r_[kt*self.tau,kr*self.tau];cdiag=cbase.copy();cdiag[2]+=dense_c
  zero=np.zeros((3,3));eye=np.eye(3);Jb=np.block([[-eye,skew(r)],[zero,-eye]])
  inverse_b=np.block([[eye/self.base_mass,zero],[zero,B@np.linalg.inv(self.base_inertia)@B.T]])
  inverse_f=np.block([[eye/self.face_mass,zero],[zero,F@np.linalg.inv(self.face_inertia)@F.T]])
  frame=np.block([[B.T,zero],[zero,B.T]])
  effective=frame@(inverse_f+Jb@inverse_b@Jb.T)@frame.T
  damping=np.linalg.solve(np.eye(6)+dt*cdiag[:,None]*effective,-cdiag*velocity)
  damped_velocity=velocity+dt*effective@damping
  ordinary_damping=-cbase*damped_velocity
  dense_force=max(0.,min(500.,fel-dense_c*damped_velocity[2]))
  local=elastic+ordinary_damping;local[2]+=dense_force-fel
  force=B@local[:3];torque_face=B@local[3:];torque_base=-torque_face-np.cross(r,force)
  energy=.5*float((kt*delta)@delta)+float(sum(coeff[i]*(1-float(B[:,i]@F[:,i])) for i in range(3)))
  return {'face_force':force,'face_torque':torque_face,'base_force':-force,'base_torque':torque_base,'compression_m':float(-delta[2]),'relative_velocity':velocity,'elastic_energy_without_densification_j':energy,'dense_elastic_n':fel,'dense_total_n':dense_force,'damping_power_w':float(damping@velocity),'global_net_force':force-force,'global_net_torque':np.cross(r,force)+torque_face+torque_base}
