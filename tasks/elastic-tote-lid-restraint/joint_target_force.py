"""Disclosed script control math. Returns ordinary position targets; no simulation APIs."""
import numpy as np

def position_target_for_force(fk, q, contact_from_tcp, world_base_R, nominal, force_world, stiffness, lower, upper, previous, max_step=.04):
    q=np.asarray(q,dtype=np.float64);r=np.asarray(contact_from_tcp,dtype=np.float64)
    B=np.asarray(world_base_R,dtype=np.float64);nominal=np.asarray(nominal,dtype=np.float64)
    force=np.asarray(force_world,dtype=np.float64);K=np.asarray(stiffness,dtype=np.float64)
    lo=np.asarray(lower,dtype=np.float64);hi=np.asarray(upper,dtype=np.float64);prev=np.asarray(previous,dtype=np.float64)
    if (q.shape!=(6,) or r.shape!=(3,) or B.shape!=(3,3) or force.shape!=(3,)
        or any(x.shape!=(6,) for x in (nominal,K,lo,hi,prev))
        or not all(np.isfinite(x).all() for x in (q,r,B,nominal,force,K,lo,hi,prev))
        or not np.isfinite(max_step) or max_step<=0 or np.any(K<=0) or np.any(lo>=hi)):
        raise ValueError('Invalid force-to-position control input')
    p,R=fk(q);contact=np.asarray(p)+np.asarray(R)@r
    J=np.zeros((3,6),dtype=np.float64)
    for j in range(6):
        perturbed=q.copy();perturbed[j]+=1e-6
        pp,RR=fk(perturbed)
        J[:,j]=B@((np.asarray(pp)+np.asarray(RR)@r-contact)/1e-6)
    # Positive J.T F commands robot force F on the environment. Own-arm G is SDK-handled.
    tau=J.T@force;offset=tau/K
    if not np.isfinite(offset).all():raise ValueError('Nonfinite contact Jacobian/target')
    desired=np.clip(nominal+offset,lo,hi)
    target=np.clip(prev+np.clip(desired-prev,-max_step,max_step),lo,hi)
    detail=dict(requested_contact_force_N=force.tolist(),torque_Nm=tau.tolist(),target_bias_rad=offset.tolist(),
                target_limit_clipped=bool(np.any(np.abs(desired-(nominal+offset))>1e-12)),
                rate_limited=bool(np.any(np.abs(desired-prev)>max_step+1e-12)))
    return target,detail
