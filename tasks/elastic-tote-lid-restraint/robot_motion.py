"""State-aware script helper: every action is a genuine public YAM joint target.
No object/score writes. Planning uses disclosed simulator robot/fixture state.
URDF kinematics supplies IK; actual-link feedback compensates bounded tracking error.
Staged development helper, not yet feasibility evidence.
"""
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from yam_arena import YAM_URDF
from yam_arena.constants import ARM_JOINTS,TCP_OFFSET
from yam_arena.urdf import forward_kinematics,parse_joints
from yam_arena.runtime import link_poses,arm_q

class ArmMotion:
    def __init__(self,robot):
        self.robot=robot
        joints=parse_joints(YAM_URDF)
        self.lo=np.array([joints[n].lower for n in ARM_JOINTS])
        self.hi=np.array([joints[n].upper for n in ARM_JOINTS])
        self.last_fit=None
    def fk(self,q):
        T=forward_kinematics(YAM_URDF,dict(zip(ARM_JOINTS,q)))["gripper"]
        return T[:3,3]+T[:3,:3]@np.array(TCP_OFFSET),T[:3,:3]
    def solve(self,pos,R,seed=None,command_ref=None):
        poses=link_poses(self.robot)
        base=poses["base"]
        desired=base[:3,:3].T@(np.asarray(pos)-base[:3,3])
        desired_R=base[:3,:3].T@R
        q=np.clip(arm_q(self.robot),self.lo+1e-7,self.hi-1e-7)
        def residual(x):
            p,r=self.fk(x)
            return np.r_[p-desired,.12*Rotation.from_matrix(desired_R@r.T).as_rotvec()]
        fit=least_squares(residual,q if seed is None else np.clip(seed,self.lo+1e-7,self.hi-1e-7),bounds=(self.lo,self.hi),max_nfev=60,
                          ftol=1e-7,xtol=1e-7,gtol=1e-7)
        self.last_q=fit.x.copy()
        self.last_fit=dict(position_error_m=float(np.linalg.norm(residual(fit.x)[:3])),
                           rotation_error_rad=float(np.linalg.norm(residual(fit.x)[3:])/.12))
        if self.last_fit["position_error_m"]>.008 or self.last_fit["rotation_error_rad"]>.10:
            raise RuntimeError("IK cannot reach target: "+str(self.last_fit))
        # Bound the command change; caller repeats at 20 Hz and checks real state.
        reference=q if command_ref is None else np.asarray(command_ref)
        return np.clip(reference+np.clip(fit.x-reference,-.04,.04),self.lo,self.hi)


    def solve_hook(self,goal,offset,seed,command_ref=None):
        poses=link_poses(self.robot);base=poses["base"]
        desired=base[:3,:3].T@(np.asarray(goal)-base[:3,3])
        actual=np.clip(arm_q(self.robot),self.lo+1e-7,self.hi-1e-7)
        initial=np.clip(seed,self.lo+1e-7,self.hi-1e-7)
        def residual(q):
            p,R=self.fk(q)
            return np.r_[p+R@offset-desired,.0003*(q-initial)]
        fit=least_squares(residual,initial,bounds=(self.lo,self.hi),max_nfev=100,
                          ftol=1e-8,xtol=1e-8,gtol=1e-8)
        error=float(np.linalg.norm(residual(fit.x)[:3]))
        if error>.005:raise RuntimeError("Payload IK unreachable: "+str(error))
        self.last_q=fit.x.copy()
        reference=actual if command_ref is None else np.asarray(command_ref)
        return np.clip(reference+np.clip(fit.x-reference,-.04,.04),self.lo,self.hi)


    def payload_compensated_target(self,nominal,cord_force,attach_world,com_world,previous):
        # State-aware script only: encode load compensation into ordinary
        # position targets. No effort API or task-state writes.
        from yam_arena.constants import SHOULDER,WRIST
        poses=link_poses(self.robot);base_R=poses["base"][:3,:3]
        q=np.asarray(arm_q(self.robot),dtype=np.float64);p,R=self.fk(q)
        tcp=base_R@p+poses["base"][:3,3]
        Jp=np.zeros((3,6));Jr=np.zeros((3,6))
        for j in range(6):
            perturbed=q.copy();perturbed[j]+=1e-6
            pp,RR=self.fk(perturbed)
            Jp[:,j]=base_R@((pp-p)/1e-6)
            Jr[:,j]=base_R@(Rotation.from_matrix(RR@R.T).as_rotvec()/1e-6)
        gravity=np.array([0.,0.,-.05*9.81])
        force=np.asarray(cord_force)+gravity
        torque=np.cross(attach_world-tcp,cord_force)+np.cross(com_world-tcp,gravity)
        tau=-Jp.T@force-Jr.T@torque
        stiffness=np.array([SHOULDER["stiffness"]]*3+[WRIST["stiffness"]]*3)
        offset=tau/stiffness
        desired=np.clip(np.asarray(nominal)+offset,self.lo,self.hi)
        self.payload_compensation=dict(torque_Nm=tau.tolist(),target_offset_rad=offset.tolist())
        previous=np.asarray(previous)
        return np.clip(previous+np.clip(desired-previous,-.04,.04),self.lo,self.hi)
