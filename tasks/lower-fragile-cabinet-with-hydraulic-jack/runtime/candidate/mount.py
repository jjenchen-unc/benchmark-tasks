"""Private embodiment fixture: upright pi-yaw right arm on a 2.5mm steel mount plate."""
import numpy as np
from yam_arena import ik
BASE=np.array([.68,-.15,.75]);ROT=np.diag([-1.,-1.,1.])
def solve(p,r,q,iters=160):return ik.solve(ROT.T@(np.asarray(p)-BASE),ik.matrix_to_quat_xyzw(ROT.T@r),q_init=q,iters=iters)
def tcp(q):
 p,r=ik.tcp_pose(q);return ROT@p+BASE,ROT@r
