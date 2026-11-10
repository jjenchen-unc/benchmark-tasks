"""Controller wrist orientation derived from actual link3/roof collision, no state writes."""
import numpy as np
from yam_arena.urdf import axis_angle_matrix
from yam_arena.demo import grip_rotation
BOX_ROTATION=axis_angle_matrix(np.array([0.,1.,0.]),np.deg2rad(-35))@axis_angle_matrix(np.array([1.,0.,0.]),np.deg2rad(-20))@grip_rotation(0.)
