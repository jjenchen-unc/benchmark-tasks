"""Collidable rear-table camera support; calibrated optical pose is unchanged."""
import numpy as np
from yam_arena import station

def camera_mount(top):
    x,y,z=top.pos
    anchor=np.array([.80,0.,station.TABLE_TOP+.016]);high=np.array([.80,0.,z+.08])
    over=np.array([x,y,z+.08])
    housing=np.array(top.pos)+station.quat_xyzw_to_matrix(np.array(top.rot_xyzw))@np.array([0,0,.014])
    def rod(a,b,r):
        a,b=np.asarray(a),np.asarray(b)
        return ('capsule',(r,float(np.linalg.norm(b-a))),tuple((a+b)/2),station.aim(b-a))
    return {
      'camera_mount_foot':('box',(.16,.16,.016),(.80,0.,station.TABLE_TOP+.008),(0,0,0,1)),
      'camera_mount_mast':rod(anchor,high,.020),
      'camera_mount_arm':rod(high,over,.016),
      'camera_mount_drop':rod(over,housing,.009),
      'camera_housing':('box',(.05,.042,.022),tuple(housing),top.rot_xyzw),
    }
