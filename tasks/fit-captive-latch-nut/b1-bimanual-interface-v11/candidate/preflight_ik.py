"""Pure SDK URDF IK of early representative waypoints, not a clearance proof."""
import argparse, json, math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from yam_arena import ik
from yam_arena.station import BASES
from yam_arena.constants import READY

def run():
    f=json.loads((Path(__file__).parent/'fixture.json').read_text())
    R=np.asarray(f['rotation']); T=np.asarray(f['translation']); base=BASES['left']; rows=[]
    def check(label, native, yaw):
        p=R@native+T; Q=R@Rotation.from_euler('z',yaw).as_matrix()@np.diag([1.,-1.,-1.])
        best=None
        # Gripper is symmetric about its approach axis. Two equivalent pad orientations.
        for flip in (0,math.pi):
            rot=Q@Rotation.from_euler('z',flip).as_matrix()
            q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(rot),READY,base_pos=base,iters=220,damping=.015)
            v=dict(label=label,native_tcp_m=list(native),world_tcp_m=p.tolist(),hand_rotation_xyzw=ik.matrix_to_quat_xyzw(rot).tolist(),q=q.tolist(),position_error_m=pe,rotation_error_rad=re,reachable=pe<.003 and re<.05)
            if best is None or pe+.3*re<best['position_error_m']+.3*best['rotation_error_rad']:best=v
        rows.append(best)
    # Every rack candidate, so fitting the near-size alternatives remains executable scope.
    for i,name in enumerate(['w12','w1_2','w13','w14','wadj']):
        for dz in (0.,.08):check(name+('_grasp' if dz==0 else '_approach'),[0.,-.40+.05*i,.8375+dz],-math.pi/2)
    for deg in (-88.,-48.,-8.):
        a=math.radians(deg); jaw=np.array([.12,-.02,.894]); hand=jaw+Rotation.from_euler('z',a).apply([0,-.10,0])
        check('nut_turn_'+str(deg),hand,a)
    for deg in (0.,-45.,-90.):
        a=math.radians(deg);check('arm_'+str(deg),[.12-.08*math.sin(a),-.02+.08*math.cos(a),.9197],a)
    return dict(kind='actual SDK nominal URDF IK only; no physics/clearance/loaded feasibility claim',base_position_m=list(base),rows=rows,all_reachable=all(r['reachable'] for r in rows))

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();a.out.write_text(json.dumps(run(),indent=2)+'\n')
