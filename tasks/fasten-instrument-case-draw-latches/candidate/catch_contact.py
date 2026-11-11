"""Measured native catch axis and individual YAM fingertip contact patch.
Plans robot targets only. No task state, dynamics, geometry or scoring writes.
"""
import numpy as np
from finite_geometry import pieces
from yam_arena.constants import PAD_TIP_PAST_TCP

def plan(controller,j):
 import kit
 name=f'A{j}_catch';bodies,joints,_=controller.state();meta=next(b for b in kit.FIXTURE['bodies'] if b['name']==name);g=next(g for g in meta['geoms'] if g['name']==name+'_tab')
 center,frame=bodies[name];C=bodies['case_A'][1];axis=frame@np.asarray(meta['joints'][0]['axis'],float);direction=-axis/np.linalg.norm(axis)
 if float(direction@C[:,0])<.999999:raise RuntimeError('Unsupported native catch axis')
 R=controller.rot(np.deg2rad(85.));jaw=.08
 q=controller.solve('left',center-PAD_TIP_PAST_TCP*R[:,2],R);part=pieces('left',q,jaw);patch=[]
 for finger in ('tip_left','tip_right'):
  vertices=np.vstack([h[0] for h in part[finger]]);depth=vertices@R[:,2]
  # Distal 3mm of the actual finger meshes; mean of both pad patches
  # centers the gripper around the tab rather than behind its pushing face.
  patch.append(vertices[depth>depth.max()-.003].mean(0))
 offset=np.mean(patch,axis=0)-center
 desired=center-C[:,1]*float(g['size'][1])*.5
 tool=desired-offset
 remaining=max(0.,float(joints[name])-float(meta['joints'][0]['range'][0]))
 return {'tool':tool,'R':R,'jaw':jaw,'direction':direction,'outward':-C[:,1], 'travel_m':remaining+.0005,'detail':{'catch':name,'native_q_m':float(joints[name]),'native_axis_world':axis.tolist(),'distal_patch_offset_world_m':offset.tolist(),'contact_patch_world_m':desired.tolist(),'drive_deflection_m':.0005}}

def return_lift(controller,R,jaw,top):
 """Retreat above the lid with 3mm private IK/lag margin plus original100um clearance.
 No task threshold or collision-check tolerance is changed.
 """
 from yam_arena import ik
 from yam_arena.station import BASES
 raw=None
 if hasattr(controller,'obs'):
  controller.q=np.asarray(controller.obs.state['joint_pos'],float).copy()
  import kit
  from kit import arr
  robot=kit.ENGINE.base.scene['left_robot'];raw={n:float(v) for n,v in zip(robot.joint_names,arr(robot.data.joint_pos)[0],strict=True)}
 pp=pieces('left',controller.q[:6],jaw,raw);bottom=min(float(h[0][:,2].min()) for f in ('tip_left','tip_right') for h in pp[f]);dz=max(0.,top+.0031-bottom)
 tcp,current_R=ik.tcp_pose(controller.q[:6],BASES['left'])
 return tcp+PAD_TIP_PAST_TCP*current_R[:,2]+[0,0,dz]
