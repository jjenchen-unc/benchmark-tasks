"""Disclosed state-aware whole-goal candidate. Task objects change only via robotAction.
This is an unvalidated reference attempt; IK or tracking failure aborts honestly.
No task poses, joints, forces, latches, scorer state or history are written.
"""
import math,numpy as np
from drawers_math import arr,world,A

def tracking_errors(actual,target):
 # Arm radians and normalized grip aperture are different coordinates. Closing
 # fingers may stop on a real grasp; retain the opening check for release.
 actual=np.asarray(actual,dtype=float);target=np.asarray(target,dtype=float)
 if actual.shape!=(14,) or target.shape!=(14,) or not np.isfinite(actual).all() or not np.isfinite(target).all():raise ValueError('nonfinite or invalid14D tracking state')
 arm_ids=[*range(6),*range(7,13)]
 arm_error=float(np.max(abs(actual[arm_ids]-target[arm_ids])))
 opening_error=max([abs(float(actual[i]-target[i])) for i in (6,13) if target[i]>=.95] or [0.])
 return arm_error,opening_error

class FullGoalReached(Exception):pass

def run(emb,obs,record):
 from inspect_robots.types import Action
 from yam_arena.ik import solve,tcp_pose,matrix_to_quat_xyzw,slerp_matrix
 from yam_arena.constants import TCP_OFFSET
 import drawers_kit as kit
 q=np.r_[kit.HOME,1.,kit.HOME,1.];bases=[];baseR=[]
 for side in ('left','right'):
  s=arr(emb._base.scene[side+'_robot'].data.root_link_pose_w)[0];bases.append(s[:3]);from drawers_math import rotation
  baseR.append(rotation(s[3:7]))
 yaw=3*math.pi/2;down=np.array([[math.cos(yaw),math.sin(yaw),0],[math.sin(yaw),-math.cos(yaw),0],[0,0,-1.]])
 def step(label,target):
  nonlocal obs,q
  result=emb.step(Action(data=target.copy()));obs=result.observation;q=target.copy();record(label,obs,result,target)
  if result.terminated:
   if result.termination_reason=='success':raise FullGoalReached()
   raise RuntimeError('Episode terminated: '+str(result.termination_reason))
  if kit.ENGINE.error:raise RuntimeError(kit.ENGINE.error)
  if kit.ENGINE.last and max(kit.ENGINE.last['wrist'])>60.:raise RuntimeError('Source maximum60N reference motion guard')
 def hold(label,n):
  for _ in range(n):step(label,q)
 smooth_orientation=False
 def move(label,targets,seconds,jaws=None):
  starts=[];orient=[]
  for arm in range(2):
   p,_=tcp_pose(np.array(obs.state['joint_pos'])[arm*7:arm*7+6]);starts.append(p)
   # Position feedback remains measured. Orientation interpolation starts from
   # the last accepted COMMAND, whose frame satisfies the native contract.
   # Loaded tracking tilt is not a permissible commanded orientation.
   _,Rcommand=tcp_pose(q[arm*7:arm*7+6]);orient.append(Rcommand)
  n=max(1,math.ceil(seconds*20));previous=q.copy();beginjaw=q[[6,13]].copy()
  for k in range(1,n+1):
   u=k/n;u=u*u*(3-2*u);target=q.copy()
   for arm,p in targets.items():
    dest=baseR[arm].T@(np.array(p)-bases[arm]);Rgoal=baseR[arm].T@down
    if smooth_orientation:Rgoal=slerp_matrix(orient[arm],Rgoal,u)
    qq,pe,re=solve(starts[arm]+(dest-starts[arm])*u,matrix_to_quat_xyzw(Rgoal),q_init=previous[arm*7:arm*7+6],iters=180,tol=.0001)
    if pe>.001 or re>.01:raise RuntimeError(f'{label}: IK failure arm={arm}, sample={k}/{n}, pe={pe}, re={re}')
    target[arm*7:arm*7+6]=qq
   if jaws is not None:
    for arm,jaw in jaws.items():target[arm*7+6]=beginjaw[arm]+(jaw-beginjaw[arm])*u
   step(label,target);previous=target
  actual=np.array(obs.state['joint_pos'],float)
  arm_error,opening_error=tracking_errors(actual,q)
  if arm_error>.20:raise RuntimeError(label+': arm joint tracking error >0.20rad')
  if opening_error>.20:raise RuntimeError(label+': opening aperture tracking error >0.20 normalized')
 def drawer():return kit.ENGINE.poses()['drawer_U']
 def sides(localx,zextra=0.):
  p,R,*_=drawer();return {i:p+R@np.array([localx,s*.126,.038+zextra]) for i,s in enumerate((-1,1))}
 def current_tcp(arm):
  p,R=tcp_pose(np.array(obs.state['joint_pos'])[arm*7:arm*7+6]);return bases[arm]+baseR[arm]@p
 # First open to the physical stop by its knob; right arm remains clear.
 p,R,*_=drawer();knob=p+R@np.array([-.15,0,.005]);move('knob_above',{0:knob+[0,0,.08]},3.,{0:1.});move('knob_grasp',{0:knob},2.)
 move('knob_close',{},1.,{0:0.});move('initial_pull',{0:knob+np.array([.108,0,0])},5.4)
 move('knob_release',{},1.,{0:1.});move('knob_clear',{0:current_tcp(0)+[0,0,.09]},2.)
 tabs=sides(-.068,.024)
 move('tabs_above',{i:p+[0,0,.07] for i,p in tabs.items()},3.,{0:.45,1:.45});move('tabs_grasp',tabs,2.);move('tabs_close',{},1.,{0:0.,1:0.})
 # Privileged measured planning reads only: native articulated paddle joints.
 # Do not infer release from arm motion or from a commanded jaw closure.
 from paddle_control import ReleasePlanner, measured_paddles, native_stop_passed
 planner=ReleasePlanner()
 press_origin=[current_tcp(i).copy() for i in (0,1)]
 press_frame=drawer()[1].copy()
 def paddle_command(label,xy,z):
  target=q.copy()
  for arm in (0,1):
   dest=press_origin[arm]+np.array([xy,0.,0.])+press_frame@np.array([0.,0.,z[arm]])
   _,rr=tcp_pose(q[arm*7:arm*7+6])
   qq,pe,re=solve(baseR[arm].T@(dest-bases[arm]),matrix_to_quat_xyzw(rr),q_init=q[arm*7:arm*7+6],iters=180,tol=.0001)
   if pe>.001 or re>.01:raise RuntimeError(label+': paddle IK failure')
   target[arm*7:arm*7+6]=q[arm*7:arm*7+6]+np.clip(qq-q[arm*7:arm*7+6],-.025,.025)
  step(label,target)
 # Bounded feedback, at most16mm vertical command, .25mm per control.
 # Do not start loaded horizontal pull before BOTH actual joints pass4mm.
 for j in range(80):
  measured=measured_paddles(emb)
  if planner.released(measured):break
  paddle_command('paddle_press_feedback_'+str(j),0.,planner.press(measured))
 else:raise RuntimeError('Both measured paddles did not release within80controls')
 # Keep the SAME grip and independently regulate each paddle while pulling.
 # Native disarm must be observed before any tab release/regrasp.
 passed=False
 for j in range(180):
  measured=measured_paddles(emb)
  p,R,*_=drawer()
  passed=native_stop_passed(p,R,kit.ENGINE)
  from drawers_math import native
  if passed and native(p)[0]<=.055:break
  xy,z=planner.pull(measured,passed)
  paddle_command('retained_paddle_pull_'+str(j),xy,z)
 else:
  # V10's actual 629 terminal is 0.189831mm short, with native disarm
  # and both paddles released. Preserve its180-command prefix exactly.
  # A finite5mm reserve covers that measured residual at the observed
  # 74.071mm/90mm displacement ratio; it is not a threshold relaxation.
  for tail in range(11):
   measured=measured_paddles(emb);p,R,*_=drawer()
   if not native_stop_passed(p,R,kit.ENGINE):raise RuntimeError('post-disarm tail requires actual native disarm')
   if not np.all(np.abs(measured)>.004):raise RuntimeError('post-disarm tail lost native paddle release')
   if native(p)[0]<=.055:break
   if tail==10:raise RuntimeError('measured regrasp clearance not reached within5mm post-disarm reserve')
   xy,z=planner.pull(measured,True)
   paddle_command('post_disarm_measured_tail_'+str(tail),xy,z)
 # Require current actual release/disarm even on the ordinary early exit.
 measured=measured_paddles(emb);p,R,*_=drawer()
 if not native_stop_passed(p,R,kit.ENGINE) or not np.all(np.abs(measured)>.004) or native(p)[0]>.055:raise RuntimeError('unsafe measured balance handoff')
 move('release_for_balance_regrasp',{},1.,{0:.4,1:.4});move('clear_for_regrasp',{i:current_tcp(i)+[0,0,.05] for i in (0,1)},1.5)
 mbox=kit.FIXTURE['parameters']['box_upper_weight']/9.81
 xgrip=float(np.clip((-.15*.105-.03*.135+mbox*.06)/(.25+.16+.16+.15+.03+.11+.25+mbox),-.032,.03))
 walls=sides(xgrip);move('balance_regrasp',walls,2.);move('balance_close',{},1.,{0:0.,1:0.})
 # Freeze the last commanded grasp frame once. Repeated actual-position rebasing
 # plus instantaneous down reorientation caused the measured paddle impact.
 extract_p=[];extract_R=[]
 for arm in (0,1):
  pp,rr=tcp_pose(q[arm*7:arm*7+6]);extract_p.append(pp);extract_R.append(rr)
 smooth_orientation=True
 for k in range(190):
  geoms=kit.ENGINE.geoms()
  from drawers_math import native,extent
  mx=max(native(g['p'])[0]+extent(g['kind'],np.array(g['size']),A@g['R'])[0] for g in geoms if g['asset']=='drawer_U')
  if mx<.115:break
  target=q.copy()
  for arm in (0,1):
   dest=extract_p[arm]+baseR[arm].T@np.array([.001*(k+1),0.,0.])
   qq,pe,re=solve(dest,matrix_to_quat_xyzw(extract_R[arm]),q_init=q[arm*7:arm*7+6],iters=180,tol=.0001)
   if pe>.001 or re>.01:raise RuntimeError('supported extraction IK failure')
   # Limit command slew while retaining the complete fixed-frame path.
   delta=qq-q[arm*7:arm*7+6]
   target[arm*7:arm*7+6]=q[arm*7:arm*7+6]+np.clip(delta,-.025,.025)
  step('supported_extract_'+str(k),target)
 else:raise RuntimeError('Extraction failed within190mm; no task teleport or limit relaxation')
 # Source goal permits any table location outside cabinets. A nominal SDK IK
 # study supports centre(.45,0), but physical payload/fixture clearance is open.
 p,R,*_=drawer();desired=np.array([.45,0.,.75+.035+.004]);delta=desired-p
 move('carry',{i:current_tcp(i)+delta+[0,0,.025] for i in (0,1)},max(2.,np.linalg.norm(delta)/.05))
 # Loaded command compliance moved the actual43914 drawer back toward the
 # cabinet during carry and set-down. Correct the measured NATIVE EXTENT,
 # retaining the last commanded grasp frame instead of rebasing its preload.
 # Only robot joint commands change; task state remains measured read-only.
 def retain_clearance(label):
  stable=0;travel=0.;previous=None
  for k in range(120):
   geoms=kit.ENGINE.geoms()
   mx=max(native(g['p'])[0]+extent(g['kind'],np.array(g['size']),A@g['R'])[0] for g in geoms if g['asset']=='drawer_U')
   if not np.isfinite(mx):raise RuntimeError('nonfinite native drawer extent')
   if kit.ENGINE.last and (not kit.ENGINE.last['contents'] or kit.ENGINE.last['drop']):raise RuntimeError('clearance correction lost safe contents')
   quiet=previous is not None and abs(mx-previous)<.0001
   stable=stable+1 if mx<=.105 and quiet else 0
   if stable>=10:return
   target=q.copy()
   if mx>.100:
    travel+=.001
    if travel>.05000001:raise RuntimeError('native extent correction exhausted50mm command reserve')
    for arm in (0,1):
     pp,rr=tcp_pose(q[arm*7:arm*7+6])
     dest=pp+baseR[arm].T@np.array([.001,0.,0.])
     qq,pe,re=solve(dest,matrix_to_quat_xyzw(rr),q_init=q[arm*7:arm*7+6],iters=180,tol=.0001)
     if pe>.001 or re>.01:raise RuntimeError('native extent correction IK failure')
     target[arm*7:arm*7+6]=q[arm*7:arm*7+6]+np.clip(qq-q[arm*7:arm*7+6],-.025,.025)
   previous=mx;step(label+'_'+str(k),target)
  raise RuntimeError('native extent correction did not settle within120controls')
 retain_clearance('carry_native_clearance')
 move('set_down',{i:current_tcp(i)-[0,0,.029] for i in (0,1)},1.5)
 retain_clearance('table_native_clearance')
 p,R,*_=drawer();pn=native(p)
 if abs(pn[2]-.835)>=.006 or pn[0]>=0. or (A@R)[2,2]<=math.cos(math.radians(3)):raise RuntimeError('measured native table placement not ready for release')
 move('release',{},1.,{0:.5,1:.5});move('retreat',{i:current_tcp(i)+[0,0,.08] for i in (0,1)},2.);hold('final_hands_off',25)
 status=kit.ENGINE.status()
 if status.get('hold_s',0.)<1.-1e-9:raise RuntimeError('actual native1s hands-off goal hold not reached')
 return status
