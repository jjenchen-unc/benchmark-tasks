"""Scripted demo controller. It acts only through 14D robot actions at 20 Hz and decides only from
observable quantities: the published start layout and geometry, object poses and mechanism angles
(lid, levers, safety catches, hook links), the robot's own joint positions, and elapsed steps.
It does not read hook-attached flags, contact or force readings, spring or friction constants,
or the scorer. Every completion check is a pose comparison with the threshold logged in checks.jsonl.
"""
import json,math
import numpy as np
from pathlib import Path
from yam_arena import ik
from yam_arena.station import BASES
from yam_arena.constants import PAD_TIP_PAST_TCP,READY
from yam_arena.urdf import axis_angle_matrix,quat_xyzw_to_matrix
from posture import PARK,packed
from finite_geometry import scene,check,ARM_JOINTS,FINGER_JOINTS
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import kit

SEAT_GAP=.0025 # hook tip to keeper-bar centre distance that counts as seated (m)
SEAT_STEPS=12 # held for 0.6 s before the back-off test
class SeatWatch:
 def __init__(self,controller,n):self.c,self.n,self.streak,self.gap=controller,n,0,None
 def __call__(self):
  self.gap=self.c.tip_gap(self.n);self.streak=self.streak+1 if self.gap<SEAT_GAP else 0;return self.streak>=SEAT_STEPS
def rx(t):return np.array([[1,0,0],[0,np.cos(t),-np.sin(t)],[0,np.sin(t),np.cos(t)]])
class Reference:
 def __init__(self,emb,obs,out):
  self.emb,self.obs,self.out=emb,obs,Path(out);self.q=np.asarray(obs.state['joint_pos'],float).copy();self.phase='initial';self.count=0;self.events=[];self.journal=open(self.out/'robot-actions.jsonl','w',buffering=1);self.checklog=open(self.out/'checks.jsonl','w',buffering=1);self.ended=None;self.clear_route=[];self.audited_phase=None
 def state(self):
  bodies,joint,_=kit.ENGINE.states()
  from kit import arr,rotation
  origin=arr(kit.ENGINE.base.scene.env_origins)[0]
  for c in ('A','B'):
   d=kit.ENGINE.base.scene['inst_'+c].data
   bodies['inst_'+c]=(arr(d.root_pos_w)[0]-origin,rotation(arr(d.root_quat_w)[0]))
  return bodies,joint,None
 def local(self,p):
  b,_,_=self.state();pos,R=b['case_A'];return pos+R@np.asarray(p)
 def rot(self,tilt=0.,twist=0.):
  b,_,_=self.state();return b['case_A'][1]@rx(tilt)@np.diag([1.,-1.,-1.])@np.array([[np.cos(twist),-np.sin(twist),0],[np.sin(twist),np.cos(twist),0],[0,0,1]])
 def act(self,q):
  from inspect_robots.types import Action
  if self.count>=7964:raise RuntimeError('reference exhausted original run limit')
  from kit import arr
  raw={}
  for side in ('left','right'):
   robot=kit.ENGINE.base.scene[side+'_robot'];values=arr(robot.data.joint_pos)[0]
   raw[side]={n:float(v) for n,v in zip(robot.joint_names,values,strict=True)}
  self.validate_raw(raw)
  # Planner diagnostic, not native engine safety: phase-entry raw finite audit.
  # Per-action numeric/raw8 validation and native physics callbacks stay active.
  if self.audited_phase!=self.phase:
   self.clearance(self.q,raw_joint=raw);self.audited_phase=self.phase
  result=self.emb.step(Action(data=np.array(q,float)));self.count+=1;self.q=np.asarray(q,float).copy();self.obs=result.observation
  self.journal.write(json.dumps({'control':self.count,'physics_s':self.count*.05,'phase':self.phase,'q_command':self.q.tolist(),'raw_before':raw},allow_nan=False)+'\n')
  if result.terminated:
   self.ended=str(result.termination_reason)
   if self.ended!='success':raise RuntimeError('Episode terminated: '+self.ended)
  return result
 def validate_raw(self,raw):
  if set(raw)!={'left','right'}:raise ValueError('Expected both measured arms')
  for values in raw.values():
   if set(values)!=set(ARM_JOINTS+FINGER_JOINTS):raise ValueError('Expected actual raw8 joint names')
   if not all(isinstance(v,(int,float,np.integer,np.floating)) and not isinstance(v,(bool,np.bool_)) and np.isfinite(v) for v in values.values()):raise ValueError('Nonfinite or nonnumeric actual raw8')
 def route_entry_audit(self):
  # Current geometry/raw fingers checked before each newly planned segment.
  if not hasattr(self,'obs'):return # CPU evaluator has its own conditional audit.
  from kit import arr
  raw={}
  for side in ('left','right'):
   robot=kit.ENGINE.base.scene[side+'_robot']
   raw[side]={n:float(v) for n,v in zip(robot.joint_names,arr(robot.data.joint_pos)[0],strict=True)}
  self.validate_raw(raw);self.clearance(self.q,raw_joint=raw);self.audited_phase=self.phase
 def wait(self,steps,watch=None):
  for _ in range(steps):
   if self.ended:break
   self.act(self.q)
   if watch:watch()
 def check(self,name,measured,value,threshold,ok):
  row={'control':self.count,'check':name,'measured':measured,'value':value,'threshold':threshold,'pass':bool(ok)}
  line=json.dumps(row,default=float);self.checklog.write(line+'\n');print('CHECK '+line,flush=True)
  return ok
 def tip_gap(self,n):
  # Hook tip (published hook geometry, current hook pose) to keeper bar 0 centre (published lid geometry, current lid pose).
  meta={b['name']:b for b in kit.FIXTURE['bodies']};bodies=self.state()[0]
  hp,hr=bodies[n+'_hook'];lp,lR=bodies['lid_'+n[0]]
  tip=hp+hr@np.array(next(x['pos'] for x in meta[n+'_hook']['sites'] if x['name']==n+'_hook_tip'))
  bar=lp+lR@np.array(next(x['pos'] for x in meta['lid_'+n[0]]['sites'] if x['name']==n+'_pin0'))
  return float(np.linalg.norm(tip-bar))
 def tilt_deg(self,name):
  return math.degrees(math.acos(np.clip(self.state()[0][name][1][2,2],-1,1)))
 def solve(self,side,pos,R):
  index=0 if side=='left' else 7;seed=self.q[index:index+6];fits=[]
  def residual(q):
   p,r=ik.tcp_pose(q,BASES[side]);return np.r_[p-pos,.3*Rotation.from_matrix(R@r.T).as_rotvec()]
  for start in [seed,PARK[side],np.asarray(READY)]:
   fit=least_squares(residual,np.clip(start,ik.LOWER+1e-9,ik.UPPER-1e-9),bounds=(ik.LOWER,ik.UPPER),max_nfev=120,ftol=1e-9,xtol=1e-9,gtol=1e-9)
   e=residual(fit.x);pe=float(np.linalg.norm(e[:3]));re=float(np.linalg.norm(e[3:])/.3)
   if pe<.003 and re<.05:
    fits.append((float(np.linalg.norm(fit.x-seed)),fit.x))
    if fits[-1][0]<.08:break
  if not fits:raise RuntimeError('No bounded IK solution in '+self.phase)
  return min(fits,key=lambda f:f[0])[1]
 def clearance(self,q,raw_joint=None):
  bodies,_,_=self.state();objects=scene(kit.FIXTURE,bodies);targets=[]
  if self.phase in ('grasp_lid_lip','pull_lid_past_upright','release_lid'):targets=['lid_A/']
  elif any(self.phase.startswith(p) for p in ['lower_','close_','capture_','lock_']):
   n=self.phase.split('_',1)[1];targets=[n+'_lever/',n+'_hook/',n+'_link/'] if not self.phase.startswith('lock_') else [n+'_catch/']
  # Native task permits fingertip contact with the selected latch hardware.
  # Only these three keeper bars are added; physics and safety remain active.
  if self.phase.startswith('capture_'):
   selected=self.phase.split('_',1)[1];targets.extend('lid_'+selected[0]+'/'+selected+'_bar'+str(i) for i in range(3))
  allowed=[('left',ln,n) for ln in ('tip_left','tip_right') for n in objects if any(n.startswith(t) for t in targets)]
  report=check(q,objects,allowed,raw_joint=raw_joint)
  if report['unresolved']:raise RuntimeError(json.dumps({'phase':self.phase,'finite_geometry_reject':report['unresolved']},allow_nan=False))
  return report
 def move(self,side,p,R,jaw=.08,seconds=2.,until=None,preserve_command=False):
  self.route_entry_audit()
  # preserve_command=True keeps the last issued public 14D self.q (no reseed from lagging obs joint_pos).
  if hasattr(self,'obs') and not preserve_command:
   self.q=np.asarray(self.obs.state['joint_pos'],float).copy()
   if self.q.shape!=(14,) or not np.isfinite(self.q).all():raise RuntimeError('Invalid measured joint observation')
  index=0 if side=='left' else 7;tip,R0=ik.tcp_pose(self.q[index:index+6],BASES[side]);target=np.asarray(p)-PAD_TIP_PAST_TCP*R[:,2]
  from scipy.spatial.transform import Rotation,Slerp
  sl=Slerp([0,1],Rotation.from_matrix([R0,R]));angle=Rotation.from_matrix(R@R0.T).magnitude();n=max(round(seconds*20),math.ceil(np.linalg.norm(target-tip)/.006),math.ceil(angle/.04),1)
  route=[];start=self.q.copy()
  def append_fraction(f,previous,depth=0):
   pos=tip+(target-tip)*f;rot=sl([f]).as_matrix()[0];arm=self.solve(side,pos,rot);command=self.q.copy();delta=arm-command[index:index+6]
   if np.max(abs(delta))>.08:
    if depth>=8:raise RuntimeError(json.dumps({'phase':self.phase,'error':'IK branch discontinuity after bounded adaptive subdivision','fraction':f,'previous_fraction':previous,'q_before':self.q.tolist(),'q_fit':arm.tolist(),'delta_max_rad':float(np.max(abs(delta)))},allow_nan=False))
    midpoint=(previous+f)/2;append_fraction(midpoint,previous,depth+1);append_fraction(f,midpoint,depth+1);return
   command[index:index+6]=arm;command[index+6]=start[index+6]+f*(jaw-start[index+6]);self.clearance(command);route.append(command.copy());self.q=command
   if self.count+len(route)>7964:raise RuntimeError('Planned route exceeds original action budget')
  try:
   for k in range(n):append_fraction((k+1)/n,k/n)
  finally:self.q=start
  for command in route:
   self.act(command);self.clear_route.append(command.copy())
   if until and until():break
 # ---- PRIVATE UNVALIDATED catch-finish draft (robot control only; no physics/scorer/limit change) ----
 CATCH_RESERVE=420
 def catch_home_check(self,j,where,tol=.0015):
  c=float(self.state()[1][f'A{j}_catch'])
  self.check(f'A{j} catch home '+where,'catch slide position (m)',c,tol,c<=tol)
  if not c<=tol:raise RuntimeError(json.dumps({'phase':self.phase,'error':'native catch-home lost','where':where,'catch_m':c},allow_nan=False))
  return c
 def catch_settle(self,j,max_hold=40,need=5,tol=.0015,vtol=.001):
  # HOLD the last PUBLIC 14D target (unchanged jaw); no gain/travel/force/tolerance/budget change.
  if self.count+max_hold+self.CATCH_RESERVE>7964:raise RuntimeError('Catch settle would consume reserved final controls')
  target=[float(v) for v in self.q];streak=0;prev=float(self.state()[1][f'A{j}_catch'])
  for k in range(max_hold):
   self.act(target)
   c=float(self.state()[1][f'A{j}_catch']);v=(c-prev)/.05;prev=c
   streak=streak+1 if (c<=tol and abs(v)<=vtol) else 0
   if streak>=need:
    self.check(f'A{j} catch settled','catch slide position (m), speed over last step (m/s), 5 steps in a row',[c,v],[tol,vtol],True);return k+1
  raise RuntimeError(json.dumps({'phase':self.phase,'error':'catch not settled within hold limit','hold':max_hold},allow_nan=False))
 def catch_withdraw(self,j,cp,dist=.025):
  # From LAST PUBLIC FK pose, move along cp.outward with jaw closed; no reseed from lagging obs.
  tcp,R0=ik.tcp_pose(np.asarray(self.q[:6],float),BASES['left']);out=[float(x) for x in cp['outward']]
  p=[float(tcp[i])+PAD_TIP_PAST_TCP*float(R0[i][2])+dist*out[i] for i in range(3)]
  self.move('left',p,R0,jaw=float(self.q[6]),seconds=1.,preserve_command=True)
  return self.catch_home_check(j,'after_withdraw')
 def catch_jaw_open(self,jaw=.20,steps=20,side='left'):
  # Jaw-only: other 13 public targets bit-exact; original linear jaw trajectory; no IK/no reseed.
  self.route_entry_audit()
  index=0 if side=='left' else 7;start=[float(v) for v in self.q];route=[]
  for k in range(steps):
   c=list(start);c[index+6]=start[index+6]+(k+1)/steps*(jaw-start[index+6]);self.clearance(np.asarray(c,float));route.append(c)
  if self.count+len(route)+self.CATCH_RESERVE>7964:raise RuntimeError('Jaw open would consume reserved final controls')
  for c in route:
   self.act(c);self.clear_route.append(list(c))
  return len(route)
 def park(self):
  self.route_entry_audit()
  # Retrace genuine command waypoints, checking current loose task geometry.
  for command in reversed(self.clear_route):self.clearance(command);self.act(command)
  self.clear_route=[]
  end=packed();start=self.q.copy();n=max(1,math.ceil(float(np.max(abs(end-start)))/.04))
  for k in range(n):
   command=start+(end-start)*(k+1)/n;self.clearance(command);self.act(command)
 def sideways(self):return self.rot(math.radians(75.))
 def latchpoint(self,j,u,h,lat=0.):return self.local([(-.065,.065)[j]+lat,-.08-u,.18+h])
 def leverpoint(self,j,theta,u,h):
  c,s=np.cos(theta),np.sin(theta);return self.latchpoint(j,.010+c*u-s*h,-.024+s*u+c*h)
 def lever_contact(self,j,theta):
  from lever_contact import target
  return target(self,j,float(theta))
 def approach_latch(self,side,p,R,jaw=.08):
  self.route_entry_audit()
  if side!='left':raise RuntimeError('This verified staged approach is left-arm only')
  if hasattr(self,'obs'):self.q=np.asarray(self.obs.state['joint_pos'],float).copy()
  if self.q.shape!=(14,) or not np.isfinite(self.q).all():raise RuntimeError('Invalid measured joint observation')
  end=self.q.copy();end[:6]=self.solve(side,np.asarray(p)-PAD_TIP_PAST_TCP*R[:,2],R);end[6]=jaw
  start=self.q.copy();fold=end.copy();fold[0]=start[0];route=[]
  for target in (fold,end):
   steps=max(30,math.ceil(float(np.max(abs(target-start)))/.04))
   for k in range(steps):
    q=start+(target-start)*(k+1)/steps;self.clearance(q);route.append(q.copy())
   start=target.copy()
  if self.count+len(route)>7964:raise RuntimeError('Staged approach exceeds original action budget')
  for q in route:self.act(q);self.clear_route.append(q.copy())
 def park_direct(self):
  self.route_entry_audit()
  if self.phase=='close_A1':
   from finite_geometry import pieces
   if hasattr(self,'obs'):self.q=np.asarray(self.obs.state['joint_pos'],float).copy()
   raw=None
   if hasattr(self,'obs'):
    from kit import arr
    robot=kit.ENGINE.base.scene['left_robot'];raw={n:float(v) for n,v in zip(robot.joint_names,arr(robot.data.joint_pos)[0],strict=True)}
   bodies,_,_=self.state();objects=scene(kit.FIXTURE,bodies);top=max(float(h[0][:,2].max()) for n,h in objects.items() if n.startswith('lid_A/'));parts=pieces('left',self.q[:6],self.q[6],raw)
   lowest=min(float(h[0][:,2].min()) for finger in ('tip_left','tip_right') for h in parts[finger]);dz=max(0.,top+.0001-lowest)
   if dz>0.:
    tcp,R=ik.tcp_pose(self.q[:6],BASES['left']);self.move('left',tcp+PAD_TIP_PAST_TCP*R[:,2]+[0,0,dz],R,jaw=float(self.q[6]),seconds=1.)
  # First yaw away at fixed elevation, then fold outside the case footprint.
  start=self.q.copy();end=packed();yaw=start.copy();yaw[0]=end[0];route=[]
  for target in (yaw,end):
   steps=max(1,math.ceil(float(np.max(abs(target-start)))/.04))
   for k in range(steps):
    q=start+(target-start)*(k+1)/steps;self.clearance(q);route.append(q.copy())
   start=target.copy()
  if self.count+len(route)>7964:raise RuntimeError('Park exceeds original action budget')
  for q in route:self.act(q)
  self.clear_route=[]
 def capture_plan(self,n):
  # Geometry only: where the hook cap must go for the hook tip to reach keeper bar 0 (the lowest bar),
  # from the published hook/link/lid geometry and their current poses.
  meta={b['name']:b for b in kit.FIXTURE['bodies']};hook=meta[n+'_hook'];link=meta[n+'_link']
  bodies,_,_=self.state();pivot,lr=bodies[n+'_link'];hp,hr=bodies[n+'_hook'];lp,lR=bodies['lid_'+n[0]]
  slide=next(j for j in hook['joints'] if j['kind']=='slide');hinge=link['joints'][0]
  tiplocal=np.array(next(x['pos'] for x in hook['sites'] if x['name']==n+'_hook_tip'));cap=next(g for g in hook['geoms'] if g['name']==n+'_hook_cap')
  tip=hp+hr@tiplocal;center=hp+hr@np.array(cap['pos']);axis=lr@np.array(hinge['axis']);radial=tip-pivot
  target=lp+lR@np.array(next(x['pos'] for x in meta['lid_'+n[0]]['sites'] if x['name']==n+'_pin0'));desired=target-pivot
  a=radial-axis*np.dot(radial,axis);b=desired-axis*np.dot(desired,axis)
  delta=float(np.arctan2(np.dot(axis,np.cross(a,b)),np.dot(a,b)));Q=axis_angle_matrix(axis,delta)
  translation=hr@np.array(slide['axis'])*(np.linalg.norm(b)-np.linalg.norm(a));goal=pivot+Q@(center-pivot+translation)
  d=goal-center;distance=float(np.linalg.norm(d));direction=d/distance if distance>.001 else bodies['case_'+n[0]][1]@np.array([0.,1.,0.])
  w,x,y,z=cap['quat'];cr=hr@quat_xyzw_to_matrix(np.array([x,y,z,w]));extent=float(np.sum(abs(cr.T@direction)*cap['size']))
  return {'front':center-extent*direction,'direction':direction,'remaining_m':distance}
 def hook_front(self,n):
  p=self.capture_plan(n);return p['front'],p['direction']
 def run(self):
  try:
   self.phase='grasp_lid_lip'
   # Contact adaptation: grasp the existing44mm lip, pull it past upright,
   # then release at60deg and let the unchanged free hinge close under gravity.
   # Pitch the approach 30 degrees above horizontal: closing-axis X stays
   # aligned to the lip, while the forearm clears the open A0 lever tab.
   # Grip and passive hinge dynamics still require fresh actual validation.
   bodies,joint,_=self.state();theta0=float(joint['A_lid'])
   lipmeta=next(g for g in kit.FIXTURE['bodies'] if g['name']=='lid_A')
   local_lip=np.asarray(next(g['pos'] for g in lipmeta['geoms'] if g['name']=='A_lip'))
   lp,lr=bodies['lid_A'];lip=lp+lr@local_lip;Rg=self.rot(math.pi/2-math.pi/6)
   self.move('left',lip-.07*Rg[:,2],Rg,jaw=1.,seconds=3.)
   self.move('left',lip+.010*Rg[:,2],Rg,jaw=1.,seconds=3.)
   self.move('left',lip+.010*Rg[:,2],Rg,jaw=.40,seconds=1.)
   self.phase='pull_lid_past_upright'
   for th in np.linspace(theta0,math.radians(60),15):
    Rg=self.rot(math.pi/2-math.pi/6+theta0-th)
    lip=self.local([0,.08,.18])+self.state()[0]['case_A'][1]@rx(-th)@local_lip
    self.move('left',lip+.010*Rg[:,2],Rg,jaw=.40,seconds=.5)
   self.phase='release_lid';self.move('left',lip+.010*Rg[:,2]+[0,0,.12],Rg,jaw=1.,seconds=1.)
   self.move('left',lip-.06*Rg[:,2]+[0,0,.12],Rg,jaw=1.,seconds=2.);self.park();self.wait(160)
   lid=float(self.state()[1]['A_lid'])
   if not self.check('lid closed after release','lid angle (rad)',lid,math.radians(2),lid<=math.radians(2)):raise RuntimeError('Lid did not close')
   for j in range(2):
    n=f'A{j}';side='left';theta=float(self.state()[1][n+'_lever']);self.phase=f'lower_{n}'
    approach=self.leverpoint(j,theta,.013,-.05)+[0,0,.06]
    if j==1:self.approach_latch(side,approach,self.rot(math.pi/4))
    else:self.move(side,approach,self.rot(math.pi/4),seconds=3.)
    for th in np.linspace(theta,math.radians(89),12):
     p,Rc,detail=self.lever_contact(j,th);self.move(side,p,Rc,seconds=.6)
    self.phase=f'capture_{n}';Rpush=self.sideways();front,direction=self.hook_front(n)
    self.move(side,front-.008*direction+[0,0,.08],Rpush,jaw=.08,seconds=2.)
    self.move(side,front-.008*direction,Rpush,jaw=.08,seconds=1.5)
    for attempt in range(20):
     # Push the hook cap toward bar 0 for the full 1 s stroke; if its tip has been within SEAT_GAP for SEAT_STEPS steps,
     # test by backing the hand off 8 mm: a seated hook stays on the bar, a loose one springs back.
     seated=SeatWatch(self,n);front,direction=self.hook_front(n)
     self.move(side,front+min(.005,self.capture_plan(n)['remaining_m']+.001)*direction,Rpush,jaw=.08,seconds=1.,until=lambda:seated() and False);self.wait(4,watch=seated)
     if seated.streak<SEAT_STEPS:continue
     self.move(side,front-.008*direction,Rpush,jaw=.08,seconds=.5);self.wait(6);gap=self.tip_gap(n)
     if self.check(n+' hook stays on bar with hand backed off (attempt '+str(attempt+1)+')','hook tip to keeper bar 0 distance (m) after the tip was within '+str(SEAT_GAP)+' m for '+str(seated.streak)+' steps',gap,SEAT_GAP,gap<SEAT_GAP):break
    front,direction=self.hook_front(n)
    self.move(side,front-.008*direction,Rpush,jaw=.08,seconds=1.)
    self.move(side,front-.008*direction+[0,0,.08],Rpush,jaw=.08,seconds=2.)
    gap=self.tip_gap(n)
    if not self.check(n+' hook still on bar after hand withdrawn','hook tip to keeper bar 0 distance (m)',gap,SEAT_GAP,gap<SEAT_GAP):raise RuntimeError('Hook not seated: '+n)
    self.phase=f'close_{n}';theta=float(self.state()[1][n+'_lever'])
    for th in np.linspace(theta,-.08,40):
     p,Rc,detail=self.lever_contact(j,th);self.move(side,p,Rc,seconds=.4)
     if self.state()[1][n+'_lever']<=.05:break
    lever=float(self.state()[1][n+'_lever'])
    if not self.check(n+' lever flat','lever angle (rad)',lever,.05,lever<=.05):raise RuntimeError('Lever not flat: '+n)
    gap=self.tip_gap(n) # test: a hook that is not on the bar is pulled off it when the lever tensions the link
    if not self.check(n+' hook held on bar under lever tension','hook tip to keeper bar 0 distance (m)',gap,SEAT_GAP,gap<SEAT_GAP):raise RuntimeError('Hook pulled off: '+n)
    retreat=p-.04*np.array(detail['contact_direction']);self.move(side,retreat,Rc,seconds=1.);self.move(side,retreat+[0,0,.08],Rc,seconds=1.);self.park_direct()
   for j in range(2):
    self.phase=f'lock_A{j}'
    from catch_contact import plan as catch_plan
    cp=catch_plan(self,j);tool,R,jaw=cp['tool'],cp['R'],cp['jaw'];outward=cp['outward']
    # Yaw above the measured lid envelope, then descend outside the front wall.
    from finite_geometry import pieces
    approach=tool+.025*outward;lowq=self.q.copy();lowq[:6]=self.solve('left',approach-PAD_TIP_PAST_TCP*R[:,2],R);lowq[6]=jaw
    objects=scene(kit.FIXTURE,self.state()[0]);top=max(float(h[0][:,2].max()) for n,h in objects.items() if n.startswith(('lid_A/','lid_B/')))
    pp=pieces('left',lowq[:6],jaw);bottom=min(float(h[0][:,2].min()) for f in ('tip_left','tip_right') for h in pp[f]);dz=max(0.,top+.0001-bottom)
    self.approach_latch('left',approach+[0,0,dz],R,jaw=jaw)
    self.move('left',approach,R,jaw=jaw,seconds=2.)
    self.move('left',tool,R,jaw=jaw,seconds=2.)
    end=tool+cp['travel_m']*cp['direction']
    self.move('left',end,R,jaw=jaw,seconds=2.,until=lambda:float(self.state()[1][f'A{j}_catch'])<=.0015)
    # PRIVATE UNVALIDATED DRAFT: settle on last public target, withdraw outward with jaw closed, jaw-only open.
    self.catch_settle(j)
    self.catch_home_check(j,'after_settle')
    self.catch_withdraw(j,cp)
    self.catch_jaw_open(.20,20)
    from catch_contact import return_lift
    self.move('left',return_lift(self,R,.20,top),R,jaw=.20,seconds=2.)
    self.park_direct()
    self.catch_home_check(j,'after_park')
   self.phase='hands_off_20s';self.park()
   _,joint,_=self.state();ok=self.check('lid closed before hands-off','lid angle (rad)',float(joint['A_lid']),math.radians(2),joint['A_lid']<=math.radians(2))
   for j in range(2):
    n=f'A{j}';gap=self.tip_gap(n)
    ok&=self.check(n+' before hands-off','lever angle (rad), catch slide (m), hook tip to bar (m)',[float(joint[n+'_lever']),float(joint[n+'_catch']),gap],[.05,.0015,SEAT_GAP],joint[n+'_lever']<=.05 and joint[n+'_catch']<=.0015 and gap<SEAT_GAP)
   tilts=[self.tilt_deg(x) for x in ('inst_A','inst_B','case_A','case_B')]
   ok&=self.check('upright before hands-off','instrument A/B and case A/B tilt (deg)',tilts,[20,20,15,15],max(tilts[:2])<20 and max(tilts[2:])<15)
   if not ok:raise RuntimeError('Pose checks before hands-off failed')
   self.wait(420)
   return {'completed_sequence':True,'ended':self.ended,'actions':self.count}
  except Exception as e:
   result={'completed_sequence':False,'failed_phase':self.phase,'error':str(e),'actions':self.count};(self.out/'reference-failure.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');return result
  finally:self.journal.close();self.checklog.close()
