"""Disclosed state-aware robot reference. Only emb.step(Action(14D)) actuates.
No evaluator loads, object/joint/force/scorer writes. Hidden friction is used
ONLY for this disclosed feasibility planner, never supplied to a model policy.
"""
# Explicit task-local bootstrap also supports the chosen isolated interpreter.
import sys as _bootstrap_sys
from pathlib import Path as _BootstrapPath
_bootstrap_sys.path.insert(0,str(_BootstrapPath(__file__).resolve().parent))
from c1_launch import bootstrap
bootstrap(__file__)
import argparse,json,hashlib,os,shutil,sys,time
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
from c1_contract import *
from c1_json import dumps

class Trial:
 def __init__(self,emb,out,seed,instruction=None):
  from inspect_robots import Scene
  from yam_arena.constants import TCP_OFFSET
  from c1_kit import arr
  self.emb=emb;self.out=out;self.obs=emb.reset(Scene(id='C1-state-aware-reference',instruction=instruction or 'Select a liner, wet it first if required, place it on the20degree rack and leave a glass on it hands-off for20s with at most5mm drift.'),seed=seed)
  self.robot=[emb._base.scene[s+'_robot'] for s in ['left','right']];origin=arr(emb._base.scene.env_origins)[0]
  self.basepos=[arr(x.data.body_pos_w)[0,0]-origin for x in self.robot];self.baseR=[rotation_xyzw(arr(x.data.body_quat_w)[0,0]) for x in self.robot];self.q=np.asarray(self.obs.state['joint_pos'],float);self.command=self.q.copy();self.actions=[];self.rows=[];self.rng=np.random.default_rng(seed)
  self.journal=open(out/'robot-journal.jsonl','w',buffering=1);self.phase='initial';self.fk=[];self.parking_done=False
  from yam_arena import ik
  for a in range(2):
   p,R=self.tcp(a);pp,RR=ik.tcp_pose(self.q[a*7:a*7+6]);pp=self.basepos[a]+self.baseR[a]@pp;RR=self.baseR[a]@RR;pe=float(np.linalg.norm(p-pp));re=float(np.linalg.norm(ik._rot_error(R,RR)))
   self.fk.append({'arm':a,'basepos':self.basepos[a].tolist(),'position_error_m':pe,'rotation_error_rad':re})
   if pe>.002 or re>.01:raise RuntimeError('Actual/URDF FK mismatch')
 def tcp(self,a):
  from c1_kit import arr
  from yam_arena.constants import TCP_OFFSET
  r=self.robot[a];j=r.body_names.index('gripper');R=rotation_xyzw(arr(r.data.body_quat_w)[0,j]);origin=arr(self.emb._base.scene.env_origins)[0]
  return arr(r.data.body_pos_w)[0,j]-origin+R@np.array(TCP_OFFSET),R
 def solve(self,a,p,R):
  from scipy.optimize import least_squares
  from yam_arena import ik
  sl=slice(a*7,a*7+6)
  def fun(q):
   pp,RR=ik.tcp_pose(q);pp=self.basepos[a]+self.baseR[a]@pp;RR=self.baseR[a]@RR;return np.r_[pp-p,.25*ik._rot_error(R,RR)]
  best=None
  for seed in [self.command[sl].copy(),self.q[sl].copy()]+[self.rng.uniform(ik.LOWER,ik.UPPER) for _ in range(5)]:
   fit=least_squares(fun,np.clip(seed,ik.LOWER+1e-8,ik.UPPER-1e-8),bounds=(ik.LOWER,ik.UPPER),max_nfev=150);pp,RR=ik.tcp_pose(fit.x);pp=self.basepos[a]+self.baseR[a]@pp;RR=self.baseR[a]@RR;pe=np.linalg.norm(pp-p);re=np.linalg.norm(ik._rot_error(R,RR));v=pe+.25*re
   if best is None or v<best[0]:best=(v,fit.x,float(pe),float(re))
   if pe<.0003 and re<.005:break
  if best[2]>.003 or best[3]>.02:raise RuntimeError('Unreachable path '+str((a,p.tolist(),best[2],best[3])))
  return best[1]
 def tick(self,phase):
  from inspect_robots.types import Action
  import c1_kit
  if len(self.actions)>=BUDGET:raise RuntimeError('2268 control budget exhausted')
  if self.actions and np.max(np.abs(self.command[np.r_[0:6,7:13]]-np.asarray(self.actions[-1])[np.r_[0:6,7:13]]))>.0450000001:raise RuntimeError('Original .045rad/action increment exceeded')
  status_before=c1_kit.ENGINE.status();dumps(dict(status=status_before,action14=self.command)) # strict numeric validation before Action
  if status_before.get('measurement_error') or status_before.get('failure') or status_before.get('force_guard_frozen'):raise RuntimeError('Original observer/guard unavailable before Action')
  self.actions.append(self.command.tolist());res=self.emb.step(Action(data=self.command.copy()));self.obs=res.observation;self.q=np.asarray(self.obs.state['joint_pos'],float)
  from route_telemetry import snapshot
  telemetry=snapshot(self)
  row={'telemetry':telemetry,'control':len(self.actions),'phase':phase,'action14':self.command.tolist(),'measured_joint14':self.q.tolist(),'status':c1_kit.ENGINE.status()};self.journal.write(dumps(row)+'\n')
  if res.terminated and res.termination_reason!='time_limit':raise RuntimeError('Physical termination '+str(res.termination_reason))
  return res
 def move(self,a,p=None,g=None,seconds=1.5,R=None,label='move'):
  from scipy.spatial.transform import Rotation,Slerp
  from yam_arena import ik
  import c1_kit
  p0,R0=self.tcp(a);p=p0 if p is None else np.asarray(p,float);R=R0 if R is None else np.asarray(R,float)
  # Source tool point is25mm above the pad bottom; YAM TCP is13mm above
  # it. Preserve the physical workspace using that explicit12mm frame shift.
  virtual=native(p+np.array([0.,0.,.012]))
  if np.any(virtual<WORKSPACE[:,0]-1e-6) or np.any(virtual>WORKSPACE[:,1]+1e-6):raise RuntimeError('Source-equivalent tool goal outside workspace '+str(virtual))
  q0=self.command[a*7:a*7+6].copy();q1=self.solve(a,p,R);g0=self.command[a*7+6];g1=g0 if g is None else g
  if self.parking_done and ((a==1 and label in ('glass_hover','glass_rack_hover')) or (a==0 and label=='left_clear_for_rejoin')):
   # Move through joint space to the checked endpoint. Right-arm Cartesian
   # slerp from the tilted park crosses the joint6 lower-limit boundary.
   # Start geometry from measured joints; cap every command increment.
   from c1_kit import arr
   measured=np.asarray(arr(self.robot[a].data.joint_pos)[0,:6],float)
   n=max(1,round(seconds*20),int(np.ceil(1.5*np.max(abs(q1-measured))/.045)))
   for j in range(n):
    t=(j+1)/n;u=t*t*(3-2*t);desired=measured+u*(q1-measured)
    sl=slice(a*7,a*7+6);self.command[sl]=np.clip(desired,self.command[sl]-.045,self.command[sl]+.045)
    self.command[a*7+6]=g0+u*(g1-g0);self.tick(label)
    if c1_kit.ENGINE.freeze_arms:raise RuntimeError('Source30N/20ms wrist or50N/20ms nonfinger motion guard: '+label)
   pe=np.linalg.norm(self.tcp(a)[0]-p)
   self.rows.append({'phase':label,'controls':len(self.actions),'target_m':p.tolist(),'actual_error_m':float(pe),'route':'measured joint-space endpoint rejoin'})
   if pe>.012:raise RuntimeError('Actual TCP tracking error exceeds12mm '+label+' '+str(pe))
   return
  n=max(1,round(seconds*20),int(np.ceil(np.max(abs(q1-q0))/.045)));slerp=Slerp([0,1],Rotation.from_matrix([R0,R]))
  for j in range(n):
   t=(j+1)/n;u=t*t*(3-2*t);pp=p0+u*(p-p0);RR=slerp(u).as_matrix();q=self.solve(a,pp,RR)
   sl=slice(a*7,a*7+6);self.command[a*7+6]=g0+u*(g1-g0)
   if self.parking_done and (label=='rack_liner_hover' or a==1):
    # Continue the SAME original Cartesian target after holding measured169.
    # Temporal subdivision prevents accumulated clipping lag; never exceed
    # original .045rad/action or change the target/20Hz/total2268 budget.
    start_q=self.command[sl].copy();subdivisions=max(1,int(np.ceil(np.max(abs(q-start_q))/.045)))
    for sub in range(subdivisions):
     self.command[sl]=start_q+(sub+1)/subdivisions*(q-start_q);self.tick(label)
     if c1_kit.ENGINE.freeze_arms:raise RuntimeError('Source30N/20ms wrist or50N/20ms nonfinger motion guard: '+label)
   else:
    self.command[sl]=np.clip(q,self.command[sl]-.045,self.command[sl]+.045);self.tick(label)
    if c1_kit.ENGINE.freeze_arms:raise RuntimeError('Source30N/20ms wrist or50N/20ms nonfinger motion guard: '+label)
   if label=='rack_liner_hover' and len(self.actions)==169 and not self.parking_done:
    from parking_controller import park_right
    park_right(self)
  pe=np.linalg.norm(self.tcp(a)[0]-p)
  self.rows.append({'phase':label,'controls':len(self.actions),'target_m':p.tolist(),'actual_error_m':float(pe)})
  # A missed grasp height may not silently become a state-aware teleport.
  if pe>.012:raise RuntimeError('Actual TCP tracking error exceeds12mm '+label+' '+str(pe))
 def wait(self,seconds,label):
  for _ in range(round(seconds*20)):self.tick(label)
 def pose(self,name):
  import c1_kit
  return c1_kit.ENGINE.state(name)
 def grasp_liner(self,i,slot,wet):
  from yam_arena.demo import grip_rotation
  from yam_arena.constants import PAD_TIP_PAST_TCP
  # World jaw axis follows the native70mm liner width (station Y).
  R=grip_rotation(np.pi);p,_=self.pose(f'liner_{i+1}');xy=p[:2];hover=TABLE_Z+.22
  # Prefer source left hand, but either unmodified YAM can carry an object.
  # Reachability is checked before motion. No object state is changed here.
  a=0
  try:self.solve(a,np.r_[xy,TABLE_Z+.002+PAD_TIP_PAST_TCP],R);self.solve(a,np.r_[slot_point(slot)[:2],hover],grip_rotation(0.))
  except RuntimeError:a=1
  self.move(a,np.r_[xy,hover],1.,2.,R,'liner_hover');self.move(a,np.r_[xy,TABLE_Z+.002+PAD_TIP_PAST_TCP],1.,2.,R,'liner_descend')
  #18N/pad at70mm actual gap with SDK5000N/m jaw stiffness.
  self.move(a,g=.669,seconds=1.,label='liner_light_pinch');self.move(a,np.r_[xy,hover],.669,2.5,R,'liner_lift')
  if self.pose(f'liner_{i+1}')[0][2]<TABLE_Z+.07:raise RuntimeError('Liner did not physically lift')
  if wet:
   pc=PAN_C[:2];self.move(a,np.r_[pc,hover],.669,2.,R,'pan_hover');self.move(a,np.r_[pc,TABLE_Z+.004+.007+PAD_TIP_PAST_TCP],.669,2.,R,'physical_dip');self.wait(.25,'dip_wait');self.move(a,np.r_[pc,hover],.669,2.,R,'dip_lift')
   import c1_kit
   if not c1_kit.ENGINE.wet[i]:raise RuntimeError('Actual liner centre never entered water')
  R=grip_rotation(0.)
  s=slot_point(slot);tip=np.r_[s[:2],s[2]+.075*np.tan(ANGLE)+.003+.002+PAD_TIP_PAST_TCP]
  self.move(a,np.r_[s[:2],hover],.669,2.,R,'rack_liner_hover');self.move(a,tip,.669,2.,R,'rack_liner_lower');self.move(a,g=1.,seconds=.8,label='liner_release');self.wait(.4,'liner_settle');self.move(a,tip+[0,0,.08],1.,1.5,R,'liner_retreat')
  p,_=self.pose(f'liner_{i+1}')
  if np.linalg.norm(p[:2]-s[:2])>.06:raise RuntimeError('Released liner missed source rack slot')
  # Retreat toward the base side before the other arm enters the rack.
  self.move(a,PARK_POINTS[a],1.,2.,R,'park_after_liner')
 def place_cup(self,c,slot):
  from yam_arena.demo import grip_rotation
  from yam_arena.constants import PAD_TIP_PAST_TCP
  if self.parking_done:
   # Liner is released before clearing the left arm from the right rejoin sweep.
   self.move(0,[.25,.20,1.0],1.,2.,grip_rotation(np.pi),'left_clear_for_rejoin')
  R=grip_rotation(np.pi);a=1;p,_=self.pose(f'cup_{c+1}');hover=TABLE_Z+.22
  self.move(a,np.r_[p[:2],hover],1.,2.,R,'glass_hover');self.move(a,p+[0,0,.040-.012],1.,1.5,R,'glass_descend');self.move(a,g=0.,seconds=1.,label='glass_close');self.move(a,np.r_[p[:2],hover],0.,2.,R,'glass_lift')
  if self.pose(f'cup_{c+1}')[0][2]<TABLE_Z+.09:raise RuntimeError('Glass did not physically lift')
  R=grip_rotation(0.)
  s=slot_point(slot)+N_RACK*.006;centre=np.r_[s[:2],s[2]+.037*np.tan(ANGLE)+.0015+.035];tip=centre+[0,0,.040-.012]
  self.move(a,np.r_[centre[:2],hover],0.,2.,R,'glass_rack_hover');self.move(a,tip,0.,2.,R,'glass_lower');self.move(a,g=1.,seconds=1.5,label='glass_release');self.move(a,tip+[0,0,.02],1.,.8,R,'glass_clear');self.move(a,tip+[-.10,-.08,.06],1.,1.5,R,'glass_retreat')
  self.move(a,PARK_POINTS[1],1.,2.,R,'right_park')
 def full_goal(self,target,seed):
  import c1_kit
  n,wet=TARGETS[target];params=parameters(seed,transfer=target=='C1-T4');chosen=[int(x) for x in np.argsort(params['mu_wet' if wet else 'mu_dry'])[::-1][:n]];slots=[1] if n==1 else [0,1]
  # Source native reference chooses hidden maxima; disclose that fact.
  self.plan={'liners':[i+1 for i in chosen],'slots':slots,'wet':wet,'planner':'simstate+hidden friction, feasibility only'}
  for i,s in zip(chosen,slots):self.grasp_liner(i,s,wet)
  for c,s in enumerate(slots):self.place_cup(c,s)
  self.wait(21.5,'hands_off_source_hold')
  return c1_kit.ENGINE.status()
 def close(self):
  self.journal.close();(self.out/'actions.json').write_text(dumps(self.actions)+'\n')

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--target',choices=list(TARGETS),default='C1-T1');ap.add_argument('--seed',type=int,default=42);ap.add_argument('--public-case',action='store_true');a=ap.parse_args();__import__('c1_launch').output_directory(a.out)
 from c1_public import INSTRUCTION
 if a.public_case and a.target!='C1-T1':raise ValueError('Public case requires C1-T1')
 physical_seed=42 if a.public_case else a.seed
 os.environ['C1_TARGET']=a.target;os.environ['C1_TRACE']=str(a.out/'measurements.jsonl')
 pass
 from yam_arena.inspect_embodiment import yam_arena_embodiment
 emb=yam_arena_embodiment(task=str(HERE/('public_entry.py' if a.public_case else 'task.py')),device='cpu',object_jitter=0,control_hz=20,cam_width=320,cam_height=176);run=None;start=time.monotonic();report={'kind':'genuine14D20Hz state-aware reference attempt','model_calls':0,'target':a.target,'seed':a.seed,'trial_seed':a.seed,'physical_seed':physical_seed,'public_case':a.public_case,'instruction':INSTRUCTION if a.public_case else None,'full_goal_demonstrated':False,'submission_ready':False}
 try:
  run=Trial(emb,a.out,a.seed,INSTRUCTION if a.public_case else None);report['robot_frame_preflight']=run.fk;from route_telemetry import snapshot
  initial=snapshot(run);(a.out/'first-action-preflight.json').write_text(dumps(initial,indent=2)+'\n');report['initial_status_JSON_preflight']=True
  from PIL import Image
  for name,frame in run.obs.images.items():Image.fromarray(frame).save(a.out/('initial-'+name+'.png'))
  status=run.full_goal(a.target,physical_seed);report.update(status=status,plan=run.plan,stages=run.rows,controls=len(run.actions),physical_raw_fullscore=status['raw_score']==1.,full_goal_demonstrated=status['score']==1.)
  for name,frame in run.obs.images.items():Image.fromarray(frame).save(a.out/('final-'+name+'.png'))
 except BaseException as e:
  report['error']=type(e).__name__+': '+str(e)
  if run:
   import c1_kit
   report.update(status=c1_kit.ENGINE.status(),stages=run.rows,controls=len(run.actions))
  (a.out/'error.json').write_text(dumps(report,indent=2)+'\n');raise
 finally:
  report['wall_s']=time.monotonic()-start
  if run:run.close()
  from c1_evidence import measured_goal_evidence
  report.update(measured_goal_evidence(report,run,a.out))
  (a.out/'result.json').write_text(dumps(report,indent=2)+'\n');emb.close()
if __name__=='__main__':main()
