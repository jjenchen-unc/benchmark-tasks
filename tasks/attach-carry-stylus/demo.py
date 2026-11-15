"""State-feedback scripted reference. All task motion uses public YAM joint actions."""
import argparse,json,math,os,sys,traceback
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
from yam_arena import ik,station
from yam_arena.constants import READY,TCP_OFFSET,FINGER_OPEN
from yam_arena.demo import grip_rotation
import runtime
from runtime import arr
TOP=.7475

class Trial:
 def __init__(self,out,record=True):
  from inspect_robots import Scene
  from yam_arena.inspect_embodiment import yam_arena_embodiment
  self.out=Path(out);self.out.mkdir(parents=True,exist_ok=True);self.record=record
  self.emb=yam_arena_embodiment(task=str(Path(__file__).with_name('task.py')),object_jitter=0,
    control_hz=20,cam_width=480,cam_height=264,device='cpu')
  obs=self.emb.reset(Scene(id='demo',instruction='Attach stylus and lift tablet.'))
  self.frames=[];self.capture(obs);self.rows=[];self.arms={};self.ik_errors=[];self.phase='reset'
  for side in ['left','right']:
   q=np.array(READY);p,r=ik.tcp_pose(q,station.BASES[side])
   self.arms[side]=dict(q=q,p=p,r=r,f=FINGER_OPEN,goal=np.r_[q,[FINGER_OPEN]*2],correction=np.zeros(3))
  self.trace=(self.out/'trace.jsonl').open('w',buffering=1)
  (self.out/'runtime.json').write_text(json.dumps(dict(physics_dt=self.emb._base.physics_dt,
     step_dt=self.emb._base.step_dt,paths=runtime.ACTIVE.paths,filters=runtime.ACTIVE.filters),indent=2))
 def physical_controls(self):
  """Evaluator-only counterexamples, strictly after saving the full robot demo."""
  rt=runtime.ACTIVE
  assert rt.scorer.score==1., 'Controls require actual full-score robot state'
  self.record=False;self.frames=[]
  self.demonstration_rows=self.rows;self.rows=[]
  self.evaluator_mode=True
  self.trace=(self.out/'evaluator-controls.jsonl').open('w',buffering=1)
  results=[]
  # Removing the force after real robot attachment must invalidate the score.
  rt.scale=0.
  for _ in range(100):self.tick('evaluator_remove_magnets')
  results.append(dict(name='remove_magnets_after_full',score=rt.scorer.score,
    pen_tablet_contact=rt.latest['pen_tablet_contact'],passed=rt.scorer.score==0. and not rt.latest['seated']))
  for name,offset,rotation in [('screen',[0,0,.0076],R.from_euler('x',90,degrees=True).as_matrix()),
                              ('wrong_face',[0,.07185,0],R.from_euler('x',180,degrees=True).as_matrix())]:
   rt.scale=1.
   ip,iq,ri=rt.pose('ipad');pos=ip+ri@np.array(offset);q=R.from_matrix(ri@rotation).as_quat()
   # A declared evaluator reset, never executed by the demonstration/policy.
   rt.views['pencil'].set_transforms(np.array([[*pos,*q]],np.float32),np.array([0],np.int32))
   rt.views['pencil'].set_velocities(np.zeros((1,6),np.float32),np.array([0],np.int32))
   seen=[]
   for _ in range(30):self.tick('evaluator_'+name);seen.append(rt.scorer.score)
   results.append(dict(name=name,scores=seen,passed=all(x==0 for x in seen)))
  self.trace.close()
  self.rows=self.demonstration_rows
  summary=dict(passed=all(x['passed'] for x in results),checks=results,
               scope='Evaluator intervention after a separately saved robot demo; not demonstration actions')
  (self.out/'physical-controls.json').write_text(json.dumps(summary,indent=2))
  assert summary['passed'],summary
 def reset_check(self):
  from inspect_robots import Scene
  before={n:runtime.ACTIVE.pose(n)[0].tolist() for n in ['ipad','pencil']}
  self.emb.reset(Scene(id='same-instance-reset',instruction=self.emb.task.module.INSTRUCTION))
  rt=runtime.ACTIVE
  record=dict(before=before,after={n:rt.pose(n)[0].tolist() for n in ['ipad','pencil']},
              score=rt.scorer.score,steps=rt.scorer.steps,full_s=rt.scorer.full_s,
              attached_before_lift=rt.scorer.attached_before_lift)
  record['passed']=bool(rt.scorer.score==0 and rt.scorer.steps==0 and not rt.scorer.attached_before_lift
     and all(np.linalg.norm(rt.pose(n)[0]-np.asarray(runtime.SPECS[n].pos))<.003 for n in ['ipad','pencil']))
  (self.out/'reset-check.json').write_text(json.dumps(record,indent=2))
  assert record['passed'],record
 def capture(self,obs):
  if self.record:
   if 'overview_cam' in self.emb._base.scene.sensors:
    view=arr(self.emb._base.scene['overview_cam'].data.output['rgb'])[0,:,:,:3].astype(np.uint8)
    import cv2
    view=cv2.resize(view,(480,264))
    self.frames.append(np.vstack([np.hstack([view,obs.images['top_cam']]),np.hstack([obs.images['left_cam'],obs.images['right_cam']])]))
   else:self.frames.append(np.hstack([np.asarray(obs.images[k]) for k in sorted(obs.images)]))
 def tcp(self,side):
  robot=self.emb._base.scene[side+'_robot'];i=list(robot.body_names).index('gripper')
  pose=arr(robot.data.body_link_pose_w)[0,i];rot=R.from_quat(pose[3:]).as_matrix()
  return pose[:3]+rot@TCP_OFFSET,rot
 def solve(self,side,p,r,seed):
  def fun(q):
   pp,rr=ik.tcp_pose(q,station.BASES[side]);return np.r_[pp-p,.2*R.from_matrix(r@rr.T).as_rotvec()]
  fit=least_squares(fun,np.clip(seed,ik.LOWER+1e-8,ik.UPPER-1e-8),bounds=(ik.LOWER,ik.UPPER),max_nfev=100)
  if np.linalg.norm(fit.fun)>.002:
   alt=[-2.6,2.5,1.8,-1.67,-.4,1.93] if side=='left' else ([2.54,2.2,.54,1.22,.92,1.93] if p[0]<0 else [0,2.3,1,-1.6,-1,1.5])
   f=least_squares(fun,np.clip(alt,ik.LOWER+1e-8,ik.UPPER-1e-8),bounds=(ik.LOWER,ik.UPPER),max_nfev=100)
   if np.linalg.norm(f.fun)<np.linalg.norm(fit.fun):fit=f
  return fit.x,np.linalg.norm(fit.fun[:3]),np.linalg.norm(fit.fun[3:])/.2
 def tick(self,label):
  from inspect_robots.types import Action
  from inspect_robots_yam.packing import pack
  cmd=[np.r_[a['goal'][:6],np.clip(a['goal'][6]/FINGER_OPEN,0,1)] for a in self.arms.values()]
  res=self.emb.step(Action(data=pack(*cmd)));self.capture(res.observation)
  row=dict(runtime.ACTIVE.latest);row.update(step=len(self.rows)+1,phase=label,commands=[c.tolist() for c in cmd])
  for side in ['left','right']:
   robot=self.emb._base.scene[side+'_robot'];row[side+'_tcp']=self.tcp(side)[0].tolist()
   row[side+'_q']=arr(robot.data.joint_pos)[0].tolist();row[side+'_qd']=arr(robot.data.joint_vel)[0].tolist()
   row[side+'_effort']=arr(robot.data.applied_torque)[0].tolist() if hasattr(robot.data,'applied_torque') else None
  self.rows.append(row);self.trace.write(json.dumps(row)+'\n')
  if len(self.rows)%100==0:print('STEP',len(self.rows),label,'score',row['score'],flush=True)
 def save(self):
  (self.out/'result.json').write_text(json.dumps(dict(steps=len(self.rows),final=self.rows[-1] if self.rows else {},
    full_goal_proven=bool(self.rows and self.rows[-1]['score']==1.),scope='Native Arena public-action scripted reference'),indent=2))
 def finish(self):
  if getattr(self,'evaluator_mode',False):
   self.trace.close();return
  self.save();self.trace.close()
  if self.frames:
   import imageio
   imageio.imwrite(self.out/'setup.png',self.frames[0]);imageio.imwrite(self.out/'final.png',self.frames[-1])
   imageio.mimsave(self.out/'demo.mp4',self.frames,fps=20,codec='libx264',output_params=['-pix_fmt','yuv420p','-movflags','+faststart','-maxrate','3500k','-bufsize','7000k'])
 def go(self,side,label,duration,p=None,r=None,f=None,joint=False,qgoal=None,align=False):
  a=self.arms[side];p0,r0,f0=a['p'].copy(),a['r'].copy(),a['f'];q0=a['q'].copy()
  p1=p0 if p is None else np.array(p);r1=r0 if r is None else r;f1=f0 if f is None else f
  if qgoal is not None:p1,r1=ik.tcp_pose(qgoal,station.BASES[side]);joint=True
  qe=None
  if qgoal is not None:
   assert np.all(qgoal>=ik.LOWER) and np.all(qgoal<=ik.UPPER)
   qe=qgoal
  elif joint:
   qe,pe,re=self.solve(side,p1,r1,q0);self.ik_errors.append([pe,re])
  for k in range(1,round(duration*20)+1):
   s=ik.smoothstep(k/round(duration*20));p=p0+s*(p1-p0);r=ik.slerp_matrix(r0,r1,s)
   if joint:q=q0+s*(qe-q0);pe=re=0.
   else:
    if align:a['correction']=np.clip(a['correction']+.15*(p-self.tcp(side)[0]),-.005,.005)
    q,pe,re=self.solve(side,p+a['correction'],r,a['q'])
   self.ik_errors.append([pe,re]);a['q']=q;a['goal']=np.r_[q,[f0+s*(f1-f0)]*2];self.tick(label)
  a['p'],a['r'],a['f']=p1,r1.copy(),f1
  print(label,'ipad_z',round(runtime.ACTIVE.pose('ipad')[0][2]-TOP,4),'pen_z',round(runtime.ACTIVE.pose('pencil')[0][2]-TOP,4),'tcp_err_mm',round(np.linalg.norm(self.tcp(side)[0]-p1)*1000,2),flush=True)
  self.save()
  if self.frames:
   import imageio
   imageio.imwrite(self.out/(label+'.png'),self.frames[-1])

 def push(self):
  rot=grip_rotation(0.);p0=np.array([.077,.10,TOP+.0145]);p1=p0+[-float(os.environ.get('COMBINED_PUSH','.15')),0,0]
  for n,t,p,r,f,j in [('preopen',1.5,None,None,0,False),('approach_push',3.5,p0+[0,0,.09],rot,0,True),('lower_push',2,p0,rot,0,False),('push',5,p1,rot,0,False),('withdraw_push',2,p1+[0,0,.10],rot,0,False)]:self.go('left',n,t,p,r,f,j)

 def pen_pick(self):
  rot=R.from_euler('x',float(os.environ.get('PENCIL_PICK_TILT','0')),degrees=True).as_matrix()@grip_rotation(0.);offset=float(os.environ.get('PENCIL_GRASP_X','-.065'));p=np.array([.20+offset,-.17,TOP+float(os.environ.get('PENCIL_PICK_TCP_Z','.0125'))]);lift=p+[0,0,.13]
  if os.environ.get('PENCIL_TIP_DEPTH'):
   p=np.array([.20+offset,-.17,TOP+.00305])-rot[:,2]*float(os.environ['PENCIL_TIP_DEPTH']);lift=p+[0,0,.13]
  opening=float(os.environ.get('PENCIL_OPEN','-.025'))
  self.go('right','pen_preopen',2,f=opening)
  self.go('right','pen_approach',4,p+[0,0,.08],rot,opening,True)
  self.go('right','pen_descend',2,p,rot,opening)
  self.go('right','pen_align',1,p,rot,opening,align=True)
  hold=float(os.environ.get('PENCIL_HOLD','-.0035'))
  self.go('right','pen_close',2.5,p,rot,hold)
  self.go('right','pen_lift',3,lift,rot,hold)
  self.go('right','pen_carry',3,lift+[0,.09,0],rot,hold)
  roll=R.from_euler('x',float(os.environ.get('PENCIL_ROLL','90')),degrees=True).as_matrix()@rot
  self.go('right','pen_roll',4,lift+[0,.09,0],roll,hold)
  self.go('right','pen_settle',2)

 def pen_place(self):
  pc,rc=self.tcp('right');pp,_,rp=runtime.ACTIVE.pose('pencil')
  localp=rc.T@(pp-pc);localr=rc.T@rp
  pi,_,ri=runtime.ACTIVE.pose('ipad')
  target=pi+ri@np.array([0,.07055,0]);target[2]=max(target[2],TOP+float(os.environ.get('PENCIL_RELEASE_Z','.018')))
  desired=R.from_euler('x',float(os.environ.get('PENCIL_SEAT_ROLL','180')),degrees=True).as_matrix();handr=desired@localr.T;handp=target-handr@localp
  (self.out/'placement-plan.json').write_text(json.dumps(dict(target_pencil=target.tolist(),handp=handp.tolist(),handr=handr.tolist(),localp=localp.tolist(),localr=localr.tolist()),indent=2))
  hold=float(os.environ.get('PENCIL_HOLD','-.0035'))
  self.go('right','pen_place_approach',4,handp+[0,-.035,.06],handr,hold,True)
  self.go('right','pen_place_lower',3,handp+[0,-.015,0],handr,hold)
  self.go('right','pen_seat',3,handp,handr,hold)
  self.go('right','pen_seat_align',1,handp,handr,hold,align=True)
  self.seat_feedback()
  handp=self.arms['right']['p'].copy();handr=self.arms['right']['r'].copy()
  opening=float(os.environ.get('PENCIL_RELEASE_OPEN','-.025'))
  self.arms['right']['f']=float(np.min(arr(self.emb._base.scene['right_robot'].data.joint_pos)[0,-2:]))
  self.go('right','pen_release',2,handp,handr,opening)
  self.go('right','pen_retract',3,handp+[0,-.10,.10],handr,opening)
  self.go('right','pen_release_hold',2)

 def seat_feedback(self):
  """Align the two magnet sites using measured object poses and bounded robot motion."""
  a=self.arms['right']
  handp=a['p'].copy();handr=a['r'].copy();origin=handp.copy()
  for _ in range(100):
   pp,_,rp=runtime.ACTIVE.pose('pencil');pi,_,ri=runtime.ACTIVE.pose('ipad')
   target=pi+ri@np.array([0,.0704,.0013])
   handp+=np.clip(.25*(target-pp),-.001,.001)
   handp=origin+np.clip(handp-origin,-.015,.015)
   q,pe,re=self.solve('right',handp,handr,a['q']);a['q']=q;a['goal']=np.r_[q,[a['f']]*2]
   self.ik_errors.append([pe,re]);self.tick('pen_magnet_alignment')
  a['p'],a['r']=ik.tcp_pose(a['q'],station.BASES['right']);a['correction']=np.zeros(3)
  self.save()
  print('MAGNET_ALIGNMENT',runtime.ACTIVE.latest['relative'],runtime.ACTIVE.latest['face_deg'],flush=True)

 def pickup(self):
  rot=R.from_euler('y',40,degrees=True).as_matrix()@np.column_stack(([0,0,1],[0,-1,0],[1,0,0]));target=np.array([-.215,.10,TOP+.00315+.008]);hover=target+[-.085,0,.035]
  self.go('left','open_for_edge',1.5,f=-.03)
  self.go('left','approach_edge',3.5,hover,R.from_euler('y',15,degrees=True).as_matrix()@rot,-.03,True)
  self.go('left','descend',2,target,rot,-.03)
  self.go('left','align',1,target,rot,-.03,align=True)
  self.go('left','close',1.5,target,rot,-.0105)
  q=self.arms['left']['q'].copy();q[1]-=.8;q[2]-=.25;self.go('left','lift',3,f=-.001,qgoal=q)
  self.go('left','settle',3)
  q=self.arms['left']['q'].copy();q[1]-=.35;self.go('left','lift2',3,f=-.001,qgoal=q)
  self.go('left','final_hold',5)


def main():
 ap=argparse.ArgumentParser();ap.add_argument('out');ap.add_argument('--stage',default='all',choices=['smoke','push','pick','all']);ap.add_argument('--no-record',action='store_true');ap.add_argument('--reset-check',action='store_true');ap.add_argument('--physical-controls',action='store_true');a=ap.parse_args()
 defaults={'PENCIL_PICK_TILT':'0','PENCIL_GRASP_X':'-.060','PENCIL_TIP_DEPTH':'.0105','PENCIL_OPEN':'-.020','PENCIL_HOLD':'-.0015','PENCIL_ROLL':'65','PENCIL_RELEASE_Z':'.008','PENCIL_RELEASE_OPEN':'-.009','PENCIL_SEAT_ROLL':'170'}
 for k,v in defaults.items():os.environ.setdefault(k,v)
 t=None
 try:
  t=Trial(a.out,not a.no_record)
  for _ in range(5):t.tick('initial_contact_check')
  initial=runtime.ACTIVE.latest
  assert all(set(initial['contacts'][n])=={'/World/envs/env_0/table/geometry/mesh'} for n in ['ipad','pencil']),initial['contacts']
  if a.stage=='smoke':
   for _ in range(20):t.tick('smoke')
  else:
   t.push()
   if a.stage!='push':
    t.pen_pick()
    if a.stage=='all':t.pen_place();t.pickup()
  t.finish()
  if a.physical_controls:t.physical_controls()
  if a.reset_check:t.reset_check()
 except BaseException:
  error=traceback.format_exc();print(error,flush=True)
  Path(a.out).mkdir(parents=True,exist_ok=True);(Path(a.out)/'exception.txt').write_text(error)
  if t is not None:t.finish()
  raise
if __name__=='__main__':main()
