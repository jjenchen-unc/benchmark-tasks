"""Actual SDK runner. Use only on an allocated root-owned GPU worker.
Modes separate evaluator diagnostics from genuine robot reference controls.
"""
import argparse,functools,json,os,sys,traceback
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
from telemetry_json import dumps
def main(forced_mode=None):
 ap=argparse.ArgumentParser();ap.add_argument('--mode',choices=['foundation','reference','negatives'],default='foundation');ap.add_argument('--out',type=Path,required=True);ap.add_argument('--legacy-rtx-override',action='store_true');a=ap.parse_args();a.mode=forced_mode or a.mode;a.out.mkdir(parents=True,exist_ok=True)
 report={'mode':a.mode,'physics_verified':False,'model_calls':0,'full_goal_verified':False,'checks':{},'episodes':[]}
 (a.out/'STATUS.json').write_text(dumps({'status':'starting','mode':a.mode}))
 emb=None
 import hashlib
 expected=json.loads((HERE/'sourcehashes.json').read_text())
 actual={str(p.relative_to(HERE)):hashlib.sha256(p.read_bytes()).hexdigest() for p in HERE.rglob('*') if p.is_file() and p.name!='sourcehashes.json' and '__pycache__' not in p.parts and p.suffix!='.pyc'}
 if actual!=expected:raise RuntimeError('preSDK embedded manifest mismatch')
 (a.out/'pre-sdk-hash-proof.json').write_text(dumps(dict(passed=True,files_sha256=actual,sdk_imports_started=False),indent=2))
 try:
  # Real installed pure TaskFile construction must precede AppLauncher.
  from yam_arena.task_file import load
  definition=load(HERE/'task.py')
  (a.out/'task-construction.json').write_text(dumps(dict(scene=definition.scene,objects=[o.name for o in definition.objects],target=__import__('s1_kit').TARGET,telemetry_origin=__import__('telemetry_json').__file__),indent=2))
  if a.legacy_rtx_override:
   from isaaclab.app import AppLauncher
   original=AppLauncher.__init__
   @functools.wraps(original)
   def init(self,*args,**kw):kw['kit_args']=(kw.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip();return original(self,*args,**kw)
   AppLauncher.__init__=init
  from inspect_robots import Scene
  from inspect_robots.types import Action
  from yam_arena.inspect_embodiment import yam_arena_embodiment
  import s1_kit as kit
  from contacts import arr
  os.environ['S1_TRACE']=str(a.out/'physics.jsonl')
  emb=yam_arena_embodiment(task=str(HERE/'task.py'),device='cpu',object_jitter=0,control_hz=20,cam_width=320,cam_height=176)
  def robot_snapshot(observation):
   robots={}
   for side in ['left','right']:
    r=emb._base.scene[side+'_robot']
    robots[side]={'joint_names':list(r.joint_names),'raw_joint_pos':arr(r.data.joint_pos)[0].copy(),'raw_joint_vel':arr(r.data.joint_vel)[0].copy(),'body_names':list(r.body_names),'body_pos_w':arr(r.data.body_pos_w)[0].copy(),'body_quat_w_xyzw':arr(r.data.body_quat_w)[0].copy(),'root_pos_w':arr(r.data.root_pos_w)[0].copy(),'root_quat_w_xyzw':arr(r.data.root_quat_w)[0].copy()}
   # Independent URDF FK from all eight raw joints and the measured base.
   from yam_arena import YAM_URDF
   from yam_arena.urdf import forward_kinematics,quat_xyzw_to_matrix
   from yam_arena.constants import TCP_OFFSET
   for side,row in robots.items():
    fk=forward_kinematics(YAM_URDF,dict(zip(row['joint_names'],map(float,row['raw_joint_pos']))))
    R=quat_xyzw_to_matrix(np.asarray(row['root_quat_w_xyzw']));base=np.asarray(row['root_pos_w'])
    row['fk_link_origins_w']={name:base+R@T[:3,3] for name,T in fk.items()}
    row['fk_tcp_w']=base+R@(fk['gripper'][:3,3]+fk['gripper'][:3,:3]@np.asarray(TCP_OFFSET))
    row['fk_approach_w']=R@fk['gripper'][:3,2]
   return {'public14':np.asarray(observation.state['joint_pos']).copy(),'robots':robots}
  def fixture_snapshot():
   E=kit.ENGINE
   return {'qv':E.state(),'body_names':list(E.asset.body_names),'body_pos_w':arr(E.asset.data.body_pos_w)[0].copy(),'body_quat_w_xyzw':arr(E.asset.data.body_quat_w)[0].copy(),'source_trigger_states':E.last,'status':E.status()}
  def geometry_proxies(robot,fixture):
   from yam_arena.urdf import quat_xyzw_to_matrix
   row=robot['robots']['right'];names=fixture['body_names'];j=names.index('cap_1')
   R=quat_xyzw_to_matrix(np.asarray(fixture['body_quat_w_xyzw'][j]));face=np.asarray(fixture['body_pos_w'][j])+R@np.array([0.,0.,.014])
   tip=np.asarray(row['fk_tcp_w'])+.013*np.asarray(row['fk_approach_w']);delta=tip-face
   lever=np.asarray(fixture['body_pos_w'][names.index('lever_1')])
   return {'cap_tip_midpoint_axial_m':float(delta@R[:,2]),'cap_tip_midpoint_transverse_m':float(np.linalg.norm(delta-(delta@R[:,2])*R[:,2])),'tcp_to_lever_origin_m':float(np.linalg.norm(np.asarray(row['fk_tcp_w'])-lever)),'scope':'FK midpoint proxies; individual mesh contacts remain measured sensor channels'}
  def reset(label,seed):
   obs=emb.reset(Scene(id='S1_'+label,instruction=__import__('task').__doc__),seed=seed)
   E=kit.ENGINE;q,v=E.state();report['episodes'].append({'label':label,'seed':seed,'q':q,'v':v,'status':E.status(),'bodies':E.asset.body_names,'joints':E.asset.joint_names,'contact_selection':{i:c.selection for i,c in E.contacts.items()}})
   report['episodes'][-1]['robot_snapshot']=robot_snapshot(obs)
   return obs
  obs=reset('foundation_start',13498237)
  report['cameras']={n:{'shape':list(im.shape),'minimum':int(im.min()),'maximum':int(im.max()),'std':float(im.std())} for n,im in obs.images.items()}
  report['checks']['three_rgb_streams_nonconstant']=len(report['cameras'])==3 and all(r['std']>0 for r in report['cameras'].values())
  robot=emb._base.scene['right_robot'];report['motors']={}
  for side in ['left','right']:
   r=emb._base.scene[side+'_robot'];view=r.root_view
   row={n:arr(getattr(view,fn)())[0].tolist() for n,fn in {'max_forces':'get_dof_max_forces','stiffness':'get_dof_stiffnesses','external_effort':'get_dof_actuation_forces','gravity':'get_gravity_compensation_forces'}.items()};report['motors'][side]=row
   report['checks'][side+'_actual_caps']=bool(np.allclose(row['max_forces'],[27,27,27,7,7,7,50,50]))
  def step(data):
   nonlocal obs
   result=emb.step(Action(data=np.asarray(data)));obs=result.observation;return result
  # Validate actual reset metadata, observer status, raw state before action 1.
  (a.out/'pre-action.json').write_text(dumps(dict(report, observer_status=kit.ENGINE.status(), observer_sample=kit.ENGINE.last,robot_snapshot=robot_snapshot(obs)),indent=2))
  neutral=np.asarray(obs.state['joint_pos'],float)
  for n in range(20):result=step(neutral)
  report['checks']['twenty_controls_one_second']=abs(kit.ENGINE.hist.elapsed-1.)<1e-7
  report['neutral_robot_snapshot']=robot_snapshot(obs)
  report['neutral_state']=kit.ENGINE.state()[0]
  report['robot_body_paths']={side:__import__('contacts').body_paths(emb._base.scene[side+'_robot']) for side in ['left','right']}
  report['neutral_status']=kit.ENGINE.status();report['checks']['neutral_no_credit']=kit.ENGINE.hist.raw_score()==0.
  first=report['episodes'][0]['q'];obs=reset('seed_routing_reset',927617)
  second=report['episodes'][-1]['q'];report['checks']['same_physical_case_different_episode_seeds']=max(abs(first[n]-second[n]) for n in first)<1e-6
  report['checks']['reset_history_zero']=kit.ENGINE.hist.elapsed==0 and kit.ENGINE.hist.hold==0 and kit.ENGINE.hist.over_max==0
  if a.mode=='reference':
   if kit.TARGET!='SH-T1':raise ValueError('Reference implements representativeSH-T1 only')
   from reference import Teacher
   teacher=Teacher();actions=[]
   (a.out/'reference-pre-action.json').write_text(dumps(dict(robot_snapshot=robot_snapshot(obs),fixture_state=fixture_snapshot(),status=kit.ENGINE.status()),indent=2))
   for n in range(4198):
    E=kit.ENGINE;forces=E.last['wrist_force_n'] if E.last else [0.,0.]
    try:data,phase=teacher.next(obs.state['joint_pos'],forces,fixture_snapshot(),robot_snapshot(obs)['robots']['right']['raw_joint_pos'][-2:])
    except Exception as controller_error:
     (a.out/'first-controller-failure.json').write_text(dumps(dict(control=n+1,phase=teacher.seq[teacher.index][0],error=repr(controller_error),robot_snapshot=robot_snapshot(obs),fixture_state=E.state(),status=E.status()),indent=2));raise
    # Append each complete pre/post record before any later action or reset.
    before={'control':n+1,'phase':phase,'command14':data,'controller_feedback':teacher.telemetry,'robot':robot_snapshot(obs),'fixture':fixture_snapshot()}
    before['geometry_proxies']=geometry_proxies(before['robot'],before['fixture'])
    actions.append({'phase':phase,'action':data.tolist()});result=step(data)
    with (a.out/'controller-stream.jsonl').open('a') as stream:
     post={'robot':robot_snapshot(obs),'fixture':fixture_snapshot()};post['geometry_proxies']=geometry_proxies(post['robot'],post['fixture'])
     stream.write(dumps({'pre':before,'post':post})+'\n')
    (a.out/'actions.json').write_text(dumps(actions))
    if result.terminated or teacher.finished:break
   report['reference_status']=kit.ENGINE.status();report['full_goal_verified']=False # gated until independent foundation calibration
  elif a.mode=='negatives':
   # Evaluator-only placed state: isolates scorer plumbing; never robot evidence.
   import torch
   obs=reset('evaluator_open_placement',12345);E=kit.ENGINE;q=torch.zeros_like(E.asset.data.joint_pos.torch)
   for j,n in enumerate(E.asset.joint_names):q[:,j]=70*np.pi/180 if n=='door_1' else 0.
   E.mech['1'].engaged=False;E.mech['1'].released=True
   E.asset.write_joint_state_to_sim(q,torch.zeros_like(q))
   for n in range(20):step(np.asarray(obs.state['joint_pos'],float))
   report['placed_diagnostic']=E.status()
   report['placed_diagnostic_note']='Evaluator joint/latch setup only; no task/demo credit or valid robot trajectory.'
   report['known_load_diagnostics']=[]
   # The verified actual B5 wrench-composer API, confined to this evaluator mode.
   for axis,amplitude in [(0,20.),(1,20.),(2,20.),(2,150.)]:
    obs=reset('evaluator_wrist_load_'+str(axis)+'_'+str(amplitude),12345+axis)
    E=kit.ENGINE;robot=emb._base.scene['right_robot'];ids=[robot.body_names.index('gripper')];neutral=np.asarray(obs.state['joint_pos'],float)
    sensor=emb._base.scene['right_s1_wrench'];si=sensor.body_names.index('gripper');series=[]
    for n in range(5):step(neutral)
    baseline=arr(sensor.data.force)[0,si].tolist();force=torch.zeros((1,1,3),device=emb._base.device);force[0,0,axis]=amplitude
    robot.permanent_wrench_composer.reset();robot.permanent_wrench_composer.add_forces_and_torques(force,torch.zeros_like(force),body_ids=ids,is_global=True)
    for n in range(8):
     result=step(neutral);series.append({'force':arr(sensor.data.force)[0,si].tolist(),'body_quaternion_xyzw':arr(robot.data.body_quat_w)[0,ids[0]].tolist(),'status':E.status()})
     if result.terminated:break
    robot.permanent_wrench_composer.reset()
    report['known_load_diagnostics'].append({'axis_world':axis,'force_N':amplitude,'baseline_force_in_sensor_frame':baseline,'samples':series,'note':'Evaluator artificial load; sensor basis/response and overload require actual review, no automatic channel validation.'})

  if a.mode!='reference':
   # Evaluator termination-guard diagnostic, never in the robot reference route.
   E=kit.ENGINE;E.done=True;before=E.hist.elapsed
   try:step(np.asarray(obs.state['joint_pos'],float));report['checks']['step_after_termination_rejected']=False
   except RuntimeError:report['checks']['step_after_termination_rejected']=E.hist.elapsed==before
  report['status']='runtime_diagnostics_complete_requires_calibration';report['physics_verified']=False
 except Exception as error:
  report['status']='failed';report['error']=repr(error);report['traceback']=traceback.format_exc();(a.out/'error.json').write_text(dumps(report,indent=2))
 finally:
  if emb:
   try:emb.close()
   except Exception as close_error:report['close_error']=repr(close_error)
  (a.out/'result.json').write_text(dumps(report,indent=2));(a.out/'STATUS.json').write_text(dumps({'status':report.get('status'),'full_goal_verified':False},indent=2))
 return 1 if report.get('status')=='failed' else 0
if __name__=='__main__':raise SystemExit(main())
