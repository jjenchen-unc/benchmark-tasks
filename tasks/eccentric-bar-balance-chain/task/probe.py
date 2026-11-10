"""Cold immutable-copy launcher; diagnostics are not robot feasibility evidence."""
# Only standard library until the copied source has been exec'd.
import argparse,hashlib,json,os,shutil,sys,time,traceback,functools
from pathlib import Path
ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--mode',choices=['foundation','reference','evaluator','calibration','replay','setup'],default='foundation');ap.add_argument('--variant',default='full');ap.add_argument('--frozen',action='store_true');ap.add_argument('--snapshot-only',action='store_true');a=ap.parse_args();HERE=Path(__file__).resolve().parent
if not a.frozen:
 a.out=a.out.resolve();a.out.mkdir(parents=True,exist_ok=False);frozen=a.out/'source';shutil.copytree(HERE,frozen,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
 hashes={str(p.relative_to(frozen)):hashlib.sha256(p.read_bytes()).hexdigest() for p in frozen.rglob('*') if p.is_file()};(a.out/'sourcehashes.json').write_text(json.dumps(hashes,indent=2))
 args=[sys.executable,*(['-I'] if sys.flags.isolated else []),'-B',str(frozen/'probe.py'),'--frozen','--out',str(a.out),'--mode',a.mode,'--variant',a.variant]
 if a.snapshot_only:args+=['--snapshot-only']
 os.execv(sys.executable,args)
a.out=a.out.resolve();sys.path.insert(0,str(HERE))
from strict_json import dumps

expected=json.loads((a.out/'sourcehashes.json').read_text())
if any(hashlib.sha256((HERE/p).read_bytes()).hexdigest()!=h for p,h in expected.items()):raise RuntimeError('copied-source hash mismatch')
(a.out/'executed-source.json').write_text(json.dumps({'file':str(Path(__file__).resolve()),'copied_source_executed_before_sdk_imports':True,'hashes_verified':True},indent=2))
embedded=json.loads((HERE/'candidate-manifest.json').read_text())
if any(hashlib.sha256((HERE/p).read_bytes()).hexdigest()!=h for p,h in embedded['files'].items()):raise RuntimeError('embedded manifest mismatch')
from runtime_binding import verify_runtime
verify_runtime()
if a.snapshot_only:print('immutable copied source executed, no SDK imported');sys.exit(0)
report={'mode':a.mode,'model_calls':0,'physics_verified':False,'fullgoal_verified':False,'media_verified':False,'score_claim':None,'checks':{},'error':None};emb=None
try:
 # Construct the real pure YAM task before any simulator bootstrap.
 from yam_arena.task_file import load
 loaded_task=load(HERE/'task.py')
 report['checks']['pure_task_constructed']=len(loaded_task.objects)==6
 if os.environ.get('B4_LEGACY_RTX_OVERRIDE')=='1':
  from isaaclab.app import AppLauncher
  old=AppLauncher.__init__
  @functools.wraps(old)
  def init(self,*args,**kw):kw['kit_args']=(kw.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip();return old(self,*args,**kw)
  AppLauncher.__init__=init
 os.environ['B4_TRACE']=str(a.out/'physics.jsonl')
 from inspect_robots import Scene
 from inspect_robots.types import Action
 from yam_arena.inspect_embodiment import yam_arena_embodiment
 import numpy as np
 from fixture import POSITIONS,ROBOT_BASES
 emb=yam_arena_embodiment(task=str(HERE/'task.py'),device='cpu',object_jitter=0,control_hz=20,cam_width=320,cam_height=176)
 obs=emb.reset(Scene(id='b4-foundation',instruction=loaded_task.module.__doc__.strip()),seed=271828)
 import kit
 from kit import arr
 report['checks']['supported_base_poses']=all(np.linalg.norm(arr(emb._base.scene[side+'_robot'].data.root_link_state_w)[0,:3]-arr(emb._base.scene.env_origins)[0]-ROBOT_BASES[side])<.001 for side in ('left','right'))
 report['seed_contract']={'physical_seed':42,'setting':'S','goal':'stack_ABC','inspect_trial_seed':271828,'SDK_reset_seed_passed_unchanged':True}
 report['live_selections']={'bar_body_paths':kit.ENGINE.bodies,'actual_robot_body_paths':kit.ENGINE.robots,'external_collider_roots':kit.ENGINE.external}
 report['checks']['14D_state']=len(obs.state['joint_pos'])==14
 report['checks']['initial_bar_layout']=all(np.linalg.norm(arr(emb._base.scene['bar_'+c].data.root_link_state_w)[0,:3]-arr(emb._base.scene.env_origins)[0]-POSITIONS[c])<.003 for c in 'ABC')
 report['cameras']={k:{'shape':list(v.shape),'std':float(v.std())} for k,v in obs.images.items()}
 report['checks']['rgb']=len(obs.images)==3 and all(v.std()>5 for v in obs.images.values())
 from PIL import Image
 for k,v in obs.images.items():Image.fromarray(v).save(a.out/('setup-'+k+'.png'))
 journal=open(a.out/'robot-actions.jsonl','w',buffering=1)
 def record(label,q,res):journal.write(dumps({'label':label,'action14':np.asarray(q).tolist(),'terminated':res.terminated,'reason':res.termination_reason,'status':kit.ENGINE.status()})+'\n')
 q=np.asarray(obs.state['joint_pos'],float).copy()
 if a.mode=='setup':
  def states():
   value={}
   for group in ('articulations','rigid_objects'):
    for name,obj in getattr(emb._base.scene,group).items():
     for field in ('body_link_state_w','root_link_state_w','joint_pos','joint_vel'):
      if hasattr(obj.data,field):value[name+'/'+field]=arr(getattr(obj.data,field)).copy()
   return value
  before=states();clock=(kit.ENGINE.history.samples,kit.ENGINE.controls,int(emb._base.sim.get_physics_step_count()))
  for _ in range(40):emb._base.sim.render()
  for name in ('top_cam','left_cam','right_cam'):
   cam=emb._base.scene[name];cam.update(0.,force_recompute=True);rgb=arr(cam.data.output['rgb'])[0,...,:3].astype(np.uint8)
   if rgb.std()<=5:raise RuntimeError('setup RGB invalid')
   Image.fromarray(rgb).save(a.out/('setup-'+name+'.png'))
  after=states()
  report['checks']['setup_state_unchanged']=before.keys()==after.keys() and all(np.array_equal(v,after[k]) for k,v in before.items())
  report['checks']['setup_no_physics']=clock==(kit.ENGINE.history.samples,kit.ENGINE.controls,int(emb._base.sim.get_physics_step_count())) and clock[:2]==(0,0)
 elif a.mode=='replay':
  from video_capture import Capture
  from validation_receipt import validated_channels,CHANNELS
  if validated_channels()!=CHANNELS:raise RuntimeError('source-bound actual receipt unavailable')
  rows=[json.loads(line) for line in (HERE/'robot-actions.jsonl').read_text().splitlines()]
  if len(rows)!=1441:raise RuntimeError('saved action inventory mismatch')
  capture=Capture(a.out,obs.images)
  try:
   for i,row in enumerate(rows,1):
    action=np.asarray(row['action14'],dtype=float)
    if action.shape!=(14,) or not np.isfinite(action).all():raise RuntimeError('invalid saved action')
    result=emb.step(Action(data=action));record(row['label'],action,result);capture.append(result.observation.images)
    status=kit.ENGINE.status()
    if status['samples']!=25*i or status['failure'] or status['measurement_error'] or status['missing_runtime_validation']:raise RuntimeError('replay measurement/clock/guard failure')
    if result.terminated and i!=len(rows):raise RuntimeError('early replay termination')
   report['checks']['new_actual_public_fullgoal']=kit.current_score()==1. and kit.ENGINE.history.hold[-1]>=20.
   report['controls']=len(rows)
  finally:
   report['media']=capture.close()
   report['checks']['encoded_media']=report['media']['all_encoded'] and all(x['bytes']<=50_000_000 for x in report['media']['files'].values())
 elif a.mode=='reference':
  from yam_arena import ik
  startup={}
  for side in ('left','right'):
   robot=emb._base.scene[side+'_robot'];raw=arr(robot.data.joint_pos)[0];names=list(robot.joint_names)
   arm=np.array([raw[names.index('joint'+str(i))] for i in range(1,7)])
   tcp,rot=ik.tcp_pose(arm,ROBOT_BASES[side])
   startup[side]={'joint_names':names,'raw8':raw,'fk_tcp':tcp,'fk_rotation':rot}
  (a.out/'startup-measured-raw8-fk.json').write_text(dumps(startup,indent=2))
  from reference import run
  run(emb,obs,record,a.variant)
 elif a.mode=='foundation':
  for i in range(20):
   target=q.copy();target[0]+=.04 if i<10 else 0;res=emb.step(Action(data=target));record('neutral_joint_probe',target,res)
   if res.terminated:raise RuntimeError(str(res.termination_reason))
  report['checks']['physics_clock']=abs(kit.ENGINE.history.time-1.)<1e-6
  report['checks']['substeps']=kit.ENGINE.history.samples==500
  report['status_before_reset']=kit.ENGINE.status()
  obs=emb.reset(Scene(id='b4-reset-routing',instruction='Reset same fixed physical case.'),seed=314159)
  report['checks']['reset_history']=kit.ENGINE.history.samples==0 and kit.ENGINE.controls==0
  report['checks']['reset_layout']=all(np.linalg.norm(arr(emb._base.scene['bar_'+c].data.root_link_state_w)[0,:3]-arr(emb._base.scene.env_origins)[0]-POSITIONS[c])<.003 for c in 'ABC')
  report['checks']['score_still_gated']=kit.current_score()==0
 elif a.mode=='calibration':
  from calibration import run
  report['calibration']=run(emb,q,record)
  report['evaluator_only']=True
 elif a.mode=='evaluator':
  import torch
  if a.variant=='wrist_abort':
   # EVALUATOR ONLY known external load, never used by robot reference.
   robot=emb._base.scene['right_robot'];force=torch.tensor([[[0.,0.,-120.]]],device=emb._base.device)
   robot.permanent_wrench_composer.reset();robot.permanent_wrench_composer.add_forces_and_torques(force,torch.zeros_like(force),body_ids=robot.find_bodies('gripper')[0],is_global=True)
   terminated=False
   for i in range(10):
    res=emb.step(Action(data=q));record('EVALUATOR_ONLY_120N',q,res)
    if res.terminated:terminated=True;report['termination_reason']=res.termination_reason;break
   robot.permanent_wrench_composer.reset()
   report['checks']['actual_wrist_abort']=terminated and res.termination_reason=='wrist_force_abort'
   if terminated:
    before=kit.ENGINE.history.samples
    try:emb.step(Action(data=q));report['checks']['step_after_termination_guard']=False
    except RuntimeError:report['checks']['step_after_termination_guard']=kit.ENGINE.history.samples==before
  else:
   # Direct preplacement is an evaluator fixture, NEVER robot feasibility.
   from fixture import SUPPORT,PARAMETERS
   origin=arr(emb._base.scene.env_origins)[0]
   order='BAC' if a.variant=='wrong_order' else 'ABC'
   order=order[:{'one_bar':1,'two_bar':2}.get(a.variant,3)]
   for j,c in enumerate(order):
    body=emb._base.scene['bar_'+c];pose=body.data.default_root_pose.torch.clone();pose[0,:3]=torch.tensor(origin+SUPPORT+[-round(PARAMETERS[c]['com'],6),0,.08+.025+j*.05],device=pose.device);body.write_root_pose_to_sim_index(root_pose=pose);body.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(body.data.default_root_vel.torch))
   for i in range(380 if a.variant=='truncate' else 430):
    res=emb.step(Action(data=q));record('EVALUATOR_ONLY_placed_'+a.variant,q,res)
    if res.terminated:break
  report['evaluator_only']=True
 report['status']=kit.ENGINE.status();journal.close()
except Exception as exc:
 report['error']=f'{type(exc).__name__}: {exc}';report['traceback']=traceback.format_exc()
 engine=getattr(sys.modules.get('kit'),'ENGINE',None)
 if engine is not None:report['primary_failure_snapshot']={'status':engine.status(),'raw_tick':engine.last}

finally:
 (a.out/'result.json').write_text(dumps(report,indent=2))
 if emb is not None:emb.close()
print(json.dumps({'mode':a.mode,'error':report['error'],'checks':report['checks']}))
if report['error'] or not all(report['checks'].values()):sys.exit(1)
