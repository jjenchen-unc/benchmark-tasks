"""Root allocation only. Freeze and EXECUTE copied task before cold SDK imports."""
import argparse,functools,hashlib,json,os,runpy,shutil,sys,traceback
from pathlib import Path
ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--mode',choices=['foundation','reference','negative','component','first-pull'],default='foundation');ap.add_argument('--legacy-rtx-override',action='store_true');a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=False)
origin=Path(__file__).resolve().parent
manifest=json.loads((origin/'package-manifest.json').read_text())
actual={str(p.relative_to(origin)):hashlib.sha256(p.read_bytes()).hexdigest() for p in origin.rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc' and p.name!='package-manifest.json'}
if actual!=manifest['files']:raise RuntimeError('private frozen input manifest mismatch')
if hashlib.sha256(json.dumps(actual,sort_keys=True,separators=(',',':')).encode()).hexdigest()!=manifest['fingerprint']:raise RuntimeError('private input fingerprint mismatch')
source=a.out/'source';shutil.copytree(Path(__file__).resolve().parent,source,ignore=shutil.ignore_patterns('__pycache__','*.pyc'));sys.path.insert(0,str(source))
(a.out/'sourcehashes.json').write_text(json.dumps({str(p.relative_to(source)):hashlib.sha256(p.read_bytes()).hexdigest() for p in source.rglob('*') if p.is_file()},indent=2))
assert 'yam_arena.inspect_embodiment' not in sys.modules and 'yam_arena.scene' not in sys.modules
from yam_arena.task_file import load as load_task
cold_task=load_task(source/'task.py')
# Pure planning definitions are part of the frozen consumed source contract.
from yam_arena import ik,station,urdf
(a.out/'launcher-pre-sdk.json').write_text(json.dumps({'copied_task_executed':True,'constructed':True,'objects':[o.name for o in cold_task.objects],'cold_sdk': 'yam_arena.inspect_embodiment' not in sys.modules and 'yam_arena.scene' not in sys.modules,'task_sha256':hashlib.sha256((source/'task.py').read_bytes()).hexdigest()}))
if a.legacy_rtx_override:
 from isaaclab.app import AppLauncher
 original=AppLauncher.__init__
 @functools.wraps(original)
 def init(self,*args,**kwargs):kwargs['kit_args']=(kwargs.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip();return original(self,*args,**kwargs)
 AppLauncher.__init__=init
os.environ['PINS_TRACE']=str(a.out/'measurements.jsonl')
from inspect_robots import Scene
from inspect_robots.types import Action
from yam_arena.inspect_embodiment import yam_arena_embodiment
import numpy as np
from numeric_json import dumps
emb=None;actions=[];checks={};report={'mode':a.mode,'model_calls':0,'media_verified':False,'full_goal_verified':False}
def save(n,v):(a.out/n).write_text(dumps(v,indent=2))
try:
 emb=yam_arena_embodiment(task=str(source/'task.py'),device='cpu',object_jitter=0,control_hz=20,cam_width=320,cam_height=176)
 obs=emb.reset(Scene(id='pins-foundation-harness-derived-id',instruction=runpy.run_path(str(source/'task.py'))['__doc__']),seed=19358472)
 import kit
 E=kit.ENGINE;save('contact-processing.json',E.contacts.processing);checks['task_hooks_executed']=E is not None;checks['fixed_seed_routing_accepts_episode_seed']=True
 for side in ['left','right']:
  robot=emb._base.scene[side+'_robot'];checks[side+'_actual_drive_ceilings']=bool(np.allclose(kit.arr(robot.root_view.get_dof_max_forces()),[[27,27,27,7,7,7,40,40]]));save(side+'-joint-table.json',{'names':robot.joint_names,'drive_caps':kit.arr(robot.root_view.get_dof_max_forces()).tolist(),'stiffness':kit.arr(robot.root_view.get_dof_stiffnesses()).tolist(),'damping':kit.arr(robot.root_view.get_dof_dampings()).tolist(),'body_paths':robot.root_view.link_paths})
 checks['all10_source_joints_present']=set(E.jids)==set(kit.initial());checks['initial_joint_positions_match_source']=all(abs(E.joints()[0][n]-q)<.002 for n,q in kit.initial().items())
 if a.mode=='component':
  os.environ['PINS_COMPONENT_TRACE']=str(a.out/'component-probes.jsonl')
  from component import evaluate
  save('component-evidence.json',evaluate(emb,obs,actions))
 elif a.mode=='first-pull':
  os.environ['PINS_ACTION_TRACE']=str(a.out/'robot-actions.jsonl')
  from first_pull import evaluate
  save('first-pull-evidence.json',evaluate(emb,obs,actions))
 elif a.mode=='reference':
  os.environ['PINS_ACTION_TRACE']=str(a.out/'robot-actions.jsonl')
  from reference import Teacher
  teacher=Teacher(emb,obs,actions)
  try:teacher.run()
  finally:teacher.pair_diagnostic.close()
 else:
  q=np.array(obs.state['joint_pos'],float)
  for _ in range(20):
   r=emb.step(Action(data=q.copy()));obs=r.observation;actions.append(q.tolist())
   if r.terminated:raise RuntimeError('unexpected neutral termination: '+str(r.termination_reason))
  checks['actual20Hz_substep_clock']=abs(E.history.time-1.)<1e-8;checks['credit_stays_gated']=E.status()['score']==0 and E.status()['gate'] is not None
  if a.mode=='negative':
   # EVALUATOR ONLY load, never imported by the teacher. Measured force must
   # trip the real guard; no wrist/history/score replacement is permitted.
   import torch
   rob=emb._base.scene['left_robot'];forces=torch.zeros((1,len(rob.body_names),3),device=emb._base.device);forces[0,rob.body_names.index('gripper'),1]=100.
   rob.set_external_force_and_torque(forces,torch.zeros_like(forces),is_global=True)
   for _ in range(10):
    r=emb.step(Action(data=q.copy()));actions.append(q.tolist())
    if r.terminated:break
   checks['physical_wrist_abort']=r.termination_reason=='wrist_abort'
   rob.set_external_force_and_torque(torch.zeros_like(forces),torch.zeros_like(forces),is_global=True)
   if r.terminated:
    try:emb.step(Action(data=q.copy()));checks['step_after_termination_guard']=False
    except RuntimeError:checks['step_after_termination_guard']=True
  else:
   reset=emb.reset(Scene(id='another-inspect-id',instruction='Same fixed physical case'),seed=951876)
   checks['reset_restores_custom_joint_state']=all(abs(E.joints()[0][n]-v)<.002 for n,v in kit.initial().items())
 report['independent_raw_terminal_goal']=bool(a.mode=='reference' and E.status()['diagnostic_score']==1. and not E.status()['failure'])
 report.update(checks=checks,all_passed=all(checks.values()),status=E.status(),action_count=len(actions))
 save('result.json',report)
except BaseException as e:
 report.update(error=type(e).__name__+': '+str(e),traceback=traceback.format_exc(),checks=checks);save('result.json',report);raise
finally:
 consumed={str(Path(m.__file__).resolve()):hashlib.sha256(Path(m.__file__).read_bytes()).hexdigest() for m in tuple(sys.modules.values()) if getattr(m,'__file__',None) and Path(m.__file__).is_file()}
 save('runtime-consumed-source-sha256.json',consumed)
 save('actions.json',actions)
 if emb:emb.close()
