"""Root-only allocated-runtime driver. No model calls. Task executes BEFORE cold SDK imports.
Foundation is robot-only. evaluator.py contains separate diagnostic interventions.
"""
import argparse,functools,hashlib,json,os,sys,time,traceback
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--mode',choices=['foundation','reference','time-limit'],default='foundation');ap.add_argument('--legacy-rtx-override',action='store_true');args=ap.parse_args();args.out.mkdir(parents=True,exist_ok=False)
 report=dict(kind=args.mode,physics_verified=False,fullgoal_verified=False,model_calls=0,media_verified=False,checks={},source=str(HERE),started=time.time());emb=None;stream=(args.out/'trace.jsonl').open('w',buffering=1)
 try:
  manifest=HERE/'sourcehashes.json'
  if manifest.exists():
   for n,h in json.loads(manifest.read_text()).items():
    if hashlib.sha256((HERE/n).read_bytes()).hexdigest()!=h:raise RuntimeError('immutable source mismatch '+n)
  cold=['yam_arena.scene','yam_arena.actions','yam_arena.inspect_embodiment']
  if any(n in sys.modules for n in cold):raise RuntimeError('SDK imported before task source execution')
  from yam_arena.task_file import load
  task=load(HERE/'task.py');report['executed_task_sha256']=hashlib.sha256((HERE/'task.py').read_bytes()).hexdigest();report['checks']['task_executed_before_cold_imports']=not any(n in sys.modules for n in cold)
  if args.legacy_rtx_override:
   from isaaclab.app import AppLauncher
   original=AppLauncher.__init__
   @functools.wraps(original)
   def launch(self,*a,**kw):
    kw['kit_args']=(kw.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip();return original(self,*a,**kw)
   AppLauncher.__init__=launch
  os.environ['HJ_PHYSICS_TRACE']=str(args.out/'physics.jsonl')
  from inspect_robots import Scene
  from inspect_robots.types import Action
  from yam_arena.inspect_embodiment import yam_arena_embodiment
  from PIL import Image
  import kit
  from telemetry import dumps,snapshot,before_action
  emb=yam_arena_embodiment(task=str(HERE/'task.py'),device='cpu',object_jitter=0,control_hz=20,cam_width=320,cam_height=176)
  def record(res,action):stream.write(dumps(snapshot(emb,res.observation,'after_action',action))+'\n')
  emb._hj_pre_action=lambda obs,action:before_action(emb,obs,action,stream)
  obs=emb.reset(Scene(id='hydraulic-foundation-harness-87321',instruction=task.instruction if hasattr(task,'instruction') else 'Lower the cabinet onto the rails gently and let go for3s.'),seed=87321)
  stream.write(dumps(snapshot(emb,obs,'initial'))+'\n')
  report['initial']=kit.current_status();report['checks']['genuine14D']=np.asarray(obs.state['joint_pos']).shape==(14,)
  report['checks']['2ms_physics_20Hz_control']=abs(emb._base.physics_dt-.002)<1e-9 and abs(emb._base.step_dt-.05)<1e-9
  report['cameras']={n:dict(shape=list(v.shape),std=float(v.std())) for n,v in obs.images.items()};report['checks']['three_rgb']=len(obs.images)==3 and all(v.std()>5 for v in obs.images.values())
  for n,v in obs.images.items():Image.fromarray(v).save(args.out/(n+'.png'))
  report['hardware']={}
  for side in ('left','right'):
   a=emb._base.scene[side+'_robot'];report['hardware'][side]=dict(names=a.joint_names,ceilings=kit.arr(a.root_view.get_dof_max_forces()).tolist(),finger_cfg_N=a.cfg.actuators['gripper'].joint_effort_limit,total_effort_checked=getattr(a,'_hj_total_effort_checked',False),total_effort_contract=getattr(a,'_hj_total_effort_contract',None))
  report['contact_selections']=kit.ENGINE.selections
  if args.mode=='reference':
   from reference import Driver,Completed
   try:Driver(emb,obs,record).run()
   except Completed:report['reference_ended_on_actual_success']=True
   report['fullgoal_verified']=kit.current_status()['score']==1. and kit.current_status()['failure'] is None
  else:
   q=np.asarray(obs.state['joint_pos'],float);initial=q.copy()
   count=3210 if args.mode=='time-limit' else 50
   for i in range(count):
    action=initial.copy()
    if args.mode=='foundation' and 10<=i<30:action[0]+=.06;action[7]-=.06
    before_action(emb,obs,action,stream)
    res=emb.step(Action(data=action));obs=res.observation;record(res,action)
    if res.terminated:
     report['termination']=res.termination_reason
     try:emb.step(Action(data=action));report['checks']['posttermination_refused']=False
     except RuntimeError:report['checks']['posttermination_refused']=True
     break
   report['checks']['credit_gated_zero']=kit.current_status()['score']==0 and kit.current_status()['gate']=='unvalidated_channels'
   before=kit.ENGINE.last.copy();obs=emb.reset(Scene(id='hydraulic-second-reset-119',instruction='Lower the cabinet gently.'),seed=119)
   report['second_reset']=kit.current_status();report['checks']['arbitrary_seed_reset']=abs(kit.ENGINE.last['height']-.05)<.003 and kit.ENGINE.controls==0
  report['final']=kit.current_status();report['completed']=True
 except Exception as e:report.update(completed=False,error=type(e).__name__+': '+str(e),traceback=traceback.format_exc())
 finally:
  engine=getattr(sys.modules.get('kit'),'ENGINE',None)
  if engine is not None:
   report['final']=engine.status();report['first_failure']=getattr(engine,'first_failure',None)
  stream.close();report['elapsed_s']=time.time()-report['started'];(args.out/'result.json').write_text(__import__('telemetry').dumps(report)+'\n')
  if emb:
   engine=getattr(sys.modules.get('kit'),'ENGINE',None)
   if engine is not None and getattr(engine,'trace',None):engine.trace.close()
   emb.close()
 return 0 if report.get('completed') else 1
if __name__=='__main__':raise SystemExit(main())
