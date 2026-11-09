"""Record a new real D3 robot-action replay and its actual runtime score.
Historical action provenance is preserved. The only permitted historical runtime
difference is explicit promotion of previously verified measurement-channel flags.
No task object state, force, latch, score or history is set by this policy.
"""
from pathlib import Path
import argparse,ast,hashlib,json,math,shutil,sys,time,subprocess,traceback

CHANNELS={'clock','shock','contact','wrist','package_force','latch','drop'}
OLD_CHANNELS=CHANNELS-{'latch','drop'}
def hashes(root):
 return {str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(root.rglob('*')) if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc'}
def channel_tree(path):
 tree=ast.parse(path.read_text());found=[]
 for n in tree.body:
  if isinstance(n,ast.Assign) and len(n.targets)==1 and isinstance(n.targets[0],ast.Name) and n.targets[0].id=='VALIDATED_RUNTIME_CHANNELS':
   v=n.value
   if not(isinstance(v,ast.Call) and isinstance(v.func,ast.Name) and v.func.id=='frozenset' and len(v.args)==1 and not v.keywords):raise ValueError('Unexpected channel declaration')
   values=set(ast.literal_eval(v.args[0]));found.append(values);n.value=ast.Constant('MEASUREMENT_VALIDATION_DECLARATION')
 if len(found)!=1:raise ValueError('Missing/ambiguous channel declaration')
 return ast.dump(tree,include_attributes=False),found[0]
def validate_history_source(historical,current):
 old=hashes(historical);new=hashes(current)
 if old==new:return {'identical':True,'changes':[]}
 if set(old)!=set(new):raise ValueError('Historical/runtime source inventory differs')
 changed=[n for n in old if old[n]!=new[n]]
 if changed!=['d3_physics.py']:raise ValueError('Changed source beyond explicit channel promotion: '+repr(changed))
 before,old_flags=channel_tree(historical/'d3_physics.py');after,new_flags=channel_tree(current/'d3_physics.py')
 if before!=after or old_flags!=OLD_CHANNELS or new_flags!=CHANNELS:raise ValueError('Unapproved runtime change; physics/scorer/guards must remain identical')
 return {'identical':False,'changes':changed,'old_verified_channels':sorted(old_flags),'new_verified_channels':sorted(new_flags),'scope':'Only static measurement-validation declaration changed. All executable physics/scoring/guard AST and every other source byte unchanged. This NEW real action replay, not historical score, establishes matching-source feasibility.'}
def read_actions(path):
 rows=[]
 for line in path.read_text().splitlines():
  r=json.loads(line);q=r.get('q_command')
  if not isinstance(q,list) or len(q)!=14 or any(type(v) not in (int,float) or not math.isfinite(v) for v in q):raise ValueError('Invalid genuine14D action')
  if type(r.get('control')) is not int or r.get('control')!=len(rows)+1 or abs(r.get('physics_s',-1)-r['control']*.05)>1e-7:raise ValueError('Noncontiguous20Hz action history')
  if not 0<=q[6]<=1 or not 0<=q[13]<=1:raise ValueError('Jaw out of bounds')
  rows.append(r)
 if not rows or len(rows)>1604:raise ValueError('Action budget violated')
 return rows
def main():
 ap=argparse.ArgumentParser()
 for n in ['task-source','journal','journal-source','expected-source-manifest','out']:ap.add_argument('--'+n,type=Path,required=True)
 ap.add_argument('--driver-check-override',action='store_true');ap.add_argument('--preflight-only',action='store_true');ap.add_argument('--width',type=int,default=640);ap.add_argument('--height',type=int,default=360)
 a=ap.parse_args();a.task_source=a.task_source.resolve();a.journal_source=a.journal_source.resolve()
 if a.width%2 or a.height%2 or a.width<320 or a.height<176:raise ValueError('Expected even camera dimensions at least320x176')
 expected=json.loads(a.expected_source_manifest.read_text())
 if expected!=hashes(a.task_source):raise ValueError('Current task source differs from registered exact manifest')
 relation=validate_history_source(a.journal_source,a.task_source);rows=read_actions(a.journal)
 a.out.mkdir(parents=True,exist_ok=False)
 def save(name,value):(a.out/name).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
 shutil.copytree(a.task_source,a.out/'source')
 if hashes(a.out/'source')!=expected or hashes(a.task_source)!=expected:raise ValueError('Source changed while freezing')
 save('source-sha256.json',expected);save('historical-source-sha256.json',hashes(a.journal_source))
 save('action-provenance.json',{'journal_sha256':hashlib.sha256(a.journal.read_bytes()).hexdigest(),'controls':len(rows),'source_relation':relation,'no_historical_score_transfer':True})
 if a.preflight_only:save('result.json',{'preflight_pass':True,'runtime_executed':False});return
 sys.dont_write_bytecode=True;sys.path.insert(0,str(a.out/'source'))
 if a.driver_check_override:
  from isaaclab.app import AppLauncher
  original=AppLauncher.__init__
  def init(self,*args,**kwargs):
   kwargs['kit_args']=(kwargs.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip();return original(self,*args,**kwargs)
  AppLauncher.__init__=init
 import numpy as np
 from inspect_robots import Scene
 from inspect_robots.types import Action
 from yam_arena.inspect_embodiment import yam_arena_embodiment
 from PIL import Image
 import d3_kit
 writers={};logs={};emb=None;started=time.monotonic();report=None;frames=0;encoder_results={}
 ff=shutil.which('ffmpeg')
 if not ff:
  import imageio_ffmpeg
  ff=imageio_ffmpeg.get_ffmpeg_exe()
 try:
  emb=yam_arena_embodiment(task=str(a.out/'source/task.py'),device='cuda:0',object_jitter=0,control_hz=20,cam_width=a.width,cam_height=a.height)
  instruction=json.loads((a.out/'source/task.draft.json').read_text())['instruction']
  obs=emb.reset(Scene(id='D3-real-robot-saved-action-demo',instruction=instruction),seed=42)
  for name,rgb in obs.images.items():
   Image.fromarray(rgb).save(a.out/('setup-'+name+'.png'));h,w=rgb.shape[:2]
   log=(a.out/(name+'-encoder.log')).open('wb');logs[name]=log
   cmd=[ff,'-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s',str(w)+'x'+str(h),'-r','20','-i','pipe:0','-an','-c:v','libx264','-pix_fmt','yuv420p','-crf','22','-threads','1','-movflags','+faststart',str(a.out/(name+'.mp4'))]
   writers[name]=subprocess.Popen(cmd,stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=log)
  def frame(images):
   nonlocal frames
   if set(images)!=set(writers):raise ValueError('Camera inventory changed')
   for name,rgb in images.items():writers[name].stdin.write(np.ascontiguousarray(rgb,dtype=np.uint8).tobytes())
   frames+=1
  frame(obs.images)
  with (a.out/'actions.jsonl').open('x',buffering=1) as journal:
   for i,row in enumerate(rows,1):
    result=emb.step(Action(data=np.asarray(row['q_command'],dtype=float)));obs=result.observation;E=d3_kit.ENGINE
    frame(obs.images);sample={'control':i,'physics_s':E.physics_s,'q_command':row['q_command'],'stage':row.get('stage'),'runtime_result':E.score(),'terminated':result.terminated,'termination_reason':result.termination_reason}
    journal.write(json.dumps(sample)+'\n');save('progress.json',dict(sample,elapsed_s=time.monotonic()-started))
    if result.terminated:break
  E=d3_kit.ENGINE;score=E.score();all_actions=E.controls==len(rows)
  checks={'all_saved_robot_controls_executed':all_actions,'all_runtime_channels_verified':all(E.history.validated.values()),'genuine20Hz_within_original_budget':abs(E.physics_s-E.controls*.05)<1e-8 and E.controls<=1604,'no_guard_or_damage_stop':E.termination_reason() in (None,'time_limit'),'matching_runtime_fullscore':score['score']==1.,'every_action_has_three_real_rgb':frames==E.controls+1 and set(writers)=={'top_cam','left_cam','right_cam'},'immutable_registered_source':hashes(a.out/'source')==expected}
  for name,rgb in obs.images.items():Image.fromarray(rgb).save(a.out/('final-'+name+'.png'))
  report={'kind':__doc__,'checks':checks,'full_goal_demonstrated':all(checks.values()),'runtime_result':score,'final':E.status(),'controls':E.controls,'physics_samples':E.samples,'physics_s':E.physics_s,'frames_per_camera':frames,'elapsed_s':time.monotonic()-started,'model_run':False,'source_relation':relation,'historical_score_transferred':False}
  save('result.json',report)
 except BaseException as exc:
  save('error.json',{'type':type(exc).__name__,'message':str(exc),'frames':frames,'traceback':traceback.format_exc()[-6000:]});raise
 finally:
  for name,w in writers.items():
   try:w.stdin.close();encoder_results[name]=w.wait(timeout=30)
   except Exception as exc:
    encoder_results[name]=type(exc).__name__;w.terminate()
   logs[name].close()
  save('media-finalization.json',{'encoder_returncodes':encoder_results,'frames_per_camera':frames,'all_encoded':bool(writers) and all(x==0 for x in encoder_results.values())})
  if emb is not None:emb.close()
 if not report or not report['full_goal_demonstrated'] or any(x!=0 for x in encoder_results.values()):raise RuntimeError('Required real fullscore/media replay did not pass')
if __name__=='__main__':main()
