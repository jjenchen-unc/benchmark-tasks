"""Actual robot action replay with the SDK's additional evaluator overview camera.
Three policy cameras, physics, robot action processing and task/scorer stay unchanged.
No fixture state/effort/score/timer writes; only ordinary task reset and robot actions.
"""
import argparse,functools,hashlib,json,shutil,subprocess,sys,time
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--task-dir',type=Path,required=True);p.add_argument('--actions',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();a.task_dir=a.task_dir.resolve();a.out=a.out.resolve();
if a.out==a.task_dir or a.task_dir in a.out.parents:raise ValueError('Recording output must be outside the task source')
a.out.mkdir(exist_ok=False);sys.path.insert(0,str(a.task_dir));shutil.copytree(a.task_dir,a.out/'source');shutil.copy2(__file__,a.out/'executed-replay.py')
from inspect_robots import Scene
from inspect_robots.types import Action
from yam_arena.inspect_embodiment import yam_arena_embodiment
from physics_observer import arr
from PIL import Image
import kit

def overview(module):
 cls=module.BimanualYamEmbodiment;old=cls.__init__
 @functools.wraps(old)
 def init(self,*args,**kw):kw['overview_cam']=True;return old(self,*args,**kw)
 cls.__init__=init
kit.Finder.targets['yam_arena.embodiment']=overview;kit.install()
emb=yam_arena_embodiment(task=str(a.task_dir/'task.py'),device='cpu',object_jitter=0,control_hz=20,cam_width=320,cam_height=176);writers={};logs={};rows=[];started=time.monotonic()
try:
 instruction=json.loads((a.task_dir/'task.json').read_text())['instruction'];obs=emb.reset(Scene(id='B5-scripted-replay-with-evaluator-overview',instruction=instruction),seed=42);base=emb._base;cam=base.scene['overview_cam'];cam.set_world_poses_from_view(np.array([[1.05,1.15,1.55]]),np.array([[.35,0.,1.0]]))
 for _ in range(30):base.sim.render()
 cam.update(0.,force_recompute=True)
 def frames():return dict(obs.images,overview_cam=np.asarray(arr(cam.data.output['rgb'])[0,...,:3],dtype=np.uint8))
 for name,v in frames().items():Image.fromarray(v).save(a.out/('initial-'+name+'.png'))
 ff=shutil.which('ffmpeg')
 if not ff:
  import imageio_ffmpeg
  ff=imageio_ffmpeg.get_ffmpeg_exe()
 for name,v in frames().items():
  h,w=v.shape[:2];log=open(a.out/(name+'-encode.log'),'wb');writers[name]=subprocess.Popen([ff,'-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s',str(w)+'x'+str(h),'-r','20','-i','pipe:0','-an','-c:v','libx264','-threads','1','-pix_fmt','yuv420p','-crf','18','-movflags','+faststart',str(a.out/(name+'.mp4'))],stdin=subprocess.PIPE,stderr=log,stdout=subprocess.DEVNULL);logs[name]=log
 actions=json.loads(a.actions.read_text());assert len(actions)<=1428
 with (a.out/'journal.jsonl').open('w',buffering=1) as journal:
  for i,command in enumerate(actions,1):
   assert np.asarray(command).shape==(14,) and np.isfinite(command).all();result=emb.step(Action(data=np.array(command)));obs=result.observation;ob=kit._OBSERVER;item={'step':i,'action':command,'measurement':ob.history.measurement(),'score':ob.result()};journal.write(json.dumps(item)+'\n');rows.append(item)
   for name,v in frames().items():writers[name].stdin.write(v.tobytes())
   if result.terminated:raise RuntimeError(str((result.termination_reason,result.info)))
 status=kit.current_status();assert status['score']==1. and status['full_credit'],status
 for name,v in frames().items():Image.fromarray(v).save(a.out/('final-'+name+'.png'))
 report={'kind':'robot-actuated scripted action replay with additional SDK evaluator overview camera; no model call','score':status,'measurement':kit._OBSERVER.history.measurement(),'steps':len(actions),'source_actions_sha256':hashlib.sha256(a.actions.read_bytes()).hexdigest(),'policy_camera_names_unchanged':list(obs.images),'overview_camera_only_added_for_recording':True,'overview_eye_w_m':[1.05,1.15,1.55],'overview_target_w_m':[.35,0.,1.0],'model_calls':0,'elapsed_s':time.monotonic()-started};(a.out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
except BaseException as exc:
 (a.out/'error.json').write_text(json.dumps({'type':type(exc).__name__,'message':str(exc),'last':rows[-1] if rows else None},indent=2)+'\n');raise
finally:
 codes={}
 for name,w in writers.items():w.stdin.close();codes[name]=w.wait(timeout=30);logs[name].close()
 (a.out/'finalization.json').write_text(json.dumps({'encoder_returncodes':codes,'steps':len(rows)})+'\n');emb.close()
