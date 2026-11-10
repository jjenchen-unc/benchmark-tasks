"""Portable genuine S5 reference/setup; source checks before SDK boundary."""
from pathlib import Path
import sys,os,argparse,time,traceback,functools
O=Path(__file__).resolve().parent.parent/'s5-static-table-portable-v13';C=O/'candidate'
sys.path.insert(0,str(O));sys.path.insert(0,str(C))
from protocol import check,receipt,CHANNELS
from strict_json import dumps
p=argparse.ArgumentParser();p.add_argument('phase',choices=['demo','setup']);p.add_argument('--review',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
m=check();review=receipt(a.review);os.environ['S5_ROOT_REVIEW']=str(a.review.resolve());task_path=O/'model_task.py'
from verify_runtime import verify
verify()
out=a.out.resolve()
if out.exists() or out.is_relative_to(C):raise ValueError('fresh output required')
from yam_arena.task_file import load
loaded=load(task_path);task=loaded.module
if len(loaded.objects)!=8:raise ValueError('native eight objects required')
sys.audit('s5.task_constructed',str(loaded.path),len(loaded.objects))
# Pure YAM construction precedes AppLauncher. No simulator import above.
from isaaclab.app import AppLauncher
old=AppLauncher.__init__
@functools.wraps(old)
def init(self,*args,**kw):
 kw['kit_args']=(kw.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip();return old(self,*args,**kw)
AppLauncher.__init__=init
from inspect_robots import Scene
from yam_arena.inspect_embodiment import yam_arena_embodiment
from observer import observe,emit
import s5_kit as kit,s5_rules
if s5_rules.VALIDATED_CHANNELS!=frozenset(CHANNELS):raise RuntimeError('approved channels required')
os.environ['S5_TRACE']=str(out/'physics.jsonl')
out.mkdir(parents=True,exist_ok=False);emb=None;capture=None;start=time.monotonic()
r=dict(kind='ACTUAL_S5_RUNTIME',phase=a.phase,manifest=m,promoted_channels=sorted(CHANNELS),completed=False,error=None)
try:
 emb=yam_arena_embodiment(task=str(task_path),device='cpu',object_jitter=0,control_hz=20,cam_width=320,cam_height=176)
 obs=emb.reset(Scene(id='s5-final-'+a.phase,instruction=loaded.docs),seed=20261109)
 from s5_kit import arr
 hardware={}
 for side in ('left','right'):
  robot=emb._base.scene[side+'_robot'];names=robot.joint_names
  forces=arr(robot.root_view.get_dof_max_forces()).tolist()[0];stiffness=arr(robot.root_view.get_dof_stiffnesses()).tolist()[0]
  force_map=dict(zip(names,forces));stiffness_map=dict(zip(names,stiffness))
  assert getattr(robot,'_s5_total_effort_enabled',False)
  for i in range(1,9):assert abs(force_map['joint'+str(i)]-(27 if i<=3 else 7 if i<=6 else 40))<1e-5
  for i in range(1,7):assert abs(stiffness_map['joint'+str(i)]-(80 if i<=3 else 10))<1e-5
  hardware[side]=dict(names=names,forces=forces,stiffness=stiffness,total_effort_enabled=True)
 (out/'actual-hardware.json').write_text(dumps(hardware,indent=2)+'\n')
 emit(out/'journal.jsonl',observe(emb,obs,'reset'))
 from PIL import Image
 for name,rgb in obs.images.items():
  if name not in {'top_cam','left_cam','right_cam'}:raise ValueError('unexpected camera')
  Image.fromarray(rgb).save(out/(name+'.png'))
 if a.phase=='demo':
  from video_capture import Capture
  capture=Capture(out,obs.images);original=emb.step;last=[obs]
  def step(action):
   emit(out/'pre-actions.jsonl',observe(emb,last[0],'before_action',action.data))
   result=original(action);last[0]=result.observation;capture.append(result.observation.images)
   emit(out/'journal.jsonl',observe(emb,result.observation,'after_action',action.data));return result
  emb.step=step
  from reference import run
  with (out/'robot-actions.jsonl').open('x') as f:r['controller_result']=run(emb,obs,f)
 r['completed']=True
except BaseException as exc:
 r['error']=type(exc).__name__+': '+str(exc);r['traceback']=traceback.format_exc();raise
finally:
 r['wall_s']=time.monotonic()-start
 if kit.ENGINE is not None:r['final_status']=kit.ENGINE.status()
 try:
  if capture is not None:r['media']=capture.close()
 except BaseException as exc:r['close_error']=type(exc).__name__+': '+str(exc)
 try:
  if emb is not None:emb.close()
 except BaseException as exc:r['close_error']=type(exc).__name__+': '+str(exc)
 (out/'result.json').write_text(dumps(r,indent=2)+'\n')
