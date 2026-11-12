"""SDK evaluator overview for recording only; original policy images stay fixed."""
from pathlib import Path
import functools,hashlib,json,shutil,subprocess
import numpy as np
EYE=[1.05,1.15,1.65]
TARGET=[.40,-.17,.95]
POLICY_CAMERAS={'top_cam','left_cam','right_cam'}

def install():
 import drawers_kit as kit
 def add_overview(module):
  cls=module.BimanualYamEmbodiment
  if getattr(cls,'_drawers_recording_overview',False):return
  original=cls.__init__
  @functools.wraps(original)
  def init(self,*args,**kw):kw['overview_cam']=True;return original(self,*args,**kw)
  cls.__init__=init;cls._drawers_recording_overview=True
 kit.Finder.targets['yam_arena.embodiment']=add_overview;kit.install()

def physical_snapshot(emb):
 import drawers_kit as kit
 from drawers_math import arr
 engine=kit.ENGINE
 return {'controls':engine.controls,'samples':engine.history.samples,'physics_s':engine.history.time,'hold_s':engine.history.hold,
         'robots':{side:np.array(arr(emb._base.scene[side+'_robot'].data.joint_pos),copy=True).tolist() for side in ('left','right')},
         'task_poses':{name:[np.array(v,copy=True).tolist() for v in state] for name,state in engine.poses().items()}}

class OverviewCapture:
 def __init__(self,emb,out,observation):
  from drawers_math import arr
  from PIL import Image
  if set(observation.images)!=POLICY_CAMERAS:raise RuntimeError('Policy camera inventory changed')
  self.emb=emb;self.out=Path(out);self.cam=emb._base.scene['overview_cam'];self.frames=0;self.closed=False;self.frame_shape=None
  before=physical_snapshot(emb)
  if before['controls']!=0 or before['samples']!=0 or before['physics_s']!=0. or before['hold_s']!=0.:raise RuntimeError('Overview requires untouched reset observation')
  self.cam.set_world_poses_from_view(np.array([EYE]),np.array([TARGET]))
  for _ in range(30):emb._base.sim.render()
  self.cam.update(0.,force_recompute=True)
  after=physical_snapshot(emb)
  if before!=after:raise RuntimeError('Render-only setup altered physical state/counters')
  self.reset_proof={'before':before,'after':after,'exact_state_unchanged':True,'warmup_render_calls':30,'extra_physics_steps':0,'policy_camera_names':sorted(observation.images)}
  (self.out/'overview-zero-step.json').write_text(json.dumps(self.reset_proof,indent=2)+'\n')
  frame=self.frame();Image.fromarray(frame).save(self.out/'overview-setup.png');h,w=frame.shape[:2]
  ff=shutil.which('ffmpeg')
  if not ff:
   import imageio_ffmpeg
   ff=imageio_ffmpeg.get_ffmpeg_exe()
  self.log=(self.out/'overview-encoder.log').open('wb')
  self.writer=subprocess.Popen([ff,'-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s',f'{w}x{h}','-r','20','-i','pipe:0','-an','-c:v','libx264','-threads','1','-pix_fmt','yuv420p','-crf','24','-maxrate','1800k','-bufsize','3600k','-movflags','+faststart',str(self.out/'overview_cam.mp4')],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=self.log)
  self.writer.stdin.write(frame.tobytes());self.frames=1
  (self.out/'overview-zero-step.json').write_text(json.dumps(self.reset_proof,indent=2)+'\n')
 def frame(self):
  from drawers_math import arr
  self.cam.update(0.,force_recompute=True);raw=np.asarray(arr(self.cam.data.output['rgb']))
  if raw.ndim==4 and raw.shape[0]==1:raw=raw[0]
  if raw.ndim!=3 or raw.shape[2] not in (3,4) or raw.dtype!=np.uint8:raise RuntimeError('Unexpected actual overview format '+str((raw.shape,str(raw.dtype))))
  frame=raw[...,:3]
  if min(frame.shape[:2])<=0 or any(v%2 for v in frame.shape[:2]):raise RuntimeError('Invalid actual overview dimensions '+str(frame.shape))
  if self.frame_shape is None:self.frame_shape=frame.shape
  elif frame.shape!=self.frame_shape:raise RuntimeError('Actual overview resolution changed')
  return np.ascontiguousarray(frame)
 def append(self,observation):
  if set(observation.images)!=POLICY_CAMERAS:raise RuntimeError('Policy camera inventory changed')
  before=physical_snapshot(self.emb);self.emb._base.sim.render();frame=self.frame();after=physical_snapshot(self.emb)
  if before!=after:raise RuntimeError('Overview render advanced or changed physical state')
  self.writer.stdin.write(frame.tobytes());self.frames+=1
 def close(self):
  if self.closed:return self.report
  self.closed=True;self.writer.stdin.close();code=self.writer.wait(timeout=40);self.log.close();p=self.out/'overview_cam.mp4'
  self.report={'kind':'Actual SDK evaluator overview; not a fourth policy observation','frames':self.frames,'frame_shape':self.frame_shape,'fps':20,'encoder_returncode':code,'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'eye_world_m':EYE,'target_world_m':TARGET,'focal_length_mm':18.,'clipping_range_m':[.05,20.],'initial_render_calls':30,'per_action_extra_render_calls':1,'extra_physics_steps':0,'source_helper_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'policy_camera_names_unchanged':sorted(POLICY_CAMERAS),'reset_state_unchanged':True}
  (self.out/'overview-capture.json').write_text(json.dumps(self.report,indent=2)+'\n')
  if code!=0 or not 0<p.stat().st_size<=50_000_000:raise RuntimeError('Overview encoding/size failed')
  return self.report
