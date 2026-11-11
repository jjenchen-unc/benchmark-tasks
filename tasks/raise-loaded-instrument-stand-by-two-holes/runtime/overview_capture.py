"""Recording-only SDK overview_cam (fourth camera excluded from policy). Policy images stay 3x320x176."""
import functools,hashlib,json,shutil,subprocess
from pathlib import Path
import numpy as np
from overview_camera import overview
POLICY_CAMERAS={'top_cam','left_cam','right_cam'}
_V=overview();EYE=_V['pos'];TARGET=_V['target']
def install():
 import kit
 prev=kit.Finder.targets.get('yam_arena.embodiment')
 def add(module):
  if prev:prev(module)
  cls=module.BimanualYamEmbodiment
  if getattr(cls,'_pins_overview',False):return
  o=cls.__init__
  @functools.wraps(o)
  def init(self,*a,**k):
   k['overview_cam']=True;result=o(self,*a,**k);cfg=self.camera_config.overview_cam
   cfg.width=_V['width'];cfg.height=_V['height'];cfg.spawn.focal_length=_V['focal_mm'];cfg.spawn.horizontal_aperture=_V['horizontal_aperture_mm']
   return result
  cls.__init__=init;cls._pins_overview=True
 kit.Finder.targets['yam_arena.embodiment']=add;kit.install()
def snapshot(emb):
 import kit;E=kit.ENGINE
 state=dict(controls=E.history.controls,time=E.history.time,hold=E.history.hold,physics_steps=int(emb._base.sim.get_physics_step_count()),env_steps=int(emb._base._sim_step_counter))
 state['assets']={}
 for name in ['pins','left_robot','right_robot']:
  d=emb._base.scene[name].data
  state['assets'][name]={n:np.asarray(kit.arr(getattr(d,n))).tolist() for n in ['joint_pos','joint_vel','body_link_state_w']}
 return state
class OverviewCapture:
 def __init__(self,emb,out,obs):
  from PIL import Image
  if set(obs.images)!=POLICY_CAMERAS:raise RuntimeError('policy camera inventory changed')
  for k,v in obs.images.items():
   if tuple(np.asarray(v).shape[:2])!=(176,320):raise RuntimeError('policy image size changed: '+k)
  self.emb=emb;self.out=Path(out);self.cam=emb._base.scene['overview_cam'];self.frames=0;self.closed=False
  b=snapshot(emb)
  if b['controls']!=0 or b['time']!=0:raise RuntimeError('overview requires untouched reset')
  self.cam.set_world_poses_from_view(np.array([EYE]),np.array([TARGET]))
  for _ in range(30):emb._base.sim.render()
  a=snapshot(emb)
  if a!=b:raise RuntimeError('render-only setup altered physics/controls')
  (self.out/'overview-zero-step.json').write_text(json.dumps(dict(before=b,after=a,exact_state_unchanged=True,extra_physics_steps=0,eye=EYE,target=TARGET,cpu_projection=_V['coverage']),indent=1))
  f=self.frame();Image.fromarray(f).save(self.out/'overview-setup.png');h,w=f.shape[:2]
  ff=shutil.which('ffmpeg')
  if not ff:
   import imageio_ffmpeg;ff=imageio_ffmpeg.get_ffmpeg_exe()
  self.log=(self.out/'overview-encoder.log').open('wb')
  self.w=subprocess.Popen([ff,'-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s',f'{w}x{h}','-r','20','-i','pipe:0','-an','-c:v','libx264','-threads','1','-pix_fmt','yuv420p','-crf','24','-movflags','+faststart',str(self.out/'overview_cam.mp4')],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=self.log)
  self.w.stdin.write(f.tobytes());self.frames=1
 def frame(self):
  import kit
  self.cam.update(0.,force_recompute=True);raw=np.asarray(kit.arr(self.cam.data.output['rgb']))
  if raw.ndim==4:raw=raw[0]
  if raw.ndim!=3 or raw.shape!=(360,640,4) and raw.shape!=(360,640,3) or raw.dtype!=np.uint8:raise RuntimeError('unexpected overview format')
  return np.ascontiguousarray(raw[...,:3])
 def append(self,obs):
  if set(obs.images)!=POLICY_CAMERAS:raise RuntimeError('policy camera inventory changed')
  b=snapshot(self.emb);self.emb._base.sim.render();f=self.frame()
  if snapshot(self.emb)!=b:raise RuntimeError('overview render changed physics')
  self.w.stdin.write(f.tobytes());self.frames+=1
 def close(self):
  if self.closed:return
  self.closed=True;self.w.stdin.close();c=self.w.wait(timeout=40);self.log.close();p=self.out/'overview_cam.mp4'
  (self.out/'overview-capture.json').write_text(json.dumps(dict(kind='recording overview, not a policy observation',frames=self.frames,returncode=c,sha256=hashlib.sha256(p.read_bytes()).hexdigest(),bytes=p.stat().st_size),indent=1))
  if c!=0:raise RuntimeError('overview encode failed')
