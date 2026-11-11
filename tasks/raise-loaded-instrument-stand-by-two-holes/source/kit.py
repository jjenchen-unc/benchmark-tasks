"""Arena adapter. Import task.py before cold SDK modules so hooks actually execute."""
from dataclasses import dataclass,replace
import importlib.abc,importlib.util,json,math,sys
from pathlib import Path
import numpy as np
from yam_arena.task_file import Box,ObjectState
from motor_limits import patch_actions
HERE=Path(__file__).resolve().parent
F=json.loads((HERE/'fixture.json').read_text());JOINTS=json.loads((HERE/'joints.json').read_text());ENGINE=None
@dataclass(frozen=True)
class Stand(Box):pass
def arr(x):
 x=getattr(x,'torch',x)
 if hasattr(x,'detach'):return x.detach().cpu().numpy()
 if hasattr(x,'numpy'):return x.numpy()
 return np.asarray(x)
def rotation(q):
 x,y,z,w=q;return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def initial():
 q={j['name']:0. for j in JOINTS};ql=.02475;qr=.02525;th=math.atan2(qr-ql,.3);q.update(legL_z=ql,legR_z=qr,plate_y=-.15+.15*math.cos(th)-.01*math.sin(th),plate_z=.945+ql+.17+.15*math.sin(th)+.01*math.cos(th)-1.125,plate_roll=th,slotR_y=.3/math.cos(th)-.3);return q

def object_for(spec):
 import isaaclab.sim as sim
 from isaaclab.actuators import ImplicitActuatorCfg
 from isaaclab.assets import ArticulationCfg
 from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
 from isaaclab.sim.utils import clone
 from isaaclab.utils import configclass
 from isaaclab_arena.assets.object import Object
 from isaaclab_arena.assets.object_type import ObjectType
 from isaaclab_arena.utils.pose import Pose
 def spawn(path,cfg,translation=None,orientation=None,**kw):
  from pxr import UsdPhysics,Sdf
  usd=sim.UsdFileCfg(usd_path=str(HERE/'stand.usda'),activate_contact_sensors=True);prim=usd.func(path,usd,translation=translation,orientation=orientation,**kw);env=path.rsplit('/',1)[0]
  for b in F['bodies']:
   for g in b['geoms']:
    if any(g['mask']) and not (g['mask'][0]&1 or g['mask'][1]&1):
     branch='links' if b['mass']>0 else 'static';s=prim.GetStage().GetPrimAtPath(path+'/'+branch+'/'+b['name']+'/'+g['name']);r=UsdPhysics.FilteredPairsAPI.Apply(s).CreateFilteredPairsRel()
     for n in ['LeftArm','RightArm','table','floor','stand_foot','stand_mast','stand_arm','camera_housing','lamp','left_robot_pedestal','right_robot_pedestal']:r.AddTarget(Sdf.Path(env+'/'+n))
  return prim
 @configclass
 class SpawnCfg(SpawnerCfg):func:object=clone(spawn)
 class Part(Object):
  def _generate_articulation_cfg(self):
   cfg=ArticulationCfg(prim_path=self.prim_path,spawn=self.spawner_cfg,actuators={j['name']:ImplicitActuatorCfg(joint_names_expr=[j['name']],stiffness=j['K'],damping=j['D']) for j in JOINTS},init_state=ArticulationCfg.InitialStateCfg(joint_pos=initial(),joint_vel={'.*':0.}));return self._add_initial_pose_to_cfg(cfg)
 return Part(name=spec.name,object_type=ObjectType.ARTICULATION,spawner_cfg=SpawnCfg(),initial_pose=Pose(position_xyz=spec.pos,rotation_xyzw=(0.,0.,math.sin(spec.yaw/2),math.cos(spec.yaw/2))))

def patch_scene(m):
 if getattr(m,'_pins',False):return
 old_obj,old_states,old_build=m.task_object,m.object_states,m.build_task_environment
 def obj(s):return object_for(s) if isinstance(s,Stand) else old_obj(s)
 def states(env,names):
  out=old_states(env,[n for n in names if n!='pins'])
  if 'pins' in names:
   asset=env.unwrapped.scene['pins'];origin=arr(env.unwrapped.scene.env_origins)[0];p=arr(asset.data.root_pos_w)[0]-origin;quat=arr(asset.data.root_quat_w)[0];rootR=rotation(quat);poses=dict(zip(asset.body_names,zip(arr(asset.data.body_pos_w)[0]-origin,arr(asset.data.body_quat_w)[0])));lows=[];highs=[]
   for b in F['bodies']:
    if b['name'] in poses:bp,bq=poses[b['name']];R=rotation(bq)
    else:bp=p+rootR@(np.array(b['pos'])-[0,0,.8]);R=rootR@rotation(np.r_[b['quat'][1:],b['quat'][0]])
    for g in b['geoms']:
     c=bp+R@g['pos'];Q=R@rotation(np.r_[g['quat'][1:],g['quat'][0]]);sz=g['size']
     if g['kind']=='box':extent=abs(Q)@sz
     else:
      z=Q[:,2];extent=sz[1]*abs(z)+(sz[0] if g['kind']=='capsule' else sz[0]*np.sqrt(np.maximum(0,1-z*z)))
     lows.append(c-extent);highs.append(c+extent)
   out['pins']=ObjectState(pos=p,quat_xyzw=quat,lin_vel=arr(asset.data.root_lin_vel_w)[0],ang_vel=arr(asset.data.root_ang_vel_w)[0],low=np.min(lows,axis=0),high=np.max(highs,axis=0))
  return out
 def build(*a,**kw):
  env=old_build(*a,**kw);task=kw.get('task')
  if task and any(isinstance(o,Stand) for o in task.objects):
   prior=env.env_cfg_callback
   def configure(cfg):
    if prior:cfg=prior(cfg)
    cfg.sim.dt=.002;cfg.decimation=25;cfg.sim.render_interval=25
    from isaaclab.sensors import JointWrenchSensorCfg
    for side in ['left','right']:
     robot=getattr(cfg.scene,side+'_robot');p=list(robot.init_state.pos);p[0]+=.32;p[2]+=.40;robot.init_state.pos=tuple(p);robot.actuators['gripper'].joint_effort_limit=40.;robot.spawn.activate_contact_sensors=True
     setattr(cfg.scene,side+'_pins_wrench',JointWrenchSensorCfg(prim_path='{ENV_REGEX_NS}/'+side.title()+'Arm'))
    return cfg
   env.env_cfg_callback=configure
  return env
 m.task_object,m.object_states,m.build_task_environment=obj,states,build;m._pins=True

def patch_embodiment(m):
 cls=m.YamArenaEmbodiment
 if getattr(cls,'_pins',False):return
 reset0,step0,task0,close0=cls.reset,cls.step,cls._task_step,cls.close
 def active(s):return s.task is not None and any(isinstance(o,Stand) for o in s.task.objects)
 def reset(s,*a,**kw):
  global ENGINE
  if not active(s):return reset0(s,*a,**kw)
  if s.object_jitter!=0 or abs(s.info.control_hz-20)>1e-8:raise ValueError('fixed pins setup requires object_jitter=0 and control_hz=20')
  s._ensure_sim(None)
  for side in ['left','right']:s._base.scene[side+'_robot']._pins_total_effort_enabled=True
  from pins_physics import Physics
  if ENGINE is None or ENGINE.base is not s._base:
   ENGINE=Physics(s);wr=s._base.scene.write_data_to_sim;up=s._base.scene.update;er=s._env.reset
   def write(*args,**kwargs):ENGINE.pre();return wr(*args,**kwargs)
   def update(dt,*args,**kwargs):v=up(dt,*args,**kwargs);ENGINE.post(float(dt));return v
   def env_reset(*args,**kwargs):v=er(*args,**kwargs);ENGINE.reset();return v
   s._base.scene.write_data_to_sim=write;s._base.scene.update=update;s._env.reset=env_reset
  ENGINE.settling=True;obs=reset0(s,*a,**kw);ENGINE.begin();s._pins_done=False;return obs
 def step(s,action):
  if active(s):
   if getattr(s,'_pins_done',True):raise RuntimeError('reset required before step or after termination')
   v=np.asarray(action.data,float)
   if v.shape!=(14,) or not np.isfinite(v).all():raise ValueError('genuine finite14D robot joint actions required')
  return step0(s,action)
 def task_step(s):
  if not active(s):return task0(s)
  ENGINE.history.controls+=1;r=task0(s);st=ENGINE.status();failure=st['failure'];done=bool(failure or ENGINE.history.controls>=4572 or st['score']==1);s._pins_done=done
  return replace(r,terminated=done,termination_reason=failure or ('time_limit' if ENGINE.history.controls>=4572 else 'success' if st['score']==1 else None),info=dict(r.info,score=st['score']))
 def close(s,*a,**kw):
  if ENGINE is not None and ENGINE.emb is s:ENGINE.close()
  return close0(s,*a,**kw)
 cls.reset,cls.step,cls._task_step,cls.close=reset,step,task_step,close;cls._pins=True
class Finder(importlib.abc.MetaPathFinder):
 targets={'yam_arena.scene':patch_scene,'yam_arena.actions':patch_actions,'yam_arena.inspect_embodiment':patch_embodiment}
 def find_spec(self,fullname,path,target=None):
  if fullname not in self.targets:return None
  sys.meta_path.remove(self)
  try:s=importlib.util.find_spec(fullname)
  finally:sys.meta_path.insert(0,self)
  if s is None or s.loader is None:return None
  execute,patch=s.loader.exec_module,self.targets[fullname]
  def run(m):execute(m);patch(m)
  s.loader.exec_module=run;return s
def install():
 for n,p in Finder.targets.items():
  if n in sys.modules:p(sys.modules[n])
 if not any(isinstance(x,Finder) for x in sys.meta_path):sys.meta_path.insert(0,Finder())
def current_score():return 0. if ENGINE is None else ENGINE.status()['score']
