"""Portable task-local Arena hooks; imported before cold SDK scene/action modules.
Only environment passive mechanisms write task efforts/constraints. Policies use14D.
"""
from dataclasses import dataclass,replace
from pathlib import Path
import importlib.abc,importlib.util,json,math,sys
import numpy as np
from yam_arena.task_file import Box,ObjectState
from motor_limits import patch_actions
from mechanics import rotation
HERE=Path(__file__).resolve().parent
FIXTURE=json.loads((HERE/'fixture.json').read_text());_SPECS={};ENGINE=None
@dataclass(frozen=True)
class Component(Box):pass
def arr(x):
 if hasattr(x,'torch'):x=x.torch
 elif type(x).__module__.startswith('warp'):
  import warp as wp
  x=wp.to_torch(x)
 return x.detach().cpu().numpy().copy() if hasattr(x,'detach') else np.asarray(x).copy()
def root_body(name):return next(b for b in FIXTURE['bodies'] if b['name']==name)
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
  from pxr import UsdGeom,UsdPhysics,Sdf
  from isaaclab.sim.utils import bind_visual_material
  u=sim.UsdFileCfg(usd_path=str(HERE/'assets'/f'{spec.name}.usda'),activate_contact_sensors=True)
  prim=u.func(path,u,translation=translation,orientation=orientation,**kw);stage=prim.GetStage()
  for p in Usd.PrimRange(prim):
   if p.IsA(UsdGeom.Gprim):
    c=UsdGeom.Gprim(p).GetDisplayColorAttr().Get()
    if c:
     mp=str(p.GetPath())+'_look';m=sim.PreviewSurfaceCfg(diffuse_color=tuple(float(v) for v in c[0]),roughness=.7);m.func(mp,m);bind_visual_material(str(p.GetPath()),mp)
  envroot=path.rsplit('/',1)[0]
  # Reproduce source shape contact masks across independently spawned assets.
  def owner(b):
   while b['parent'] is not None:b=next(x for x in FIXTURE['bodies'] if x['name']==b['parent'])
   return b['name']
  for b in FIXTURE['bodies']:
   if owner(b)!=spec.name:continue
   for g in b['geoms']:
    if not (g['contype'] or g['conaffinity']):continue
    gp=stage.GetPrimAtPath(path+'/links/'+b['name']+'/'+g['name'])
    rel=UsdPhysics.FilteredPairsAPI.Apply(gp).CreateFilteredPairsRel()
    for other in FIXTURE['bodies']:
     for h in other['geoms']:
      if not (h['contype'] or h['conaffinity']):continue
      if not ((g['contype']&h['conaffinity']) or (h['contype']&g['conaffinity'])):
       rel.AddTarget(Sdf.Path(envroot+'/'+owner(other)+'/links/'+other['name']+'/'+h['name']))
    if g['contype']==2 and g['conaffinity']==2:
     for external in ['LeftArm','RightArm','table','floor','stand_foot','stand_mast','stand_arm','camera_housing','left_b6_pedestal','right_b6_pedestal','body_right_bracket_arm','body_right_bracket_post']:
      rel.AddTarget(Sdf.Path(envroot+'/'+external))
  return prim
 # USD traversal is deferred until after AppLauncher.
 from pxr import Usd
 @configclass
 class SpawnCfg(SpawnerCfg):func:object=clone(spawn)
 b=root_body(spec.name);q=b['quat'];pose=Pose(position_xyz=spec.pos,rotation_xyzw=tuple(q[1:]+q[:1]))
 if spec.name.startswith(('lens_','body_')):
  class Part(Object):
   def _generate_articulation_cfg(self):
    js=[j for b in FIXTURE['bodies'] if b['name'].startswith(spec.name) for j in b['joints']]
    actuators={j['name']:ImplicitActuatorCfg(joint_names_expr=[j['name']],stiffness=j['stiffness'],damping=j['damping']) for j in js}
    cfg=ArticulationCfg(prim_path=self.prim_path,spawn=self.spawner_cfg,actuators=actuators,init_state=ArticulationCfg.InitialStateCfg(joint_pos={'.*':0.},joint_vel={'.*':0.}))
    return self._add_initial_pose_to_cfg(cfg)
  cls,typ=Part,ObjectType.ARTICULATION
 else:cls,typ=Object,ObjectType.RIGID
 return cls(name=spec.name,object_type=typ,spawner_cfg=SpawnCfg(),initial_pose=pose)
def patch_scene(module):
 if getattr(module,'_b6_patched',False):return
 old_object,old_states,old_build=module.task_object,module.object_states,module.build_task_environment
 def task_object(spec):
  if isinstance(spec,Component):_SPECS[spec.name]=spec;return object_for(spec)
  return old_object(spec)
 def object_states(env,names):
  mine=[n for n in names if n in _SPECS];out=old_states(env,[n for n in names if n not in mine]) if len(mine)<len(names) else {}
  origin=arr(env.unwrapped.scene.env_origins)[0]
  for n in mine:
   asset=env.unwrapped.scene[n];p=arr(asset.data.root_pos_w)[0]-origin;q=arr(asset.data.root_quat_w)[0];lo=[];hi=[]
   if hasattr(asset,'body_names'):
    poses=dict(zip(asset.body_names,zip(arr(asset.data.body_pos_w)[0]-origin,arr(asset.data.body_quat_w)[0])))
   else:poses={n:(p,q)}
   for b in FIXTURE['bodies']:
    if b['name'] not in poses:continue
    bp,bq=poses[b['name']];R=rotation(bq)
    for g in b['geoms']:
     gp=bp+R@np.array(g['pos']);Q=R@rotation(np.r_[g['quat'][1:],g['quat'][0]]);sz=g['size']
     extent=abs(Q)@np.array(sz) if g['kind']=='box' else (np.ones(3)*sz[0] if g['kind']=='sphere' else abs(Q[:,2])*sz[1]+sz[0]*np.sqrt(np.maximum(0,1-Q[:,2]**2)))
     lo.append(gp-extent);hi.append(gp+extent)
   out[n]=ObjectState(pos=p,quat_xyzw=q,lin_vel=arr(asset.data.root_lin_vel_w)[0],ang_vel=arr(asset.data.root_ang_vel_w)[0],low=np.min(lo,axis=0),high=np.max(hi,axis=0))
  return out
 def build(*a,**kw):
  env=old_build(*a,**kw);task=kw.get('task')
  if task and any(isinstance(x,Component) for x in task.objects):
   previous=env.env_cfg_callback
   def configure(cfg):
    if previous:cfg=previous(cfg)
    cfg.sim.dt=.002;cfg.decimation=25;cfg.sim.render_interval=25
    from isaaclab.sensors import JointWrenchSensorCfg,ContactSensorCfg
    for side in ['left','right']:
     setattr(cfg.scene,side+'_b6_wrench',JointWrenchSensorCfg(prim_path='{ENV_REGEX_NS}/'+side.capitalize()+'Arm'))
    for n in ['body_left','body_right']:setattr(cfg.scene,n+'_b6_wrench',JointWrenchSensorCfg(prim_path='{ENV_REGEX_NS}/'+n))
    cfg.scene.b6_contact_processing=ContactSensorCfg(prim_path='{ENV_REGEX_NS}/lens_blue/links/lens_blue')
    for robot in [cfg.scene.left_robot,cfg.scene.right_robot]:robot.actuators['gripper'].joint_effort_limit=40.;robot.spawn.activate_contact_sensors=True
    return cfg
   env.env_cfg_callback=configure
  return env
 module.task_object,module.object_states,module.build_task_environment=task_object,object_states,build;module._b6_patched=True

def patch_embodiment(module):
 cls=module.YamArenaEmbodiment
 if getattr(cls,'_b6_patched',False):return
 old_init,old_reset,old_step,old_task_step,old_close=cls.__init__,cls.reset,cls.step,cls._task_step,cls.close
 def active(self):return self.task and any(isinstance(x,Component) for x in self.task.objects)
 def init(self,*a,**kw):
  old_init(self,*a,**kw)
  if active(self):
   # Both arm bases have explicit rigid pedestals. No fixture geometry changes.
   self.rig=replace(self.rig,bases={s:(0.,y,.90) for s,y in [('left',.305),('right',-.305)]})
 def reset(self,*a,**kw):
  global ENGINE
  if not active(self):return old_reset(self,*a,**kw)
  if self.object_jitter!=0 or not math.isclose(self.info.control_hz,20.):raise ValueError('B6 requires fixed object_jitter=0 and20Hz')
  if str(self.device)!='cpu':raise ValueError('B6 foundation requests CPU PhysX with GPU rendering; reactions need calibration')
  self._ensure_sim(None)
  for s in ['left','right']:self._base.scene[s+'_robot']._b6_total_effort_enabled=True
  from b6_physics import Physics
  if ENGINE is None or ENGINE.base is not self._base:
   ENGINE=Physics(self,FIXTURE,_SPECS)
   write,update,reset_env=self._base.scene.write_data_to_sim,self._base.scene.update,self._env.reset
   def wrapped_write(*a,**kw):ENGINE.pre_step(float(self._base.sim.get_physics_dt()));return write(*a,**kw)
   def wrapped_update(dt,*a,**kw):
    result=update(dt,*a,**kw);ENGINE.post_step(float(dt));return result
   def wrapped_reset(*a,**kw):
    ENGINE.prepare_reset();result=reset_env(*a,**kw);ENGINE.restore_joints();return result
   self._base.scene.write_data_to_sim=wrapped_write;self._base.scene.update=wrapped_update;self._env.reset=wrapped_reset
  ENGINE.settling=True
  obs=old_reset(self,*a,**kw);ENGINE.begin_episode();return obs
 def step(self,action):
  if active(self):
   if ENGINE.done:raise RuntimeError('B6 episode terminated; reset before stepping')
   value=np.asarray(action.data,float)
   if value.shape!=(14,) or not np.isfinite(value).all():raise ValueError('B6 requires genuine finite14D action')
  return old_step(self,action)
 def task_step(self):
  if not active(self):return old_task_step(self)
  ENGINE.controls+=1;result=old_task_step(self);reason=ENGINE.termination_reason();ENGINE.done=bool(reason or result.terminated)
  return replace(result,terminated=ENGINE.done,termination_reason=reason or result.termination_reason,info=dict(result.info,b6=ENGINE.status()))
 def close(self,*a,**kw):
  if ENGINE is not None and ENGINE.emb is self:ENGINE.close()
  return old_close(self,*a,**kw)
 cls.__init__,cls.reset,cls.step,cls._task_step,cls.close=init,reset,step,task_step,close;cls._b6_patched=True

def current_score():return ENGINE.status()['score'] if ENGINE else 0.
class Finder(importlib.abc.MetaPathFinder):
 targets={'yam_arena.scene':patch_scene,'yam_arena.actions':patch_actions,'yam_arena.inspect_embodiment':patch_embodiment}
 def find_spec(self,fullname,path,target=None):
  if fullname not in self.targets:return None
  sys.meta_path.remove(self)
  try:spec=importlib.util.find_spec(fullname)
  finally:sys.meta_path.insert(0,self)
  if not spec or not spec.loader:return None
  original,patch=spec.loader.exec_module,self.targets[fullname]
  def execute(m):original(m);patch(m)
  spec.loader.exec_module=execute;return spec
def install():
 for name,patch in Finder.targets.items():
  if name in sys.modules:patch(sys.modules[name])
 if not any(isinstance(x,Finder) for x in sys.meta_path):sys.meta_path.insert(0,Finder())
