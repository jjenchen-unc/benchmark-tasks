"""B4-only SDK hooks; free task bodies are changed by contact only after reset."""
from dataclasses import dataclass,replace
import importlib.abc,importlib.util,sys,math
import numpy as np
from yam_arena.task_file import Box
from fixture import PARAMETERS,inertia
from motor_adapter import patch_actions
ENGINE=None
@dataclass(frozen=True)
class Bar(Box):label:str='A'
def arr(x):
 if hasattr(x,'torch'):x=x.torch
 elif type(x).__module__.startswith('warp'):
  import warp as wp
  x=wp.to_torch(x)
 return x.detach().cpu().numpy().copy() if hasattr(x,'detach') else np.asarray(x).copy()
def patch_scene(module):
 if getattr(module,'_b4',False):return
 old_object,old_build=module.task_object,module.build_task_environment
 def task_object(spec):
  if not isinstance(spec,Bar):
   obj=old_object(spec)
   if spec.name=='support':
    import isaaclab.sim as sim
    obj.spawner_cfg.physics_material=sim.RigidBodyMaterialCfg(static_friction=.5,dynamic_friction=.5,friction_combine_mode='average')
   return obj
  import isaaclab.sim as sim
  from isaaclab_arena.assets.object import Object
  from isaaclab_arena.assets.object_type import ObjectType
  from isaaclab_arena.utils.pose import Pose
  from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
  from isaaclab.sim.utils import clone
  from isaaclab.utils import configclass
  def spawn(path,cfg,translation=None,orientation=None,**kw):
   from pxr import UsdGeom,UsdPhysics,Gf
   from isaaclab.sim.utils import bind_visual_material
   cube=sim.CuboidCfg(size=(.32,.05,.05),mass_props=sim.MassPropertiesCfg(mass=spec.mass),rigid_props=sim.RigidBodyPropertiesCfg(linear_damping=0.,angular_damping=0.),collision_props=sim.CollisionPropertiesCfg(contact_offset=.0005,rest_offset=0.),physics_material=sim.RigidBodyMaterialCfg(static_friction=.6,dynamic_friction=.6,friction_combine_mode='average'),visual_material=sim.PreviewSurfaceCfg(diffuse_color=spec.color,roughness=.7),activate_contact_sensors=True)
   prim=cube.func(path,cube,translation=translation,orientation=orientation,**kw)
   c,I=inertia(PARAMETERS[spec.label]['mass'],PARAMETERS[spec.label]['com'])
   mass=UsdPhysics.MassAPI.Apply(prim);mass.CreateCenterOfMassAttr(Gf.Vec3f(c,0,0));mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*I))
   # Visual-only black cap occupies the native8mm cap interval, no extra collider/mass.
   cap=UsdGeom.Cube.Define(prim.GetStage(),path+'/cap');cap.CreateSizeAttr(1.);cap.AddTranslateOp().Set(Gf.Vec3d(-.156,0,0));cap.AddScaleOp().Set(Gf.Vec3f(.008,.05001,.05001))
   mat=sim.PreviewSurfaceCfg(diffuse_color=(.08,.08,.09),roughness=.7);mat.func(path+'/cap_material',mat);bind_visual_material(path+'/cap',path+'/cap_material')
   return prim
  @configclass
  class SpawnCfg(SpawnerCfg):func:object=clone(spawn)
  return Object(name=spec.name,object_type=ObjectType.RIGID,spawner_cfg=SpawnCfg(),initial_pose=Pose(position_xyz=tuple(spec.pos),rotation_xyzw=(0.,0.,0.,1.)))
 def build(*a,**kw):
  env=old_build(*a,**kw);task=kw.get('task')
  if task and any(isinstance(x,Bar) for x in task.objects):
   prior=env.env_cfg_callback
   def configure(cfg):
    if prior:cfg=prior(cfg)
    cfg.sim.dt=.002;cfg.decimation=25;cfg.sim.render_interval=25
    from isaaclab.sensors import JointWrenchSensorCfg,ContactSensorCfg
    for side in ('left','right'):
     setattr(cfg.scene,side+'_b4_wrench',JointWrenchSensorCfg(prim_path='{ENV_REGEX_NS}/'+('LeftArm' if side=='left' else 'RightArm')))
     robot=getattr(cfg.scene,side+'_robot');robot.init_state.pos=(0.,.305 if side=='left' else -.305,.97);robot.actuators['gripper'].joint_effort_limit=40.;robot.spawn.activate_contact_sensors=True
    cfg.scene.b4_contacts=ContactSensorCfg(prim_path='{ENV_REGEX_NS}/bar_A')
    return cfg
   env.env_cfg_callback=configure
  return env
 module.task_object,module.build_task_environment=task_object,build;module._b4=True

def patch_embodiment(module):
 cls=module.YamArenaEmbodiment
 if getattr(cls,'_b4',False):return
 old_reset,old_step,old_task=cls.reset,cls.step,cls._task_step
 def active(self):return self.task is not None and any(isinstance(x,Bar) for x in self.task.objects)
 def reset(self,*a,**kw):
  global ENGINE
  if not active(self):return old_reset(self,*a,**kw)
  if self.object_jitter!=0 or not math.isclose(self.info.control_hz,20.):raise ValueError('B4 requires object_jitter=0 and20Hz')
  if ENGINE is not None and ENGINE.trace is not None:ENGINE.trace.close()
  ENGINE=None
  self._ensure_sim(None)
  for side in ('left','right'):self._base.scene[side+'_robot']._b4_total_effort_enabled=True
  obs=old_reset(self,*a,**kw)
  from observer import Observer
  ENGINE=Observer(self)
  if not getattr(self._base.scene,'_b4',False):
   previous=self._base.scene.update
   def update(dt,*args,**kwargs):
    value=previous(dt,*args,**kwargs)
    if ENGINE is not None and ENGINE.base.scene is self._base.scene:ENGINE.sample(float(dt))
    return value
   self._base.scene.update=update;self._base.scene._b4=True
  return obs
 def step(self,action):
  if active(self) and ENGINE is not None:
   if ENGINE.done:raise RuntimeError('B4 terminated: reset required before further physics')
   if ENGINE.history.guard:
    from inspect_robots.types import Action
    # Hold actual arm joints from scene; leave current jaw command untouched.
    from yam_arena.constants import FINGER_OPEN
    values=[]
    for side in ('left','right'):
     asset=self._base.scene[side+'_robot'];positions=arr(asset.data.joint_pos)[0];names=asset.joint_names
     values.extend([float(positions[names.index('joint'+str(i))]) for i in range(1,7)])
     values.append(float(np.mean([positions[names.index('joint7')],positions[names.index('joint8')]])/FINGER_OPEN))
    action=Action(data=np.asarray(values))
  return old_step(self,action)
 def task_step(self):
  if not active(self):return old_task(self)
  ENGINE.controls+=1;result=old_task(self);reason=ENGINE.history.failure
  if ENGINE.controls>=1896:reason=reason or 'time_limit'
  ENGINE.done=bool(reason or result.terminated)
  return replace(result,terminated=ENGINE.done,termination_reason=reason or result.termination_reason,info=dict(result.info,b4=ENGINE.status()))
 cls.reset,cls.step,cls._task_step=reset,step,task_step;cls._b4=True
class Finder(importlib.abc.MetaPathFinder):
 targets={'yam_arena.scene':patch_scene,'yam_arena.actions':patch_actions,'yam_arena.inspect_embodiment':patch_embodiment}
 def find_spec(self,fullname,path,target=None):
  if fullname not in self.targets:return None
  sys.meta_path.remove(self)
  try:spec=importlib.util.find_spec(fullname)
  finally:sys.meta_path.insert(0,self)
  if spec is None or spec.loader is None:return None
  old,patch=spec.loader.exec_module,self.targets[fullname]
  def execute(module):old(module);patch(module)
  spec.loader.exec_module=execute;return spec
def install():
 for name,patch in Finder.targets.items():
  if name in sys.modules:patch(sys.modules[name])
 if not any(isinstance(x,Finder) for x in sys.meta_path):sys.meta_path.insert(0,Finder())
def current_score():return ENGINE.status()['score'] if ENGINE else 0.
