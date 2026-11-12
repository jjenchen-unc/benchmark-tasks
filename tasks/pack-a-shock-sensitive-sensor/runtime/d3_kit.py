"""Task-local scene adapter. All physics is actual Arena/PhysX; no native score transfer.
Episode initialization may restore poses/limits. Policies only issue genuine14D YAM targets.
"""
from dataclasses import dataclass,replace
from pathlib import Path
import importlib.abc,importlib.util,json,sys,math
import numpy as np
from yam_arena.task_file import Box,ObjectState
from d3_motor_adapter import patch_actions
HERE=Path(__file__).resolve().parent;FIXTURE=json.loads((HERE/'fixture.json').read_text())
_SPECS={};ENGINE=None
@dataclass(frozen=True)
class Carton(Box):
 number:int=1
@dataclass(frozen=True)
class Face(Box):
 number:int=1
@dataclass(frozen=True)
class Cargo(Box):
 label:str='A'
def arr(x):
 if hasattr(x,'torch'):x=x.torch
 elif type(x).__module__.startswith('warp'):
  import warp as wp
  x=wp.to_torch(x)
 return x.detach().cpu().numpy().copy() if hasattr(x,'detach') else np.asarray(x).copy()
def _look(stage,path,color):
 import isaaclab.sim as sim
 from isaaclab.sim.utils import bind_visual_material
 material=sim.PreviewSurfaceCfg(diffuse_color=tuple(float(x) for x in color),roughness=.7)
 material.func(path,material);return path
def _box_visual(stage,path,size,pos,color):
 from pxr import UsdGeom,Gf
 from isaaclab.sim.utils import bind_visual_material
 shape=UsdGeom.Cube.Define(stage,path);shape.CreateSizeAttr(2.);shape.AddTranslateOp().Set(Gf.Vec3d(*pos));shape.AddScaleOp().Set(Gf.Vec3f(*size));shape.CreateDisplayColorAttr([Gf.Vec3f(*color)])
 mat=_look(stage,path+'_look',color);bind_visual_material(path,mat);return shape
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
  from pxr import UsdGeom,UsdPhysics,Gf
  from isaaclab.sim.utils import bind_visual_material
  if isinstance(spec,Carton):
   usd=sim.UsdFileCfg(usd_path=str(HERE/f'assets/carton-{spec.number}.usda'),activate_contact_sensors=True)
   prim=usd.func(path,usd,translation=translation,orientation=orientation,**kw)
   for child in prim.GetStage().Traverse():
    if str(child.GetPath()).startswith(path+'/') and child.IsA(UsdGeom.Cube):
     colors=UsdGeom.Cube(child).GetDisplayColorAttr().Get()
     if colors:bind_visual_material(str(child.GetPath()),_look(prim.GetStage(),str(child.GetPath())+'_look',colors[0]))
   return prim
  cube=sim.CuboidCfg(size=tuple(spec.size),mass_props=sim.MassPropertiesCfg(mass=spec.mass),rigid_props=sim.RigidBodyPropertiesCfg(linear_damping=0.,angular_damping=0.),collision_props=sim.CollisionPropertiesCfg(contact_offset=.0005,rest_offset=0.),physics_material=sim.RigidBodyMaterialCfg(static_friction=1. if isinstance(spec,Cargo) else .8,dynamic_friction=1. if isinstance(spec,Cargo) else .8),visual_material=sim.PreviewSurfaceCfg(diffuse_color=tuple(spec.color),roughness=.7),activate_contact_sensors=True)
  prim=cube.func(path,cube,translation=translation,orientation=orientation,**kw);stage=prim.GetStage()
  if isinstance(spec,Face):
   inertia=.015*np.array([.0605**2+.003**2/12,.0805**2+.003**2/12,.0805**2+.0605**2])
   UsdPhysics.MassAPI.Apply(prim).CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float,inertia)))
   skin=UsdGeom.Mesh.Define(stage,path+'/foam_skin');skin.CreateFaceVertexCountsAttr([4]*6);skin.CreateFaceVertexIndicesAttr([0,1,3,2,4,6,7,5,0,4,5,1,2,3,7,6,0,2,6,4,1,5,7,3]);skin.CreateDoubleSidedAttr(True)
   points=[[x,y,z] for z in [-.0275,-.0015] for x in [-.082,.082] for y in [-.062,.062]];skin.CreatePointsAttr(points)
   bind_visual_material(str(skin.GetPath()),_look(stage,path+'/foam_skin_look',spec.color))
  elif isinstance(spec,Cargo):
   for g in FIXTURE['cargo_geoms']:
    if g['body']=='box_'+spec.label and not g['contact_enabled']:_box_visual(stage,path+'/'+g['name'],g['size'],g['local_position_m'],g['rgba'][:3])
  return prim
 @configclass
 class SpawnCfg(SpawnerCfg):func:object=clone(spawn)
 if isinstance(spec,Carton):
  class Part(Object):
   def _generate_articulation_cfg(self):
    cfg=ArticulationCfg(prim_path=self.prim_path,spawn=self.spawner_cfg,actuators={'hinge':ImplicitActuatorCfg(joint_names_expr=['lid_hinge'],stiffness=0.,damping=.004)},init_state=ArticulationCfg.InitialStateCfg(joint_pos={'.*':1.75},joint_vel={'.*':0.}))
    return self._add_initial_pose_to_cfg(cfg)
  cls,typ=Part,ObjectType.ARTICULATION
 else:cls,typ=Object,ObjectType.RIGID
 return cls(name=spec.name,object_type=typ,spawner_cfg=SpawnCfg(),initial_pose=Pose(position_xyz=tuple(spec.pos),rotation_xyzw=(0.,0.,0.,1.)))
def patch_scene(module):
 if getattr(module,'_d3_full_patched',False):return
 old_object,old_states,old_build=module.task_object,module.object_states,module.build_task_environment
 def task_object(spec):
  if isinstance(spec,(Carton,Face,Cargo)):_SPECS[spec.name]=spec;return object_for(spec)
  return old_object(spec)
 def object_states(env,names):
  mine=[n for n in names if isinstance(_SPECS.get(n),Carton)]
  out=old_states(env,[n for n in names if n not in mine]) if len(mine)<len(names) else {}
  from d3_material import rotation_xyzw
  for name in mine:
   obj=env.unwrapped.scene[name];states=arr(obj.data.body_link_state_w)[0];byname=dict(zip(obj.body_names,states));number=_SPECS[name].number;lows=[];highs=[]
   for g in FIXTURE['carton_geoms']:
    if not g['contact_enabled'] or g['body'] not in [f'carton_{number}',f'lid_{number}']:continue
    b=byname['base' if g['body'].startswith('carton_') else 'lid'];R=rotation_xyzw(b[3:7]);q=g['local_quaternion_wxyz'];Q=R@rotation_xyzw(np.r_[q[1:],q[0]]);centre=b[:3]+R@np.asarray(g['local_position_m']);extent=abs(Q)@np.asarray(g['size']);lows.append(centre-extent);highs.append(centre+extent)
   lo=np.min(lows,axis=0);hi=np.max(highs,axis=0);b=byname['base']
   out[name]=ObjectState(pos=(lo+hi)/2,quat_xyzw=b[3:7],lin_vel=arr(obj.data.root_lin_vel_w)[0],ang_vel=arr(obj.data.root_ang_vel_w)[0],low=lo,high=hi)
  return out
 def build(*a,**kw):
  env=old_build(*a,**kw);task=kw.get('task')
  if task and any(isinstance(x,Carton) for x in task.objects):
   previous=env.env_cfg_callback
   def configure(cfg):
    if previous:cfg=previous(cfg)
    cfg.sim.dt=.0005;cfg.decimation=100;cfg.sim.render_interval=100
    from isaaclab.sensors import JointWrenchSensorCfg
    cfg.scene.left_d3_wrench=JointWrenchSensorCfg(prim_path='{ENV_REGEX_NS}/LeftArm');cfg.scene.right_d3_wrench=JointWrenchSensorCfg(prim_path='{ENV_REGEX_NS}/RightArm')
    for n in range(1,5):setattr(cfg.scene,f'carton_{n}_d3_wrench',JointWrenchSensorCfg(prim_path='{ENV_REGEX_NS}/carton_'+str(n)))
    for robot in [cfg.scene.left_robot,cfg.scene.right_robot]:robot.actuators['gripper'].joint_effort_limit=40.;robot.spawn.activate_contact_sensors=True
    return cfg
   env.env_cfg_callback=configure
  return env
 module.task_object,module.object_states,module.build_task_environment=task_object,object_states,build;module._d3_full_patched=True
def patch_embodiment(module):
 cls=module.YamArenaEmbodiment
 if getattr(cls,'_d3_full_patched',False):return
 old_reset,old_step,old_task_step,old_close=cls.reset,cls.step,cls._task_step,cls.close
 def active(self):return self.task is not None and any(isinstance(x,Carton) for x in self.task.objects)
 def reset(self,*a,**kw):
  global ENGINE
  if not active(self):return old_reset(self,*a,**kw)
  if self.object_jitter!=0:raise ValueError('D3 fixed setup requires object_jitter=0')
  self._ensure_sim(None)
  for side in ['left','right']:self._base.scene[side+'_robot']._d3_total_effort_enabled=True
  from d3_physics import Physics
  if ENGINE is None or ENGINE.base is not self._base:
   ENGINE=Physics(self,FIXTURE,_SPECS)
   previous_write=self._base.scene.write_data_to_sim;previous_update=self._base.scene.update
   def write(*args,**kwargs):ENGINE.pre_step(float(self._base.sim.get_physics_dt()));return previous_write(*args,**kwargs)
   def update(dt,*args,**kwargs):
    value=previous_update(dt,*args,**kwargs);ENGINE.post_step(float(dt));return value
   self._base.scene.write_data_to_sim=write;self._base.scene.update=update
   previous_reset=self._env.reset
   def env_reset(*args,**kwargs):
    value=previous_reset(*args,**kwargs);ENGINE.initialize_episode_state();return value
   self._env.reset=env_reset
  ENGINE.settling=True
  self._base.cfg.sim.dt=.0005;self._base.sim.cfg.dt=.0005;self._base.cfg.decimation=100;self._base.cfg.sim.render_interval=100
  obs=old_reset(self,*a,**kw);ENGINE.begin_episode();return obs
 def step(self,action):
  if active(self):
   if ENGINE.done:raise RuntimeError('D3 episode already terminated; no further physics is permitted')
   dt=.000125 if ENGINE.needs_fine_step() else .0005;decimation=round(.05/dt)
   self._base.cfg.sim.dt=dt;self._base.sim.cfg.dt=dt;self._base.cfg.decimation=decimation;self._base.cfg.sim.render_interval=decimation
  return old_step(self,action)
 def task_step(self):
  if not active(self):return old_task_step(self)
  ENGINE.controls+=1;result=old_task_step(self);status=ENGINE.status();reason=ENGINE.termination_reason();ENGINE.done=reason is not None
  return replace(result,terminated=ENGINE.done or result.terminated,termination_reason=reason if reason else result.termination_reason,info=dict(result.info,d3=status))
 def close(self,*a,**kw):
  if ENGINE is not None and ENGINE.emb is self:ENGINE.close()
  return old_close(self,*a,**kw)
 cls.reset,cls.step,cls._task_step,cls.close=reset,step,task_step,close;cls._d3_full_patched=True
def current_score():
 # Missing actual latch/drop validation keeps the measured rubric at zero.
 return ENGINE.score()['score'] if ENGINE is not None else 0.
class Finder(importlib.abc.MetaPathFinder):
 targets={'yam_arena.scene':patch_scene,'yam_arena.actions':patch_actions,'yam_arena.inspect_embodiment':patch_embodiment}
 def find_spec(self,fullname,path,target=None):
  if fullname not in self.targets:return None
  sys.meta_path.remove(self)
  try:spec=importlib.util.find_spec(fullname)
  finally:sys.meta_path.insert(0,self)
  if not spec or not spec.loader:return None
  old,patch=spec.loader.exec_module,self.targets[fullname]
  def execute(module):old(module);patch(module)
  spec.loader.exec_module=execute;return spec
def install():
 for name,patch in Finder.targets.items():
  if name in sys.modules:patch(sys.modules[name])
 if not any(isinstance(x,Finder) for x in sys.meta_path):sys.meta_path.insert(0,Finder())
