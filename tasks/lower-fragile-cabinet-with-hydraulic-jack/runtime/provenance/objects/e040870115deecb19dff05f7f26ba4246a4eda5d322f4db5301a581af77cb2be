"""Task-local Arena/Inspect adapter. No SDK edits or native runtime imports."""
from dataclasses import dataclass,replace
from pathlib import Path
import importlib.abc,importlib.util,json,sys
import numpy as np
from yam_arena.task_file import Box,ObjectState
from motor_adapter import patch_actions
from contacts import arr
from scoring import History
HERE=Path(__file__).resolve().parent;F=json.loads((HERE/'fixture.json').read_text());SPECS={};ENGINE=None
@dataclass(frozen=True)
class Jack(Box):tag:str='S'
@dataclass(frozen=True)
class Cabinet(Box):pass
@dataclass(frozen=True)
class Rail(Box):mu:float=.35

def object_for(spec):
 import isaaclab.sim as sim
 from isaaclab.assets import ArticulationCfg
 from isaaclab.actuators import ImplicitActuatorCfg
 from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
 from isaaclab.sim.utils import clone
 from isaaclab.utils import configclass
 from isaaclab_arena.assets.object import Object
 from isaaclab_arena.assets.object_type import ObjectType
 from isaaclab_arena.utils.pose import Pose
 if isinstance(spec,Rail):
  return Object(name=spec.name,object_type=ObjectType.BASE,spawner_cfg=sim.CuboidCfg(size=tuple(spec.size),collision_props=sim.CollisionPropertiesCfg(contact_offset=.0005,rest_offset=0.),physics_material=sim.RigidBodyMaterialCfg(static_friction=spec.mu,dynamic_friction=spec.mu),visual_material=sim.PreviewSurfaceCfg(diffuse_color=tuple(spec.color))),initial_pose=Pose(position_xyz=tuple(map(float,spec.pos)),rotation_xyzw=(0.,0.,0.,1.)))
 def spawn(path,cfg,translation=None,orientation=None,**kw):
  usd=sim.UsdFileCfg(usd_path=str(HERE/(spec.name+'.usda')),activate_contact_sensors=True)
  return usd.func(path,usd,translation=translation,orientation=orientation,**kw)
 @configclass
 class SpawnCfg(SpawnerCfg):func:object=clone(spawn)
 if isinstance(spec,Jack):
  class Part(Object):
   def _generate_articulation_cfg(self):
    tag=spec.tag;cfg=ArticulationCfg(prim_path=self.prim_path,spawn=self.spawner_cfg,actuators={
     'ram':ImplicitActuatorCfg(joint_names_expr=['ram_'+tag],stiffness=200000.,damping=400.),
     'lever':ImplicitActuatorCfg(joint_names_expr=['lever_'+tag],stiffness=.5,damping=.3),
     'valve':ImplicitActuatorCfg(joint_names_expr=['valve_'+tag],stiffness=0.,damping=.01),
     'sleeve':ImplicitActuatorCfg(joint_names_expr=['grip_'+tag],stiffness=0.,damping=.002)},init_state=ArticulationCfg.InitialStateCfg(joint_pos={'ram_'+tag:.055 if tag=='S' else 0.,'lever_'+tag:.3,'valve_'+tag:0.,'grip_'+tag:0.},joint_vel={'.*':0.}))
    return self._add_initial_pose_to_cfg(cfg)
  cls,typ=Part,ObjectType.ARTICULATION
 else:cls,typ=Object,ObjectType.RIGID
 return cls(name=spec.name,object_type=typ,spawner_cfg=SpawnCfg(),initial_pose=Pose(position_xyz=tuple(map(float,spec.pos)),rotation_xyzw=(0.,0.,0.,1.)))

def patch_scene(m):
 if getattr(m,'_hj_patched',False):return
 old_obj,old_states,old_build=m.task_object,m.object_states,m.build_task_environment
 def task_object(spec):
  if isinstance(spec,(Jack,Cabinet,Rail)):SPECS[spec.name]=spec;return object_for(spec)
  return old_obj(spec)
 def states(env,names):
  mine=[n for n in names if isinstance(SPECS.get(n),Jack)];out=old_states(env,[n for n in names if n not in mine]) if len(mine)<len(names) else {}
  for n in mine:
   p=np.array(SPECS[n].pos);out[n]=ObjectState(pos=p,quat_xyzw=np.array([0,0,0,1]),lin_vel=np.zeros(3),ang_vel=np.zeros(3),low=p-np.array([.22,.06,0]),high=p+np.array([.23,.06,.23]))
  return out
 def build(*a,**kw):
  env=old_build(*a,**kw);task=kw.get('task')
  if task and any(isinstance(o,Jack) for o in task.objects):
   prev=env.env_cfg_callback
   def configure(cfg):
    if prev:cfg=prev(cfg)
    cfg.sim.dt=.002;cfg.decimation=25;cfg.sim.render_interval=25
    from mount import BASE
    cfg.scene.right_robot.init_state.pos=tuple(map(float,BASE))
    cfg.scene.right_robot.init_state.rot=(0.,0.,1.,0.)
    from isaaclab.sensors import JointWrenchSensorCfg,ContactSensorCfg
    for side in ('left','right'):
     setattr(cfg.scene,side+'_hj_wrench',JointWrenchSensorCfg(prim_path='{ENV_REGEX_NS}/'+side.title()+'Arm'))
     robot=getattr(cfg.scene,side+'_robot');robot.actuators['gripper'].joint_effort_limit=40.;robot.spawn.activate_contact_sensors=True
    cfg.scene.hj_contact_processing=ContactSensorCfg(prim_path='{ENV_REGEX_NS}/cabinet')
    return cfg
   env.env_cfg_callback=configure
  return env
 m.task_object,m.object_states,m.build_task_environment=task_object,states,build;m._hj_patched=True

def patch_embodiment(m):
 cls=m.YamArenaEmbodiment
 if getattr(cls,'_hj_patched',False):return
 old_reset,old_step,old_task_step=cls.reset,cls.step,cls._task_step
 def active(self):return self.task is not None and any(isinstance(o,Jack) for o in self.task.objects)
 def reset(self,*a,**kw):
  global ENGINE
  if not active(self):return old_reset(self,*a,**kw)
  if self.object_jitter!=0 or float(self.info.control_hz)!=20.:raise ValueError('Fixed hydraulic setup requires object_jitter=0 and control_hz=20; all harness seeds accepted')
  self._ensure_sim(None)
  for side in ('left','right'):self._base.scene[side+'_robot']._hj_total_effort_enabled=True
  from physics import Physics
  if ENGINE is None or ENGINE.base is not self._base:
   ENGINE=Physics(self,F,SPECS);pw,pu,pr=self._base.scene.write_data_to_sim,self._base.scene.update,self._env.reset
   def write(*args,**kwargs):ENGINE.pre(float(self._base.physics_dt));return pw(*args,**kwargs)
   def update(dt,*args,**kwargs):
    v=pu(dt,*args,**kwargs);ENGINE.post(float(dt));return v
   def env_reset(*args,**kwargs):
    v=pr(*args,**kwargs);ENGINE.initialize();return v
   self._base.scene.write_data_to_sim=write;self._base.scene.update=update;self._env.reset=env_reset
  ENGINE.settling=True;obs=old_reset(self,*a,**kw);ENGINE.begin();return obs
 def step(self,action):
  if active(self):
   if ENGINE is None or ENGINE.done:raise RuntimeError('reset required before step and after termination')
   data=np.asarray(action.data,dtype=float)
   if data.shape!=(14,) or not np.isfinite(data).all():raise ValueError('finite genuine14D robot action required')
  return old_step(self,action)
 def task_step(self):
  if not active(self):return old_task_step(self)
  ENGINE.controls+=1;res=old_task_step(self);status=ENGINE.status();failure=status['failure']
  terminal=bool(failure or ENGINE.controls>=3210 or status['score']>=1.)
  ENGINE.done=terminal
  return replace(res,terminated=terminal,termination_reason=failure or ('time_limit' if ENGINE.controls>=3210 else 'success' if status['score']>=1 else None),info=dict(res.info,hydraulic_jack=status))
 cls.reset,cls.step,cls._task_step=reset,step,task_step;cls._hj_patched=True

def current_status():return ENGINE.status() if ENGINE is not None else History().result()
class Finder(importlib.abc.MetaPathFinder):
 targets={'yam_arena.scene':patch_scene,'yam_arena.actions':patch_actions,'yam_arena.inspect_embodiment':patch_embodiment}
 def find_spec(self,fullname,path,target=None):
  if fullname not in self.targets:return None
  sys.meta_path.remove(self)
  try:spec=importlib.util.find_spec(fullname)
  finally:sys.meta_path.insert(0,self)
  if not spec or not spec.loader:return None
  execute,patch=spec.loader.exec_module,self.targets[fullname]
  def load(module):execute(module);patch(module)
  spec.loader.exec_module=load;return spec

def install():
 for name,patch in Finder.targets.items():
  if name in sys.modules:patch(sys.modules[name])
 if not any(isinstance(x,Finder) for x in sys.meta_path):sys.meta_path.insert(0,Finder())
