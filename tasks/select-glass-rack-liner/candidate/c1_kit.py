"""Task-local scene composition and hooks, no installed SDK edits.
Duplicate same-body colliders with mutually exclusive filters preserve explicit
cup/liner pair materials independently of the liner's rubber contact material.
"""
from dataclasses import dataclass,replace
from pathlib import Path
import importlib.abc,importlib.util,sys,math,os,json,time,faulthandler
import numpy as np
from yam_arena.task_file import Box
from c1_contract import *
from c1_motor_adapter import patch_actions
HERE=Path(__file__).resolve().parent
SPECS={};ENGINE=None
def startup_mark(stage):
 path=os.environ.get('C1_STARTUP_TRACE')
 row={'stage':stage,'monotonic_s':time.monotonic()}
 if path:
  with Path(path).open('a') as stream:stream.write(json.dumps(row)+'\n')
 print('C1 startup:',stage,flush=True)
@dataclass(frozen=True)
class Part(Box):
 kind:str='liner'
 number:int=0
 roll:float=0.
def arr(x):
 if hasattr(x,'torch'):x=x.torch
 elif type(x).__module__.startswith('warp'):
  import warp as wp
  x=wp.to_torch(x)
 return x.detach().cpu().numpy().copy() if hasattr(x,'detach') else np.asarray(x).copy()
def _material(path,mu,combine='max',compliant=False):
 import isaaclab.sim as sim
 from isaaclab_physx.sim.spawners.materials.physics_materials_cfg import PhysxRigidBodyMaterialCfg
 cfg=PhysxRigidBodyMaterialCfg(static_friction=float(mu),dynamic_friction=float(mu),friction_combine_mode=combine,restitution=0.,compliant_contact_stiffness=6540. if compliant else 0.,compliant_contact_damping=72.33 if compliant else 0.,compliant_contact_acceleration_spring=False)
 cfg.func(path,cfg);return path

def object_for(spec):
 import isaaclab.sim as sim
 from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
 from isaaclab.sim.utils import clone
 from isaaclab.utils import configclass
 from isaaclab_arena.assets.object import Object
 from isaaclab_arena.assets.object_type import ObjectType
 from isaaclab_arena.utils.pose import Pose
 def spawn(path,cfg,translation=None,orientation=None,**kw):
  from pxr import UsdGeom,UsdPhysics,PhysxSchema,Gf
  from isaaclab.sim.utils import bind_physics_material
  if spec.kind=='cup':
   cfg0=sim.CylinderCfg(radius=.037,height=.070,mass_props=sim.MassPropertiesCfg(mass=.20),rigid_props=sim.RigidBodyPropertiesCfg(linear_damping=0.,angular_damping=0.),collision_props=sim.CollisionPropertiesCfg(contact_offset=.0005,rest_offset=0.),physics_material=sim.RigidBodyMaterialCfg(static_friction=.3,dynamic_friction=.3,friction_combine_mode='max'),visual_material=sim.PreviewSurfaceCfg(diffuse_color=(.82,.90,.95),opacity=.55,roughness=.1),activate_contact_sensors=True)
  else:
   cfg0=sim.CuboidCfg(size=spec.size,mass_props=sim.MassPropertiesCfg(mass=spec.mass) if spec.free else None,rigid_props=sim.RigidBodyPropertiesCfg(linear_damping=0.,angular_damping=0.) if spec.free else None,collision_props=sim.CollisionPropertiesCfg(contact_offset=.0005,rest_offset=0.),physics_material=sim.RigidBodyMaterialCfg(static_friction=1. if spec.kind=='liner' else .3 if spec.kind=='rack' else .5,dynamic_friction=1. if spec.kind=='liner' else .3 if spec.kind=='rack' else .5,friction_combine_mode='max'),visual_material=sim.PreviewSurfaceCfg(diffuse_color=spec.color,roughness=.8),activate_contact_sensors=spec.free)
  prim=cfg0.func(path,cfg0,translation=translation,orientation=orientation,**kw);stage=prim.GetStage()
  if spec.kind in ('cup','liner'):
   # SDK's geometry collider stays at its generated path; the pair-only
   # geometry is a second shape on the SAME rigid body, with explicit mass/I.
   colliders=[p for p in stage.Traverse() if str(p.GetPath()).startswith(path+'/') and p.HasAPI(UsdPhysics.CollisionAPI)]
   if len(colliders)!=1:raise RuntimeError('Expected one authored primitive collider: '+path)
   main=colliders[0];pairpath=path+'/pair_contact'
   if spec.kind=='liner':
    shape=UsdGeom.Cube.Define(stage,pairpath);shape.CreateSizeAttr(1.);shape.AddScaleOp().Set(Gf.Vec3f(*spec.size))
    mu=parameters(42)['mu_dry'][spec.number-1]
    I=.075/12*np.array([.15**2+.006**2,.07**2+.006**2,.07**2+.15**2]);com=(0.,0.,0.)
    # Number dots are visual only and carry no collision/mass.
    for k in range(spec.number):
     dot=UsdGeom.Cube.Define(stage,path+f'/dot{k}');dot.CreateSizeAttr(1.);dot.AddScaleOp().Set(Gf.Vec3f(.006,.006,.0004));dot.AddTranslateOp().Set(Gf.Vec3d(.012*(k%3-1),-.062+.011*(k//3),.0032));dot.CreateDisplayColorAttr([Gf.Vec3f(.92,.92,.90)])
   else:
    shape=UsdGeom.Cylinder.Define(stage,pairpath);shape.CreateRadiusAttr(.037);shape.CreateHeightAttr(.070);shape.CreateAxisAttr('Z');mu=1.
    I=.8*np.array([.2*(3*.037**2+.070**2)/12]*2+[.2*.037**2/2]);com=(0.,0.,-.007)
   UsdPhysics.CollisionAPI.Apply(shape.GetPrim());cp=PhysxSchema.PhysxCollisionAPI.Apply(shape.GetPrim());cp.CreateContactOffsetAttr(.0005);cp.CreateRestOffsetAttr(0.)
   UsdGeom.Imageable(shape.GetPrim()).MakeInvisible()
   mat=_material(path+'/pair_material',mu,'min',compliant=spec.kind=='liner');bind_physics_material(pairpath,mat)
   mass=UsdPhysics.MassAPI.Apply(prim);mass.CreateMassAttr(float(spec.mass));mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float,I)));mass.CreateCenterOfMassAttr(Gf.Vec3f(*com))
   # Relationships may target future prims; final complete filter audit runs
   # after scene creation and before the first reset control is executed.
   from pxr import Sdf
   main.CreateAttribute('c1:mainCollider',Sdf.ValueTypeNames.Bool).Set(True)
   if spec.kind=='liner':bind_physics_material(str(main.GetPath()),_material(path+'/rubber_material',1.,'max',compliant=True))
  if spec.name=='pan_floor':
   water=UsdGeom.Cube.Define(stage,path+'/water_visual');water.CreateSizeAttr(1.);water.AddScaleOp().Set(Gf.Vec3f(.139,.169,.020));water.AddTranslateOp().Set(Gf.Vec3d(0.,0.,.012));water.CreateDisplayColorAttr([Gf.Vec3f(.35,.6,.9)]);water.CreateDisplayOpacityAttr([.45])
  from c1_physics import author_filters
  author_filters(stage,path.rsplit('/',1)[0])
  return prim
 @configclass
 class SpawnCfg(SpawnerCfg):func:object=clone(spawn)
 # Proper rotation: native rack Rx(20) and global Rz(-90).
 from scipy.spatial.transform import Rotation
 R=A if spec.kind in ('liner','cup','tray','pan','leg') else A@Rotation.from_euler('x',spec.roll).as_matrix()
 q=Rotation.from_matrix(R).as_quat()
 return Object(name=spec.name,object_type=ObjectType.RIGID if spec.free else ObjectType.BASE,spawner_cfg=SpawnCfg(),initial_pose=Pose(position_xyz=tuple(map(float,spec.pos)),rotation_xyzw=tuple(map(float,q))))

def patch_scene(module):
 if getattr(module,'_c1_patched',False):return
 old_object,old_build=module.task_object,module.build_task_environment
 def task_object(spec):
  if isinstance(spec,Part):SPECS[spec.name]=spec;return object_for(spec)
  return old_object(spec)
 def build(*a,**kw):
  env=old_build(*a,**kw);task=kw.get('task')
  if task and any(isinstance(x,Part) for x in task.objects):
   previous=env.env_cfg_callback
   def configure(cfg):
    if previous:cfg=previous(cfg)
    cfg.sim.dt=DT;cfg.decimation=50;cfg.sim.render_interval=50
    from isaaclab.sensors import JointWrenchSensorCfg,ContactSensorCfg
    cfg.scene.left_c1_wrench=JointWrenchSensorCfg(prim_path='{ENV_REGEX_NS}/LeftArm');cfg.scene.right_c1_wrench=JointWrenchSensorCfg(prim_path='{ENV_REGEX_NS}/RightArm')
    cfg.scene.c1_contact_activation=ContactSensorCfg(prim_path='{ENV_REGEX_NS}/cup_1')
    for robot in [cfg.scene.left_robot,cfg.scene.right_robot]:
     robot.actuators['gripper'].joint_effort_limit=40.;robot.actuators['gripper'].joint_velocity_limit=.075
     robot.actuators['shoulder'].joint_velocity_limit=2.175;robot.actuators['wrist'].joint_velocity_limit=2.175
     robot.init_state.joint_pos['joint[7-8]']=-.045;robot.spawn.activate_contact_sensors=True
     robot.init_state.joint_pos['joint1']=-.25 # task-local collision-clear start, before physics
    for side,robot in [('left',cfg.scene.left_robot),('right',cfg.scene.right_robot)]:
     robot.init_state.pos=ROBOT_BASES[side];yaw=ROBOT_YAW[side];robot.init_state.rot=(0.,0.,math.sin(yaw/2),math.cos(yaw/2))
    return cfg
   env.env_cfg_callback=configure
  return env
 module.task_object,module.build_task_environment=task_object,build;module._c1_patched=True

def patch_embodiment(module):
 cls=module.YamArenaEmbodiment
 if getattr(cls,'_c1_patched',False):return
 old_reset,old_step,old_task_step,old_close=cls.reset,cls.step,cls._task_step,cls.close
 def active(self):return self.task is not None and any(isinstance(x,Part) for x in self.task.objects)
 def reset(self,*a,**kw):
  global ENGINE
  if not active(self):return old_reset(self,*a,**kw)
  if float(self.info.control_hz)!=20 or self.object_jitter!=0:raise ValueError('C1 requires 20Hz and fixed object_jitter=0')
  from c1_public import select
  trial_seed=kw.get('seed')
  seed=select(self.task.module,trial_seed,os.environ.get('C1_TARGET','C1-T1'),self.object_jitter)
  seed_row={'stage':'reset_seeds','trial_seed':None if trial_seed is None else int(trial_seed),'physical_seed':seed}
  print(json.dumps(seed_row),flush=True)
  if os.environ.get('C1_STARTUP_TRACE'):
   with Path(os.environ['C1_STARTUP_TRACE']).open('a') as stream:stream.write(json.dumps(seed_row)+'\n')
  startup_mark('ensure_sim_start');faulthandler.dump_traceback_later(120,repeat=False)
  self._ensure_sim(None)
  startup_mark('ensure_sim_complete');faulthandler.dump_traceback_later(120,repeat=False)
  for side in ['left','right']:self._base.scene[side+'_robot']._c1_total_effort_enabled=True
  from c1_physics import Physics
  if ENGINE is None or ENGINE.base is not self._base:
   startup_mark('physics_engine_create_start');ENGINE=Physics(self,SPECS);startup_mark('physics_engine_create_complete')
   previous=self._base.scene.update
   def update(dt,*args,**kwargs):
    value=previous(dt,*args,**kwargs)
    if ENGINE.active:ENGINE.sample(float(dt))
    return value
   self._base.scene.update=update
  ENGINE.active=False
  startup_mark('reset_materials_start');ENGINE.reset_materials(seed);startup_mark('reset_materials_complete')
  startup_mark('sdk_reset_settle_start');faulthandler.dump_traceback_later(120,repeat=False)
  obs=old_reset(self,*a,**kw);startup_mark('sdk_reset_settle_complete');ENGINE.begin_episode(seed);faulthandler.cancel_dump_traceback_later();return obs
 def step(self,action):
  if active(self):
   if ENGINE.done:raise RuntimeError('C1 episode already ended')
   ENGINE.validate_action(action)
  return old_step(self,action)
 def task_step(self):
  if not active(self):return old_task_step(self)
  ENGINE.controls+=1;result=old_task_step(self);status=ENGINE.status();reason=ENGINE.termination_reason();ENGINE.done=reason is not None
  return replace(result,terminated=bool(reason) or result.terminated,termination_reason=reason or result.termination_reason,info=dict(result.info,c1=status))
 def close(self,*a,**kw):
  if ENGINE is not None and ENGINE.emb is self:ENGINE.close()
  return old_close(self,*a,**kw)
 cls.reset,cls.step,cls._task_step,cls.close=reset,step,task_step,close;cls._c1_patched=True

def current_score():return ENGINE.status()['score'] if ENGINE is not None else 0.
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
