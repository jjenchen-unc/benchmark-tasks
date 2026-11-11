"""Task-local Arena registration; frozen task assets, genuine YAM, passive mechanics."""
from dataclasses import dataclass,replace
import importlib.abc,importlib.util,json,math,sys,os
from pathlib import Path
import numpy as np
from yam_arena.task_file import Box,Cylinder,ObjectState
from motor_limits import patch_actions
HERE=Path(__file__).resolve().parent;FIXTURE=json.loads((HERE/'fixture.json').read_text());ENGINE=None;SPECS={}
R=np.array([[0,1,0],[-1,0,0],[0,0,1.]])
SHIFT=np.array([.28,0.,-.0525])
def world(p):return SHIFT+R@np.asarray(p)
def arr(x):
 if hasattr(x,'torch'):x=x.torch
 elif type(x).__module__.startswith('warp'):
  import warp as wp
  x=wp.to_torch(x)
 return x.detach().cpu().numpy().copy() if hasattr(x,'detach') else np.asarray(x).copy()
def rotation(q):
 x,y,z,w=q;return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
@dataclass(frozen=True)
class Case(Box):
 label:str='A'
@dataclass(frozen=True)
class Instrument(Cylinder):pass
@dataclass(frozen=True)
class Plinth(Box):pass
def primitive_for(spec):
 import isaaclab.sim as sim
 from isaaclab_arena.assets.object import Object
 from isaaclab_arena.assets.object_type import ObjectType
 from isaaclab_arena.utils.pose import Pose
 kwargs=dict(collision_props=sim.CollisionPropertiesCfg(contact_offset=.0002,rest_offset=0.),physics_material=sim.RigidBodyMaterialCfg(static_friction=1. if isinstance(spec,Instrument) else .7,dynamic_friction=1. if isinstance(spec,Instrument) else .7),visual_material=sim.PreviewSurfaceCfg(diffuse_color=spec.color,roughness=.7))
 if isinstance(spec,Instrument):
  cfg=sim.UsdFileCfg(usd_path=str(HERE/'assets'/('instrument-'+spec.name[-1]+'.usda')),activate_contact_sensors=True)
 else:cfg=sim.CuboidCfg(size=spec.size,**kwargs)
 return Object(name=spec.name,object_type=ObjectType.RIGID if isinstance(spec,Instrument) else ObjectType.BASE,spawner_cfg=cfg,initial_pose=Pose(position_xyz=tuple(map(float,spec.pos)),rotation_xyzw=(0.,0.,0.,1.)))
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
  usd=sim.UsdFileCfg(usd_path=str(HERE/'assets'/f'case-{spec.label}.usda'),activate_contact_sensors=True)
  prim=usd.func(path,usd,translation=translation,orientation=orientation,**kw)
  from pxr import Sdf,UsdPhysics
  envroot=path.rsplit('/',1)[0]
  for b in FIXTURE['bodies']:
   if not (b['name']=='case_'+spec.label or b['name']=='lid_'+spec.label or b['name'].startswith(spec.label)):continue
   for g in b['geoms']:
    shape=prim.GetStage().GetPrimAtPath(path+'/links/'+b['name']+'/'+g['name'])
    targets=[]
    # Source trim/noses only collide with matching masks, not table/instruments.
    if not ((g['contype']&1) or (g['conaffinity']&1)):
     targets += [envroot+'/'+n for n in ['table','floor','plinth','inst_A','inst_B','stand_foot','stand_mast','stand_arm','camera_housing','lamp']]
    other='B' if spec.label=='A' else 'A'
    for ob in FIXTURE['bodies']:
     if not (ob['name']=='case_'+other or ob['name']=='lid_'+other or ob['name'].startswith(other)):continue
     for og in ob['geoms']:
      if not ((g['contype']&og['conaffinity']) or (og['contype']&g['conaffinity'])):targets.append(envroot+'/case_'+other+'/links/'+ob['name']+'/'+og['name'])
    if targets:
     rel=UsdPhysics.FilteredPairsAPI.Apply(shape).CreateFilteredPairsRel()
     for t in targets:rel.AddTarget(Sdf.Path(t))
  return prim
 @configclass
 class SpawnCfg(SpawnerCfg):func:object=clone(spawn)
 class Part(Object):
  def _generate_articulation_cfg(self):
   joints={n:v for n,v in FIXTURE['initial_joints'].items() if n.startswith(spec.label)}
   cfg=ArticulationCfg(prim_path=self.prim_path,spawn=self.spawner_cfg,actuators={'passive':ImplicitActuatorCfg(joint_names_expr=list(joints),stiffness=0.,damping=0.)},init_state=ArticulationCfg.InitialStateCfg(joint_pos=joints,joint_vel={'.*':0.}))
   return self._add_initial_pose_to_cfg(cfg)
 return Part(name=spec.name,object_type=ObjectType.ARTICULATION,spawner_cfg=SpawnCfg(),initial_pose=Pose(position_xyz=tuple(map(float,spec.pos)),rotation_xyzw=(0.,0.,math.sin(spec.yaw/2),math.cos(spec.yaw/2))))
def patch_scene(m):
 if getattr(m,'_s2_patched',False):return
 old_object,old_states,old_build=m.task_object,m.object_states,m.build_task_environment
 def task_object(spec):
  if isinstance(spec,Case):SPECS[spec.name]=spec;return object_for(spec)
  if isinstance(spec,(Instrument,Plinth)):return primitive_for(spec)
  return old_object(spec)
 def states(env,names):
  mine=[n for n in names if n in SPECS];out=old_states(env,[n for n in names if n not in mine]) if len(mine)<len(names) else {}
  for n in mine:
   a=env.unwrapped.scene[n];p=arr(a.data.body_pos_w)[0];lo=p.min(axis=0)-.12;hi=p.max(axis=0)+.12
   # Conservative bounds only for SDK bookkeeping; rubric uses body/joint/contact state.
   out[n]=ObjectState(pos=arr(a.data.root_pos_w)[0],quat_xyzw=arr(a.data.root_quat_w)[0],lin_vel=arr(a.data.root_lin_vel_w)[0],ang_vel=arr(a.data.root_ang_vel_w)[0],low=lo,high=hi)
  return out
 def build(*a,**kw):
  env=old_build(*a,**kw);task=kw.get('task')
  if task and any(isinstance(o,Case) for o in task.objects):
   prev=env.env_cfg_callback
   def configure(cfg):
    if prev:cfg=prev(cfg)
    cfg.sim.dt=.002;cfg.decimation=25;cfg.sim.render_interval=25
    from isaaclab.sensors import JointWrenchSensorCfg,ContactSensorCfg
    for side,prim in [('left','LeftArm'),('right','RightArm')]:
     setattr(cfg.scene,side+'_s2_wrench',JointWrenchSensorCfg(prim_path='{ENV_REGEX_NS}/'+prim))
     setattr(cfg.scene,side+'_s2_contact',ContactSensorCfg(prim_path='{ENV_REGEX_NS}/'+prim+'/.*'))
    from posture import PARK
    for side,robot in [('left',cfg.scene.left_robot),('right',cfg.scene.right_robot)]:
     robot.actuators['gripper'].joint_effort_limit=40.;robot.spawn.activate_contact_sensors=True
     from copy import deepcopy
     robot.init_state=deepcopy(robot.init_state)
     robot.init_state.joint_pos.update({f'joint{i+1}':float(v) for i,v in enumerate(PARK[side])})
    return cfg
   env.env_cfg_callback=configure
  return env
 m.task_object,m.object_states,m.build_task_environment=task_object,states,build;m._s2_patched=True
def patch_embodiment(m):
 cls=m.YamArenaEmbodiment
 if getattr(cls,'_s2_patched',False):return
 reset0,step0,task0=cls.reset,cls.step,cls._task_step
 def active(self):return self.task is not None and any(isinstance(o,Case) for o in self.task.objects)
 def reset(self,*a,**kw):
  global ENGINE
  if not active(self):return reset0(self,*a,**kw)
  if self.object_jitter!=0 or abs(self.info.control_hz-20)>1e-8:raise ValueError('S2 requires object_jitter=0 and control_hz=20; arbitrary harness seeds route to physical seed42')
  self._ensure_sim(None)
  # Robot initial configuration and settling targets agree before first callback.
  from posture import packed
  self._ready=self._torch.tensor([packed().tolist()],dtype=self._torch.float32,device=self._base.device)
  for side in ['left','right']:self._base.scene[side+'_robot']._s2_total_effort_enabled=True
  from physics import Physics
  if ENGINE is None or ENGINE.base is not self._base:
   ENGINE=Physics(self);write0=self._base.scene.write_data_to_sim;update0=self._base.scene.update;envreset0=self._env.reset
   def write(*args,**kwargs):ENGINE.pre_step();return write0(*args,**kwargs)
   def update(dt,*args,**kwargs):
    v=update0(dt,*args,**kwargs);ENGINE.post_step(float(dt));return v
   def envreset(*args,**kwargs):
    v=envreset0(*args,**kwargs);ENGINE.initialize();return v
   self._base.scene.write_data_to_sim=write;self._base.scene.update=update;self._env.reset=envreset
  ENGINE.settling=True;obs=reset0(self,*a,**kw);ENGINE.begin();return obs
 def step(self,action):
  if active(self):
   if ENGINE.done:raise RuntimeError('S2 already terminated; reset before stepping')
   q=np.asarray(action.data,float)
   if q.shape!=(14,) or not np.isfinite(q).all() or not 0<=q[6]<=1 or not 0<=q[13]<=1:raise ValueError('S2 requires finite genuine14D joint/gripper target')
   ENGINE.history.guard=[0.,0.];ENGINE.history.guard_stopped=[False,False]
   for side in ['left','right']:self._base.scene[side+'_robot']._s2_stop=False
  return step0(self,action)
 def task_step(self):
  if not active(self):return task0(self)
  ENGINE.controls+=1;v=task0(self);reason=ENGINE.reason();ENGINE.done=reason is not None
  return replace(v,terminated=ENGINE.done,termination_reason=reason,info=(dict(v.info,s2=ENGINE.status()) if os.environ.get('S2_EVALUATOR_DIAGNOSTICS')=='1' else dict(v.info)))
 cls.reset,cls.step,cls._task_step=reset,step,task_step;cls._s2_patched=True
def current_score():return ENGINE.credit() if ENGINE else 0.
class Finder(importlib.abc.MetaPathFinder):
 targets={'yam_arena.scene':patch_scene,'yam_arena.actions':patch_actions,'yam_arena.inspect_embodiment':patch_embodiment}
 def find_spec(self,fullname,path,target=None):
  if fullname not in self.targets:return None
  sys.meta_path.remove(self)
  try:s=importlib.util.find_spec(fullname)
  finally:sys.meta_path.insert(0,self)
  if not s or not s.loader:return None
  old,patch=s.loader.exec_module,self.targets[fullname]
  def execute(m):old(m);patch(m)
  s.loader.exec_module=execute;return s
def install():
 for n,p in Finder.targets.items():
  if n in sys.modules:p(sys.modules[n])
 if not any(isinstance(f,Finder) for f in sys.meta_path):sys.meta_path.insert(0,Finder())
