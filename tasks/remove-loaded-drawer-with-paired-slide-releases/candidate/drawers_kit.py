"""Task-local Arena registration. No native/MuJoCo runtime dependency."""
from dataclasses import dataclass,replace
from pathlib import Path
import json,math,sys,importlib.abc,importlib.util
import numpy as np
from yam_arena.task_file import Box,ObjectState
from drawers_math import arr,world,rotation
from motor_limits import patch_actions
HERE=Path(__file__).resolve().parent
FIXTURE=json.loads((HERE/'fixture.json').read_text());ASSETS=json.loads((HERE/'assets.json').read_text());HOME=json.loads((HERE/'home.json').read_text())['q'];SPECS={};ENGINE=None
MOUNT=json.loads((HERE/'mount.json').read_text())
@dataclass(frozen=True)
class DrawerAsset(Box):pass

def object_for(spec):
 import isaaclab.sim as sim
 from isaaclab.assets import ArticulationCfg
 from isaaclab.actuators import ImplicitActuatorCfg
 from isaaclab_arena.assets.object import Object
 from isaaclab_arena.assets.object_type import ObjectType
 from isaaclab_arena.utils.pose import Pose
 from reset_pose_api import float32_reset_pose
 from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
 from isaaclab.sim.utils import clone
 from isaaclab.utils import configclass
 asset=next(a for a in ASSETS if a['name']==spec.name)
 def spawn(prim_path,cfg,translation=None,orientation=None,**kw):
  from pxr import Sdf,UsdPhysics
  usd=sim.UsdFileCfg(usd_path=str(HERE/(spec.name+'.usda')),activate_contact_sensors=not asset['fixed'])
  p=usd.func(prim_path,usd,translation=translation,orientation=orientation,**kw)
  # Preserve source mask4: stops/bumps interact only with each other.
  env=prim_path.rsplit('/',1)[0]
  normals=[];masked=[]
  for a in ASSETS:
   for i in a['body_ids']:
    for g in FIXTURE['bodies'][i]['geoms']:
     if not(g['contype'] or g['conaffinity']):continue
     path=env+'/'+a['name']+'/'+FIXTURE['bodies'][i]['name']+'/'+g['name'].replace('-','_minus_')
     (masked if g['contype']==4 else normals).append(path)
  external=[env+'/'+n for n in ('LeftArm','RightArm','table')]+['/World/GroundPlane']
  for path in masked:
   if not path.startswith(prim_path+'/'):continue
   rel=UsdPhysics.FilteredPairsAPI.Apply(p.GetStage().GetPrimAtPath(path)).CreateFilteredPairsRel()
   for other in normals+external:rel.AddTarget(Sdf.Path(other))
  return p
 @configclass
 class SpawnCfg(SpawnerCfg):func:object=clone(spawn)
 class Part(Object):
  def _generate_articulation_cfg(self):
   actuators={j['name']:ImplicitActuatorCfg(joint_names_expr=[j['name']],stiffness=j['stiffness'],damping=j['damping']) for i in asset['body_ids'] for j in FIXTURE['bodies'][i]['joints'] if j['type']!=0}
   return self._add_initial_pose_to_cfg(ArticulationCfg(prim_path=self.prim_path,spawn=self.spawner_cfg,actuators=actuators,init_state=ArticulationCfg.InitialStateCfg(joint_pos={'.*':0.},joint_vel={'.*':0.})))
 typ=ObjectType.BASE if asset['fixed'] else ObjectType.ARTICULATION if asset['articulation'] else ObjectType.RIGID
 return Part(name=spec.name,object_type=typ,spawner_cfg=SpawnCfg(),initial_pose=float32_reset_pose(Pose, position_xyz=spec.pos, rotation_xyzw=(0,0,1,0)))

def patch_scene(module):
 if getattr(module,'_drawers_patched',False):return
 old_object,old_states,old_build=module.task_object,module.object_states,module.build_task_environment
 def task_object(spec):
  if isinstance(spec,DrawerAsset):SPECS[spec.name]=spec;return object_for(spec)
  return old_object(spec)
 def states(env,names):
  mine=[n for n in names if n in SPECS];out=old_states(env,[n for n in names if n not in SPECS]) if len(mine)<len(names) else {}
  for n in mine:
   spec=SPECS[n];lo=np.asarray(spec.pos)-.001;hi=lo+.002
   if ENGINE is not None:
    geoms=[g for g in ENGINE.geoms() if g['asset']==n]
    if geoms:lo=np.min([g['p']-g['extent'] for g in geoms],axis=0);hi=np.max([g['p']+g['extent'] for g in geoms],axis=0)
   out[n]=ObjectState(pos=(lo+hi)/2,quat_xyzw=np.array([0,0,1,0]),lin_vel=np.zeros(3),ang_vel=np.zeros(3),low=lo,high=hi)
  return out
 def build(*a,**kw):
  env=old_build(*a,**kw);task=kw.get('task')
  if task and any(isinstance(o,DrawerAsset) for o in task.objects):
   prior=env.env_cfg_callback
   def configure(cfg):
    if prior:cfg=prior(cfg)
    from isaaclab.sensors import JointWrenchSensorCfg,ContactSensorCfg
    from isaaclab.assets import AssetBaseCfg
    import isaaclab.sim as sim
    for support in MOUNT['supports']:
     setattr(cfg.scene,support['name'],AssetBaseCfg(
      prim_path='{ENV_REGEX_NS}/'+support['name'],
      spawn=sim.CuboidCfg(size=tuple(support['size_m']),collision_props=sim.CollisionPropertiesCfg(),
       visual_material=sim.PreviewSurfaceCfg(diffuse_color=(.35,.35,.38))),
      init_state=AssetBaseCfg.InitialStateCfg(pos=tuple(support['position_m']))))
    for side in ('left','right'):
     setattr(cfg.scene,side+'_drawers_wrench',JointWrenchSensorCfg(prim_path='{ENV_REGEX_NS}/'+side.title()+'Arm'))
     robot=getattr(cfg.scene,side+'_robot');robot.actuators['gripper'].joint_effort_limit=40.;robot.spawn.activate_contact_sensors=True
     robot.init_state.pos=tuple(MOUNT['robot_base_positions_m'][side])
     robot.init_state.joint_pos.update({f'joint{i+1}':v for i,v in enumerate(HOME)})
    cfg.scene.drawers_contacts=ContactSensorCfg(prim_path='{ENV_REGEX_NS}/U_jar/U_jar')
    return cfg
   env.env_cfg_callback=configure
  return env
 module.task_object,module.object_states,module.build_task_environment=task_object,states,build;module._drawers_patched=True

def patch_embodiment(module):
 cls=module.YamArenaEmbodiment
 if getattr(cls,'_drawers_patched',False):return
 old_reset,old_step,old_task=cls.reset,cls.step,cls._task_step
 def active(self):return self.task and any(isinstance(o,DrawerAsset) for o in self.task.objects)
 def reset(self,*a,**kw):
  global ENGINE
  if not active(self):return old_reset(self,*a,**kw)
  if self.object_jitter!=0 or not math.isclose(self.info.control_hz,20):raise ValueError('Fixed physical case requires object_jitter=0, control_hz=20; any Inspect episode seed accepted')
  self._ensure_sim(None)
  import torch
  self._ready=torch.tensor([[*HOME,1.,*HOME,1.]],dtype=torch.float32,device=self._base.device)
  for side in ('left','right'):self._base.scene[side+'_robot']._drawers_total_effort_enabled=True
  if ENGINE is None or ENGINE.emb is not self:
   from drawers_physics import Physics
   ENGINE=Physics(self,FIXTURE,ASSETS)
   previous_update=self._base.scene.update;previous_write=self._base.scene.write_data_to_sim;previous_reset=self._env.reset
   def update(dt,*args,**kwargs):
    r=previous_update(dt,*args,**kwargs)
    if ENGINE.running:ENGINE.post_step(float(dt))
    return r
   def write(*args,**kwargs):ENGINE.pre_step();return previous_write(*args,**kwargs)
   def env_reset(*args,**kwargs):
    r=previous_reset(*args,**kwargs);ENGINE.reset_physical_joints();return r
   self._base.scene.update=update;self._base.scene.write_data_to_sim=write;self._env.reset=env_reset
  ENGINE.initialize();ENGINE.history.damage_grace_s=.3;ENGINE._make_contacts();ENGINE.running=True
  obs=old_reset(self,*a,**kw);ENGINE.begin();return obs
 def step(self,action):
  if active(self):
   if ENGINE.done:raise RuntimeError('Episode terminated; reset required before physics advances')
   q=np.asarray(action.data)
   if q.shape!=(14,) or not np.isfinite(q).all():raise ValueError('Expected genuine finite 14D joint target')
   if np.any(q<self._low-1e-8) or np.any(q>self._high+1e-8):raise ValueError('Action outside actual YAM limits')
  return old_step(self,action)
 def task_step(self):
  if not active(self):return old_task(self)
  ENGINE.controls+=1;r=old_task(self);status=ENGINE.status();reason=ENGINE.history.failure or ('time_limit' if ENGINE.controls>=3586 else 'success' if status['score']==1 else None);ENGINE.done=reason is not None
  return replace(r,terminated=ENGINE.done,termination_reason=reason,info=dict(r.info,score=status['score']))
 cls.reset,cls.step,cls._task_step=reset,step,task_step;cls._drawers_patched=True
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
 for n,p in Finder.targets.items():
  if n in sys.modules:p(sys.modules[n])
 if not any(isinstance(f,Finder) for f in sys.meta_path):sys.meta_path.insert(0,Finder())
def current_score():return ENGINE.status()['score'] if ENGINE is not None else 0.
