"""Task-local Arena integration. Policies change objects only through robot14D actions."""
from dataclasses import dataclass,replace
from pathlib import Path
import importlib.abc,importlib.util,json,sys,math
import numpy as np
from yam_arena.task_file import Box,ObjectState
from s5_motor import patch_actions
HERE=Path(__file__).resolve().parent;FIXTURE=json.loads((HERE/'fixture.json').read_text());SPECS={};ENGINE=None
def arr(x):
 if hasattr(x,'torch'):x=x.torch
 elif type(x).__module__.startswith('warp'):
  import warp as wp
  x=wp.to_torch(x)
 return x.detach().cpu().numpy().copy() if hasattr(x,'detach') else np.asarray(x).copy()
@dataclass(frozen=True)
class Cabinet(Box):pass
@dataclass(frozen=True)
class Cargo(Box):pass
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
  usd=sim.UsdFileCfg(usd_path=str(HERE/'assets'/f'{spec.name}.usda'),activate_contact_sensors=True)
  prim=usd.func(path,usd,translation=translation,orientation=orientation,**kw)
  # Bind actual RTX materials; displayColor alone is insufficient in all delegates.
  from pxr import UsdGeom
  from isaaclab.sim.utils import bind_visual_material
  for child in prim.GetStage().Traverse():
   if str(child.GetPath()).startswith(path+'/') and child.IsA(UsdGeom.Gprim):
    col=UsdGeom.Gprim(child).GetDisplayColorAttr().Get()
    if col:
     material=sim.PreviewSurfaceCfg(diffuse_color=tuple(float(x) for x in col[0]),roughness=.65);mp=str(child.GetPath())+'_look';material.func(mp,material);bind_visual_material(str(child.GetPath()),mp)
  return prim
 @configclass
 class SpawnCfg(SpawnerCfg):func:object=clone(spawn)
 class Part(Object):
  def _generate_articulation_cfg(self):
   bs=[b for b in FIXTURE['bodies'] if b['name']==spec.name or b['parent']==spec.name]
   acts={j['name']:ImplicitActuatorCfg(joint_names_expr=[j['name']],stiffness=j['stiffness'],damping=j['damping']) for b in bs for j in b['joints']}
   cfg=ArticulationCfg(prim_path=self.prim_path,spawn=self.spawner_cfg,actuators=acts,init_state=ArticulationCfg.InitialStateCfg(joint_pos={'.*':0.},joint_vel={'.*':0.}))
   return self._add_initial_pose_to_cfg(cfg)
 cls,typ=(Part,ObjectType.ARTICULATION) if isinstance(spec,Cabinet) else (Object,ObjectType.RIGID)
 return cls(name=spec.name,object_type=typ,spawner_cfg=SpawnCfg(),initial_pose=Pose(position_xyz=tuple(spec.pos),rotation_xyzw=(0.,0.,0.,1.)))
def patch_scene(module):
 if getattr(module,'_s5_patched',False):return
 old_object,old_states,old_build=module.task_object,module.object_states,module.build_task_environment
 def task_object(spec):
  if isinstance(spec,(Cabinet,Cargo)):SPECS[spec.name]=spec;return object_for(spec)
  return old_object(spec)
 def object_states(env,names):
  mine=[n for n in names if isinstance(SPECS.get(n),Cabinet)];out=old_states(env,[n for n in names if n not in mine]) if len(mine)<len(names) else {}
  from s5_contacts import rot_xyzw
  for name in mine:
   obj=env.unwrapped.scene[name];poses=dict(zip(obj.body_names,arr(obj.data.body_link_state_w)[0]));lo=[];hi=[]
   for b in FIXTURE['bodies']:
    if b['name'] not in poses:continue
    state=poses[b['name']];R=rot_xyzw(state[3:7])
    for g in b['geoms']:
     if not g['collision']:continue
     c=state[:3]+R@np.array(g['pos']);Q=R@rot_xyzw(np.r_[g['quat_wxyz'][1:],g['quat_wxyz'][0]]);s=g['size'];extent=abs(Q)@(np.array(s) if g['kind']=='box' else np.array([s[0],s[0],s[1]]));lo.append(c-extent);hi.append(c+extent)
   low=np.min(lo,axis=0);high=np.max(hi,axis=0);out[name]=ObjectState(pos=(low+high)/2,quat_xyzw=np.array([0,0,0,1]),lin_vel=np.zeros(3),ang_vel=np.zeros(3),low=low,high=high)
  return out
 def build(*a,**kw):
  env=old_build(*a,**kw);task=kw.get('task')
  if task and any(isinstance(x,Cabinet) for x in task.objects):
   prior=env.env_cfg_callback
   def configure(cfg):
    if prior:cfg=prior(cfg)
    cfg.sim.dt=.002;cfg.decimation=25;cfg.sim.render_interval=25
    from isaaclab.sensors import JointWrenchSensorCfg,ContactSensorCfg
    for side in ['left','right']:
     setattr(cfg.scene,side+'_s5_wrench',JointWrenchSensorCfg(prim_path='{ENV_REGEX_NS}/'+side.capitalize()+'Arm'))
     robot=getattr(cfg.scene,side+'_robot');robot.init_state.pos=tuple(FIXTURE['robot_bases'][side]);robot.actuators['gripper'].joint_effort_limit=40.;robot.spawn.activate_contact_sensors=True
     if side=='left':robot.init_state.joint_pos.update({'joint1':0.,'joint2':.3,'joint3':2.4,'joint4':-.5,'joint5':0.,'joint6':0.})
    cfg.scene.s5_enable_contacts=ContactSensorCfg(prim_path='{ENV_REGEX_NS}/cab_A/links/A_door_R')
    return cfg
   env.env_cfg_callback=configure
  return env
 module.task_object,module.object_states,module.build_task_environment=task_object,object_states,build;module._s5_patched=True
def patch_embodiment(module):
 cls=module.YamArenaEmbodiment
 if getattr(cls,'_s5_patched',False):return
 old_init,old_reset,old_step,old_task,old_close=cls.__init__,cls.reset,cls.step,cls._task_step,cls.close
 def active(self):return self.task is not None and any(isinstance(x,Cabinet) for x in self.task.objects)
 def init(self,*a,**kw):
  old_init(self,*a,**kw)
  if active(self):self.rig=replace(self.rig,bases={k:tuple(v) for k,v in FIXTURE['robot_bases'].items()})
 def reset(self,*a,**kw):
  global ENGINE
  if not active(self):return old_reset(self,*a,**kw)
  if self.object_jitter!=0 or self.info.control_hz!=20:raise ValueError('S5 fixed physical case requires object_jitter=0 and control_hz=20; arbitrary episode seeds are supported')
  if str(self.device)!='cpu':raise ValueError('S5 first foundation uses CPU PhysX with allocated GPU cameras')
  self.rig=replace(self.rig,bases={k:tuple(v) for k,v in FIXTURE['robot_bases'].items()})
  self._ensure_sim(None)
  for side in ['left','right']:self._base.scene[side+'_robot']._s5_total_effort_enabled=True
  if ENGINE is None or ENGINE.base is not self._base:
   from s5_physics import Physics
   ENGINE=Physics(self)
   write,update,env_reset=self._base.scene.write_data_to_sim,self._base.scene.update,self._env.reset
   def pre(*a,**kw):ENGINE.pre_step();return write(*a,**kw)
   def post(dt,*a,**kw):
    val=update(dt,*a,**kw);ENGINE.post_step(float(dt));return val
   def reset_env(*a,**kw):
    val=env_reset(*a,**kw);ENGINE.initialize();return val
   self._base.scene.write_data_to_sim=pre;self._base.scene.update=post;self._env.reset=reset_env
  ENGINE.settling=True;obs=old_reset(self,*a,**kw);ENGINE.begin();return obs
 def step(self,action):
  if active(self):
   if ENGINE is None or ENGINE.done:raise RuntimeError('S5 reset required; episode terminated, refusing further physics')
   values=np.asarray(action.data)
   if values.shape!=(14,) or not np.isfinite(values).all():raise ValueError('S5 requires finite genuine14D joint actions')
  return old_step(self,action)
 def task_step(self):
  if not active(self):return old_task(self)
  ENGINE.controls+=1;result=old_task(self);reason=ENGINE.reason();ENGINE.done=bool(reason or result.terminated)
  return replace(result,terminated=ENGINE.done,termination_reason=reason or result.termination_reason,info=dict(result.info,s5=ENGINE.status()))
 def close(self,*a,**kw):
  if ENGINE is not None and ENGINE.emb is self:ENGINE.close()
  return old_close(self,*a,**kw)
 cls.__init__,cls.reset,cls.step,cls._task_step,cls.close=init,reset,step,task_step,close;cls._s5_patched=True
def current_score():return ENGINE.status()['score'] if ENGINE else 0.
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
 if not any(isinstance(f,Finder) for f in sys.meta_path):sys.meta_path.insert(0,Finder())
