"""Task-scoped passive physical Arena/YAM scene, genuine SDK action contract."""
from dataclasses import dataclass,replace
from pathlib import Path
import importlib.abc,importlib.util,json,math,sys
import numpy as np
from yam_arena.task_file import Box,ObjectState
from step_guard import EpisodeGuard
from scoring import History,CONTROL_LIMIT,DT
from contacts import arr,rot,rot_xyzw
HERE=Path(__file__).resolve().parent
FIX=json.loads((HERE/'fixture.json').read_text());SHIFT=np.array(FIX['translation']);BODIES={b['name']:b for b in FIX['bodies']};FIX['stand_world']=(np.array(BODIES['stand_C']['pos'])+SHIFT).tolist();FIX['sleeve_world']=(np.array(BODIES['sleeve_C']['pos'])+SHIFT).tolist()
_OBSERVER=None;_SPECS={};_STEPS=0
@dataclass(frozen=True)
class Coupling(Box):pass

def object_for(spec):
 import isaaclab.sim as sim
 from isaaclab.actuators import ImplicitActuatorCfg
 from isaaclab.assets import ArticulationCfg
 from isaaclab_arena.assets.object import Object
 from isaaclab_arena.assets.object_type import ObjectType
 from isaaclab_arena.utils.pose import Pose
 if spec.name=='coupling_sleeve':return Object(name=spec.name,object_type=ObjectType.RIGID,spawner_cfg=sim.UsdFileCfg(usd_path=str(HERE/'coupling-sleeve.usda'),activate_contact_sensors=True),initial_pose=Pose(position_xyz=tuple(spec.pos),rotation_xyzw=(0,0,0,1)))
 class Fixture(Object):
  def _generate_articulation_cfg(self):
   cfg=ArticulationCfg(prim_path=self.prim_path,spawn=self.spawner_cfg,actuators={'plunger':ImplicitActuatorCfg(joint_names_expr=['plunger_slide_C'],stiffness=0.,damping=0.)},init_state=ArticulationCfg.InitialStateCfg(joint_pos={'plunger_slide_C':-.015},joint_vel={'.*':0.}))
   return self._add_initial_pose_to_cfg(cfg)
 return Fixture(name=spec.name,object_type=ObjectType.ARTICULATION,spawner_cfg=sim.UsdFileCfg(usd_path=str(HERE/'coupling-fixture.usda'),activate_contact_sensors=True),initial_pose=Pose(position_xyz=tuple(spec.pos),rotation_xyzw=(0,0,0,1)))

def patch_scene(module):
 if getattr(module,'_b7_patched',False):return
 oo,os,ob=module.task_object,module.object_states,module.build_task_environment
 def obj(spec):
  if isinstance(spec,Coupling):_SPECS[spec.name]=spec;return object_for(spec)
  return oo(spec)
 def states(env,names):
  # Report actual physical object states; these remain outside public observations.
  out=os(env,[n for n in names if n not in _SPECS]) if any(n not in _SPECS for n in names) else {}
  base=env.unwrapped;origin=arr(base.scene.env_origins)[0]
  for n in names:
   if n not in _SPECS:continue
   asset=base.scene[n];groups=[]
   if n=='coupling_sleeve':
    p=arr(asset.data.root_pos_w)[0]-origin;q=arr(asset.data.root_quat_w)[0];v=arr(asset.data.root_vel_w)[0];groups=[(p,rot_xyzw(q),BODIES['sleeve_C']['geoms'])]
   else:
    i=asset.body_names.index('stand');p=arr(asset.data.body_pos_w)[0,i]-origin;q=arr(asset.data.body_quat_w)[0,i];v=np.zeros(6)
    for part in ('stand','plunger','tray_floor','tray_walls'):
     i=asset.body_names.index(part);bp=arr(asset.data.body_pos_w)[0,i]-origin;bR=rot_xyzw(arr(asset.data.body_quat_w)[0,i])
     gs=BODIES[part+'_C']['geoms'] if part in ('stand','plunger') else [g for g in BODIES['tray']['geoms'] if (g['name']=='tray_floor')==(part=='tray_floor')]
     groups.append((bp,bR,gs))
   bounds=[]
   for bp,bR,geoms in groups:
    for g in geoms:
     centre=bp+bR@np.array(g['pos']);Rg=bR@rot(g['quat']);size=np.array(g['size'])
     if g['type']=='box':half=np.abs(Rg)@size
     else:
      axis=Rg[:,2];half=size[1]*np.abs(axis)+size[0]*np.sqrt(np.clip(1-axis*axis,0,1))
     bounds.extend([centre-half,centre+half])
   lo=np.min(bounds,axis=0);hi=np.max(bounds,axis=0)
   out[n]=ObjectState(pos=(lo+hi)/2,quat_xyzw=q,lin_vel=v[:3],ang_vel=v[3:],low=lo,high=hi)
  return out
 def build(*a,**kw):
  env=ob(*a,**kw);task=kw.get('task')
  if task is not None and any(isinstance(o,Coupling) for o in task.objects):
   prev=env.env_cfg_callback
   def configure(cfg):
    if prev is not None:cfg=prev(cfg)
    from isaaclab.sensors import JointWrenchSensorCfg
    from isaaclab.assets import AssetBaseCfg
    from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg
    import isaaclab.sim as sim
    for side in ('left','right'):
     setattr(cfg.scene,side+'_b7_wrench',JointWrenchSensorCfg(prim_path='{ENV_REGEX_NS}/'+side.title()+'Arm'))
     robot=getattr(cfg.scene,side+'_robot');robot.spawn.activate_contact_sensors=True
     p=list(robot.init_state.pos);p[2]=FIX['robot_base_z'];robot.init_state.pos=tuple(p)
     # Real physical15cm support extensions. No robot limit changes.
     setattr(cfg.scene,side+'_b7_pedestal_extension',AssetBaseCfg(prim_path='{ENV_REGEX_NS}/'+side+'_b7_pedestal_extension',spawn=sim.CuboidCfg(size=(.14,.14,.15),collision_props=PhysxCollisionPropertiesCfg(),visual_material=sim.PreviewSurfaceCfg(diffuse_color=(.25,.25,.25))),init_state=AssetBaseCfg.InitialStateCfg(pos=(p[0],p[1],.825))))
     for name,act in robot.actuators.items():
      if 'grip' in name or 'finger' in name:act.joint_effort_limit=40.
    cfg.sim.dt=DT;cfg.decimation=25;cfg.sim.render_interval=25
    return cfg
   env.env_cfg_callback=configure
  return env
 module.task_object,module.object_states,module.build_task_environment=obj,states,build;module._b7_patched=True

def patch_embodiment(module):
 cls=module.YamArenaEmbodiment
 if getattr(cls,'_b7_patched',False):return
 reset0,step0,public0=cls.reset,cls._task_step,cls.step
 def active(self):return self.task is not None and any(isinstance(o,Coupling) for o in self.task.objects)
 def reset(self,*a,**kw):
  global _OBSERVER,_STEPS
  if not active(self):return reset0(self,*a,**kw)
  # Fixture is compiled from physical seed42; Inspect trial seeds do not regenerate it.
  if not math.isclose(float(self.info.control_hz),20.) or str(self.device)!='cpu':raise ValueError('B7 requires20Hz CPU PhysX with allocated RTX cameras')
  _OBSERVER=None;_STEPS=0;self._b7_episode_guard=EpisodeGuard(CONTROL_LIMIT);self._b7_episode_guard.stop('reset_incomplete');self._ensure_sim(None)
  for side in ('left','right'):self._base.scene[side+'_robot']._b7_total_effort_enabled=True
  from observer import Observer
  _OBSERVER=Observer(self)
  asset=self._base.scene['coupling_fixture']
  if not getattr(asset,'_b7_write_callback',False):
   previous=asset.write_data_to_sim
   def write(*aa,**kk):
    if _OBSERVER is not None:_OBSERVER.apply_passive()
    return previous(*aa,**kk)
   asset.write_data_to_sim=write;asset._b7_write_callback=True
  if not getattr(self._base.scene,'_b7_callback',False):
   prev=self._base.scene.update
   def update(dt,*aa,**kk):
    value=prev(dt,*aa,**kk)
    if _OBSERVER is not None:_OBSERVER.sample(float(dt))
    return value
   self._base.scene.update=update;self._base.scene._b7_callback=True
  obs=reset0(self,*a,**kw)
  # Settling is measured for failures but earns no episode credit.
  error=_OBSERVER.error;_OBSERVER.history=History()
  if error:_OBSERVER.history.failure='measurement_unavailable'
  self._b7_episode_guard.reset();return obs
 def step(self):
  global _STEPS
  if not active(self):return step0(self)
  _STEPS+=1;r=step0(self);s=current_status();failure=s['failure'];info=dict(r.info,b7_status=s)
  if failure:return replace(r,terminated=True,termination_reason=failure,info=info)
  if _STEPS>=CONTROL_LIMIT:return replace(r,terminated=True,termination_reason='time_limit',info=info)
  return replace(r,info=info)
 def public(self,action):
  if not active(self):return public0(self,action)
  guard=getattr(self,'_b7_episode_guard',None)
  if guard is None:raise RuntimeError('B7 reset required')
  failure=current_status().get('failure')
  if failure:guard.stop(failure)
  data=np.asarray(action.data,dtype=float)
  if data.shape!=(14,) or not np.isfinite(data).all():guard.stop('invalid_action');raise ValueError('B7 requires finite14D joint/jaw Actions')
  return guard.invoke(lambda:public0(self,action))
 cls.reset,cls._task_step,cls.step=reset,step,public;cls._b7_patched=True

def current_status():return History().result() if _OBSERVER is None else _OBSERVER.result()
def current_score():return float(current_status()['score'])
class Finder(importlib.abc.MetaPathFinder):
 targets={'yam_arena.scene':patch_scene,'yam_arena.inspect_embodiment':patch_embodiment}
 def find_spec(self,fullname,path,target=None):
  if fullname not in self.targets:return None
  sys.meta_path.remove(self)
  try:spec=importlib.util.find_spec(fullname)
  finally:sys.meta_path.insert(0,self)
  if spec is None or spec.loader is None:return None
  execute,patch=spec.loader.exec_module,self.targets[fullname]
  def run(module):execute(module);patch(module)
  spec.loader.exec_module=run;return spec

def install():
 from motor_limits import patch_actions
 Finder.targets['yam_arena.actions']=patch_actions
 for name,patch in Finder.targets.items():
  if name in sys.modules:patch(sys.modules[name])
 if not any(isinstance(f,Finder) for f in sys.meta_path):sys.meta_path.insert(0,Finder())
