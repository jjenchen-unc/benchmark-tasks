"""Task-local Arena integration, genuine 14D control, measured substep history."""
from dataclasses import dataclass,replace
from pathlib import Path
import importlib.abc,importlib.util,json,os,sys,math
import numpy as np
from telemetry_json import dumps
from yam_arena.task_file import Box,ObjectState
from motor_limits import patch_actions
from contacts import arr,body_paths,Contacts,rot_xyzw
from mechanisms import Mechanism
from rubric import History
HERE=Path(__file__).resolve().parent
FIXTURE=json.loads((HERE/'fixture.json').read_text());TARGET=os.environ.get('S1_TARGET','SH-T1');F=FIXTURE['fixtures'][TARGET]
SPECS={};ENGINE=None
# Only root-owned actual sensor/mechanism/motor tests may populate this after
# matching package hashes. Never enabled by static/native evidence.
VERIFIED_CHANNELS=frozenset()
@dataclass(frozen=True)
class Enclosures(Box):pass

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
  usd=sim.UsdFileCfg(usd_path=str(HERE/'assets'/f'{TARGET}.usda'),activate_contact_sensors=True)
  return usd.func(path,usd,translation=translation,orientation=orientation,**kw)
 @configclass
 class SpawnCfg(SpawnerCfg):func:object=clone(spawn)
 class Part(Object):
  def _generate_articulation_cfg(self):
   joints={j['name']:0. for b in F['bodies'] for j in b['joints']}
   joints.update({n:-.02 for n in joints if n.startswith('door_')})
   cfg=ArticulationCfg(prim_path=self.prim_path,spawn=self.spawner_cfg,actuators={'passive':ImplicitActuatorCfg(joint_names_expr=['.*'],stiffness=0.,damping=0.)},init_state=ArticulationCfg.InitialStateCfg(joint_pos=joints,joint_vel={'.*':0.}))
   return self._add_initial_pose_to_cfg(cfg)
 return Part(name=spec.name,object_type=ObjectType.ARTICULATION,spawner_cfg=SpawnCfg(),initial_pose=Pose(position_xyz=tuple(spec.pos),rotation_xyzw=(0.,0.,0.,1.)))

class Physics:
 def __init__(self,emb):
  self.emb=emb;self.base=emb._base;self.asset=self.base.scene['enclosures'];self.joints={j['name']:j for b in F['bodies'] for j in b['joints']};self.settling=True;self.done=False;self.trace=None
  import carb.settings
  if carb.settings.get_settings().get('/physics/disableContactProcessing') is not False:raise RuntimeError('S1 requires actual CPU contact processing enabled by standard ContactSensor')
  from isaaclab_physx.physics import PhysxManager
  self.simview=PhysxManager.get_physics_sim_view();self.contacts={}
  robots=sum([body_paths(self.base.scene[s+'_robot']) for s in ['left','right']],[])
  paths=body_paths(self.asset)
  for i in F['boxes']:
   selected=[p for p in paths if p.rsplit('/',1)[-1] in [f'{prefix}_{i}' for prefix in ['box','door','swing','push','lever','cap']]]
   self.contacts[i]=Contacts(self.simview,selected,robots)
  self.initialize()
 def initialize(self):
  import torch
  self.mech={i:Mechanism(s) for i,s in F['boxes'].items()};self.hist=History(F['target']);self.controls=0;self.done=False
  q=torch.zeros_like(self.asset.data.joint_pos.torch)
  for index,n in enumerate(self.asset.joint_names):q[:,index]=-.02 if n.startswith('door_') else 0.
  self.asset.write_joint_state_to_sim(q,torch.zeros_like(q));self.asset.set_joint_effort_target(torch.zeros_like(q));self.last=None
 def begin(self):
  self.hist=History(F['target']);self.controls=0;self.done=False;self.settling=False
  if self.trace:self.trace.close()
  self.trace=open(os.environ['S1_TRACE'],'a',buffering=1) if os.environ.get('S1_TRACE') else None
 def state(self):
  return dict(zip(self.asset.joint_names,arr(self.asset.data.joint_pos)[0])),dict(zip(self.asset.joint_names,arr(self.asset.data.joint_vel)[0]))
 def pre_step(self,dt):
  import torch
  q,v=self.state();effort={n:0. for n in q}
  for i,m in self.mech.items():
   keys=[n for n in q if n.endswith('_'+i)];qq={n.rsplit('_',1)[0]:q[n] for n in keys};vv={n.rsplit('_',1)[0]:v[n] for n in keys}
   passive=m.force(qq,vv)
   for k,value in passive.items():effort[k+'_'+i]+=value
  for n,j in self.joints.items():
   effort[n]+=-j['stiffness']*(q[n]-j['springref'])-j['damping']*v[n]
  self.asset.set_joint_effort_target(torch.tensor([[effort[n] for n in self.asset.joint_names]],device=self.base.device,dtype=torch.float32))
 def post_step(self,dt):
  if self.settling:return
  q,v=self.state();forces=[]
  for side in ['left','right']:
   s=self.base.scene[side+'_s1_wrench'];forces.append(float(np.linalg.norm(arr(s.data.force)[0,s.body_names.index('gripper')])))
  i=str(F['target']['box']);m=self.mech[i];touch,force,count=self.contacts[i].read(dt)
  row={n:q[n+'_'+i] for n in ['door','swing','pop']};row.update(wrist_force_n=forces,robot_contact=touch,contact_count=count,contact_force_n=force,order=m.spec['order'],engaged=m.engaged,released=m.released,pop_catch=m.pop_catch,swing_lock=m.swing_lock)
  self.hist.sample(row,dt);self.last=row
  if self.trace:self.trace.write(dumps(dict(control=self.controls,measurement=row,status=self.hist.status(VERIFIED_CHANNELS)))+'\n')
 def status(self):return self.hist.status(VERIFIED_CHANNELS)
 def reason(self):
  if self.hist.failure:return self.hist.failure
  if self.controls>=4198:return 'time_limit'
  if self.status()['score']>=1.:return 'success'
  return None

def bounds(base):
 asset=base.scene['enclosures'];poses=dict(zip(asset.body_names,zip(arr(asset.data.body_pos_w)[0]-arr(base.scene.env_origins)[0],arr(asset.data.body_quat_w)[0])));lo=[];hi=[]
 for b in F['bodies']:
  pos,q=poses[b['name']];R=rot_xyzw(q)
  for g in b['geoms']:
   if not g['collision']:continue
   Q=R@rot_xyzw(np.r_[g['quaternion_wxyz'][1:],g['quaternion_wxyz'][0]]);c=pos+R@np.array(g['position']);s=g['size'];z=Q[:,2]
   e=abs(Q)@np.array(s) if g['kind']=='box' else np.ones(3)*s[0] if g['kind']=='sphere' else abs(z)*s[1]+s[0]*np.sqrt(np.maximum(0,1-z*z))
   lo.append(c-e);hi.append(c+e)
 return np.min(lo,axis=0),np.max(hi,axis=0)

def patch_scene(module):
 if getattr(module,'_s1_patched',False):return
 old_object,old_states,old_build=module.task_object,module.object_states,module.build_task_environment
 def obj(spec):
  if isinstance(spec,Enclosures):SPECS[spec.name]=spec;return object_for(spec)
  return old_object(spec)
 def states(env,names):
  mine=[n for n in names if n in SPECS];out=old_states(env,[n for n in names if n not in mine]) if len(mine)<len(names) else {}
  for n in mine:
   lo,hi=bounds(env.unwrapped);out[n]=ObjectState(pos=(lo+hi)/2,quat_xyzw=np.array([0,0,0,1]),lin_vel=np.zeros(3),ang_vel=np.zeros(3),low=lo,high=hi)
  return out
 def build(*a,**kw):
  env=old_build(*a,**kw);task=kw.get('task')
  if task and any(isinstance(o,Enclosures) for o in task.objects):
   previous=env.env_cfg_callback
   def configure(cfg):
    if previous:cfg=previous(cfg)
    cfg.sim.dt=.001;cfg.decimation=50;cfg.sim.render_interval=50
    from isaaclab.sensors import JointWrenchSensorCfg,ContactSensorCfg
    cfg.scene.s1_contact_enabler=ContactSensorCfg(prim_path='{ENV_REGEX_NS}/enclosures/links/door_'+str(F['target']['box']))
    for side in ['left','right']:
     setattr(cfg.scene,side+'_s1_wrench',JointWrenchSensorCfg(prim_path='{ENV_REGEX_NS}/'+side.title()+'Arm'))
     robot=getattr(cfg.scene,side+'_robot');robot.spawn.activate_contact_sensors=True;robot.actuators['gripper'].joint_effort_limit=50.
    return cfg
   env.env_cfg_callback=configure
  return env
 module.task_object,module.object_states,module.build_task_environment=obj,states,build;module._s1_patched=True

def patch_embodiment(module):
 cls=module.YamArenaEmbodiment
 if getattr(cls,'_s1_patched',False):return
 old_reset,old_step,old_task,old_close=cls.reset,cls.step,cls._task_step,cls.close
 def active(self):return self.task is not None and any(isinstance(o,Enclosures) for o in self.task.objects)
 def reset(self,*a,**kw):
  global ENGINE
  if not active(self):return old_reset(self,*a,**kw)
  if not math.isclose(self.info.control_hz,20.):raise ValueError('S1 requires 20Hz')
  if self.object_jitter!=0:raise ValueError('S1 declares one fixed physical case: object_jitter=0')
  self._ensure_sim(None)
  for side in ['left','right']:self._base.scene[side+'_robot']._s1_total_effort_enabled=True
  if ENGINE is None or ENGINE.base is not self._base:
   ENGINE=Physics(self);write=self._base.scene.write_data_to_sim;update=self._base.scene.update;resetenv=self._env.reset
   def wrapped_write(*aa,**kk):ENGINE.pre_step(self._base.sim.get_physics_dt());return write(*aa,**kk)
   def wrapped_update(dt,*aa,**kk):value=update(dt,*aa,**kk);ENGINE.post_step(float(dt));return value
   def wrapped_reset(*aa,**kk):value=resetenv(*aa,**kk);ENGINE.initialize();return value
   self._base.scene.write_data_to_sim=wrapped_write;self._base.scene.update=wrapped_update;self._env.reset=wrapped_reset
  ENGINE.settling=True;obs=old_reset(self,*a,**kw);ENGINE.begin();return obs
 def step(self,action):
  if active(self):
   if ENGINE is None or ENGINE.done:raise RuntimeError('S1 reset required; episode is terminated')
   a=np.asarray(action.data)
   if a.shape!=(14,) or not np.isfinite(a).all():raise ValueError('finite genuine14D robot target required')
  return old_step(self,action)
 def task_step(self):
  if not active(self):return old_task(self)
  ENGINE.controls+=1;result=old_task(self);reason=ENGINE.reason();ENGINE.done=bool(reason) or result.terminated
  return replace(result,terminated=ENGINE.done,termination_reason=reason or result.termination_reason,info=dict(result.info,s1=ENGINE.status()))
 def close(self,*aa,**kk):
  if ENGINE is not None and ENGINE.trace:ENGINE.trace.close();ENGINE.trace=None
  return old_close(self,*aa,**kk)
 cls.reset,cls.step,cls._task_step,cls.close=reset,step,task_step,close;cls._s1_patched=True

class Finder(importlib.abc.MetaPathFinder):
 targets={'yam_arena.scene':patch_scene,'yam_arena.actions':patch_actions,'yam_arena.inspect_embodiment':patch_embodiment}
 def find_spec(self,fullname,path,target=None):
  if fullname not in self.targets:return None
  sys.meta_path.remove(self)
  try:spec=importlib.util.find_spec(fullname)
  finally:sys.meta_path.insert(0,self)
  if spec is None or spec.loader is None:return None
  execute,patch=spec.loader.exec_module,self.targets[fullname]
  def execute_module(module):execute(module);patch(module)
  spec.loader.exec_module=execute_module;return spec

def install():
 for name,patch in Finder.targets.items():
  if name in sys.modules:patch(sys.modules[name])
 if not any(isinstance(x,Finder) for x in sys.meta_path):sys.meta_path.insert(0,Finder())
def current_score():return ENGINE.status()['score'] if ENGINE else 0.
