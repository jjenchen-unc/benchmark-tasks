"""Task-scoped Arena integration. Environment forces implement passive mechanics;
policies can only submit the standard 14D joint-position/jaw Action.
"""
from dataclasses import dataclass,replace
from pathlib import Path
import importlib.abc,importlib.util,json,math,sys
import numpy as np
from yam_arena.task_file import Box,ObjectState
from step_guard import EpisodeGuard
from mechanism import Mechanism,History,REQUIRED,CONTROL_LIMIT,DT,cam
HERE=Path(__file__).resolve().parent;FIX=json.loads((HERE/'fixture.json').read_text());_OBSERVER=None;_SPECS={};_STEPS=0
@dataclass(frozen=True)
class Latch(Box):pass

def arr(x):
    x=getattr(x,'torch',x)
    if hasattr(x,'detach'):return x.detach().cpu().numpy()
    return np.asarray(x)

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
    def spawn(prim_path,cfg,translation=None,orientation=None,**kw):
        from pxr import UsdPhysics,Sdf
        cfg=sim.UsdFileCfg(usd_path=str(HERE/'latch.usda'),activate_contact_sensors=True)
        prim=cfg.func(prim_path,cfg,translation=translation,orientation=orientation,**kw)
        # Source pawl mask4 never contacts robot/furniture, only flange mask5.
        env=prim_path.rsplit('/',1)[0];p=prim.GetStage().GetPrimAtPath(prim_path+'/links/pawl/pawl_tip');rel=UsdPhysics.FilteredPairsAPI.Apply(p).CreateFilteredPairsRel()
        for name in ('LeftArm','RightArm','floor','table','stand_foot','stand_mast','stand_arm','camera_housing','lamp','left_robot_pedestal','right_robot_pedestal'):rel.AddTarget(Sdf.Path(env+'/'+name))
        return prim
    @configclass
    class SpawnCfg(SpawnerCfg):func:object=clone(spawn)
    class Part(Object):
        def _generate_articulation_cfg(self):
            actuators={n:ImplicitActuatorCfg(joint_names_expr=[n],stiffness=k,damping=d) for n,k,d in [('lid_slide',FIX['spec']['k'],200.),('handle',0.,.01),('knob',0.,.02),('pawl',0.,5.)]}
            cfg=ArticulationCfg(prim_path=self.prim_path,spawn=self.spawner_cfg,actuators=actuators,init_state=ArticulationCfg.InitialStateCfg(joint_pos={'.*':0.},joint_vel={'.*':0.}))
            return self._add_initial_pose_to_cfg(cfg)
    return Part(name=spec.name,object_type=ObjectType.ARTICULATION,spawner_cfg=SpawnCfg(),initial_pose=Pose(position_xyz=tuple(spec.pos),rotation_xyzw=(0.,0.,math.sin(FIX.get('arena_yaw_rad',0.)/2),math.cos(FIX.get('arena_yaw_rad',0.)/2))))

def patch_scene(module):
    if getattr(module,'_s4_patched',False):return
    old_object,old_states,old_build=module.task_object,module.object_states,module.build_task_environment
    def obj(spec):
        if isinstance(spec,Latch):_SPECS[spec.name]=spec;return object_for(spec)
        return old_object(spec)
    def states(env,names):
        out=old_states(env,[n for n in names if n not in _SPECS]) if any(n not in _SPECS for n in names) else {}
        for n in names:
            if n in _SPECS:
                p=np.array(_SPECS[n].pos);yaw=FIX.get('arena_yaw_rad',0.);c,s=math.cos(yaw),math.sin(yaw);R=np.array([[c,-s,0],[s,c,0],[0,0,1.]])
                corners=np.array([[x,y,z] for x in (-.15,.162) for y in (-.20,.20) for z in (0.,.21)])@R.T+p
                lo=corners.min(axis=0);hi=corners.max(axis=0)
                out[n]=ObjectState(pos=p,quat_xyzw=np.array([0.,0.,0.,1.]),lin_vel=np.zeros(3),ang_vel=np.zeros(3),low=lo,high=hi)
        return out
    def build(*a,**kw):
        env=old_build(*a,**kw);task=kw.get('task')
        if task is not None and any(isinstance(o,Latch) for o in task.objects):
            prev=env.env_cfg_callback
            def configure(cfg):
                if prev is not None:cfg=prev(cfg)
                from isaaclab.sensors import JointWrenchSensorCfg,ContactSensorCfg
                for side in ('left','right'):
                    setattr(cfg.scene,side+'_s4_wrench',JointWrenchSensorCfg(prim_path='{ENV_REGEX_NS}/'+side.title()+'Arm'))
                    robot=getattr(cfg.scene,side+'_robot');robot.spawn.activate_contact_sensors=True
                    from yam_arena.station import BASES
                    robot.init_state.pos=tuple(np.asarray(BASES[side])+[0.,0.,FIX.get('robot_base_rise_m',0.)])
                    # YAM hardware50N is below source235N ceiling; right source40N.
                    # Lower capacity needs actual loaded feasibility validation.
                    for name,act in robot.actuators.items():
                        if 'grip' in name or 'finger' in name:
                            act.joint_effort_limit=50. if side=='left' else 40.
                cfg.scene.s4_contacts=ContactSensorCfg(prim_path='{ENV_REGEX_NS}/compression_latch/links/lid')
                cfg.sim.dt=DT;cfg.decimation=25;cfg.sim.render_interval=25
                return cfg
            env.env_cfg_callback=configure
        return env
    module.task_object,module.object_states,module.build_task_environment=obj,states,build;module._s4_patched=True

def patch_embodiment(module):
    cls=module.YamArenaEmbodiment
    if getattr(cls,'_s4_patched',False):return
    old_reset,old_step,old_public_step=cls.reset,cls._task_step,cls.step
    def active(self):return self.task is not None and any(isinstance(o,Latch) for o in self.task.objects)
    def reset(self,*a,**kw):
        global _OBSERVER,_STEPS
        if not active(self):return old_reset(self,*a,**kw)
        # Inspect derives a per-trial seed from the CLI seed and scene identity.
        # FIX remains the declared source-seed42 fixture for every harness seed.
        if self.object_jitter!=0:raise ValueError('S4 fixed fixture requires object_jitter=0')
        if not math.isclose(float(self.info.control_hz),20.):raise ValueError('S4 requires 20Hz')
        if str(self.device)!='cpu':raise ValueError('S4 candidate requires CPU PhysX pending channel calibration')
        _OBSERVER=None;_STEPS=0
        # Remain stopped if reset itself fails; settling bypasses public step.
        self._s4_episode_guard=EpisodeGuard(CONTROL_LIMIT)
        self._s4_episode_guard.stop('reset_incomplete')
        self._ensure_sim(None)
        import torch
        asset=self._base.scene['compression_latch'];q=torch.zeros_like(asset.data.joint_pos.torch)
        q[:,asset.joint_names.index('pawl')]=cam(0,FIX['spec']['grip'],0)[0]
        q[:,asset.joint_names.index('lid_slide')]=9.81*1.6/FIX['spec']['k']
        # RESET ONLY: standard initial mechanism state before episode observation.
        asset.write_joint_state_to_sim(q,torch.zeros_like(q));asset.set_joint_effort_target(torch.zeros_like(q));asset.set_joint_position_target(torch.zeros_like(q))
        for side in ('left','right'):self._base.scene[side+'_robot']._s4_total_effort_enabled=True
        from observer import Observer
        _OBSERVER=Observer(self)
        if not getattr(asset,'_s4_callback',False):
            prevwrite=asset.write_data_to_sim
            def write(*args,**kwargs):
                if _OBSERVER is not None:_OBSERVER.apply_passive()
                return prevwrite(*args,**kwargs)
            asset.write_data_to_sim=write;asset._s4_callback=True
        if not getattr(self._base.scene,'_s4_callback',False):
            prevupdate=self._base.scene.update
            def update(dt,*args,**kwargs):
                result=prevupdate(dt,*args,**kwargs)
                if _OBSERVER is not None:_OBSERVER.sample(float(dt))
                return result
            self._base.scene.update=update;self._base.scene._s4_callback=True
        obs=old_reset(self,*a,**kw)
        # Reset settling never earns episode hold credit.
        _OBSERVER.history=History()
        self._s4_episode_guard.reset()
        return obs
    def step(self):
        global _STEPS
        if not active(self):return old_step(self)
        _STEPS+=1;result=old_step(self);status=current_status();failure=status['failure']
        info=dict(result.info,s4_status=status)
        if failure:return replace(result,terminated=True,termination_reason=failure,info=info)
        if _STEPS>=CONTROL_LIMIT:return replace(result,terminated=True,termination_reason='time_limit',info=info)
        return replace(result,info=info)
    def public_step(self,action):
        if not active(self):return old_public_step(self,action)
        guard=getattr(self,'_s4_episode_guard',None)
        if guard is None:raise RuntimeError('S4 reset required before step')
        # Observe an already-known failure before entering SDK physics.
        failure=current_status().get('failure')
        if failure:guard.stop(failure)
        return guard.invoke(lambda:old_public_step(self,action))
    cls.reset,cls._task_step,cls.step=reset,step,public_step;cls._s4_patched=True

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
