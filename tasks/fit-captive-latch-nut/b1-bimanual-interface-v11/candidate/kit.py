"""Task-scoped Arena integration. No global SDK edits or native runtime imports."""
import source_rubric  # byte-identical source executes before cold SDK imports
from dataclasses import dataclass, replace
import importlib.abc, importlib.util, json, math, sys
from pathlib import Path
import numpy as np
from contacts import arr
from motor_limits import patch_actions
from yam_arena.task_file import Box, ObjectState

HERE=Path(__file__).resolve().parent
F=json.loads((HERE/'fixture.json').read_text());ENGINE=None

@dataclass(frozen=True)
class WrenchPart(Box):
    articulation:bool=False

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
        from pxr import Sdf,UsdPhysics
        usd=sim.UsdFileCfg(usd_path=str(HERE/'assets'/('fixture.usda' if spec.articulation else spec.name+'.usda')),activate_contact_sensors=True)
        prim=usd.func(path,usd,translation=translation,orientation=orientation,**kw)
        if spec.articulation:
            # Native head mask2 vs robot/furniture mask1: tool(mask3) only.
            env=path.rsplit('/',1)[0]
            for b in F['bodies']:
                if b['free']:continue
                for g in b['geoms']:
                    if g['contype']==2:
                        shape=prim.GetStage().GetPrimAtPath(path+'/links/'+b['name']+'/'+g['name'])
                        rel=UsdPhysics.FilteredPairsAPI.Apply(shape).CreateFilteredPairsRel()
                        for n in ['LeftArm','RightArm','table','floor','stand_foot','stand_mast','stand_arm','camera_housing']:
                            rel.AddTarget(Sdf.Path(env+'/'+n))
        return prim
    @configclass
    class SpawnCfg(SpawnerCfg):func:object=clone(spawn)
    if spec.articulation:
        class Part(Object):
            def _generate_articulation_cfg(self):
                cfg=ArticulationCfg(prim_path=self.prim_path,spawn=self.spawner_cfg,
                    actuators={j['name']:ImplicitActuatorCfg(joint_names_expr=[j['name']],stiffness=0.,damping=float(j.get('damping',0.))) for b in F['bodies'] for j in b['joints']},
                    init_state=ArticulationCfg.InitialStateCfg(joint_pos={'.*':0.},joint_vel={'.*':0.}))
                return self._add_initial_pose_to_cfg(cfg)
        cls,typ=Part,ObjectType.ARTICULATION
    else:cls,typ=Object,ObjectType.RIGID
    return cls(name=spec.name,object_type=typ,spawner_cfg=SpawnCfg(),initial_pose=Pose(position_xyz=tuple(map(float,spec.pos)),rotation_xyzw=(0,0,math.sin(spec.yaw/2),math.cos(spec.yaw/2))))

def patch_scene(m):
    oldobj,oldstates,oldbuild=m.task_object,m.object_states,m.build_task_environment
    def obj(spec):return object_for(spec) if isinstance(spec,WrenchPart) else oldobj(spec)
    def states(env,names):
        other=[n for n in names if n!='wrench_fixture'];out=oldstates(env,other) if other else {}
        if 'wrench_fixture' in names:
            a=env.unwrapped.scene['wrench_fixture'];p=arr(a.data.root_pos_w)[0]-arr(env.unwrapped.scene.env_origins)[0]
            # Explicit coarse adapter summary. Private evaluator uses actual
            # links/joints/contacts, never this summary for a scored predicate.
            out['wrench_fixture']=ObjectState(pos=p,quat_xyzw=np.array([0,0,1.,0]),lin_vel=np.zeros(3),ang_vel=np.zeros(3),low=p,high=p)
        return out
    def build(*args,**kw):
        env=oldbuild(*args,**kw);task=kw.get('task')
        if task and any(isinstance(s,WrenchPart) for s in task.objects):
            prev=env.env_cfg_callback
            def configure(cfg):
                if prev:cfg=prev(cfg)
                cfg.sim.dt=.002;cfg.decimation=25;cfg.sim.render_interval=25
                from isaaclab.sensors import JointWrenchSensorCfg,ContactSensorCfg
                for side in ['left','right']:
                    robot=getattr(cfg.scene,side+'_robot');robot.actuators['gripper'].joint_effort_limit=40.
                    robot.spawn.activate_contact_sensors=True
                    setattr(cfg.scene,side+'_b1_wrench',JointWrenchSensorCfg(prim_path='{ENV_REGEX_NS}/'+('LeftArm' if side=='left' else 'RightArm')))
                cfg.scene.b1_contact_enabler=ContactSensorCfg(prim_path='{ENV_REGEX_NS}/wrench_fixture/links/swing_arm')
                return cfg
            env.env_cfg_callback=configure
        return env
    m.task_object,m.object_states,m.build_task_environment=obj,states,build

def patch_embodiment(m):
    cls=m.YamArenaEmbodiment
    oldreset,oldstep,oldtask,oldclose=cls.reset,cls.step,cls._task_step,cls.close
    def active(s):return s.task is not None and any(isinstance(o,WrenchPart) for o in s.task.objects)
    def reset(self,*args,**kw):
        global ENGINE
        if not active(self):return oldreset(self,*args,**kw)
        if self.object_jitter!=0 or not math.isclose(float(self.info.control_hz),20.):raise ValueError('W1 fixed setup requires object_jitter=0,control_hz=20; all Inspect episode seeds accepted')
        self._ensure_sim(None)
        for side in ['left','right']:self._base.scene[side+'_robot']._b1_total_effort_enabled=True
        if ENGINE is None or ENGINE.base is not self._base:
            from physics import Physics
            ENGINE=Physics(self,F)
            pw,pu,pr=self._base.scene.write_data_to_sim,self._base.scene.update,self._env.reset
            def write(*a,**k):ENGINE.pre_step(float(self._base.physics_dt));return pw(*a,**k)
            def update(dt,*a,**k):v=pu(dt,*a,**k);ENGINE.post_step(float(dt));return v
            def envreset(*a,**k):v=pr(*a,**k);ENGINE.reset_mechanisms();return v
            self._base.scene.write_data_to_sim=write;self._base.scene.update=update;self._env.reset=envreset
        ENGINE.settling=True;obs=oldreset(self,*args,**kw);ENGINE.begin();return obs
    def step(self,action):
        if active(self):
            if ENGINE is None or ENGINE.done:raise RuntimeError('W1 requires reset; step after termination forbidden')
            v=np.asarray(action.data,float)
            if v.shape!=(14,) or not np.isfinite(v).all():raise ValueError('W1 requires finite genuine14D robot Action')
        return oldstep(self,action)
    def taskstep(self):
        if not active(self):return oldtask(self)
        ENGINE.controls+=1;result=oldtask(self);reason=ENGINE.reason();ENGINE.done=reason is not None or result.terminated
        return replace(result,terminated=ENGINE.done,termination_reason=reason or result.termination_reason,info=dict(result.info,b1=ENGINE.status()))
    def close(self,*a,**kw):
        if ENGINE is not None and ENGINE.emb is self:ENGINE.close()
        return oldclose(self,*a,**kw)
    cls.reset,cls.step,cls._task_step,cls.close=reset,step,taskstep,close

class Finder(importlib.abc.MetaPathFinder):
    targets={'yam_arena.scene':patch_scene,'yam_arena.actions':patch_actions,'yam_arena.inspect_embodiment':patch_embodiment}
    def find_spec(self,fullname,path,target=None):
        if fullname not in self.targets:return None
        sys.meta_path.remove(self)
        try:spec=importlib.util.find_spec(fullname)
        finally:sys.meta_path.insert(0,self)
        if not spec or not spec.loader:return None
        execute,patch=spec.loader.exec_module,self.targets[fullname]
        def load(m):execute(m);patch(m)
        spec.loader.exec_module=load;return spec

_INSTALLED=False
def install():
    global _INSTALLED
    if _INSTALLED:return
    for name,patch in Finder.targets.items():
        if name in sys.modules:patch(sys.modules[name])
    sys.meta_path.insert(0,Finder());_INSTALLED=True

def current_score():return ENGINE.status()['score'] if ENGINE else 0.
