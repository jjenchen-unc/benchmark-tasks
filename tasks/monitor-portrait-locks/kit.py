"""Task-local Arena adapter for the compiled B5 USD fixture.

Only static passive spring targets are written; no object/joint poses, unlocks,
score fields or timers are changed to force a result. Genuine YAM remains the
installed embodiment. Runtime scoring stays gated until contact/wrench observers
are installed and tested at physics-substep resolution.
"""
from __future__ import annotations
from dataclasses import dataclass,replace
import importlib.abc,importlib.util,json,math,os,sys
from pathlib import Path
import numpy as np
from yam_arena.task_file import Box,ObjectState
from contract import evaluate
HERE=Path(__file__).resolve().parent
FIXTURE=json.loads((HERE/'fixture.json').read_text())
_SPECS={};_TRACE=None;_STEPS=0;_OBSERVER=None

@dataclass(frozen=True)
class Monitor(Box):
    pass

def arr(x):return getattr(x,'torch',x).detach().cpu().numpy()

def rot(q):
    w,x,y,z=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def rot_xyzw(q):
    # IsaacLab3 ProxyArray/wp.quatf values are xyzw; frozen MuJoCo JSON is wxyz.
    return rot(np.r_[q[3],q[:3]])

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
        from pxr import Sdf,UsdPhysics
        usd=sim.UsdFileCfg(usd_path=str(HERE/'monitor.usda'),activate_contact_sensors=True)
        prim=usd.func(prim_path,usd,translation=translation,orientation=orientation,**kw)
        # Add exclusions to the composed scene, where external prim targets are
        # valid. Task objects spawn before the arms, so target their known paths
        # without requiring them to exist yet. This never filters visible task
        # objects against robot/table; only the hidden mask2 lock parts are affected.
        env_path=prim_path.rsplit('/',1)[0]
        external=[Sdf.Path(env_path+'/'+name) for name in ('LeftArm','RightArm','floor','table','stand_foot','stand_mast','stand_arm','camera_housing','lamp','left_robot_pedestal','right_robot_pedestal')]
        for body in FIXTURE['bodies']:
            for geom in body['geoms']:
                if geom['contype']==2:
                    shape=prim.GetStage().GetPrimAtPath(prim_path+'/links/'+body['name']+'/'+geom['name'])
                    rel=UsdPhysics.FilteredPairsAPI.Apply(shape).CreateFilteredPairsRel()
                    for target in external:rel.AddTarget(target)
        # The native fixed stand root and fixed furniture do not collide. In
        # PhysX an articulated fixed child otherwise shares support with table
        # contacts, hiding part of the foot reaction from its load cell.
        fixed=next(b for b in FIXTURE['bodies'] if b['name']=='monitor_stand')
        for geom in fixed['geoms']:
            shape=prim.GetStage().GetPrimAtPath(prim_path+'/links/monitor_stand/'+geom['name'])
            rel=UsdPhysics.FilteredPairsAPI.Apply(shape).CreateFilteredPairsRel()
            for target in ('table','floor'):rel.AddTarget(Sdf.Path(env_path+'/'+target))
        return prim
    @configclass
    class SpawnCfg(SpawnerCfg):func:object=clone(spawn)
    class Part(Object):
        def _generate_articulation_cfg(self):
            actuators={j['name']:ImplicitActuatorCfg(joint_names_expr=[j['name']],stiffness=j['stiffness'],damping=j['damping']) for b in FIXTURE['bodies'] for j in b['joints']}
            cfg=ArticulationCfg(prim_path=self.prim_path,spawn=self.spawner_cfg,actuators=actuators,init_state=ArticulationCfg.InitialStateCfg(joint_pos={'.*':0.},joint_vel={'.*':0.}))
            return self._add_initial_pose_to_cfg(cfg)
    return Part(name=spec.name,object_type=ObjectType.ARTICULATION,spawner_cfg=SpawnCfg(),initial_pose=Pose(position_xyz=tuple(spec.pos),rotation_xyzw=(0.,0.,math.sin(spec.yaw/2),math.cos(spec.yaw/2))))


def passive(base):
    import torch
    refs={j['name']:j['springref'] for b in FIXTURE['bodies'] for j in b['joints']}
    for name in _SPECS:
        asset=base.scene[name];q=getattr(asset.data.joint_pos,'torch',asset.data.joint_pos)
        targets=torch.tensor([refs[n] for n in asset.joint_names],dtype=q.dtype,device=q.device).expand_as(q)
        asset.set_joint_position_target(targets)


def bounds(base,name,spec):
    asset=base.scene[name];origin=arr(base.scene.env_origins)[0]
    poses=dict(zip(asset.body_names,zip(arr(asset.data.body_pos_w)[0]-origin,arr(asset.data.body_quat_w)[0])))
    rootR=rot([math.cos(spec.yaw/2),0,0,math.sin(spec.yaw/2)]);low=[];high=[]
    for b in FIXTURE['bodies']:
        if b['name'] in poses:pos,q=poses[b['name']];R=rot_xyzw(q)
        else:pos=np.array(spec.pos)+rootR@(np.array(b['native_world_pos'])-FIXTURE['native_origin']);R=rootR@rot(b['native_world_quat_wxyz'])
        for g in b['geoms']:
            if not(g['contype'] or g['conaffinity']):continue
            gp=np.array(g['pos'])-(np.array(FIXTURE['native_origin']) if b['name']=='monitor_stand' else 0.)
            c=pos+R@gp;Q=R@rot(g['quat_wxyz']);sz=g['size']
            if g['kind']=='box':extent=abs(Q)@np.array(sz)
            else:
                z=Q[:,2];extent=sz[1]*abs(z)+(sz[0] if g['kind']=='capsule' else sz[0]*np.sqrt(np.maximum(0,1-z*z)))
            low.append(c-extent);high.append(c+extent)
    return np.min(low,axis=0),np.max(high,axis=0)


def patch_actions(module):
    """B5-only total motor bound: gravity joins PD inside the implicit drive.

    The default SDK clips only PD at27/7Nm then adds gravity as a separate
    effort. Actual contact-free43386 requires36.1Nm on joint2. Representing
    G as the internal target offset G/K preserves unsaturated PD+G exactly,
    while the existing PhysX drive cap now bounds the total. Public targets,
    joints, gains, damping, velocity limits and robot assets are unchanged.
    """
    cls=module.YamArmAction
    if getattr(cls,'_b5_total_effort_patched',False):return
    original=cls.apply_actions
    def apply_actions(self):
        if not getattr(self._asset,'_b5_total_effort_enabled',False):return original(self)
        import torch
        if not hasattr(self,'_b5_drive_stiffness'):
            from physics_observer import arr as measured_array
            stiffness=torch.as_tensor(measured_array(self._asset.root_view.get_dof_stiffnesses()).copy(),device=self.device,dtype=self._processed.dtype)[:,self._joint_ids]
            limits=torch.as_tensor(measured_array(self._asset.root_view.get_dof_max_forces()).copy(),device=self.device,dtype=self._processed.dtype)[:,self._joint_ids]
            expected_K=torch.tensor([80.,80.,80.,10.,10.,10.],device=self.device)
            expected_L=torch.tensor([27.,27.,27.,7.,7.,7.],device=self.device)
            if not torch.isfinite(stiffness).all() or not torch.allclose(stiffness,expected_K.expand_as(stiffness)) or not torch.allclose(limits,expected_L.expand_as(limits)):raise RuntimeError('B5 total-effort contract requires verified SDK drive gains/caps')
            self._b5_drive_stiffness=stiffness
        gravity=torch.zeros_like(self._processed)
        if self.cfg.gravity_compensation:
            full=self._asset.data.gravity_compensation_forces.torch
            if full.shape[1]==self._asset.num_joints+6:full=full[:,6:]
            gravity=full[:,self._joint_ids]
        if not torch.isfinite(gravity).all():raise RuntimeError('B5 nonfinite gravity compensation')
        command=self._asset.actuators.target_command
        command.set_position_index(value=(self._processed+gravity/self._b5_drive_stiffness).contiguous(),joint_ids=self._joint_ids)
        command.set_effort_index(value=torch.zeros_like(self._processed),joint_ids=self._joint_ids)
        if self._camera is not None:self.update_wrist_camera()
    cls.apply_actions=apply_actions;cls._b5_total_effort_patched=True


def patch_scene(module):
    if getattr(module,'_b5_monitor_patched',False):return
    old_object,old_states,old_build=module.task_object,module.object_states,module.build_task_environment
    def task_object(spec):
        if isinstance(spec,Monitor):_SPECS[spec.name]=spec;return object_for(spec)
        return old_object(spec)
    def object_states(env,names):
        mine=[n for n in names if n in _SPECS]
        out=old_states(env,[n for n in names if n not in _SPECS]) if len(mine)<len(names) else {}
        for n in mine:
            spec=_SPECS[n];lo,hi=bounds(env.unwrapped,n,spec)
            out[n]=ObjectState(pos=np.array(spec.pos),quat_xyzw=np.array([0.,0.,math.sin(spec.yaw/2),math.cos(spec.yaw/2)]),lin_vel=np.zeros(3),ang_vel=np.zeros(3),low=lo,high=hi)
        return out
    def build(*a,**kw):
        env=old_build(*a,**kw)
        task=kw.get('task')
        if task is not None and any(isinstance(o,Monitor) for o in task.objects):
            prior=env.env_cfg_callback
            def configure(cfg):
                if prior is not None:cfg=prior(cfg)
                from isaaclab.sensors import JointWrenchSensorCfg
                cfg.scene.left_b5_wrench=JointWrenchSensorCfg(prim_path='{ENV_REGEX_NS}/LeftArm')
                cfg.scene.right_b5_wrench=JointWrenchSensorCfg(prim_path='{ENV_REGEX_NS}/RightArm')
                cfg.scene.b5_base_wrench=JointWrenchSensorCfg(prim_path='{ENV_REGEX_NS}/monitor_stand')
                # Standard sensor enables process-local CPU contact processing,
                # which PhysxManager otherwise disables even for raw tensor views.
                from isaaclab.sensors import ContactSensorCfg
                cfg.scene.b5_root_contacts=ContactSensorCfg(prim_path='{ENV_REGEX_NS}/monitor_stand/links/monitor_stand')
                cfg.scene.left_robot.spawn.activate_contact_sensors=True
                cfg.scene.right_robot.spawn.activate_contact_sensors=True
                for robot in (cfg.scene.left_robot,cfg.scene.right_robot):
                    xyz=robot.init_state.pos
                    robot.init_state.pos=(xyz[0],xyz[1],xyz[2]+.30)
                    # Preserve the native B5 task's40N per-finger ceiling;
                    # official YAM defaults to50N, not this task-specific limit.
                    robot.actuators['gripper'].joint_effort_limit=40.
                return cfg
            env.env_cfg_callback=configure
        return env
    module.task_object,module.object_states,module.build_task_environment=task_object,object_states,build;module._b5_monitor_patched=True


def patch_embodiment(module):
    cls=module.YamArenaEmbodiment
    if getattr(cls,'_b5_monitor_patched',False):return
    old_reset,old_step=cls.reset,cls._task_step
    def active(self):return self.task is not None and any(isinstance(o,Monitor) for o in self.task.objects)
    def reset(self,*a,**kw):
        global _TRACE,_STEPS,_OBSERVER
        if not active(self):return old_reset(self,*a,**kw)
        if not math.isclose(float(self.info.control_hz),20.):raise ValueError('B5 requires control_hz=20')
        if str(self.device)!='cpu':raise ValueError('B5 requires device=cpu: GPU PhysX incoming base reaction failed known-load calibration; RTX cameras still use GPU')
        _OBSERVER=None
        self._ensure_sim(None)
        for name in _SPECS:
            asset=self._base.scene[name]
            if not getattr(asset,'_b5_passive_callback',False):
                previous=asset.write_data_to_sim
                def write(*args,_prev=previous,**kwargs):passive(self._base);return _prev(*args,**kwargs)
                asset.write_data_to_sim=write;asset._b5_passive_callback=True
            # Arena's generic object reset restores the root, not custom fixture
            # joint state. Reset the nine native joints only at episode reset,
            # before old_reset settles physics and produces observations.
            import torch
            zeros=torch.zeros_like(asset.data.joint_pos.torch)
            asset.set_joint_effort_target(zeros)
            asset.write_joint_state_to_sim(zeros,zeros)
        for side in ('left','right'):self._base.scene[side+'_robot']._b5_total_effort_enabled=True
        obs=old_reset(self,*a,**kw);passive(self._base);_STEPS=0
        from physics_observer import Observer
        _OBSERVER=Observer(self,FIXTURE,_SPECS['monitor_stand'])
        if not getattr(self._base.scene,'_b5_sample_callback',False):
            previous=self._base.scene.update
            def update(dt,*args,**kwargs):
                result=previous(dt,*args,**kwargs)
                if _OBSERVER is not None and _OBSERVER.base.scene is self._base.scene:_OBSERVER.sample(float(dt))
                return result
            self._base.scene.update=update;self._base.scene._b5_sample_callback=True
        if _TRACE is not None:_TRACE.close()
        _TRACE=open(os.environ['B5_TRACE'],'w',buffering=1) if os.environ.get('B5_TRACE') else None
        return obs
    def task_step(self):
        global _STEPS
        if not active(self):return old_step(self)
        _STEPS+=1
        status=current_status()
        if _TRACE is not None:
            _TRACE.write(json.dumps({'step':_STEPS,'status':status,'measurement':_OBSERVER.history.measurement() if _OBSERVER else None})+'\n')
        result=old_step(self)
        info=dict(result.info,b5_score_gate=status.get('gate'),b5_measurement_error=status.get('measurement_error'),b5_physics_samples=status.get('physics_samples',0))
        failure=_OBSERVER.history.failure if _OBSERVER else 'observer_unavailable'
        if failure:return replace(result,terminated=True,termination_reason=failure,info=info)
        if _STEPS>=1428:return replace(result,terminated=True,termination_reason='time_limit',info=info)
        return replace(result,info=info)
    cls.reset,cls._task_step=reset,task_step;cls._b5_monitor_patched=True


def current_status():
    return evaluate(None) if _OBSERVER is None else _OBSERVER.result()

def current_score():
    return float(current_status()['score'])


class Finder(importlib.abc.MetaPathFinder):
    targets={'yam_arena.actions':patch_actions,'yam_arena.scene':patch_scene,'yam_arena.inspect_embodiment':patch_embodiment}
    def find_spec(self,fullname,path,target=None):
        if fullname not in self.targets:return None
        sys.meta_path.remove(self)
        try:spec=importlib.util.find_spec(fullname)
        finally:sys.meta_path.insert(0,self)
        if spec is None or spec.loader is None:return None
        execute,patch=spec.loader.exec_module,self.targets[fullname]
        def exec_module(module):execute(module);patch(module)
        spec.loader.exec_module=exec_module;return spec

def install():
    for name,patch in Finder.targets.items():
        if name in sys.modules:patch(sys.modules[name])
    if not any(isinstance(f,Finder) for f in sys.meta_path):sys.meta_path.insert(0,Finder())
