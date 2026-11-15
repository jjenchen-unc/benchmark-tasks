"""Task-local Arena integration: free bodies, per-physics-step magnetic wrenches."""
from dataclasses import dataclass
import importlib.abc,importlib.util,sys,math,os
import numpy as np
from scipy.spatial.transform import Rotation
from yam_arena.task_file import Box,ObjectState
from mechanism import wrenches, TABLET_SIZE, PANEL_SIZE, ROOT, SECOND_OFFSET
from scoring import Scorer,geometry,CONFIG
from native_overlap_monitor import NativeOverlapMonitor

@dataclass(frozen=True)
class StandBody(Box):
    quat:tuple=(0.,0.,0.,1.)

SPECS={}
ACTIVE=None

def spawn(path,cfg,translation=None,orientation=None,**kwargs):
    import isaaclab.sim as sim
    from isaaclab.sim.utils.stage import get_current_stage
    from isaaclab.sim.utils import bind_physics_material,bind_visual_material
    from pxr import Gf,UsdGeom,UsdPhysics,PhysxSchema
    from scipy.spatial import ConvexHull
    stage=get_current_stage();spec=SPECS[cfg.body_name]
    root=UsdGeom.Xform.Define(stage,path)
    root.AddTranslateOp().Set(Gf.Vec3d(*translation))
    x,y,z,w=orientation;root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w,x,y,z))
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    PhysxSchema.PhysxContactReportAPI.Apply(root.GetPrim()).CreateThresholdAttr(0.)
    UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(spec.mass)
    rb=PhysxSchema.PhysxRigidBodyAPI.Apply(root.GetPrim())
    rb.CreateSolverPositionIterationCountAttr(32);rb.CreateSolverVelocityIterationCountAttr(8)
    rb.CreateLinearDampingAttr(0.);rb.CreateAngularDampingAttr(0.)
    rb.CreateMaxDepenetrationVelocityAttr(1.)
    shape=UsdGeom.Cube.Define(stage,path+'/shape');shape.CreateSizeAttr(1.)
    shape.AddScaleOp().Set(Gf.Vec3d(*spec.size))
    UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
    cp=PhysxSchema.PhysxCollisionAPI.Apply(shape.GetPrim());cp.CreateContactOffsetAttr(.00005);cp.CreateRestOffsetAttr(0.)
    mat=sim.RigidBodyMaterialCfg(static_friction=.8,dynamic_friction=.8,friction_combine_mode='min')
    mat.func(path+'/physics',mat);bind_physics_material(str(shape.GetPath()),path+'/physics',stage=stage)
    vis=sim.PreviewSurfaceCfg(diffuse_color=spec.color,roughness=.4)
    vis.func(path+'/visual',vis);bind_visual_material(str(shape.GetPath()),path+'/visual',stage=stage)
    if spec.name=='tablet':
        screen=UsdGeom.Cube.Define(stage,path+'/screen');screen.CreateSizeAttr(1.)
        screen.AddScaleOp().Set(Gf.Vec3d(.168,.228,.0001));screen.AddTranslateOp().Set(Gf.Vec3d(0,0,36.))
        # Translation is after scale: 32 * 0.1 mm = 3.2 mm. Visual only.
        black=sim.PreviewSurfaceCfg(diffuse_color=(.025,.05,.08),roughness=.22)
        black.func(path+'/glass',black);bind_visual_material(str(screen.GetPath()),path+'/glass',stage=stage)
    if spec.name=='panel2':
        parent=path.rsplit('/',1)[0]
        for name,b0,b1,p0,p1,limits in [
            ('fold1','tablet','panel1',tuple(ROOT),(0,.075,0),(0.,143.239)),
            ('fold2','panel1','panel2',tuple(SECOND_OFFSET),(0,.075,0),(0.,159.946))]:
            joint=UsdPhysics.RevoluteJoint.Define(stage,parent+'/'+name)
            joint.CreateBody0Rel().SetTargets([parent+'/'+b0]);joint.CreateBody1Rel().SetTargets([parent+'/'+b1])
            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*p0));joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*p1))
            if name=='fold2':joint.CreateLocalRot0Attr().Set(Gf.Quatf(0.,Gf.Vec3f(-1.,0.,0.)))
            joint.CreateAxisAttr().Set('X');joint.CreateLowerLimitAttr().Set(limits[0]);joint.CreateUpperLimitAttr().Set(limits[1])
            joint.CreateCollisionEnabledAttr().Set(True)
            drive=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'angular')
            drive.CreateTypeAttr().Set('force');drive.CreateStiffnessAttr().Set(0.)
            drive.CreateDampingAttr().Set(.002*math.pi/180.)
            drive.CreateTargetVelocityAttr().Set(0.)
            PhysxSchema.PhysxJointAPI.Apply(joint.GetPrim()).CreateJointFrictionAttr().Set(.006)
    return root.GetPrim()

def arena_object(spec):
    from isaaclab.utils import configclass
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    @configclass
    class SpawnCfg(SpawnerCfg):
        func:object=clone(spawn)
        body_name:str=''
    SPECS[spec.name]=spec
    return Object(name=spec.name,object_type=ObjectType.RIGID,spawner_cfg=SpawnCfg(body_name=spec.name),
                  initial_pose=Pose(position_xyz=spec.pos,rotation_xyzw=spec.quat))

def arr(v):
    v=getattr(v,'torch',v)
    return v.detach().cpu().numpy() if hasattr(v,'detach') else np.asarray(v)

class Runtime:
    def __init__(self,base):
        import omni.physics.tensors as tensors
        from pxr import UsdUtils,UsdPhysics
        self.base=base;self.scale=1.;self.scorer=Scorer();self.latest={};self.active=True
        self.overlap_monitor=NativeOverlapMonitor(dict(tablet=TABLET_SIZE,panel1=PANEL_SIZE,panel2=PANEL_SIZE))
        base.sim.set_setting('/physics/disableContactProcessing',False)
        stage=base.sim.stage;sid=UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        self.sv=tensors.create_simulation_view('numpy',stage_id=sid);self.sv.set_subspace_roots('/')
        self.paths={n:str(base.scene[n].cfg.prim_path).replace('env_.*','env_0').replace('env_[^/]+','env_0') for n in ('tablet','panel1','panel2')}
        self.views={n:self.sv.create_rigid_body_view(path) for n,path in self.paths.items()}
        assert all(v.count==1 for v in self.views.values()),self.paths
        rigid=[str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
        static=[str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)
                and not any(str(p.GetPath()).startswith(r+'/') or str(p.GetPath())==r for r in rigid)]
        self.filters=sorted(set(rigid+static))
        self.robot_paths=tuple(str(base.scene[n].cfg.prim_path).replace('env_.*','env_0').replace('env_[^/]+','env_0') for n in ('left_robot','right_robot'))
        self.desk_paths={p for p in static if '/table/' in p}
        self.hand_paths={p for p in rigid if p.startswith(self.robot_paths) and '/gripper' in p}
        self.contacts={n:self.sv.create_rigid_contact_view(path,filter_patterns=self.filters,max_contact_data_count=4096)
                       for n,path in self.paths.items()}
        assert all(v.filter_count==len(self.filters) for v in self.contacts.values())
        from pxr import Usd,UsdGeom,Gf
        bbox=UsdGeom.BBoxCache(Usd.TimeCode.Default(),[UsdGeom.Tokens.default_,UsdGeom.Tokens.render,UsdGeom.Tokens.proxy,UsdGeom.Tokens.guide],useExtentsHint=False,ignoreVisibility=True)
        self.hand_bounds=[]
        for side in ('left','right'):
            robot=base.scene[side+'_robot']
            for index,name in enumerate(robot.body_names):
                if name not in ('gripper','tip_left','tip_right'):continue
                paths=[p for p in rigid if p.startswith(self.robot_paths[0 if side=='left' else 1]) and p.endswith('/'+name)]
                assert len(paths)==1,(side,name,paths)
                body=stage.GetPrimAtPath(paths[0]);candidates=[]
                worldscale=np.array(Gf.Transform(UsdGeom.Xformable(body).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).GetScale())
                assert np.allclose(worldscale,1.,atol=1e-6),(side,name,worldscale)
                for collider in Usd.PrimRange(body):
                    if not collider.HasAPI(UsdPhysics.CollisionAPI):continue
                    parent=collider
                    while parent and not parent.HasAPI(UsdPhysics.RigidBodyAPI):parent=parent.GetParent()
                    if parent!=body:continue
                    bound=bbox.ComputeRelativeBound(collider,body).ComputeAlignedRange()
                    lo=np.array(bound.GetMin());hi=np.array(bound.GetMax())
                    if np.isfinite(lo).all() and np.isfinite(hi).all():candidates.append((lo,hi))
                assert candidates,(side,name,'missing collision bounds')
                lo=np.min([a for a,b in candidates],axis=0);hi=np.max([b for a,b in candidates],axis=0)
                self.hand_bounds.append(dict(side=side,index=index,name=name,body_path=paths[0],world_scale=worldscale.tolist(),local_center=((lo+hi)/2).tolist(),radius_m=float(np.linalg.norm((hi-lo)/2))))
        self.robot_contact=False;self.illegal_support=False;self.samples=0
        self.current_contacts={n:{} for n in self.paths}
        print('STAND_RUNTIME',self.paths,'filters',self.filters,flush=True)
    def pose(self,n):
        t=self.views[n].get_transforms()[0].copy()
        return t[:3],t[3:],Rotation.from_quat(t[3:]).as_matrix()
    def before(self):
        snapshots={n:self.pose(n) for n in self.paths}
        poses={n:(p,r) for n,(p,q,r) in snapshots.items()}
        self.overlap_monitor.observe(poses,self.base.physics_dt)
        forces,torques=wrenches(poses,self.scale)
        for n in self.paths:
            self.views[n].apply_forces_and_torques_at_position(np.array([forces[n]],np.float32),np.array([torques[n]],np.float32),np.array([poses[n][0]],np.float32),np.array([0],np.int32),True)
    def after(self):
        self.samples+=1
        c={n:np.linalg.norm(v.get_contact_force_matrix(dt=self.base.physics_dt).reshape(-1,3),axis=1) for n,v in self.contacts.items()}
        self.current_contacts={n:{p:float(x) for p,x in zip(self.filters,values) if x>.01} for n,values in c.items()}
        task_paths=set(self.paths.values())
        for cs in self.current_contacts.values():
            self.robot_contact |= any(p.startswith(self.robot_paths) for p in cs)
            self.illegal_support |= bool(set(cs)-task_paths-self.desk_paths-{p for p in cs if p.startswith(self.robot_paths)})
    def measure(self):
        ip,iq,ri=self.pose('tablet');m=geometry(ip,iq)
        poses={n:self.pose(n) for n in self.paths}
        points=[];hand_spheres=[];hand_poses=[];clear=[]
        for bound in self.hand_bounds:
            pose=arr(self.base.scene[bound['side']+'_robot'].data.body_link_pose_w)[0,bound['index']]
            hand_poses.append(dict(name=bound['side']+'/'+bound['name'],pos=pose[:3].tolist(),quat=pose[3:].tolist()))
            r=Rotation.from_quat(pose[3:]).as_matrix();point=pose[:3]+r@np.array(bound['local_center'])
            points.append(pose[:3]);hand_spheres.append(dict(name=bound['side']+'/'+bound['name'],center=point.tolist(),radius_m=bound['radius_m']))
            for n,(pos,q,rot) in poses.items():
                half=(TABLET_SIZE if n=='tablet' else PANEL_SIZE)/2
                clear.append(float(np.linalg.norm(np.maximum(np.abs(rot.T@(point-pos))-half,0))-bound['radius_m']))
        on_table=True
        for n,(pos,q,r) in poses.items():
            size=TABLET_SIZE if n=='tablet' else PANEL_SIZE
            import itertools
            corners=np.array(list(itertools.product(*[[-h,h] for h in size/2])))@r.T+pos
            on_table &= bool(np.all(corners[:,0]>=-.19-.002) and np.all(corners[:,0]<=.91+.002) and np.all(np.abs(corners[:,1])<=.65+.002) and corners[:,2].min()>=.7475-.002)
        m.update(substep_overlap=self.overlap_monitor.interval_report(),overlap_sampling_phase="pre_physics_step",tablet_pos=ip.tolist(),tablet_quat=iq.tolist(),body_poses={n:dict(pos=p.tolist(),quat=q.tolist()) for n,(p,q,r) in poses.items()},body_velocities={n:self.views[n].get_velocities()[0].tolist() for n in poses},
            tablet_table_contact=bool(set(self.current_contacts['tablet'])&self.desk_paths),
            stand_table_contact=any(bool(set(self.current_contacts[n])&self.desk_paths) for n in ('panel1','panel2')),
            robot_contact=bool(self.robot_contact),illegal_support_contact=bool(self.illegal_support),
            hand_clearance_m=min(clear,default=0.),all_body_on_table=bool(on_table),contacts=self.current_contacts,physics_samples=self.samples,hand_body_poses=hand_poses,hand_link_origins=[point.tolist() for point in points],hand_bounding_spheres=hand_spheres,desk_paths=sorted(self.desk_paths))
        self.scorer.update(m);m.update(score=self.scorer.score,full_s=self.scorer.full_s,partial_s=self.scorer.partial_s)
        self.latest=m;self.robot_contact=False;self.illegal_support=False;self.samples=0
        return m

def patch_scene(module):
    original_object=module.task_object;original_build=module.build_task_environment
    def task_object(spec):return arena_object(spec) if isinstance(spec,StandBody) else original_object(spec)
    def build(*a,**k):
        task=k.get('task')
        magnetic=task is not None and any(isinstance(s,StandBody) for s in task.objects)
        original_room=module.task_room
        if magnetic:
            from mount import camera_mount
            def room(scene,top):
                assert scene=='station',scene
                return [*module._station_room(),*(module._stand_part(n,*part) for n,part in camera_mount(top).items())]
            module.task_room=room
        try:result=original_build(*a,**k)
        finally:module.task_room=original_room
        if task is not None and any(isinstance(s,StandBody) for s in task.objects):
            prev=result.env_cfg_callback
            def configure(cfg):
                cfg=prev(cfg) if prev else cfg
                cfg.sim.dt=float(os.environ.get('STAND_PHYSICS_DT','.001'));cfg.decimation=round(.05/cfg.sim.dt);cfg.sim.render_interval=cfg.decimation
                assert cfg.sim.dt in (.0005,.001),cfg.sim.dt
                # Higher servo stiffness, with original effort/velocity caps retained.
                for name,gain in [('left_robot',8.),('right_robot',4.)]:
                    for group in ('shoulder','wrist'):
                        actuator=getattr(cfg.scene,name).actuators[group]
                        actuator.stiffness*=gain
                        actuator.damping*=math.sqrt(gain)
                # Review-only fixed overview; Dream retains its three SDK camera inputs.
                import copy
                from isaaclab.sensors import CameraCfg
                import isaaclab.sim as sim
                cfg.scene.overview_cam=copy.deepcopy(cfg.scene.top_cam)
                overview=cfg.scene.overview_cam
                overview.prim_path='{ENV_REGEX_NS}/StandOverview'
                forward=np.array([.72,.60,-.47]);forward/=np.linalg.norm(forward)
                left=np.cross([0,0,1.],forward);left/=np.linalg.norm(left)
                up=np.cross(forward,left)
                quat=Rotation.from_matrix(np.column_stack([forward,left,up])).as_quat()
                overview.offset=CameraCfg.OffsetCfg(pos=(-.8,-.6,1.35),rot=tuple(quat),convention='world')
                overview.spawn=sim.PinholeCameraCfg(focal_length=18.,clipping_range=(.02,20.))
                # RTX denoising verified on the Brev L40S driver; frame generation stays disabled.
                for name in ('top_cam','left_cam','right_cam','overview_cam'):
                    camera=getattr(cfg.scene,name)
                    camera.renderer_cfg=copy.deepcopy(camera.renderer_cfg)
                    settings=camera.renderer_cfg.global_settings
                    settings.antialiasing_mode='DLSS'
                    settings.enable_dlssg=False
                    settings.enable_dl_denoiser=True
                    settings.samples_per_pixel=16
                return cfg
            result.env_cfg_callback=configure
        return result
    module.task_object=task_object;module.build_task_environment=build

def patch_embodiment(module):
    cls=module.YamArenaEmbodiment;old_reset=cls.reset;old_step=cls._task_step
    def active(self):return self.task is not None and any(isinstance(s,StandBody) for s in self.task.objects)
    def reset(self,*a,**k):
        global ACTIVE
        if not active(self):return old_reset(self,*a,**k)
        if self.device!='cpu' or not math.isclose(self.info.control_hz,20.):
            raise ValueError('Stand task requires device=cpu, control_hz=20')
        if ACTIVE is not None:ACTIVE.active=False
        obs=old_reset(self,*a,**k)
        base=self._base;ACTIVE=Runtime(base)
        if not getattr(base.sim,'_stand_hook',False):
            previous=base.sim.step
            def step(*args,**kwargs):
                if ACTIVE is not None and ACTIVE.active:ACTIVE.before()
                result=previous(*args,**kwargs)
                if ACTIVE is not None and ACTIVE.active:ACTIVE.after()
                return result
            base.sim.step=step;base.sim._stand_hook=True
        return obs
    def task_step(self):
        if active(self):ACTIVE.measure()
        return old_step(self)
    cls.reset=reset;cls._task_step=task_step

class Finder(importlib.abc.MetaPathFinder):
    targets={'yam_arena.scene':patch_scene,'yam_arena.inspect_embodiment':patch_embodiment}
    def find_spec(self,fullname,path,target=None):
        if fullname not in self.targets:return None
        sys.meta_path.remove(self)
        try:spec=importlib.util.find_spec(fullname)
        finally:sys.meta_path.insert(0,self)
        loader=spec.loader;patch=self.targets[fullname]
        class Loader(importlib.abc.Loader):
            def create_module(self,spec):return loader.create_module(spec)
            def exec_module(self,module):loader.exec_module(module);patch(module)
        spec.loader=Loader();return spec

def install():
    finder=Finder()
    for name,patch in finder.targets.items():
        if name in sys.modules:patch(sys.modules[name])
    sys.meta_path.insert(0,finder)
install()
