"""Task-local Arena integration: free bodies, per-physics-step magnetic wrenches."""
from dataclasses import dataclass
import importlib.abc,importlib.util,sys,math
import numpy as np
from scipy.spatial.transform import Rotation
from yam_arena.task_file import Box,ObjectState
from magnet import pencil_vertices,wrench
from scoring import Scorer,geometry,support_flags

@dataclass(frozen=True)
class MagneticBody(Box):
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
    if spec.name=='pencil':
        vertices=pencil_vertices();hull=ConvexHull(vertices)
        shape=UsdGeom.Mesh.Define(stage,path+'/shape')
        shape.CreatePointsAttr([Gf.Vec3f(*(v*1000)) for v in vertices])
        shape.CreateFaceVertexCountsAttr([3]*len(hull.simplices))
        shape.CreateFaceVertexIndicesAttr(hull.simplices.flatten().tolist())
        shape.AddScaleOp().Set(Gf.Vec3f(.001,.001,.001));shape.CreateSubdivisionSchemeAttr('none')
        UsdPhysics.MeshCollisionAPI.Apply(shape.GetPrim()).CreateApproximationAttr('convexHull')
    else:
        shape=UsdGeom.Cube.Define(stage,path+'/shape');shape.CreateSizeAttr(1.)
        shape.AddScaleOp().Set(Gf.Vec3d(*spec.size))
    UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
    cp=PhysxSchema.PhysxCollisionAPI.Apply(shape.GetPrim());cp.CreateContactOffsetAttr(.00005);cp.CreateRestOffsetAttr(0.)
    mat=sim.RigidBodyMaterialCfg(static_friction=.8,dynamic_friction=.8,friction_combine_mode='min')
    mat.func(path+'/physics',mat);bind_physics_material(str(shape.GetPath()),path+'/physics',stage=stage)
    vis=sim.PreviewSurfaceCfg(diffuse_color=spec.color,roughness=.4)
    vis.func(path+'/visual',vis);bind_visual_material(str(shape.GetPath()),path+'/visual',stage=stage)
    if spec.name=='ipad':
        screen=UsdGeom.Cube.Define(stage,path+'/screen');screen.CreateSizeAttr(1.)
        screen.AddScaleOp().Set(Gf.Vec3d(.180,.120,.0001));screen.AddTranslateOp().Set(Gf.Vec3d(0,0,32.))
        # Translation is after scale: 32 * 0.1 mm = 3.2 mm. Visual only.
        black=sim.PreviewSurfaceCfg(diffuse_color=(.025,.05,.08),roughness=.22)
        black.func(path+'/glass',black);bind_visual_material(str(screen.GetPath()),path+'/glass',stage=stage)
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
        base.sim.set_setting('/physics/disableContactProcessing',False)
        stage=base.sim.stage;sid=UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        self.sv=tensors.create_simulation_view('numpy',stage_id=sid);self.sv.set_subspace_roots('/')
        self.paths={n:str(base.scene[n].cfg.prim_path).replace('env_.*','env_0').replace('env_[^/]+','env_0') for n in ('ipad','pencil')}
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
        self.pen_other=False;self.pen_illegal=False;self.tablet_env=False;self.samples=0
        print('MAGNET_RUNTIME',self.paths,'filters',self.filters,flush=True)
    def pose(self,n):
        t=self.views[n].get_transforms()[0].copy()
        return t[:3],t[3:],Rotation.from_quat(t[3:]).as_matrix()
    def before(self):
        pp,pq,rp=self.pose('pencil');ip,iq,ri=self.pose('ipad')
        fp,tp,fi,ti,_=wrench(pp,rp,ip,ri,self.scale)
        for n,f,t,pos in [('pencil',fp,tp,pp),('ipad',fi,ti,ip)]:
            self.views[n].apply_forces_and_torques_at_position(np.array([f],np.float32),np.array([t],np.float32),np.array([pos],np.float32),np.array([0],np.int32),True)
    def after(self):
        self.samples+=1
        c={n:np.linalg.norm(v.get_contact_force_matrix(dt=self.base.physics_dt).reshape(-1,3),axis=1) for n,v in self.contacts.items()}
        self.current_contacts={n:{p:float(x) for p,x in zip(self.filters,values) if x>.01} for n,values in c.items()}
        flags=support_flags(self.current_contacts,self.paths['ipad'],self.paths['pencil'],self.hand_paths,self.desk_paths)
        self.pen_other |= flags['pen_external_contact']
        self.pen_illegal |= flags['pen_illegal_support']
        self.tablet_env |= flags['tablet_environment_contact']
    def measure(self):
        ip,iq,ri=self.pose('ipad');pp,pq,rp=self.pose('pencil');m=geometry(ip,iq,pp,pq)
        contacts=getattr(self,'current_contacts',{'ipad':{},'pencil':{}})
        flags=support_flags(contacts,self.paths['ipad'],self.paths['pencil'],self.hand_paths,self.desk_paths)
        m.update(pen_tablet_contact=flags['pen_tablet_contact'],
                 pen_external_contact=bool(self.pen_other),pen_illegal_support=bool(self.pen_illegal),tablet_environment_contact=bool(self.tablet_env),
                 tablet_robot_contact=flags['tablet_robot_contact'],contacts=contacts,
                 ipad_pos=ip.tolist(),ipad_quat=iq.tolist(),pencil_pos=pp.tolist(),pencil_quat=pq.tolist(),
                 physics_samples=self.samples)
        self.scorer.update(m);m.update(score=self.scorer.score,full_s=self.scorer.full_s,partial_s=self.scorer.partial_s)
        self.latest=m;self.pen_other=False;self.pen_illegal=False;self.tablet_env=False;self.samples=0
        return m

def patch_scene(module):
    original_object=module.task_object;original_build=module.build_task_environment
    def task_object(spec):return arena_object(spec) if isinstance(spec,MagneticBody) else original_object(spec)
    def build(*a,**k):
        task=k.get('task')
        magnetic=task is not None and any(isinstance(s,MagneticBody) for s in task.objects)
        original_room=module.task_room
        if magnetic:
            from mount import camera_mount
            def room(scene,top):
                assert scene=='station',scene
                return [*module._station_room(),*(module._stand_part(n,*part) for n,part in camera_mount(top).items())]
            module.task_room=room
        try:result=original_build(*a,**k)
        finally:module.task_room=original_room
        if task is not None and any(isinstance(s,MagneticBody) for s in task.objects):
            prev=result.env_cfg_callback
            def configure(cfg):
                cfg=prev(cfg) if prev else cfg
                cfg.sim.dt=.0005;cfg.decimation=100;cfg.sim.render_interval=100
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
                overview.prim_path='{ENV_REGEX_NS}/PencilOverview'
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
    def active(self):return self.task is not None and any(isinstance(s,MagneticBody) for s in self.task.objects)
    def reset(self,*a,**k):
        global ACTIVE
        if not active(self):return old_reset(self,*a,**k)
        if self.device!='cpu' or not math.isclose(self.info.control_hz,20.):
            raise ValueError('Pencil task requires device=cpu, control_hz=20')
        if ACTIVE is not None:ACTIVE.active=False
        obs=old_reset(self,*a,**k)
        base=self._base;ACTIVE=Runtime(base)
        if not getattr(base.sim,'_pencil_hook',False):
            previous=base.sim.step
            def step(*args,**kwargs):
                if ACTIVE is not None and ACTIVE.active:ACTIVE.before()
                result=previous(*args,**kwargs)
                if ACTIVE is not None and ACTIVE.active:ACTIVE.after()
                return result
            base.sim.step=step;base.sim._pencil_hook=True
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
