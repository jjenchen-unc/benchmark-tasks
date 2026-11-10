"""Static XML-to-USD authoring. No native simulation or robot assets are copied."""
import json,math,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent

def author(output=None):
    from types import SimpleNamespace
    from pxr import Gf,Sdf,Usd,UsdGeom,UsdPhysics,UsdShade
    f=json.loads((HERE/'fixture.json').read_text());origin=np.array(f['native_origin']);spec=f['spec']
    compiled=json.loads((HERE/'compiled-fixture.json').read_text())
    m=SimpleNamespace(**{k:(np.array(v) if isinstance(v,list) else v) for k,v in compiled.items() if k not in ('names','provenance')})
    for kind in ('body','geom','joint'):setattr(m,kind,lambda i,_kind=kind:SimpleNamespace(name=compiled['names'][_kind][i]))
    stage=Usd.Stage.CreateNew(str(output or HERE/'latch.usda'));UsdGeom.SetStageUpAxis(stage,'Z');UsdGeom.SetStageMetersPerUnit(stage,1.)
    def schema(p,n):p.AddAppliedSchema(n)
    def attr(p,n,v,t=Sdf.ValueTypeNames.Float):p.CreateAttribute(n,t,custom=False).Set(v)
    def xf(path,p,q):
        x=UsdGeom.Xform.Define(stage,path);x.AddTranslateOp().Set(Gf.Vec3d(*map(float,p)));x.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(q[0]),Gf.Vec3d(*map(float,q[1:]))));return x.GetPrim()
    R='/CompressionLatch';stage.SetDefaultPrim(xf(R,[0,0,0],[1,0,0,0]));UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(R));schema(stage.GetPrimAtPath(R),'PhysxArticulationAPI');attr(stage.GetPrimAtPath(R),'physxArticulation:enabledSelfCollisions',True,Sdf.ValueTypeNames.Bool)
    attr(stage.GetPrimAtPath(R),'physxArticulation:solverPositionIterationCount',64,Sdf.ValueTypeNames.Int)
    names={i:m.body(i).name for i in range(1,m.nbody)};positions={};bodypaths={}
    # All initial body orientations are identity in the source fixture.
    for i,name in names.items():
        parent=int(m.body_parentid[i]);positions[i]=(positions[parent] if parent in positions else np.zeros(3))+m.body_pos[i]
        bp=R+'/links/'+name;bodypaths[i]=bp;shift=positions[i]-origin if i==1 else np.zeros(3);p=xf(bp,np.zeros(3) if i==1 else positions[i]-origin,m.body_quat[i]);UsdPhysics.RigidBodyAPI.Apply(p)
        mass=UsdPhysics.MassAPI.Apply(p);mass.CreateMassAttr(float(m.body_mass[i]));mass.CreateCenterOfMassAttr(Gf.Vec3f(*map(float,m.body_ipos[i]+shift)));mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float,m.body_inertia[i])));q=m.body_iquat[i];mass.CreatePrincipalAxesAttr(Gf.Quatf(float(q[0]),Gf.Vec3f(*map(float,q[1:]))))
        schema(p,'PhysxRigidBodyAPI');attr(p,'physxRigidBody:maxDepenetrationVelocity',.5)
        for gi in range(m.ngeom):
            if m.geom_bodyid[gi]!=i:continue
            nameg=m.geom(gi).name;s=m.geom_size[gi];gp=bp+'/'+nameg
            if m.geom_type[gi]==6:
                shape=UsdGeom.Cube.Define(stage,gp);shape.CreateSizeAttr(1.);shape.AddScaleOp().Set(Gf.Vec3f(*map(float,2*s)))
            elif m.geom_type[gi]==5:
                shape=UsdGeom.Cylinder.Define(stage,gp);shape.CreateAxisAttr('Z');shape.CreateRadiusAttr(float(s[0]));shape.CreateHeightAttr(float(2*s[1]))
            else:raise ValueError('unsupported source shape')
            shape.AddTranslateOp().Set(Gf.Vec3d(*map(float,m.geom_pos[gi]+shift)));q=m.geom_quat[gi];shape.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(q[0]),Gf.Vec3d(*map(float,q[1:]))))
            ops=shape.GetOrderedXformOps();shape.SetXformOpOrder(sorted(ops,key=lambda o:{UsdGeom.XformOp.TypeTranslate:0,UsdGeom.XformOp.TypeOrient:1,UsdGeom.XformOp.TypeScale:2}[o.GetOpType()]))
            shape.CreateDisplayColorAttr([Gf.Vec3f(*map(float,m.geom_rgba[gi,:3]))])
            if m.geom_contype[gi] or m.geom_conaffinity[gi]:
                p=shape.GetPrim();UsdPhysics.CollisionAPI.Apply(p);schema(p,'PhysxCollisionAPI');attr(p,'physxCollision:contactOffset',.00005);attr(p,'physxCollision:restOffset',0.)
                mat=UsdShade.Material.Define(stage,R+'/materials/'+nameg);pm=UsdPhysics.MaterialAPI.Apply(mat.GetPrim());pm.CreateStaticFrictionAttr(float(m.geom_friction[gi,0]));pm.CreateDynamicFrictionAttr(float(m.geom_friction[gi,0]));pm.CreateRestitutionAttr(0.);UsdShade.MaterialBindingAPI.Apply(p).Bind(mat,UsdShade.Tokens.weakerThanDescendants,'physics')
    fixed=UsdPhysics.FixedJoint.Define(stage,R+'/joints/world_fixed');fixed.CreateBody1Rel().SetTargets([bodypaths[1]])
    for ji in range(m.njnt):
        bi=int(m.jnt_bodyid[ji]);pa=int(m.body_parentid[bi]);name=m.joint(ji).name;angular=m.jnt_type[ji]==3
        j=(UsdPhysics.RevoluteJoint if angular else UsdPhysics.PrismaticJoint).Define(stage,R+'/joints/'+name);j.CreateAxisAttr('Z');j.CreateBody0Rel().SetTargets([bodypaths[pa] if pa else bodypaths[1]]);j.CreateBody1Rel().SetTargets([bodypaths[bi]])
        anchor=positions[bi]+m.jnt_pos[ji];j.CreateLocalPos0Attr(Gf.Vec3f(*map(float,anchor-(origin if pa in (0,1) else positions[pa]))));j.CreateLocalPos1Attr(Gf.Vec3f(*map(float,m.jnt_pos[ji])))
        # Joint Z is reversed for clockwise hinges and downward compression.
        q=Gf.Quatf(0,Gf.Vec3f(1,0,0)) if m.jnt_axis[ji,2]<0 else Gf.Quatf(1,Gf.Vec3f(0,0,0));j.CreateLocalRot0Attr(q);j.CreateLocalRot1Attr(q)
        scale=180/math.pi if angular else 1.;j.CreateLowerLimitAttr(float(m.jnt_range[ji,0]*scale));j.CreateUpperLimitAttr(float(m.jnt_range[ji,1]*scale));j.CreateCollisionEnabledAttr(False)
        p=j.GetPrim();schema(p,'PhysxJointAPI');attr(p,'physxJoint:armature',float(m.dof_armature[ji]))
        inst='angular' if angular else 'linear';p.AddAppliedSchema('PhysxJointAxisAPI:'+inst)
        for n in ('staticFrictionEffort','dynamicFrictionEffort'):attr(p,'physxJointAxis:'+inst+':'+n,float(m.dof_frictionloss[ji]))
        drive=UsdPhysics.DriveAPI.Apply(p,inst);drive.CreateTypeAttr('force');drive.CreateStiffnessAttr(float(m.jnt_stiffness[ji]/scale));drive.CreateDampingAttr(float(m.dof_damping[ji]/scale));drive.CreateTargetPositionAttr(0.);drive.CreateMaxForceAttr(float('inf'))
    # Source masks: pawl contacts flange only, other task/robot contacts remain.
    UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(bodypaths[2])).CreateFilteredPairsRel().AddTarget(Sdf.Path(bodypaths[1]))
    pawl=next(i for i,n in names.items() if n=='pawl')
    for bi,bp in bodypaths.items():
        if bi not in (1,pawl):UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(bodypaths[pawl])).CreateFilteredPairsRel().AddTarget(Sdf.Path(bp))
    # Pawl must not contact solid shell under the flange (native masks 4 versus 1).
    rel=UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(bodypaths[pawl]+'/pawl_tip')).CreateFilteredPairsRel()
    for gi in range(m.ngeom):
        if m.geom_bodyid[gi]==1 and m.geom(gi).name!='flange':rel.AddTarget(Sdf.Path(bodypaths[1]+'/'+m.geom(gi).name))
    stage.GetRootLayer().Save();return {'bodies':len(names),'joints':m.njnt,'geoms':m.ngeom,'source_spec':spec}
if __name__=='__main__':print(json.dumps(author(),indent=2))
