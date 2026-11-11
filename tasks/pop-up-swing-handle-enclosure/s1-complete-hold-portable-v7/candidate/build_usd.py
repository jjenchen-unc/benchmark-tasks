"""Offline pxr primitive asset authoring. No native runtime dependency."""
from pathlib import Path
import json,math
import numpy as np
from pxr import Gf,Sdf,Usd,UsdGeom,UsdPhysics,UsdShade
HERE=Path(__file__).resolve().parent
F=json.loads((HERE/'fixture.json').read_text())
def quat(q):return Gf.Quatd(float(q[0]),Gf.Vec3d(*map(float,q[1:])))
def align_x(a):
 a=np.asarray(a);q=np.r_[1+a[0],np.cross([1,0,0],a)]
 return q/np.linalg.norm(q) if np.linalg.norm(q)>1e-9 else np.array([0,0,0,1.])
def xf(stage,path,pos,q):
 x=UsdGeom.Xform.Define(stage,path);x.AddTranslateOp().Set(Gf.Vec3d(*map(float,pos)));x.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(quat(q));return x.GetPrim()
def attr(p,n,v,t):p.CreateAttribute(n,t,custom=False).Set(v)
def author(f,target):
 stage=Usd.Stage.CreateNew(str(HERE/'assets'/f'{target}.usda'));UsdGeom.SetStageUpAxis(stage,'Z');UsdGeom.SetStageMetersPerUnit(stage,1.)
 root='/SwingBoxes';rp=xf(stage,root,[0,0,0],[1,0,0,0]);stage.SetDefaultPrim(rp);UsdPhysics.ArticulationRootAPI.Apply(rp);rp.AddAppliedSchema('PhysxArticulationAPI')
 attr(rp,'physxArticulation:enabledSelfCollisions',True,Sdf.ValueTypeNames.Bool)
 attr(rp,'physxArticulation:solverPositionIterationCount',64,Sdf.ValueTypeNames.Int)
 attr(rp,'physxArticulation:solverVelocityIterationCount',8,Sdf.ValueTypeNames.Int)
 support=xf(stage,root+'/links/support',[0,0,0],[1,0,0,0]);UsdPhysics.RigidBodyAPI.Apply(support);UsdPhysics.MassAPI.Apply(support).CreateMassAttr(.001)
 fixed=UsdPhysics.FixedJoint.Define(stage,root+'/joints/world');fixed.CreateBody1Rel().SetTargets([support.GetPath()]);fixed.CreateLocalPos0Attr(Gf.Vec3f(*F['translation']))
 world={};paths={}
 for b in f['bodies']:
  p=np.array(b['position']);parent=b['parent'];world[b['name']]=p+(world[parent] if parent in world else 0)
  bp=root+'/links/'+b['name'];paths[b['name']]=Sdf.Path(bp);prim=xf(stage,bp,world[b['name']],b['quaternion_wxyz']);UsdPhysics.RigidBodyAPI.Apply(prim)
  mass=UsdPhysics.MassAPI.Apply(prim);mass.CreateMassAttr(max(b['mass'],.001));mass.CreateCenterOfMassAttr(Gf.Vec3f(*b['com']));mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*np.maximum(b['inertia'],1e-9)));q=b['inertia_quaternion_wxyz'];mass.CreatePrincipalAxesAttr(Gf.Quatf(q[0],Gf.Vec3f(*q[1:])))
  for g in b['geoms']:
   gp=bp+'/'+g['name'];s=g['size'];k=g['kind']
   if k=='box':shape=UsdGeom.Cube.Define(stage,gp);shape.CreateSizeAttr(2.);shape.AddScaleOp().Set(Gf.Vec3f(*s))
   elif k=='sphere':shape=UsdGeom.Sphere.Define(stage,gp);shape.CreateRadiusAttr(s[0])
   else:
    shape=(UsdGeom.Cylinder if k=='cylinder' else UsdGeom.Capsule).Define(stage,gp);shape.CreateRadiusAttr(s[0]);shape.CreateHeightAttr(2*s[1]);shape.CreateAxisAttr('Z')
   shape.AddTranslateOp().Set(Gf.Vec3d(*g['position']));shape.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(quat(g['quaternion_wxyz']));ops=shape.GetOrderedXformOps();shape.SetXformOpOrder([o for typ in [UsdGeom.XformOp.TypeTranslate,UsdGeom.XformOp.TypeOrient,UsdGeom.XformOp.TypeScale] for o in ops if o.GetOpType()==typ]);shape.CreateDisplayColorAttr([Gf.Vec3f(*g['color'][:3])])
   # Explicit shader: primitive displayColor alone is unreliable in RTX.
   mat=UsdShade.Material.Define(stage,gp+'_visual');shader=UsdShade.Shader.Define(stage,gp+'_visual/shader');shader.CreateIdAttr('UsdPreviewSurface');shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*g['color'][:3]));mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface');UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(mat)
   if g['collision']:
    prim=shape.GetPrim();UsdPhysics.CollisionAPI.Apply(prim);prim.AddAppliedSchema('PhysxCollisionAPI');attr(prim,'physxCollision:contactOffset',.0002,Sdf.ValueTypeNames.Float);attr(prim,'physxCollision:restOffset',0.,Sdf.ValueTypeNames.Float)
    mat=UsdShade.Material.Define(stage,gp+'_physical');pm=UsdPhysics.MaterialAPI.Apply(mat.GetPrim());pm.CreateStaticFrictionAttr(g['friction'][0]);pm.CreateDynamicFrictionAttr(g['friction'][0]);pm.CreateRestitutionAttr(0.);UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat,UsdShade.Tokens.weakerThanDescendants,'physics')
  if not b['joints']:
   joint=UsdPhysics.FixedJoint.Define(stage,root+'/joints/fixed_'+b['name']);joint.CreateBody0Rel().SetTargets([support.GetPath()]);joint.CreateBody1Rel().SetTargets([paths[b['name']]]);joint.CreateLocalPos0Attr(Gf.Vec3f(*world[b['name']]))
  for j in b['joints']:
   joint=(UsdPhysics.RevoluteJoint if j['kind']=='hinge' else UsdPhysics.PrismaticJoint).Define(stage,root+'/joints/'+j['name']);joint.CreateBody0Rel().SetTargets([paths[parent] if parent in paths else support.GetPath()]);joint.CreateBody1Rel().SetTargets([paths[b['name']]]);joint.CreateAxisAttr('X');joint.CreateLocalPos0Attr(Gf.Vec3f(*b['position']));q=align_x(j['axis']);rotation=Gf.Quatf(q[0],Gf.Vec3f(*q[1:]));joint.CreateLocalRot0Attr(rotation);joint.CreateLocalRot1Attr(rotation);scale=180/math.pi if j['kind']=='hinge' else 1.;joint.CreateLowerLimitAttr(j['range'][0]*scale);joint.CreateUpperLimitAttr(j['range'][1]*scale);joint.CreateCollisionEnabledAttr(False)
   prim=joint.GetPrim();prim.AddAppliedSchema('PhysxJointAPI');attr(prim,'physxJoint:armature',j['armature'],Sdf.ValueTypeNames.Float)
   instance='angular' if j['kind']=='hinge' else 'linear';prim.AddAppliedSchema('PhysxJointAxisAPI:'+instance)
   for n in ['staticFrictionEffort','dynamicFrictionEffort']:attr(prim,'physxJointAxis:'+instance+':'+n,j['friction'],Sdf.ValueTypeNames.Float)
   # Passive springs are applied by environment in SI,
   # once per physics substep. No motors on task objects.
 for name,path in paths.items():
  UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(path)).CreateFilteredPairsRel().SetTargets([p for n,p in paths.items() if n!=name])
 stage.GetRootLayer().Save();return sum(1 for _ in stage.Traverse())
if __name__=='__main__':
 (HERE/'assets').mkdir(exist_ok=True)
 print(json.dumps({t:author(f,t) for t,f in F['fixtures'].items()}))
