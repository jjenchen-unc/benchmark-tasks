"""Offline pxr authoring from exact-source task-only compiled JSON. No physics."""
import json,math
from pathlib import Path
import numpy as np
from pxr import Gf,Sdf,Usd,UsdGeom,UsdPhysics,UsdShade
HERE=Path(__file__).resolve().parent
F=json.loads((HERE/'fixture.json').read_text())
def q(v):return Gf.Quatd(float(v[0]),Gf.Vec3d(*map(float,v[1:])))
def mat(v):return np.array(Gf.Matrix3d(q(v))).T
def mul(a,b):
 v=q(a)*q(b);return [v.GetReal(),*v.GetImaginary()]
def inv(a):return [a[0],*-np.array(a[1:])]
def align(axis):
 a=np.array(axis);a=a/np.linalg.norm(a)
 if a[0]<-1+1e-8:return [0,0,0,1]
 v=np.r_[1+a[0],np.cross([1,0,0],a)];return v/np.linalg.norm(v)
def xf(stage,path,pos,quat):
 p=UsdGeom.Xform.Define(stage,path);p.AddTranslateOp().Set(Gf.Vec3d(*map(float,pos)));p.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(q(quat));return p.GetPrim()
def attr(p,n,v,t):p.CreateAttribute(n,t,custom=False).Set(v)
def run(c):
 rootname='case_'+c;B=[b for b in F['bodies'] if b['name']==rootname or b['name']=='lid_'+c or b['name'].startswith(c)]
 rootbody=next(b for b in B if b['name']==rootname);origin=np.array(rootbody['pos']);stage=Usd.Stage.CreateNew(str(HERE/'assets'/f'case-{c}.usda'));UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,'Z');root=xf(stage,'/Case',[0,0,0],[1,0,0,0]);stage.SetDefaultPrim(root);UsdPhysics.ArticulationRootAPI.Apply(root);root.AddAppliedSchema('PhysxArticulationAPI');attr(root,'physxArticulation:enabledSelfCollisions',True,Sdf.ValueTypeNames.Bool);attr(root,'physxArticulation:solverPositionIterationCount',64,Sdf.ValueTypeNames.Int);attr(root,'physxArticulation:solverVelocityIterationCount',8,Sdf.ValueTypeNames.Int)
 bodies={b['name']:b for b in B};geoms=[];materials={}
 for b in B:
  bp='/Case/links/'+b['name'];p=xf(stage,bp,np.array(b['pos'])-origin,b['quat']);UsdPhysics.RigidBodyAPI.Apply(p);p.AddAppliedSchema('PhysxRigidBodyAPI');attr(p,'physxRigidBody:maxDepenetrationVelocity',.5,Sdf.ValueTypeNames.Float)
  mass=UsdPhysics.MassAPI.Apply(p);mass.CreateMassAttr(b['mass']);mass.CreateCenterOfMassAttr(Gf.Vec3f(*b['com']));mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*b['inertia']));a=b['inertia_quat'];mass.CreatePrincipalAxesAttr(Gf.Quatf(a[0],Gf.Vec3f(*a[1:])))
  for g in b['geoms']:
   path=bp+'/'+g['name'];size=g['size']
   if g['kind']=='box':s=UsdGeom.Cube.Define(stage,path);s.CreateSizeAttr(2.);s.AddScaleOp().Set(Gf.Vec3f(*size))
   else:s=UsdGeom.Cylinder.Define(stage,path);s.CreateRadiusAttr(size[0]);s.CreateHeightAttr(2*size[1]);s.CreateAxisAttr('Z')
   s.AddTranslateOp().Set(Gf.Vec3d(*g['pos']));s.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(q(g['quat']));ops=s.GetOrderedXformOps();s.SetXformOpOrder(sorted(ops,key=lambda op:{UsdGeom.XformOp.TypeTranslate:0,UsdGeom.XformOp.TypeOrient:1,UsdGeom.XformOp.TypeScale:2}[op.GetOpType()]))
   s.CreateDisplayColorAttr([Gf.Vec3f(*g['rgba'][:3])]);s.CreateDisplayOpacityAttr([g['rgba'][3]])
   if g['contype'] or g['conaffinity']:
    gp=s.GetPrim();UsdPhysics.CollisionAPI.Apply(gp);gp.AddAppliedSchema('PhysxCollisionAPI');attr(gp,'physxCollision:contactOffset',.0002,Sdf.ValueTypeNames.Float);attr(gp,'physxCollision:restOffset',0.,Sdf.ValueTypeNames.Float)
    mu=g['friction'][0]
    if mu not in materials:
     mt=UsdShade.Material.Define(stage,'/Case/materials/m'+str(len(materials)));pm=UsdPhysics.MaterialAPI.Apply(mt.GetPrim());pm.CreateStaticFrictionAttr(mu);pm.CreateDynamicFrictionAttr(mu);pm.CreateRestitutionAttr(0.);materials[mu]=mt
    UsdShade.MaterialBindingAPI.Apply(gp).Bind(materials[mu],UsdShade.Tokens.weakerThanDescendants,'physics');geoms.append((b,g,path))
  if not b['joints'] and b['name']!=rootname:
   parent=bodies[b['parent']];jt=UsdPhysics.FixedJoint.Define(stage,'/Case/joints/'+b['name']+'_weld');jt.CreateBody0Rel().SetTargets(['/Case/links/'+b['parent']]);jt.CreateBody1Rel().SetTargets([bp]);jt.CreateLocalPos0Attr(Gf.Vec3f(*(mat(parent['quat']).T@(np.array(b['pos'])-parent['pos']))));jt.CreateLocalRot0Attr(Gf.Quatf(1.,Gf.Vec3f(0,0,0)))
  for j in b['joints']:
   parent=bodies[b['parent']];jp='/Case/joints/'+j['name'];jt=(UsdPhysics.PrismaticJoint if j['kind']=='slide' else UsdPhysics.RevoluteJoint).Define(stage,jp);jt.CreateAxisAttr('X');jt.CreateBody0Rel().SetTargets(['/Case/links/'+b['parent']]);jt.CreateBody1Rel().SetTargets([bp]);R=mat(b['quat']);Rp=mat(parent['quat']);anchor=np.array(b['pos'])+R@j['anchor'];a=align(j['axis']);a0=mul(mul(inv(parent['quat']),b['quat']),a)
   jt.CreateLocalPos0Attr(Gf.Vec3f(*(Rp.T@(anchor-parent['pos']))));jt.CreateLocalPos1Attr(Gf.Vec3f(*j['anchor']));jt.CreateLocalRot0Attr(Gf.Quatf(a0[0],Gf.Vec3f(*a0[1:])));jt.CreateLocalRot1Attr(Gf.Quatf(float(a[0]),Gf.Vec3f(*map(float,a[1:]))));factor=1. if j['kind']=='slide' else 180/math.pi;lim=j['range'].copy()
   jt.CreateLowerLimitAttr(lim[0]*factor);jt.CreateUpperLimitAttr(lim[1]*factor);jt.CreateCollisionEnabledAttr(False);jt.GetPrim().AddAppliedSchema('PhysxJointAPI');attr(jt.GetPrim(),'physxJoint:jointFriction',0.,Sdf.ValueTypeNames.Float);attr(jt.GetPrim(),'physxJoint:armature',j['armature'],Sdf.ValueTypeNames.Float)
   drive=UsdPhysics.DriveAPI.Apply(jt.GetPrim(),'linear' if j['kind']=='slide' else 'angular');drive.CreateStiffnessAttr(0.);drive.CreateDampingAttr(0.);drive.CreateMaxForceAttr(1e9)
   # Zero-gain task drives expose only the unchanged passive SI spring efforts.
   # Robot drive limits remain the separate actual27/7Nm contract.
   # Springs/damping are applied passively in SI torque/force units each substep.
 for eq in F['connections']:
  if not eq['name'].startswith(c):continue
  jt=UsdPhysics.SphericalJoint.Define(stage,'/Case/connections/'+eq['name']);jt.CreateBody0Rel().SetTargets(['/Case/links/'+eq['body0']]);jt.CreateBody1Rel().SetTargets(['/Case/links/'+eq['body1']]);jt.CreateLocalPos0Attr(Gf.Vec3f(*eq['anchor0']));jt.CreateLocalPos1Attr(Gf.Vec3f(*eq['anchor1']));jt.CreateJointEnabledAttr(eq['enabled']);jt.CreateExcludeFromArticulationAttr(True);jt.CreateCollisionEnabledAttr(True)
 # Native mask pairs and explicit excludes retained on each shape.
 for bi,gi,pi in geoms:
  targets=[]
  for bj,gj,pj in geoms:
   if pi==pj or bi['name']==bj['name']:continue
   masks=(gi['contype']&gj['conaffinity']) or (gj['contype']&gi['conaffinity'])
   ex=any(set(v)=={bi['name'],bj['name']} for v in F['excludes'])
   if not masks or ex:targets.append(Sdf.Path(pj))
  if targets:UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(pi)).CreateFilteredPairsRel().SetTargets(targets)
 stage.GetRootLayer().Save()
 return {'case':c,'bodies':len(B),'geoms':sum(len(b['geoms']) for b in B),'mass_kg':sum(b['mass'] for b in B),'joint_count':sum(len(b['joints']) for b in B)}
def instrument(c):
 b=next(b for b in F['bodies'] if b['name']=='inst_'+c);stage=Usd.Stage.CreateNew(str(HERE/'assets'/f'instrument-{c}.usda'));UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,'Z');root=xf(stage,'/Instrument',[0,0,0],[1,0,0,0]);stage.SetDefaultPrim(root);UsdPhysics.RigidBodyAPI.Apply(root);mass=UsdPhysics.MassAPI.Apply(root);mass.CreateMassAttr(b['mass']);mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*b['inertia']));mass.CreateCenterOfMassAttr(Gf.Vec3f(*b['com']))
 material=UsdShade.Material.Define(stage,'/Instrument/material');pm=UsdPhysics.MaterialAPI.Apply(material.GetPrim());pm.CreateStaticFrictionAttr(1.);pm.CreateDynamicFrictionAttr(1.);pm.CreateRestitutionAttr(0.)
 for g in b['geoms']:
  shape=UsdGeom.Cylinder.Define(stage,'/Instrument/'+g['name']);shape.CreateRadiusAttr(g['size'][0]);shape.CreateHeightAttr(2*g['size'][1]);shape.CreateAxisAttr('Z');shape.AddTranslateOp().Set(Gf.Vec3d(*g['pos']));shape.CreateDisplayColorAttr([Gf.Vec3f(*g['rgba'][:3])]);UsdPhysics.CollisionAPI.Apply(shape.GetPrim());shape.GetPrim().AddAppliedSchema('PhysxCollisionAPI');attr(shape.GetPrim(),'physxCollision:contactOffset',.0002,Sdf.ValueTypeNames.Float);attr(shape.GetPrim(),'physxCollision:restOffset',0.,Sdf.ValueTypeNames.Float);UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(material,UsdShade.Tokens.weakerThanDescendants,'physics')
 stage.GetRootLayer().Save()
if __name__=='__main__':
 print(json.dumps([run(c) for c in ['A','B']]))
 for c in ['A','B']:instrument(c)

