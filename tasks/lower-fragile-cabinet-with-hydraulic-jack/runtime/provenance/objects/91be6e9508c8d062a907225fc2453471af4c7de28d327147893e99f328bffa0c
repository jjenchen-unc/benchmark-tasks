"""Pure pxr asset generation; no Isaac/MuJoCo/physics imports."""
import json,math
from pathlib import Path
import numpy as np
from pxr import Gf,Sdf,Usd,UsdGeom,UsdPhysics,UsdShade
HERE=Path(__file__).resolve().parent
F=json.loads((HERE/'fixture.json').read_text())
def quat(q):return Gf.Quatd(float(q[0]),Gf.Vec3d(*map(float,q[1:])))
def xf(stage,path,pos,q=(1,0,0,0)):
 x=UsdGeom.Xform.Define(stage,path);x.AddTranslateOp().Set(Gf.Vec3d(*map(float,pos)));x.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(quat(q));return x.GetPrim()
def attr(p,n,v,t):p.CreateAttribute(n,t,custom=False).Set(v)
def rb(p,b):
 UsdPhysics.RigidBodyAPI.Apply(p);p.AddAppliedSchema('PhysxRigidBodyAPI');attr(p,'physxRigidBody:maxDepenetrationVelocity',.2,Sdf.ValueTypeNames.Float)
 a=UsdPhysics.MassAPI.Apply(p);a.CreateMassAttr(b['mass']);a.CreateCenterOfMassAttr(Gf.Vec3f(*b['com']));a.CreateDiagonalInertiaAttr(Gf.Vec3f(*b['inertia']));q=b['inertia_quat_wxyz'];a.CreatePrincipalAxesAttr(Gf.Quatf(q[0],Gf.Vec3f(*q[1:])))
def shapes(stage,bp,b,rebase=None):
 for g in b['geoms']:
  s=g['size'];path=bp+'/'+g['name']
  if g['kind']=='box':o=UsdGeom.Cube.Define(stage,path);o.CreateSizeAttr(1.);o.AddScaleOp().Set(Gf.Vec3f(*(2*np.array(s))))
  else:
   o=(UsdGeom.Capsule if g['kind']=='capsule' else UsdGeom.Cylinder).Define(stage,path);o.CreateRadiusAttr(s[0]);o.CreateHeightAttr(2*s[1]);o.CreateAxisAttr('Z')
  pos=np.array(g['pos'])-(np.array(rebase) if rebase is not None else 0.)
  o.AddTranslateOp().Set(Gf.Vec3d(*pos));o.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(quat(g['quat_wxyz']));ops=o.GetOrderedXformOps();o.SetXformOpOrder(sorted(ops,key=lambda v:{UsdGeom.XformOp.TypeTranslate:0,UsdGeom.XformOp.TypeOrient:1,UsdGeom.XformOp.TypeScale:2}[v.GetOpType()]))
  o.CreateDisplayColorAttr([Gf.Vec3f(*g['rgba'][:3])]);o.CreateDisplayOpacityAttr([g['rgba'][3]])
  if g['collision']:
   p=o.GetPrim();UsdPhysics.CollisionAPI.Apply(p);p.AddAppliedSchema('PhysxCollisionAPI');attr(p,'physxCollision:contactOffset',.0005,Sdf.ValueTypeNames.Float);attr(p,'physxCollision:restOffset',0.,Sdf.ValueTypeNames.Float)
   mat=UsdShade.Material.Define(stage,'/Fixture/materials/mu_'+str(g['friction'][0]).replace('.','_'));a=UsdPhysics.MaterialAPI.Apply(mat.GetPrim());a.CreateStaticFrictionAttr(g['friction'][0]);a.CreateDynamicFrictionAttr(g['friction'][0]);a.CreateRestitutionAttr(0.)
   UsdShade.MaterialBindingAPI.Apply(p).Bind(mat,UsdShade.Tokens.weakerThanDescendants,'physics')
def stage_new(name):
 s=Usd.Stage.CreateNew(str(HERE/(name+'.usda')));UsdGeom.SetStageMetersPerUnit(s,1);UsdGeom.SetStageUpAxis(s,'Z');p=xf(s,'/Fixture',[0,0,0]);s.SetDefaultPrim(p);return s,p

def author():
 for tag in ('S','T'):
  selected=[b for b in F['bodies'] if b['name'].endswith('_'+tag)];root=next(b for b in selected if b['name']=='jack_'+tag);origin=np.array(root['world_pos']);origin[2]=.8
  stage,p=stage_new('jack_'+tag);UsdPhysics.ArticulationRootAPI.Apply(p);p.AddAppliedSchema('PhysxArticulationAPI');attr(p,'physxArticulation:enabledSelfCollisions',True,Sdf.ValueTypeNames.Bool)
  attr(p,'physxArticulation:solverPositionIterationCount',64,Sdf.ValueTypeNames.Int);attr(p,'physxArticulation:solverVelocityIterationCount',8,Sdf.ValueTypeNames.Int)
  by={b['name']:b for b in selected}
  for b in selected:
   bp='/Fixture/links/'+b['name'];pos=np.zeros(3) if b is root else np.array(b['world_pos'])-origin;bb=dict(b)
   if b is root:bb['com']=(np.array(b['com'])-np.array([0,0,.8])).tolist()
   prim=xf(stage,bp,pos,b['quat_wxyz']);rb(prim,bb);shapes(stage,bp,b,[0,0,.8] if b is root else None)
   for j in b['joints']:
    jp='/Fixture/joints/'+j['name'];slide=j['kind']=='slide';o=(UsdPhysics.PrismaticJoint if slide else UsdPhysics.RevoluteJoint).Define(stage,jp)
    parent=by[b['parent']];o.CreateBody0Rel().SetTargets([Sdf.Path('/Fixture/links/'+parent['name'])]);o.CreateBody1Rel().SetTargets([Sdf.Path(bp)])
    axis=np.array(j['axis_local']);q=np.r_[1+axis[0],np.cross([1,0,0],axis)];q=q/np.linalg.norm(q)
    o.CreateAxisAttr('X');o.CreateLocalRot0Attr(Gf.Quatf(q[0],Gf.Vec3f(*q[1:])));o.CreateLocalRot1Attr(Gf.Quatf(q[0],Gf.Vec3f(*q[1:])))
    anchor=np.array(b['world_pos'])+j['anchor_local']-np.array(parent['world_pos']);anchor-=np.array([0,0,.8]) if parent is root else 0
    o.CreateLocalPos0Attr(Gf.Vec3f(*anchor));o.CreateLocalPos1Attr(Gf.Vec3f(*j['anchor_local']));o.CreateCollisionEnabledAttr(False)
    scale=1 if slide else 180/math.pi
    if j['range'] is not None:o.CreateLowerLimitAttr(j['range'][0]*scale);o.CreateUpperLimitAttr(j['range'][1]*scale)
    inst='linear' if slide else 'angular';drive=UsdPhysics.DriveAPI.Apply(o.GetPrim(),inst);drive.CreateTypeAttr('force');drive.CreateStiffnessAttr((200000. if slide else j['stiffness'])/scale);drive.CreateDampingAttr((400. if slide else j['damping'])/scale);drive.CreateTargetPositionAttr(j['springref']*scale);drive.CreateMaxForceAttr(float('inf'))
    o.GetPrim().AddAppliedSchema('PhysxJointAPI');attr(o.GetPrim(),'physxJoint:armature',j['armature'],Sdf.ValueTypeNames.Float)
    o.GetPrim().AddAppliedSchema('PhysxJointAxisAPI:'+inst)
    for key in ('staticFrictionEffort','dynamicFrictionEffort'):attr(o.GetPrim(),'physxJointAxis:'+inst+':'+key,j['frictionloss'],Sdf.ValueTypeNames.Float)
  o=UsdPhysics.FixedJoint.Define(stage,'/Fixture/joints/world_fixed');o.CreateBody1Rel().SetTargets([Sdf.Path('/Fixture/links/jack_'+tag)])
  stage.GetRootLayer().Save()
 stage,p=stage_new('cabinet');b=next(b for b in F['bodies'] if b['name']=='cabinet');rb(p,b);shapes(stage,'/Fixture',b);stage.GetRootLayer().Save()
if __name__=='__main__':author();print('Authored jack_S.usda jack_T.usda cabinet.usda; physics unverified')
