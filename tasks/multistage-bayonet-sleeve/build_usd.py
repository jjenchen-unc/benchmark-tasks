"""USD authoring after Kit launch (or offline pxr). All bore sectors are separate boxes.
No convex hull of the sleeve, no substituted release state machine.
"""
import json,math
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent
def author():
 from pxr import Gf,Sdf,Usd,UsdGeom,UsdPhysics,UsdShade
 f=json.loads((HERE/'fixture.json').read_text());bodies={b['name']:b for b in f['bodies']};stand=np.array(bodies['stand_C']['pos'])
 def schema(p,n):p.AddAppliedSchema(n)
 def attr(p,n,v,t=Sdf.ValueTypeNames.Float):p.CreateAttribute(n,t,custom=False).Set(v)
 def xf(stage,path,p,q=(1,0,0,0)):
  x=UsdGeom.Xform.Define(stage,path);x.AddTranslateOp().Set(Gf.Vec3d(*map(float,p)));x.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(q[0]),Gf.Vec3d(*map(float,q[1:]))));return x.GetPrim()
 def shapes(stage,path,body,shift):
  for i,g in enumerate(body['geoms']):
   gp=path+'/g_'+str(i)+'_'+g['name'].replace('+','p').replace('-','m').replace('.','_');s=g['size']
   if g['type']=='box':
    shape=UsdGeom.Cube.Define(stage,gp);shape.CreateSizeAttr(1.);shape.AddScaleOp().Set(Gf.Vec3f(*map(float,2*np.array(s))))
   elif g['type']=='cylinder':
    shape=UsdGeom.Cylinder.Define(stage,gp);shape.CreateAxisAttr('Z');shape.CreateRadiusAttr(float(s[0]));shape.CreateHeightAttr(float(2*s[1]))
   else:raise ValueError(g['type'])
   shape.AddTranslateOp().Set(Gf.Vec3d(*map(float,np.array(g['pos'])+shift)));q=g['quat'];shape.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(q[0]),Gf.Vec3d(*map(float,q[1:]))))
   ops=shape.GetOrderedXformOps();shape.SetXformOpOrder(sorted(ops,key=lambda o:{UsdGeom.XformOp.TypeTranslate:0,UsdGeom.XformOp.TypeOrient:1,UsdGeom.XformOp.TypeScale:2}[o.GetOpType()]));shape.CreateDisplayColorAttr([Gf.Vec3f(*g['rgba'][:3])])
   if g['collision']:
    prim=shape.GetPrim();UsdPhysics.CollisionAPI.Apply(prim);schema(prim,'PhysxCollisionAPI');attr(prim,'physxCollision:contactOffset',.00005);attr(prim,'physxCollision:restOffset',0.)
    # Mesh-free primitives preserve the open annulus and each hidden channel.
    mat=UsdShade.Material.Define(stage,'/Asset/materials/m_'+body['name']+'_'+str(i));pm=UsdPhysics.MaterialAPI.Apply(mat.GetPrim());pm.CreateStaticFrictionAttr(g['friction']);pm.CreateDynamicFrictionAttr(g['friction']);pm.CreateRestitutionAttr(0.);UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat,UsdShade.Tokens.weakerThanDescendants,'physics')
 def rigid(stage,path,mass,com,inertia):
  p=stage.GetPrimAtPath(path);UsdPhysics.RigidBodyAPI.Apply(p);m=UsdPhysics.MassAPI.Apply(p);m.CreateMassAttr(mass);m.CreateCenterOfMassAttr(Gf.Vec3f(*map(float,com)));m.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float,inertia)));schema(p,'PhysxRigidBodyAPI');attr(p,'physxRigidBody:maxDepenetrationVelocity',.5)
 paths=[]
 for kind in ('fixture','sleeve'):
  path=HERE/('coupling-'+kind+'.usda');stage=Usd.Stage.CreateNew(str(path));UsdGeom.SetStageUpAxis(stage,'Z');UsdGeom.SetStageMetersPerUnit(stage,1.);stage.SetDefaultPrim(xf(stage,'/Asset',[0,0,0]))
  if kind=='sleeve':
   b=bodies['sleeve_C'];ii=b['inertial'];rigid(stage,'/Asset',float(ii['mass']),[float(x) for x in ii['pos'].split()],[float(x) for x in ii['diaginertia'].split()]);shapes(stage,'/Asset',b,np.zeros(3))
  else:
   root=stage.GetPrimAtPath('/Asset');UsdPhysics.ArticulationRootAPI.Apply(root);schema(root,'PhysxArticulationAPI');attr(root,'physxArticulation:enabledSelfCollisions',True,Sdf.ValueTypeNames.Bool);attr(root,'physxArticulation:solverPositionIterationCount',64,Sdf.ValueTypeNames.Int)
   xf(stage,'/Asset/stand',[0,0,0]);rigid(stage,'/Asset/stand',2.,[0,0,0],[.01,.01,.01]);shapes(stage,'/Asset/stand',bodies['stand_C'],np.zeros(3))
   # Tray is physically fixed to world through the stand; relative transform exact.
   for part,selected in [('tray_floor',[g for g in bodies['tray']['geoms'] if g['name']=='tray_floor']),('tray_walls',[g for g in bodies['tray']['geoms'] if g['name']!='tray_floor'])]:
    offset=np.array(bodies['tray']['pos'])-stand;xf(stage,'/Asset/'+part,offset);rigid(stage,'/Asset/'+part,.2,[0,0,0],[.001,.001,.001]);shapes(stage,'/Asset/'+part,dict(name=part,geoms=selected),np.zeros(3))
    fixed=UsdPhysics.FixedJoint.Define(stage,'/Asset/fixed_'+part);fixed.CreateBody0Rel().SetTargets(['/Asset/stand']);fixed.CreateBody1Rel().SetTargets(['/Asset/'+part]);fixed.CreateLocalPos0Attr(Gf.Vec3f(*map(float,offset)));fixed.CreateLocalPos1Attr(Gf.Vec3f(0,0,0));fixed.CreateCollisionEnabledAttr(False)
   b=bodies['plunger_C'];xf(stage,'/Asset/plunger',b['pos']);rigid(stage,'/Asset/plunger',.025,[0,0,-.0033],[1.87775e-6,1.87775e-6,.85e-6]);shapes(stage,'/Asset/plunger',b,np.zeros(3))
   j=UsdPhysics.FixedJoint.Define(stage,'/Asset/world_fixed');j.CreateBody1Rel().SetTargets(['/Asset/stand'])
   j=UsdPhysics.PrismaticJoint.Define(stage,'/Asset/plunger_slide_C');j.CreateAxisAttr('Z');j.CreateBody0Rel().SetTargets(['/Asset/stand']);j.CreateBody1Rel().SetTargets(['/Asset/plunger']);j.CreateLocalPos0Attr(Gf.Vec3f(*b['pos']));j.CreateLocalPos1Attr(Gf.Vec3f(0,0,0));j.CreateLowerLimitAttr(-.032);j.CreateUpperLimitAttr(0.);j.CreateCollisionEnabledAttr(False);schema(j.GetPrim(),'PhysxJointAPI');attr(j.GetPrim(),'physxJoint:armature',.001)
   drive=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'linear');drive.CreateTypeAttr('force');drive.CreateStiffnessAttr(f['parameters']['plunger_k']);drive.CreateDampingAttr(1.5);drive.CreateTargetPositionAttr(0.);drive.CreateMaxForceAttr(float('inf'))
  stage.GetRootLayer().Save();paths.append(str(path))
 return paths
if __name__=='__main__':print(json.dumps(author()))
