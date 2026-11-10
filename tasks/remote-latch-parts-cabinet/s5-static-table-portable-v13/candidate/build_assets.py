"""Offline USD authoring. Exact source primitive sizes; no simulator or native imports."""
import json,math
from pathlib import Path
import numpy as np
from pxr import Gf,Sdf,Usd,UsdGeom,UsdPhysics,UsdShade
HERE=Path(__file__).resolve().parent
F=json.loads((HERE/'fixture.json').read_text())
def xf(stage,path,pos):
 x=UsdGeom.Xform.Define(stage,path);x.AddTranslateOp().Set(Gf.Vec3d(*map(float,pos)));return x.GetPrim()
def attr(p,n,v,t):p.CreateAttribute(n,t,custom=False).Set(v)
def mass_data(b):
 gs=[g for g in b['geoms'] if g['mass']>0];mass=sum(g['mass'] for g in gs)
 if not mass:return 1.,[0.,0.,0.],np.eye(3)*.001
 com=sum(g['mass']*np.array(g['pos']) for g in gs)/mass;I=np.zeros((3,3))
 for g in gs:
  m=g['mass'];s=np.array(g['size']);q=g['quat_wxyz'];R=np.array(Gf.Matrix3d(Gf.Quatd(q[0],Gf.Vec3d(*q[1:])))).T
  if g['kind']=='box':J=np.diag(m/3*np.array([s[1]**2+s[2]**2,s[0]**2+s[2]**2,s[0]**2+s[1]**2]))
  else:J=np.diag([m*(3*s[0]**2+4*s[1]**2)/12]*2+[m*s[0]**2/2])
  d=np.array(g['pos'])-com;I+=R@J@R.T+m*(np.dot(d,d)*np.eye(3)-np.outer(d,d))
 return mass,com,I
def author(name,free=False):
 bs=[b for b in F['bodies'] if b['name']==name or b['parent']==name];base=bs[0];origin=np.array(base['world_pos']);path=HERE/'assets'/f'{name}.usda';stage=Usd.Stage.CreateNew(str(path));UsdGeom.SetStageUpAxis(stage,'Z');UsdGeom.SetStageMetersPerUnit(stage,1.)
 root=xf(stage,'/Fixture',[0,0,0]);stage.SetDefaultPrim(root)
 if not free:
  UsdPhysics.ArticulationRootAPI.Apply(root);root.AddAppliedSchema('PhysxArticulationAPI');attr(root,'physxArticulation:enabledSelfCollisions',True,Sdf.ValueTypeNames.Bool)
 for b in bs:
  bp='/Fixture/links/'+b['name'];prim=xf(stage,bp,np.array(b['world_pos'])-origin);UsdPhysics.RigidBodyAPI.Apply(prim)
  m,c,I=mass_data(b);vals,R=np.linalg.eigh(I)
  if np.linalg.det(R)<0:R[:,0]*=-1
  Q=Gf.Matrix3d(*R.T.ravel()).ExtractRotation().GetQuat();mass=UsdPhysics.MassAPI.Apply(prim);mass.CreateMassAttr(m);mass.CreateCenterOfMassAttr(Gf.Vec3f(*map(float,c)));mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float,vals)));mass.CreatePrincipalAxesAttr(Gf.Quatf(Q))
  for g in b['geoms']:
   p=bp+'/'+g['name'];s=g['size']
   if g['kind']=='box':shape=UsdGeom.Cube.Define(stage,p);shape.CreateSizeAttr(1.)
   else:shape=UsdGeom.Cylinder.Define(stage,p);shape.CreateRadiusAttr(s[0]);shape.CreateHeightAttr(2*s[1]);shape.CreateAxisAttr('Z')
   shape.AddTranslateOp().Set(Gf.Vec3d(*g['pos']));q=g['quat_wxyz'];shape.AddOrientOp().Set(Gf.Quatf(q[0],Gf.Vec3f(*q[1:])))
   if g['kind']=='box':shape.AddScaleOp().Set(Gf.Vec3f(*(2*np.array(s))))
   shape.CreateDisplayColorAttr([Gf.Vec3f(*g['rgba'][:3])])
   if g['collision']:
    pr=shape.GetPrim();UsdPhysics.CollisionAPI.Apply(pr);pr.AddAppliedSchema('PhysxCollisionAPI');attr(pr,'physxCollision:contactOffset',.00025,Sdf.ValueTypeNames.Float);attr(pr,'physxCollision:restOffset',0.,Sdf.ValueTypeNames.Float)
    mat=UsdShade.Material.Define(stage,'/Fixture/material');pm=UsdPhysics.MaterialAPI.Apply(mat.GetPrim());pm.CreateStaticFrictionAttr(1.1);pm.CreateDynamicFrictionAttr(1.1);pm.CreateRestitutionAttr(0.);UsdShade.MaterialBindingAPI.Apply(pr).Bind(mat,UsdShade.Tokens.weakerThanDescendants,'physics')
  for j in b['joints']:
   typ=UsdPhysics.RevoluteJoint if j['kind']=='hinge' else UsdPhysics.PrismaticJoint;joint=typ.Define(stage,'/Fixture/joints/'+j['name']);axis=np.array(j['axis']);ii=int(np.argmax(abs(axis)));token='XYZ'[ii]
   joint.CreateAxisAttr(token);joint.CreateBody0Rel().SetTargets([Sdf.Path('/Fixture/links/'+b['parent'])]);joint.CreateBody1Rel().SetTargets([Sdf.Path(bp)])
   parent=next(x for x in bs if x['name']==b['parent']);joint.CreateLocalPos0Attr(Gf.Vec3f(*(np.array(b['world_pos'])-parent['world_pos'])));joint.CreateLocalPos1Attr(Gf.Vec3f(0,0,0))
   # Negative native axis is represented by a 180deg local joint-frame rotation.
   if axis[ii]<0:
    vv=np.zeros(3);vv[(ii+1)%3]=1.;q=Gf.Quatf(0.,Gf.Vec3f(*vv));joint.CreateLocalRot0Attr(q);joint.CreateLocalRot1Attr(q)
   scale=180/math.pi if j['kind']=='hinge' else 1.;joint.CreateLowerLimitAttr(j['range'][0]*scale);joint.CreateUpperLimitAttr(j['range'][1]*scale);joint.CreateCollisionEnabledAttr(False)
   pr=joint.GetPrim();pr.AddAppliedSchema('PhysxJointAPI');attr(pr,'physxJoint:armature',j['armature'],Sdf.ValueTypeNames.Float)
   instance='angular' if j['kind']=='hinge' else 'linear';pr.AddAppliedSchema('PhysxJointAxisAPI:'+instance)
   for key in ['staticFrictionEffort','dynamicFrictionEffort']:attr(pr,'physxJointAxis:'+instance+':'+key,j['frictionloss'],Sdf.ValueTypeNames.Float)
   drive=UsdPhysics.DriveAPI.Apply(pr,instance);drive.CreateTypeAttr('force');drive.CreateStiffnessAttr(j['stiffness']/scale);drive.CreateDampingAttr(j['damping']/scale);drive.CreateTargetPositionAttr(j['springref']*scale);drive.CreateMaxForceAttr(1000.)
 if not free:
  fixed=UsdPhysics.FixedJoint.Define(stage,'/Fixture/joints/fixed');fixed.CreateBody1Rel().SetTargets([Sdf.Path('/Fixture/links/'+name)])
  # Preserve native exclusions cabinet-door/paddle and door-door only.
  for b in bs[1:]:UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath('/Fixture/links/'+name)).CreateFilteredPairsRel().AddTarget(Sdf.Path('/Fixture/links/'+b['name']))
  cab=name[-1];UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath('/Fixture/links/'+cab+'_door_L')).CreateFilteredPairsRel().AddTarget(Sdf.Path('/Fixture/links/'+cab+'_door_R'))
 stage.GetRootLayer().Save();return dict(path=str(path.name),prims=sum(1 for _ in stage.Traverse()))
if __name__=='__main__':
 print(json.dumps([author(b['name'],b['free']) for b in F['bodies'] if b['parent'] is None]))
