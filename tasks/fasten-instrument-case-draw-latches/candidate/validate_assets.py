"""pxr static asset audit; never starts Isaac or advances physics."""
import json,math
from pathlib import Path
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics
HERE=Path(__file__).resolve().parent;F=json.loads((HERE/'fixture.json').read_text());checks={};summary=[]
for c in ['A','B']:
 stage=Usd.Stage.Open(str(HERE/'assets'/f'case-{c}.usda'));root=stage.GetDefaultPrim();bodies=[b for b in F['bodies'] if b['name']=='case_'+c or b['name']=='lid_'+c or b['name'].startswith(c)];origin=np.array(next(b for b in bodies if b['name']=='case_'+c)['pos']);cache=UsdGeom.XformCache();count=0
 checks[c+'_floating_root_no_world_weld']=not any(p.IsA(UsdPhysics.FixedJoint) and not UsdPhysics.Joint(p).GetBody0Rel().GetTargets() for p in stage.Traverse())
 for b in bodies:
  p=stage.GetPrimAtPath('/Case/links/'+b['name']);mass=UsdPhysics.MassAPI(p);checks[c+'_mass_'+b['name']]=abs(float(mass.GetMassAttr().Get())-b['mass'])<1e-6
  T=np.array(cache.GetLocalToWorldTransform(p));checks[c+'_bodypose_'+b['name']]=np.allclose(T[3,:3],np.array(b['pos'])-origin,atol=1e-7)
  for g in b['geoms']:
   gp=stage.GetPrimAtPath(str(p.GetPath())+'/'+g['name']);checks[g['name']+'_present']=bool(gp);checks[g['name']+'_collision']=gp.HasAPI(UsdPhysics.CollisionAPI)==bool(g['contype'] or g['conaffinity']);count+=1
   ops={op.GetOpType():op.Get() for op in UsdGeom.Xformable(gp).GetOrderedXformOps()};checks[g['name']+'_local_position']=np.allclose(ops[UsdGeom.XformOp.TypeTranslate],g['pos'],atol=1e-7)
   quat=ops[UsdGeom.XformOp.TypeOrient];checks[g['name']+'_local_orientation']=np.allclose([quat.GetReal(),*quat.GetImaginary()],g['quat'],atol=1e-7)
   if g['kind']=='box':checks[g['name']+'_size']=np.allclose(ops[UsdGeom.XformOp.TypeScale],g['size'],atol=1e-7) and UsdGeom.Cube(gp).GetSizeAttr().Get()==2.
   else:checks[g['name']+'_size']=abs(UsdGeom.Cylinder(gp).GetRadiusAttr().Get()-g['size'][0])<1e-7 and abs(UsdGeom.Cylinder(gp).GetHeightAttr().Get()-2*g['size'][1])<1e-7
  for j in b['joints']:
   jp=UsdPhysics.Joint(stage.GetPrimAtPath('/Case/joints/'+j['name']));checks[j['name']+'_bodies']=len(jp.GetBody0Rel().GetTargets())==len(jp.GetBody1Rel().GetTargets())==1
 for e in F['connections']:
  if e['name'].startswith(c):
   j=UsdPhysics.Joint(stage.GetPrimAtPath('/Case/connections/'+e['name']));checks[e['name']+'_initial']=j.GetJointEnabledAttr().Get()==e['enabled'];checks[e['name']+'_loop_excluded']=j.GetExcludeFromArticulationAttr().Get() is True;checks[e['name']+'_capture_collision_enabled']=j.GetCollisionEnabledAttr().Get() is True
 if c=='B':checks['B_fixed_catches']=all(stage.GetPrimAtPath('/Case/joints/B'+str(i)+'_catch_weld').IsA(UsdPhysics.FixedJoint) for i in range(2))
 summary.append(dict(case=c,body_count=len(bodies),geom_count=count,mass_kg=sum(b['mass'] for b in bodies)))
 s=Usd.Stage.Open(str(HERE/'assets'/f'instrument-{c}.usda'));pr=s.GetDefaultPrim();checks['instrument_'+c+'_free']=pr.HasAPI(UsdPhysics.RigidBodyAPI);checks['instrument_'+c+'_mass']=abs(UsdPhysics.MassAPI(pr).GetMassAttr().Get()-.3)<1e-6;checks['instrument_'+c+'_cap_geometry_preserved']=len([p for p in s.Traverse() if p.IsA(UsdGeom.Cylinder)])==2
report=dict(kind=__doc__,checks=checks,count=len(checks),all_passed=all(checks.values()),summary=summary);print(json.dumps(report,indent=2))
if not report['all_passed']:raise SystemExit(1)
