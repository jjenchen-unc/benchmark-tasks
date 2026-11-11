"""Offline USD identity/asset closure verification, not a dynamics test."""
import json,math,sys
from pathlib import Path
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics
HERE=Path(__file__).resolve().parent
f=json.loads((HERE/'fixture.json').read_text());s=Usd.Stage.Open(str(HERE/'stand.usda'));checks={};n=0;errors=[]
for b in f['bodies']:
 branch='links' if b['mass']>0 else 'static';bp='/Pins/'+branch+'/'+b['name'];p=s.GetPrimAtPath(bp);checks['body_'+b['name']]=p.IsValid()
 if b['mass']>0:checks['mass_'+b['name']]=abs(float(UsdPhysics.MassAPI(p).GetMassAttr().Get())-b['mass'])<1e-6
 for g in b['geoms']:
  p=s.GetPrimAtPath(bp+'/'+g['name']);x=UsdGeom.Xformable(p);ops=x.GetOrderedXformOps();pos=next(o.Get() for o in ops if o.GetOpType()==UsdGeom.XformOp.TypeTranslate);q=next(o.Get() for o in ops if o.GetOpType()==UsdGeom.XformOp.TypeOrient);quat=[q.GetReal(),*q.GetImaginary()];err=max(np.max(abs(np.array(pos)-g['pos'])),np.max(abs(np.array(quat)-g['quat'])));errors.append(float(err))
  if g['kind']=='box':v=next(o.Get() for o in ops if o.GetOpType()==UsdGeom.XformOp.TypeScale);ok=np.allclose(v,2*np.array(g['size']),atol=1e-8)
  else:cl=UsdGeom.Cylinder if g['kind']=='cylinder' else UsdGeom.Capsule;shape=cl(p);ok=abs(shape.GetRadiusAttr().Get()-g['size'][0])<1e-8 and abs(shape.GetHeightAttr().Get()-2*g['size'][1])<1e-8
  checks['shape_'+g['name']]=bool(ok and err<1e-8 and p.HasAPI(UsdPhysics.CollisionAPI)==any(g['mask']));n+=1
 checks['source_63_shapes']=n==63
for j in s.Traverse():
 if j.IsA(UsdPhysics.Joint):
  for rel in ['physics:body0','physics:body1']:
   checks[str(j.GetPath())+rel]=all(s.GetPrimAtPath(p).IsValid() for p in j.GetRelationship(rel).GetTargets())
checks['two_passive_loop_spherical_joints_excluded']=all(UsdPhysics.Joint(s.GetPrimAtPath('/Pins/joints/join'+l)).GetExcludeFromArticulationAttr().Get() is True for l in 'LR')
checks['source_joint10_present']=len([p for p in s.Traverse() if p.IsA(UsdPhysics.PrismaticJoint) or p.IsA(UsdPhysics.RevoluteJoint)])==10
checks['no_external_asset_dependencies']=all(not l.subLayerPaths for l in s.GetLayerStack()) and not any(p.HasAuthoredReferences() or p.HasAuthoredPayloads() for p in s.Traverse())
out=dict(kind=__doc__,checks=checks,all_passed=all(checks.values()),shape_count=n,max_shape_transform_error=max(errors),physics_steps=0)
print(json.dumps({k:v for k,v in out.items() if k!='checks'},indent=2))
if '--out' in sys.argv:Path(sys.argv[sys.argv.index('--out')+1]).write_text(json.dumps(out,indent=2))
if not out['all_passed']:print([k for k,v in checks.items() if not v]);raise SystemExit(1)
