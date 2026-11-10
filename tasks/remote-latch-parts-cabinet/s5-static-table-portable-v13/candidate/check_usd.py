"""Offline pxr validation; no physics or native imports."""
import json
from pathlib import Path
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics
HERE=Path(__file__).parent;f=json.loads((HERE/'fixture.json').read_text());checks={};counts={}
for b in f['bodies']:
 if b['parent'] is not None:continue
 stage=Usd.Stage.Open(str(HERE/'assets'/f"{b['name']}.usda"));cache=UsdGeom.XformCache();origin=np.array(b['world_pos']);geom_checks=[];n=0
 for body in f['bodies']:
  if body['name']!=b['name'] and body['parent']!=b['name']:continue
  for g in body['geoms']:
   prim=stage.GetPrimAtPath('/Fixture/links/'+body['name']+'/'+g['name']);n+=1;matrix=cache.GetLocalToWorldTransform(prim);actual=np.array(matrix.ExtractTranslation());expected=np.array(body['world_pos'])-origin+g['pos'];geom_checks.append(bool(np.allclose(actual,expected,atol=1e-7)))
   geom_checks.append(prim.HasAPI(UsdPhysics.CollisionAPI)==g['collision'])
   if g['kind']=='box':geom_checks.append(bool(np.allclose(np.array(UsdGeom.Xformable(prim).GetOrderedXformOps()[-1].Get()),2*np.array(g['size']),atol=1e-7)))
 checks[b['name']]=all(geom_checks);counts[b['name']]=n
 if b['free']:checks[b['name']+'_free']=not any(p.IsA(UsdPhysics.Joint) for p in stage.Traverse())
print(json.dumps(dict(all_pass=all(checks.values()),checks=checks,geom_counts=counts,physics_steps=0),indent=2))
assert all(checks.values())
