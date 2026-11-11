"""Offline geometry/mass/axis checks only; cannot prove PhysX mechanisms."""
import json,math
from pathlib import Path
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,Gf
from build_usd import R
HERE=Path(__file__).resolve().parent

def run():
 f=json.loads((HERE/'fixture.json').read_text());max_pos=max_rot=0.;checks=[];shapes=0;joints=0
 for name in ['body_left','body_right','lens_orange','lens_blue','lens_purple','tray_left','tray_right']:
  bs=[next(b for b in f['bodies'] if b['name']==name)];names={name}
  for b in f['bodies']:
   if b['parent'] in names:bs.append(b);names.add(b['name'])
  root=bs[0];s=Usd.Stage.Open(str(HERE/'assets'/f'{name}.usda'));cache=UsdGeom.XformCache();Ro=R(root['quat']);origin=np.array(root['pos'])+f['translation']
  for b in bs:
   bp=s.GetPrimAtPath('/B6/links/'+b['name']);mat=cache.GetLocalToWorldTransform(bp);rot=np.asarray(mat.ExtractRotationMatrix()).T;pos=np.array(mat.ExtractTranslation());expected_mass=max(b['mass'],1e-6)
   checks.append(('mass_'+b['name'],math.isclose(UsdPhysics.MassAPI(bp).GetMassAttr().Get(),expected_mass,rel_tol=1e-5)))
   for g in b['geoms']:
    p=s.GetPrimAtPath(str(bp.GetPath())+'/'+g['name']);m=cache.GetLocalToWorldTransform(p);c=origin+Ro@np.array(m.ExtractTranslation());goal=np.array(b['pos'])+f['translation']+R(b['quat'])@np.array(g['pos']);err=float(np.linalg.norm(c-goal));max_pos=max(max_pos,err);shapes+=1
    checks.append(('shape_collision_'+g['name'],p.HasAPI(UsdPhysics.CollisionAPI)==bool(g['contype'] or g['conaffinity'])))
    # Unit-normalized columns remove box scale from its transform.
    A=np.asarray(m)[:3,:3].T;A=A/np.linalg.norm(A,axis=0);goalR=R(b['quat'])@R(g['quat']);max_rot=max(max_rot,float(np.max(abs(Ro@A-goalR))))
   for j in b['joints']:
    p=s.GetPrimAtPath('/B6/joints/'+j['name']);checks.append(('joint_'+j['name'],bool(p)));joints+=1
  checks.append(('root_identity_'+name,np.allclose(pos if len(bs)==1 else np.array(cache.GetLocalToWorldTransform(s.GetPrimAtPath('/B6/links/'+name)).ExtractTranslation()),0.)))
 checks.extend([('all_75_task_shapes',shapes==75),('all_10_task_joints',joints==10),('geometry_world_error',max_pos<1e-7),('geometry_rotation_error',max_rot<1e-6)])
 return dict(state='pass' if all(v for _,v in checks) else 'failed',checks=dict(checks),shape_count=shapes,joint_count=joints,max_world_position_error_m=max_pos,max_rotation_matrix_error=max_rot,physics_steps=0,backend_proof=False)
if __name__=='__main__':
 import argparse
 p=argparse.ArgumentParser();p.add_argument('--out',type=Path);a=p.parse_args();r=run();text=json.dumps(r,indent=2);print(text)
 if a.out:a.out.write_text(text+'\n')
 if r['state']!='pass':raise SystemExit(2)
