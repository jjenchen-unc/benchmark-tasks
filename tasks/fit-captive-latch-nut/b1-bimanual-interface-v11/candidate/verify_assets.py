"""Offline USD/JSON geometry and mass checks; does not load a simulator."""
import json,math
from pathlib import Path
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,Gf
from scipy.spatial.transform import Rotation
HERE=Path(__file__).resolve().parent

def run():
    f=json.loads((HERE/'fixture.json').read_text());checks={};inventory=[]
    fixture=Usd.Stage.Open(str(HERE/'assets/fixture.usda'));by={b['name']:b for b in f['bodies']}
    world={}
    def pose(b):
        if b['name'] in world:return world[b['name']]
        p=np.asarray(b['pos']);r=Rotation.from_quat(np.roll(b['quat'],-1))
        if b['parent']:
            pp,pr=pose(by[b['parent']]);p=pp+pr.apply(p);r=pr*r
        world[b['name']]=(p,r);return p,r
    for b in f['bodies']:
        stage=Usd.Stage.Open(str(HERE/'assets'/(b['name']+'.usda'))) if b['free'] else fixture
        root='/Wrench' if b['free'] else '/Wrench/links/'+b['name']
        prim=stage.GetPrimAtPath(root)
        checks[b['name']+'_rigid_body']=prim.HasAPI(UsdPhysics.RigidBodyAPI)
        if not b['free']:
            M=UsdGeom.Xformable(prim).GetLocalTransformation();p,r=pose(b)
            checks[b['name']+'_world_pose']=np.allclose(np.asarray(M.ExtractTranslation()),p,atol=1e-8) and np.allclose(np.asarray(M.ExtractRotationMatrix()).T,r.as_matrix(),atol=1e-6)
        material_names=[]
        for g in b['geoms']:
            path=root+'/'+g['name'];shape=stage.GetPrimAtPath(path)
            checks[g['name']+'_present']=bool(shape)
            checks[g['name']+'_collision_mask']=shape.HasAPI(UsdPhysics.CollisionAPI)==bool(g['contype'] or g['conaffinity'])
            if g['type']=='box':checks[g['name']+'_dimensions']=np.allclose(shape.GetAttribute('xformOp:scale').Get(),g['size'])
            if g['type']=='cylinder':checks[g['name']+'_dimensions']=np.isclose(shape.GetAttribute('radius').Get(),g['size'][0]) and np.isclose(shape.GetAttribute('height').Get(),2*g['size'][1])
            if g['type']=='mesh':checks[g['name']+'_vertices']=np.allclose(np.asarray(shape.GetAttribute('points').Get()).ravel(),f['meshes'][g['mesh']])
            if g['contype'] or g['conaffinity']:
                mat=stage.GetPrimAtPath(path+'_material')
                checks[g['name']+'_friction']=np.isclose(mat.GetAttribute('physics:staticFriction').Get(),g['friction'][0])
            material_names.append(dict(name=g['name'],type=g['type'],local_pos_m=g['pos'],quat_wxyz=g['quat'],half_sizes_or_radius_halfheight_m=g['size'],rgba=g['rgba'],friction=g['friction'],collision_masks=[g['contype'],g['conaffinity']],source_mass_kg=float(g['mass']) if 'mass' in g else None))
        mass=UsdPhysics.MassAPI(prim).GetMassAttr().Get()
        if b['free']:checks[b['name']+'_mass']=math.isclose(mass,sum(float(g.get('mass',0)) for g in b['geoms']),abs_tol=1e-7)
        if b['name'] in ['w12','w1_2','w13','w14']:
            checks[b['name']+'_numeric_stamp']=any(p.GetName().startswith('size_') for p in prim.GetChildren())
        inventory.append(dict(name=b['name'],free=b['free'],source_body_position_m=b['pos'],source_body_quat_wxyz=b['quat'],usd_body_mass_kg=mass,geometries=material_names))
    for n in ['arm_hinge','latch_slip']:
        j=UsdPhysics.RevoluteJoint(fixture.GetPrimAtPath('/Wrench/joints/'+n));checks[n+'_initial_locked']=j.GetLowerLimitAttr().Get()==j.GetUpperLimitAttr().Get()==0.
    checks['three_task_joints']=sum(bool(p.IsA(UsdPhysics.RevoluteJoint)) for p in fixture.Traverse())==3
    checks['no_external_asset_paths']=all('/mnt/' not in p.read_text() and '@' not in p.read_text() for p in (HERE/'assets').glob('*.usda'))
    checks={k:bool(v) for k,v in checks.items()}
    (HERE/'physical-inventory.json').write_text(json.dumps(dict(transform=dict(rotation=f['rotation'],translation_m=f['translation']),bodies=inventory),indent=2)+'\n')
    return dict(kind='offline USD structural checks, not backend proof',checks=checks,passed=sum(checks.values()),total=len(checks),all_pass=all(checks.values()))

if __name__=='__main__':
    r=run();print(json.dumps(r,indent=2));raise SystemExit(0 if r['all_pass'] else 1)
