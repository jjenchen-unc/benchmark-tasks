"""Offline USD constraint authoring/anchor test; no simulator and no contacts."""
import json
from pathlib import Path
import numpy as np
from pxr import Usd,Gf,UsdGeom
from b6_physics import Physics
from mechanics import rotation
HERE=Path(__file__).resolve().parent
if __name__=='__main__':
 import argparse
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path);args=ap.parse_args()
 f=json.loads((HERE/'fixture.json').read_text());E=Physics.__new__(Physics);E.stage=Usd.Stage.CreateInMemory();E.envroot='/World/envs/env_0';E.constraints={};E.pin_constraints={};E.seat_contacts={(m,o):None for m,o in [('body_left','lens_orange'),('body_right','lens_blue'),('body_right','lens_purple')]};E.paths={}
 poses={}
 for root in ['body_left','body_right','lens_orange','lens_blue','lens_purple']:
  E.paths[root]=[]
  for b in f['bodies']:
   if b['name'].startswith(root):
    p=E.envroot+'/'+root+'/links/'+b['name'];UsdGeom.Xform.Define(E.stage,p);E.paths[root].append(p);poses[root,b['name']]=(np.array(b['pos'])+f['translation'],rotation(np.r_[b['quat'][1:],b['quat'][0]]))
 E.pose=lambda a,b:poses[a,b];E.make_constraints();E.prepare_reset();checks={'three_capture_constraints':len(E.constraints)==3,'two_physical_pins':len(E.pin_constraints)==2,'disabled_after_reset':all(not j.GetJointEnabledAttr().Get() for j in [*E.constraints.values(),*E.pin_constraints.values()])}
 j=E.constraints['body_right','lens_blue'];E.activate_current(j,'body_right','body_right_ring','lens_blue','lens_blue');pa,Ra=E.pose('body_right','body_right_ring');pb,Rb=E.pose('lens_blue','lens_blue');p0=pa+Ra@np.array(j.GetLocalPos0Attr().Get());p1=pb+Rb@np.array(j.GetLocalPos1Attr().Get());q=j.GetLocalRot0Attr().Get();Q=rotation(np.r_[q.GetImaginary(),q.GetReal()]);checks['current_anchor_no_snap']=np.linalg.norm(p0-p1)<1e-7;checks['current_rotation_no_snap']=np.max(abs(Ra@Q-Rb))<1e-6;checks['solver_joint_enabled']=j.GetJointEnabledAttr().Get();checks['excluded_from_articulation']=j.GetExcludeFromArticulationAttr().Get();E.prepare_reset();checks['reset_disables_constraint']=not j.GetJointEnabledAttr().Get();checks={k:bool(v) for k,v in checks.items()};result=dict(state='pass' if all(checks.values()) else 'failed',checks=checks,physics_steps=0,backend_proof=False);print(json.dumps(result,indent=2))
 if args.out:args.out.write_text(json.dumps(result,indent=2)+'\n')
 if result['state']!='pass':raise SystemExit(2)
