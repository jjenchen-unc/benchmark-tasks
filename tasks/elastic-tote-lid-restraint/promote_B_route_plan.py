from pathlib import Path
import argparse,json,hashlib,os,sys,numpy as np
from cord_gripper_contact import classify
parser=argparse.ArgumentParser();parser.add_argument('--trial',type=Path,required=True);parser.add_argument('--state-name',default='b-live-state.json');parser.add_argument('--audit-subdir',default='.');parser.add_argument('--output-name',default='b-plan.json');args=parser.parse_args();p=args.trial.resolve()
assert Path(args.state_name).name==args.state_name and Path(args.output_name).name==args.output_name
aud=(p/args.audit_subdir).resolve();assert aud.is_relative_to(p)
assert not (p/'result.json').exists(),'Trial has already ended'
assert not (p/args.output_name).exists(),'Plan already delivered'
blob=(p/args.state_name).read_bytes();state=json.loads(blob);digest=hashlib.sha256(blob).hexdigest()
reference={str(f.relative_to(p/'source')):hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted((p/'source').rglob('*')) if f.is_file() and '__pycache__' not in f.parts and f.suffix!='.pyc'}
assert len(reference)==24 and state['task_source_sha256']==reference==json.loads((p/'source-sha256.json').read_text())
assert state['row']['straps']['strap_4']['seated_a']==0 and not state['row']['robot_contacts']['strap_4_a']
assert state['row']['totes']['tote_tall']['lid_gap_m']<.002
assert all(e['position_m']<.001 and e['rotation_rad']<.01 for es in state['fk_calibration'].values() for e in es)
path=json.loads((aud/'b-path-audit.json').read_text());release=json.loads((aud/'b-release-retreat-audit.json').read_text())
assert path['state_sha256']==digest==release['state_sha256']
assert path['status']=='checked' and release['passed'] and path['initial_guard_passed']
a=path['attempts'][-1];assert not a['issues'] and a['max_tension_n']<28
assert a['minimum_fixture_m']>.001 and a['minimum_self_m']>.001 and a['minimum_cross_m']>.001 and a['minimum_held_fixture_m']>=-.0002
assert release['candidate']==a['candidate']
sys.path.insert(0,str(p/"source/d1r_arena"))
import geometry as geo,strap
from scipy.spatial.transform import Rotation
assert path["source_goal_seated_bar"]==1
assert strap.seat_check(np.array(a["candidate"]["hook_pos"]),Rotation.from_matrix(np.array(a["candidate"]["hook_R"])).as_quat(),0.,strap.ToteFrame())==1,"B goal fails source seating predicate"
from measured_planning_scene import MeasuredScene
_verified_scene=MeasuredScene(state,geo,strap)
_verified_held=[_verified_scene.check_held_hook("strap_4_b",np.array(w["hook_pos"]),np.array(w["hook_R"])) for w in path["waypoints"]]
assert all(not h["fixture_hits"] and min(v["gap_m"] for v in h["all_bar_capsule_gaps"])>=-.0001 for h in _verified_held),"Hook intersects physical fixture in recomputed source geometry"
for w in path["waypoints"]:
 held=w["held_check"]
 assert held["fixture_bar_count"]==4 and len(held["all_bar_capsule_gaps"])==12
 assert not held["fixture_hits"] and all(v["gap_m"]>=-.0001 for v in held["all_bar_capsule_gaps"]),"Hook intersects eye bar"
for check in [w["robot_check"] for w in path["waypoints"]]+[r["check"] for r in release["opening"]+release["right_retreat"]+release["left_retreat"]]:
 assert check["fixture_bar_count"]==4 and check["bar_capsule_checks"]
 assert check["bar_minimum_separation_m"]>.001 and all(v["gap_lower_m"]>.001 for v in check["bar_capsule_checks"]),"Robot intersects eye bar"

assert len(release['opening'])==121 and len(release['right_retreat'])==46 and len(release['left_retreat'])==61
assert all(not classify(w['robot_check'],segment=w['owned_segment'],pad_proxy=True)['issues'] for w in path['waypoints'])
assert all(not classify(r['check'],segment=release['owned_segment'],pad_proxy=True)['issues'] for r in release['opening'])
assert all(not classify(r['check'],segment=release['owned_segment'],pad_proxy=False)['issues'] for r in release['right_retreat'])
assert not classify(release['right_retreat'][-1]['check'],segment=release['owned_segment'],pad_proxy=False)['surface_contacts']
_retreat_vector=np.asarray(release["right_retreat_vector"],float)
assert _retreat_vector.shape==(3,) and np.isfinite(_retreat_vector).all() and np.linalg.norm(_retreat_vector)<.2
for _r in release["right_retreat"]:
 _p,_R=_verified_scene.tcp("right",np.asarray(_r["q"],float))
 assert np.linalg.norm(_p-(np.array(a["candidate"]["tcp"])+float(_r["fraction"])*_retreat_vector))<.00101,"Retreat vector does not bind checked FK path"
 assert np.linalg.norm(Rotation.from_matrix(_R@np.array(a["candidate"]["R"]).T).as_rotvec())<.01001
assert 1<=len(path['waypoints'])<1000
for w in path['waypoints']:
 q=np.array(w['q']);assert q.shape==(6,) and np.isfinite(q).all()
 xyz=np.array(w['hook_pos']);R=np.array(w['hook_R'])
 assert xyz.shape==(3,) and R.shape==(3,3) and np.isfinite(xyz).all() and np.isfinite(R).all()
 assert np.allclose(R.T@R,np.eye(3),atol=1e-6) and np.linalg.det(R)>.999999
assert np.allclose(path['waypoints'][-1]['hook_pos'],a['candidate']['hook_pos'],atol=1e-8)
assert np.allclose(path['waypoints'][-1]['hook_R'],a['candidate']['hook_R'],atol=1e-8)
plan=dict(state_sha256=digest,status='ready',waypoints=path['waypoints'],contact_local=path['contact_local'],bar_goal=path['bar_goal'],right_retreat_vector=_retreat_vector.tolist(),
 provenance=dict(path_audit_sha256=hashlib.sha256((aud/'b-path-audit.json').read_bytes()).hexdigest(),release_audit_sha256=hashlib.sha256((aud/'b-release-retreat-audit.json').read_bytes()).hexdigest(),scope='Nominal planning only; actual robot/fullgoal determined solely by runtime'))
tmp=p/(args.output_name+'.tmp');tmp.write_text(json.dumps(plan,indent=2));os.replace(tmp,p/args.output_name)
print(json.dumps(dict(delivered=True,state_sha256=digest,waypoints=len(plan['waypoints']),max_predicted_tension_n=a['max_tension_n'])))
