from pathlib import Path
import json,hashlib
R=Path(__file__).resolve().parent;OLD=R.parent/'hydraulic-actual-contact-whole-v3-20261110-0840';D=R.parents[1]/'continuation/server-s5-hydraulic-c1-finish-20261110-0920/Hydraulic_supported_YAM_valve_route';D.mkdir(parents=True,exist_ok=True)
def read(p):return json.loads(p.read_text())
def put(p,v):p.write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
route=read(R/'command-route.json');robot=read(R/'self-audit.json');controller=read(R/'controller-contract.json');left=read(R/'left-fixture-audit.json');passed=route['passed'] and robot['passed'] and controller['passed'] and left['passed']
x=dict(unit='Hydraulic_supported_YAM_valve_route',owned_roots=[str(R)],delivery=str(D),status='CPU_CONDITIONAL_CANDIDATE_ROOT_REVIEW_REQUIRED' if passed else 'CPU_REJECTED_ROOT_REVIEW_REQUIRED',whole_goal_candidate_ready=passed,full_goal_verified=False,submission_ready=False,only_final_brev_validation_remains=False,root_review_required=True,geometry_conditional_pass=route['passed'],self_interarm_conditional_pass=robot['passed'],parked_left_fixture_pass=left['passed'],controller_synthetic_pass=controller['passed'],native_internal_geometry_physics_scorer_caps_budget_changed=False,robot_initial_mount_changed=True,new_physical_mount_plate=True,base_position_m=[.68,-.15,.75],base_yaw_rad=3.141592653589793,grasp_tcp_z_m=.826,native_budget_controls=3210,native_hold_s=3,synthetic_actions=controller['actions'],synthetic_control_wall_s=controller['wall_s'],geometry_wall_s=route['wall_s'],self_wall_s=robot['wall_s'],sampled_configurations=len(route['rows']),minimum_fixture_separation_m=min(v['worst'][0] for v in route['rows']),minimum_self_interarm_separation_m=robot['min_separation_m'],gpu_requests_submitted=0,simulation_starts=0,AppLauncher_calls=0,model_calls=0,brev_operations=0,simulator_import_inspection_incident=True,evidence_class='CPU conditional synthetic geometry/controller only; historical physical43767 failure retained, no transferred score',source_map=str(R/'consumed-sourceSHA.json'),gpu_request=str(R/'GPU_REQUEST.sh'),request_enabled=False,request_requires='ROOT path-clear review and fresh affected physical validation plan; no dispatch until reviewed',resources=dict(gpu=1,cpu=4,ram_gb=32,wall_minutes=30),remaining='Fresh actual SDK reset/contact/hydraulic channels and genuine whole-goal execution on moved robot; score channels remain disabled; real model/media/submission requirements later')
put(R/'result.json',x)
# Reuse the verified installed SDK closure without rehashing the entire SDK.
m=read(OLD/'consumed-sourceSHA.json')
for name in ('geometry-consumed-SHA256.json',):m.update(read(R/name))
logs=[json.loads(s) for s in (R/'guard.jsonl').read_text().splitlines()];m.update(logs[-1]['consumed_files'])
for p in R.rglob('*'):
 if p.is_file() and p.name not in ('consumed-sourceSHA.json','frozen-verification.json') and (p.suffix in ('.py','.sh','.usda','.json','.jsonl','.md')):m[str(p)]=sha(p)
for p in (OLD/'guard-regressions.json',OLD/'guard.py',OLD/'full-route.json'):
 if p.exists():m[str(p)]=sha(p)
put(R/'consumed-sourceSHA.json',m)
for n in ('REPORT.md','result.json'):(D/n).write_bytes((R/n).read_bytes())
put(D/'DELIVERY.json',dict(unit=x['unit'],owned_root=str(R),report=str(R/'REPORT.md'),result=str(R/'result.json'),frozen_candidate_sha256=read(R/'fingerprint.json')['sha256'],source_map=str(R/'consumed-sourceSHA.json'),gpu_request=str(R/'GPU_REQUEST.sh'),dispatch_enabled=False,status=x['status']))
put(R/'frozen-verification.json',dict(files=len(m),candidate_files=len(read(R/'candidate-SHA256.json')),source_map_sha256=sha(R/'consumed-sourceSHA.json'),strict_json=True))
print(json.dumps(x,indent=2))
