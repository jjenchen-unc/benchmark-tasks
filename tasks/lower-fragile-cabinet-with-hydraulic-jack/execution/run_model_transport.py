"""Reproduce actual model from declared outer helpers and immutable native task."""
from pathlib import Path
import argparse,sys,os,subprocess,json,hashlib,time,signal,datetime
p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--workflow',type=Path,required=True);p.add_argument('--review',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--preflight-only',action='store_true');a=p.parse_args();S=a.bundle.resolve();W=a.workflow.resolve();O=a.out.resolve();sys.path.insert(0,str(S))
from protocol import check,receipt
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();m=json.loads((W/'execution-manifest.json').read_text());assert check()==m['native_bundle_fingerprint'];receipt(a.review)
for name,key in [('model_entry.py','actual_model_entry_sha256'),('verify_model_transport.py','actual_verifier_sha256')]:assert sha(W/name)==m[key]
assert sha(S/'model_child.py')==m['original_model_child_sha256'] and sha(S/'verify_model.py')==m['original_verifier_sha256']
assert not O.exists() and not O.is_relative_to(S)
plan={'at':datetime.datetime.now(datetime.timezone.utc).isoformat(),'original_native_bundle_fingerprint':check(),'executed_helpers':m,'review_sha256':sha(a.review),'out':str(O),'native_action_budget':3210,'policy':'dreamlabs/dream-1-beta','model_calls':0 if a.preflight_only else 'actualtrial_pending','preflight_only':a.preflight_only}
if a.preflight_only:print(json.dumps(plan));raise SystemExit(0)
O.with_suffix('.execution-plan.json').write_text(json.dumps(plan,indent=2)+'\n');env=dict(os.environ,HYDRAULIC_NATIVE_BUNDLE=str(S),OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1')
cmd=[sys.executable,'-I','-B',str(W/'model_entry.py'),'--review',str(a.review.resolve()),'--out',str(O)]
q=subprocess.Popen(cmd,env=env,start_new_session=True)
phase=O.with_suffix('.phase.json')
phase.write_text(json.dumps({'at':datetime.datetime.now(datetime.timezone.utc).isoformat(),'status':'running','pid':q.pid,'actual_helper_sha256':m['actual_model_entry_sha256'],'action_budget':3210},indent=2)+'\n')
try:rc=q.wait(timeout=3600)
except subprocess.TimeoutExpired:
 os.killpg(q.pid,signal.SIGTERM)
 try:q.wait(timeout=20)
 except subprocess.TimeoutExpired:os.killpg(q.pid,signal.SIGKILL);q.wait()
 phase.write_text(json.dumps({'status':'task_timeout_preserved','returncode':q.returncode,'VM_stop_sent':False},indent=2)+'\n');raise SystemExit(1)
assert check()==m['native_bundle_fingerprint']
phase.write_text(json.dumps({'at':datetime.datetime.now(datetime.timezone.utc).isoformat(),'status':'model_process_ended_needs_verifier','returncode':rc},indent=2)+'\n')
if rc:raise SystemExit(rc)
r=subprocess.run([sys.executable,'-I','-B',str(W/'verify_model_transport.py'),str(O)],env=env)
phase.write_text(json.dumps({'at':datetime.datetime.now(datetime.timezone.utc).isoformat(),'status':'valid_model_requires_blind_media_review' if r.returncode==0 else 'model_verification_failed_preserved','returncode':r.returncode},indent=2)+'\n');raise SystemExit(r.returncode)
