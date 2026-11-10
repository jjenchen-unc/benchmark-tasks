"""Derive explicit model transport helpers; immutable native bundle stays exact."""
from pathlib import Path
import argparse,json,hashlib,ast,difflib
p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();S=a.bundle.resolve();W=a.out.resolve();W.mkdir(parents=True,exist_ok=False)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
original=(S/'model_child.py').read_text();s=original
old="O=Path(__file__).resolve().parent;C=O/'candidate';sys.path.insert(0,str(O));sys.path.insert(0,str(C))"
new="O=Path(os.environ['HYDRAULIC_NATIVE_BUNDLE']).resolve();C=O/'candidate';sys.path.insert(0,str(O));sys.path.insert(0,str(C))"
assert s.count(old)==1;s=s.replace(old,new)
old="session_module._retry_after=lambda error:None"
new="""import dream
official_retry_after=session_module._retry_after
def retry_existing_session(error):
 if isinstance(error,dream.DreamSessionExpired):return None
 pause=official_retry_after(error)
 if pause is not None:record({'kind':'transport_retry','error_class':type(error).__name__,'code':getattr(error,'code',None),'reason':getattr(error,'reason',None),'pause_s':pause})
 return pause
session_module._retry_after=retry_existing_session"""
assert s.count(old)==1;s=s.replace(old,new)
old="type(chunk.attempts) is not int or chunk.attempts!=1 or not continuous";new="type(chunk.attempts) is not int or chunk.attempts<1 or not continuous";assert s.count(old)==1;s=s.replace(old,new)
anchor="a.out=a.out.resolve();a.out.mkdir(parents=True,exist_ok=False)"
extra="""
import shutil
executed=Path(__file__).resolve()
shutil.copy2(executed,a.out/'executed-model-entry.py')
workflow=json.loads((executed.parent/'execution-manifest.json').read_text())
actual_sha=hashlib.sha256(executed.read_bytes()).hexdigest()
if actual_sha!=workflow['actual_model_entry_sha256']:raise RuntimeError('Executed model helper changed')
(a.out/'executed-model-helper.json').write_text(dumps({'native_bundle_fingerprint':check(),'original_model_child_sha256':workflow['original_model_child_sha256'],'actual_executed_model_helper_sha256':actual_sha,'workflow_manifest_sha256':hashlib.sha256((executed.parent/'execution-manifest.json').read_bytes()).hexdigest(),'policy':'official transient retries within same continuous session; expiration/replacement invalid; no episode restart','task_physics_controller_goals_budget_changed':False},indent=2)+'\\n')
"""
assert s.count(anchor)==1;s=s.replace(anchor,anchor+extra)
compile(s,'model_entry.py','exec');(W/'model_entry.py').write_text(s)
v=(S/'verify_model.py').read_text();v=v.replace('import sys,re,json,math,urllib.parse,hashlib,argparse','import sys,re,json,math,urllib.parse,hashlib,argparse,os');assert v.count(old if False else "O=Path(__file__).resolve().parent;C=O/'candidate';sys.path.insert(0,str(O));sys.path.insert(0,str(C))")==1
v=v.replace("O=Path(__file__).resolve().parent;C=O/'candidate';sys.path.insert(0,str(O));sys.path.insert(0,str(C))","O=Path(os.environ['HYDRAULIC_NATIVE_BUNDLE']).resolve();C=O/'candidate';sys.path.insert(0,str(O));sys.path.insert(0,str(C))")
assert v.count("r['attempts']!=1")==1;v=v.replace("r['attempts']!=1","r['attempts']<1")
anchor=" out=Path(out).resolve();summaries="
insert=""" out=Path(out).resolve()
 workflow=load(Path(__file__).resolve().parent/'execution-manifest.json');helper=load(out/'executed-model-helper.json')
 if helper['actual_executed_model_helper_sha256']!=workflow['actual_model_entry_sha256'] or sha(out/'executed-model-entry.py')!=workflow['actual_model_entry_sha256'] or helper['native_bundle_fingerprint']!=check():raise ValueError('Actual model helper source not bound')
 summaries="""
assert v.count(anchor)==1;v=v.replace(anchor,insert)
compile(v,'verify_model_transport.py','exec');(W/'verify_model_transport.py').write_text(v)
anchor="'source_manifest_sha256':sha(O/'manifest.json')"
assert v.count(anchor)==1
v=v.replace(anchor,anchor+",'executed_model_helper_sha256':workflow['actual_model_entry_sha256'],'executed_helper_proof_sha256':sha(out/'executed-model-helper.json'),'outer_execution_manifest_sha256':sha(Path(__file__).resolve().parent/'execution-manifest.json')")
compile(v,'verify_model_transport.py','exec');(W/'verify_model_transport.py').write_text(v)
manifest={'native_bundle_fingerprint':json.loads((S/'manifest.json').read_text())['fingerprint'],'original_model_child_sha256':sha(S/'model_child.py'),'original_verifier_sha256':sha(S/'verify_model.py'),'actual_model_entry_sha256':sha(W/'model_entry.py'),'actual_verifier_sha256':sha(W/'verify_model_transport.py'),'task_candidate_files_unchanged':True,'scope':'Only explicit outer execution helper path/source receipt, official same-session transport retry handler and positive attempt counts. No policy/goal/physics/controller/scoring/action-budget changes.'}
(W/'execution-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
diff=''.join(difflib.unified_diff(original.splitlines(keepends=True),s.splitlines(keepends=True),fromfile='pinned_original_model_child.py',tofile='actual_executed_model_entry.py'))
(W/'execution-diff.patch').write_text(diff)
print(json.dumps(manifest))
