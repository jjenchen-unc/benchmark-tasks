"""CPU tests of the exact derived functions; no simulator/provider/network."""
from pathlib import Path
import ast,argparse,types,time,hashlib,json
p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--workflow',type=Path,required=True);a=p.parse_args();S=a.bundle;W=a.workflow
s=(W/'model_entry.py').read_text();tree=ast.parse(s);original=ast.parse((S/'model_child.py').read_text())
def find(tree,name):return next(n for n in ast.walk(tree) if isinstance(n,(ast.FunctionDef,ast.AsyncFunctionDef)) and n.name==name)
def func(name,ns):
 node=find(tree,name);exec(compile(ast.Module(body=[node],type_ignores=[]),str(W/'model_entry.py'),'exec'),ns);return ns[name]
checks={}
old_argv=[n for n in ast.walk(original) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Attribute) and isinstance(t.value,ast.Name) and t.value.id=='sys' and t.attr=='argv' for t in n.targets)]
new_argv=[n for n in ast.walk(tree) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Attribute) and isinstance(t.value,ast.Name) and t.value.id=='sys' and t.attr=='argv' for t in n.targets)]
assert [ast.dump(n,include_attributes=False) for n in old_argv]==[ast.dump(n,include_attributes=False) for n in new_argv];checks['exact_native_instruction_policy_budget_scene_argv']=True
for name in ['checked_reset','checked_step']:
 assert ast.dump(find(tree,name),include_attributes=False)==ast.dump(find(original,name),include_attributes=False)
 checks[name+'_unchanged_AST']=True
class Expired(Exception):pass
class Transient(Exception):pass
rows=[];calls=[]
ns={'dream':types.SimpleNamespace(DreamSessionExpired=Expired),'official_retry_after':lambda e:calls.append(e) or (1. if isinstance(e,Transient) else None),'record':rows.append}
retry=func('retry_existing_session',ns)
assert retry(Transient())==1. and len(rows)==1 and rows[-1]['kind']=='transport_retry';checks['transient_uses_official_handler_and_records']=True
calls.clear();assert retry(Expired()) is None and not calls;checks['expired_session_not_recovered']=True
assert retry(ValueError()) is None and len(rows)==1;checks['nonretryable_error_not_recovered']=True
def run_chunk(attempts,session='same',previous=None):
 rows=[];chunk=types.SimpleNamespace(session_id=session,attempts=attempts);ns={'time':time,'hashlib':hashlib,'record':rows.append,'act':lambda *x,**k:chunk};wrapped=func('wrapped_act',ns);obj=types.SimpleNamespace(_hydraulic_state={'session':previous,'count':0,'finished':None})
 return wrapped(obj),rows
old=hashlib.sha256(b'same').hexdigest();c,r=run_chunk(2,previous=old);assert c.attempts==2 and r[-1]['continuous'] is True;checks['two_attempts_same_session_accepted_no_episode_restart']=True
for key,attempts,session,prev in [('changed_session_rejected',1,'new',old),('missing_session_rejected',1,None,None),('boolean_attempt_rejected',True,'same',old),('zero_attempt_rejected',0,'same',old)]:
 try:run_chunk(attempts,session,prev)
 except RuntimeError:checks[key]=True
 else:raise AssertionError(key)
manifest=json.loads((W/'execution-manifest.json').read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert sha(W/'model_entry.py')==manifest['actual_model_entry_sha256'] and sha(W/'verify_model_transport.py')==manifest['actual_verifier_sha256'];checks['actual_executable_helper_sha_declared']=True
assert sha(S/'model_child.py')==manifest['original_model_child_sha256'];checks['original_bundle_model_helper_retained']=True
report={'passed':all(checks.values()),'checks':checks,'test_count':len(checks),'simulation_starts':0,'model_calls':0};(W/'CPU-transport-tests.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
