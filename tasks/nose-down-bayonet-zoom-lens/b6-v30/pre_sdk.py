"""Clean -I persistent guard of the actual entry. No guard sys.path injection."""
from pathlib import Path
import sys,os,runpy,json,hashlib,time,subprocess,_posixsubprocess,importlib.util
O=Path(__file__).resolve().parent;initial=list(sys.path);mode=sys.argv[1];proof=Path(sys.argv[2]);events=[];constructs=[];start=time.monotonic()
blocked={'isaaclab','isaaclab_physx','isaacsim','isaaclab_arena','omni','carb','pxr','inspect_robots_dream'}
class Denied(BaseException):pass
def audit(event,args):
 if event in ('b6.task_constructed','b6.model_task_constructed'):constructs.append([event,*args])
 if event=='import' and str(args[0]).split('.')[0] in blocked:events.append(str(args[0]));raise Denied('SDK:'+str(args[0]))
 if event=='exec' and any(x in blocked for x in args[0].co_filename.replace('\\','/').split('/')):raise Denied('SDK_EXEC')
 if event in {'os.exec','os.execv','os.execve','os.fork','os.forkpty','os.posix_spawn','os.posix_spawnp','subprocess.Popen','os.system','os.spawn','shutil.rmtree'}:raise Denied('ESCAPE:'+event)
sys.addaudithook(audit)
def raw(*a,**kw):raise Denied('ESCAPE:raw_spawn')
_posixsubprocess.fork_exec=raw;subprocess._fork_exec=raw
synthetic=False;boundary=None;missing=False;pre_entry_path=list(sys.path)
try:
 if mode.startswith('escape-'):
  name=mode[7:]
  if name=='execv':os.execv('/bin/true',['true'])
  elif name=='execve':os.execve('/bin/true',['true'],{})
  elif name=='Popen':subprocess.Popen(['/bin/true'])
  elif name=='raw':_posixsubprocess.fork_exec()
  elif name=='cleanup':__import__('shutil').rmtree(O/'must-never-delete',ignore_errors=True)
  else:raise ValueError(name)
 elif mode in ('reference','replay','setup'):
  sys.argv=[str(O/'runtime.py'),mode,'--out',str(O/'checks/never-created'),'--rtx-driver-check-override']
  if mode=='replay':sys.argv+=['--actions',str(O/'checks/synthetic-actions.json')]
  runpy.run_path(str(O/'runtime.py'),run_name='__main__')
 elif mode in ('model-missing','model-synthetic'):
  review=O/'checks/nonexistent-review.json'
  if mode=='model-synthetic':
   synthetic=True
   spec=importlib.util.spec_from_file_location('protocol',O/'protocol.py');module=importlib.util.module_from_spec(spec);sys.modules['protocol']=module;spec.loader.exec_module(module)
   module.receipt=lambda p:{'synthetic':True,'approved':False}
   review=O/'checks/synthetic-actions.json' # Only hashable synthetic marker; no receipt file produced.
  sys.argv=[str(O/'model_child.py'),'--out',str(O/'checks/never-model'),'--review',str(review)]
  try:runpy.run_path(str(O/'model_child.py'),run_name='__main__')
  except FileNotFoundError:
   if mode!='model-missing':raise
   missing=True
 else:raise ValueError(mode)
except Denied as e:boundary=str(e)
checks=dict(isolated=sys.flags.isolated==1,guard_path_preserved=initial==pre_entry_path,no_sim_loaded=not any(n.split('.')[0] in blocked for n in sys.modules),persistent=True)
if mode.startswith('escape-'):checks['boundary']=bool(boundary and boundary.startswith('ESCAPE:'))
elif mode=='model-missing':checks['missing_receipt_failclosed']=missing and not events and not constructs
else:checks.update(boundary=boundary in ('SDK:isaaclab','SDK:isaaclab.app'),task_constructed=bool(constructs),helper_origin=Path(sys.modules['kit'].__file__).resolve().parent==O/'candidate')
consumed={str(Path(m.__file__).resolve()):hashlib.sha256(Path(m.__file__).read_bytes()).hexdigest() for m in list(sys.modules.values()) if getattr(m,'__file__',None) and Path(m.__file__).is_file()}
r=dict(passed=all(checks.values()),mode=mode,checks=checks,boundary=boundary,events=events,constructs=constructs,initial_sys_path=initial,guard_injected_paths=False,synthetic=synthetic,physical_proof=False,consumed_source_sha256=consumed,wall_s=time.monotonic()-start)
proof.write_text(json.dumps(r,indent=2,allow_nan=False)+'\n');print(json.dumps({k:v for k,v in r.items() if k!='consumed_source_sha256'}))
if not r['passed']:raise SystemExit(1)
