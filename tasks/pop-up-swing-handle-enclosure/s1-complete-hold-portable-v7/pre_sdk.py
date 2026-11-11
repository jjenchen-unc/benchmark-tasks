"""Persistent guard: preserve initial -I sys.path; actual launcher owns bootstrap."""
import sys,os,runpy,json,hashlib
from pathlib import Path
O=Path(__file__).resolve().parent;initial=list(sys.path);constructed=[];events=[]
class Denied(BaseException):pass
def audit(event,args):
 if event in {'s1.task_constructed','s1.model_task_constructed'}:constructed.append(str(args[0]))
 if event=='import' and (str(args[0]) in {'isaaclab','isaaclab.app'} or str(args[0]).startswith(('isaaclab.app.','isaaclab.sim','isaaclab.assets','isaacsim','omni','carb','pxr','isaaclab_physx'))):raise Denied('SDK:'+str(args[0]))
 if event=='exec' and any(n in {'isaaclab','isaaclab_arena','isaacsim','omni','carb','pxr','isaaclab_physx'} for n in args[0].co_filename.replace('\\','/').split('/')):raise Denied('SDK_EXEC')
 if event in {'os.exec','subprocess.Popen','os.posix_spawn','os.system','os.fork','os.forkpty'}:raise Denied('ESCAPE:'+event)
 if event=='ctypes.dlsym' and any(x in str(args[1]).lower() for x in ['exec','spawn','fork','system','syscall','popen','clone']):raise Denied('CTYPES_ESCAPE')
sys.addaudithook(audit)
import _posixsubprocess
_posixsubprocess.fork_exec=lambda *a,**k:(_ for _ in ()).throw(Denied('RAW_SPAWN'))
# Existing v5 eight escape proofs retained; these changed-entry targeted probes
# prove guard still applies to process replacement, cleanup and raw spawn.
for name,fn in [('execv',lambda:os.execv(sys.executable,[sys.executable])),('execve',lambda:os.execve(sys.executable,[sys.executable],{})),('Popen',lambda:__import__('subprocess').Popen([sys.executable])),('raw_spawn',lambda:_posixsubprocess.fork_exec()),('cleanup',lambda:__import__('isaaclab.app'))]:
 try:fn()
 except Denied:events.append(name)
 else:raise RuntimeError('escape '+name)
assert sys.flags.isolated and sys.path==initial
mode=sys.argv[1] if len(sys.argv)>1 else 'replay'
target=O/'runtime.py';args=['replay','--out',str(O/'work/guard-never-created')]
if mode.startswith('model'):target=O/'model_child.py';args=['--out',str(O/'work/guard-model-never-created'),'--review',sys.argv[2] if len(sys.argv)>2 else str(O/'work/absent-review.json')]
sys.argv=[str(target),*args];boundary=None
try:runpy.run_path(str(target),run_name='__main__')
except Denied as e:boundary=str(e)
except FileNotFoundError:
 if mode!='model':raise
 boundary='REVIEW_REFUSED'
modules={str(Path(m.__file__).resolve()):hashlib.sha256(Path(m.__file__).read_bytes()).hexdigest() for m in list(sys.modules.values()) if getattr(m,'__file__',None) and Path(m.__file__).is_file()}
checks=dict(isolated_initial_path_preserved=True,guard_persistent=True,entry_expected_boundary=(boundary in ['SDK:isaaclab.app','SDK:isaaclab'] and len(constructed)==1) if mode in ['replay','model-approved'] else boundary=='REVIEW_REFUSED',no_simulator_loaded=not any(n.startswith(('isaaclab.app','isaaclab.sim','isaacsim','omni','carb','pxr')) for n in sys.modules),no_artificial_target_injection=str(O) not in initial and str(O/'candidate') not in initial)
r=dict(passed=all(checks.values()),checks=checks,boundary=boundary,constructed=constructed,escape_probes=events,initial_sys_path=initial,consumed_source_sha256=modules,physics_proof=False)
(O/'checks').mkdir(exist_ok=True);(O/('checks/pre-sdk-'+mode+'.json')).write_text(json.dumps(r,allow_nan=False,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items() if k!='consumed_source_sha256'}))
if not r['passed']:raise SystemExit(1)
