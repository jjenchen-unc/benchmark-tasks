"""Persistent isolated guard of actual runtime, preserving wrapper -I sys.path."""
import sys,os,runpy,json,hashlib,time,subprocess
from pathlib import Path
O=Path(__file__).resolve().parent;initial=list(sys.path);events=[];checks={};blocked={'isaaclab','isaaclab_physx','isaacsim','isaaclab_arena','omni','carb','pxr'}
class Denied(BaseException):pass
def audit(event,args):
 if event=='exec' and any(n in blocked for n in args[0].co_filename.replace('\\','/').split('/')):raise Denied('SDK_EXEC')
 if event=='import' and str(args[0]).split('.')[0] in blocked:events.append(str(args[0]));raise Denied('SDK:'+str(args[0]))
 if event in {'os.exec','subprocess.Popen','os.posix_spawn','os.system','os.fork','os.forkpty'}:raise Denied('ESCAPE:'+event)
sys.addaudithook(audit)
for name,fn in [('execv',lambda:os.execv(sys.executable,[sys.executable])),('execve',lambda:os.execve(sys.executable,[sys.executable],{})),('Popen',lambda:subprocess.Popen([sys.executable])),('spawn',lambda:os.posix_spawn(sys.executable,[sys.executable],{})),('cleanup',lambda:__import__('isaaclab')),('raw_fork',lambda:os.fork()),('loader_exec',lambda:exec(compile('pass','/synthetic/isaaclab/x.py','exec')))]:
 try:fn()
 except Denied:checks[name]=True
 else:raise RuntimeError('escape '+name)
checks['isolated_path_preserved']=bool(sys.flags.isolated) and initial==sys.path
import yam_arena.task_file as tf
constructed=[];old_load=tf.load
def tracked_load(*args,**kwargs):
 result=old_load(*args,**kwargs);constructed.append(len(result.objects));return result
tf.load=tracked_load
sys.argv=[str(O/'runtime.py'),'replay','--out',str(O/'checks'/('never-created-'+str(time.time_ns())))]
boundary=None
try:runpy.run_path(str(O/'runtime.py'),run_name='__main__')
except Denied as e:boundary=str(e)
checks['boundary']=boundary in ('SDK:isaaclab','SDK:isaaclab.app')
checks['task_constructed']=bool(constructed)
checks['object_count']=constructed==[14]
checks['native_helper_origin']=Path(sys.modules['kit'].__file__).resolve().parent==O/'candidate'
checks['no_sim_loaded']=not any(n.split('.')[0] in blocked for n in sys.modules)
consumed={str(Path(m.__file__).resolve()):hashlib.sha256(Path(m.__file__).read_bytes()).hexdigest() for m in list(sys.modules.values()) if getattr(m,'__file__',None) and Path(m.__file__).is_file()}
r={'passed':all(checks.values()),'checks':checks,'boundary':boundary,'events':events,'initial_sys_path':initial,'consumed_source_sha256':consumed,'physics_proof':False}
(O/'checks').mkdir(exist_ok=True);(O/'checks/pre-sdk.json').write_text(json.dumps(r,allow_nan=False,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items() if k!='consumed_source_sha256'}))
if not r['passed']:raise SystemExit(1)
