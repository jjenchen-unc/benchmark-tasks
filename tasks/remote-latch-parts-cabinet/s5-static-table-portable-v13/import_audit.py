"""Persistent audit guard: no AppLauncher/Simulation import or process escape.
Python children are rewrapped, including isolated (-I) launches. Other process
creation and exec are denied; hook cleanup does not remove sys.addaudithook.
"""
import sys,os,json,runpy,subprocess,hashlib
from pathlib import Path
INITIAL=list(sys.path)
SELF=str(Path(__file__).resolve());LOG=sys.argv[1];target=sys.argv[2];args=sys.argv[3:]
def record(event,**kw):
 fd=os.open(LOG,os.O_WRONLY|os.O_CREAT|os.O_APPEND,0o600)
 os.write(fd,(json.dumps(dict(event=event,pid=os.getpid(),**kw))+'\n').encode());os.close(fd)
def audit(event,a):
 if event=='s5.task_constructed':record('task_constructed',path=a[0],objects=a[1])
 if event=='exec' and any(n in {'isaaclab','isaaclab_physx','isaacsim','omni','carb','pxr'} for n in a[0].co_filename.replace('\\','/').split('/')):record('blocked_sdk_exec');os._exit(86)
 if event=='import' and a[0].split('.')[0] in {'isaaclab','isaaclab_physx','isaaclab_arena','isaacsim','omni','carb','pxr'}:
  frames=[];f=sys._getframe()
  while f:
   task=f.f_locals.get('task')
   if task is not None and hasattr(task,'objects'):frames.append(dict(name=task.name,objects=len(task.objects)))
   f=f.f_back
  consumed={str(Path(m.__file__).resolve()):hashlib.sha256(Path(m.__file__).read_bytes()).hexdigest() for m in list(sys.modules.values()) if getattr(m,'__file__',None) and Path(m.__file__).is_file()}
  record('sdk_boundary',consumed_source_sha256=consumed,initial_sys_path=INITIAL,constructed_tasks=frames,module=a[0],telemetry_path=getattr(sys.modules.get('telemetry_json'),'__file__',None),numeric_path=getattr(sys.modules.get('numeric'),'__file__',None),isolated=sys.flags.isolated,target=target);os._exit(86)
 if event in {'os.exec','os.posix_spawn','os.system','os.fork','os.forkpty'}:
  record('blocked_process_escape',kind=event);os._exit(87)
 if event=='subprocess.Popen':
  cmd=a[1]
  valid=isinstance(cmd,(list,tuple)) and len(cmd)>1 and Path(str(cmd[0])).resolve()==Path(sys.executable).resolve()
  rest=list(cmd[1:]) if valid else []
  while rest and rest[0] in ['-I','-B','-u']:rest.pop(0)
  valid=valid and bool(rest) and rest[0]==SELF
  if not valid:
   record('blocked_process_escape',kind=event);os._exit(87)
sys.addaudithook(audit)
original=subprocess.Popen
class GuardedPopen(original):
 def __init__(self,cmd,*a,**kw):
  if isinstance(cmd,(list,tuple)) and Path(str(cmd[0])).name.startswith('python'):
   c=list(cmd);flags=[]
   while len(c)>1 and c[1] in ['-I','-B','-u']:flags.append(c.pop(1))
   if len(c)<2 or c[1].startswith('-'):
    record('blocked_process_escape',kind='unsupported_python_command');os._exit(87)
   cmd=[c[0],*flags,SELF,LOG,*c[1:]]
  super().__init__(cmd,*a,**kw)
subprocess.Popen=GuardedPopen
record('guard_installed',target=target)
if not sys.flags.isolated:
 sys.path[0]=str(Path(target).resolve().parent)
sys.argv=[target,*args]
runpy.run_path(target,run_name='__main__')
