"""Run with -I in a fresh interpreter. Persistent import and process audit guard."""
import sys,runpy,importlib.abc,json,os,subprocess,_posixsubprocess,posix,shutil
from pathlib import Path
class Boundary(BaseException):pass
class Forbidden(BaseException):pass
initial=list(sys.path)
assert sys.flags.isolated and not any(n.startswith(('omni','isaaclab.app','yam_arena')) for n in sys.modules)
events=[]
def audit(event,args):
 if event in ('os.exec','os.posix_spawn','os.spawn','os.fork','os.forkpty','os.system','subprocess.Popen','shutil.rmtree','os.remove','os.rmdir'):
  events.append(event);raise Forbidden(event)
 if event=='ctypes.dlsym' and any(t in str(args[1]) for t in ('exec','spawn','fork','system')):raise Forbidden(event)
sys.addaudithook(audit)
def forbidden(*args,**kwargs):raise Forbidden('process/cleanup API')
for module,names in ((os,('execv','execve','execvp','execvpe','execl','execlp','execlpe','execle','posix_spawn','posix_spawnp','spawnv','spawnve','spawnvp','spawnvpe','fork','forkpty','system')),(posix,('execv','execve','posix_spawn','posix_spawnp','fork','forkpty','system')),(_posixsubprocess,('fork_exec',)),(subprocess,('Popen',)),(shutil,('rmtree',))):
 for name in names:
  if hasattr(module,name):setattr(module,name,forbidden)
class Guard(importlib.abc.MetaPathFinder):
 def find_spec(self,fullname,path=None,target=None):
  if fullname=='isaaclab.app':raise Boundary(fullname)
  if fullname.startswith(('omni','isaacsim','isaaclab.sim','isaaclab_physx','isaaclab_arena')):raise Forbidden(fullname)
sys.meta_path.insert(0,Guard())
launcher=Path(sys.argv[1]).resolve();out=Path(sys.argv[2]).resolve();mode=sys.argv[3] if len(sys.argv)>3 else 'api';sys.argv=[str(launcher),'--out',str(out),'--legacy-rtx-override','--mode',mode]
try:runpy.run_path(str(launcher),run_name='__main__')
except Boundary as e:
 cold=json.loads((out/'cold-task.json').read_text());assert cold['constructed'] and len(cold['names'])==7
 assert not any(n.startswith(('omni','isaacsim','isaaclab.app')) for n in sys.modules)
 origins={name:str(Path(sys.modules[name].__file__).resolve()) for name in ('drawers_kit','drawers_math','motor_limits','strict_json','model_physics','pad_region','runtime_credit')}
 assert all(Path(p).parent==out/'source' for p in origins.values())
 assert sys.modules['drawers_physics'].Physics is sys.modules['model_physics'].ModelPhysics
 print(json.dumps({'helper_origins':origins,'status':'CPU_BOUNDARY_REACHED','boundary':str(e),'task':cold,'initial_sys_path':initial,'events':events,'artificial_path_injection':False}))
else:raise AssertionError('launcher never reached app boundary')
