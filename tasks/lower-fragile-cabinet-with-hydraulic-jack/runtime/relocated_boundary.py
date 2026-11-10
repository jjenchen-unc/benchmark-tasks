"""One relocation boundary check, preserving -I paths; no escape-suite repetition."""
from pathlib import Path
import sys,runpy,json,time
O=Path(__file__).resolve().parent;start=list(sys.path);denied=[];t=time.monotonic()
class Stop(BaseException):pass
def audit(event,args):
 if event=='open' and isinstance(args[0],str):
  p=Path(args[0]).absolute()
  if p.is_relative_to('/mnt/bum/jjenchen/robocurve') and not p.is_relative_to(O):denied.append(str(p));raise Stop('outside-package')
 if event=='import' and str(args[0]).split('.')[0] in {'isaaclab','isaaclab_physx','isaacsim','omni','carb','pxr'}:raise Stop('SDK:'+str(args[0]))
 if event in {'os.exec','subprocess.Popen','os.posix_spawn','os.system','os.fork','os.forkpty'}:raise Stop('escape')
sys.addaudithook(audit)
assert sys.flags.isolated and start==sys.path
sys.argv=[str(O/'runtime.py'),'replay','--out',str(O/'work/not-created')]
try:runpy.run_path(str(O/'runtime.py'),run_name='__main__')
except Stop as e:boundary=str(e)
else:raise RuntimeError('guard not reached')
r={'passed':boundary in ('SDK:isaaclab','SDK:isaaclab.app') and not denied,'boundary':boundary,'outside_package_reads':denied,'isolated':sys.flags.isolated,'wall_s':time.monotonic()-t,'physics_proof':False}
(O/'checks').mkdir(exist_ok=True);(O/'checks/relocation.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r))
if not r['passed']:raise SystemExit(1)
