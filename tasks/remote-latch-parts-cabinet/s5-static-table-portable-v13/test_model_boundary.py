"""CPU boundary test: missing receipt fails before provider/simulator.
Pure task construction uses an explicitly synthetic in-memory receipt substitute;
no approved review file, runtime channel credit or physical proof is produced.
"""
import sys,os,runpy,json,time
from pathlib import Path
O=Path(__file__).resolve().parent;sys.path.insert(0,str(O));sys.path.insert(0,str(O/'candidate'))
blocked={'isaaclab','isaaclab_physx','isaacsim','isaaclab_arena','omni','carb','pxr','inspect_robots_dream'};events=[]
class Denied(BaseException):pass
def audit(event,args):
 if event=='import' and str(args[0]).split('.')[0] in blocked:events.append(str(args[0]));raise Denied(str(args[0]))
 if event in {'os.exec','subprocess.Popen','os.posix_spawn','os.system','os.fork','os.forkpty'}:raise Denied(event)
sys.addaudithook(audit)
t=time.monotonic();tests=[]
sys.argv=[str(O/'model_child.py'),'--out',str(O/'work/never-model'),'--review',str(O/'work/nonexistent-review.json')]
try:runpy.run_path(str(O/'model_child.py'),run_name='__main__')
except FileNotFoundError:tests.append('actual model child missing receipt fails before SDK/provider')
else:raise AssertionError('missing receipt accepted')
assert not events
# Conditional loader API check only; receipt validation is tested separately.
import protocol
protocol.receipt=lambda p:{'synthetic':True,'approved':False}
os.environ['S5_ROOT_REVIEW']='CPU-SYNTHETIC-IN-MEMORY'
from yam_arena.task_file import load
one=load(O/'model_task.py');two=load(O/'model_task.py')
assert one.module.OBJECTS and two.module.OBJECTS
assert one.module.__doc__==two.module.__doc__
import s5_rules as physics
physics.VALIDATED_CHANNELS=frozenset()
tests.append('actual installed TaskFile constructs pure YAM task twice under synthetic in-memory receipt; repeated load idempotent')
assert not events and not any(n.split('.')[0] in blocked for n in sys.modules)
r={'passed':True,'synthetic':True,'physical_proof':False,'tests':tests,'objects':len(two.module.OBJECTS),'wall_s':time.monotonic()-t,'events':events,'production_channels_promoted':[]}
(O/'checks/model-boundary.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r))
