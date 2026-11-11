"""Fail closed before any App import: package manifest, frozen source, Root receipt+evidence, SDK origins, TaskFile twice."""
import hashlib,json,sys
from pathlib import Path
R=Path(__file__).resolve().parent.parent;sys.path.insert(0,str(R/'source'));sys.path.insert(0,str(R/'runtime'))
import guard
STATION_SHA256='0b24ccab52cabf8e14279b599c2173bed25f16c1a3d9656d8d739886b27fc0c7'
def run(credit=True):
 for m in ('isaaclab.app','omni.kit','yam_arena.inspect_embodiment','yam_arena.scene'):
  if m in sys.modules:raise guard.GuardViolation('App imported before preflight: '+m)
 fp=guard.verify_package()
 from sdk_pins import verify
 sdk_source_pins=verify()
 if hashlib.sha256((R/'runtime/gripper_saturation.py').read_bytes()).hexdigest()!=guard.ADAPTER_SHA256:raise guard.GuardViolation('gripper adapter changed')
 if credit:guard.enable_credit(fp)
 import yam_arena,inspect_robots,importlib.util
 st=importlib.util.find_spec('inspect_robots_dream._station').origin
 if hashlib.sha256(Path(st).read_bytes()).hexdigest()!=STATION_SHA256:raise guard.GuardViolation('SDK station decoder changed')
 from yam_arena.task_file import load
 loads=[sorted(o.name for o in load(R/'source/task.py').objects) for _ in range(2)]
 if loads[0]!=loads[1]:raise guard.GuardViolation('TaskFile loads differ')
 for m in ('yam_arena.inspect_embodiment','yam_arena.scene','isaaclab.app'):
  if m in sys.modules:raise guard.GuardViolation('cold gate broken: '+m)
 import contract
 return dict(sdk_source_pins_verified=sdk_source_pins,source_fingerprint=fp,credit_enabled=contract.VALIDATED==contract.REQUIRED,task_loads=2,objects=loads[0],origins=dict(yam_arena=yam_arena.__file__,inspect_robots=inspect_robots.__file__,station=st),station_sha256=STATION_SHA256)
