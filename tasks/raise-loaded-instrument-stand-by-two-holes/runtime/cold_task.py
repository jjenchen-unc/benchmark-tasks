"""Load the copied frozen task file twice before any App/Kit import (CPU only)."""
import hashlib,json,sys
from pathlib import Path
R=Path(__file__).resolve().parent.parent;sys.path.insert(0,str(R/'source'));sys.path.insert(0,str(R/'runtime'))
import guard
fp=guard.verify_package()
from yam_arena.task_file import load
loads=[]
for _ in range(2):
 t=load(R/'source/task.py');loads.append(sorted(o.name for o in t.objects))
app=[m for m in ('isaaclab.app','omni.kit','yam_arena.inspect_embodiment','yam_arena.scene') if m in sys.modules]
print(json.dumps(dict(source_fingerprint=fp,loads=2,identical=loads[0]==loads[1],objects=loads[0],app_modules_loaded=app,task_sha256=hashlib.sha256((R/'source/task.py').read_bytes()).hexdigest())))
