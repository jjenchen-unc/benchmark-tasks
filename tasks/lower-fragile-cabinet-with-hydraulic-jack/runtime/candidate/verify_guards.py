"""Pure adapter contract tests; fake backend sentinel proves rejection precedes physics."""
from pathlib import Path
import sys,types
import numpy as np
from yam_arena.task_file import load
P=Path(__file__).resolve().parent;task=load(P/'task.py');import kit
class Fake:
 def reset(self,*a,**k):raise AssertionError('unexpected reset')
 def step(self,action):raise AssertionError('physics reached')
 def _task_step(self):raise AssertionError('physics reached')
module=types.SimpleNamespace(YamArenaEmbodiment=Fake);kit.patch_embodiment(module);e=Fake();e.task=task
checks={}
kit.ENGINE=types.SimpleNamespace(done=True)
try:e.step(types.SimpleNamespace(data=np.zeros(14)))
except RuntimeError:checks['terminated_step_rejected_before_physics']=True
kit.ENGINE=types.SimpleNamespace(done=False)
for name,data in [('wrong_dim',np.zeros(13)),('nonfinite',np.full(14,np.nan))]:
 try:e.step(types.SimpleNamespace(data=data))
 except ValueError:checks[name+'_rejected_before_physics']=True
checks['only_cabinet_free']=task.free_objects==('cabinet',)
checks['no_exact42_seed_rejection']='seed=42' not in (P/'kit.py').read_text()
print(__import__('json').dumps({'kind':'pure adapter guard test; not runtime proof','checks':checks,'all_pass':len(checks)==5 and all(checks.values())},indent=2))
assert len(checks)==5 and all(checks.values())
