"""Actual pure TaskFile objects and native patched reset on a CPU fake backend."""
from pathlib import Path
import sys,types,os
O=Path(__file__).resolve().parent;sys.path.insert(0,str(O));sys.path.insert(0,str(O/'candidate'))
from strict_json import dumps
from yam_arena.task_file import load
import s1_kit as kit
first=load(O/'candidate/task.py');second=load(O/'candidate/task.py');checks={'doubleload_same_type':type(first.objects[0]) is type(second.objects[0]),'fixed_case42':kit.FIXTURE['native_seed']==42}
class Robot:pass
base=types.SimpleNamespace(scene={s+'_robot':Robot() for s in ['left','right']})
class Engine:
 def __init__(self):self.base=base;self.settling=False;self.begun=0
 def begin(self):self.begun+=1
engine=Engine();kit.ENGINE=engine
class Fake:
 def __init__(self):self.task=second;self.info=types.SimpleNamespace(control_hz=20.);self.object_jitter=0;self._base=base;self.seeds=[]
 def _ensure_sim(self,arg):pass
 def reset(self,*args,**kwargs):self.seeds.append(kwargs['seed']);return kwargs['seed']
 def step(self,action):pass
 def _task_step(self):pass
 def close(self):pass
kit.patch_embodiment(types.SimpleNamespace(YamArenaEmbodiment=Fake));f=Fake()
for seed in [42,3867854786,927617]:checks['derived_seed_'+str(seed)]=f.reset(None,seed=seed)==seed
checks['fixture_unchanged']=kit.FIXTURE['native_seed']==42 and kit.TARGET=='SH-T1'
f.object_jitter=.01
try:f.reset(None,seed=42)
except ValueError:checks['jitter_refused']=True
else:checks['jitter_refused']=False
f.object_jitter=0;f.info.control_hz=10.
try:f.reset(None,seed=42)
except ValueError:checks['wrong_hz_refused']=True
else:checks['wrong_hz_refused']=False
checks['no_sdk_initialized']=not any(n.startswith(('isaaclab.app','isaacsim','omni','carb','pxr')) for n in sys.modules)
r=dict(passed=all(checks.values()),checks=checks,kind='CPU_FAKE_BACKEND_API_CONTRACT_ONLY',physical_evidence=False)
(O/'checks/entry.json').write_text(dumps(r,indent=2)+'\n');print(dumps(r))
if not r['passed']:raise SystemExit(1)
