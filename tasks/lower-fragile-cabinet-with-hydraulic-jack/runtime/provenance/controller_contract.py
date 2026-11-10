"""Explicitly SYNTHETIC feedback; executes actual Driver, NEVER physical evidence."""
import sys,json,types,math,time
from pathlib import Path
R=Path(__file__).resolve().parent;sys.path.insert(0,str(R/'candidate'))
import numpy as np,mount
F=json.loads((R/'candidate/fixture.json').read_text());angle=np.zeros((1,1));engine=types.SimpleNamespace(jacks={'S':types.SimpleNamespace(data=types.SimpleNamespace(joint_pos=angle),joint_names=['valve_S'])},last={'on_rails':False,'vz':-.01})
sys.modules['kit']=types.SimpleNamespace(F=F,ENGINE=engine,arr=np.asarray)
sys.modules['yam_arena.runtime']=types.SimpleNamespace(tcp_world=lambda robot:mount.tcp(robot.q[7:13])[0])
from reference import Driver
class Emb:
 def __init__(self):
  self.q=np.array([0,1,1,-.6,0,0,1]*2,dtype=float);self._base=types.SimpleNamespace(scene={'right_robot':self});self._hj_pre_action=lambda *a:None;self.loaded_steps=0
 def step(self,action):
  self.q=np.asarray(action.data,float).copy();_,rot=mount.tcp(self.q[7:13]);y=math.atan2(-rot[0,1],-rot[0,0]);angle[0,0]=(y-math.pi)%(2*math.pi)
  if angle[0,0]>3.:angle[0,0]=0.
  if angle[0,0]>math.radians(F['parameters']['phi0_S']):self.loaded_steps+=1
  engine.last['on_rails']=self.loaded_steps>=40
  return types.SimpleNamespace(observation=types.SimpleNamespace(state={'joint_pos':self.q.copy()}),terminated=False)
e=Emb();rows=[];start=time.monotonic();d=Driver(e,types.SimpleNamespace(state={'joint_pos':e.q.copy()}),lambda r,q:rows.append(q.tolist()))
try:d.run();error=None
except Exception as ex:error=type(ex).__name__+': '+str(ex)
a=np.array([([0,1,1,-.6,0,0,1]*2)]+rows);out=dict(passed=error is None,error=error,actions=len(rows),max_command_increment=float(abs(np.diff(a,axis=0)).max()) if rows else 0,wall_s=time.monotonic()-start,evidence='SYNTHETIC feedback only; valve/load state supplied by test double, no physical retention/flow/contact/score proof',actions_14d=rows)
(R/'controller-contract.json').write_text(json.dumps(out,indent=2,allow_nan=False)+'\n');print(json.dumps({k:v for k,v in out.items() if k!='actions_14d'}))
