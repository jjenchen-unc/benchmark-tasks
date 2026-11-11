import json,sys,tempfile,unittest
from dataclasses import dataclass,field
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'runtime'))
import guard as G

@dataclass
class Obs:
 images:dict=field(default_factory=lambda:{'top_cam':b'rgb'});state:dict=field(default_factory=lambda:{'joint_pos':[0.]*14})
@dataclass(eq=False)
class Result:
 observation:Obs;terminated:bool;termination_reason:object;info:dict
class Action:
 def __init__(self,data):self.data=data

class FakeEmb:
 def __init__(self,end_at,reason,score,stale_at=None,raise_at=None):self.n=0;self.end_at=end_at;self.reason=reason;self.score=score;self.stale_at=stale_at;self.raise_at=raise_at;self.last=None
 def step(self,a):
  self.n+=1
  if self.raise_at==self.n:raise ValueError('arbitrary simulator error')
  if self.stale_at==self.n:return self.last
  done=self.n>=self.end_at
  self.last=Result(Obs(),done,self.reason if done else None,{'score':self.score if done else 0.})
  return self.last

class Teacher:  # faithful mirror of frozen Teacher.tick termination behaviour
 def __init__(self,emb):self.emb=emb;self.record=[]
 def tick(self,phase):
  if len(self.record)>=4572:raise RuntimeError('reference time budget exhausted')
  r=self.emb.step(Action([0.]*14));self.record.append(dict(phase=phase,info=r.info))
  if r.terminated:raise RuntimeError('terminated: '+str(r.termination_reason))

def script(n):
 def s(t):
  for _ in range(n):t.tick('p')
 return s

class T(unittest.TestCase):
 def test_success_stops_reference(self):
  g=G.GuardedEmbodiment(FakeEmb(5,'success',1.));out=G.run_reference(Teacher(g),script(10),lambda:1)
  self.assertEqual(out,dict(outcome='native_success',controls=5,public_score=1.))
 def test_success_needs_native_history(self):
  g=G.GuardedEmbodiment(FakeEmb(5,'success',1.))
  with self.assertRaises(RuntimeError):G.run_reference(Teacher(g),script(10),lambda:0)
 def test_success_needs_public_score(self):
  g=G.GuardedEmbodiment(FakeEmb(5,'success',.5))
  with self.assertRaises(RuntimeError):G.run_reference(Teacher(g),script(10),lambda:1)
 def test_failure_terminations_propagate(self):
  for reason in ('wrist_over_80N','jolt_3g','time_limit','nonfinite'):
   g=G.GuardedEmbodiment(FakeEmb(3,reason,0.))
   with self.assertRaisesRegex(RuntimeError,'terminated: '+reason):G.run_reference(Teacher(g),script(10),lambda:1)
 def test_unrelated_runtime_error_not_swallowed_even_after_success(self):
  g=G.GuardedEmbodiment(FakeEmb(2,'success',1.));t=Teacher(g)
  def s(t):
   try:t.tick('a');t.tick('b')
   except RuntimeError:pass
   raise RuntimeError('IK rejects phase')
  with self.assertRaisesRegex(RuntimeError,'IK rejects'):G.run_reference(t,s,lambda:1)
 def test_stale_result_rejected(self):
  g=G.GuardedEmbodiment(FakeEmb(9,'success',1.,stale_at=3))
  with self.assertRaises(G.StaleResult):G.run_reference(Teacher(g),script(10),lambda:1)
 def test_arbitrary_exception_propagates(self):
  g=G.GuardedEmbodiment(FakeEmb(9,'success',1.,raise_at=2))
  with self.assertRaises(ValueError):G.run_reference(Teacher(g),script(10),lambda:1)
 def test_script_without_termination_is_not_success(self):
  g=G.GuardedEmbodiment(FakeEmb(99,'success',1.))
  with self.assertRaises(G.GuardViolation):G.run_reference(Teacher(g),script(3),lambda:1)
 def test_each_action_once_and_no_step_after_termination(self):
  g=G.GuardedEmbodiment(FakeEmb(9,'success',1.));a=Action([0.]*14);g.step(a)
  with self.assertRaises(G.GuardViolation):g.step(a)
  g=G.GuardedEmbodiment(FakeEmb(1,'success',1.));g.step(Action(0))
  with self.assertRaises(G.GuardViolation):g.step(Action(0))
 def test_budget_preserved(self):
  g=G.GuardedEmbodiment(FakeEmb(10**6,'success',1.))
  for _ in range(4572):g.step(Action(0))
  with self.assertRaises(G.GuardViolation):g.step(Action(0))

class Transport:
 def __init__(self,resps):self.resps=list(resps);self.calls=[]
 def request(self,sid,obs):
  self.calls.append((sid,sorted(obs)));r=self.resps.pop(0)
  if isinstance(r,Exception):raise r
  return r
R4=[[0.]*14]*4
class D(unittest.TestCase):
 def _removed_custom_transport_test_one_session_four_rows(self):
  t=Transport([dict(session='s',rows=R4)]*3);g=G.GuardedEmbodiment(FakeEmb(10,'success',1.))
  out=G.run_model(g,Obs(),G.DreamSession(t),Action)
  self.assertEqual((out['controls'],out['requests'],out['reason']),(10,3,'success'))
  self.assertEqual(t.calls[0],(None,['images','joint_pos']));self.assertTrue(all(c[0]=='s' for c in t.calls[1:]))
 def _removed_custom_transport_test_errors_fail_not_score(self):
  for bad in (TimeoutError('x'),dict(session='s',expired=True),dict(session='s2',rows=R4),dict(session='s',rows=R4[:3]),dict(session='s',rows=[[float('nan')]*14]*4),dict(session='s',rows=[[0.]*13]*4)):
   t=Transport([dict(session='s',rows=R4),bad]);g=G.GuardedEmbodiment(FakeEmb(99,'success',1.))
   with self.assertRaises(G.PolicyFailure):G.run_model(g,Obs(),G.DreamSession(t),Action)
   self.assertEqual(len(g.actions),4)
 def _removed_custom_transport_test_no_rgb_fails(self):
  with self.assertRaises(G.PolicyFailure):G.DreamSession.public_obs(Obs(images={}))

class Root(unittest.TestCase):
 def test_fail_closed(self):
  fp='a'*64;d=Path(tempfile.mkdtemp());p=d/'r.json'
  self.assertFalse(G.credit_allowed(p,fp)[0])
  p.write_text(json.dumps(dict(placeholder=True,reviewer='ROOT',approved=True)));self.assertFalse(G.credit_allowed(p,fp)[0])
  ch={c:dict(approved=True,evidence_sha256='b'*64) for c in G.ROOT_CHANNELS}
  p.write_text(json.dumps(dict(reviewer='ROOT',approved=True,source_fingerprint=fp,channels=dict(ch,reset=dict(approved=False)))));self.assertFalse(G.credit_allowed(p,fp)[0])
  p.write_text(json.dumps(dict(reviewer='ROOT',approved=True,source_fingerprint='c'*64,channels=ch)));self.assertFalse(G.credit_allowed(p,fp)[0])
  p.write_text(json.dumps(dict(reviewer='ROOT',approved=True,source_fingerprint=fp,channels=ch)));self.assertTrue(G.credit_allowed(p,fp)[0])
 def test_seed(self):
  self.assertEqual(G.harness_seed(7),7)
 def test_package(self):
  self.assertEqual(G.verify_package(),'6ca25a60cac2458a6088ad04b450adfc2a87429040b46a120844d9d1829b6f92')
if __name__=='__main__':unittest.main()
