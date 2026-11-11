import sys,hashlib,json
from pathlib import Path
R=Path(__file__).resolve().parent.parent;sys.path.insert(0,str(R/'source'));sys.path.insert(0,str(R/'runtime'))
import guard
def test_seed_unchanged():
 assert guard.harness_seed(12345)==12345
 try:guard.harness_seed('1');assert False
 except guard.GuardViolation:pass
def test_no_custom_transport():
 assert not hasattr(guard,'DreamSession') and not (R/'runtime/run_model.py').exists()
def test_receipt_and_credit():
 fp=guard.verify_package();d=guard.verify_receipt(fp);assert d['source_fingerprint']==fp
 import contract;assert contract.VALIDATED==frozenset()
 guard.enable_credit(fp);assert contract.VALIDATED==contract.REQUIRED
 h=contract.History();assert h.result()['gate'] is None
 contract.VALIDATED=frozenset()
def test_real_result_identity():
 from inspect_robots.types import Action
 class E:
  def __init__(s):s.n=0
  def step(s,a):
   s.n+=1;from types import SimpleNamespace as N
   return N(terminated=s.n==3,termination_reason='success' if s.n==3 else None,info={'score':1},observation=None)
 g=guard.GuardedEmbodiment(E())
 class Tch:
  emb=g;record=[]
  def run(s):
   while True:
    r=s.emb.step(Action(data=[0.]*14));s.record.append(dict(info=r.info))
    if r.terminated:raise RuntimeError('terminated: success')
 assert guard.run_reference(Tch(),lambda t:t.run(),lambda:1)['controls']==3
 g2=guard.GuardedEmbodiment(E());T2=Tch();T2.emb=g2;T2.record=[]
 try:guard.run_reference(T2,lambda t:t.run(),lambda:0);assert False
 except RuntimeError as e:assert str(e)=='terminated: success'
