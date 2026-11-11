"""External wrapper around the frozen pins runtime. Frozen source is never edited."""
import hashlib,json,math
from pathlib import Path
BUDGET=4572
PHYSICAL_CASE=42
ROOT=Path(__file__).resolve().parent.parent
class StaleResult(RuntimeError):pass
class GuardViolation(RuntimeError):pass
class PolicyFailure(RuntimeError):"""Run failed (error/expiry): not a score of 0, never stitched."""

def harness_seed(inspect_seed):
 if type(inspect_seed) is not int:raise GuardViolation('integer Inspect seed required')
 return inspect_seed  # passed unchanged; Inspect CLI derives the actual seed. Physical case fixed 42, jitter 0.

class GuardedEmbodiment:
 def __init__(self,emb):self._emb=emb;self.actions=[];self.results=[];self.terminal=None
 def __getattr__(self,k):return getattr(self._emb,k)
 def reset(self,*a,**k):
  if self.actions:raise GuardViolation('reset inside a trial')
  return self._emb.reset(*a,**k)
 def step(self,action):
  if self.terminal is not None:raise GuardViolation('step after termination')
  if len(self.actions)>=BUDGET:raise GuardViolation('native 4572 control budget exhausted')
  if any(a is action for a in self.actions):raise GuardViolation('action object submitted twice')
  self.actions.append(action)
  r=self._emb.step(action)
  if any(r is x for x in self.results):raise StaleResult('embodiment returned a stale Result')
  self.results.append(r)
  if r.terminated:self.terminal=r
  return r

def is_success(r,native_score):
 return (r is not None and r.terminated is True and r.termination_reason=='success'
  and isinstance(r.info,dict) and r.info.get('score')==1 and native_score==1)

def run_reference(teacher,script,native_score):
 """script(teacher) drives Teacher.tick. Teacher.tick raises RuntimeError on ANY termination;
 only the cached actual success Result ends the reference; everything else propagates."""
 g=teacher.emb
 if not isinstance(g,GuardedEmbodiment):raise GuardViolation('teacher must hold a GuardedEmbodiment')
 try:script(teacher)
 except RuntimeError as e:
  r=g.terminal
  if (r is not None and g.results and g.results[-1] is r and teacher.record
    and teacher.record[-1].get('info') is r.info and len(teacher.record)==len(g.actions)
    and str(e)=='terminated: '+str(r.termination_reason) and is_success(r,native_score())):
   return dict(outcome='native_success',controls=len(g.actions),public_score=r.info['score'])
  raise
 r=g.terminal
 raise GuardViolation('reference script ended without terminal success: '+repr(None if r is None else r.termination_reason))

import sys as _s;_s.path.insert(0,str(ROOT/'source'))
from contract import REQUIRED as _REQ
ROOT_CHANNELS=tuple(sorted(_REQ))  # exactly the frozen contract.REQUIRED eight
ADAPTER_SHA256='b525f6c889d0accbfa7ac3183371150138d4b717a6ea5814f45e4dc2f8eeac06'
REFERENCE_SHA256='b00ca1b46b20a9580179893937d2dbbbbca36da2071f01f41ddf0efee0630812'  # = source/reference.py
def credit_allowed(receipt_path,source_fingerprint):
 """Fail closed: missing/placeholder/mismatched Root receipts never permit public credit."""
 p=Path(receipt_path)
 if not p.is_file():return False,'missing Root receipt'
 try:d=json.loads(p.read_text())
 except Exception:return False,'unreadable Root receipt'
 if d.get('placeholder') or d.get('reviewer')!='ROOT' or d.get('approved') is not True:return False,'placeholder or unapproved receipt'
 if d.get('source_fingerprint')!=source_fingerprint:return False,'receipt bound to different source'
 ch=d.get('channels',{})
 bad=[c for c in ROOT_CHANNELS if not (isinstance(ch.get(c),dict) and ch[c].get('approved') is True and len(str(ch[c].get('evidence_sha256','')))==64)]
 if bad:return False,'channels not approved: '+','.join(bad)
 return True,'approved'

APPROVAL_SHA256='bce42aeba806b71fd6cdb0df19127038dd5c2cb01454018523f31c50ab2697de'
EVIDENCE=ROOT/'evidence'
def verify_receipt(source_fingerprint):
 p=EVIDENCE/'ROOT-eight-channels-approval.json'
 if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=APPROVAL_SHA256:raise GuardViolation('Root approval missing/stale')
 ok,why=credit_allowed(p,source_fingerprint)
 if not ok:raise GuardViolation(why)
 d=json.loads(p.read_text())
 if d.get('reference_sha256')!=REFERENCE_SHA256 or hashlib.sha256((ROOT/'source/reference.py').read_bytes()).hexdigest()!=REFERENCE_SHA256:raise GuardViolation('reference identity mismatch')
 if set(d['channels'])!=set(ROOT_CHANNELS):raise GuardViolation('receipt channels != contract.REQUIRED')
 for c,v in d['channels'].items():
  for e in v['evidence']:
   f=EVIDENCE/e['artifact']
   if not f.is_file() or hashlib.sha256(f.read_bytes()).hexdigest()!=e['sha256']:raise GuardViolation('evidence missing/stale: '+e['artifact'])
 return d
def enable_credit(source_fingerprint):
 verify_receipt(source_fingerprint)
 import contract
 contract.VALIDATED=contract.REQUIRED  # runtime-only; frozen file bytes unchanged
 return True
def verify_package():
 m=json.loads((ROOT/'manifest.json').read_text())
 actual={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(ROOT.rglob('*')) if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc' and p.name!='manifest.json'}
 if actual!=m['files']:raise GuardViolation('package manifest mismatch')
 src=json.loads((ROOT/'source/package-manifest.json').read_text())
 fp=hashlib.sha256(json.dumps(src['files'],sort_keys=True,separators=(',',':')).encode()).hexdigest()
 if fp!=m['source_fingerprint'] or fp!=src['fingerprint']:raise GuardViolation('frozen source fingerprint mismatch')
 source_actual={str(p.relative_to(ROOT/'source')):hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/'source').rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc' and p.name!='package-manifest.json'}
 if source_actual!=src['files']:raise GuardViolation('actual physical source differs from approved source map')
 if any(p.is_symlink() for p in ROOT.rglob('*')):raise GuardViolation('package symlink not allowed')
 return fp
