"""CPU synthetic contract tests; never emits an approved physical review."""
from pathlib import Path
import sys,json,unittest,copy,math
from unittest.mock import patch
O=Path(__file__).resolve().parent;sys.path.insert(0,str(O/'candidate'))
import runtime_credit as r
class ReceiptTests(unittest.TestCase):
 def test_strict_numbers(self):
  for s in ('{"a":NaN}','{"a":1e999}','{"a":1,"a":2}'):
   with self.assertRaises(ValueError):r.loads(s)
  self.assertIs(r.loads('{"a":true}')['a'],True)
 def test_missing_review(self):
  with self.assertRaises(OSError):r.validate(O/'candidate',O/'NO_SUCH_ROOT_REVIEW.json')
 def fixture(self):
  contract=r.load(O/'candidate/credit-contract.json');expected=contract['selected_source_hashes'];digest='f'*64
  review={'schema':'drawers-root-selected-43920-v1','reviewer':'ROOT','approved':True,'job_id':43920,'core_sha256':contract['core_sha256'],'proof_sha256':contract['proof_sha256'],'channels':{k:{'approved':True,'evidence':['actual:journal']} for k in r.REQUIRED},'actual_inputs':{k:{'path':k,'sha256':digest} for k in ('launcher','result','journal','sourcehashes')}}
  physics={'contents':True,'drawer_touch':False,'drop':False,'max_x':.10,'drawer_p':[-.02,0,.835],'drawer_R':[[1,0,0],[0,1,0],[0,0,1]]}
  rows=[]
  for i in range(21):
   status={'controls':i,'samples':i*12,'physics_s':i/20,'failure':None,'measurement_error':None,'candidate_score':1. if i==20 else .5,'hold_s':i/20}
   rows.append({'status':status,'physics':physics,'first_termination':None})
  data={'review':review,'contract':contract,'launcher':{'exitcode':0,'source_unchanged':True,'source_hashes_before':expected,'source_hashes_after':expected,'argv':['x','--mode','reference','job-43920/trial']},'result':{'status':'foundation_observed','first_termination':None,'status_channels':rows[-1]['status']},'sourcehashes':{'sha256':{k:v for k,v in expected.items() if k!='manifest.json'}}}
  def load(p):
   p=Path(p)
   return data['contract' if p.name=='credit-contract.json' else 'review' if p.name=='review.json' else p.name]
  def sha(p):
   p=Path(p);return contract['core_sha256'].get(p.name,contract['proof_sha256'].get('proofs/'+p.name,digest))
  return data,rows,load,sha
 def attempt(self,change=None):
  data,rows,load,sha=self.fixture()
  if change:change(data,rows)
  with patch.object(r,'task_code_sha',return_value=data['contract']['task_code_sha256']),patch.object(r,'load',side_effect=load),patch.object(r,'sha',side_effect=sha),patch.object(Path,'read_text',return_value='\n'.join(json.dumps(x) for x in rows)):
   return r.validate(O/'candidate',O/'review.json')
 def test_synthetic_complete_contract(self):
  v=self.attempt();self.assertEqual(v['candidate_score'],1.);self.assertFalse(v['submission_ready'])
 def test_reject_mutations(self):
  mutations=[lambda d,rs:d['review'].update(approved=1),lambda d,rs:d['review'].update(job_id=True),lambda d,rs:d['launcher'].update(exitcode=False),lambda d,rs:d['review']['channels']['pad_identity'].update(approved=False),lambda d,rs:d['review']['channels']['clock'].update(evidence=['invented']),lambda d,rs:d['launcher'].update(source_hashes_after={}),lambda d,rs:rs[-1]['status'].update(controls=True),lambda d,rs:rs[-1]['status'].update(hold_s=.999),lambda d,rs:rs[-1]['physics'].update(max_x=.12),lambda d,rs:rs[-1]['physics'].update(contents=1),lambda d,rs:rs[-1]['status'].update(failure='jar_damage'),lambda d,rs:rs[-1]['status'].update(samples=239),lambda d,rs:rs[-1]['physics']['drawer_p'].__setitem__(0,0.)]
  for i,f in enumerate(mutations):
   with self.subTest(i=i),self.assertRaises(ValueError):self.attempt(f)
 def test_native_instruction_and_task_code(self):
  import ast
  self.assertEqual(r.task_code_sha(O/'candidate/task.py'),r.load(O/'candidate/credit-contract.json')['task_code_sha256'])
  self.assertEqual(ast.get_docstring(ast.parse((O/'candidate/task.py').read_text())),r.load(O/'candidate/task.draft.json')['instruction'])
 def test_core_exact(self):
  contract=r.load(O/'candidate/credit-contract.json')
  for n,d in contract['core_sha256'].items():self.assertEqual(r.sha(O/'candidate'/n),d,n)
if __name__=='__main__':unittest.main()
