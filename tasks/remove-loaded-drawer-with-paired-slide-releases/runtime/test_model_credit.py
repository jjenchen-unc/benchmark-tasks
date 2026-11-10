"""Focused CPU synthetic model binding tests, no approved reviews produced."""
from pathlib import Path
import sys,unittest,types
from unittest.mock import patch
O=Path(__file__).resolve().parent;C=O/'candidate';sys.path.insert(0,str(C))
import model_credit as m
class ModelCreditTests(unittest.TestCase):
 def test_path(self):
  for value in (None,'','/tmp/review.json','../../outside.json','missing.json'):
   with self.subTest(value=value),self.assertRaises(ValueError):m.review_path(C,value)
  with patch.object(Path,'is_file',return_value=True):
   self.assertEqual(m.review_path(C,'../reviewed-actual-portable/ROOT-review.json'),O/'reviewed-actual-portable/ROOT-review.json')
 def test_invalid_review_never_promotes(self):
  fake=types.SimpleNamespace(VALIDATED_CHANNELS=frozenset())
  with patch.dict(sys.modules,drawers_physics=fake),patch.object(m,'review_path',return_value=O/'bad.json'),patch.object(m,'validate',side_effect=ValueError('bad review')):
   with self.assertRaisesRegex(ValueError,'bad review'):m.configure_model(C,'bad')
  self.assertEqual(fake.VALIDATED_CHANNELS,frozenset())
 def test_selected_no_contact_review_never_authorizes_general_model(self):
  fake=types.SimpleNamespace(VALIDATED_CHANNELS=frozenset())
  with patch.dict(sys.modules,drawers_physics=fake),patch.object(m,'review_path',return_value=O/'synthetic.json'),patch.object(m,'validate',return_value={'status':'SYNTHETIC'}),patch.object(m,'load',side_effect=[{},m.load(C/'pad-identity.json')]):
   with self.assertRaisesRegex(ValueError,'ROOT model-entry/pad source binding missing'):m.configure_model(C,'synthetic')
  self.assertEqual(fake.VALIDATED_CHANNELS,frozenset())
 def test_fresh_affected_required_and_numeric(self):
  review={'model_entry_sha256':m.sha(C/'model_entry.py'),'model_credit_sha256':m.sha(C/'model_credit.py'),'pad_identity_sha256':m.sha(C/'pad-identity.json'),'model_scope':'general-native-pad-mapping'}
  identity=m.load(C/'pad-identity.json')
  def attempt(affected):
   data=dict(review,affected_pad_validation=affected)
   with patch.object(m,'review_path',return_value=O/'review.json'),patch.object(m,'validate',return_value={'status':'SYNTHETIC_ONLY'}),patch.object(m,'load',side_effect=[data,identity]):
    return m.validate_model(C,'synthetic')
  for affected in ({},{'approved':1,'physical':True},{'approved':True,'physical':1}):
   with self.assertRaises(ValueError):attempt(affected)
  bound={'approved':True,'physical':True,'source_sha256':{n:m.sha(C/n) for n in ('model_physics.py','pad_region.py','pad-region.json','pad_foundation.py','runtime_credit.py')},'controls':True,'hold_s':1.,'evidence':[{'path':'x','sha256':'bad'}]}
  with self.assertRaisesRegex(ValueError,'controls invalid'):attempt(bound)
  bound.update(controls=20,hold_s=True)
  with self.assertRaisesRegex(ValueError,'hold missing'):attempt(bound)
  bound.update(hold_s=1.,evidence=[])
  with self.assertRaisesRegex(ValueError,'evidence absent'):attempt(bound)
 def test_instruction(self):
  import ast,json
  self.assertEqual(ast.get_docstring(ast.parse((C/'model_entry.py').read_text())),ast.get_docstring(ast.parse((C/'task.py').read_text())))
  self.assertEqual(ast.get_docstring(ast.parse((C/'model_entry.py').read_text())),json.loads((C/'task.draft.json').read_text())['instruction'])
if __name__=='__main__':unittest.main()
