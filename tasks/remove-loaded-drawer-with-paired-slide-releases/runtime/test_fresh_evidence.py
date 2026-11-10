"""CPU synthetic protocol tests; synthetic data never constitute physical approval."""
from pathlib import Path
import unittest,sys,tempfile,json,copy
from unittest.mock import patch
O=Path(__file__).resolve().parent;C=O/'candidate';sys.path.insert(0,str(O));sys.path.insert(0,str(C))
from fresh_evidence import check,validate_review,relative_file,NAMES
from runtime_credit import load,sha
class FreshEvidenceTests(unittest.TestCase):
 def setUp(self):
  (O/'work').mkdir(exist_ok=True)
  self.d=Path(tempfile.mkdtemp(dir=O/'work',prefix='CPU-SYNTHETIC-'));self.m=load(C/'manifest.json')
  self.status={'controls':1,'samples':12,'physics_s':.05,'failure':None,'measurement_error':None,'score':0.,'candidate_score':1.,'hold_s':1.}
  physics={'contents':True,'drawer_touch':False,'drop':False,'max_x':.1,'drawer_p':[-.02,0,.835],'drawer_R':[[1,0,0],[0,1,0],[0,0,1]]}
  rows=[{'status':dict(self.status,controls=0,samples=0,physics_s=0.),'physics':physics,'first_termination':None},{'status':self.status,'physics':physics,'first_termination':None}]
  self.write('sourcehashes.json',self.m);self.write('launcher.json',{'exitcode':0,'phase':'replay','source_unchanged':True,'source_before':self.m['sha256'],'source_after':self.m['sha256'],'manifest_sha256':sha(C/'manifest.json')})
  self.write('credit-binding.json',{'status':'PENDING_ROOT_PHYSICAL_REVIEW','runtime_channels_promoted':[]})
  self.write('result.json',{'status':'foundation_observed','first_termination':None,'status_channels':self.status})
  (self.d/'journal.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
  for n in ('top_cam.mp4','left_cam.mp4','right_cam.mp4'):(self.d/n).write_bytes(b'SYNTHETIC_NOT_VIDEO')
  self.write('media-finalization.json',{'all_encoded':True,'frames_per_camera':2,'files':{n:{'bytes':len(b'SYNTHETIC_NOT_VIDEO')} for n in ('top_cam.mp4','left_cam.mp4','right_cam.mp4')}})
 def tearDown(self):pass  # Preserve explicitly synthetic test evidence; avoid shared-filesystem cleanup races.
 def write(self,n,v):(self.d/n).write_text(json.dumps(v))
 def test_valid_numeric_protocol_is_not_physical_approval(self):
  r=check(C,self.d);self.assertFalse(r['physical_approval']);self.assertTrue(r['review_required'])
 def test_bool_control_and_hold_rejected(self):
  for key in ('controls','hold_s'):
   rows=[json.loads(l) for l in (self.d/'journal.jsonl').read_text().splitlines()];old=copy.deepcopy(rows);rows[-1]['status'][key]=True
   (self.d/'journal.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
   with self.assertRaises(ValueError):check(C,self.d)
   (self.d/'journal.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in old))
 def test_old_source_rejected(self):
  r=load(self.d/'launcher.json');r['source_before']['pad_region.py']='old';self.write('launcher.json',r)
  with self.assertRaises(ValueError):check(C,self.d)
 def test_relative_and_symlink_escape_rejected(self):
  for value in ('/etc/passwd','../candidate/task.py'):
   with self.assertRaises(ValueError):relative_file(self.d,value)
  (self.d/'escape').symlink_to(C/'task.py')
  with self.assertRaises(ValueError):relative_file(self.d,'escape')
 def test_review_binds_all_evidence_and_exact_numbers(self):
  a={'schema':'drawers-fresh-pad-replay-v19','controls':1,'hold_s':1.,'evidence':[{'name':n,'path':n,'sha256':sha(self.d/n)} for n in NAMES]}
  self.assertEqual(validate_review(C,self.d/'form.json',a)['controls'],1)
  a['controls']=True
  with self.assertRaises(ValueError):validate_review(C,self.d/'form.json',a)
  a['controls']=1;a['evidence'].pop()
  with self.assertRaises(ValueError):validate_review(C,self.d/'form.json',a)
 def test_builder_emits_only_unapproved_source_bound_packet(self):
  from build_fresh_review import build
  for n in ('live-stage-ownership.json','pre-action.json','cold-task.json'):self.write(n,{'kind':'CPU_SYNTHETIC_ONLY'})
  packet=self.d/'packet';build(self.d,packet)
  form=load(packet/'ROOT-general-pad-UNAPPROVED.json')
  self.assertIs(form['affected_pad_validation']['approved'],False)
  self.assertIs(form['affected_pad_validation']['physical'],False)
  from model_credit import validate_model
  import os
  with self.assertRaisesRegex(ValueError,'fresh affected pad physical validation required'):validate_model(C,os.path.relpath(packet/'ROOT-general-pad-UNAPPROVED.json',C))
  # Synthetic ROOT approval exists only in this mock's memory. No physical
  # approved file or configured runtime channel is produced by this test.
  import model_credit as m
  approved=copy.deepcopy(form);approved['affected_pad_validation'].update(approved=True,physical=True)
  original=m.load
  with patch.object(m,'load',side_effect=lambda p: approved if Path(p).name=='ROOT-general-pad-UNAPPROVED.json' else original(p)):
   receipt=validate_model(C,os.path.relpath(packet/'ROOT-general-pad-UNAPPROVED.json',C))
   self.assertIs(receipt['model_entry_source_bound'],True)
   self.assertIs(receipt['old_public_score_transferred'],False)
 def test_public_credit_not_transferred(self):
  rows=[json.loads(l) for l in (self.d/'journal.jsonl').read_text().splitlines()];rows[-1]['status']['score']=1
  (self.d/'journal.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
  with self.assertRaises(ValueError):check(C,self.d)
if __name__=='__main__':unittest.main()
