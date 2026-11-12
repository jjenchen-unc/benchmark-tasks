"""Explicit source-bound model-route receipt. No score/task-state assignments."""
from pathlib import Path
from runtime_credit import load,validate,sha,REQUIRED

def review_path(core,value):
 if not isinstance(value,str) or not value.strip():raise ValueError('DRAWERS_CREDIT_REVIEW: operator-provided portable relative review path required')
 p=Path(value)
 if p.is_absolute():raise ValueError('credit review must be a portable relative path')
 # Review travels within the complete portable bundle (candidate or sibling).
 base=Path(core).resolve().parent;resolved=(Path(core)/p).resolve()
 if not resolved.is_relative_to(base):raise ValueError('credit review escapes portable bundle')
 if not resolved.is_file():raise ValueError('credit review missing')
 return resolved

def validate_model(core,value):
 core=Path(core).resolve();path=review_path(core,value)
 # Same exact selected physical review and native instruction as reference.
 receipt=validate(core,path)
 review=load(path);identity=load(core/'pad-identity.json')
 # Selected no-jar-contact evidence can never authorize arbitrary model paths.
 if identity.get('general_mapping_closed') is not True or identity.get('runtime_mapping_implemented') is not True:
  raise ValueError('general model credit blocked: complete native pad-solid mapping unresolved; selected no-contact reference scope is insufficient')
 if review.get('model_entry_sha256')!=sha(core/'model_entry.py') or review.get('model_credit_sha256')!=sha(core/'model_credit.py') or review.get('pad_identity_sha256')!=sha(core/'pad-identity.json'):
  raise ValueError('ROOT model-entry/pad source binding missing')
 if review.get('model_scope')!='general-native-pad-mapping':raise ValueError('explicit general native pad ROOT review required')
 # Fresh affected physical approval cannot be inferred from selected old score.
 affected=review.get('affected_pad_validation',{})
 if affected.get('approved') is not True or affected.get('physical') is not True:raise ValueError('fresh affected pad physical validation required')
 from pad_region import DATA
 for name in ('model_physics.py','pad_region.py','pad-region.json','pad_foundation.py','runtime_credit.py'):
  if affected.get('source_sha256',{}).get(name)!=sha(core/name):raise ValueError('affected pad source binding missing: '+name)
 if type(affected.get('controls')) is not int or not 0<affected['controls']<=3586:raise ValueError('affected controls invalid')
 if type(affected.get('hold_s')) not in (int,float) or affected['hold_s']<1:raise ValueError('affected native hold missing')
 evidence=affected.get('evidence',[])
 if not evidence:raise ValueError('affected evidence absent')
 for row in evidence:
  ep=(path.parent/row['path']).resolve()
  if not ep.is_relative_to(core.parent) or sha(ep)!=row['sha256']:raise ValueError('affected evidence binding invalid')
 from fresh_evidence import validate_review
 validate_review(core,path,affected)
 return dict(receipt,model_entry_source_bound=True,affected_pad_validation=affected,old_public_score_transferred=False)

def configure_model(core,value):
 receipt=validate_model(core,value)
 from model_physics import install
 install()
 import drawers_physics
 drawers_physics.VALIDATED_CHANNELS=REQUIRED
 return receipt
