"""Pure model preflight; no SDK/provider imports. Fresh replay requires ROOT review."""
from pathlib import Path
from c1_validation import strict,sha,verify,source_map,integer,number

def check(root,review_path):
 root=Path(root)
 if not verify(root):raise ValueError('ROOT-approved split receipt unavailable')
 review_path=Path(review_path).resolve();review=strict(review_path)
 if review['kind']!='ROOT_APPROVED_FRESH_PUBLIC_C1_REPLAY' or review['root_approved'] is not True:raise ValueError('fresh public replay not ROOT approved')
 if review['candidate_source_sha256']!=source_map(root):raise ValueError('fresh replay review source mismatch')
 trial=(review_path.parent/review['trial']).resolve()
 if not trial.is_relative_to(review_path.parent):raise ValueError('review path escape')
 for n,h in review['files_sha256'].items():
  f=(trial/n).resolve()
  if not f.is_relative_to(trial) or sha(f)!=h:raise ValueError('fresh replay evidence mismatch')
 if not {'result.json','source-sha256.json','measurements.jsonl'}.issubset(review['files_sha256']):raise ValueError('incomplete fresh replay evidence')
 result=strict(trial/'result.json');frozen=strict(trial/'source-sha256.json')
 if {n:h for n,h in frozen.items() if n not in ('manifest.json','c1-review-receipt.json')}!=source_map(root):raise ValueError('fresh replay actual frozen source mismatch')
 if result.get('error') or result['public_case'] is not True or integer(result['physical_seed'])!=42 or result['physical_raw_fullscore'] is not True or result['verified_fullscore'] is not True:raise ValueError('fresh public replay incomplete')
 if not 0<integer(result['controls'])<=2268 or number(result['status']['raw_score'])!=1 or number(result['status']['score'])!=1:raise ValueError('invalid actual replay score/budget')
 return dict(ready=True,physical_evidence='ROOT-reviewed fresh public replay',model_executed=False)
