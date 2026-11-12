"""Create an UNAPPROVED ROOT review template from fresh public replay; never activate."""
from pathlib import Path
import sys,argparse,json
R=Path(__file__).resolve().parent;sys.path.insert(0,str(R/'candidate'))
from c1_validation import strict,sha,source_map,integer,number,verify
p=argparse.ArgumentParser();p.add_argument('--candidate',type=Path,required=True);p.add_argument('--trial',type=Path,required=True);a=p.parse_args();trial=a.trial.resolve();candidate=a.candidate.resolve()
if not verify(candidate):raise ValueError('approved split receipt required')
result=strict(trial/'result.json');actual=strict(trial/'source-sha256.json')
if {n:h for n,h in actual.items() if n not in ('manifest.json','c1-review-receipt.json')}!=source_map(candidate):raise ValueError('fresh replay source mismatch')
if result.get('error') or result['public_case'] is not True or integer(result['physical_seed'])!=42 or result['physical_raw_fullscore'] is not True or result['verified_fullscore'] is not True:raise ValueError('fresh public replay not full')
if not 0<integer(result['controls'])<=2268 or number(result['status']['score'])!=1 or number(result['status']['raw_score'])!=1:raise ValueError('invalid score/budget')
if result['actions_sha256']!=sha(R/'historical43748-actions.json'):raise ValueError('unexpected replay actions')
files={n:sha(trial/n) for n in ['result.json','source-sha256.json','measurements.jsonl','launch.json','task-construction.json']}
d=dict(kind='ROOT_APPROVED_FRESH_PUBLIC_C1_REPLAY',root_approved=False,trial=trial.name,candidate_source_sha256=source_map(candidate),files_sha256=files,review_note='ROOT inspect complete native20s hold/contact/motor/guard/raw14D trace and fresh public score; old score transfer prohibited. Set true only after actual review.')
out=trial.parent/'ROOT-fresh-public-review-UNAPPROVED.json'
with out.open('x') as f:f.write(json.dumps(d,indent=2,allow_nan=False)+'\n')
print(str(out))
