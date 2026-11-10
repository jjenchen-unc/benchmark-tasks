"""Build an UNAPPROVED portable ROOT form, or validate explicit ROOT approval."""
from pathlib import Path
import sys,argparse,shutil
O=Path(__file__).resolve().parent;sys.path.insert(0,str(O))
from protocol import check,fresh,receipt,sha,CHANNELS
from strict_json import dumps
p=argparse.ArgumentParser();p.add_argument('--replay',type=Path);p.add_argument('--out',type=Path);p.add_argument('--validate',type=Path);a=p.parse_args()
if a.validate:print(dumps({'validated':True,'summary':receipt(a.validate)['actual_summary']}));raise SystemExit()
if not a.replay or not a.out:raise ValueError('replay and fresh output required')
m=check();summary=fresh(a.replay);out=a.out.resolve()
if out.exists():raise ValueError('fresh review folder required')
shutil.copytree(a.replay,out/'replay');files={str(p.relative_to(out/'replay')):sha(p) for p in (out/'replay').rglob('*') if p.is_file()}
r={'schema':'b1-v8-root-physical-review','approved':False,'reviewer':None,'manifest':m,'replay':'replay','actual_summary':summary,'evidence_sha256':files,'historical_fit545_is_full_success':False,'mounted_support_approved':False,'channels':{c:{'approved':False,'rationale':'','proofs':[]} for c in sorted(CHANNELS)}}
(out/'ROOT-REVIEW-UNAPPROVED.json').write_text(dumps(r,indent=2)+'\n')
print(dumps({'unapproved_form':str(out/'ROOT-REVIEW-UNAPPROVED.json'),'actual_summary':summary}))
