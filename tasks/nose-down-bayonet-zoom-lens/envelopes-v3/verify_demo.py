"""Recompute an actual source-bound scripted demo with approved public channels."""
from pathlib import Path
import argparse,sys,json,hashlib,importlib.util
p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--review',type=Path,required=True);p.add_argument('--report',type=Path);a=p.parse_args();sys.path[:0]=[str(a.bundle),str(a.bundle/'candidate')]
import protocol as P
from mechanics import History
review=P.receipt(a.review);m=P.check();sealed_reference=(a.review.resolve().parent/review['reference']).resolve();is_original_reference=a.out.resolve()==sealed_reference
if is_original_reference and a.report is None:raise ValueError('sealed original reference requires separate --report; cache stays immutable')
summary=P.fresh(a.out,'reference',review['evidence_manifest'] if is_original_reference else None);h=History();n=0
for line in (a.out/'physics.jsonl').open():
 row=P.loads(line);P.validate_physics_row(row['row']);h.update(row['row'],.002);n+=1
 for k,v in h.result().items():
  if row['status'].get(k)!=v:raise ValueError('actual native History mismatch '+k)
 if row['status']['measurement_error'] is not None:raise ValueError('invalid physics measurement')
assert n==25*summary['controls'] and n>0
verifier=Path(__file__).with_name('verify_b6_force_trace.py');assert P.sha(verifier)=='76e98757ce093d3e1b50795417a06488db83c0a7ec2b43e8c2bfd28ce49a905e'
spec=importlib.util.spec_from_file_location('_b6_force_verifier',verifier);f=importlib.util.module_from_spec(spec);spec.loader.exec_module(f);force=f.verify(a.bundle,a.out/'force-vectors.jsonl',a.out/'physics.jsonl')
score=h.result(P.CHANNELS)['score'];assert P.check()==m
r={'kind':'SOURCE_BOUND_APPROVED_CHANNEL_DEMO_SCORE','source_manifest':m,'review_sha256':P.sha(a.review),'actual_summary':summary,'controls':summary['controls'],'physics_samples':n,'public_score':score,'historical_raw_journal_not_modified':True,'full_force_audit':force}
destination=a.report if a.report is not None else a.out/'approved-demo-score.json'
if destination.resolve().is_relative_to(a.review.resolve().parent) or destination.resolve().is_relative_to(a.bundle.resolve()):raise ValueError('report must not modify sealed evidence or physical source')
destination.write_text(json.dumps(r,indent=2,allow_nan=False)+'\n');print(json.dumps(r,indent=2,allow_nan=False))
