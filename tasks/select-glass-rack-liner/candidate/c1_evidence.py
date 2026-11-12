"""Measured evidence only; never changes scoring or channel certification."""
import json,hashlib,math
from pathlib import Path
import numpy as np

def measured_goal_evidence(report,run,out):
 status=report.get('status',{});out=Path(out)
 if run is not None and not run.journal.closed:run.journal.flush()
 checks={
  'raw_score_one':status.get('raw_score')==1.,
  'no_error':not report.get('error') and status.get('measurement_error') is None,
  'no_veto':status.get('failure') is None and status.get('cup_fell') is False,
  'controls_in_budget':type(report.get('controls')) is int and 0<report['controls']<=2268,
  'required_hold':len(status.get('items',[]))==status.get('required') and all(x.get('value')==1. and math.isfinite(x.get('hold_s',float('nan'))) and x['hold_s']>=20. for x in status.get('items',[])),
 }
 try:
  frozen=out/'source-frozen';expected=json.loads((out/'source-sha256.json').read_text())
  actual={str(p.relative_to(frozen)):hashlib.sha256(p.read_bytes()).hexdigest() for p in frozen.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
  checks['source_bound']=actual==expected and bool(expected)
  rows=[json.loads(x) for x in (out/'robot-journal.jsonl').read_text().splitlines()]
  checks['genuine_finite_14D_journal']=bool(rows) and len(rows)==report['controls']==len(run.actions) and all(r['control']==i and np.asarray(r['action14']).shape==(14,) and np.isfinite(r['action14']).all() and np.asarray(r['measured_joint14']).shape==(14,) and np.isfinite(r['measured_joint14']).all() and r['action14']==run.actions[i-1] for i,r in enumerate(rows,1))
  # Source physics journal must contain the actual 1ms sampling history.
  samples=[json.loads(x) for x in (out/'measurements.jsonl').read_text().splitlines()]
  checks['physical_journal_present']=len(samples)==report['controls'] and all(s['controls']==i-1 and s['raw_score'].get('failure') is None and not s['raw_score'].get('cup_fell',False) and all(np.isfinite(np.asarray(s['row'][k],dtype=float)).all() for k in ['cup_pos','cup_R','liner_pos','liner_R','wrist_force_n']) for i,s in enumerate(samples,1))
  checks['actual_20Hz']=status['physics_samples']==report['controls']*50 and math.isclose(status['physics_s'],report['controls']/20,rel_tol=0,abs_tol=1e-7)
 except (OSError,ValueError,KeyError,TypeError,AttributeError):
  checks['journal_validation_complete']=False
 return {'measured_goal_reached':all(checks.values()),'measured_goal_checks':checks,'evidence_is_submission_certification':False}
