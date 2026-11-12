"""Validate genuine Inspect model trial, continuous session and matching source."""
from pathlib import Path
import json,sys,math
out=Path(sys.argv[1]);summaries=list((out/'model').glob('adhoc_*.json'));assert len(summaries)==1,'missing/ambiguous model summary';summary=json.loads(summaries[0].read_text());records=list((out/'model/records').rglob('record.json'));assert len(records)==1;record=json.loads(records[0].read_text())
assert summary['status']=='success' and summary['error'] is None and summary['results']['errored_trials']==0,summary.get('error')
assert record['status']=='success' and record['error'] is None,record.get('error')
assert record['termination']['reason'] not in ['measurement_error','initialization_error'],record['termination']
assert record['configuration']['max_steps']==1604
assert record['configuration']['scene']['instruction']==json.loads((out/'registration.json').read_text())['instruction']
assert summary['eval']['policy']=='dream' and record['configuration']['policy']['name']=='dream'
assert 0<record['steps']<=1604
rows=[json.loads(x) for x in (out/'continuity.jsonl').read_text().splitlines()];chunks=[x for x in rows if x['kind']=='chunk'];assert chunks and all(x['continuous'] for x in chunks);assert len({x['session_sha256'] for x in chunks})==1
frames=list((out/'model/frames').rglob('*.npy'));counts={name:len([p for p in frames if name in p.name]) for name in ['top_cam','left_cam','right_cam']};assert all(v==record['steps']+1 for v in counts.values()),counts
score=summary['results']['metrics']['task_score'];assert math.isfinite(score) and 0.<=score<=1.
report={'kind':'actualDream1 model evaluation; score is scorer output, not a demo','valid_completed_trial':True,'policy':'dream','model':'dreamlabs/dream-1-beta','score':score,'steps':record['steps'],'max_steps':1604,'termination':record['termination'],'continuous_model_evaluation':True,'request_count':len(chunks),'sessions':1,'frame_counts':counts,'record':str(records[0]),'summary':str(summaries[0]),'submission_ready':False}
(out/'model-verification.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
