"""Validate an actual completed Dream trial and independently regrade its trace."""
import argparse,json
from pathlib import Path
from scoring import Scorer

def analyze(directory):
    logs=[p for p in directory.glob('adhoc_*.json') if not p.name.endswith('.live.json')]
    if len(logs)!=1:raise ValueError('Expected exactly one final Inspect evaluation log')
    log=json.loads(logs[0].read_text());samples=log.get('samples',[]);result=log.get('results',{})
    rows=[json.loads(x) for x in (directory/'measurements.jsonl').read_text().splitlines()]
    scorer=Scorer();mismatches=[]
    for i,row in enumerate(rows):
        if scorer.update(row)!=row['score']:mismatches.append(i)
    counted=sum(e['kind']=='step' for s in samples for epoch in s.get('events',[]) for e in epoch)
    errors=[log.get('error'),*(s.get('error') for s in samples)]
    score=result.get('metrics',{}).get('task_score')
    valid=bool(not any(errors) and result.get('errored_trials')==0 and result.get('total_trials')==1
       and rows and counted==len(rows) and score is not None and not mismatches and abs(float(score)-scorer.score)<1e-9)
    ts=[t for s in samples for t in (s.get('policy_transcripts') or []) if isinstance(t,dict)]
    requests=[r for t in ts for r in t.get('requests',[]) if isinstance(r,dict)]
    ids=[r.get('session_id') for r in requests]
    gate=[json.loads(x) for x in (directory/'continuity.jsonl').read_text().splitlines()]
    chunks=[x for x in gate if x.get('kind')=='chunk']
    continuous=bool(ids) and all(ids) and len(set(ids))==1 and len(chunks)==len(requests) and all(x['continuous'] for x in chunks)
    return dict(valid_completed_trial=valid,continuous_model_evaluation=valid and continuous,
       benchmark_score=float(score) if valid and continuous else None,framework_score=score,
       trace_score=scorer.score,trace_score_mismatches=mismatches,recorded_steps=len(rows),framework_steps=counted,
       model=ts[0].get('model') if ts else None,request_count=len(requests),unique_server_sessions=len(set(ids)),
       termination_reasons=[r for s in samples for r in s.get('termination_reasons',[])],
       infrastructure_errors=errors,framework_results=result,final_log=logs[0].name,
       scope='One actual fixed-layout model trial; no success-rate or robustness claim.')
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory',type=Path);a=p.parse_args();r=analyze(a.directory)
    (a.directory/'analysis.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
