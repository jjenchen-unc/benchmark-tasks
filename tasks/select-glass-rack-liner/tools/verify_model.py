"""External read-only verifier for the frozen C1 v9 actual Inspect trial.

Run in the existing Brev Python after the process has ended, never via a CLI
that retries nonzero model invocations. This tool does not launch a simulator.
"""
from pathlib import Path
import argparse,hashlib,json,math,sys,datetime

def require(condition,message):
    if not condition:raise ValueError(message)
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def rows(path):
    def bad(value):raise ValueError('Nonfinite JSON token '+value)
    with Path(path).open() as stream:
        for line in stream:
            if line.strip():yield json.loads(line,parse_constant=bad)
def finite_number(value):return type(value) in (int,float) and math.isfinite(value)
def verify(candidate,out,fresh_review):
    candidate=Path(candidate).resolve();out=Path(out).resolve()
    sys.path.insert(0,str(candidate))
    from c1_validation import strict,CHANNELS,verify as verify_receipt
    from c1_public import INSTRUCTION
    from model_preflight import check
    import numpy as np
    require(verify_receipt(candidate),'Source/runtime channel receipt invalid')
    check(candidate,Path(fresh_review).resolve())
    frozen=out/'source-frozen'
    actual={str(p.relative_to(frozen)):sha(p) for p in frozen.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
    expected={str(p.relative_to(candidate)):sha(p) for p in candidate.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
    require(actual==expected==strict(out/'source-sha256.json'),'Frozen model source differs from reviewed candidate')
    registration=strict(out/'registration.json')
    for key,value in {'physical_seed':42,'target':'C1-T1','policy':'dream','model':'dreamlabs/dream-1-beta','max_steps':2268,'control_hz':20,'seed':42,'rows_per_request':4,'object_jitter':0,'device':'cpu','camera_size':[320,176],'scorer':'task_score','instruction':INSTRUCTION,'source_sha256':actual}.items():
        require(registration.get(key)==value,'Registration mismatch '+key)
    summaries=list((out/'model').glob('adhoc_*.json'));records=list((out/'model/records').rglob('record.json'))
    require(len(summaries)==len(records)==1,'Missing/ambiguous summary or record')
    summary=strict(summaries[0]);record=strict(records[0]);configuration=record['configuration'];steps=record['steps']
    require(summary['status']=='success' and summary['error'] is None,'Inspect summary error')
    require(summary['results']['errored_trials']==0 and summary['results']['total_trials']==1 and summary['results']['total_scenes']==1,'Non-single or errored evaluation')
    require(record['status']=='success' and record['error'] is None,'Inspect trial error')
    require(type(steps) is int and 0<steps<=2268,'Invalid control count')
    require(configuration['max_steps']==2268 and configuration['seed']==42,'Record budget/seed mismatch')
    require(configuration['scene']['instruction']==INSTRUCTION,'Actual model instruction mismatch')
    require(configuration['policy']['name']=='dream' and configuration['policy']['checkpoint']=='dreamlabs/dream-1-beta','Actual policy mismatch')
    require(configuration['embodiment']['name']=='yam_arena' and configuration['embodiment']['control_hz']==20,'Actual embodiment mismatch')
    require(summary['eval']['policy']=='dream' and summary['eval']['policy_checkpoint']=='dreamlabs/dream-1-beta' and summary['eval']['max_steps']==2268,'Summary policy/budget mismatch')
    reason=record['termination']['reason']
    require(reason not in ('measurement_error','measurement_unavailable','initialization_error','policy_error','error',None),'Invalid terminal condition')
    if reason=='time_limit':require(steps==2268,'Premature time-limit termination')
    require(len([e for e in record['events'] if e['kind']=='reset'])==1,'Actual record restarted episode')
    seed_contract=strict(out/'seed-contract.json')
    require(seed_contract['physical_seed']==42 and seed_contract['target']=='C1-T1' and seed_contract['inspect_trial_seed']==record['seed'],'Physical/Inspect reset-seed mismatch')
    continuity=list(rows(out/'continuity.jsonl'));chunks=[r for r in continuity if r['kind']=='chunk'];retries=[r for r in continuity if r['kind']=='transport_retry']
    require(sum(r['kind']=='episode_start' for r in continuity)==1,'Episode restart in continuity log')
    require(bool(chunks) and all(r['continuous'] is True and type(r['attempts']) is int and r['attempts']>=1 for r in chunks),'Invalid retry or discontinuity')
    require([r['index'] for r in chunks]==list(range(1,len(chunks)+1)),'Missing/duplicate chunk log index')
    identifiers={r['session_sha256'] for r in chunks}
    require(len(identifiers)==1 and all(isinstance(s,str) and len(s)==64 for s in identifiers),'Changed/missing session identifier')
    require(all(r['error_class']!='DreamSessionExpired' for r in retries),'Session expiration accepted')
    actions=list(rows(out/'model-actions.jsonl'));require(len(actions)==steps,'Actual telemetry/action count mismatch')
    for index,row in enumerate(actions,1):
        require(row['control']==index and row['before']['controls']==index-1 and row['after']['controls']==index,'Telemetry control discontinuity')
        action=row['action14'];require(type(action) is list and len(action)==14 and all(finite_number(v) for v in action),'Malformed actual action')
        for status in (row['before'],row['after']):
            require(status['missing']==[] and status['gate'] is None and status['measurement_error'] is None,'Unverified/missing/error runtime channel')
        after=row['after'];require(after['physics_samples']==50*index and abs(after['physics_s']-.05*index)<1e-6,'Physics clock/substep mismatch')
    measurements=list(rows(out/'measurements.jsonl'));require(len(measurements)==steps,'Per-control measured physics trace missing')
    require(all(r['raw_score']['physics_samples']==50*(i+1) for i,r in enumerate(measurements)),'Physics measurement sequence mismatch')
    score=summary['results']['metrics']['task_score'];require(finite_number(score) and 0<=score<=1,'Invalid scorer value')
    require(abs(actions[-1]['after']['score']-score)<1e-9,'Summary scorer differs from actual final source telemetry')
    frame_counts={};frame_inventory={}
    all_frames=list((out/'model/frames').rglob('*.npy'))
    for camera in ('top_cam','left_cam','right_cam'):
        frames=[p for p in all_frames if camera in p.name or camera in p.parts]
        require(len(frames)==steps+1,'Camera frame count mismatch '+camera)
        for path in frames:
            frame=np.load(path,mmap_mode='r',allow_pickle=False)
            require(frame.dtype==np.uint8 and frame.shape==(176,320,3),'Camera frame shape/type mismatch '+str(path))
        frame_counts[camera]=len(frames)
        frame_inventory[camera]=hashlib.sha256(''.join(str(p.relative_to(out))+'\n' for p in sorted(frames)).encode()).hexdigest()
    require(len(all_frames)==3*(steps+1),'Unexpected/overlapping frame inventory')
    files={str(p.relative_to(out)):sha(p) for p in [summaries[0],records[0],out/'registration.json',out/'source-sha256.json',out/'seed-contract.json',out/'continuity.jsonl',out/'model-actions.jsonl',out/'measurements.jsonl']}
    return {'at':datetime.datetime.now(datetime.timezone.utc).isoformat(),'kind':'completed actual Dream-1 C1 v9 trial; source scorer output','valid_completed_trial':True,'score':score,'steps':steps,'max_steps':2268,'termination':record['termination'],'model':'dreamlabs/dream-1-beta','continuous_model_evaluation':True,'sessions':1,'request_count':len(chunks),'transport_retry_events':len(retries),'session_sha256':next(iter(identifiers)),'verified_channels':sorted(CHANNELS),'frame_counts':frame_counts,'frame_filename_inventory_sha256':frame_inventory,'source_fingerprint':strict(candidate/'manifest.json')['source_fingerprint'],'evidence_sha256':files,'actual_media_encoding_and_visual_review_pending':True,'submission_ready':False}
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--candidate',type=Path,required=True);parser.add_argument('--out',type=Path,required=True);parser.add_argument('--fresh-review',type=Path,required=True);parser.add_argument('--report',type=Path);args=parser.parse_args()
    report=verify(args.candidate,args.out,args.fresh_review);target=args.report or args.out/'model-verification.json';target.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n');print(json.dumps(report))
if __name__=='__main__':main()
