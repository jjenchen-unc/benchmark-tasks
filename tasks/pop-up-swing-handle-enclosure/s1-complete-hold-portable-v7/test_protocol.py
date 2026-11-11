"""Synthetic contract checks only; never physical evidence or score transfer."""
from pathlib import Path
import sys,tempfile,json,copy,time
O=Path(__file__).resolve().parent;sys.path.insert(0,str(O));sys.path.insert(0,str(O/'candidate'))
from protocol import fresh,receipt,sha,check,CHANNELS,number
from strict_json import dumps
from rubric import History
start=time.monotonic();root=Path(tempfile.mkdtemp(prefix='CPU-SYNTHETIC-',dir=O/'work'));replay=root/'replay';replay.mkdir();hist=History({'goal':'open'});tests={}
row=dict(door=1.1,swing=1.9,pop=.55,wrist_force_n=[0.,0.],robot_contact=False,engaged=False,released=True,order='B',pop_catch=False)
with (replay/'physics.jsonl').open('w') as f:
 for i in range(401*50):hist.sample(row,.001);f.write(dumps(dict(control=i//50,measurement=row,status=hist.status()))+'\n')
with (replay/'controller-stream.jsonl').open('w') as f:
 for i in range(401):f.write(dumps(dict(control=i+1,action=[0.]*14))+'\n')
for name in ['top_cam','left_cam','right_cam']:
 for ext in ['png','mp4']:(replay/(name+'.'+ext)).write_bytes(b'SYNTHETIC CONTRACT PLACEHOLDER; NOT MEDIA EVIDENCE')
(replay/'media-finalization.json').write_text(dumps(dict(all_encoded=True,frames_per_camera=402)))
(replay/'result.json').write_text(dumps(dict(kind='ACTUAL_S1_RUNTIME',phase='replay',manifest=check(),completed=True,error=None,promoted_channels=[],final_status=hist.status())))
r=fresh(replay);tests['synthetic_complete_native_hold']=r['native_status']['hold_s']>=20.
path=root/'ROOT-APPROVED.json'
def refuse(fn):
 try:fn()
 except (ValueError,TypeError,FileNotFoundError,KeyError):return True
 return False
path.write_text(dumps(r));tests['unapproved_refused']=refuse(lambda:receipt(path))
r.update(reviewer='ROOT',approved=True)
for n,v in r['channels'].items():v.update(approved=True,rationale='SYNTHETIC TEST ONLY',evidence=[dict(file='replay/result.json',sha256=sha(replay/'result.json'))])
path.write_text(dumps(r));tests['synthetic_approved_contract']=receipt(path)['approved'] is True
for n in CHANNELS:
 bad=copy.deepcopy(r);bad['channels'][n]['approved']=False;path.write_text(dumps(bad));tests['refuse_channel_'+n]=refuse(lambda:receipt(path))
path.write_text(dumps(r));target=replay/'top_cam.png';original=target.read_bytes();target.write_bytes(b'changed');tests['changed_replay_refused']=refuse(lambda:receipt(path));target.write_bytes(original)
bad=copy.deepcopy(r);bad['manifest']='wrong';path.write_text(dumps(bad));tests['wrong_source_refused']=refuse(lambda:receipt(path));path.write_text(dumps(r))
original_stream=(replay/'physics.jsonl').read_bytes();(replay/'physics.jsonl').write_text('\n'.join(original_stream.decode().splitlines()[:19000])+'\n');tests['incomplete_native_hold_refused']=refuse(lambda:fresh(replay));(replay/'physics.jsonl').write_bytes(original_stream)
for val in [True,float('nan'),float('inf'),'1']:tests['strict_number_'+repr(val)]=refuse(lambda:number(val))
for val in [float('nan'),float('inf'),object()]:tests['strict_json_'+type(val).__name__+str(len(tests))]=refuse(lambda:dumps({'value':val}))
report=dict(passed=all(tests.values()),checks=tests,kind='CPU_SYNTHETIC_CONTRACT_ONLY',physical_evidence=False,synthetic_review=str(path),wall_s=time.monotonic()-start)
(O/'checks/protocol.json').write_text(dumps(report,indent=2)+'\n');print(dumps(report))
if not report['passed']:raise SystemExit(1)
