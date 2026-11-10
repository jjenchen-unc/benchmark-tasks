"""CPU-SYNTHETIC contract tests. Fabricated fixtures are never physical evidence."""
from pathlib import Path
import sys,tempfile,copy,json,time
O=Path(__file__).resolve().parent;sys.path.insert(0,str(O));sys.path.insert(0,str(O/'candidate'))
from protocol import check,fresh,receipt,sha,CHANNELS
from strict_json import dumps
from s5_rules import History
start=time.monotonic();root=Path(tempfile.mkdtemp(prefix='CPU-SYNTHETIC-',dir=O/'work'));f=root/'replay';f.mkdir();m=check()
write=lambda p,v:p.write_text(dumps(v)+'\n')
measure=dict(wrist_force_n=[0.,0.],door_q=dict(AL=0.,AR=.2,BL=0.,BR=0.),door_v=dict(AL=0.,AR=0.,BL=0.,BR=0.),released=True,box=dict(z=.7775,upright_cos=1.,spatial_speed=0.,outside_cabinet=True,inside_table=True,table_contact=True,robot_contact=False))
h=History();rows=[];trace=[dict(event='begin',clock_dt=.002)]
def journal(i):return dict(kind='ACTUAL_RUNTIME_OBSERVER',status=dict(h.result(),controls=i,measurement_error=None,mechanism_events=[],physics_verified=False),action=None if i==0 else [0.]*14,physics_tick=100+25*i,physics_dt=.002,control_dt=.05,robots={side:dict(raw8=[0.]*8,tcp_fk_measured_error_m=0.) for side in ('left','right')},observer=measure)
rows.append(journal(0))
for i in range(1,401):
 for _ in range(25):
  h.add(measure,.002);trace.append(dict(dt=.002,measurement=measure,physics_s=h.time,status=h.result()))
 rows.append(journal(i))
(f/'journal.jsonl').write_text(''.join(dumps(r)+'\n' for r in rows));(f/'physics.jsonl').write_text(''.join(dumps(r)+'\n' for r in trace))
launch=dict(phase='replay',exitcode=0,source_before=m,source_after=m)
result=dict(kind='ACTUAL_S5_RUNTIME',phase='replay',completed=True,error=None,manifest=m,promoted_channels=[],final_status=rows[-1]['status'])
write(f/'launcher.json',launch);write(f/'result.json',result)
for n in ('top_cam','left_cam','right_cam'):(f/(n+'.mp4')).write_bytes(b'SYNTHETIC-NOT-VIDEO')
write(f/'media-finalization.json',dict(all_encoded=True,frames_per_camera=len(rows),files={n+'.mp4':dict(bytes=(f/(n+'.mp4')).stat().st_size) for n in ('top_cam','left_cam','right_cam')}))
summary=fresh(f);passed=['synthetic baseline validates contract only']
def rejects(name,fn):
 try:fn()
 except (ValueError,KeyError,FileNotFoundError,TypeError):passed.append(name)
 else:raise AssertionError('accepted '+name)
for name,key,value in [('synthetic','synthetic',True),('partial','completed',False),('error','error','failed'),('source mismatch','manifest',{}),('credit flag only','promoted_channels',['clock'])]:
 changed=dict(result,**{key:value});write(f/'result.json',changed);rejects(name,lambda:fresh(f));write(f/'result.json',result)
changed=copy.deepcopy(rows);changed[-1]['status']['hold_s']=0.;(f/'journal.jsonl').write_text(''.join(dumps(r)+'\n' for r in changed));rejects('incomplete hold',lambda:fresh(f));(f/'journal.jsonl').write_text(''.join(dumps(r)+'\n' for r in rows))
original=(f/'physics.jsonl').read_text();(f/'physics.jsonl').write_text('');rejects('missing native substeps',lambda:fresh(f));(f/'physics.jsonl').write_text(original)
write(f/'launcher.json',dict(launch,exitcode=True));rejects('bool exitcode numeric ownership',lambda:fresh(f));write(f/'launcher.json',launch)
from protocol import loads
for literal in ('NaN','1e999','{"a":1,"a":2}'):
 rejects('strict JSON '+literal,lambda:loads(literal))
proof=root/'channel-proof.txt';proof.write_text('CPU SYNTHETIC ONLY')
r=dict(schema='s5-v10-root-physical-review',approved=True,reviewer='ROOT',manifest=m,replay='replay',actual_summary=summary,evidence_sha256={str(p.relative_to(f)):sha(p) for p in f.rglob('*') if p.is_file()},historical_scores_transferred=False,mounted_support_approved=True,channels={c:dict(approved=True,rationale='CPU synthetic validator test',proofs=[dict(path='channel-proof.txt',sha256=sha(proof))]) for c in CHANNELS})
review=root/'CPU-SYNTHETIC-REVIEW.json';write(review,r);receipt(review);passed.append('synthetic receipt baseline validates binding only')
for name,key,value in [('unapproved','approved',False),('old score transfer','historical_scores_transferred',True),('mount unreviewed','mounted_support_approved',False),('channel evidence flag only','channels',{c:dict(approved=True,rationale='flag',proofs=[]) for c in CHANNELS})]:
 write(review,dict(r,**{key:value}));rejects(name,lambda:receipt(review))
write(review,r);proof.write_text('changed');rejects('changed proof',lambda:receipt(review))
# Remove test-only approved form so it cannot be mistaken for operator review.
review.unlink()
report=dict(passed=True,synthetic=True,physical_proof=False,tests=passed,wall_s=time.monotonic()-start,approved_receipt_delivered=False)
write(O/'checks/protocol.json',report);print(dumps(report))
