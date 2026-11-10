"""Explicit CPU-SYNTHETIC protocol tests; zero simulator/model/physical approvals."""
from pathlib import Path
import sys,tempfile,copy,time,json,ast
O=Path(__file__).resolve().parent;sys.path.insert(0,str(O))
import protocol as p
from strict_json import dumps
start=time.monotonic();m=p.check();work=O/'work';work.mkdir(exist_ok=True);f=Path(tempfile.mkdtemp(prefix='CPU-SYNTHETIC-',dir=work));tests=[]
def put(n,d): (f/n).write_text(dumps(d)+'\n')
status={'controls':0,'physics_samples':0,'physics_s':0.,'score':0.,'missing_runtime_channels':sorted(p.CHANNELS),'error':None,'measurement':{'nut_backoff_deg':61.,'arm_open_deg':90.,'latch_rounded':False,'arm_free':True,'contact_free':True},'tier':'none','candidate_score':0.,'history':{'rounded_any':False,'hold_full_s':0.,'hold_release_s':0.,'invalid_measurement_seen':False}}
rows=[]
for i in range(401):
 s=dict(status,controls=i,physics_samples=25*i,physics_s=i*.05,candidate_score=1. if i==400 else 0.,history={'rounded_any':False,'hold_full_s':20. if i==400 else 0.,'hold_release_s':20. if i==400 else 0.,'invalid_measurement_seen':False})
 rows.append({'kind':'ACTUAL_RUNTIME_OBSERVER','status':s,'physics_dt':.002,'control_dt':.05,'physics_tick':25*i,'action':[0.]*14 if i else None})
def journal(rs): (f/'journal.jsonl').write_text(''.join(dumps(r)+'\n' for r in rs))
launch={'phase':'replay','exitcode':0,'source_before':m,'source_after':m};result={'kind':'ACTUAL_B1_RUNTIME','phase':'replay','completed':True,'error':None,'manifest':m,'promoted_channels':[],'final_status':rows[-1]['status']}
put('launcher.json',launch);put('result.json',result);journal(rows)
media={'all_encoded':True,'frames_per_camera':401,'files':{}}
for n in ['top_cam.mp4','left_cam.mp4','right_cam.mp4']:
 (f/n).write_bytes(b'CPU-SYNTHETIC-NOT-VIDEO');media['files'][n]={'bytes':(f/n).stat().st_size}
put('media-finalization.json',media)
assert p.fresh(f)['controls']==400;tests.append('synthetic complete numeric branch only')
def rejected(name,fn):
 try:fn()
 except (ValueError,TypeError,KeyError,FileNotFoundError,IsADirectoryError):tests.append(name)
 else:raise AssertionError('accepted '+name)
bad=copy.deepcopy(rows);bad[1]['status']['controls']=True;journal(bad);rejected('boolean control rejected',lambda:p.fresh(f));journal(rows)
bad=copy.deepcopy(rows);bad[3]['status']['physics_samples']=74;journal(bad);rejected('wrong samples rejected',lambda:p.fresh(f));journal(rows)
bad=copy.deepcopy(result);bad['completed']=False;put('result.json',bad);rejected('failed actual rejected',lambda:p.fresh(f));put('result.json',result)
bad=copy.deepcopy(rows);bad[-1]['status']['history']['hold_full_s']=19.99;journal(bad);rejected('short native hold rejected',lambda:p.fresh(f));journal(rows)
bad=copy.deepcopy(result);bad['promoted_channels']=['passive_release'];put('result.json',bad);rejected('unproved runtime flag rejected',lambda:p.fresh(f));put('result.json',result)
bad=copy.deepcopy(launch);bad['source_after']={};put('launcher.json',bad);rejected('mixed source rejected',lambda:p.fresh(f));put('launcher.json',launch)
rejected('nonfinite JSON rejected',lambda:p.loads('{"v":NaN}'));rejected('overflow JSON rejected',lambda:p.loads('{"v":1e999}'));rejected('duplicate key rejected',lambda:p.loads('{"v":1,"v":2}'))
rejected('boolean numeric rejected',lambda:p.number(True));rejected('relative escape rejected',lambda:p.relative(f,'../outside'))
put('historical_fit545.json',p.load(O/'provenance/review.json'));rejected('historical_fit545 rejected as receipt',lambda:p.receipt(f/'historical_fit545.json'))
put('unapproved.json',{'schema':'b1-v8-root-physical-review','approved':False});rejected('unapproved ROOT form rejected',lambda:p.receipt(f/'unapproved.json'))
# Bind a complete synthetic replay and channel proof in memory; no approved file.
import shutil
reviewdir=Path(tempfile.mkdtemp(prefix='CPU-SYNTHETIC-REVIEW-',dir=work));shutil.copytree(f,reviewdir/'replay')
(reviewdir/'source-proof.txt').write_text('CPU-SYNTHETIC-NOT-PHYSICAL')
proof={'path':'source-proof.txt','sha256':p.sha(reviewdir/'source-proof.txt')}
review={'schema':'b1-v8-root-physical-review','approved':True,'reviewer':'ROOT','manifest':m,'replay':'replay','actual_summary':p.fresh(reviewdir/'replay'),'evidence_sha256':{str(x.relative_to(reviewdir/'replay')):p.sha(x) for x in (reviewdir/'replay').rglob('*') if x.is_file()},'historical_fit545_is_full_success':False,'mounted_support_approved':True,'channels':{c:{'approved':True,'rationale':'CPU synthetic validation branch, no physical approval','proofs':[proof]} for c in p.CHANNELS}}
original_load=p.load;memory_path=reviewdir/'MEMORY-ONLY-NOT-WRITTEN.json'
p.load=lambda path:copy.deepcopy(review) if Path(path)==memory_path else original_load(path)
assert p.receipt(memory_path)['actual_summary']['controls']==400;tests.append('synthetic in-memory approved receipt validates; no runtime credit')
review['channels']['passive_release']['approved']=False;rejected('unproved retention remains sealed',lambda:p.receipt(memory_path));review['channels']['passive_release']['approved']=True
(reviewdir/'source-proof.txt').write_text('changed');rejected('changed channel proof rejected',lambda:p.receipt(memory_path))
p.load=original_load
# Source parses on host/CPU, no imports of actual SDK or providers.
for n in ['runtime.py','model_child.py','model_task.py','verify_model.py']:ast.parse((O/n).read_text(),filename=n)
tests.append('chosen entries syntax checked')
report={'synthetic':True,'physics_proof':False,'approved_physical_review_written':False,'passed':True,'tests':tests,'wall_s':time.monotonic()-start,'fixture':str(f)}
(O/'checks').mkdir(exist_ok=True);(O/'checks/protocol-tests.json').write_text(dumps(report,indent=2)+'\n');print(dumps(report))
