"""CPU SYNTHETIC protocol failures and real installed TaskFile double-load only."""
from pathlib import Path
import sys,json,tempfile,time,runpy,os
O=Path(__file__).resolve().parent;sys.path.insert(0,str(O));sys.path.insert(0,str(O/'candidate'))
import protocol as p
from strict_json import dumps
t=time.monotonic();checks={}
def reject(name,fn):
 try:fn()
 except (ValueError,TypeError,FileNotFoundError,IsADirectoryError,KeyError):checks[name]=True
 else:raise AssertionError(name)
with tempfile.TemporaryDirectory(dir=O/'checks') as tmp:
 root=Path(tmp)
 for name,data in [('duplicate','{"a":1,"a":2}'),('nan','{"a":NaN}'),('overflow','{"a":1e999}')]:
  f=root/name;f.write_text(data);reject(name,lambda:p.load(f))
 reject('numeric_bool',lambda:p.numeric(True));reject('unknown_json',lambda:dumps(object()))
 reject('missing_review',lambda:p.receipt(root/'absent'))
 f=root/'review.json';f.write_text(dumps({'kind':'HYDRAULIC_ROOT_PHYSICAL_REVIEW','approved':False,'reviewer':'ROOT','manifest':p.check()}));reject('unapproved_review',lambda:p.receipt(f))
 # Exercise the complete source/history protocol using explicitly invented rows.
 import scoring
 trial=root/'synthetic';trial.mkdir();h=scoring.History();samples=[];trace=[];action=[0.]*14
 for i in range(1,81):
  trace.append({'phase':'before_action','tick':i-1,'action':action})
  for j in range(25):
   initial=i==1 and j==0
   row={'height':.001 if initial else 0.,'vz':-.001 if initial else 0.,'tilt':0.,'glass_force':0.,'wrist_force':[0.,0.],'flow_lowered':.05,'speed':0.,'spin':0.,'on_rails':not initial,'on_pad':initial,'robot_contact':False}
   h.update(row,.002);samples.append({'settling':False,'dt':.002,'row':row,'history':{'failure':None}})
  status=h.result();status.update(controls=i,measurement_error=None,last=row,physics_device='CPU_SYNTHETIC')
  trace.append({'phase':'after_action','tick':i,'action':action,'robots':{s:{'raw8':[[0.]*8]} for s in ('left','right')},'status':status})
 result={'kind':'ACTUAL_HYDRAULIC_RUNTIME','phase':'replay','completed':True,'error':None,'manifest':p.check(),'final':status,'synthetic_test_fixture':True}
 def write_result():(trial/'result.json').write_text(dumps(result))
 write_result();(trial/'launcher.json').write_text(dumps({'exitcode':0,'source_before':p.check(),'source_after':p.check()}))
 (trial/'trace.jsonl').write_text(''.join(dumps(x)+'\n' for x in trace));(trial/'physics.jsonl').write_text(''.join(dumps(x)+'\n' for x in samples))
 (trial/'media-finalization.json').write_text(dumps({'all_encoded':True,'frames_per_camera':81,'encoder_returncodes':{n:0 for n in ('left_cam','right_cam','top_cam')}}))
 for n in ('left_cam','right_cam','top_cam'):(trial/(n+'.mp4')).write_bytes(b'EXPLICITLY SYNTHETIC TEST ONLY')
 checks['synthetic_complete_protocol']=p.fresh(trial)['native_diagnostic']==1
 status['hold_s']=2.9;write_result();reject('short_native_hold',lambda:p.fresh(trial));status['hold_s']=h.hold
 status['controls']=True;write_result();reject('bool_controls',lambda:p.fresh(trial));status['controls']=80
 status['physics_samples']=1999;write_result();reject('sample_gap',lambda:p.fresh(trial));status['physics_samples']=2000
 status['hold_s']+=.01;write_result();reject('recomputed_history_mismatch',lambda:p.fresh(trial));status['hold_s']=h.hold
 status['score']=1;write_result();reject('uncredited_replay_promoted',lambda:p.fresh(trial));status['score']=0;write_result()
 status['score']=False;write_result();reject('bool_score',lambda:p.fresh(trial));status['score']=0;write_result()
 samples[0]['row']['glass_force']=False;(trial/'physics.jsonl').write_text(''.join(dumps(x)+'\n' for x in samples));reject('bool_physical_force',lambda:p.fresh(trial))
 # Real model child must reject before any simulator or provider import.
 events=[]
 class Denied(BaseException):pass
 def audit(event,args):
  if event=='import' and str(args[0]).split('.')[0] in {'isaaclab','isaacsim','omni','carb','pxr','inspect_robots_dream'}:events.append(args[0]);raise Denied(args[0])
 sys.addaudithook(audit);sys.argv=[str(O/'model_child.py'),'--out',str(root/'never'),'--review',str(f)]
 reject('actual_model_missing_review',lambda:runpy.run_path(str(O/'model_child.py'),run_name='__main__'));checks['no_provider_or_sdk_import']=not events
 # Conditional API-only fixture: an in-memory substitute, never a production receipt.
 p.receipt=lambda path:{'kind':'CPU_SYNTHETIC_NOT_PHYSICAL'}
 from yam_arena.task_file import load
 one=load(O/'model_task.py');two=load(O/'model_task.py')
 checks['actual_taskfile_double_load']=len(one.objects)==len(two.objects)==14
 checks['task_helper_origin']=Path(sys.modules['kit'].__file__).resolve()==O/'candidate/kit.py'
 import physics
 checks['status_adapter_idempotent']=physics.Physics._root_review_adapter is True
 checks['fixed_physical_seed']=json.loads((O/'candidate/fixture.json').read_text())['seed']==42
 checks['native_channel_contract']=set(p.CHANNELS)==set(__import__('scoring').REQUIRED)
r={'passed':all(checks.values()),'checks':checks,'wall_s':time.monotonic()-t,'evidence_class':'CPU_SYNTHETIC protocol and conditional actual TaskFile API; no physical review or credit','simulation_starts':0,'model_calls':0}
(O/'checks/cpu.json').write_text(dumps(r,indent=2)+'\n');print(dumps(r))
if not r['passed']:raise SystemExit(1)
