"""Pure source/parity/construct checks. No physics, app, rendering or GPU."""
import argparse,ast,copy,hashlib,json,sys,types
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
from c1_contract import *
from c1_history import History

def run(native_source=None):
 checks={};details={};rows=[]
 # Hash + syntax checks include every executable source, without importing Kit.
 files=sorted(HERE.glob('*.py'))
 for p in files:ast.parse(p.read_text(),filename=str(p))
 checks['all_source_parses']=True
 checks['proper_station_transform']=bool(np.allclose(A.T@A,np.eye(3)) and np.isclose(np.linalg.det(A),1.) and all(np.allclose(native(world(x)),x) for x in [[-.3,-.4,.8],[.35,.4,1.18]]))
 checks['quantitative_contract']=BUDGET==2268 and CONTROL_HZ==20 and DT==.001 and abs(ANGLE-np.radians(20))<1e-15
 R=np.column_stack([A[:,0],U_SLOPE,N_RACK]);cup0=RACK_TOP_C+N_RACK*.041
 def frame(edges=((10,20),(20,30)),wet=False):
  return {'contacts':list(edges),'cup_pos':[cup0.copy(),CUP_START[1].copy()],'cup_R':[R.copy(),np.eye(3)],'liner_pos':[RACK_TOP_C.copy() for _ in range(6)],'liner_R':[R.copy() for _ in range(6)],'wet':[wet]*6,'wrist_force_n':[0.,0.]}
 def advance(h,f,n):
  for _ in range(n):h.update(f,.001)
 def case(label,edges,expected,target='C1-T1',wet=False):
  h=History(target);f=frame(edges,wet);advance(h,f,20600);score=h.score()['score'];rows.append({'case':label,'score':score,'expected':expected});checks[label]=bool(abs(score-expected)<1e-12);return h,f
 case('hands_off_full',[(10,20),(20,30)],1.)
 case('direct_robot_glass',[(1,10),(10,20),(20,30)],.25)
 case('direct_robot_liner',[(1,20),(10,20),(20,30)],0.)
 case('robot_spare_liner_glass',[(1,21),(21,10),(10,20),(20,30)],.25)
 case('robot_spare_liner_liner',[(1,21),(21,20),(10,20),(20,30)],0.)
 case('static_table_not_mobile_support',[(1,31),(20,31),(10,20),(20,30)],0.)
 case('two_liner_glass_no_hold',[(10,20),(10,21),(20,30),(21,30)],.25)
 case('wet_goal_dry_liner',[(10,20),(20,30)],0.,target='C1-T2')
 case('wet_goal_wet_liner',[(10,20),(20,30)],1.,target='C1-T2',wet=True)
 h,f=case('before_drift',[(10,20),(20,30)],1.);f['cup_pos'][0]=cup0+U_SLOPE*.00501;advance(h,f,1);f['cup_pos'][0]=cup0.copy();advance(h,f,20600);checks['drift_void_no_hands_off_recovery']=h.void[0] and h.score()['score']==.25
 f['contacts'].append((1,10));advance(h,f,1);f['contacts'].remove((1,10));advance(h,f,20600);checks['new_robot_placement_recovers']=h.score()['score']==1.
 h=History();f=frame();advance(h,f,499);checks['settle_not_full_hold']=h.hold[0]==0 and h.score()['score']==.5
 advance(h,f,10002);checks['ten_second_partial']=h.score()['score']==.75
 h=History();f=frame();advance(h,f,600);f['cup_R'][0]=np.diag([1.,-1.,-1.]);advance(h,f,1);checks['tip_veto']=h.cup_fell and h.score()['score']==0.
 h=History();f=frame([(1,10)]);f['cup_pos'][0]=cup0+[0,0,.1];advance(h,f,1);f['contacts']=[(10,31)];f['cup_pos'][0]=cup0;advance(h,f,1);checks['30mm_drop_veto']=h.cup_fell and h.score()['score']==0.
 h=History();f=frame();f['wrist_force_n']=[80.,0.];advance(h,f,200);checks['80N_exact_no_abort']=h.failure is None;f['wrist_force_n']=[80.001,0.];advance(h,f,99);checks['99ms_no_abort']=h.failure is None;advance(h,f,1);checks['100ms_abort']=h.failure=='wrist_force_limit'
 h=History('C1-T3');f=frame([(10,20),(20,30),(11,21),(21,30)]);f['cup_pos'][1]=cup0+[.1,0,0];f['cup_R'][1]=R.copy();advance(h,f,20600);checks['two_distinct_simultaneous']=h.score()['score']==1.
 f['contacts']=[(10,20),(20,30),(11,20)];advance(h,f,1);checks['no_two_cups_same_liner_fullcredit']=h.score()['score']<=.5
 # Public reference actions are the sole actuation path. Evaluator is separate.
 ref=ast.parse((HERE/'reference.py').read_text());calls=[n for n in ast.walk(ref) if isinstance(n,ast.Call)]
 banned=('write_root','write_joint','set_external','set_joint_effort','set_mu','initialize_episode','permanent_wrench','qpos','qvel')
 checks['reference_no_task_state_writes']=not any(isinstance(n.func,ast.Attribute) and any(x in n.func.attr for x in banned) for n in calls)
 checks['reference_real_Action']=any(isinstance(n.func,ast.Name) and n.func.id=='Action' for n in calls)
 checks['no_fake_task_json']=not(HERE/'task.json').exists()
 if native_source:
  # Execute the ACTUAL source predicates against fake data, not physics. This
  # is a truth-table oracle; mj_step is a no-op and clearly labelled as such.
  native_source=Path(native_source);text=native_source.read_text();tree=ast.parse(text)
  names={'sample_liners','_u','parameters','transfer_parameters','robot_supported_objects','step','score_detail'};nodes=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in names]
  ns={'np':np,'N_LINER':6,'N_CUP':2,'MU_HOLD':(.46,1.),'MU_SLIP':(.15,.27),'GRAY':(.31,.41),'UPRIGHT_DEG':15.,'HOLD_S':20.,'SETTLE_S':.5,'MOVE_TOL':.005,'DROP_TOL':.030,'TIP_DEG':60.,'RACK_LOW_Z':RACK_LOW_Z,'N_RACK':N_RACK,'PAN_C':np.array([.05,.035]),'PAN_IN':np.array([.07,.085]),'WATER_TOP':-.1,'mujoco':types.SimpleNamespace(mj_step=lambda m,d:None)}
  exec(compile(ast.Module(body=nodes,type_ignores=[]),str(native_source),'exec'),ns)
  checks['source_parameter_distribution_exact']=all(parameters(s)==ns['parameters'](s) and parameters(s,True)==ns['transfer_parameters'](None,s) for s in [42,43,44,100,901])
  parity=[]
  for label,edges in [('ordinary',[(10,20),(20,30)]),('indirect_brace',[(1,21),(21,10),(10,20),(20,30)]),('chain',[(1,22),(22,21),(21,20),(10,20),(20,30)]),('table_robot',[(1,31),(10,20),(20,30)])]:
   h=History();f=frame(edges);T=set(edges)|{(b,a) for a,b in edges};k={'v':[[],[]],'cups':[0,1],'liners':list(range(2,8))};state={'ids':k,'wet':[True]*6,'ok_t':[0.]*2,'ref':[None]*2,'void':[False]*2,'hold':[0.]*2,'on_liner':[-1]*2,'was_on_rack':[False]*2,'release_z':[None]*2,'cup_fell':False,'fell_reason':None,'need_cups':1,'need_wet':False};model=types.SimpleNamespace(opt=types.SimpleNamespace(timestep=.001));data=types.SimpleNamespace(qfrc_applied=np.zeros(0),qfrc_bias=np.zeros(0),xpos=np.array([*f['cup_pos'],*f['liner_pos']]),xmat=np.array([*f['cup_R'],*f['liner_R']]).reshape(8,9));ns['contacts']=lambda m,d,k:T
   for _ in range(20600):ns['step'](model,data,state);h.update(f,.001)
   # Native score_detail also expects the six liner body IDs above.
   old=ns['score_detail'](model,data,state,'C1-T1');new=h.score();ok=all(old[key]==new[key] for key in old);parity.append({'case':label,'match':ok,'native':old,'arena_pure':new})
  checks['repaired_native_predicate_parity']=all(p['match'] for p in parity);details['native_predicate_parity']=parity;details['native_bench_sha256']=hashlib.sha256(native_source.read_bytes()).hexdigest()
 return {'kind':'PURE_STATIC_SYNTHETIC_NO_PHYSICS','checks':{k:bool(v) for k,v in checks.items()},'all_checks_passed':all(checks.values()),'cases':rows,'details':details,'source_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in files}}
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--native-source',type=Path);ap.add_argument('--out',type=Path);a=ap.parse_args();r=run(a.native_source);s=json.dumps(r,indent=2)+'\n'
 if a.out:a.out.write_text(s)
 print(json.dumps({'checks':r['checks'],'passed':r['all_checks_passed']}));sys.exit(0 if r['all_checks_passed'] else 1)
