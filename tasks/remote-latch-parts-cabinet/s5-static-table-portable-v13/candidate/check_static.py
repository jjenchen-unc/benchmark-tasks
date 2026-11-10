"""Pure/static counterexamples, native algebra comparison, USD-independent checks.
No native scene build, simulation step, GPU, model call or SDK mutation.
"""
import copy,hashlib,json,math,runpy,sys
from pathlib import Path
import numpy as np
from s5_rules import History,Latches,torque,DEG
HERE=Path(__file__).resolve().parent
source=runpy.run_path(str(HERE/'source/benches/S5_remote_release.py'));checks={}
errors=[]
for th in np.linspace(-.15*DEG,140*DEG,350):
 for two in [False,True]:
  for latch in ['open','stage1','stage2']:
   for armed in [False,True]:errors.append(abs(torque(th,24.682534696028178,two,latch,armed)-source['door_torque'](th,24.682534696028178,two,latch,armed)))
checks['exact_native_torque_algebra']=max(errors)<1e-12
F=json.loads((HERE/'fixture.json').read_text());L=Latches(F['cabinets']);q={j['name']:0. for b in F['bodies'] for j in b['joints']};q['A_paddle2_slide']=.008;L.update(q,0.);checks['8mm_does_not_release']=L.latch['AR']=='stage2';q['A_paddle2_slide']=.009;L.update(q,.002);checks['9mm_releases_right_only']=L.latch['AR']=='open' and L.latch['AL']=='stage2';q['A_paddle2_slide']=0.;q['A_hinge_R']=1*DEG;L.update(q,.004);checks['released_unarmed_stays_open']=L.latch['AR']=='open';q['A_hinge_R']=3*DEG;L.update(q,.006);q['A_hinge_R']=1.3*DEG;L.update(q,.008);checks['stage1_catch']=L.latch['AR']=='stage1';q['A_hinge_R']=.09*DEG;L.update(q,.01);checks['stage2_catch']=L.latch['AR']=='stage2'
def row():return dict(wrist_force_n=[0.,0.],door_q={'AL':0.,'AR':10*DEG,'BL':0.,'BR':0.},door_v={k:0. for k in ['AL','AR','BL','BR']},released=True,box=dict(z=.7775,upright_cos=1.,spatial_speed=0.,outside_cabinet=True,inside_table=True,table_contact=True,robot_contact=False))
h=History();r=row()
for _ in range(9999):h.add(r,.002)
checks['before20s_not_full']=h.result()['diagnostic_score']==.5;h.add(r,.002);checks['exact20s_full_diagnostic']=h.result()['diagnostic_score']==1. and h.result()['score']==0
bad=copy.deepcopy(r);bad['box']['robot_contact']=True;h.add(bad,.002);checks['touch_resets_hold']=h.hold==0
h=History();bad=copy.deepcopy(r);bad['wrist_force_n']=[81.,0.]
for _ in range(49):h.add(bad,.002)
checks['98ms_no_abort']=h.failure is None;h.add(bad,.002);checks['100ms_abort']=h.failure=='wrist_overload';h.add(r,.002);checks['damage_sticky_zero']=h.result()['diagnostic_score']==0
h=History();h.add(r,.002);slam=copy.deepcopy(r);slam['door_q']['AR']=-.14*DEG;h.add(slam,.002);checks['frame_slam_veto']=h.failure=='door_slam'
h=History();off=copy.deepcopy(r);off['box']['inside_table']=False;off['box']['table_contact']=False
for _ in range(10001):h.add(off,.002)
checks['off_table_never_full']=h.hold==0 and h.result()['diagnostic_score']<.5
# Exact immutable source box predicate accepts an impossible unsupported box at
# x10m if its centre height/orientation/velocity are supplied. The new predicate
# requires physical tabletop footprint + support contact. This is a static
# function counterexample, not an object teleport in a policy or physics trial.
class M:pass
class D:pass
d=D();d.xpos=np.array([[10.,-.29,.83]]);d.xmat=np.array([[1,0,0,0,1,0,0,0,1.]]);d.cvel=np.zeros((1,6));native=source['box_on_table'](M(),d,{'box':{'AR':{'body':0}}},'AR');checks['native_off_table_counterexample']=bool(native)
rng=np.random.default_rng(5);K=np.array([80,80,80,10,10,10]);cap=np.array([27,27,27,7,7,7]);pd=rng.uniform(-50,50,(100,6));G=rng.uniform(-20,20,(100,6));offset=G/K;checks['total_effort_not_residual_identity']=bool(np.allclose(np.clip(pd+K*offset,-cap,cap),np.clip(pd+G,-cap,cap)))
checks['source_exact_copy_hash']=hashlib.sha256((HERE/'source/benches/S5_remote_release.py').read_bytes()).hexdigest()==F['source_sha256']
checks={k:bool(v) for k,v in checks.items()}
print(json.dumps(dict(checks=checks,all_pass=all(checks.values()),native_builds=0,physics_steps=0,runtime_channels_validated=[],source_off_table_counterexample=dict(native=True,new_final=False,x_m=10.)),indent=2));sys.exit(0 if all(checks.values()) else 1)
