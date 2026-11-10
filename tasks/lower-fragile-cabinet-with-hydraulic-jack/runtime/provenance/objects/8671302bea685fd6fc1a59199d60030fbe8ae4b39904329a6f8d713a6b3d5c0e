"""Meaningful pure counterexamples. Synthetic rows are NOT backend evidence."""
import ast,copy,json,math
from pathlib import Path
from mechanism import Hydraulic
from scoring import History,REQUIRED
HERE=Path(__file__).resolve().parent;F=json.loads((HERE/'fixture.json').read_text());checks={}
def row(**kw):
 d=dict(height=.05,vz=0.,speed=0.,spin=0.,tilt=0.,glass_force=0.,wrist_force=[0.,0.],flow_lowered=0.,on_rails=False,on_pad=True,robot_contact=False);d.update(kw);return d
def feed(h,r,n=1):
 for _ in range(n):h.update(r,.002)
def landing(speed=.010):
 h=History();feed(h,row());feed(h,row(height=.0002,vz=-speed,flow_lowered=.0498));feed(h,row(height=0.,vz=0.,flow_lowered=.05,on_rails=True));return h
h=landing();feed(h,row(height=0,flow_lowered=.05,on_rails=True),1498);checks['incomplete_hold_not_full']=h.result(REQUIRED)['score']==.75
feed(h,row(height=0,flow_lowered=.05,on_rails=True),1);checks['three_second_hold_full']=h.result(REQUIRED)['score']==1.
checks['unvalidated_positive_credit_zero']=h.result()['score']==0.
h=landing();feed(h,row(height=0,flow_lowered=.05,on_rails=True,speed=.01),1600);checks['moving_on_rails_no_full_hold']=h.result(REQUIRED)['score']==.75
h=landing(.026);feed(h,row(height=0,flow_lowered=.05,on_rails=True),1600);checks['hard_landing_persistent_zero']=h.result(REQUIRED)['score']==0 and h.failure=='hard_landing'
h=landing();feed(h,row(glass_force=15.001));feed(h,row(),10);checks['glass_contact_irreversible']=h.failure=='glass_damage' and h.result(REQUIRED)['score']==0
h=History();feed(h,row(height=.01,flow_lowered=.04,robot_contact=True));checks['hand_descent_no_partial']=h.result(REQUIRED)['score']==0.
h=History();feed(h,row(height=.0002,vz=-.01,on_pad=False,robot_contact=True));feed(h,row(height=0,on_pad=False,on_rails=True),1600);checks['direct_hand_landing_not_full_or_partial']=h.result(REQUIRED)['score']==0.
h=History();feed(h,row(height=.035,flow_lowered=.015));checks['safe_10mm_partial']=h.result(REQUIRED)['score']==.25
h=History();feed(h,row(height=.015,flow_lowered=.035));checks['safe_30mm_partial']=h.result(REQUIRED)['score']==.5
for key,r in [('drop',row(height=-.05)),('tip',row(tilt=math.radians(11))),('nonfinite',row(height=float('nan'))),('wrist',row(wrist_force=[31.,0.]))]:
 h=History();feed(h,r);checks[key+'_zero']=h.failure is not None and h.result(REQUIRED)['score']==0
h=Hydraulic(F['parameters'],'S');V=h.V;h.update(.2,-.2,0,80.,.002);checks['closed_handle_pumps']=h.V>V
V=h.V
for _ in range(100):h.update(.2,0,0,80.,.002)
checks['stationary_handle_no_pump']=h.V==V
V=h.V;h.update(.2,0,math.radians(F['parameters']['phi0_S']+30),50.,.002);checks['valve_flow_source_law']=abs(V-h.V-F['parameters']['v30_S']*.001*.002)<1e-12
# Teacher must contain no task manipulation primitives.
t=ast.parse((HERE/'reference.py').read_text());bad={'write_joint_state_to_sim','write_root_pose_to_sim_index','set_external_force_and_torque','set_joint_effort_target','set_joint_position_target'}
checks['teacher_no_object_write_calls']=not any(isinstance(n,ast.Attribute) and n.attr in bad for n in ast.walk(t))
checks['native_no_runtime_imports']=all('import mujoco' not in (HERE/n).read_text() for n in ['kit.py','task.py','physics.py','reference.py','foundation.py'])
print(json.dumps(dict(kind='pure synthetic counterexamples only; not physics proof',checks=checks,all_pass=all(checks.values())),indent=2))
raise SystemExit(0 if all(checks.values()) else 1)
