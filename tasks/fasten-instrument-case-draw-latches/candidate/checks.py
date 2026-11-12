"""Pure numeric/scorer/source safety checks. No simulation or physics evidence."""
import ast,copy,json,math
from pathlib import Path
from scoring import History
HERE=Path(__file__).resolve().parent
checks={}
def base():return dict(lid=0.,lever=[0.,0.],slide=[0.,0.],catch=[0.,0.],engaged=[0,0],robot_contact=False,case_tilt_deg=[0.,0.],instrument_tilt_deg=[0.,0.],wrist_n=[0.,0.],nonfinger_n=[0.,0.])
def sample(row,seconds=.002):
 h=History()
 for i in range(round(seconds/.002)):h.update(row,.002)
 return h
r=base();r['engaged']=[-1,-1];h=sample(r,22.);checks['wave13_unattached_counterexample']=h.score()==.25 and h.hold==0
for j in [0,1]:
 r=base();r['engaged'][j]=-1;checks[f'only_one_fastener_{j}']=sample(r,22.).score()==.5
for bar in [0,1,2]:
 r=base();r['engaged']=[bar,bar];checks[f'any_keeper_{bar}']=sample(r,20.).score()==1
r=base();r['lid']=math.radians(2)+1e-8;checks['lid_outside_boundary']=sample(r).score()==0
r=base();r['lid']=math.radians(2);checks['lid_exact_boundary']=sample(r).score()==.75
for field,value in [('lever',.0500001),('catch',.0015001),('slide',-.0025001)]:
 r=base();r[field][0]=value;checks[field+'_bad_fastener']=sample(r).score()==.5
r=base();r['robot_contact']=True;checks['robot_touch_no_partial']=sample(r,22.).score()==0
h=sample(base(),19.998);checks['no_early_fullscore']=h.score()==.75;h.update(base(),.002);checks['hold_full_at20']=h.score()==1
r=base();r['robot_contact']=True;h.update(r,.002);checks['contact_breaks_hold']=h.score()==0 and h.hold==0;h.update(base(),.002);checks['hold_restarts']=h.score()==.75 and abs(h.hold-.002)<1e-12
for field,threshold in [('case_tilt_deg',15),('instrument_tilt_deg',20)]:
 for index in [0,1]:
  r=base();r[field][index]=threshold+1e-6;h=sample(r);h.update(base(),.002);checks[f'persistent_{field}_{index}']=h.score()==0 and h.failed is not None
 r=base();r[field][0]=threshold;checks[field+'_exact_limit_allowed']=sample(r).score()==.75
r=base();r['wrist_n']=[80.,80.];checks['80N_exact_not_abort']=sample(r,.11).failed is None
r['wrist_n']=[80.001,0.];checks['under_100ms_no_abort']=sample(r,.098).failed is None;checks['100ms_abort']=sample(r,.1).failed=='wrist_force_abort'
r=base();r['wrist_n']=[60.001,0.];checks['20ms_force_stop']=sample(r,.02).guard_stopped[0]
r=base();r['nonfinger_n']=[50.001,0.];checks['20ms_nonfinger_stop']=sample(r,.02).guard_stopped[0]
r=base();r['lid']=float('nan')
try:sample(r);checks['nan_rejected']=False
except ValueError:checks['nan_rejected']=True
for p in HERE.glob('*.py'):
 ast.parse(p.read_text());checks['syntax_'+p.name]=True
# Teacher may only call the embodiment through actual Actions.
s=(HERE/'reference.py').read_text();checks['reference_no_task_writes']=not any(x in s for x in ['write_joint','write_root','set_joint','set_external','engaged[',".eq["] if x!='engaged[') and '.step(Action(data=' in s
checks['runtime_no_native_imports']=not any('import mujoco' in (HERE/p).read_text() for p in ['task.py','kit.py','physics.py','reference.py','scoring.py'])
checks['model_observation_private_state_not_added']='obs.state[' not in (HERE/'kit.py').read_text()
report=dict(kind=__doc__,checks=checks,all_passed=all(checks.values()),count=len(checks));print(json.dumps(report,indent=2))
if not report['all_passed']:raise SystemExit(1)
