"""Pure math/source/schema counterexamples; does not establish backend credit."""
import ast,json,math,sys
from pathlib import Path
from contract import History,Mechanism
HERE=Path(__file__).resolve().parent
checks={}
def row(**kw):
 d=dict(leg=[.075,.075],pin=[0.,0.],wrist=[10.,10.],acceleration_g=0.,leg_velocity=[0.,0.],robot_contact=False);d.update(kw);return d
def hold(h,r,n=500):
 for _ in range(n):h.update(.002,r)
 return h.result()
h=History();r=hold(h,row());checks['full_diagnostic_after1s']=r['diagnostic_score']==1;checks['static_cannot_enable_credit']=r['score']==0 and r['gate'] is not None
h=History();checks['partial_at_hole2']=hold(h,row(leg=[.05,.05]))['diagnostic_score']==.5
checks['off_goal_zero']=hold(History(),row(leg=[.025,.025]))['diagnostic_score']==0
for k,v in [('robot_contact',True),('pin',[-.004,0]),('leg',[.075,.05])]:checks[k+'_veto']=hold(History(),row(**{k:v}))['diagnostic_score']==0
h=History();hold(h,row());h.update(.002,row(robot_contact=True));checks['regrasp_resets_terminal_hold']=h.result()['diagnostic_score']==0
h=History();hold(h,row());h.update(.002,row(leg_velocity=[-.201,0]));checks['drop_permanent_veto']=hold(h,row())['diagnostic_score']==0 and h.failure=='plate_drop'
h=History();hold(h,row(acceleration_g=4),25);checks['jolt_permanent_veto']=hold(h,row())['diagnostic_score']==0 and h.failure=='instrument_jolt'
h=History();hold(h,row(wrist=[81,1]),49);checks['wrist_98ms_not_abort']=h.failure is None;h.update(.002,row(wrist=[81,1]));checks['wrist100ms_aborts']=h.failure=='wrist_abort'
h=History();hold(h,row(wrist=[80,1]),100);checks['wrist_exact80_not_over']=h.failure is None
h=History();h.update(.002,row(pin=[float('nan'),0.]));checks['nonfinite_latched']=h.failure=='nonfinite'
m=Mechanism();a=m.limits(0,[.025,.025],[0.,0.],[1.,0.]);checks['twist_without_pull_cannot_retract_pin']=a['pinL_x'][1]==0. and a['knobL_rot'][1]==.15
m=Mechanism();m.limits(0,[.025,.025],[-.011,0.],[0.,0.]);a=m.limits(.002,[.025,.025],[-.011,0.],[1.,0.]);checks['physical_pull_then_turn_retains_no_extra_pull']=a['pinL_x'][1]==-.0106
b=m.limits(.004,[.025,.025],[-.0106,0.],[0.,0.]);checks['turnback_releases_passive_spring']=b['pinL_x'][1]==0.
m=Mechanism();a=m.limits(0,[.025,.025],[0,0],[0,0]);checks['pawl_lower_stop_source']=a['legL_z'][0]==-.0006
m.limits(.02,[.025,.025],[0,0],[0,0]);a=m.limits(.04,[.031,.025],[0,0],[0,0]);checks['5mm_lift_clears_pawl']=a['legL_z'][0]==-.05
f=json.loads((HERE/'fixture.json').read_text());checks['source_63_shapes']=sum(len(b['geoms']) for b in f['bodies'])==63;checks['source10_joints']=sum(len(b['joints']) for b in f['bodies'])==10
checks['source_ranges_not_relaxed']=all(j['range']==[-.05,.13] for b in f['bodies'] for j in b['joints'] if j['name'].startswith('leg'))
# Teacher's sole simulator mutation is emb.step(Action). Guard against accidental
# evaluator writes being imported or added to a purported robot demonstration.
tree=ast.parse((HERE/'reference.py').read_text());attrs=[n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)]
checks['teacher_no_object_writes']=not any(x.startswith(('write_','set_joint','set_external')) for x in attrs)
for p in HERE.glob('*.py'):ast.parse(p.read_text())
checks['all_python_parses']=True
out=dict(kind=__doc__,checks=checks,all_passed=all(checks.values()),physics_steps=0,model_calls=0,credit_channels_activated=[])
print(json.dumps(out,indent=2))
if '--out' in sys.argv:Path(sys.argv[sys.argv.index('--out')+1]).write_text(json.dumps(out,indent=2))
if not all(checks.values()):raise SystemExit(1)
