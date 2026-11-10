"""Pure counterexamples, never a physical or full-goal proof."""
import copy,json
from contract import History

def row(pairs):return {'contacts':pairs,'wrist_force_n':[0.,0.],'link_force_n':{},'bars':{c:{'pos':[0,0,0],'quat_xyzw':[0,0,0,1],'lin_vel':[0,0,0],'ang_vel':[0,0,0]} for c in 'ABC'}}
def trial(r,n=10000):
 h=History()
 for _ in range(n):h.update(r,.002)
 return h
checks={}
full=row([('A','support'),('A','B'),('B','C')]);h=trial(full)
checks['full_chain_not_early_break']=h.result()['diagnostic_ungated_score']==1
checks['credits_gated']=h.result()['score']==0
for n,pairs in [(1,[('A','support')]),(2,[('A','support'),('A','B')])]:checks['partial_'+str(n)]=trial(row(pairs)).result()['diagnostic_ungated_score']==n/3
for label,r in [('robot_on_any_bar',row(full['contacts']+[('C','robot')])),('wrong_order',row([('B','support'),('B','A'),('A','C')])),('table_brace',row(full['contacts']+[('A','other')]))]:checks[label]=trial(r).result()['diagnostic_ungated_score']==0
checks['truncated_hold']=trial(full,9500).result()['diagnostic_ungated_score']==0
r=copy.deepcopy(full);r['bars']['A']['ang_vel']=[0,0,.026];checks['angular_velocity_counts']=trial(r).result()['diagnostic_ungated_score']==0
r=copy.deepcopy(full);r['bars']['A']['quat_xyzw']=[0,0,.7071067811865476,.7071067811865476];checks['ninety_degree_heading_rejected']=trial(r).result()['diagnostic_ungated_score']==0
h=trial(full);h.update(row(full['contacts']+[('A','robot')]),.002);checks['terminal_not_best']=h.result()['diagnostic_ungated_score']==0
h=History();r=copy.deepcopy(full);r['wrist_force_n']=[81,0]
for _ in range(49):h.update(r,.002)
checks['abort_not_before_100ms']=h.failure is None
h.update(r,.002);checks['abort_at_100ms']=h.failure=='wrist_force_abort'
h=History();r['wrist_force_n']=[31,0]
for _ in range(9):h.update(r,.002)
checks['guard_not_before_20ms']=not h.guard
h.update(r,.002);checks['guard_at_20ms']=h.guard
h=History();r=copy.deepcopy(full);r['link_force_n']={'right_link2':51.}
for _ in range(10):h.update(r,.002)
checks['50N_link_window_guard']=h.guard
h=History();r=copy.deepcopy(full);r['outside_workspace_robot_bar_contact']=True;h.update(r,.002)
checks['outside_workspace_contact_aborts']=h.failure=='workspace_contact_violation'
checks['reset_history_empty']=History().samples==0 and History().hold.sum()==0
print(json.dumps({'kind':'pure math counterexamples only','checks':{k:bool(v) for k,v in checks.items()},'passed':all(checks.values())},indent=2))
assert all(checks.values())
