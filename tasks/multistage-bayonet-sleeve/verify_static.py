"""Lightweight source/geometry/scorer/guard checks. No physics, GPU or model."""
import ast,hashlib,json,math,sys,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from source_geometry import HERE,N,xml
from scoring import History,REQUIRED,DT
from step_guard import EpisodeGuard,EpisodeStopped

def run():
 checks={};w,p=xml();checks['exact_native_geometry_xml']=ET.tostring(w,encoding='unicode')+'\n'==(HERE/'fixture.xml').read_text();f=json.loads((HERE/'fixture.json').read_text());checks['native_sha_matches']=f['native_source_sha256']==hashlib.sha256((HERE/'native_source.py').read_bytes()).hexdigest();checks['source_path_all_five_stages']=p['path_C']==[[0.,0.],[-.012,0.],[-.012,30.],[.024,30.],[.024,-30.],[.1,-30.]];checks['dead_end_false_seat_kept']=N['dead_ends'](p['path_C'])==[((-.012,0.),(-.012,-90.)),((-.012,-90.),(.005999999999999998,-90.)),((.005999999999999998,-90.),(.005999999999999998,-70.))]
 counts={b['name']:len(b['geoms']) for b in f['bodies']};checks['all_geometries_exported']=sum(counts.values())==len(w.findall('.//geom'));checks['no_mesh_hull_substitution']=all(g['type'] in ('box','cylinder') for b in f['bodies'] for g in b['geoms'])
 # Near/far track channels remain materially different. Wrong angle has blocked axial exit.
 checks['wrong_angle_no_open_exit']=not any(lo<0 for lo,hi in N['free_intervals'](p['path_C'],90.));checks['correct_final_angle_has_exit']=any(lo<0 for lo,hi in N['free_intervals'](p['path_C'],-30.))
 shift=np.array(f['translation']);row=dict(pos=(np.array([.05,.26,.812])+shift).tolist(),R=np.eye(3).tolist(),velocity=[0.]*6,translation=shift.tolist(),floor_contact=True,post_contact=False,robot_contact=False,wrist_force=[0.,0.]);h=History();h.update(row,DT);checks['partial_rest_0p5']=h.result(REQUIRED)['score']==.5;checks['unverified_score_zero']=h.result()['score']==0
 for _ in range(9998):h.update(row,DT)
 checks['short_hold_not_full']=h.result(REQUIRED)['score']==.5
 h.update(row,DT);checks['20second_full']=h.result(REQUIRED)['score']==1.
 h.update(dict(row,robot_contact=True),DT);checks['recontact_revokes_full']=h.result(REQUIRED)['score']==0 and h.hold==0
 for flag in ('post_contact','robot_contact'):
  h=History();h.update(dict(row,**{flag:True}),DT);checks[flag+'_zero']=h.result(REQUIRED)['score']==0
 h=History();h.update(dict(row,floor_contact=False),DT);checks['unsupported_zero']=h.result(REQUIRED)['score']==0
 for extra,label in [([.08,0,0],'over_wall'),([0,0,0],'fast')]:
  rr=dict(row);rr['pos']=(np.array(row['pos'])+extra).tolist()
  if label=='fast':rr['velocity']=[.021,0,0,0,0,0]
  h=History();h.update(rr,DT);checks[label+'_zero']=h.result(REQUIRED)['score']==0
 h=History()
 for _ in range(49):h.update(dict(row,wrist_force=[80.1,0.]),DT)
 checks['98ms_not_abort']=h.failure is None;h.update(dict(row,wrist_force=[80.1,0.]),DT);checks['100ms_abort_vetoes_all']=h.failure=='wrist_force_abort' and h.result(REQUIRED)['score']==0
 h=History()
 for _ in range(100):h.update(dict(row,wrist_force=[80.,0.]),DT)
 checks['exact80N_not_above']=h.failure is None
 calls=[];g=EpisodeGuard(1)
 class Result:terminated=False
 g.invoke(lambda:(calls.append(1),Result())[1])
 try:g.invoke(lambda:calls.append(2))
 except EpisodeStopped:pass
 checks['budget_blocks_before_physics']=calls==[1]
 g=EpisodeGuard(6000);g.stop('wrist_force_abort')
 try:g.invoke(lambda:calls.append(3))
 except EpisodeStopped:pass
 checks['terminated_blocks_before_physics']=calls==[1]
 tree=ast.parse((HERE/'reference.py').read_text());forbidden=('write_root','write_joint','set_joint','wrench_composer','qpos','qvel','setattr','exec','eval')
 calls0=[ast.unparse(n.func) for n in ast.walk(tree) if isinstance(n,ast.Call)];checks['robot_reference_has_no_sim_writes']=not any(any(word in call for word in forbidden) for call in calls0)
 checks['reference_imports_no_evaluator']=not any(isinstance(n,(ast.Import,ast.ImportFrom)) and 'foundation' in ast.unparse(n) for n in ast.walk(tree))
 checks['physics_DT_2ms_and_6000controls']=DT==.002 and N['TICK_BUDGET']==6000*25
 return dict(kind='pure static source/geometry/scorer/guard checks; no runtime evidence',passed=all(checks.values()),checks=checks,geometry_counts=counts)
if __name__=='__main__':
 result=run();print(json.dumps(result,indent=2));sys.exit(0 if result['passed'] else 1)
