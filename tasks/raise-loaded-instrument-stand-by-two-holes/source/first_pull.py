"""EVALUATOR ONLY (DIAGNOSTIC PREFIX, never full goal): changed tracking first-withdrawal actions, bounded read-only capture.
No placements, loads, state/effort/score/clock writes. No full-goal claim.
"""
import os,json
from pathlib import Path
import numpy as np
from numeric_json import dumps,plain
from reference import Teacher

class FirstPullComplete(Exception):pass

class MeasuredPrefix(Teacher):
 def tick(self,phase):
  if len(self.record)>=450:raise RuntimeError('bounded first-withdrawal action cap')
  super().tick(phase)
  if phase in ('L_pull','L_withdraw_track','L_withdraw_overtravel') and self.q('pinL_x')<=-.0105:
   raise FirstPullComplete()

def evaluate(emb,obs,record):
 from kit import ENGINE,arr
 from approach_diagnostic import capture
 fixture=json.loads((Path(__file__).parent/'fixture.json').read_text())
 owners={g['name']:b['name'] for b in fixture['bodies'] for g in b['geoms']}
 if owners['knobL']!='knobbL' or owners['handle_L']!='plate':raise RuntimeError('intended contact ownership changed')
 teacher=MeasuredPrefix(emb,obs,record);diag=teacher.pair_diagnostic
 base=Path(os.environ['PINS_ACTION_TRACE']).parent
 trace=(base/'first-pull-raw.jsonl').open('w',buffering=1)
 resolved=[]
 for sensor,view in diag.views:
  resolved.append(dict(requested_sensor=sensor,sensor_paths=view.sensor_paths,filter_paths=view.filter_paths,sensor_count=view.sensor_count,filter_count=view.filter_count))
 (base/'first-pull-resolved-views.json').write_text(dumps(resolved,indent=2))
 original=ENGINE.post
 summary={'samples':0,'phase_samples':{},'net_positive_sensor_samples':0,'matrix_positive_sensor_samples':0,'target_positive':{},'raw8_alllinks_complete':True,'completed_prefix':False}
 def measured_post(dt):
  result=original(dt)
  if ENGINE.settling:return result
  snapshot=capture(emb);phase=diag.phase;forces=[]
  for sensor,view in diag.views:
   net=arr(view.get_net_contact_forces(dt)).copy();matrix=arr(view.get_contact_force_matrix(dt)).copy()
   if net.shape!=(1,3) or matrix.shape!=(1,len(diag.filters),3):raise RuntimeError('unexpected contact force tensor shape')
   if not np.isfinite(net).all() or not np.isfinite(matrix).all():raise RuntimeError('nonfinite contact force')
   ids=np.flatnonzero(np.linalg.norm(matrix[0],axis=1)>0.)
   pairs=[dict(filter_index=int(j),other=diag.filters[j],normal_force_w_n=matrix[0,j]) for j in ids]
   forces.append(dict(sensor=sensor,net_contact_force_w_n=net[0],nonzero_normal_pairs=pairs,zero_normal_pairs=len(diag.filters)-len(ids)))
   if np.linalg.norm(net)>0:summary['net_positive_sensor_samples']+=1
   if len(ids):summary['matrix_positive_sensor_samples']+=1
   for pair in pairs:
    target='knobbL' if '/LeftArm/' in sensor else 'plate'
    if sensor.rsplit('/',1)[-1] in ('tip_left','tip_right') and pair['other'].endswith('/'+target):
     key=sensor.rsplit('/',1)[-1]+':'+target
     summary['target_positive'][key]=summary['target_positive'].get(key,0)+1
  for side,r in snapshot['robots'].items():
   if len(r['raw8'])!=8 or len(r['body_names'])!=9 or len(r['body_positions'])!=9 or len(r['body_quaternions'])!=9:raise RuntimeError('incomplete independent finger/link telemetry')
  summary['samples']+=1;summary['phase_samples'][phase]=summary['phase_samples'].get(phase,0)+1
  trace.write(dumps(dict(substep=summary['samples'],control=ENGINE.history.controls+1,time_s=ENGINE.time,phase=phase,action14=getattr(teacher,'pending_command',teacher.target).copy(),snapshot=snapshot,force_views=forces))+'\n')
  return result
 ENGINE.post=measured_post
 error=None
 try:teacher.run()
 except FirstPullComplete:summary['completed_prefix']=True
 except BaseException as e:error=type(e).__name__+': '+str(e);raise
 finally:
  ENGINE.post=original;trace.close();diag.close()
  summary.update(error=error,controls=len(record),budget=450,physical_success=False,full_goal_verified=False,score_claim=None,observation_only=True,
   positive_intended_finger_coverage=all(summary['target_positive'].get(f+':'+b,0)>0 for f in ('tip_left','tip_right') for b in ('knobbL','plate')),
   interpretation='Net forces include all colliders; pair matrix is normal force only. Positive intended finger pairs establish reader coverage, not grasp/friction retention. Zero coverage leaves reader/grasp unresolved. Review raw FK geometry and target motion separately.')
  (base/'first-pull-evidence.json').write_text(dumps(summary,indent=2))
 return plain(summary)
