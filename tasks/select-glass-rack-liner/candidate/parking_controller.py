"""One source-bound right-arm parking insertion, using genuine14D Actions only.
The left arm holds its measured precontact joint pose and original jaw command.
No object, fixture, physics, history, scorer or credit writes.
"""
import json
from pathlib import Path
import numpy as np
from c1_json import dumps

def park_right(run):
 import c1_kit
 from route_telemetry import snapshot
 plan=json.loads((Path(__file__).resolve().parent/'parking-plan.json').read_text())
 if run.plan['liners']!=[2] or run.plan['slots']!=[1] or run.plan['wet']:raise RuntimeError('Private parking DATA requires physical42 dry liner2 slot1 plan')
 if plan.get('remaining_approach_requires_subdivision') is not True:raise RuntimeError('Parking DATA missing reviewed continuation subdivision requirement')
 if run.parking_done or len(run.actions)!=169:raise RuntimeError('Parking insertion requires original action169 exactly once')
 if c1_kit.ENGINE.samples!=8450:raise RuntimeError('Parking insertion requires sample8450')
 observed=snapshot(run)
 expected=np.asarray(plan['start_robots_raw8'],float)
 actual=np.asarray([r['raw8'] for r in observed['robots']],float)
 # Reject a different measured state; never insert from guessed reset joints.
 joint_error=float(np.max(abs(actual[:,:6]-expected[:,:6])))
 finger_error=float(np.max(abs(actual[:,6:]-expected[:,6:])))
 if joint_error>.003 or finger_error>.001:raise RuntimeError('Parking measured start differs from CPU checked state '+str((joint_error,finger_error)))
 bases=np.asarray(plan['bases_world4x4'],float)
 for a in range(2):
  if np.linalg.norm(run.basepos[a]-bases[a,:3,3])>1e-5 or np.max(abs(run.baseR[a]-bases[a,:3,:3]))>1e-5:raise RuntimeError('Parking base transform mismatch')
 grasp=plan['held_liner_transform'];p,R=run.pose('liner_2')
 from c1_contract import rotation_xyzw
 robot=run.robot[0];j=robot.body_names.index('gripper')
 gp=np.asarray(observed['robots'][0]['body_pos_w'][j]);gR=rotation_xyzw(np.asarray(observed['robots'][0]['body_quat_w'][j]))
 relp=gR.T@(p-gp);relR=gR.T@R
 if np.linalg.norm(relp-np.asarray(grasp['translation']))>.001 or np.max(abs(relR-np.asarray(grasp['rotation'])))>.01:raise RuntimeError('Measured held-liner transform differs from CPU geometry')
 waypoints=np.asarray(plan['right_waypoints_q8'],float)
 if waypoints.ndim!=2 or waypoints.shape[1]!=8 or not np.isfinite(waypoints).all():raise RuntimeError('Invalid parking waypoint DATA')
 if np.max(abs(np.diff(waypoints[:,:6],axis=0)))>.0450000001:raise RuntimeError('Parking DATA exceeds original increment limit')
 from c1_contract import native,WORKSPACE
 from yam_arena import ik
 # Match Trial.move's destination predicate. Native HOME is explicitly above
 # the declared tool box; joint interpolation from that retained pose is not a
 # sequence of new Cartesian goals. Never expand the box to admit HOME.
 target=np.asarray(plan['target_world_m'],float)
 if target.shape!=(3,) or not np.isfinite(target).all():raise RuntimeError('Invalid parking destination DATA')
 virtual=native(target+np.array([0.,0.,.012]))
 if np.any(virtual<WORKSPACE[:,0]-1e-6) or np.any(virtual>WORKSPACE[:,1]+1e-6):raise RuntimeError('Source-equivalent tool goal outside workspace '+str(virtual))
 # Additional path safety: only the inherited excursion may persist while
 # returning to the box; no component may worsen, and a re-exit is rejected.
 # Use the actual measured right pose, not a guessed HOME or enlarged bounds.
 start_virtual=native(run.tcp(1)[0]+np.array([0.,0.,.012]))
 previous=np.maximum(WORKSPACE[:,0]-start_virtual,0.)+np.maximum(start_virtual-WORKSPACE[:,1],0.)
 for q8 in waypoints:
  pp,_=ik.tcp_pose(q8[:6]);pp=run.basepos[1]+run.baseR[1]@pp
  virtual=native(pp+np.array([0.,0.,.012]))
  excursion=np.maximum(WORKSPACE[:,0]-virtual,0.)+np.maximum(virtual-WORKSPACE[:,1],0.)
  if np.any(excursion>previous+1e-6):raise RuntimeError('Parking path worsens inherited tool workspace excursion')
  previous=excursion
 if np.any(previous>1e-6):raise RuntimeError('Parking path does not return to tool workspace')
 if len(run.actions)+len(waypoints)>=2268:raise RuntimeError('Parking insertion exceeds original total budget')
 run.parking_done=True
 (run.out/'parking-preflight.json').write_text(dumps(dict(observed=observed,joint_error_rad=joint_error,finger_error_m=finger_error,measured_liner_translation=relp,measured_liner_rotation=relR,inserted_actions=len(waypoints),physical_proof=False),indent=2)+'\n')
 # First DATA point is actual8450; clip any command-to-measured delta under
 # the original per-action cap. Left fingers remain the original .669 command.
 hold=actual[0,:6].copy()
 for q8 in waypoints:
  run.command[:6]=np.clip(hold,run.command[:6]-.045,run.command[:6]+.045)
  run.command[7:13]=np.clip(q8[:6],run.command[7:13]-.045,run.command[7:13]+.045)
  run.tick('right_park_before_interarm_contact')
  if c1_kit.ENGINE.freeze_arms:raise RuntimeError('Original30N/20ms or50N/20ms motion guard during parking')
  if c1_kit.ENGINE.error or c1_kit.ENGINE.history.failure:raise RuntimeError('Original physics observer failure during parking')
  if np.linalg.norm(run.tcp(0)[0]-np.asarray(observed['tcp'][0]['position']))>.012:raise RuntimeError('Left hold TCP drift exceeds original12mm tracking threshold')
 if np.linalg.norm(run.tcp(1)[0]-np.asarray(plan['target_world_m']))>.012:raise RuntimeError('Parked right TCP exceeds original12mm tracking threshold')
 run.rows.append(dict(phase='right_park_before_interarm_contact',controls=len(run.actions),inserted_actions=len(waypoints),target_m=plan['target_world_m'],actual_error_m=float(np.linalg.norm(run.tcp(1)[0]-np.asarray(plan['target_world_m'])))))
