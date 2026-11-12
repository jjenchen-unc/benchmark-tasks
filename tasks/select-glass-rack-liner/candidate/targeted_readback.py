"""EVALUATOR ONLY. Read-only complete stage inventory and positive known loads.
No production/controller hooks; physical probes never establish task feasibility.
"""
import json,math
import numpy as np
from c1_kit import arr
from c1_contract import rotation_xyzw,supported

def numeric(v):
 if v is None or isinstance(v,(str,bool,int)):return v
 if isinstance(v,float):
  if not math.isfinite(v):raise ValueError('Nonfinite telemetry')
  return v
 if isinstance(v,dict):
  if not all(isinstance(k,str) for k in v):raise TypeError('Telemetry keys must be strings')
  return {k:numeric(x) for k,x in v.items()}
 if isinstance(v,(list,tuple)):return [numeric(x) for x in v]
 if hasattr(v,'tolist'):return numeric(v.tolist())
 if hasattr(v,'item'):return numeric(v.item())
 try:return [numeric(x) for x in v]
 except TypeError:raise TypeError('Unsupported numeric telemetry '+type(v).__name__)

def dump(path,value):path.write_text(json.dumps(numeric(value),indent=2,allow_nan=False)+'\n')

def usd_attribute_value(value):
 # USD schema metadata can explicitly use unbounded limits. Preserve these
 # values as tagged data; measured robot/force/control telemetry still goes
 # through numeric() and rejects every NaN/Inf.
 if isinstance(value,(float,np.floating)) and not math.isfinite(float(value)):
  return {'usd_nonfinite':('nan' if math.isnan(float(value)) else ('+infinity' if float(value)>0 else '-infinity'))}
 if value is None or isinstance(value,(str,bool,int,float)):return value
 if isinstance(value,dict):return {key:usd_attribute_value(item) for key,item in value.items()}
 if isinstance(value,(list,tuple)):return [usd_attribute_value(item) for item in value]
 if hasattr(value,'tolist'):return usd_attribute_value(value.tolist())
 if hasattr(value,'item'):return usd_attribute_value(value.item())
 try:return [usd_attribute_value(item) for item in value]
 except TypeError:raise TypeError('Unsupported USD metadata '+type(value).__name__)


def snapshot(e,path):
 from pxr import UsdPhysics
 prims=[];colliders=[];bodies=[];materials=[]
 for p in e.stage.Traverse():
  collision=p.HasAPI(UsdPhysics.CollisionAPI);body=p.HasAPI(UsdPhysics.RigidBodyAPI);material=p.HasAPI(UsdPhysics.MaterialAPI)
  # Every relationship across the whole actual stage, not only predicted pairs.
  rels={r.GetName():[str(t) for t in r.GetTargets()] for r in p.GetRelationships()}
  attrs={}
  for a in p.GetAttributes():
   name=a.GetName()
   if not (collision or body or material or any(k in name.lower() for k in ('physics','physx','drive','mass','centerofmass','inertia','xform'))):continue
   v=a.Get()
   try:attrs[name]={'type':str(a.GetTypeName()),'value':usd_attribute_value(v)}
   except TypeError:attrs[name]={'type':str(a.GetTypeName()),'usd_text':str(v)}
  item={'path':str(p.GetPath()),'type':str(p.GetTypeName()),'apis':list(p.GetAppliedSchemas()),'relationships':rels,'attributes':attrs}
  prims.append(item)
  if body:bodies.append(str(p.GetPath()))
  if material:materials.append(str(p.GetPath()))
  if collision:
   parent=p
   while parent and not parent.HasAPI(UsdPhysics.RigidBodyAPI):parent=parent.GetParent()
   colliders.append({'path':str(p.GetPath()),'rigid_body_owner':str(parent.GetPath()) if parent else None})
 # Resolve actual collider-level filters including all ancestor relationships.
 tasks=[x['path'] for x in colliders if any(x['path'].startswith(e.env+'/'+n+'/') for n in [*[f'liner_{i}' for i in range(1,7)],'cup_1','cup_2'])]
 def excludes(a,b):
  p=e.stage.GetPrimAtPath(a)
  while p and str(p.GetPath())!='/':
   api=UsdPhysics.FilteredPairsAPI(p)
   if api:
    for t in api.GetFilteredPairsRel().GetTargets():
     if b==str(t) or b.startswith(str(t)+'/'):return True
   p=p.GetParent()
  return False
 matrix=[]
 for a in tasks:
  if '/cup_' not in a:continue
  for b in tasks:
   if '/liner_' not in b:continue
   pair_a=a.endswith('/pair_contact');pair_b=b.endswith('/pair_contact')
   actual=not(excludes(a,b) or excludes(b,a));expected=pair_a and pair_b
   matrix.append({'cup':a,'liner':b,'collision_allowed':actual,'expected_allowed':expected,'matches':actual==expected})
 robots={}
 for side in ('left','right'):
  robot=e.base.scene[side+'_robot'];sensor=e.base.scene[side+'_c1_wrench'];rv=robot.root_view
  item={'body_names':list(robot.body_names),'joint_names':list(robot.joint_names),'link_paths':rv.link_paths,'joint_pos':arr(robot.data.joint_pos),'joint_vel':arr(robot.data.joint_vel),'body_pos_w':arr(robot.data.body_pos_w),'body_quat_w':arr(robot.data.body_quat_w),'wrench_body_names':list(sensor.body_names),'incoming_sensor_force':arr(sensor.data.force),'sensor_joint_pos_b':arr(sensor._joint_pos_b),'sensor_joint_quat_b':arr(sensor._joint_quat_b)}
  # These exact installed PhysX APIs are required; absence is an explicit error,
  # never a manufactured empty vector or a passed calibration.
  for key,method in [('coms','get_coms'),('masses','get_masses'),('inertias','get_inertias'),('incoming_joint_wrenches','get_link_incoming_joint_force'),('motor_caps','get_dof_max_forces'),('stiffness','get_dof_stiffnesses'),('damping','get_dof_dampings')]:
   try:item[key]=arr(getattr(rv,method)())
   except Exception as ex:item[key]={'unavailable':type(ex).__name__+': '+str(ex)}
  for field in ('applied_torque','computed_torque','gravity_compensation_forces'):
   try:item[field]=arr(getattr(robot.data,field))
   except Exception as ex:item[field]={'unavailable':type(ex).__name__+': '+str(ex)}
  robots[side]=item
 T=e.history.last['contacts'] if e.history.last else []
 dump(path,{'kind':'actual read-only USD/PhysX snapshot; not a calibration pass','controls':e.controls,'physics_samples':e.samples,'environment':e.env,'filter_audit':e.filter_audit,'actual_cup_liner_filter_matrix':matrix,'pair_filter_matrix_matches':bool(matrix) and all(x['matches'] for x in matrix),'prims':prims,'colliders':colliders,'rigid_bodies':bodies,'materials':materials,'robots':robots,'contacts':e.contacts.diagnostic_contacts if hasattr(e.contacts,'diagnostic_contacts') else [],'support_graph':{'edges':sorted(map(list,T)),'robot_supported':sorted(supported(T)),'indirect_supported':sorted(supported(T,False))},'params':e.params,'wet':e.wet,'status':e.status()})

def force_basis(emb,e,q,step,report,out):
 """Evaluator-only finite windows; preserves production math and tolerances."""
 import torch
 # Actual43824 fails because the initially adjacent fingertips brace each other.
 # Separate ONLY through public14D robot Actions; fixtures/sensor math unchanged.
 q=q.copy();start=q.copy();goal=q.copy();goal[0]=-.6;goal[7]=-.6
 n=max(1,int(np.ceil(np.max(np.abs(goal-start))/.02)))
 for k in range(1,n+1):step(start+(goal-start)*(k/n),1,'EVALUATOR_ONLY_robot_separate_wrists')
 q=goal;step(q,40,'EVALUATOR_ONLY_robot_separation_settle')
 report['interventions'].append({'kind':'EVALUATOR_ONLY_robot14D_setup','joint1_targets_rad':[-.6,-.6],'controls':n+40,'maximum_command_increment_rad':.02,'task_pose_state_score_writes':False,'cause':'actual43824 interarm fingertip bracing'})
 snapshot(e,out/'actual-separated-force-setup.json')
 rows=[]
 def outside_distal_contacts(side):
  robot=e.base.scene[side+'_robot'];root=robot.root_view.link_paths[0][robot.body_names.index('gripper')]
  return [v for v in e.contacts.diagnostic_contacts if (v['sensor']==root or v['sensor'].startswith(root+'/')) and not(v['other']==root or v['other'].startswith(root+'/')) and (v['separation_m']<=0 or abs(v['force_n'])>0)]
 def read(side):
  robot=e.base.scene[side+'_robot'];s=e.base.scene[side+'_c1_wrench'];i=s.body_names.index('gripper');j=robot.body_names.index('gripper')
  F=arr(s.data.force)[0,i];R=rotation_xyzw(arr(robot.data.body_quat_w)[0,j])@rotation_xyzw(arr(s._joint_quat_b)[i])
  return {'sensor':F.tolist(),'world':(R@F).tolist(),'R':R.tolist(),'joint_vel':arr(robot.data.joint_vel)[0].tolist(),'incoming_joint_wrenches':arr(robot.root_view.get_link_incoming_joint_force()).tolist(),'samples':e.samples,'outside_distal_contacts':outside_distal_contacts(side)}
 def window(side,label):
  samples=[]
  for _ in range(40):
   step(q,1,label);sample=read(side)
   if sample['outside_distal_contacts']:raise RuntimeError('Force diagnostic wrist is externally braced: '+side)
   samples.append(sample)
  return samples
 for side in ('left','right'):
  robot=e.base.scene[side+'_robot'];composer=robot.permanent_wrench_composer
  for axis in (0,):
   for sign in (-1,1):
    composer.reset();before=window(side,'EVALUATOR_ONLY_unloaded_window');f=np.zeros(3);f[axis]=5.*sign
    load=torch.tensor(f.reshape(1,1,3),dtype=torch.float32,device=emb._base.device)
    report['interventions'].append({'kind':'EVALUATOR_ONLY_known_load','arm':side,'force_world_n':f.tolist(),'moment_world_nm':[0.,0.,0.],'composer_global':True})
    try:
     composer.add_forces_and_torques(load,torch.zeros_like(load),body_ids=robot.find_bodies('gripper')[0],is_global=True)
     after=window(side,'EVALUATOR_ONLY_5N_window')
     snapshot(e,out/('force-'+side+'-'+str(axis)+'-'+str(sign)+'.json'))
    finally:composer.reset()
    b=np.array([v['world'] for v in before[-20:]]);a=np.array([v['world'] for v in after[-20:]]);delta=a.mean(axis=0)-b.mean(axis=0)
    rows.append({'arm':side,'axis':axis,'sign':sign,'applied_world_n':f.tolist(),'before':before,'after':after,'mean_world_delta_n':delta.tolist(),'norm_error_n':float(abs(np.linalg.norm(delta)-5.)),'opposing_load_vector_error_n':float(np.linalg.norm(delta+f)),'unloaded_world_std_n':b.std(axis=0).tolist(),'loaded_world_std_n':a.std(axis=0).tolist(),'last20_max_abs_joint_velocity_rad_s':float(max(np.max(np.abs(v['joint_vel'])) for v in before[-20:]+after[-20:]))})
 dump(out/'known-force-basis.json',{'kind':'EVALUATOR ONLY FOUR signed X loads; not XYZ12; finite windows do not presume equilibrium','scope':'FOUR_X_ONLY','rows':rows,'expected_delta_norm_n':5.,'tolerance_n':1.,'window_controls':40,'mean_tail_controls':20,'settled_claim':False,'robot_setup':'public14D joint1 -.6 both arms; no external distal contacts permitted in every sampled window'})
 report['checks']['known_5N_four_X_both_arms_magnitude']=all(x['norm_error_n']<=1. for x in rows)
 report['checks']['known_5N_four_X_both_arms_reaction_basis']=all(x['opposing_load_vector_error_n']<=1. for x in rows)
