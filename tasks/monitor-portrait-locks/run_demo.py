"""Disclosed state-aware genuine-YAM full portrait_side attempt.
Only public14D robot actions change the episode after the ordinary scene reset.
Evaluator loads, fixture/object/joint/score/timer writes are absent from this runner.
No model calls. A gated raw-rubric success is not a verified submission demo.
"""
import argparse,functools,hashlib,json,math,os,shutil,subprocess,sys,time
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent
ROOT=HERE;sys.path.insert(0,str(HERE))
ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);args=ap.parse_args()
if args.out.resolve().is_relative_to(HERE):raise ValueError('Output must be outside the task source directory')
args.out.mkdir(parents=True,exist_ok=False)
shutil.copytree(HERE,args.out/'source',ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
(args.out/'source-sha256.json').write_text(json.dumps({str(p.relative_to(args.out/'source')):hashlib.sha256(p.read_bytes()).hexdigest() for p in (args.out/'source').rglob('*') if p.is_file()},indent=2)+'\n')
shutil.copy2(__file__,args.out/'executed-runner.py');shutil.copy2(ROOT/'grasp_mesh_preflight.py',args.out/'executed-grasp-mesh-preflight.py')
if os.environ.get('B5_DRIVER_CHECK_OVERRIDE')=='1':
 from isaaclab.app import AppLauncher
 old=AppLauncher.__init__
 @functools.wraps(old)
 def init(self,*a,**kw):kw['kit_args']=(kw.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip();return old(self,*a,**kw)
 AppLauncher.__init__=init
os.environ['B5_TRACE']=str(args.out/'scoring-measurements.jsonl')
from inspect_robots import Scene
from inspect_robots.types import Action
from yam_arena.inspect_embodiment import yam_arena_embodiment
from yam_arena import ik
from yam_arena.constants import TCP_OFFSET
from yam_arena.station import BASES
from yam_arena.demo import grip_rotation
from yam_arena.urdf import forward_kinematics
from yam_arena.analysis import read_stl
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation,Slerp
from PIL import Image
from physics_observer import arr,rot_xyzw
from numeric_score import score
from grasp_mesh_preflight import swept_translation_hits,verify as verify_SAT
import xml.etree.ElementTree as ET
import kit
INSTRUCTION=json.loads((HERE/'task.json').read_text())['instruction']
emb=yam_arena_embodiment(task=str(HERE/'task.py'),device='cpu',object_jitter=0,control_hz=20,cam_width=320,cam_height=176)
report={'kind':'state-aware genuineYAM fullgoal attempt; no evaluator actuation, no model evidence','model_calls':0,'full_goal_demonstrated':False,'physical_goal_measured':False,'stages':[]};started=time.monotonic();run=None
class Trial:
 def __init__(self):
  self.obs=emb.reset(Scene(id='B5-side-controls-full-portrait',instruction=INSTRUCTION),seed=42);self.base=emb._base;self.fixture=self.base.scene['monitor_stand'];self.ob=kit._OBSERVER;self.origin=arr(self.base.scene.env_origins)[0];self.robot=[self.base.scene['left_robot'],self.base.scene['right_robot']];self.basepos=[arr(r.data.body_pos_w)[0,0]-self.origin for r in self.robot];self.measured=np.asarray(self.obs.state['joint_pos'],float);self.desired=self.measured.copy();self.desired[6]=self.desired[13]=0.;self.integral=np.zeros((2,6));self.freeze=[None,None];self.last_command=self.desired.copy();self.actions=[];self.rows=[];self.journal=open(args.out/'robot-journal.jsonl','w',buffering=1);self.rng=np.random.default_rng(42);self.mesh_cache={};self.writers={};self.writer_logs={}
  self.preflight=[]
  for side,i in [('left',0),('right',1)]:
   p,R=self.tcp(i);sl=slice(i*7,i*7+6);pp,RR=ik.tcp_pose(self.measured[sl],self.basepos[i]);pe=float(np.linalg.norm(p-pp));re=float(np.linalg.norm(ik._rot_error(R,RR)));assert np.linalg.norm(self.basepos[i]-(np.array(BASES[side])+[0,0,.30]))<1e-5;assert pe<.002 and re<.01
   self.preflight.append({'arm':side,'actual_base':self.basepos[i].tolist(),'FK_position_error_m':pe,'FK_rotation_error_rad':re})
  report['robot_frame_preflight']=self.preflight
  report['contact_selections']=self.ob.selections
  from room_geometry import serializable
  geoms,poses=self.ob.world_geoms()
  def plain(value):
   if isinstance(value,np.ndarray):return value.tolist()
   if isinstance(value,dict):return {k:plain(v) for k,v in value.items()}
   if isinstance(value,(list,tuple)):return [plain(v) for v in value]
   return value
  inventory={'kind':'measured initial fixture geometry and live-validated station/pedestals','fixture_geometry':plain(geoms),'fixture_body_poses':plain(poses),'station_and_pedestals':serializable(self.ob.furniture_shapes),'live_external_geometry_validation':self.ob.furniture_validation,'fixture_mass_properties':[{k:b[k] for k in ('name','mass','com','diagonal_inertia','inertia_quat_wxyz')} for b in self.ob.fixture['bodies']],'robots':{side:{'base_origin_m':self.basepos[i].tolist(),'joint_names':r.joint_names,'drive_max_forces':arr(r.root_view.get_dof_max_forces())[0].tolist(),'drive_stiffnesses':arr(r.root_view.get_dof_stiffnesses())[0].tolist()} for i,(side,r) in enumerate(zip(('left','right'),self.robot))},'limits_caveat':'Drive caps alone do not calibrate total motor effort including separate gravity compensation.'}
  (args.out/'setup-inventory.json').write_text(json.dumps(inventory,indent=2)+'\n')
  for cam,frame in self.obs.images.items():Image.fromarray(frame).save(args.out/('setup-'+cam+'.png'))
  report['setup_inventory']='setup-inventory.json'
  self.start_videos()
 def tcp(self,i):
  r=self.robot[i];j=r.body_names.index('gripper');R=rot_xyzw(arr(r.data.body_quat_w)[0,j]);p=arr(r.data.body_pos_w)[0,j]-self.origin+R@np.array(TCP_OFFSET);return p,R
 def start_videos(self):
  ff=shutil.which('ffmpeg')
  if not ff:
   try:
    import imageio_ffmpeg
    ff=imageio_ffmpeg.get_ffmpeg_exe()
   except ImportError:return
  for name,frame in self.obs.images.items():
   h,w=frame.shape[:2];log=open(args.out/(name+'-encode.log'),'wb');cmd=[ff,'-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s',str(w)+'x'+str(h),'-r','20','-i','pipe:0','-an','-vcodec','libx264','-pix_fmt','yuv420p','-crf','24','-movflags','+faststart',str(args.out/(name+'.mp4'))]
   self.writers[name]=subprocess.Popen(cmd,stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=log);self.writer_logs[name]=log
 def close(self):
  (args.out/'actions.json').write_text(json.dumps(self.actions)+'\n');self.journal.close();media={}
  for name,p in self.writers.items():
   try:p.stdin.close();media[name]=p.wait(timeout=30)
   except Exception as e:media[name]=str(e);p.terminate()
   self.writer_logs[name].close()
  report['development_video_encoder_returncodes']=media
 def solve(self,i,p,R):
  sl=slice(i*7,i*7+6)
  def fun(q):
   pp,RR=ik.tcp_pose(q,self.basepos[i]);return np.r_[pp-p,.25*ik._rot_error(R,RR)]
  best=None
  for seed in [self.desired[sl].copy(),self.measured[sl].copy()]+[self.rng.uniform(ik.LOWER,ik.UPPER) for _ in range(4)]:
   fit=least_squares(fun,np.clip(seed,ik.LOWER+1e-8,ik.UPPER-1e-8),bounds=(ik.LOWER,ik.UPPER),max_nfev=150);pp,RR=ik.tcp_pose(fit.x,self.basepos[i]);pe=np.linalg.norm(pp-p);re=np.linalg.norm(ik._rot_error(R,RR));v=pe+.25*re
   if best is None or v<best[0]:best=(v,fit.x,float(pe),float(re))
   if pe<.0003 and re<.005:break
  if best[2]>.003 or best[3]>.02:raise RuntimeError('IK rejected '+str((i,p.tolist(),best[2],best[3])))
  return best[1]
 def target(self,i,p,R):self.desired[i*7:i*7+6]=self.solve(i,np.asarray(p,float),np.asarray(R,float))
 def step(self,phase):
  command=self.desired.copy()
  for i in range(2):
   sl=slice(i*7,i*7+6)
   if self.freeze[i] is None:
    e=self.desired[sl]-self.measured[sl];self.integral[i]=np.clip(self.integral[i]+1.5*.05*e,-.35,.35);v=self.desired[sl]+self.integral[i]+.15*e;v=np.clip(v,ik.LOWER,ik.UPPER);v=np.clip(v,self.last_command[sl]-.05,self.last_command[sl]+.05);command[sl]=v;self.integral[i]=np.clip(v-self.desired[sl]-.15*e,-.35,.35)
   else:command[sl]=self.freeze[i]
  self.last_command=command.copy();self.actions.append(command.tolist());result=emb.step(Action(data=command));self.obs=result.observation;self.measured=np.asarray(self.obs.state['joint_pos'],float);h=self.ob.history;row=self.ob.last
  item={'step':len(self.actions),'phase':phase,'command_joint14':command.tolist(),'nominal_joint14':self.desired.tolist(),'actual_joint14':self.measured.tolist(),'left_TCP':self.tcp(0)[0].tolist(),'right_TCP':self.tcp(1)[0].tolist(),'row':row,'measurement':h.measurement(),'foot_metrics':getattr(h,'base_last',None),'native_numeric_score':score(h.measurement()),'runtime_status':self.ob.result(),'pivot_force_control':getattr(self,'pivot_control',None)};self.rows.append(item);self.journal.write(json.dumps(item)+'\n')
  for name,p in list(self.writers.items()):
   try:p.stdin.write(np.asarray(self.obs.images[name],dtype=np.uint8).tobytes())
   except (BrokenPipeError,OSError) as e:report.setdefault('development_video_errors',{})[name]=str(e);self.writers.pop(name);self.writer_logs[name].close()
  if result.terminated:raise RuntimeError(str((result.termination_reason,result.info)))
  if h.base_bad or h.integrity or h.furniture_steps or h.robot_box_steps:raise RuntimeError('Task veto '+str(h.measurement()))
  return row
 def note(self,name):
  item={'label':name,'step':len(self.actions),'row':self.ob.last,'measurement':self.ob.history.measurement(),'native_score':score(self.ob.history.measurement()),'runtime_status':self.ob.result(),'pivot_force_control':getattr(self,'pivot_control',None)};report['stages'].append(item)
  for cam,frame in self.obs.images.items():Image.fromarray(frame).save(args.out/(name+'-'+cam+'.png'))
  (args.out/'partial.json').write_text(json.dumps(report,indent=2)+'\n');print(name,len(self.actions),'pose',self.ob.last['pose'],'wrist',self.ob.last['wrist_force_n'],flush=True)
 def move(self,name,targets,n,settle=0,grip=None):
  start={i:self.tcp(i) for i in targets};slerps={i:Slerp([0,1],Rotation.from_matrix(np.stack([start[i][1],np.asarray(targets[i][1])]))) for i in targets};g0=self.desired[[6,13]].copy()
  for j in range(n):
   t=(j+1)/n;u=t*t*(3-2*t)
   for i,(p,R) in targets.items():self.target(i,start[i][0]+u*(np.asarray(p)-start[i][0]),slerps[i](u).as_matrix())
   if grip:
    for i,v in grip.items():self.desired[i*7+6]=g0[i]+u*(v-g0[i])
   self.step(name)
  for _ in range(settle):self.step(name+'_settle')
  self.note(name)
 def unfreeze(self,i):self.freeze[i]=None;self.integral[i]=0.;self.desired[i*7:i*7+6]=self.measured[i*7:i*7+6]
 def button(self,index):
  b=self.fixture.body_names.index('button_'+str(index));R=rot_xyzw(arr(self.fixture.data.body_quat_w)[0,b]);return arr(self.fixture.data.body_pos_w)[0,b]-self.origin+R@np.array([0.,0.,.007]),R[:,2]
 def finger_local(self):
  robot=self.robot[0];j=robot.body_names.index('gripper');p=arr(robot.data.body_pos_w)[0,j]-self.origin;R=rot_xyzw(arr(robot.data.body_quat_w)[0,j]);return {name:R.T@(arr(robot.data.body_pos_w)[0,robot.body_names.index(name)]-self.origin-p) for name in ('tip_left','tip_right')}
 def tool_cloud(self):
  if hasattr(self,'closed_cloud'):return self.closed_cloud
  doc=ET.parse(ik.YAM_URDF).getroot();fk=forward_kinematics(ik.YAM_URDF,{f'joint{i}':0. for i in range(1,9)});G=np.linalg.inv(fk['gripper']);points=[]
  for name in ('gripper','tip_left','tip_right'):
   for c in doc.find("link[@name='"+name+"']").findall('collision'):
    v=self.collision_vertices(c);M=G@fk[name];points.append(v@M[:3,:3].T+M[:3,3])
  self.closed_cloud=np.concatenate(points);return self.closed_cloud
 def collision_vertices(self,c):
  m=c.find('geometry/mesh');path=Path(ik.YAM_URDF).parent/m.attrib['filename'];key=str(path)
  if key not in self.mesh_cache:self.mesh_cache[key]=read_stl(path).reshape(-1,3)*np.fromstring(m.attrib.get('scale','1 1 1'),sep=' ')
  o=c.find('origin');p=np.fromstring(o.attrib.get('xyz','0 0 0'),sep=' ');R=Rotation.from_euler('xyz',np.fromstring(o.attrib.get('rpy','0 0 0'),sep=' ')).as_matrix();return self.mesh_cache[key]@R.T+p
 def jaw_preflight(self,closed,descent):
  assert verify_SAT()==4;opened=self.finger_local();delta=(opened['tip_right']-opened['tip_left'])-(closed['tip_right']-closed['tip_left']);robot=self.robot[0];g=robot.body_names.index('gripper');Rg=rot_xyzw(arr(robot.data.body_quat_w)[0,g]);axis=Rg@delta;axis/=np.linalg.norm(axis);panel=next(g for g in self.ob.world_geoms()[0] if g['name']=='monitor_panel');alignment=abs(float(axis@panel['R'][:,0]));assert abs(np.linalg.norm(delta)-.0939)<.001 and alignment>.999
  doc=ET.parse(ik.YAM_URDF).getroot();parts={};poses={}
  for name in ('gripper','tip_left','tip_right'):
   j=robot.body_names.index(name);p=arr(robot.data.body_pos_w)[0,j]-self.origin;R=rot_xyzw(arr(robot.data.body_quat_w)[0,j]);tri=[];poses[name]={'pos':p.tolist(),'R':R.tolist()}
   for c in doc.find("link[@name='"+name+"']").findall('collision'):tri.append((self.collision_vertices(c)@R.T+p).reshape(-1,3,3))
   tri=np.concatenate(tri);parts[name]={'triangles':len(tri),'swept_hits_with_2mm_padding':int(swept_translation_hits(tri,descent,panel['centre'],panel['R'],panel['half'],padding=.002).sum())}
  proof={'opening_axis_world':axis.tolist(),'opening_change_gripper_m':delta.tolist(),'thickness_alignment':alignment,'parts':parts,'actual_poses':poses};(args.out/'jaw-preflight.json').write_text(json.dumps(proof,indent=2)+'\n');assert not any(v['swept_hits_with_2mm_padding'] for v in parts.values()),parts;report['jaw_preflight']=proof
 def press(self,axis,contact,normal,R,prepare_left=None,max_steps=110):
  depth=0.;stable=0
  for j in range(max_steps):
   if prepare_left is not None:prepare_left(j)
   pin=self.ob.last['joint_q']['pin_'+axis]
   if self.freeze[1] is None:
    if pin>=.0048:self.freeze[1]=self.last_command[7:13].copy();stable=0
    else:depth=min(.006,depth+.00015);self.target(1,contact-normal*depth,R)
   else:
    stable+=1
    if pin<.004:self.unfreeze(1);stable=0
   self.step('press_'+axis)
   if stable>=12 and j>=50:break
  if self.ob.last['joint_q']['pin_'+axis]<.0045:raise RuntimeError(axis+' pin failed to establish clearance')
  self.note('held_'+axis)
 def push_pose(self,Rtool,gy,lead=0.,gap=0.):
  panel=next(g for g in self.ob.world_geoms()[0] if g['name']=='monitor_panel');pi=self.fixture.body_names.index('stand_pivot');pivot=arr(self.fixture.data.body_pos_w)[0,pi]-self.origin;surface=panel['centre']+panel['R'][:,1]*gy+panel['R'][:,2]*panel['half'][2];normal=panel['R'][:,2];axis=-panel['R'][:,0]
  if lead:
   D=Rotation.from_rotvec(axis*lead).as_matrix();surface=pivot+D@(surface-pivot);normal=D@normal
  cloud=self.tool_cloud();cp=cloud[np.argmin((cloud-np.array(TCP_OFFSET))@(Rtool.T@normal))];return surface-Rtool@(cp-np.array(TCP_OFFSET))+normal*gap,normal
try:
 run=Trial()
 for _ in range(15):run.step('close_empty_hands')
 closed=run.finger_local();run.move('open_left_before_approach',{},20,grip={0:1.})
 plan=json.loads((ROOT/'controller-parameters.json').read_text());(args.out/'layout-plan.json').write_text(json.dumps(plan,indent=2)+'\n');H=next(v for v in plan['right_buttons'] if v['button']=='H' and v['alpha_deg']==30 and abs(v['tool_yaw_rad'])<.01);P=next(v for v in plan['right_buttons'] if v['button']=='P' and v['alpha_deg']==30 and abs(v['tool_yaw_rad'])<.01);Rbutton=np.array(H['R']);Rgrasp=grip_rotation();faceH,nH=run.button(2);contactH=faceH-Rbutton@(np.array(H['support_point_in_gripper'])-np.array(TCP_OFFSET));panel=next(g for g in run.ob.world_geoms()[0] if g['name']=='monitor_panel');grasp=panel['centre']+panel['R'][:,1]*.12+np.array([0,0,.115]);above=grasp+[0,0,.10]
 run.move('bimanual_hover',{0:(above,Rgrasp),1:(contactH+nH*.035,Rbutton)},60,15);run.jaw_preflight(closed,np.array([0,0,-.10]));run.move('grasp_and_H_contact',{0:(grasp,Rgrasp),1:(contactH+nH*.0005,Rbutton)},50,15)
 run.press('H',contactH,nH,Rbutton,prepare_left=lambda j:run.desired.__setitem__(6,max(.4,1.-.6*(j+1)/35)))
 # Preserve the measured grasp transform; nominal fingertip pose is not the
 # actual clamped TCP/panel relation. The outer index servo accumulates against
 # its previous target, rather than cancelling its correction against actualTCP.
 grasp_p,lift_R=run.tcp(0);initial_h=run.ob.last['joint_q']['height'];height_target=grasp_p+np.array([0,0,.13-initial_h-.0007]);report['measured_height_grasp_reference']={'actual_TCP':grasp_p.tolist(),'initial_height_m':initial_h,'goal_TCP':height_target.tolist(),'R':lift_R.tolist(),'controller_upstroke_bias_compensation_m':-.0007,'bias_evidence':'43368/43370 actualheight exceeds nominal by0.63-0.76mm; source goal/score unchanged'}
 run.move('raise_130mm',{0:(height_target,lift_R)},120,20)
 for _ in range(6):
  error=.13-run.ob.last['joint_q']['height'];velocity=run.ob.last['joint_vel'][0]
  if abs(error)<.00015 and abs(velocity)<.0003:break
  height_target[2]+=float(np.clip(1.5*error,-.0025,.0025));run.move('height_index_alignment',{0:(height_target,lift_R)},10,10)
 if abs(run.ob.last['joint_q']['height']-.13)>.0002:raise RuntimeError('Height not aligned with130mm lock notch')
 # Freeze the actually issued load-bearing command while the H pin seats.
 run.freeze[0]=run.last_command[:6].copy();run.note('height_aligned_frozen')
 run.unfreeze(1);run.move('release_H_and_relock_height',{1:(contactH+nH*.025,Rbutton)},25,15)
 if run.ob.last['joint_q']['pin_H']>.0015:raise RuntimeError('H lock did not return '+str(run.ob.last['joint_q']))
 faceP,nP=run.button(1);contactP=faceP-Rbutton@(np.array(P['support_point_in_gripper'])-np.array(TCP_OFFSET));run.move('open_left_and_hover_P',{1:(contactP+nP*.025,Rbutton)},20,grip={0:1.})
 run.unfreeze(0)
 Rpush=Rotation.from_euler('y',-30,degrees=True).as_matrix()@grip_rotation();pLeft,_=run.tcp(0);run.move('reorient_open_left_and_contact_P',{0:(pLeft,Rpush),1:(contactP+nP*.0005,Rbutton)},25,5)
 gy=.12;pHover,_=run.push_pose(Rpush,gy,gap=.010);leftStart,_=run.tcp(0)
 def prepare_push(j):
  if j<30:
   t=(j+1)/30;u=t*t*(3-2*t);run.target(0,leftStart+u*(pHover-leftStart),Rpush)
  if j>=30:run.desired[6]=max(0.,1.-(j-29)/20)
 run.press('P',contactP,nP,Rbutton,prepare_left=prepare_push)
 if len(run.actions)>690:raise RuntimeError('Insufficient action budget reserved for fullpivot/release/20s hold')
 run.note('ready_to_pivot')
 preload=.001;target_force=4.
 report['pivot_control_parameters']={'native_contact_y_m':gy,'target_normal_force_N':target_force,'maximum_normal_preload_m':.008,'angle_lead_deg':.8,'declared_joint_friction_Nm':.302494,'nominal_force_torque_Nm':gy*target_force}
 push_steps=min(450,1428-len(run.actions)-50-420)
 report['reserved_pivot_steps']=push_steps
 if push_steps<1:raise RuntimeError('No fullgoal action budget remains')
 for j in range(push_steps):
  q=run.ob.last['joint_q']['pivot']
  if q>=math.radians(89.95):break
  if run.ob.last['joint_q']['pin_P']<.004:raise RuntimeError('P clearance lost during pivot')
  lead=min(math.radians(.8),max(0.,math.pi/2-q));_,normal_now=run.push_pose(Rpush,gy);selection=run.ob.selections['robot_stand'];force_now=0.
  for contact in run.ob.last['contact_details']['robot_stand']:
   if selection['sensors'][contact['sensor_index']].endswith('/stand_pivot') and '/LeftArm/' in selection['filters'][contact['filter_index']]:force_now+=max(0.,contact['normal_force_n']*float(np.dot(contact['normal_w'],-normal_now)))
  preload=float(np.clip(preload+.00003*(target_force-force_now),.0005,.008));run.pivot_control={'target_normal_N':target_force,'measured_normal_N':force_now,'preload_m':preload,'actual_pivot_rad':q,'lead_rad':lead};pTarget,_=run.push_pose(Rpush,gy,lead=lead,gap=-preload);run.target(0,pTarget,Rpush);run.step('physical_panel_pivot')
 if run.ob.last['joint_q']['pivot']<math.radians(89.9):raise RuntimeError('Pivot did not reach portrait notch')
 run.note('portrait_reached');pOut,_=run.push_pose(Rpush,gy,gap=.030);run.unfreeze(1);run.move('release_P_and_withdraw_left',{0:(pOut,Rpush),1:(contactP+nP*.025,Rbutton)},30,20)
 if run.ob.last['joint_q']['pin_P']>.0015:raise RuntimeError('P lock did not return '+str(run.ob.last['joint_q']))
 for j in range(420):
  run.step('terminal_hands_off_hold')
  if run.ob.history.hold_s>=20.-1e-8:break
 run.note('terminal_hold');native=score(run.ob.history.measurement());report['physical_goal_measured']=native['score']==1. and native['full_credit'];report['native_numeric_result']=native;report['runtime_result']=kit.current_status();report['full_goal_demonstrated']=bool(report['physical_goal_measured'] and kit.current_score()==1.);report['control_steps']=len(run.actions);report['elapsed_s']=time.monotonic()-started;report['checks']={'actual_native_fullgoal_and20s_hands_off':report['physical_goal_measured'],'budget_respected':len(run.actions)<=1428,'no_model_calls':True,'genuine_joint_actions_only':True};(args.out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
except BaseException as exc:
 report['control_steps']=len(run.actions) if run else 0;(args.out/'error.json').write_text(json.dumps({'type':type(exc).__name__,'message':str(exc),'partial':report},indent=2,default=str)+'\n');raise
finally:
 if run is not None:
  run.close()
  (args.out/'finalization.json').write_text(json.dumps({'control_steps':len(run.actions),'development_video_encoder_returncodes':report.get('development_video_encoder_returncodes',{}),'development_video_errors':report.get('development_video_errors',{})},indent=2)+'\n')
 emb.close()
