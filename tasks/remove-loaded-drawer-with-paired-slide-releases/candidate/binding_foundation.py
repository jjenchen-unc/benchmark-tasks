"""Minimal independent static-binding foundation. Genuine neutral robot actions only."""
from pathlib import Path
import sys,json,hashlib,shutil,argparse,time
HERE=Path(__file__).resolve().parent
# Bootstrap belongs to the actual -I launcher, never to its audit wrapper.
sys.path.insert(0,str(HERE))
ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--mode',choices=['api','reference'],default='api');ap.add_argument('--legacy-rtx-override',action='store_true');ap.add_argument('--credit-review',type=Path);a=ap.parse_args()
a.out=a.out.resolve();a.out.mkdir(parents=True,exist_ok=False)
manifest=json.loads((HERE/'manifest.json').read_text())
for name,digest in manifest['sha256'].items():
 if hashlib.sha256((HERE/name).read_bytes()).hexdigest()!=digest:raise RuntimeError('source hash mismatch: '+name)
frozen=a.out/'source';shutil.copytree(HERE,frozen,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
sys.path[0]=str(frozen)
from strict_json import primitive,dumps
# The same implementation is byte-identical in the frozen copy.
def save(name,row):(a.out/name).write_text(dumps(row,indent=2)+'\n')
save('sourcehashes.json',manifest)
from yam_arena.task_file import load
loaded=load(frozen/'task.py')
save('cold-task.json',{'constructed':True,'names':loaded.names,'scene':loaded.scene,'task_sha256':loaded.sha256,'isolated':sys.flags.isolated})
from runtime_credit import configure
if a.credit_review is not None and a.mode!='reference':raise ValueError('selected-path credit is reference-only')
save('credit-binding.json',configure(frozen,a.credit_review))
# First simulation-related import: a persistent CPU guard stops at this boundary.
from isaaclab.app import AppLauncher
if a.legacy_rtx_override:
 original=AppLauncher.__init__
 def init(self,*args,**kwargs):
  kwargs['kit_args']=(kwargs.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip()
  return original(self,*args,**kwargs)
 AppLauncher.__init__=init
emb=None;first_termination=None;trace=None
try:
 import numpy as np
 from inspect_robots import Scene
 from inspect_robots.types import Action
 from yam_arena.inspect_embodiment import yam_arena_embodiment
 from drawers_math import arr
 import drawers_kit as kit
 emb=yam_arena_embodiment(task=str(frozen/'task.py'),device='cpu',object_jitter=0.,control_hz=20,cam_width=320,cam_height=176)
 from reset_trace import install as install_reset_trace
 trace=install_reset_trace(a.out)
 obs=emb.reset(Scene(id='drawers-static-binding',instruction=loaded.docs),seed=42)
 last_command=None
 def telemetry(label,result=None):
  robots={}
  for side in ('left','right'):
   robot=emb._base.scene[side+'_robot']
   robots[side]={'joint_names':robot.joint_names,'raw8':arr(robot.data.joint_pos)[0],
    'base':arr(robot.data.root_link_pose_w)[0],'body_names':robot.body_names,
    'all_links':arr(robot.data.body_link_state_w)[0],
    'effort_caps':arr(robot.root_view.get_dof_max_forces()),'stiffness':arr(robot.root_view.get_dof_stiffnesses())}
  row=primitive({'label':label,'robots':robots,'public14':obs.state['joint_pos'],
    'roles':kit.ENGINE.static_roles,'selection':kit.ENGINE.contacts.selection,
    'task_poses':kit.ENGINE.poses(),'command14':last_command,
    'contacts':getattr(kit.ENGINE.contacts,'last_contacts',[]),'physics':kit.ENGINE.last,
    'paddle_joints':dict(zip(emb._base.scene['drawer_U'].joint_names,arr(emb._base.scene['drawer_U'].data.joint_pos)[0])),
    'paddle_armed':dict(kit.ENGINE.armed),'status':kit.ENGINE.status(),'first_termination':first_termination})
  if any(len(r['raw8'])!=8 for r in row['robots'].values()):raise RuntimeError('missing raw8')
  with (a.out/'journal.jsonl').open('a') as f:f.write(dumps(row)+'\n')
  return row
 save('pre-action.json',telemetry('pre_action'))
 from static_binding import stage_rows
 import omni.usd
 save('live-stage-ownership.json',stage_rows(omni.usd.get_context().get_stage()))
 # Observational scene queries only; no placements/loads. Query hit paths never
 # replace USD ownership or actual tensor-view cardinality gates.
 from omni.physx import get_physx_scene_query_interface
 query=get_physx_scene_query_interface();hits=[]
 def hit(row):
  def field(name):return row[name] if isinstance(row,dict) else getattr(row,name)
  hits.append({'collision':str(field('collision')),'rigid_body':str(field('rigid_body')),
    'position':[float(v) for v in field('position')],'normal':[float(v) for v in field('normal')],
    'distance':float(field('distance'))})
  return True
 query.raycast_all((.42,-.17,2.),(0.,0.,-1.),4.,hit)
 save('scene-query.json',{'scope':'observational ray only; not all-scene coverage','hits':hits})
 if a.mode=='reference' and kit.ENGINE.history.failure is None:
  from reference import run,FullGoalReached
  def record(label,observation,result,command=None):
   global obs,first_termination,last_command
   last_command=command;obs=observation
   if result.terminated and first_termination is None:first_termination={'reason':result.termination_reason,'stage':label}
   telemetry(label,result)
  try:run(emb,obs,record)
  except FullGoalReached:pass
 target=np.r_[kit.HOME,1.,kit.HOME,1.]
 for i in range(1 if a.mode=='api' and kit.ENGINE.history.failure is None else 0):
  result=emb.step(Action(data=target.copy()));obs=result.observation
  if result.terminated and first_termination is None:first_termination={'step':i+1,'reason':result.termination_reason}
  telemetry('neutral_'+str(i+1),result)
  if result.terminated:break
 save('result.json',{'status':'foundation_observed','first_termination':first_termination,'status_channels':kit.ENGINE.status(),'physics_verified':False,'fullgoal_verified':False,'submission_ready':False})
except Exception as error:
 save('result.json',{'status':'failed','error':repr(error),'first_termination':first_termination,'physics_verified':False,'fullgoal_verified':False,'submission_ready':False});raise
finally:
 if trace is not None:trace.close()
 if emb is not None:emb.close()
