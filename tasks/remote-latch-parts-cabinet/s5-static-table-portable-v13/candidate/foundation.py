"""Root-only bounded actual-runtime probe. Execute frozen source, not a live import.
No model calls. GPU allocation required for RTX even with CPU rigid dynamics.
"""
import argparse,hashlib,json,os,sys,time,traceback
from pathlib import Path
HERE=Path(__file__).resolve().parent
ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--mode',choices=['foundation','reference','negative','targeted'],default='foundation');ap.add_argument('--legacy-rtx-override',action='store_true');args=ap.parse_args()
args.out.mkdir(parents=True,exist_ok=False)
report=dict(mode=args.mode,source=str(HERE),executed_file=__file__,physics_verified=False,model_verified=False,media_verified=False,model_calls=0,checks={},started_at=time.time());emb=None
try:
 # All source hashes are checked BEFORE cold SDK/Isaac imports. This exact file
 # executes from the immutable copied source requested by gpu-request.json.
 manifest=json.loads((HERE/'sourcehashes.json').read_text())
 actual={n:hashlib.sha256((HERE/n).read_bytes()).hexdigest() for n in manifest}
 if actual!=manifest:raise RuntimeError('immutable source hash mismatch')
 report['sourcehashes']=actual;report['checks']['exact_copied_source_executed']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()==manifest['foundation.py']
 (args.out/'executed-sourcehashes.json').write_text(json.dumps(actual,indent=2)+'\n')
 sys.path.insert(0,str(HERE));os.environ['S5_TRACE']=str(args.out/'physics.jsonl')
 # Load exact frozen task before embodiment import so all hooks apply cold.
 from yam_arena.task_file import load
 task=load(HERE/'task.py');report['task']=task.name
 (args.out/'task-construction.json').write_text(json.dumps(dict(name=task.name,objects=len(task.objects),source=str(HERE/'task.py')),allow_nan=False))
 if args.legacy_rtx_override:
  import functools
  from isaaclab.app import AppLauncher
  previous=AppLauncher.__init__
  @functools.wraps(previous)
  def launch(self,*a,**kw):kw['kit_args']=(kw.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip();return previous(self,*a,**kw)
  AppLauncher.__init__=launch
 from inspect_robots import Scene
 from inspect_robots.types import Action
 from yam_arena.inspect_embodiment import yam_arena_embodiment
 import numpy as np
 from PIL import Image
 emb=yam_arena_embodiment(task=str(HERE/'task.py'),device='cpu',object_jitter=0,control_hz=20,cam_width=320,cam_height=176)
 obs=emb.reset(Scene(id='S5-fixedcase-foundation',instruction=task.docs),seed=20261109)
 import s5_kit
 from s5_kit import arr
 E=s5_kit.ENGINE
 report['checks']['actual_seed_routing']=True;report['physics_selection']=E.selection
 report['cameras']={k:dict(shape=list(v.shape),std=float(v.std())) for k,v in obs.images.items()}
 report['checks']['three_rgb']=set(obs.images)=={'top_cam','left_cam','right_cam'} and all(v.std()>3 for v in obs.images.values())
 for n,img in obs.images.items():Image.fromarray(img).save(args.out/(n+'-initial.png'))
 report['drive_limits']={side:dict(names=emb._base.scene[side+'_robot'].joint_names,forces=arr(emb._base.scene[side+'_robot'].root_view.get_dof_max_forces()).tolist(),stiffness=arr(emb._base.scene[side+'_robot'].root_view.get_dof_stiffnesses()).tolist()) for side in ['left','right']}
 for side,data in report['drive_limits'].items():
  caps=dict(zip(data['names'],data['forces'][0]));report['checks'][side+'_caps']=all(abs(caps['joint'+str(i)]-(27 if i<=3 else 7))<1e-5 for i in range(1,7)) and all(abs(caps['joint'+str(i)]-40)<1e-5 for i in [7,8])
 report['initial_body_states']={n:arr(emb._base.scene[n].data.body_link_state_w).tolist() for n in ['cab_A','cab_B','A_box_R']}
 from telemetry import preflight,dumps
 report['first_action_preflight']=preflight(emb,obs,args.out)
 q=np.asarray(obs.state['joint_pos'],float);actions=[]
 def step(target,n,label):
  result=None
  for _ in range(n):
   actions.append(dict(label=label,action=target.tolist()));result=emb.step(Action(data=target))
   if result.terminated:raise RuntimeError(f'{label} terminated: {result.termination_reason}')
  return result
 if args.mode=='foundation':
  first=step(q,5,'neutral');target=q.copy();target[0]+=.05;target[7]-=.05;changed=step(target,5,'genuine14Dmove');step(q,5,'return')
  report['checks']['genuine14D']=len(q)==14;report['checks']['robot_motion']=float(np.linalg.norm(np.asarray(changed.observation.state['joint_pos'])[[0,7]]-q[[0,7]]))>.005
  report['checks']['clock_20Hz_500Hz']=E.hist.samples==15*25 and abs(E.hist.time-.75)<1e-7
  report['checks']['score_gate']=E.status()['score']==0 and bool(E.status()['missing_runtime_validation'])
  before={n:arr(emb._base.scene[n].data.root_link_state_w).tolist() for n in ['A_box_L','A_box_R','B_box_L','B_box_R']}
  obs=emb.reset(Scene(id='S5-reset-other-episode',instruction=task.docs),seed=99117);step(np.asarray(obs.state['joint_pos'],float),1,'post_reset')
  after={n:arr(emb._base.scene[n].data.root_link_state_w).tolist() for n in before};report['reset_comparison']=dict(before=before,after=after)
  report['checks']['reset_clears_clock']=E.controls==1 and E.hist.samples==25 and abs(E.hist.time-.05)<1e-8
  report['checks']['fixed_seed_geometry']=all(np.linalg.norm(np.array(before[n])[0,:3]-np.array(after[n])[0,:3])<.002 for n in before)
 elif args.mode=='reference':
  from reference import run
  with (args.out/'robot-actions.jsonl').open('w') as f:report['reference_result']=run(emb,obs,f)
 elif args.mode=='targeted':
  from targeted import run
  report['targeted_result']=run(emb,obs,args.out)
 else:
  from negative import run
  report['negative_result']=run(emb,obs,args.out)
 report['checks']['no_native_runtime_import']=not any(n=='mujoco' or n.startswith(('dual_arm_demo','portable_demo')) for n in sys.modules)
 report['final_status']=E.status();report['last_measurement']=E.last;report['actions']=actions;report['completed']=True
except Exception as exc:
 report['error']=type(exc).__name__+': '+str(exc);report['traceback']=traceback.format_exc();report['completed']=False
finally:
 report['finished_at']=time.time();(args.out/'result.json').write_text(__import__('telemetry').dumps(report)+'\n');print(json.dumps({k:report.get(k) for k in ['mode','completed','checks','error']}),flush=True)
 if emb:emb.close()
