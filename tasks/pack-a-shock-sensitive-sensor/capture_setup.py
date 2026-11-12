"""Actual initial D3 RGB setup photographs; rendering never advances physics."""
from pathlib import Path
import argparse,hashlib,json,sys,time
HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--driver-check-override',action='store_true');a=p.parse_args();a.out.mkdir(parents=True,exist_ok=False)
if a.driver_check_override:
 from isaaclab.app import AppLauncher
 old=AppLauncher.__init__
 def init(self,*args,**kwargs):kwargs['kit_args']=(kwargs.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip();return old(self,*args,**kwargs)
 AppLauncher.__init__=init
sys.path.insert(0,str(HERE/'runtime'))
import numpy as np
from inspect_robots import Scene
from yam_arena.inspect_embodiment import yam_arena_embodiment
from PIL import Image
import d3_kit
emb=yam_arena_embodiment(task=str(HERE/'runtime/task.py'),device='cuda:0',object_jitter=0,control_hz=20,cam_width=640,cam_height=360)
try:
 instruction=json.loads((HERE/'task.draft.json').read_text())['instruction'];obs=emb.reset(Scene(id='D3-initial-setup-photographs',instruction=instruction),seed=42);base=emb._base;E=d3_kit.ENGINE
 def state():
  result={}
  for side in ['left','right']:result[side]=d3_kit.arr(base.scene[side+'_robot'].data.joint_pos)
  for n in range(1,5):result['carton'+str(n)]=d3_kit.arr(base.scene['carton_'+str(n)].data.body_link_state_w);result['pad'+str(n)]=d3_kit.arr(base.scene['pad_'+str(n)].data.root_com_state_w)
  for name in 'AB':result[name]=d3_kit.arr(base.scene['box_'+name].data.root_link_state_w)
  return result
 before=state()
 for _ in range(40):base.sim.render()
 captures=[]
 for name in ['top_cam','left_cam','right_cam']:
  cam=base.scene[name];cam.update(0.,force_recompute=True);rgb=d3_kit.arr(cam.data.output['rgb'])[0,...,:3].astype(np.uint8);Image.fromarray(rgb).save(a.out/(name+'.png'));captures.append({'camera':name,'shape':list(rgb.shape),'std':float(rgb.std())})
 after=state();unchanged=all(np.array_equal(before[k],after[k]) for k in before)
 report={'kind':__doc__,'captures':captures,'physics_controls':E.controls,'physics_s':E.physics_s,'all_robot_object_states_unchanged':unchanged,'model_calls':0}
 (a.out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
 assert unchanged and E.controls==0 and E.physics_s==0 and all(x['std']>5 for x in captures)
finally:emb.close()
