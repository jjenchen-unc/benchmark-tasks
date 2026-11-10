"""EVALUATOR ONLY: direct valve-state negative, never feasibility or model evidence.
This distinct driver deliberately opens the real valve fully to test overspeed
landing and permanent damage invalidation. It writes no scores or history.
"""
import argparse,functools,json,sys
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--legacy-rtx-override',action='store_true');a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=False)
 from yam_arena.task_file import load
 load(HERE/'task.py') # source hook installation before SDK import
 if a.legacy_rtx_override:
  from isaaclab.app import AppLauncher
  original=AppLauncher.__init__
  @functools.wraps(original)
  def launch(self,*args,**kw):
   kw['kit_args']=(kw.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip();return original(self,*args,**kw)
  AppLauncher.__init__=launch
 from inspect_robots import Scene
 from inspect_robots.types import Action
 from yam_arena.inspect_embodiment import yam_arena_embodiment
 import kit,torch
 e=yam_arena_embodiment(task=str(HERE/'task.py'),device='cpu',object_jitter=0,control_hz=20);rows=[];report={'kind':'evaluator-only full-open-valve negative; not robot evidence','model_calls':0}
 try:
  obs=e.reset(Scene(id='eval-negative',instruction='Evaluator-only negative'),seed=42);jack=kit.ENGINE.jacks['S'];q=jack.data.joint_pos.torch.clone();v=torch.zeros_like(q);q[:,jack.joint_names.index('valve_S')]=np.radians(kit.F['parameters']['phi0_S']+90.);jack.write_joint_state_to_sim(q,v)
  action=np.asarray(obs.state['joint_pos'],float)
  for _ in range(400):
   r=e.step(Action(data=action));rows.append(kit.current_status())
   if r.terminated:break
  report['result']=kit.current_status();report['expected_hard_landing_seen']=report['result']['failure']=='hard_landing'
 except Exception as ex:report['error']=type(ex).__name__+': '+str(ex)
 finally:
  (a.out/'result.json').write_text(json.dumps(report,indent=2)+'\n');(a.out/'rows.json').write_text(json.dumps(rows,indent=2)+'\n');e.close()
if __name__=='__main__':main()
