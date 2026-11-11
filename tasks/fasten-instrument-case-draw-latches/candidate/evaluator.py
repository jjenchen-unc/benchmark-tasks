"""Evaluator-only negative/reset diagnostic. Direct joint writes are NOT robot evidence."""
import json,math
import numpy as np
from pathlib import Path
from inspect_robots import Scene
from inspect_robots.types import Action
import kit

def run(emb,out):
 import torch
 obs=emb.reset(Scene(id='s2-evaluator-negative',instruction='Evaluator diagnostic, not a robot trial'),seed=17);E=kit.ENGINE;a=E.parts['A'];q=a.data.joint_pos.torch.clone();v=torch.zeros_like(q)
 # Concrete historical counterexample: lid/lever/catch flat and slide preload,
 # but both hooks disconnected. Must never acquire fastening credit or hold.
 for name,val in [('A_lid',0.),('A0_lever',0.),('A1_lever',0.),('A0_catch',0.),('A1_catch',0.),('A0_slide',0.),('A1_slide',0.)]:q[:,a.joint_names.index(name)]=val
 a.write_joint_state_to_sim(q,v)
 for j in range(2):
  for bar in range(3):E.eq[f'A{j}_eq{bar}'].Set(False)
  E.engaged[f'A{j}']=-1
 held=np.asarray(obs.state['joint_pos'],float)
 rows=[]
 for _ in range(4):emb.step(Action(data=held.copy()));rows.append(E.status())
 obs=emb.reset(Scene(id='s2-evaluator-reset',instruction='Evaluator reset diagnostic'),seed=987654);reset=E.status()
 return {'kind':__doc__,'unattached_hook_rows':rows,'reset':reset,'checks':{'no_unattached_fullgoal':all(r['hold_s']==0 and r['measured_score']<=.25 for r in rows),'arbitrary_seed_reset_history':reset['samples']==reset['controls']==0,'unattached_initial_A':all(reset['engaged'][f'A{j}']==-1 for j in range(2))},'robot_feasibility':False,'calibration':False}
