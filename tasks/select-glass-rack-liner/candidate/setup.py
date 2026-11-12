"""Reset-only setup capture; no evaluator placements, loads or robot actions."""
import sys
from pathlib import Path
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
from c1_launch import bootstrap
bootstrap(__file__)
import argparse,json,os,time
from c1_json import dumps
p=argparse.ArgumentParser();p.add_argument('--out',required=True,type=Path);p.add_argument('--public-case',action='store_true');p.add_argument('--target',default='C1-T1');p.add_argument('--seed',type=int,default=3867854786);a=p.parse_args()
from c1_launch import output_directory
output_directory(a.out)
if not a.public_case or a.target!='C1-T1':raise ValueError('Requires explicit fixed public task')
os.environ['C1_TARGET']='C1-T1';os.environ['C1_TRACE']=str(a.out/'measurements.jsonl')
from yam_arena.inspect_embodiment import yam_arena_embodiment
from inspect_robots import Scene
from c1_public import INSTRUCTION
from PIL import Image
start=time.monotonic();emb=yam_arena_embodiment(task=str(HERE/'public_entry.py'),device='cpu',object_jitter=0,control_hz=20,cam_width=320,cam_height=176)
try:
 obs=emb.reset(Scene(id='C1-public-setup',instruction=INSTRUCTION),seed=a.seed)
 import c1_kit
 status=c1_kit.ENGINE.status()
 if status['physics_samples']!=0 or c1_kit.ENGINE.controls!=0:raise RuntimeError('Setup advanced scored episode')
 for name,frame in obs.images.items():Image.fromarray(frame).save(a.out/(name+'.png'))
 (a.out/'result.json').write_text(dumps(dict(kind='actual reset observation; zero robot controls',trial_seed=a.seed,physical_seed=42,status=status,cameras=sorted(obs.images),wall_s=time.monotonic()-start,submission_ready=False),indent=2)+'\n')
finally:emb.close()
