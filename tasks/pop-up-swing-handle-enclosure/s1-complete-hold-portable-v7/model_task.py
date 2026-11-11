"""S1 native objects/scorer, credit only after fresh source-bound ROOT review."""
from pathlib import Path
import sys,os,runpy
O=Path(__file__).resolve().parent;sys.path.insert(0,str(O));sys.path.insert(0,str(O/'candidate'))
from protocol import receipt,CHANNELS
REVIEW=receipt(os.environ.get('S1_ROOT_REVIEW',''))
os.environ['S1_TARGET']='SH-T1'
native=runpy.run_path(str(O/'candidate/task.py'))
__doc__=__import__('protocol').load(O/'candidate/task.draft.json')['instruction']
for name in ['SCENE','OBJECTS','SCORE_OVER','score']:globals()[name]=native[name]
import s1_kit,rubric
if set(CHANNELS)!=rubric.REQUIRED:raise RuntimeError('native channel contract mismatch')
s1_kit.VERIFIED_CHANNELS=frozenset(CHANNELS)
