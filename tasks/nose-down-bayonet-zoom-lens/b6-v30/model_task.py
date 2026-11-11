"""Public B6 task; credit requires fresh source-bound whole-goal ROOT receipt."""
from pathlib import Path
import sys,os,runpy
O=Path(__file__).resolve().parent;sys.path.insert(0,str(O));sys.path.insert(0,str(O/'candidate'))
from protocol import receipt,load,CHANNELS
REVIEW=receipt(os.environ.get('B6_ROOT_REVIEW',''))
native=runpy.run_path(str(O/'candidate/task.py'))
__doc__=load(O/'candidate/task.draft.json')['instruction']
for name in ('SCENE','OBJECTS','SCORE_OVER'):globals()[name]=native[name]
def score(objects):
 import kit
 if kit.ENGINE is None or kit.ENGINE.error:return 0.
 # Review opens only public scoring; native mechanics/history/controller stay exact.
 return kit.ENGINE.history.result(CHANNELS)['score']
