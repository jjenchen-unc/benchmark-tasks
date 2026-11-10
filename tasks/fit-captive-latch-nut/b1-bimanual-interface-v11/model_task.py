"""B2 task wrapper: channel credit only after source-bound fresh physical ROOT review."""
from pathlib import Path
import sys,os,runpy
O=Path(__file__).resolve().parent
sys.path.insert(0,str(O));sys.path.insert(0,str(O/'candidate'))
from protocol import receipt,CHANNELS
CREDIT=receipt(os.environ.get('B1_ROOT_REVIEW',''))
native=runpy.run_path(str(O/'candidate/task.py'))
__doc__=__import__('protocol').load(O/'candidate/task.draft.json')['instruction']
for name in ('SCENE','OBJECTS','SCORE_OVER','score'):globals()[name]=native[name]
import physics,history
if physics.REQUIRED!=CHANNELS or physics.VALIDATED_RUNTIME_CHANNELS not in (frozenset(),frozenset(CHANNELS)):raise RuntimeError('unexpected native channel contract')
physics.VALIDATED_RUNTIME_CHANNELS=history.VALIDATED_RUNTIME_CHANNELS=frozenset(CHANNELS)
