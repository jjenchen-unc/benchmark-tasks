"""S5 task wrapper: channel credit only after source-bound fresh physical ROOT review."""
from pathlib import Path
import sys,os,runpy
O=Path(__file__).resolve().parent
sys.path.insert(0,str(O));sys.path.insert(0,str(O/'candidate'))
from protocol import receipt,CHANNELS
CREDIT=receipt(os.environ.get('S5_ROOT_REVIEW',''))
native=runpy.run_path(str(O/'candidate/task.py'))
__doc__=native['__doc__']
for name in ('SCENE','OBJECTS','SCORE_OVER','score'):globals()[name]=native[name]
import s5_rules as physics
if physics.REQUIRED!=CHANNELS or physics.VALIDATED_CHANNELS not in (frozenset(),frozenset(CHANNELS)):raise RuntimeError('unexpected native channel contract')
physics.VALIDATED_CHANNELS=frozenset(CHANNELS)
