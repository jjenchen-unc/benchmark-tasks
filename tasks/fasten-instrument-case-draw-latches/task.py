"""Close the blue-grey instrument case and fasten both spring draw latches.
Seat each hook on a keeper bar, tension it with the lever flat, and return both
secondary catches home. Leave both hands off the case and instrument for 20 s.
Do not tip either instrument more than 20 degrees or either case more than 15 degrees.
The cases stand loose on a wooden plinth; the green case remains closed.
"""
from pathlib import Path
import importlib.util,os,runpy
_root=Path(__file__).resolve().parent
_native=runpy.run_path(str(_root/'candidate/task.py'))
for _n in ('SCENE','OBJECTS','SCORE_OVER','score'):globals()[_n]=_native[_n]

# Model runs: clip both gripper columns of every Dream-1 reply to [0,1] before the standard decoder (arm values unchanged).
import inspect_robots_dream._station as _station
if not getattr(_station.rows,'_s2_gripper_clip',False):
 _log=Path(os.environ.get('S2_GRIPPER_LOG',str(Path.home()/'.cache'/'s2-gripper-clipping.jsonl')));_log.parent.mkdir(parents=True,exist_ok=True)
 _spec=importlib.util.spec_from_file_location('_s2_gripper_rows',_root/'reproduction/gripper_saturation.py');_a=importlib.util.module_from_spec(_spec);_spec.loader.exec_module(_a)
 _station.rows=_a.PublicGripperRows(_station.rows,_log);_station.rows._s2_gripper_clip=True
