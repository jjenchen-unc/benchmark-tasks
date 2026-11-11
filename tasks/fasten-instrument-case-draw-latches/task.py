"""Close the blue-grey instrument case and fasten both spring draw latches.
Seat each hook on a keeper bar, tension it with the lever flat, and return both
secondary catches home. Leave both hands off the case and instrument for20s.
Do not tip either instrument more than20deg or either case more than15deg.
The cases stand loose on a wooden plinth; the green case remains closed.
"""
from pathlib import Path
import importlib.util,os,runpy,json
_root=Path(__file__).resolve().parent
_spec=importlib.util.spec_from_file_location('_s2_public_verified_cache',_root/'run.py');_cache=importlib.util.module_from_spec(_spec);_spec.loader.exec_module(_cache)
_payload=_cache.prepare(_cache.default_cache(),Path(os.environ['S2_REVIEW_ARCHIVE']) if os.environ.get('S2_REVIEW_ARCHIVE') else None)
os.environ['S2_ROOT_REVIEW']=str(_payload/_cache.REVIEW)
_native=runpy.run_path(str(_payload/'s2-v30/model_task.py'))
if __doc__!=_native['__doc__']:raise ValueError('public instruction differs from reviewed task')
for _n in ('SCENE','OBJECTS','SCORE_OVER','score'):globals()[_n]=_native[_n]

# Apply the same disclosed all-row gripper conversion for direct Inspect task use.
import hashlib
import inspect_robots_dream._station as _station
_adapter_path=_root/'reproduction/gripper_saturation.py'
_expected=json.loads((_root/'model-envelope-sha256.json').read_text())['reproduction/gripper_saturation.py']
if hashlib.sha256(_adapter_path.read_bytes()).hexdigest()!=_expected:raise ValueError('public gripper adapter changed')
_existing=getattr(_station.rows,'_s2_public_gripper_adapter_sha256',None)
if _existing not in (None,_expected):raise ValueError('different model adapter already active')
if _existing is None:
 if hashlib.sha256(Path(_station.__file__).read_bytes()).hexdigest()!='0b24ccab52cabf8e14279b599c2173bed25f16c1a3d9656d8d739886b27fc0c7':raise ValueError('original station decoder changed')
 _projection_log=Path(os.environ.get('S2_GRIPPER_PROJECTION_LOG',str(Path.home()/'.cache'/('robocurve-s2-gripper-'+str(os.getpid())+'.jsonl')))).resolve()
 if _projection_log.is_relative_to(_root) or _projection_log.is_relative_to(_payload):raise ValueError('projection log must be outside source/evidence cache')
 _projection_log.parent.mkdir(parents=True,exist_ok=True)
 _a_spec=importlib.util.spec_from_file_location('_s2_public_gripper_rows',_adapter_path);_a=importlib.util.module_from_spec(_a_spec);_a_spec.loader.exec_module(_a)
 _station.rows=_a.PublicGripperRows(_station.rows,_projection_log);_station.rows._s2_public_gripper_adapter_sha256=_expected
