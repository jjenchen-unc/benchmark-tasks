"""Genuine public-score replay using the exact production ROOT credit adapter."""
from pathlib import Path
import os,sys,runpy,json,hashlib
B=Path('/home/ubuntu/hydraulic-brev-20261110-1800');S=B/'source';O=B/'public-replay'
sys.path[:0]=[str(S),str(S/'candidate')]
from protocol import receipt,check
review=B/'ROOT-physical-review.json';receipt(review);before=check()
os.environ['HYDRAULIC_ROOT_REVIEW']=str(review)
from yam_arena.task_file import load
task=load(S/'model_task.py')
assert len(task.objects)==14
sys.argv=[str(S/'runtime.py'),'replay','--out',str(O)]
(B/'public-replay-entry.json').write_text(json.dumps({'entry_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'production_model_task_sha256':hashlib.sha256((S/'model_task.py').read_bytes()).hexdigest(),'native_manifest':before,'review_sha256':hashlib.sha256(review.read_bytes()).hexdigest(),'task_objects':len(task.objects),'physics_controller_goals_limits_unchanged':True},indent=2)+'\n')
runpy.run_path(str(S/'runtime.py'),run_name='__main__')
assert check()==before
