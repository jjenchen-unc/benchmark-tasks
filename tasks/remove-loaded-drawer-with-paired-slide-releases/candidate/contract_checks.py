"""Actual wrapper exercised against a non-physics embodiment for rejection behavior."""
from pathlib import Path
import types,json,numpy as np
import drawers_kit as kit
from yam_arena.constants import READY
from yam_arena.ik import LOWER,UPPER
checks={};calls=[]
class Dummy:
 def reset(self,*a,**kw):pass
 def step(self,action):calls.append(action);return 'advanced'
 def _task_step(self):pass
module=types.SimpleNamespace(YamArenaEmbodiment=Dummy);kit.patch_embodiment(module)
e=Dummy();e.task=types.SimpleNamespace(objects=[kit.DrawerAsset(name='drawer_U',pos=(0,0,0))]);kit.ENGINE=types.SimpleNamespace(done=True)
a=types.SimpleNamespace(data=np.r_[kit.HOME,1.,kit.HOME,1.])
try:e.step(a)
except RuntimeError:checks['terminated_step_rejected_before_old_physics']=not calls
kit.ENGINE.done=False
for key,value in [('nan',np.full(14,np.nan)),('wrong_shape',np.zeros(13))]:
 try:e.step(types.SimpleNamespace(data=value))
 except ValueError:checks[key+'_rejected_before_physics']=not calls
poseL=np.array([[0.,.3,.75,0.,0.,0.,1.]]);poseR=np.array([[0.,-.3,.75,0.,0.,0.,1.]])
e._base=types.SimpleNamespace(scene={'left_robot':types.SimpleNamespace(data=types.SimpleNamespace(root_link_pose_w=poseL)),'right_robot':types.SimpleNamespace(data=types.SimpleNamespace(root_link_pose_w=poseR))});e._low=np.r_[LOWER,0,LOWER,0];e._high=np.r_[UPPER,1,UPPER,1]
try:e.step(types.SimpleNamespace(data=np.r_[READY,1,READY,1]))
except ValueError as exc:checks['nonvertical_READY_rejected_before_physics']='vertical' in str(exc) and not calls
checks['vertical_home_action_accepted']=e.step(a)=='advanced' and len(calls)==1
print(json.dumps({'scope':'wrapper and actual SDK FK only, no simulation','checks':checks,'passed':all(checks.values())},indent=2));assert all(checks.values())
