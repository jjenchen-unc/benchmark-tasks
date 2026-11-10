from pathlib import Path
import sys,types
import numpy as np
import torch
sys.path.insert(0,str(Path(__file__).resolve().parent))
import sys as _sys, types as _types
# The motor math needs only measured-array conversion; no scene/task imports.
_sys.modules["kit"]=_types.SimpleNamespace(arr=lambda x: np.asarray(x))
from motor_adapter import patch_actions
K=np.array([80.,80.,80.,10.,10.,10.]);L=np.array([27.,27.,27.,7.,7.,7.])
class Command:
 def set_position_index(self,value,joint_ids):self.position=value.clone();self.ids=joint_ids
 def set_effort_index(self,value,joint_ids):self.effort=value.clone()
def make(gravity,floating=False,enabled=True,badK=False,badL=False):
 class Original:
  def apply_actions(self):self.original_called=True;return "original"
  def update_wrist_camera(self):self.camera_updated=True
 module=types.SimpleNamespace(YamArmAction=Original);patch_actions(module)
 fn=module.YamArmAction.apply_actions;patch_actions(module);assert module.YamArmAction.apply_actions is fn
 term=module.YamArmAction();term._joint_ids=list(range(6));term.device="cpu"
 term._processed=torch.tensor([[.04,-.03,.02,.01,-.01,.03]],dtype=torch.float32)
 term.cfg=types.SimpleNamespace(gravity_compensation=True);term._camera=object()
 stiffness=np.array([[*(K*.5 if badK else K),0.,0.]])
 limits=np.array([[*(L+1 if badL else L),40.,40.]])
 values=np.array([[*gravity,0.,0.]])
 if floating:values=np.c_[np.full((1,6),999.),values]
 command=Command()
 asset=types.SimpleNamespace(num_joints=8,_hj_total_effort_enabled=enabled,
  root_view=types.SimpleNamespace(get_dof_stiffnesses=lambda:stiffness,get_dof_max_forces=lambda:limits),
  data=types.SimpleNamespace(gravity_compensation_forces=types.SimpleNamespace(torch=torch.tensor(values,dtype=torch.float32))),
  actuators=types.SimpleNamespace(target_command=command))
 term._asset=asset
 return term,command
# Independent PD+G identity below saturation, with zero separate effort.
g=np.array([1.,2.,3.,.2,.3,.4])
term,cmd=make(g);term.apply_actions()
np.testing.assert_allclose(cmd.position.numpy()[0]*K,K*term._processed.numpy()[0]+g,atol=3e-6)
assert not cmd.effort.any() and term._asset._hj_total_effort_checked and term.camera_updated
# PhysX drive clipping now bounds total, including gravity, rather than PD alone.
term,cmd=make(np.array([40.,-40.,50.,12.,-12.,9.]));term.apply_actions()
total=np.clip(cmd.position.numpy()[0]*K,-L,L)+cmd.effort.numpy()[0]
np.testing.assert_allclose(total,[27.,-27.,27.,7.,-7.,7.],atol=1e-6)
# Floating-base gravity data keeps only the robot joint DOFs.
term,cmd=make(g,floating=True);term.apply_actions()
np.testing.assert_allclose(cmd.position.numpy()[0]*K,K*term._processed.numpy()[0]+g,atol=3e-6)
# Non-task assets preserve the original SDK path.
term,cmd=make(g,enabled=False);assert term.apply_actions()=="original"
assert not hasattr(cmd,"position")
# Disabled gravity and invalid physical-contract inputs fail or remain explicit.
term,cmd=make(g);term.cfg.gravity_compensation=False;term.apply_actions()
np.testing.assert_array_equal(cmd.position.numpy(),term._processed.numpy())
for flags in [{"badK":True},{"badL":True}]:
 term,cmd=make(g,**flags)
 try:term.apply_actions();raise AssertionError("invalid drive contract accepted")
 except RuntimeError:pass
term,cmd=make(np.array([np.nan,0,0,0,0,0]))
try:term.apply_actions();raise AssertionError("nonfinite gravity accepted")
except RuntimeError:pass
print("total motor bounds: unsaturatedPD+G identity, saturation, floatingbase, inactive path, camera, nonfinite and contract failures passed")
