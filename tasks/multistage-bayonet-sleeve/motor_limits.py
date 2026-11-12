"""Task-scoped total27/7Nm drive bounds, including gravity compensation.
No global SDK files are changed; non-B7 assets keep their original action path.
"""
import numpy as np
def _array(value):
    if hasattr(value,"torch"):value=value.torch
    if hasattr(value,"detach"):value=value.detach()
    if hasattr(value,"cpu"):value=value.cpu()
    if hasattr(value,"numpy"):value=value.numpy()
    return np.asarray(value)
def patch_actions(module):
    cls=module.YamArmAction
    if getattr(cls,"_b7_total_effort_patched",False):return
    original=cls.apply_actions
    def apply_actions(self):
        if not getattr(self._asset,"_b7_total_effort_enabled",False):return original(self)
        import torch
        if not hasattr(self,"_b7_drive_stiffness"):
            stiffness=torch.as_tensor(_array(self._asset.root_view.get_dof_stiffnesses()).copy(),
                device=self.device,dtype=self._processed.dtype)[:,self._joint_ids]
            limits=torch.as_tensor(_array(self._asset.root_view.get_dof_max_forces()).copy(),
                device=self.device,dtype=self._processed.dtype)[:,self._joint_ids]
            expected_K=torch.tensor([80.,80.,80.,10.,10.,10.],device=self.device,dtype=self._processed.dtype)
            expected_L=torch.tensor([27.,27.,27.,7.,7.,7.],device=self.device,dtype=self._processed.dtype)
            if (not torch.isfinite(stiffness).all() or not torch.isfinite(limits).all()
                or not torch.allclose(stiffness,expected_K.expand_as(stiffness))
                or not torch.allclose(limits,expected_L.expand_as(limits))):
                raise RuntimeError("B7 total-effort contract requires verified SDK drive gains/caps")
            self._b7_drive_stiffness=stiffness
            self._asset._b7_total_effort_contract=dict(
                stiffness=stiffness.detach().cpu().tolist()[0],
                limits_Nm=limits.detach().cpu().tolist()[0],gravity_inside_drive=True)
        gravity=torch.zeros_like(self._processed)
        if self.cfg.gravity_compensation:
            full=self._asset.data.gravity_compensation_forces.torch
            if full.shape[1]==self._asset.num_joints+6:full=full[:,6:]
            gravity=full[:,self._joint_ids]
        if not torch.isfinite(gravity).all():raise RuntimeError("B7 nonfinite gravity compensation")
        command=self._asset.actuators.target_command
        command.set_position_index(value=(self._processed+gravity/self._b7_drive_stiffness).contiguous(),joint_ids=self._joint_ids)
        command.set_effort_index(value=torch.zeros_like(self._processed),joint_ids=self._joint_ids)
        self._asset._b7_total_effort_checked=True
        if self._camera is not None:self.update_wrist_camera()
    cls.apply_actions=apply_actions
    cls._b7_total_effort_patched=True
