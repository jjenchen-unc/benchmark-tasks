"""Task-local total-effort adapter derived from verified B5implementation; freshB4runtime check still required."""
import numpy as np
def patch_actions(module):
    """B4-only total motor bound: gravity joins PD inside the implicit drive.

    The default SDK clips only PD at27/7Nm then adds gravity as a separate
    effort. Actual contact-free43386 requires36.1Nm on joint2. Representing
    G as the internal target offset G/K preserves unsaturated PD+G exactly,
    while the existing PhysX drive cap now bounds the total. Public targets,
    joints, gains, damping, velocity limits and robot assets are unchanged.
    """
    cls=module.YamArmAction
    if getattr(cls,'_b4_total_effort_patched',False):return
    original=cls.apply_actions
    def apply_actions(self):
        if not getattr(self._asset,'_b4_total_effort_enabled',False):return original(self)
        import torch
        if not hasattr(self,'_b4_drive_stiffness'):
            from kit import arr as measured_array
            stiffness=torch.as_tensor(measured_array(self._asset.root_view.get_dof_stiffnesses()).copy(),device=self.device,dtype=self._processed.dtype)[:,self._joint_ids]
            limits=torch.as_tensor(measured_array(self._asset.root_view.get_dof_max_forces()).copy(),device=self.device,dtype=self._processed.dtype)[:,self._joint_ids]
            expected_K=torch.tensor([80.,80.,80.,10.,10.,10.],device=self.device)
            expected_L=torch.tensor([27.,27.,27.,7.,7.,7.],device=self.device)
            if not torch.isfinite(stiffness).all() or not torch.allclose(stiffness,expected_K.expand_as(stiffness)) or not torch.allclose(limits,expected_L.expand_as(limits)):raise RuntimeError('B4 total-effort contract requires verified SDK drive gains/caps')
            self._b4_drive_stiffness=stiffness
        gravity=torch.zeros_like(self._processed)
        if self.cfg.gravity_compensation:
            full=self._asset.data.gravity_compensation_forces.torch
            if full.shape[1]==self._asset.num_joints+6:full=full[:,6:]
            gravity=full[:,self._joint_ids]
        if not torch.isfinite(gravity).all():raise RuntimeError('B4 nonfinite gravity compensation')
        # Safety is applied at the next physics substep, not delayed to20Hz.
        # Only robot motor targets change; task objects are never set here.
        import kit
        engine=kit.ENGINE
        stopped=engine is not None and (engine.history.guard or engine.history.failure is not None)
        if stopped:
            if getattr(self,'_b4_safety_hold',None) is None:
                self._b4_safety_hold=self._asset.data.joint_pos.torch[:,self._joint_ids].clone()
            target=self._b4_safety_hold
        else:
            self._b4_safety_hold=None;target=self._processed
        command=self._asset.actuators.target_command
        command.set_position_index(value=(target+gravity/self._b4_drive_stiffness).contiguous(),joint_ids=self._joint_ids)
        command.set_effort_index(value=torch.zeros_like(self._processed),joint_ids=self._joint_ids)
        if self._camera is not None:self.update_wrist_camera()
    cls.apply_actions=apply_actions;cls._b4_total_effort_patched=True
