"""Caps the total joint torque, gravity compensation included, at the SDK drive limits (27 / 7 N m)."""
def patch_actions(module):
    # The SDK clips only the PD torque and adds gravity compensation on top. Here gravity is folded into the
    # position target (G/K), so the drive's own 27 / 7 N m cap bounds the total. Targets, gains and assets unchanged.
    cls=module.YamArmAction
    if getattr(cls,'_total_effort_patched',False):return
    original=cls.apply_actions
    def apply_actions(self):
        if not getattr(self._asset,'_total_effort_enabled',False):return original(self)
        import torch
        if not hasattr(self,'_total_effort_drive_stiffness'):
            from contacts import arr as measured_array
            stiffness=torch.as_tensor(measured_array(self._asset.root_view.get_dof_stiffnesses()).copy(),device=self.device,dtype=self._processed.dtype)[:,self._joint_ids]
            limits=torch.as_tensor(measured_array(self._asset.root_view.get_dof_max_forces()).copy(),device=self.device,dtype=self._processed.dtype)[:,self._joint_ids]
            expected_K=torch.tensor([80.,80.,80.,10.,10.,10.],device=self.device)
            expected_L=torch.tensor([27.,27.,27.,7.,7.,7.],device=self.device)
            if not torch.isfinite(stiffness).all() or not torch.allclose(stiffness,expected_K.expand_as(stiffness)) or not torch.allclose(limits,expected_L.expand_as(limits)):raise RuntimeError('unexpected SDK drive gains or limits')
            self._total_effort_drive_stiffness=stiffness
        gravity=torch.zeros_like(self._processed)
        if self.cfg.gravity_compensation:
            full=self._asset.data.gravity_compensation_forces.torch
            if full.shape[1]==self._asset.num_joints+6:full=full[:,6:]
            gravity=full[:,self._joint_ids]
        if not torch.isfinite(gravity).all():raise RuntimeError('nonfinite gravity compensation')
        command=self._asset.actuators.target_command
        command.set_position_index(value=(self._processed+gravity/self._total_effort_drive_stiffness).contiguous(),joint_ids=self._joint_ids)
        command.set_effort_index(value=torch.zeros_like(self._processed),joint_ids=self._joint_ids)
        if self._camera is not None:self.update_wrist_camera()
    cls.apply_actions=apply_actions;cls._total_effort_patched=True
