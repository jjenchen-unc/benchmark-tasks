"""Task-local total-effort adapter derived from verified B5implementation; freshC1runtime check still required."""
import numpy as np
def patch_actions(module):
    """C1-only total motor bound: gravity joins PD inside the implicit drive.

    The default SDK clips only PD at27/7Nm then adds gravity as a separate
    effort. Actual contact-free43386 requires36.1Nm on joint2. Representing
    G as the internal target offset G/K preserves unsaturated PD+G exactly,
    while the existing PhysX drive cap now bounds the total. Public targets,
    joints, gains, damping and robot assets are unchanged. Task-local source
    speed/jaw bounds are applied below and in scene configuration.
    """
    cls=module.YamArmAction
    if getattr(cls,'_c1_total_effort_patched',False):return
    original=cls.apply_actions
    def apply_actions(self):
        if not getattr(self._asset,'_c1_total_effort_enabled',False):return original(self)
        import torch
        if not hasattr(self,'_c1_drive_stiffness'):
            from c1_kit import arr as measured_array
            stiffness=torch.as_tensor(measured_array(self._asset.root_view.get_dof_stiffnesses()).copy(),device=self.device,dtype=self._processed.dtype)[:,self._joint_ids]
            limits=torch.as_tensor(measured_array(self._asset.root_view.get_dof_max_forces()).copy(),device=self.device,dtype=self._processed.dtype)[:,self._joint_ids]
            expected_K=torch.tensor([80.,80.,80.,10.,10.,10.],device=self.device)
            expected_L=torch.tensor([27.,27.,27.,7.,7.,7.],device=self.device)
            if not torch.isfinite(stiffness).all() or not torch.allclose(stiffness,expected_K.expand_as(stiffness)) or not torch.allclose(limits,expected_L.expand_as(limits)):raise RuntimeError('C1 total-effort contract requires verified SDK drive gains/caps')
            self._c1_drive_stiffness=stiffness
        gravity=torch.zeros_like(self._processed)
        if self.cfg.gravity_compensation:
            full=self._asset.data.gravity_compensation_forces.torch
            if full.shape[1]==self._asset.num_joints+6:full=full[:,6:]
            gravity=full[:,self._joint_ids]
        if not torch.isfinite(gravity).all():raise RuntimeError('C1 nonfinite gravity compensation')
        command=self._asset.actuators.target_command
        from c1_kit import ENGINE
        target=self._processed
        # Source target slew is85% of2.175rad/s, interpolated at1ms.
        if getattr(self,'_c1_previous_target',None) is None:self._c1_previous_target=self._asset.data.joint_pos.torch[:,self._joint_ids].clone()
        target=torch.maximum(torch.minimum(target,self._c1_previous_target+.85*2.175*.001),self._c1_previous_target-.85*2.175*.001)
        self._c1_previous_target=target.clone()
        if ENGINE is not None and ENGINE.active and ENGINE.freeze_arms:
            if getattr(self._asset,'_c1_guard_hold',None) is None:self._asset._c1_guard_hold=self._asset.data.joint_pos.torch[:,self._joint_ids].clone()
            target=self._asset._c1_guard_hold
            self._c1_previous_target=target.clone()
        command.set_position_index(value=(target+gravity/self._c1_drive_stiffness).contiguous(),joint_ids=self._joint_ids)
        command.set_effort_index(value=torch.zeros_like(self._processed),joint_ids=self._joint_ids)
        if self._camera is not None:self.update_wrist_camera()
    cls.apply_actions=apply_actions;cls._c1_total_effort_patched=True

    # Preserve native90mm maximum jaw opening within the installed93.9mm
    # YAM travel. This is task-local; observations retain actual SDK units.
    grip=module.YamGripperAction
    if not getattr(grip,'_c1_gap_patched',False):
        previous=grip.apply_actions
        def apply_gripper(self):
            if not getattr(self._asset,'_c1_total_effort_enabled',False):return previous(self)
            from yam_arena.constants import FINGER_OPEN
            import math
            desired=self._processed.clamp(0.,.09/(2*abs(FINGER_OPEN)))*FINGER_OPEN
            if getattr(self,'_c1_filtered_finger',None) is None:self._c1_filtered_finger=self._asset.data.joint_pos.torch[:,self._finger_ids].mean(dim=1,keepdim=True).clone()
            delta=((desired-self._c1_filtered_finger)*(-math.expm1(-.001/.3))).clamp(-.075*.001,.075*.001)
            self._c1_filtered_finger=self._c1_filtered_finger+delta
            target=self._c1_filtered_finger.expand(-1,len(self._finger_ids)).contiguous()
            self._asset.actuators.target_command.set_position_index(value=target,joint_ids=self._finger_ids)
        grip.apply_actions=apply_gripper;grip._c1_gap_patched=True

    for action_cls,attribute in [(cls,'_c1_previous_target'),(grip,'_c1_filtered_finger')]:
        if not getattr(action_cls,'_c1_reset_filter_patched',False):
            old_reset=action_cls.reset
            def reset_filter(self,env_ids=None,_previous=old_reset,_attribute=attribute):
                result=_previous(self,env_ids);setattr(self,_attribute,None);return result
            action_cls.reset=reset_filter;action_cls._c1_reset_filter_patched=True
