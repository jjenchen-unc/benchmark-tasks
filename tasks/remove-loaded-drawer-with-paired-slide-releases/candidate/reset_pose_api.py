"""Task-local reset Pose serialization for PhysX transformf, never controller state."""
def float32_reset_pose(Pose, *, position_xyz, rotation_xyzw):
 class TransformfResetPose(Pose):
  def to_tensor(self, device):
   import torch
   value=torch.tensor((*self.position_xyz,*self.rotation_xyzw),dtype=torch.float32,device=device)
   if value.shape!=(7,) or not bool(torch.isfinite(value).all()):
    raise ValueError('Reset transform must be finite shape (7,)')
   return value
 return TransformfResetPose(position_xyz=position_xyz,rotation_xyzw=rotation_xyzw)
