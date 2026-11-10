"""Controller-only public joint-target correction; no task/SDK dependencies."""
import numpy as np
def update_bias(bias,nominal,measured,label):
 bias=np.asarray(bias,float);nominal=np.asarray(nominal,float);measured=np.asarray(measured,float)
 if any(x.shape!=(6,) for x in [bias,nominal,measured]) or not np.isfinite(np.r_[bias,nominal,measured]).all():raise ValueError('invalid measured feedback input')
 if label in ('pull_rear_paddle','paddle_release_hold'):
  return np.clip(bias+np.clip(.08*(nominal-measured),-.004,.004),-.12,.12)
 return bias+np.clip(-bias,-.004,.004)
