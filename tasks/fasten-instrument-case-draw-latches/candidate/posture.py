"""Robot-only reset posture, same upright station bases; radians and normalized jaw."""
import numpy as np
PARK={'left':np.array([1.2,1.,1.,-.6,0.,0.]),'right':np.array([-1.2,1.,1.,-.6,0.,0.])}
JAW=1.
def packed():return np.r_[PARK['left'],JAW,PARK['right'],JAW]
