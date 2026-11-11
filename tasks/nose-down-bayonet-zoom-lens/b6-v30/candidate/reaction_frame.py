"""Incoming joint wrench adapter calibrated by actual43856 signed loads.
Authored revolute axis is joint X; localRot1 maps that axis to optical Z.
This is joint-frame torque, not the child-body wrench convention.
"""
import numpy as np

def optical_torsion(raw):
 a=np.asarray(raw,dtype=float)
 if a.shape!=(6,) or not np.isfinite(a).all():raise ValueError('invalid incoming joint wrench')
 return float(a[3])
