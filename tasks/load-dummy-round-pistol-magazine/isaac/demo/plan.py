"""arm1 planned path in the magazine frame (numpy only; shared by reach.py and run_arm.py).
N = nose arm (vertical gripper, pinches the bullet across y), P = press arm (closed fingers pitched PHI about mag y, body
up and rearward, the rear-bottom edge of the fingertip face presses the case top PRESS_DX ahead of the base).
"""
import math
import os

import numpy as np

E = os.environ.get
MM = 0.001
R_CASE, L_CASE, R_BUL, L_BUL = 0.00498, 0.01915, 0.00451, 0.0055
OAL = L_CASE + L_BUL + R_BUL
TIP_Z, TIP_HW, TIP_T = 0.01284, 0.0077, 0.0039  # fingertip: past the TCP, half width (TCP y), thickness per finger
TAPER = (0.0130 - 0.0077) / 0.01284  # half width gained per m above the tip
PHI = math.radians(float(E("ARM_PHI", "50")))
PRESS_DX = float(E("ARM_PRESS_DX", "0.003"))
PIN_D = float(E("ARM_PIN_D", "0.0015"))  # N fingertips this far below the bullet axis
PIN_GAP = float(E("ARM_PIN_GAP", "0.001"))  # N pad rear edge ahead of the case mouth
X_LAY = float(E("ARM_LAY_X", "0.005"))
X_PRESS = X_LAY  # N holds the base x during the press
X_LIP_FRONT = 0.002  # ahead of this the lip ramp no longer holds a round near the lip line
X_PINCH_END = float(E("ARM_PINCH_END", "-0.003"))
X_END = float(E("ARM_X_END", "-0.0133"))
Z_FRONT, X_FRONT = 0.10738, 0.0165
Z_RET0 = 0.11086
TILT_END = math.radians(float(E("ARM_TILT_END", "7")))
TILT_PRESS = math.radians(float(E("ARM_TILT_PRESS", "10")))
NOSE_BOTTOM = float(E("ARM_NOSE_BOTTOM", "0.1088"))  # closed N fingertips' bottom during the final nose push


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def tilt_at(xb):
    """planned nose-up tilt for base x: TILT_PRESS at the press, falling linearly to TILT_END at the end of the pinch"""
    f = np.clip((X_PRESS - xb) / (X_PRESS - X_PINCH_END), 0, 1)
    return TILT_PRESS + f * (TILT_END - TILT_PRESS)


def base_z_at(xb, tilt):
    """planned base-centre height: 0.3 mm under the lip line for a round touching the lip front end when tilted"""
    return Z_RET0 - 0.0003 - max(0.0, X_LIP_FRONT - xb) * math.tan(tilt)


def axis(tilt):
    return np.array([math.cos(tilt), 0.0, math.sin(tilt)])


BETA = math.radians(float(E("ARM_BETA", "0")))  # N gripper leaning forward (+x) about mag y


def n_rot(psi, beta=None):
    from yam_arena.demo import grip_rotation
    return rot_y(BETA if beta is None else beta) @ grip_rotation(psi)


def n_tcp(base, tilt, y=0.0, d=PIN_D, psi=0.0, beta=None):
    """N TCP position (mag frame) pinching a round with base centre ``base`` (x, z) and nose-up ``tilt``: the rear
    corner of the fingertip face sits d below the round axis, PIN_GAP (+ the finger taper over d) ahead of the case mouth"""
    r = n_rot(psi, beta)
    a = axis(tilt)
    s = L_CASE + PIN_GAP + TAPER * d
    corner = np.array([base[0] + a[0] * s, y, base[1] + a[2] * s - d])
    return corner - r @ np.array([0.0, -math.copysign(TIP_HW, r[0, 1]), TIP_Z])


def p_rot(sgn, phi=None):
    phi = PHI if phi is None else phi
    z = np.array([math.sin(phi), 0.0, -math.cos(phi)])
    y = sgn * np.array([math.cos(phi), 0.0, math.sin(phi)])
    return np.c_[np.cross(y, z), y, z]


def p_edge(sgn):
    return np.array([0.0, -sgn * TIP_HW, TIP_Z])


def p_contact(base, tilt, y=0.0):
    a = axis(tilt)
    n = np.array([-a[2], 0.0, a[0]])
    c = np.array([base[0], 0.0, base[1]]) + a * PRESS_DX + n * R_CASE
    c[1] = y
    return c


def p_tcp(c, sgn, phi=None):
    return np.asarray(c) - p_rot(sgn, phi) @ p_edge(sgn)


def nose_push_tcp(tip_x, zc, y=0.0, psi=0.0, beta=None):
    """closed N gripper: the slanted rear edge of the fingertips (rear tip corner at NOSE_BOTTOM) touching the nose
    sphere whose tip is at tip_x and centre height zc"""
    r = n_rot(psi, beta)
    b = BETA if beta is None else beta
    t = math.tan(math.atan(TAPER) - b)  # rearward lean of the rear edge per unit height
    corner = np.array([tip_x + (math.hypot(1, t) - 1) * R_BUL + t * (zc - NOSE_BOTTOM), y, NOSE_BOTTOM])
    return corner - r @ np.array([0.0, -math.copysign(TIP_HW, r[0, 1]), TIP_Z])


def slide_samples(n=8):
    return np.linspace(X_PRESS, X_PINCH_END, n)
