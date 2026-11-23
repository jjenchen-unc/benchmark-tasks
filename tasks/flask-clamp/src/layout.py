"""Scene layout and geometry of the flask-clamp task (station frame: x forward from the arm bases, y to the robot's
left, z up; metres). Engine-independent (numpy only).

A benchtop orbital shaker stands on the bench in front of the arms. Its 305 x 305 mm platform carries four stainless
spring-finger clamps for 250 mL Erlenmeyer flasks in a 2 x 2 pattern (125 mm pitch). Three clamps already hold
capped culture flasks; one of the two near-row clamps (seeded) is free. The empty 250 mL flask to load stands upright on the bench beside the
shaker, on the side of the free clamp.

Frames:
* clamp frame C: origin on the platform's top surface at the clamp's axis, z up; yaw = the clamp's finger phase.
  The clamp's base plate (2 mm) is the seat: a seated flask's bottom is at C z = SEAT_Z.
* flask frame F: origin at the centre of the flask's bottom, z along its axis.
* finger frame (one per finger): origin on the hinge line, x radially outward, y tangential, z up. Its rotation
  about +y by theta > 0 swings the finger outward.
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475  # station table top (yam_arena TABLE_TOP["station"])
OFFICIAL_SEED = 1415158438  # the trial seed inspect-robots passes by default (eval seed 0)


def rz(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def ry_rad(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def box_corners(x0, x1, y0, y1, z0, z1):
    return np.array([[x, y, z] for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)], float)


def ring(r, z, n=24, phase=0.0):
    a = phase + np.arange(n) * 2 * math.pi / n
    return np.column_stack([r * np.cos(a), r * np.sin(a), np.full(n, z)])


# ---------------------------------------------------------------- orbital shaker (benchtop, 12 x 12 in platform)
SHAKER_XY = (0.405, 0.0)  # nominal centre of the platform (per-trial jitter)
BODY_HALF = (0.170, 0.170)  # housing footprint 340 x 340 mm
BODY_TOP = TOP + 0.085  # housing top
PLATFORM_HALF = 0.1525  # 305 x 305 mm platform
PLATFORM_T = 0.010
PLATFORM_Z = TOP + 0.100  # platform top surface (5 mm above the housing; low-profile unit)
ORBIT_R = 0.0095  # 19 mm orbit diameter
SHAKE_RPM = 200.0
SHAKE_RAMP_S = 0.5
SHAKE_HOLD_S = 1.5
PITCH = 0.125
# socket offsets from the platform centre: 0 far-left, 1 far-right, 2 near-left, 3 near-right (left = +y)
SOCKETS = ((PITCH / 2, PITCH / 2), (PITCH / 2, -PITCH / 2), (-PITCH / 2, PITCH / 2), (-PITCH / 2, -PITCH / 2))


def socket_side(socket):
    """+1 if the socket is in the robot's left column (+y), -1 if in the right column."""
    return 1 if SOCKETS[socket][1] > 0 else -1


# ---------------------------------------------------------------- 250 mL Erlenmeyer flask (narrow mouth, borosilicate)
# Corning Pyrex 4980-250 class: base OD 85 mm, height 145 mm, neck OD 34 mm.
F_BASE_R = 0.0425
F_HEEL_R = 0.0370  # bottom edge radius (rounded heel, as a chamfer)
F_HEEL_Z = 0.005
F_SIDE_Z = 0.009  # top of the cylindrical heel band
F_SHOULDER_R = 0.0180
F_SHOULDER_Z = 0.095
F_NECK_R = 0.0170
F_NECK_Z = (0.090, 0.138)
F_LIP_R = 0.0185
F_LIP_Z = (0.138, 0.145)
F_HEIGHT = 0.145
F_MASS = 0.140
F_COM_Z = 0.048
F_FRICTION = 0.7  # glass on the gripper pads (averaged with the pad material)
NECK_PATCH_R = 0.009  # torsional patch radius of the neck contacts (pad contact area)
NECK_GRIP_Z = 0.125  # grip height on the neck (flask frame)
CONE_SLOPE = (F_BASE_R - F_SHOULDER_R) / (F_SHOULDER_Z - F_SIDE_Z)


def flask_radius(z):
    """Outer radius of the flask body at height z (flask frame)."""
    if z < 0:
        return 0.0
    if z < F_HEEL_Z:
        return F_HEEL_R + (F_BASE_R - F_HEEL_R) * z / F_HEEL_Z
    if z < F_SIDE_Z:
        return F_BASE_R
    if z < F_SHOULDER_Z:
        return F_BASE_R - CONE_SLOPE * (z - F_SIDE_Z)
    if z < F_LIP_Z[0]:
        return F_NECK_R
    if z <= F_HEIGHT:
        return F_LIP_R
    return 0.0


def flask_hulls():
    """Convex pieces of the flask (flask frame): body (heel + cone), neck, lip."""
    body = np.vstack([ring(F_HEEL_R, 0.0), ring(F_BASE_R, F_HEEL_Z), ring(F_BASE_R, F_SIDE_Z),
                      ring(F_SHOULDER_R, F_SHOULDER_Z)])
    neck = np.vstack([ring(F_NECK_R, F_NECK_Z[0]), ring(F_NECK_R, F_NECK_Z[1])])
    lip = np.vstack([ring(F_LIP_R, F_LIP_Z[0]), ring(F_LIP_R, F_LIP_Z[1])])
    return {"body": body, "neck": neck, "lip": lip}


# ---------------------------------------------------------------- 250 mL spring-finger platform clamp (stainless)
SEAT_Z = 0.002  # base plate thickness: seated flask bottom height in C
PLATE_R = 0.050
N_FINGERS = 5
FINGER_W = 0.012  # strip width (tangential)
FINGER_T = 0.002  # collision thickness of the strip
HINGE = (0.046, 0.004)  # (r, z) of the hinge line in C
PAD = (0.030, 0.040)  # (r, z) of the finger's inward kink (the pad) at rest
TIP = (0.047, 0.078)  # (r, z) of the flared tip at rest
FINGER_MASS = 0.020
FINGER_K = 0.55  # N m / rad about the hinge (estimate; see README)
FINGER_C = 0.004  # N m s / rad
FINGER_LIMITS_DEG = (-3.0, 40.0)
FINGER_FRICTION = 0.2  # polished stainless on glass (combined with the flask by "min")
PLATE_FRICTION = 0.4
LOADED_THETA = 0.103  # finger angle of a clamp holding a seated flask (pad on the cone)


def finger_points(theta=0.0):
    """Collision pieces of one finger in its finger frame (hinge at the origin), rotated by theta (outward > 0):
    {'lower': pts, 'upper': pts}. The inner faces run hinge -> pad -> tip; the strip is FINGER_T thick outward."""
    h = np.array([HINGE[0], HINGE[1]])
    pts2 = {"lower": (np.array(HINGE), np.array(PAD)), "upper": (np.array(PAD), np.array(TIP))}
    out = {}
    for k, (a, b) in pts2.items():
        a, b = a - h, b - h
        d = (b - a) / np.linalg.norm(b - a)
        n = np.array([d[1], -d[0]])  # outward normal (x radial)
        if n[0] < 0:
            n = -n
        quad = [a, b, a + n * FINGER_T, b + n * FINGER_T]
        p = np.array([[q[0], y, q[1]] for q in quad for y in (-FINGER_W / 2, FINGER_W / 2)])
        out[k] = p @ ry_rad(theta).T
    return out


def finger_frame(i, yaw_deg):
    """Hinge position (C frame) and rotation of finger i (rest: theta = 0)."""
    phi = math.radians(yaw_deg) + i * 2 * math.pi / N_FINGERS
    r = rz(math.degrees(phi))
    return r @ np.array([HINGE[0], 0.0, HINGE[1]]), r


def finger_pad_radius(theta):
    """Radius (C frame) of the pad kink for hinge angle theta, and its height."""
    v = np.array([PAD[0] - HINGE[0], 0.0, PAD[1] - HINGE[1]])
    w = ry_rad(theta) @ v
    return HINGE[0] + w[0], HINGE[1] + w[2]


# ---------------------------------------------------------------- flask start on the bench
FLASK_X = (0.300, 0.460)
FLASK_ABS_Y = (0.240, 0.300)
SHAKER_JITTER = 0.008
OFFICIAL_MIN_SEP = 0.030  # other seeded starts keep their flask >= 30 mm from the official start or its mirror


FREE_SOCKETS = (2, 3)  # the free clamp is in the near row (the far row is beyond the arms' reach at clamp height)


def _raw_spec(seed):
    rng = np.random.default_rng(int(seed))
    socket = int(FREE_SOCKETS[int(rng.integers(len(FREE_SOCKETS)))])
    sdx, sdy = (float(v) for v in rng.uniform(-SHAKER_JITTER, SHAKER_JITTER, 2))
    fx = float(rng.uniform(*FLASK_X))
    fy = float(rng.uniform(*FLASK_ABS_Y)) * socket_side(socket)
    yaw = float(rng.uniform(0.0, 72.0))
    return dict(socket=socket, shaker_dx=sdx, shaker_dy=sdy, flask_x=fx, flask_y=fy, clamp_yaw=yaw)


def _guard_point(socket):
    """The start a recorded official trajectory fits: the official flask start for the official free clamp, and its
    left-right mirror (y -> -y) for the other near clamp (the YAM pair is mirror-symmetric, so a mirrored replay of
    the official trace would fit there)."""
    off = _raw_spec(OFFICIAL_SEED)
    y = off["flask_y"] if socket == off["socket"] else -off["flask_y"]
    return np.array([off["flask_x"], y])


def _push_from_official(spec):
    """Keep every non-official seeded start at least OFFICIAL_MIN_SEP away from the official flask start (same free
    clamp) or from its mirror image (other free clamp), so that neither a recorded trajectory nor its mirror can be
    replayed onto a near-identical start. The official start itself is unchanged."""
    r = _guard_point(spec["socket"])
    d = np.array([spec["flask_x"], spec["flask_y"]]) - r
    n = float(np.linalg.norm(d))
    if n >= OFFICIAL_MIN_SEP:
        return spec
    u = d / n if n > 1e-9 else np.array([1.0, 0.0])
    side = socket_side(spec["socket"])
    for sgn in (1.0, -1.0):  # push along the offset; if that leaves the region, the other way
        p = r + sgn * u * OFFICIAL_MIN_SEP
        if FLASK_X[0] <= p[0] <= FLASK_X[1] and FLASK_ABS_Y[0] <= p[1] * side <= FLASK_ABS_Y[1]:
            break
    else:
        p = np.array([r[0] + (OFFICIAL_MIN_SEP if r[0] < np.mean(FLASK_X) else -OFFICIAL_MIN_SEP), r[1]])
    return dict(spec, flask_x=float(p[0]), flask_y=float(p[1]))


SETUPS = (  # named (unseeded) setups: socket, shaker offset, flask start, finger phase
    dict(socket=3, shaker_dx=0.0, shaker_dy=0.0, flask_x=0.38, flask_y=-0.27, clamp_yaw=0.0),
    dict(socket=2, shaker_dx=0.004, shaker_dy=-0.004, flask_x=0.33, flask_y=0.26, clamp_yaw=20.0),
    dict(socket=3, shaker_dx=-0.004, shaker_dy=0.003, flask_x=0.44, flask_y=-0.29, clamp_yaw=40.0),
    dict(socket=2, shaker_dx=0.002, shaker_dy=0.006, flask_x=0.31, flask_y=0.25, clamp_yaw=60.0),
)


def setup_spec(setup):
    """Start spec for a named setup (int) or a seeded one (('seed', s))."""
    if isinstance(setup, (tuple, list)) and setup and setup[0] == "seed":
        s = int(setup[1])
        spec = _raw_spec(s)
        if s != OFFICIAL_SEED:
            spec = _push_from_official(spec)
        return dict(spec, name=f"seed {s}")
    return dict(SETUPS[int(setup) % len(SETUPS)], name=f"setup {int(setup) % len(SETUPS)}")


def platform_xy(spec):
    return np.array([SHAKER_XY[0] + spec["shaker_dx"], SHAKER_XY[1] + spec["shaker_dy"]])


def socket_center(spec, socket):
    """Clamp origin (C frame origin) of a socket, station frame, platform at rest."""
    p = platform_xy(spec)
    ox, oy = SOCKETS[socket]
    return np.array([p[0] + ox, p[1] + oy, PLATFORM_Z])


def loaded_sockets(spec):
    return [s for s in range(4) if s != spec["socket"]]


def flask_start(spec):
    return np.array([spec["flask_x"], spec["flask_y"], TOP]), np.eye(3)


# ---------------------------------------------------------------- measurements (pure geometry)
def flask_in_clamp(pf, rf, pc, rc):
    """Flask pose relative to the free clamp: bottom-centre offset (C frame), axis tilt (deg)."""
    q = rc.T @ (np.asarray(pf, float) - pc)
    ax = rf[:, 2]
    tilt = math.degrees(math.acos(float(np.clip(ax @ rc[:, 2], -1.0, 1.0))))
    return dict(dx=float(q[0]), dy=float(q[1]), dz=float(q[2]), radial=float(math.hypot(q[0], q[1])), tilt_deg=tilt)


def finger_theta(rf_i, rc, rest_i):
    """Hinge angle of a finger from its rotation, the clamp base rotation and its rest rotation (in C)."""
    m = rest_i.T @ rc.T @ rf_i
    return float(math.atan2(m[0, 2], m[0, 0]))


def point_to_flask(point, pf, rf):
    """Distance (m) from a point to the flask surface (axisymmetric profile; negative inside)."""
    q = rf.T @ (np.asarray(point, float) - pf)
    z = float(np.clip(q[2], 0.0, F_HEIGHT))
    r = math.hypot(q[0], q[1])
    radial = r - flask_radius(z if z > 0 else 1e-6)
    above = q[2] - F_HEIGHT if q[2] > F_HEIGHT else (-q[2] if q[2] < 0 else 0.0)
    if above > 0:
        return float(math.hypot(max(radial, 0.0), above))
    return float(radial)
