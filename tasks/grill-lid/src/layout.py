"""Geometry and start states of the grill-lid task (pure NumPy; no simulator).

Weber Q 1200 tabletop LP gas grill (black, model 51010001) on the station table, front toward the robot.
Grill frame G: origin on the table top under the cookbox centre, +x toward the grill's rear (away from the robot),
+y to the grill's left (the robot's left), +z up. Station frame: x forward from the line between the arm bases, y to
the robot's left of their midpoint, z up (yam_arena "station" scene; table top z = TOP).

Outline (owner's-manual / retailer figures, from memory; not checked against a drawing, see README fidelity): lid
closed about 15.5 in (39 cm) high, 40.9 in (104 cm) wide with the side tables out, 16.5 in (42 cm) deep; about 30 lb
(13.6 kg). Part sizes below are estimates that fit that outline.
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475  # station table top (yam_arena station TABLE_TOP)


def rz(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def ry(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rx(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


# ------------------------------------------------------------------------------------------------ grill body (root)
BODY_MASS = 11.4  # kg: 13.6 kg grill less the lid, knob and button
FOOT_SIZE = (0.050, 0.050, 0.020)
FEET = ((-0.150, 0.290), (-0.150, -0.290), (0.150, 0.290), (0.150, -0.290))  # (x, y) of the four rubber feet
FRAME = ((-0.175, 0.175), (-0.340, 0.340), 0.020, 0.075)  # x range, y range, z0, z1: cast frame under the cookbox
COOKBOX = ((-0.180, 0.180), (-0.215, 0.215), 0.075, 0.215)  # outer x, y, z0, rim z (cast aluminium)
WALL_T = 0.006
RIM_Z = COOKBOX[3]
CONTROL = ((-0.200, -0.150), (-0.320, -0.165), 0.075, 0.200)  # control housing at the right front (x, y, z)
PANEL_X = CONTROL[0][0]  # front face of the control housing (knob and igniter mount here)
TABLES = (((-0.150, 0.150), (0.345, 0.520)), ((-0.150, 0.150), (-0.520, -0.345)))  # side tables (x, y), folded out
TABLE_Z = (0.180, 0.192)  # side-table top surface z range
CYL_CENTER = np.array([0.010, -0.420, 0.110])  # 1 lb (16.4 oz) LP cylinder, lying along -y under the right table
CYL_R, CYL_L = 0.050, 0.200
GRATE_Z = 0.190  # cooking grate top (two porcelain-enamelled cast-iron halves; visual)
BURNER_Z = 0.120  # burner tube (oval loop; visual), flame drawn here
BURNER_OVAL = (0.115, 0.150)  # semi-axes x, y of the burner loop

# ------------------------------------------------------------------------------------------------ lid (revolute)
HINGE = np.array([0.185, 0.0, RIM_Z - 0.015])  # rear hinge axis (along y), on the hinge lugs 15 mm below the rim
LID_MASS = 2.1  # kg, cast-aluminium lid with thermometer and handle (estimate)
LID_FOOT = ((-0.188, 0.185), (-0.220, 0.220))  # lid footprint at the rim (x, y)
LID_TOP = ((-0.120, 0.120), (-0.180, 0.180), RIM_Z + 0.150)  # dome top face (x, y, z): closed height 0.365 m
LID_MID = ((-0.175, 0.170), (-0.215, 0.215), RIM_Z + 0.075)  # dome shoulder ring
LID_COM = np.array([0.020, 0.0, RIM_Z + 0.068])  # rear-biased: hinge lugs and thermometer at the rear
LID_OPEN_STOP_DEG = 95.0  # rests open on its rear stop (past the balance angle)
HANDLE_R = 0.0125  # full-width front handle: 25 mm round bar (glass-reinforced nylon)
HANDLE_X = -0.240  # bar axis, grill frame, lid closed (40 mm clear of the lid front for fingers)
HANDLE_Z = RIM_Z + 0.040
HANDLE_HALF = 0.190  # bar length 380 mm
HANDLE_POSTS = (0.180, -0.180)  # y of the two brackets joining the bar to the lid front
HINGE_FRICTION_NM = 0.2  # Coulomb friction at the hinge pins (estimate)

# ------------------------------------------------------------------------------------------------ knob and igniter
KNOB_C = np.array([PANEL_X, -0.272, 0.140])  # knob axis on the panel face
KNOB_R = 0.028  # 56 mm round knob
KNOB_T = 0.014  # knob skirt depth (proud of its push travel)
KNOB_GAP = 0.006  # skirt stands this far off the panel when out (push travel 5 mm)
FIN = (0.016, 0.044, 0.012)  # grip fin on the knob face: depth (proud), length, width
FIN_OFF_DEG = 25.0  # at OFF the fin's index end points 25 deg counter-clockwise from 9 o'clock (seen from the front)
PUSH_TRAVEL = 0.005
PUSH_UNLOCK = 0.0030  # pushed in this far, the knob can leave OFF
PUSH_SPRING = 600.0  # N/m
PUSH_PRELOAD = 7.2  # N at rest (spring target 12 mm behind the out stop)
KNOB_MAX_DEG = 170.0
KNOB_HI_DEG = 90.0  # START/HI, counter-clockwise from OFF
KNOB_LO_DEG = 160.0
VALVE_FRICTION_NM = 0.20  # running torque of the valve (estimate)
GATE_NM = 10.0  # OFF stop (stop pin) holds this much torque while the knob is not pushed in
GATE_DEG = 8.0  # within this angle of OFF the stop is engaged unless pushed in
IGN_C = np.array([PANEL_X, -0.200, 0.140])  # igniter push button axis on the panel face
IGN_R = 0.010  # 20 mm red button
IGN_PROUD = 0.010
IGN_TRAVEL = 0.004
IGN_SPRING = 700.0  # N/m
IGN_PRELOAD = 7.0  # N at rest

FRICTION = dict(feet=0.8, metal=0.5, handle=0.7, knob=0.8, button=0.7)

# ------------------------------------------------------------------------------------------------ placement
# The grill stands centred between the arms: at the arms' start (READY) pose their open grippers are 49 cm apart, just
# clear of the 44 cm lid, so the start states vary mostly in depth (x).
NOMINAL = (0.440, 0.000, 0.0)  # grill origin x, y (station) and yaw (deg) of the start-state box's centre
NAMED_SETUPS = ((0.420, 0.008, -2.5), (0.465, -0.008, 2.5), (0.415, -0.006, 2.0), (0.460, 0.006, -2.0),
                (0.428, 0.000, 0.0))
SEEDED_DX = ((-0.025, -0.010), (0.010, 0.025))  # |dx| 10..25 mm from the nominal, either way
SEEDED_DY = (-0.010, 0.010)
SEEDED_YAW = (-3.0, 3.0)


def setup_spec(setup):
    """Start state of a trial: {'x', 'y', 'yaw', 'name'}. ``setup`` is a named setup index (int) or ('seed', n): grill
    origin x = NOMINAL + dx with |dx| uniform in 10..25 mm (either way), y = NOMINAL + uniform SEEDED_DY, yaw uniform
    in SEEDED_YAW, from seed n. Lid closed, knob OFF, igniter out at every start."""
    if isinstance(setup, (tuple, list)) and setup[0] == "seed":
        rng = np.random.default_rng(int(setup[1]))
        lo, hi = SEEDED_DX[int(rng.integers(2))]
        dx = float(rng.uniform(lo, hi))
        dy = float(rng.uniform(*SEEDED_DY))
        yaw = float(rng.uniform(*SEEDED_YAW))
        return dict(x=NOMINAL[0] + dx, y=NOMINAL[1] + dy, yaw=yaw, name=f"seed {int(setup[1])}")
    k = int(setup) % len(NAMED_SETUPS)
    x, y, yaw = NAMED_SETUPS[k]
    return dict(x=x, y=y, yaw=yaw, name=f"setup {k}")


def grill_pose(spec):
    """(position, rotation) of the grill frame in the station frame."""
    return np.array([spec["x"], spec["y"], TOP]), rz(spec["yaw"])


def lid_rot(deg):
    """Lid rotation about the hinge (grill frame): positive lifts the front."""
    return ry(deg)


def handle_point(deg, y=0.0):
    """Handle bar axis point at grill-frame y, lid at ``deg``, in the grill frame."""
    p = np.array([HANDLE_X, y, HANDLE_Z])
    return HINGE + lid_rot(deg) @ (p - HINGE)


def to_station(spec, p_g):
    pos, rot = grill_pose(spec)
    return pos + rot @ np.asarray(p_g, float)


def balance_deg():
    """Lid angle at which its centre of mass is straight above the hinge."""
    d = LID_COM - HINGE
    return 90.0 - math.degrees(math.atan2(d[2], -d[0]))


def handle_force_closed():
    """Vertical force at the handle that holds the closed lid (N): lid weight x COM lever / handle lever."""
    return LID_MASS * 9.81 * (HINGE[0] - LID_COM[0]) / (HINGE[0] - HANDLE_X)


# ------------------------------------------------------------------------------------------------ shapes
def box(center, size):
    hx, hy, hz = (s / 2 for s in size)
    c = np.asarray(center, float)
    return np.array([[sx * hx, sy * hy, sz * hz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) + c


def box_xyz(xr, yr, zr):
    return box(((xr[0] + xr[1]) / 2, (yr[0] + yr[1]) / 2, (zr[0] + zr[1]) / 2),
               (xr[1] - xr[0], yr[1] - yr[0], zr[1] - zr[0]))


def cyl(axis, center, r, length, n=16):
    """Vertices of a cylinder (n-gon prism) along unit ``axis`` ('x', 'y' or 'z'), centred at ``center``."""
    c = np.asarray(center, float)
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    ring = np.stack([r * np.cos(a), r * np.sin(a)], axis=1)
    out = []
    for h in (-length / 2, length / 2):
        for u, v in ring:
            if axis == "x":
                out.append([h, u, v])
            elif axis == "y":
                out.append([u, h, v])
            else:
                out.append([u, v, h])
    return np.array(out) + c


def lid_hull():
    """Dome of the lid as one convex hull (grill frame, lid closed)."""
    pts = []
    for (xr, yr, z) in ((LID_FOOT[0], LID_FOOT[1], RIM_Z), (LID_MID[0], LID_MID[1], LID_MID[2]), LID_TOP):
        for x in xr:
            for y in yr:
                pts.append([x, y, z])
    return np.array(pts)


def cookbox_hulls():
    (x0, x1), (y0, y1), z0, z1 = COOKBOX
    t = WALL_T
    return [
        box_xyz((x0, x1), (y0, y1), (z0, z0 + 0.012)),  # floor
        box_xyz((x0, x0 + t), (y0, y1), (z0, z1)),  # front wall
        box_xyz((x1 - t, x1), (y0, y1), (z0, z1)),  # rear wall
        box_xyz((x0, x1), (y0, y0 + t), (z0, z1)),  # right wall
        box_xyz((x0, x1), (y1 - t, y1), (z0, z1)),  # left wall
    ]
