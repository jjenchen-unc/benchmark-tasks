"""Scene layout and geometry of the panel-deadfront task (station frame: x forward from the arm bases, y to the
robot's left, z up; metres). Engine-independent (numpy only).

Panel frame P: origin at the centre of the load-centre tub's front face (the plane the dead-front's back face rests
on), axes parallel to the station axes (x points into the wall). Cover frame V: origin at the centre of the
dead-front's back face, same axes as P when the cover is installed (cover front face at x = -COVER_T).

The load centre is a Square D Homeline HOM612L100S class (6 spaces / 12 circuits, 100 A main lugs, surface
mount). Its catalogue dimensions could not be checked from the work host, so the enclosure, cover and breaker
dimensions below are representative of that size class (see README fidelity).
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475  # station table top (yam_arena TABLE_TOP["station"])
INCH = 0.0254


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


# ---------------------------------------------------------------- wall stub and tub (panel frame P)
TUB_W, TUB_H, TUB_D = 0.200, 0.320, 0.095  # enclosure outside (y, z, x)
TUB_WALL = 0.002
P0 = np.array([0.380, 0.0, TOP + 0.135 + TUB_H / 2])  # tub front-face centre (station); bottom 135 mm above the table
WALL_T = 0.019  # plywood backboard
WALL_W, WALL_H = 0.42, 0.44
FLANGE_H = 0.025  # inward front flanges at the tub's top and bottom (screw holes)
FLANGE_T = 0.002

# breakers: 6 spaces, two columns (left y > 0, right y < 0) of three, 1 in per space
SPACE_PITCH = INCH
ROW_Z = (0.0580, 0.0580 + INCH, 0.0580 + 2 * INCH)  # space centres (z, panel frame): upper half of the tub
COL_Y = {"L": 0.045, "R": -0.045}  # breaker face (handle shroud) centres
SPACES = [(c, k) for c in ("L", "R") for k in range(3)]  # space index = position in this list
FACE_W, FACE_H = 0.046, 0.021  # handle shroud (y, z)
FACE_FRONT = -0.009  # shroud front face 9 mm in front of the tub face (7.5 mm past the cover's front)
FACE_BACK = 0.010
BODY_BACK = 0.070  # breaker body depth behind the tub face
BODY_W = 0.076
HANDLE = (0.009, 0.012, 0.007)  # toggle handle (y, z, x): 7 mm proud of the shroud (16 mm off the tub face)
HANDLE_OFF = 0.008  # handle offset from the shroud centre toward ON (outboard) / OFF (inboard)
# all six spaces hold a single-pole breaker (HOM115 / HOM120 class); the handles are ON or OFF per trial

# lower cover screws (#10-32 hex washer head, 5/16 in hex), started in the bottom flange
SCREW_Y = (0.075, -0.075)  # (left, right)
SCREW_Z = -TUB_H / 2 + FLANGE_H / 2  # -0.1475
SHANK_R = 0.00241  # #10 major diameter 4.83 mm
HEAD_R = 0.0057  # washer face 11.4 mm
HEAD_HEX_R = 0.0046  # hex across corners 9.2 mm (5/16 in across flats)
HEAD_WASHER_T = 0.0008
HEAD_HEX_T = 0.0030
PITCH = INCH / 32  # 10-32 thread: 0.794 mm per turn
START_GAP = 0.0035  # started screws: head 3.5 mm off the seated cover's front face (4.4 turns to seat)
SNUG_INTERFERENCE = 0.0  # the head stops on the seated cover's front face (traps it, no play; no interference force)

# ---------------------------------------------------------------- dead-front cover (frame V)
COVER_W, COVER_H = 0.250, 0.340  # 25 mm past the tub sides, 10 mm past top and bottom
COVER_T = 0.0015  # 16 gauge steel
STEEL = 7850.0
WIN_CLEAR_Y = 0.0010  # window clearance per side around a shroud
WIN_CLEAR_Z = 0.0015
SLOT_W = 0.008  # open-bottom hanging slots over the lower screws (1.6 mm lateral play a side on the shank)
SLOT_TOP = SCREW_Z + SHANK_R  # slot tops rest on the shanks with the cover at its installed height
HEM_W = 0.015  # both side edges hemmed (folded back on the rear face): 3.0 mm thick where the hands pinch it
HEM_Z = (-0.150, COVER_H / 2)  # the hem stops 20 mm above the bottom edge
GRIP_IN = 0.006  # TCP 6 mm in from the side edge (pad tips about 18 mm in)
GRIP_Z = 0.0  # at the height of the cover's centre of mass

# ---------------------------------------------------------------- bench: cover stand and nut-driver rest
# The cover stands upright in a grooved bench stand in front of the panel, front face toward the robot, leaning back
# LEAN deg. Stand frame S: origin at the groove floor centre, rotated by ry(LEAN); the groove runs along y.
STAND = np.array([0.200, 0.0, TOP + 0.012])
LEAN = 4.0
GROOVE_W = 0.0035  # 1.5 mm plate in a 3.5 mm groove (the hems stop above it)
GROOVE_D = 0.012
STAND_BASE = (0.060, 0.270, 0.012)  # (x, y, z) block under the groove
GROOVE_WALL_T = 0.008
BACKREST_H = 0.060  # rear wall of the groove carried up 60 mm in the middle (|y| < BACKREST_Y): the cover leans on it
BACKREST_Y = 0.085
# driver holster: a block on the bench right of the cover stand; the driver stands in it socket down, its handle
# resting on the rim
DRIVER_HOLSTER = np.array([0.340, -0.250, TOP])
HOLSTER_SIZE = (0.050, 0.050, 0.060)
HOLSTER_HOLE = 0.0155  # square hole the shaft and socket stand in (socket 13.6 mm: the driver stands within ~2 deg of upright)
# ratcheting 5/16 in nut driver (frame D: origin at the socket mouth on the axis, +z toward the handle)
DRV_SOCKET_R = (0.0050, 0.0068)  # round bore over the 9.2 mm hex (0.2 mm clearance at the 12-gon flats)
DRV_SOCKET_D = 0.007  # socket bore depth (a cap closes it)
DRV_LEADIN = (0.0064, 0.0020)  # mouth chamfer: bore radius 6.4 mm at the mouth, down to the bore over 2 mm
DRV_SHAFT_R = 0.0045
DRV_SHAFT_L = 0.040  # stubby driver (1.5 in shaft)
DRV_HANDLE_R = 0.016
DRV_HANDLE_L = 0.105
DRV_MASS = 0.16
DRV_HANDLE_Z0 = DRV_SOCKET_D + 0.002 + DRV_SHAFT_L  # handle starts here (D frame z)
RATCHET_SIGN = 1.0  # tightening = right-hand rotation about the screw axis pointing into the wall (+x_P)

# ---------------------------------------------------------------- start variation (seeded trials)
SEED_STAND_DX = 0.010  # stand moved +-10 mm toward / away from the robot
SEED_STAND_DY = 0.015  # and +-15 mm sideways
SEED_COVER_DY = 0.010  # cover slid +-10 mm along the groove
SEED_PANEL_DY = 0.015  # panel (wall stub) +-15 mm along the wall
SEED_PANEL_DZ = 0.005  # and +-5 mm up / down


def space_center(space):
    c, k = SPACES[space]
    return np.array([0.0, COL_Y[c], ROW_Z[k]])


def setup_spec(setup):
    """Start state: {'on': per-space handle ON flag, 'stand_dxy', 'cover_dy', 'panel_dyz', 'name'}. ``setup`` is a
    named setup (int) or ('seed', n)."""
    if isinstance(setup, tuple) and setup[0] == "seed":
        rng = np.random.default_rng(int(setup[1]))
        on = [bool(b) for b in rng.integers(0, 2, size=6)]
        sdx = float(rng.uniform(-SEED_STAND_DX, SEED_STAND_DX))
        sdy = float(rng.uniform(-SEED_STAND_DY, SEED_STAND_DY))
        cdy = float(rng.uniform(-SEED_COVER_DY, SEED_COVER_DY))
        pdy = float(rng.uniform(-SEED_PANEL_DY, SEED_PANEL_DY))
        pdz = float(rng.uniform(-SEED_PANEL_DZ, SEED_PANEL_DZ))
        return dict(on=on, stand_dxy=[sdx, sdy], cover_dy=cdy, panel_dyz=[pdy, pdz], name=f"seed {int(setup[1])}")
    named = (
        dict(on=[True, False, True, True, True, False], stand_dxy=[0.008, -0.012], cover_dy=0.007,
             panel_dyz=[0.012, -0.004]),
        dict(on=[False, True, True, True, False, True], stand_dxy=[-0.009, 0.010], cover_dy=-0.008,
             panel_dyz=[-0.013, 0.004]),
        dict(on=[True, True, False, False, True, True], stand_dxy=[0.0, 0.014], cover_dy=0.0,
             panel_dyz=[0.006, 0.0]),
    )
    k = int(setup) % len(named)
    return dict(named[k], name=f"setup {k}")


def panel_origin(setup):
    sp = setup_spec(setup)
    return P0 + np.array([0.0, sp["panel_dyz"][0], sp["panel_dyz"][1]])


def to_station(p_panel, setup):
    return panel_origin(setup) + np.asarray(p_panel, float)


def cover_installed_pose(setup):
    """(pos, rot) of the cover frame installed (hung on the screws, flat on the tub)."""
    return panel_origin(setup).copy(), np.eye(3)


def stand_pos(setup):
    sp = setup_spec(setup)
    return STAND + np.array([sp["stand_dxy"][0], sp["stand_dxy"][1], 0.0])


def cover_stand_pose(setup):
    """(pos, rot) of the cover frame standing in the groove (bottom edge on the groove floor, back face against the
    backrest)."""
    sp = setup_spec(setup)
    r = ry(LEAN)
    pos = stand_pos(setup) + r @ np.array([GROOVE_W / 2 - 0.0001, sp["cover_dy"], COVER_H / 2 + 0.00005])
    return pos, r


def driver_rot_x(phi):
    """Driver frame with the socket pointing +x (z_D = -x) and roll phi (x_D = (0, cos phi, sin phi))."""
    zd = np.array([-1.0, 0.0, 0.0])
    xd = np.array([0.0, math.cos(phi), math.sin(phi)])
    return np.column_stack([xd, np.cross(zd, xd), zd])


def driver_holster_pose():
    """(pos, rot) of the driver frame standing socket down in its holster (handle on the rim)."""
    pos = DRIVER_HOLSTER + np.array([0.0, 0.0, HOLSTER_SIZE[2] + 0.0001 - DRV_HANDLE_Z0])
    return pos, np.eye(3)  # D +z (toward the handle) points up


def screw_axis_point(i, setup):
    """Station point on screw i's axis at the tub face (x_P = 0)."""
    return to_station(np.array([0.0, SCREW_Y[i], SCREW_Z]), setup)


def head_under_x(advance):
    """Panel-frame x of the screw head's underside for a given advance (m) from the started position."""
    return -(COVER_T + START_GAP) + advance


def max_advance():
    """Advance at which the head would bottom on the tub face (no cover under it)."""
    return COVER_T + START_GAP


# ---------------------------------------------------------------- cover pieces
def window_rects():
    """Window rectangles (y0, y1, z0, z1) in the cover frame: one opening per breaker column (all three twist-outs of
    each column removed, since every space holds a breaker)."""
    out = []
    for c in ("L", "R"):
        y = COL_Y[c]
        out.append((y - FACE_W / 2 - WIN_CLEAR_Y, y + FACE_W / 2 + WIN_CLEAR_Y,
                    ROW_Z[0] - FACE_H / 2 - WIN_CLEAR_Z, ROW_Z[-1] + FACE_H / 2 + WIN_CLEAR_Z))
    return out


def slot_rects():
    return [(y - SLOT_W / 2, y + SLOT_W / 2, -COVER_H / 2 - 0.001, SLOT_TOP) for y in SCREW_Y]


def cover_boxes():
    """Solid rectangles (y0, y1, z0, z1) tiling the cover plate minus its windows and slots."""
    holes = window_rects() + slot_rects()
    ys = sorted({-COVER_W / 2, COVER_W / 2, -(COVER_W / 2 - HEM_W), COVER_W / 2 - HEM_W}
                | {v for h in holes for v in h[:2] if abs(v) < COVER_W / 2})
    zs = sorted({-COVER_H / 2, COVER_H / 2} | {v for h in holes for v in h[2:] if abs(v) < COVER_H / 2})
    out = []
    for y0, y1 in zip(ys[:-1], ys[1:]):
        run = None
        for z0, z1 in zip(zs[:-1], zs[1:]):
            yc, zc = (y0 + y1) / 2, (z0 + z1) / 2
            solid = not any(h[0] < yc < h[1] and h[2] < zc < h[3] for h in holes)
            if solid:
                run = (z0, z1) if run is None else (run[0], z1)
            elif run is not None:
                out.append((y0, y1, run[0], run[1]))
                run = None
        if run is not None:
            out.append((y0, y1, run[0], run[1]))
    return out


def cover_mass_props():
    """(mass, centre of mass (V), diagonal inertia about the COM in V axes) of the plate."""
    m_tot, c_acc, items = 0.0, np.zeros(3), []
    for y0, y1, z0, z1 in cover_boxes():
        w, h = y1 - y0, z1 - z0
        m = STEEL * w * h * COVER_T
        c = np.array([-COVER_T / 2, (y0 + y1) / 2, (z0 + z1) / 2])
        items.append((m, c, w, h))
        m_tot += m
        c_acc += m * c
    for sy in (1.0, -1.0):
        w, h = HEM_W, HEM_Z[1] - HEM_Z[0]
        m = STEEL * w * h * COVER_T
        c = np.array([COVER_T / 2, sy * (COVER_W / 2 - HEM_W / 2), (HEM_Z[0] + HEM_Z[1]) / 2])
        items.append((m, c, w, h))
        m_tot += m
        c_acc += m * c
    com = c_acc / m_tot
    ixx = iyy = izz = 0.0
    for m, c, w, h in items:
        d = c - com
        ixx += m * (w * w + h * h) / 12 + m * (d[1] ** 2 + d[2] ** 2)
        iyy += m * (h * h + COVER_T ** 2) / 12 + m * (d[0] ** 2 + d[2] ** 2)
        izz += m * (w * w + COVER_T ** 2) / 12 + m * (d[0] ** 2 + d[1] ** 2)
    return m_tot, com, np.array([ixx, iyy, izz])


def hem_boxes():
    """Hem strips behind the plate (x from 0 to COVER_T in V), (y0, y1, z0, z1)."""
    return [(COVER_W / 2 - HEM_W, COVER_W / 2, HEM_Z[0], HEM_Z[1]), (-COVER_W / 2, -COVER_W / 2 + HEM_W, HEM_Z[0], HEM_Z[1])]


def cover_corners_v():
    """Back-face corner points of the cover (V frame): top-left, top-right, bottom-left, bottom-right."""
    return [np.array([0.0, sy * COVER_W / 2, sz * COVER_H / 2]) for sz in (1, -1) for sy in (1, -1)]


def cylinder(r, z0, z1, n=24, center=(0.0, 0.0)):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = lambda z: np.c_[center[0] + r * np.cos(a), center[1] + r * np.sin(a), np.full(n, z)]  # noqa: E731
    return np.r_[ring(z0), ring(z1)]


def ring_pieces(r0, r1, z0, z1, n=12, r0_at_z0=None):
    """Convex pieces of a tube (axis z); ``r0_at_z0`` makes the bore conical (a lead-in widening toward z0)."""
    out = []
    ra = r0 if r0_at_z0 is None else r0_at_z0
    for i in range(n):
        a0, a1 = 2 * np.pi * i / n, 2 * np.pi * (i + 1) / n
        pts = []
        for a in (a0, a1):
            for r, z in ((ra, z0), (r0, z1), (r1, z0), (r1, z1)):
                pts.append([r * math.cos(a), r * math.sin(a), z])
        out.append(np.array(pts))
    return out
