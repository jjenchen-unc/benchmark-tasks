"""Scene layout and geometry of the panel-deadfront task (station frame: x forward from the arm bases, y to the
robot's left, z up; metres). Engine-independent (numpy only).

Panel frame P: origin at the centre of the load-centre tub's front face (the plane the dead-front's back face rests
on), axes parallel to the station axes (x points into the wall). Cover frame V: origin at the centre of the
dead-front's back face, same axes as P when the cover is installed (cover front face at x = -COVER_T).

The product pairing and dimensional estimates are documented in hardware-fit.md.
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
TUB_W, TUB_H, TUB_D = 15*INCH, 30*INCH, 4.4*INCH
TUB_WALL = 0.002
P0 = np.array([0.509, 0.0, TOP + 0.009 + TUB_H / 2])
WALL_T = 0.019
WALL_W, WALL_H = 0.43, 0.82
FLANGE_T = 0.002
COVER_W, COVER_H = 0.340, 0.668
COVER_T = 0.0012
STEEL = 7850.0
GUIDE_Z = COVER_H / 2 - 0.020
GUIDE_Y = COVER_W / 2 - 0.009
INSERT_ANGLE = 45.0
# Approach offsets of the pivot from the final guide midpoint in panel x/z (metres).
# The top corner must clear the front return before it rises above the side protrusions.
ENTRY_WAYPOINTS = ((-.055, -.010), (-.014, -.006), (0., 0.))

SPACE_PITCH = INCH
ROW_Z = tuple(0.045 - (2-k)*SPACE_PITCH for k in range(3))
COL_Y = {"L": 0.052, "R": -0.052}
SPACES = [(c, k) for c in ("L", "R") for k in range(3)]
FACE_W, FACE_H = 0.074, 0.0224
FACE_FRONT, FACE_BACK = -0.009, 0.010
BODY_BACK, BODY_W = 0.079, 0.099
HANDLE = (0.009, 0.012, 0.007)
WIN_CLEAR_Y, WIN_CLEAR_Z = 0.001, 0.0015

# Two symmetric edge notches, one lower-right fastening location in this mounting orientation.
NOTCH_Y = (0.080, -0.080)
SCREW_Y = (NOTCH_Y[1],)
SLOT_W, NOTCH_DEPTH = 0.007, 0.008
SLOT_TOP = -COVER_H / 2 + NOTCH_DEPTH
SCREW_Z = SLOT_TOP - 0.003
SHANK_R, HEAD_R, HEAD_T = 0.00241, 0.0057, 0.0035
SHANK_L = 0.018
# Approximate effective travel per turn for the benchmark screw and magnetic driver.
PITCH = 2 * INCH / 32
START_GAP = SHANK_L - COVER_T
SNUG_INTERFERENCE = 0.0
HEM_W = 0.009
HEM_Z = (-COVER_H/2 + 0.016, COVER_H/2 - 0.016)
# Removable training handles fitted to solid metal beyond the breaker windows.
GRIP_X, GRIP_Y, GRIP_Z = -.049, .132, -.007
GRIP_HALF = np.array([.014, .016, .018])
MAGNET_R = .018
HANDLE_MASS = .085


def grip_center(sign):
    return np.array([GRIP_X, sign*GRIP_Y, GRIP_Z])


def grip_frame(sign):
    # Both hands approach from outboard and slightly in front. Closing axes are mostly vertical.
    left = rz(-99.3) @ ry(76.6) @ rx(-42.5)
    mirror = np.diag([1., -1., 1.])
    return left if sign > 0 else mirror @ left @ np.diag([-1., 1., 1.])


# ---------------------------------------------------------------- bench: cover stand and nut-driver rest
# The cover stands upright in a grooved bench stand in front of the panel, front face toward the robot, leaning back
# LEAN deg. Stand frame S: origin at the groove floor centre, rotated by ry(LEAN); the groove runs along y.
STAND = np.array([0.200, 0.0, TOP + 0.012])
LEAN = 4.0
GROOVE_W = 0.0035  # 1.2 mm plate in a 3.5 mm groove (the hems stop above it)
GROOVE_D = 0.012
STAND_BASE = (0.060, 0.270, 0.012)  # (x, y, z) block under the groove
GROOVE_WALL_T = 0.008
BACKREST_H = 0.060  # rear wall of the groove carried up 60 mm in the middle (|y| < BACKREST_Y): the cover leans on it
BACKREST_Y = 0.085
# driver holster: a block on the bench right of the cover stand; the driver stands in it socket down, its handle
# resting on the rim
DRIVER_HOLSTER = np.array([0.340, -0.250, TOP])
HOLSTER_SIZE = (0.050, 0.050, 0.085)
HOLSTER_HOLE = 0.0155  # square hole the shaft and socket stand in (retains the vertical shaft and leaves room for the screw head)
# Ratcheting magnetic driver; frame D origin is the working face, +z toward the handle.
DRV_SOCKET_R = (0.003, 0.0053)
DRV_SOCKET_D = 0.007
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
        on = [False] * len(SPACES)
        sdx = float(rng.uniform(-SEED_STAND_DX, SEED_STAND_DX))
        sdy = float(rng.uniform(-SEED_STAND_DY, SEED_STAND_DY))
        cdy = float(rng.uniform(-SEED_COVER_DY, SEED_COVER_DY))
        pdy = float(rng.uniform(-SEED_PANEL_DY, SEED_PANEL_DY))
        pdz = float(rng.uniform(-SEED_PANEL_DZ, SEED_PANEL_DZ))
        return dict(on=on, stand_dxy=[sdx, sdy], cover_dy=cdy, panel_dyz=[pdy, pdz], name=f"seed {int(setup[1])}")
    named = (
        dict(on=[False] * len(SPACES), stand_dxy=[0.008, -0.012], cover_dy=0.007,
             panel_dyz=[0.012, -0.004]),
        dict(on=[False] * len(SPACES), stand_dxy=[-0.009, 0.010], cover_dy=-0.008,
             panel_dyz=[-0.013, 0.004]),
        dict(on=[False] * len(SPACES), stand_dxy=[0.0, 0.014], cover_dy=0.0,
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
    """(pos, rot) of the cover frame installed (upper edge in the guides, lower edge against the mounting land)."""
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
    low = -COVER_H / 2
    slots = [(y - SLOT_W/2, y + SLOT_W/2, lo, hi) for y in NOTCH_Y
            for lo, hi in ((low - .001, low + NOTCH_DEPTH), (-low - NOTCH_DEPTH, -low + .001))]
    return slots + [(-.025, .025, lo, hi) for lo, hi in ((low-.001, low+.009), (-low-.009,-low+.001))]


def upper_fit_boxes():
    """Physical side protrusions and folded upper return in panel coordinates."""
    hw, top = TUB_W/2, COVER_H/2
    for sign in (-1, 1):
        yl, yr = sorted((sign*(GUIDE_Y-.003), sign*hw))
        yield f"upper_guide_{int(sign>0)}", (.002, yl, GUIDE_Z-.005), (.027, yr, GUIDE_Z)
    yield "top_return", (-.005, -hw, top+.0008), (TUB_D, hw, top+.003)
    yield "top_lip", (-.007, -hw, top-.004), (-COVER_T-.0004, hw, top+.003)


def lower_mount_boxes():
    """Raised screw land above the separate full-width sill, split around the pilot hole."""
    # Leviton installation sheet Fig. 12 and the door-open family photograph show the land
    # ending before either sidewall. It spans approximately 65% of the clear enclosure width.
    half_width = .65*(TUB_W-2*TUB_WALL)/2
    bottom = -COVER_H/2
    y, z, rad = SCREW_Y[0], SCREW_Z, SHANK_R+.0003
    sections = ((-half_width, y-rad, bottom, bottom+.020), (y+rad, half_width, bottom, bottom+.020),
                (y-rad, y+rad, bottom, z-rad), (y-rad, y+rad, z+rad, bottom+.020))
    return [((0., ya, za), (FLANGE_T, yb, zb)) for ya, yb, za, zb in sections]


def guide_pose(angle, origin, dx=0.0, dy=0.0, dz=0.0):
    """Pose pivoting about the span between the two upper side guides; positive angle brings the bottom forward."""
    r = ry(angle)
    pivot = np.array([0.0, 0.0, GUIDE_Z])
    return np.asarray(origin) + [dx, dy, dz] + pivot - r @ pivot, r


def screw_in_driver(dpos, drot):
    """Screw tip at +x of the screw frame, 18 mm ahead of the underside of the head."""
    r = np.column_stack((-drot[:, 2], drot[:, 0], -drot[:, 1]))
    return np.asarray(dpos) + r[:, 0] * (SHANK_L + HEAD_T), r


def drive_alignment(dpos, drot, screw_axis, advance):
    axis = -drot[:, 2]
    delta = np.asarray(dpos) - screw_axis
    radial = float(np.linalg.norm(delta[1:]))
    axial = float(delta[0] - head_under_x(advance) + HEAD_T)
    angle = math.degrees(math.acos(np.clip(axis[0], -1.0, 1.0)))
    return (angle < 8 and radial < .001 and -.0007 <= axial <= .0015,
            dict(ang=angle, radial_mm=radial * 1000, axial_mm=axial * 1000))


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
    # Lump each 85 g carrying aid halfway between its magnet base and knob.
    for sy in (1., -1.):
        c = grip_center(sy)
        c[0] /= 2
        m_tot += HANDLE_MASS
        c_acc += HANDLE_MASS*c
        items.append((HANDLE_MASS, c, 2*MAGNET_R, 2*MAGNET_R))
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
