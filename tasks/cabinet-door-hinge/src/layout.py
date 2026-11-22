"""Geometry and start states for the cabinet-door-hinge task (pure numpy; station frame, metres).

Station frame: x forward (away from the robot), y left, z up; the bench top is at TABLE_TOP.

The scene (real parts at real scale):
* a frameless wall-cabinet carcass, 300 (w) x 400 (h) x 300 (d) mm, 18 mm white melamine particleboard, with an
  8 mm back, standing on two 2x4 offcuts laid flat (38 mm) on the bench, its open front facing the robot;
* two Blum CLIP cruciform 0 mm mounting plates (173H7100 class) on the inside of its LEFT side panel, on the 37 mm
  system line, 80 mm from the top and the bottom of the door;
* a 19 mm MDF door, 298 x 398 mm (1.69 kg), with two Blum CLIP top 110 deg full-overlay hinges (71B3550 class) in
  35 mm cup bores (cup centre 22.5 mm from the hinge edge, 80 mm from the door's top and bottom), standing upright in
  the groove of a third 2x4 offcut in front of the cabinet, roughly in its open (90 deg) attitude.

Frames:
* Door frame D (articulation root): origin on the hinge (knuckle) axis at the door's bottom edge; a = from the hinge
  edge to the free edge, b = into the door from its outer face (b = 0 outer face, b = DOOR_T inner face with the
  cups), c = up. The knuckle axis is the line a = b = 0 (the door's outer hinge-edge corner): a single-pivot model of
  the hinge's four-bar linkage (disclosed).
* Arm frame A_i (hinge arm link i): world-aligned when the arm sits on its plate (seated); origin on the plate's
  bearing surface (rail top) on the plate's centre line. The arm lies on the -y side (cabinet interior) of A's origin.
* Door open angle alpha: 90 deg = open square to the cabinet front (hanging attitude), 0 = closed.
"""

from __future__ import annotations

import math

import numpy as np

DEG = math.pi / 180
TABLE_TOP = 0.7475  # yam_arena station bench top (station.TABLE_TOP)

# ---------------------------------------------------------------------------------------------- stock / carcass
BLOCK_T = 0.038  # 2x4 laid flat (1-1/2 in)
BLOCK_W = 0.089  # 3-1/2 in
CAB_W, CAB_H, CAB_D = 0.300, 0.400, 0.300
PANEL_T = 0.018
BACK_T = 0.008
X_FRONT0 = 0.580  # nominal carcass front (station x)
Y_LEFT0 = 0.060  # nominal outer face of the carcass's left side panel (station y)
Z_CAB = TABLE_TOP + BLOCK_T  # carcass underside

# ---------------------------------------------------------------------------------------------- door
DOOR_W, DOOR_H, DOOR_T = 0.298, 0.398, 0.019
MDF_RHO = 750.0
M_DOOR = DOOR_W * DOOR_H * DOOR_T * MDF_RHO  # 1.69 kg
DOOR_DZ = 0.001  # door bottom above the carcass underside when hung (full overlay, 2 mm total height gap)
DOOR_GAP_X = 0.021  # knuckle axis (door's outer hinge-edge corner) in front of the carcass front, = door closed 2 mm
CUP_A = 0.0225  # cup centre from the hinge edge (35 mm cup, 5 mm boring distance)
CUP_R = 0.0175
HINGE_C = (0.080, DOOR_H - 0.080)  # hinge (cup) heights above the door's bottom edge
ALPHA_OPEN = 90.0
ALPHA_MAX = 105.0

# ---------------------------------------------------------------------------------------------- plate and arm
PLATE_LINE = 0.037  # system line: plate centre behind the carcass front
RAIL_H = 0.010  # plate bearing surface (rail top) proud of the panel's inner face (0 mm plate + arm seat)
RAIL_L, RAIL_W = 0.050, 0.014
WING_L, WING_W, WING_T = 0.024, 0.044, 0.003  # cruciform wings (visual)
HOOK_A = np.array([-0.022, 0.0, 0.0])  # arm's front hook (lug) point in A; seats on the plate's front nose
REAR_A = np.array([0.020, 0.0, 0.0])  # arm's rear latch point in A; seats on the plate's rear catch
TOP_A = np.array([0.0, 0.0, 0.012])  # third reference point (clipped constraint)
ARM_X = (-0.030, 0.028)  # arm channel over the rail, in A x
ARM_Y = (-0.012, 0.0)
ARM_Z = 0.0085  # half width (17 mm arm)
M_ARM = 0.10
ARM_INERTIA = (2.0e-4, 2.0e-4, 2.0e-4)  # inflated (numerical stability of the explicit plate springs), see fidelity

# ---------------------------------------------------------------------------------------------- mechanism (hinge)
F_CLICK = 22.0  # N at the arm's rear to push it the last 2 mm over the latch (estimate, see fidelity)
G_CLICK = 0.0020  # latch travel: the detent acts over the last 2 mm of the rear's approach
G_LATCHED = 0.0002  # rear within 0.2 mm of seated = clicked
K_PLATE = 1.5e4  # N/m, plate contact springs (hook, rear stop, clipped points)
C_PLATE = 30.0  # N s/m
HOOK_TOL_X = 0.003  # lug over the nose (along the plate)
HOOK_TOL_Y = 0.003  # lug against the panel side of the nose
HOOK_DROP = (0.0005, 0.003)  # lug comes down onto the nose: from above +0.5 mm to at most 3 mm below the seat
HOOK_REAR_GAP = (0.0015, 0.012)  # rear must be clear of the catch (not on top of it) when the lug drops on
HOOK_BREAK = 0.004  # lug pulled 4 mm off its seat (~60 N) = unhooked
HOOK_REAR_MAX = 0.014  # rear swung more than 14 mm off the plate = the lug rolls off the nose
TILT_MAX_DEG = 10.0
KNUCKLE_FRICTION_NM = 0.8  # hinge pivot friction (capped damping drive), per hinge
KNUCKLE_DAMP = 1.0  # N m s/rad for that drive
CLOSE_NM = 1.2  # self-closing spring torque per hinge below CLOSE_FROM_DEG
CLOSE_FROM_DEG = 35.0
SOFT_DAMP = 2.0  # N m s/rad soft-close damping below SOFT_FROM_DEG (knuckle drive, implicit)
SOFT_CAP_NM = 2.0
SOFT_FROM_DEG = 20.0

# ---------------------------------------------------------------------------------------------- start states
GROOVE_W, GROOVE_D = 0.0195, 0.012
GROOVE_BLOCK_L = 0.260
START_DX = (-0.050, -0.015)  # door start: this far toward the robot from its hung position ...
START_DY = (0.085, 0.125)  # ... and this far outboard (+y): beside the carcass, arms clear of its side panel
START_YAW_DEG = 5.0
CAB_JITTER = (0.015, 0.010)  # carcass position +-dx, +-dy
SETUPS = (
    dict(cdx=0.0, cdy=0.0, sdx=-0.030, sdy=0.100, yaw=0.0),
    dict(cdx=0.012, cdy=-0.008, sdx=-0.038, sdy=0.092, yaw=-3.0),
    dict(cdx=-0.010, cdy=0.006, sdx=-0.022, sdy=0.112, yaw=2.5),
    dict(cdx=0.006, cdy=0.010, sdx=-0.035, sdy=0.108, yaw=3.5),
    dict(cdx=-0.014, cdy=-0.006, sdx=-0.025, sdy=0.095, yaw=-2.0),
)


def setup_spec(setup):
    """Start state dict(cdx, cdy, sdx, sdy, yaw, name). ``setup`` is a named setup index or ('seed', n): carcass
    shifted uniformly within +-15 mm (x) / +-10 mm (y); door start 15..50 mm in front of and 85..125 mm outboard of
    its hung position (beside the carcass; it can never start hooked), turned up to +-5 deg."""
    if isinstance(setup, (tuple, list)) and setup[0] == "seed":
        rng = np.random.default_rng(int(setup[1]))
        return dict(
            cdx=float(rng.uniform(-CAB_JITTER[0], CAB_JITTER[0])),
            cdy=float(rng.uniform(-CAB_JITTER[1], CAB_JITTER[1])),
            sdx=float(rng.uniform(*START_DX)),
            sdy=float(rng.uniform(*START_DY)),
            yaw=float(rng.uniform(-START_YAW_DEG, START_YAW_DEG)),
            name=f"seed {int(setup[1])}",
        )
    k = int(setup) % len(SETUPS)
    return dict(SETUPS[k], name=f"setup {k}")


def rz(deg):
    a = deg * DEG
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def door_rot(alpha_deg):
    """Door frame rotation for open angle alpha (hinge axis fixed): a = (-sin, -cos), b = (cos, -sin)."""
    a = alpha_deg * DEG
    return np.column_stack([[-math.sin(a), -math.cos(a), 0.0], [math.cos(a), -math.sin(a), 0.0], [0.0, 0.0, 1.0]])


def cabinet_origin(spec):
    """(x_front, y_left_outer) of the carcass for a start state."""
    return X_FRONT0 + spec["cdx"], Y_LEFT0 + spec["cdy"]


def knuckle_hung(spec):
    xf, yl = cabinet_origin(spec)
    return np.array([xf - DOOR_GAP_X, yl, Z_CAB + DOOR_DZ])


def plate_origin(spec, i):
    """World origin of A_i when seated: rail top, plate centre line, hinge height."""
    xf, yl = cabinet_origin(spec)
    return np.array([xf + PLATE_LINE, yl - PANEL_T - RAIL_H, Z_CAB + DOOR_DZ + HINGE_C[i]])


def door_pose_hung(spec, alpha=ALPHA_OPEN):
    return knuckle_hung(spec), door_rot(alpha)


def arm_in_door(spec, i):
    """Seated arm pose relative to the door at alpha = 90 (the authored hinge geometry): (pos, rot) in D."""
    dp, dr = door_pose_hung(spec)
    return dr.T @ (plate_origin(spec, i) - dp), dr.T


def knuckle_in_arm(spec, i):
    """The knuckle axis point (at hinge i's height) in A_i."""
    return knuckle_hung(spec) + np.array([0.0, 0.0, HINGE_C[i]]) - plate_origin(spec, i)


def door_pose_start(spec):
    """Door standing in the groove block: hung attitude turned by yaw about its own vertical, moved by (sdx, sdy) and
    lowered so its bottom edge rests on the groove floor."""
    kp, kr = door_pose_hung(spec)
    r = rz(spec["yaw"]) @ kr
    # rotate about the door's centre (not the knuckle) so the groove block stays under the door
    cen_local = np.array([DOOR_W / 2, DOOR_T / 2, 0.0])
    c_h = kp + kr @ cen_local
    p = c_h - r @ cen_local + np.array([spec["sdx"], spec["sdy"], 0.0])
    p[2] = TABLE_TOP + BLOCK_T - GROOVE_D
    return p, r


def arm_pose_from_door(dp, dr, spec, i, alpha=ALPHA_OPEN):
    """World pose of arm i for door pose (dp, dr) with the knuckle at open angle alpha."""
    ap, ar = arm_in_door(spec, i)
    # arm relative to the door turns about the knuckle axis (D's c axis through a = b = 0) by (alpha - 90)
    kp = np.array([0.0, 0.0, HINGE_C[i]])
    rj = rz(alpha - ALPHA_OPEN)
    ap2 = kp + rj @ (ap - kp)
    return dp + dr @ ap2, dr @ rj @ ar


def groove_block_pose(spec):
    """Groove block (2x4 offcut laid flat, groove along its length) centred under the start door."""
    dp, dr = door_pose_start(spec)
    c = dp + dr @ np.array([DOOR_W / 2, DOOR_T / 2, 0.0])
    return np.array([c[0], c[1], TABLE_TOP]), dr @ rz(180.0)  # block frame: x along the groove


def box_pts(center, size, rot=None):
    c = np.asarray(center, float)
    h = np.asarray(size, float) / 2
    pts = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * h
    if rot is not None:
        pts = pts @ np.asarray(rot).T
    return pts + c


def carcass_pieces(spec):
    """(name, center, size, color) boxes of the carcass in the carcass frame = station frame (kinematic)."""
    xf, yl = cabinet_origin(spec)
    yr = yl - CAB_W
    z0 = Z_CAB
    d = CAB_D
    out = [
        ("left", (xf + d / 2, yl - PANEL_T / 2, z0 + CAB_H / 2), (d, PANEL_T, CAB_H), "white"),
        ("right", (xf + d / 2, yr + PANEL_T / 2, z0 + CAB_H / 2), (d, PANEL_T, CAB_H), "white"),
        ("bottom", (xf + (d - BACK_T) / 2, (yl + yr) / 2, z0 + PANEL_T / 2), (d - BACK_T, CAB_W - 2 * PANEL_T, PANEL_T),
         "white"),
        ("top", (xf + (d - BACK_T) / 2, (yl + yr) / 2, z0 + CAB_H - PANEL_T / 2),
         (d - BACK_T, CAB_W - 2 * PANEL_T, PANEL_T), "white"),
        ("back", (xf + d - BACK_T / 2, (yl + yr) / 2, z0 + CAB_H / 2), (BACK_T, CAB_W - 2 * PANEL_T, CAB_H - 0.0),
         "back"),
    ]
    return out


def blocks_pieces(spec):
    """The two 2x4 offcuts under the carcass sides (laid flat, along x)."""
    xf, yl = cabinet_origin(spec)
    yr = yl - CAB_W
    ln = CAB_D + 0.020
    return [
        ("blk_l", (xf + CAB_D / 2, yl - BLOCK_W / 2 + 0.005, TABLE_TOP + BLOCK_T / 2), (ln, BLOCK_W, BLOCK_T), "pine"),
        ("blk_r", (xf + CAB_D / 2, yr + BLOCK_W / 2 - 0.005, TABLE_TOP + BLOCK_T / 2), (ln, BLOCK_W, BLOCK_T), "pine"),
    ]


def plate_pieces_a():
    """Mounting plate boxes in the seated A frame (rail + cruciform wings; the plate sits between A's y = 0 and the
    panel at y = +RAIL_H)."""
    return [
        ("rail", (0.0, RAIL_H / 2, 0.0), (RAIL_L, RAIL_H, RAIL_W)),
        ("wing_v", (0.0, RAIL_H - WING_T / 2, 0.0), (WING_L, WING_T, WING_W)),
        ("nose", (HOOK_A[0] - 0.002, RAIL_H / 2 - 0.001, 0.0), (0.004, RAIL_H + 0.002, RAIL_W + 0.002)),
    ]


def arm_pieces_a():
    """Hinge arm boxes in A (collision + visual): channel over the rail, and the neck forward to the cup."""
    ax0, ax1 = ARM_X
    return [
        ("channel", ((ax0 + ax1) / 2, (ARM_Y[0] + ARM_Y[1]) / 2, 0.0), (ax1 - ax0, ARM_Y[1] - ARM_Y[0], 2 * ARM_Z)),
    ]


def door_pieces():
    """Door slab in D (collision + visual)."""
    return [("slab", (DOOR_W / 2, DOOR_T / 2, DOOR_H / 2), (DOOR_W, DOOR_T, DOOR_H))]


def groove_pieces():
    """Groove block boxes in its frame (origin on the bench under the groove centre, x along the groove)."""
    wall = (BLOCK_W - GROOVE_W) / 2
    return [
        ("base", (0.0, 0.0, (BLOCK_T - GROOVE_D) / 2), (GROOVE_BLOCK_L, BLOCK_W, BLOCK_T - GROOVE_D)),
        ("wall_a", (0.0, GROOVE_W / 2 + wall / 2, BLOCK_T / 2), (GROOVE_BLOCK_L, wall, BLOCK_T)),
        ("wall_b", (0.0, -GROOVE_W / 2 - wall / 2, BLOCK_T / 2), (GROOVE_BLOCK_L, wall, BLOCK_T)),
    ]


def door_alpha(theta_j):
    """Door open angle (deg) from the knuckle joint angle (rad): theta_j = 0 at 90 deg, -90 deg closed."""
    return ALPHA_OPEN + math.degrees(theta_j)
