"""Geometry, mechanism constants, pressure plant and start states of the regulator-set task (pure NumPy; no simulator).

Scene (station frame: x forward from the line between the arm bases, y to the robot's left of their midpoint, z up;
table top z = TOP): a shop-air drop on the bench in front of the robot. A 1/4 in non-relieving general-purpose
regulator (modelled on the SMC AR20 non-relieving option, push-to-lock knob on top) hangs on a steel bracket from a
plywood drop board; the supply comes in on the robot's left, the outlet goes to the robot's right through a tee
carrying a 2.5 in (63 mm) 0-160 psi centre-back-mount gauge (face up), then a 1/4 in lever ball valve (the bleed)
venting through a muffler.

Regulator frame R: origin at the regulator body centre, +x away from the robot (toward the board), +y to the robot's
left (inlet side), +z up (knob axis). Station rotation of R = rz(yaw).

Mechanism (runtime in ``isaac/regkit.py``; all motion is PhysX, driven only by the robot's contacts):
* knob: a prismatic dof along R z (0 = pushed in / locked, KNOB_OUT = pulled out / free), held at either end by a
  detent (stiff drive capped at F_DETENT), then a revolute dof about R z. Locked: the lock ring's teeth hold the knob
  (stiff drive capped at T_LOCK). Free: adjusting-screw friction (elasto-plastic anchor capped at knob_torque(P_set)).
  Clockwise (seen from above) raises the spring setpoint by K_PSI psi per turn (seeded).
* bleed ball valve: revolute about its vertical stem, 0 = closed (lever across the pipe, toward the robot), 90 deg =
  open (lever along the pipe); packing friction T_VALVE; hard stops at 0 and 90 deg.
* pressure: non-relieving regulator (no relief path: turning the knob down never lowers the outlet pressure) feeding a
  downstream volume that the bleed vents (see ``step_pressure``); the gauge needle follows with a snubber lag.
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475  # station table top (yam_arena station TABLE_TOP)
G = 9.81
OFFICIAL_SEED = 1415158438  # inspect-robots trial seed of eval seed 0 (the default start)


def rz(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# ------------------------------------------------------------------------------------------------ target and tolerances
SETPOINT_PSI = 90.0  # the instruction's pressure (standard inlet pressure of pneumatic hand tools)
TOL_PSI = 2.0  # gauge and regulator setting within +-2 psi (one minor division of a 0-160 psi gauge is 2 psi)
HOLD_S = 2.0  # end conditions held continuously this long (ends the trial)
CLEAR_S = 1.5  # no robot contact with the air drop parts this long
VALVE_CLOSED_DEG = 3.0  # bleed counted closed within this of its closed stop
GAUGE_STILL_PSI_S = 1.0  # needle slower than this

# ------------------------------------------------------------------------------------------------ pressure plant
P_SUPPLY = 150.0  # psi, compressor tank / shop main (regulated outlet cannot exceed it)
P_MAX_SET = 145.0  # spring range end (0-145 psi adjusting spring)
PB_PSI = 4.0  # regulator proportional band: valve fully open when the outlet is this far below the setpoint
A_IN = 3.0  # 1/s: fill rate coefficient of the regulator valve into the downstream volume (fully open)
A_OUT = 0.6  # 1/s: vent rate coefficient of the fully open bleed (downstream volume / bleed orifice)
TAU_GAUGE = 0.30  # s: gauge snubber lag (needle follows the line pressure first-order)


def step_pressure(p, p_set, bleed_frac, dt):
    """One explicit step of the downstream line pressure (psi gauge). Non-relieving: the regulator valve only admits
    supply air (when the outlet is below the spring setpoint, proportionally within PB_PSI); the only way out is the
    bleed. Sub-stepped for stability."""
    n = max(1, int(math.ceil(dt / 5e-4)))
    h = dt / n
    for _ in range(n):
        u = min(1.0, max(0.0, (p_set - p) / PB_PSI))
        dp = A_IN * u * max(0.0, P_SUPPLY - p) - A_OUT * bleed_frac * p
        p = max(0.0, p + h * dp)
    return p


def bleed_frac(valve_deg):
    """Flow fraction of the 1/4 in ball valve: 0 below 8 deg (the ball's port still sealed by the seat), then rising
    with the port overlap to 1 at 90 deg."""
    x = (valve_deg - 8.0) / (90.0 - 8.0)
    x = min(1.0, max(0.0, x))
    return x * x * (3.0 - 2.0 * x)


# ------------------------------------------------------------------------------------------------ knob
KNOB_R = 0.018  # 36 mm knurled knob
KNOB_H = 0.024
KNOB_Z0 = 0.057  # knob bottom above the body centre when pushed in (locked)
KNOB_OUT = 0.004  # pull-out travel to unlock (m)
LOCK_MID = 0.002  # past this the detent snaps to the other end
F_DETENT = 18.0  # N: pull / push needed to move the push-to-lock knob (estimate)
T_LOCK = 4.0  # N m: lock-ring teeth hold (knob cannot be turned while pushed in)
T_KNOB0 = 0.12  # N m: adjusting-screw friction at zero spring load
T_KNOB_PER_PSI = 0.0022  # N m per psi of setpoint (spring force x screw friction), 0.32 N m at 90 psi
COLLAR_TOP = 0.0610  # bonnet collar top: hides the knob's orange lock band (0.057-0.0605) while pushed in


def knob_torque(p_set):
    return T_KNOB0 + T_KNOB_PER_PSI * max(0.0, p_set)


# ------------------------------------------------------------------------------------------------ valve, gauge, layout
BODY = ((-0.022, 0.022), (-0.022, 0.022), (-0.030, 0.025))
BONNET_R, BONNET_Z = 0.020, (0.025, 0.055)
H_BODY = 0.050  # body centre above the bench (knob top 0.876 m: inside the YAM top-down reach)
GAUGE_C = (0.0, -0.080, 0.0)  # tee centre (R)
GAUGE_R = 0.0315  # 2.5 in case
GAUGE_Z = (0.028, 0.054)  # case bottom / face (R z)
NEEDLE_L = 0.026
VALVE_C = (0.0, -0.140, 0.0)  # ball valve stem axis (R)
STEM_TOP = 0.026  # lever plane (bottom of the lever) above the pipe axis
LEVER_L = 0.080  # stem to lever tip
GRIP_SPAN = (0.030, 0.078)  # vinyl sleeve along the lever (from the stem)
GRIP_W, GRIP_T = 0.014, 0.012  # sleeve width (across the lever) and thickness
T_VALVE = 0.45  # N m ball-valve operating torque (PTFE seats, 1/4 in, estimate)
BOARD_X = (0.100, 0.118)
BOARD_Y = (-0.240, 0.160)
BOARD_ZTOP = 0.130
PIPE_R = 0.0068  # 1/4 NPT nipple OD 13.7 mm


def gauge_angle(psi):
    """Needle angle about R z (rad, CCW from R +x): 0 psi at 135 deg (lower left seen from the robot), 160 psi at
    -135 deg (270 deg sweep, clockwise seen from above)."""
    return math.radians(135.0 - 270.0 * min(170.0, max(-5.0, psi)) / 160.0)


def psi_of_angle(a):
    return (135.0 - math.degrees(a)) * 160.0 / 270.0


# ------------------------------------------------------------------------------------------------ placement / starts
NOMINAL = (0.290, 0.060)  # regulator body centre station x, y
POS_D = 0.015  # m: seeded offset in x and y
YAW = (-6.0, 6.0)  # deg
P0_RANGE = (104.0, 122.0)  # psi: the regulator was left set high (and the line is at that pressure)
K_RANGE = (12.0, 18.0)  # psi per knob turn (spring rate / screw pitch / diaphragm area spread)
GUARD_TURNS = 0.30  # open-loop guard: required net knob turn at least this far from the default start's
FRICTION = dict(body=0.6, knob=0.9, lever=0.9, pipe=0.5, board=0.6)

NAMED_SETUPS = (
    dict(p0=112.0, k=15.0, dx=0.0, dy=0.0, yaw=0.0, knob0=0.0),
    dict(p0=118.0, k=13.0, dx=0.008, dy=-0.006, yaw=3.0, knob0=40.0),
    dict(p0=106.0, k=17.0, dx=-0.007, dy=0.009, yaw=-4.0, knob0=110.0),
)


def need_turns(spec):
    """Net knob turns (clockwise +) from the start that put the spring setpoint on SETPOINT_PSI."""
    return (SETPOINT_PSI - spec["p0"]) / spec["k"]


def _seeded(n):
    rng = np.random.default_rng(int(n))
    p0 = round(float(rng.uniform(*P0_RANGE)), 2)
    k = round(float(rng.uniform(*K_RANGE)), 3)
    dx, dy = (float(v) for v in rng.uniform(-POS_D, POS_D, 2))
    yaw = float(rng.uniform(*YAW))
    knob0 = float(rng.uniform(0.0, 360.0))
    return dict(p0=p0, k=k, dx=dx, dy=dy, yaw=yaw, knob0=knob0, name=f"seed {int(n)}")


def setup_spec(setup):
    """Start state {'p0' (psi: spring setpoint = line pressure at the start), 'k' (psi per turn), 'dx', 'dy', 'yaw'
    (stand pose), 'knob0' (deg, knob's knurl phase, cosmetic), 'name'}. ``setup`` is a named-setup index or
    ('seed', n).

    Open-loop guard: a seeded start whose required net knob turn is within GUARD_TURNS of the default (official-seed)
    start's has its (p0, k) redrawn from a second seeded stream, so the default trace's knob turn leaves every other
    start at least GUARD_TURNS x K_RANGE[0] = 3.6 psi off."""
    if isinstance(setup, (tuple, list)) and setup[0] == "seed":
        n = int(setup[1])
        sp = _seeded(n)
        if n == OFFICIAL_SEED:
            return sp
        ref = need_turns(_seeded(OFFICIAL_SEED))
        if abs(need_turns(sp) - ref) < GUARD_TURNS:
            rng = np.random.default_rng([n, 7])
            for _ in range(1000):
                p0 = round(float(rng.uniform(*P0_RANGE)), 2)
                k = round(float(rng.uniform(*K_RANGE)), 3)
                if abs((SETPOINT_PSI - p0) / k - ref) >= GUARD_TURNS:
                    sp["p0"], sp["k"] = p0, k
                    break
        return sp
    k = int(setup) % len(NAMED_SETUPS)
    return dict(NAMED_SETUPS[k], name=f"setup {k}")


def reg_pose(spec):
    """Station position of the regulator body centre and the rotation of frame R."""
    p = np.array([NOMINAL[0] + spec["dx"], NOMINAL[1] + spec["dy"], TOP + H_BODY])
    return p, rz(spec["yaw"])


def to_station(spec, pr, pose=None):
    p, r = reg_pose(spec) if pose is None else pose
    return np.asarray(p, float) + np.asarray(r, float) @ np.asarray(pr, float)


def lever_dir(valve_deg):
    """Unit vector (R) from the stem along the lever: closed = toward the robot (-x), open = along the pipe (-y)."""
    a = math.radians(180.0 + valve_deg)  # 0 deg -> -x; 90 deg -> -y (clockwise seen from above... see regkit)
    return np.array([math.cos(a), math.sin(a), 0.0])


# ------------------------------------------------------------------------------------------------ shapes
def box(center, size):
    hx, hy, hz = (s / 2 for s in size)
    c = np.asarray(center, float)
    return np.array([[sx * hx, sy * hy, sz * hz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) + c


def box_xyz(xr, yr, zr):
    return box(((xr[0] + xr[1]) / 2, (yr[0] + yr[1]) / 2, (zr[0] + zr[1]) / 2),
               (xr[1] - xr[0], yr[1] - yr[0], zr[1] - zr[0]))


def cyl_z(center, r, h, n=24):
    c = np.asarray(center, float)
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    return np.array([[r * math.cos(t), r * math.sin(t), z] for z in (-h / 2, h / 2) for t in a]) + c


def cyl_axis(p0, p1, r, n=16):
    """Convex cylinder between points p0 and p1."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    ax = p1 - p0
    ax /= np.linalg.norm(ax)
    t = np.array([1.0, 0.0, 0.0]) if abs(ax[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(ax, t)
    u /= np.linalg.norm(u)
    v = np.cross(ax, u)
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    ring = np.array([r * (math.cos(q) * u + math.sin(q) * v) for q in a])
    return np.vstack([ring + p0, ring + p1])


def ring_segments(r_in, r_out, z0, z1, n=32, a0=0.0, a1=2 * math.pi):
    out = []
    for i in range(n):
        b0 = a0 + (a1 - a0) * i / n
        b1 = a0 + (a1 - a0) * (i + 1) / n
        pts = []
        for a in (b0, b1):
            c, s = math.cos(a), math.sin(a)
            pts += [[r_out * c, r_out * s, z0], [r_out * c, r_out * s, z1], [r_in * c, r_in * s, z1],
                    [r_in * c, r_in * s, z0]]
        out.append(np.array(pts))
    return out
