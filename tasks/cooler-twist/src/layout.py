"""Geometry, mechanism constants and start states of the cooler-twist task (pure NumPy; no simulator).

Scene (station frame: x forward from the line between the arm bases, y to the robot's left of their midpoint, z up;
table top z = TOP): an open-air PC test bench (steel tray on rubber feet) carrying a micro-ATX AM4 motherboard. An AMD
Ryzen processor sits in the AM4 socket (ZIF lever closed) and an AMD Wraith Prism cooler sits on it, held down by its
cam-lever retention clip on the two AM4 bracket lugs; its thermal paste has dried. A grey anti-static mat lies on the
bench to the right of the test bench.

Bench frame Bn: origin on the table top under the socket's centre, +x along the socket's lug axis pointing away from
the robot at bench yaw 0 (the clip's lever end is on the -x side, toward the robot), +z up.

Articulation (``isaac/coolerkit.py``): the test bench is the floating root. The processor hangs off it through a chain
of six joints (``px``, ``py``, ``pz`` slides, ``prx``, ``pry``, ``prz`` rotations; zero = seated in the socket), the
cooler hangs off the processor through six more (``bx`` .. ``brz``; zero = seated on the processor's lid), and the cam
lever hangs off the cooler (``lev``, revolute about the bench +y axis; 0 = clip locked).

Real dimensions are estimates from product photos and published outline sizes (no drawing was available on this
host; see README fidelity): AM4 PGA package 40 x 40 mm, Wraith Prism about 106 x 108 x 93 mm, 0.55 kg.
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


# ------------------------------------------------------------------------------------------------ test bench (root)
TRAY = ((-0.165, 0.095), (-0.120, 0.115))  # steel carrier around the specified board
TRAY_Z = (0.008, 0.012)  # tray plate (on 8 mm rubber feet)
# ASRock A320M-HDV R4.0 product outline. Socket centre is estimated from the
# board diagram in section 1.3 of ASRock's manual. +x is page-up, +y page-left (viewed from above).
BOARD = ((-0.140, 0.090), (-0.092, 0.109))
BOARD_Z = (0.0184, 0.0200)  # on 6.4 mm standoffs, 1.6 mm PCB
BOARD_TOP = BOARD_Z[1]
SOCKET = ((-0.0225, 0.0225), (-0.0250, 0.0250), BOARD_TOP, BOARD_TOP + 0.0055)  # AM4 socket body
BRACKET_X = 0.0500  # AM4 retention bracket centres at x = +-50 mm (lug to lug about 108 mm)
BRACKET = (0.010, 0.060, 0.012)  # each bracket: x depth, y length, height above the board
LUG = (0.004, 0.012, 0.005)  # lug on the bracket's outer face, centre 7 mm above the board
BENCH_MASS = 4.5  # open-air test bench + board (estimate)
BENCH_COM = (-0.06, 0.0, 0.012)

# ------------------------------------------------------------------------------------------------ processor
CPU_SUB = (0.040, 0.040, 0.0012)  # PGA substrate
CPU_IHS = (0.038, 0.038, 0.0035)  # integrated heat spreader (lid)
H_IHS = SOCKET[3] + CPU_SUB[2] + CPU_IHS[2]  # lid top above the table: 0.0302
CPU_MASS = 0.046

# ------------------------------------------------------------------------------------------------ cooler (cooler frame: origin = base bottom centre)
BASE = (0.038, 0.038, 0.006)  # copper base block (the paste contact face, 38 x 38 mm)
FIN_R = 0.043  # radial fin stack radius
FIN_Z = (0.006, 0.064)
SHROUD_R = 0.054  # fan shroud 108 mm across (the published outline), overhangs the fins by 11 mm
SHROUD_Z = (0.064, 0.093)
COOLER_MASS = 0.55
COOLER_COM = (0.0, 0.0, 0.045)
COOLER_INERTIA = (6.5e-4, 6.5e-4, 5.6e-4)
# cam lever on the clip's near (-x) end: pivot in the cooler frame, axis +y; lever points -x when locked
LEV_PIVOT = (-0.066, 0.0, 0.026)
LEV_LEN = 0.040
LEV_BAR = (0.008, 0.004)  # width (y), thickness (z)
LEV_TAB = (0.016, 0.018, 0.004)  # flat finger paddle at the tip: length along the lever, width (y), thickness
LEV_MASS = 0.02

# ------------------------------------------------------------------------------------------------ mechanism (estimates, see DESIGN.md)
CLIP_LOCK = dict(f=400.0, t=20.0)  # clip locked: cooler held to the bench (hooks on lugs, clip through the fin slot)
LEV_T_PEAK = 0.60  # N m: cam lever resisting torque (clip preload ~300 N x ~2 mm cam throw)
LEV_K = LEV_T_PEAK / math.radians(20.0)  # N m / rad: rises to the peak over the first 20 deg
LEV_RELEASE_DEG = 55.0  # past this the cam is over its high point: the clip goes slack
LEV_RECLAMP_DEG = 30.0  # pushed back below this the cam re-tensions the clip
LEV_OPEN_DEG = 100.0  # open stop
LEV_OPEN_T = 0.03  # N m: the slack clip's spring nudges the lever onto its open stop
F_ZIF = 25.0  # N: pull-out force of the processor from the closed ZIF socket (1331 pins; estimate)
YANK_MM = 1.0  # processor lifted this far out of the socket = pins out (bent): irreversible
PASTE_A = BASE[0] * BASE[1]  # m^2 contact
PASTE_RA = 0.3826 * BASE[0] ** 3  # integral of r dA over the square face (m^3)
PASTE_REFF = PASTE_RA / PASTE_A  # 14.5 mm: lateral slip -> equivalent twist
T_BREAK = (0.60, 1.40)  # N m: seeded twist torque at which the dried paste film starts to shear (30-67 kPa)
F_TENS = 110.0  # N: tensile hold of the dried paste film (> F_ZIF: a straight pull takes the processor)
T_TILT_BOND = 4.0  # N m: peel moment the bonded film carries
SLIP_BREAK_DEG = 6.0  # cumulative shear slip (twist, or lateral / r_eff) that breaks the film
F_TACK = 2.0  # N: residual tack of a sheared film until the faces part
PART_MM = 1.5  # faces parted this far: free
T_SLIDE = 0.05  # N m: sheared paste: running twist resistance while the faces touch
F_SLIDE = 3.0  # N: sheared paste: running lateral resistance while the faces touch
# Estimated motion of each slack clip end as it is drawn outward under its lug before springing up.
HOOKS = (("near_hook", -1), ("far_hook", 1))
HOOK_X = BRACKET_X + BRACKET[0] / 2 + LUG[0] + 0.001
HOOK_TRAVEL = 0.015
HOOK_RELEASE = 0.009
HOOK_EAR = (0.007, 0.016, 0.017)
HOOK_SIZE = (0.014, 0.015, 0.003)

FRICTION = dict(bench=0.7, cpu=0.4, cooler=0.8, lever=0.6, mat=0.6)

# ------------------------------------------------------------------------------------------------ anti-static mat (static)
MAT = ((0.27, 0.47), (-0.25, -0.115), 0.003)  # station x range, y range, thickness

# ------------------------------------------------------------------------------------------------ start states
NOMINAL = (0.340, 0.040)  # socket centre x, y (station)
BENCH_D = (0.015, 0.035)  # seeded bench offset |d| from the nominal (any direction)
BENCH_YAW = (-102.0, -78.0)  # nominal -90: the clip lever (bench -x end) faces the left arm
NAMED_SETUPS = (
    dict(bx=0.360, by=0.050, byaw=-85.0, tb=0.80),
    dict(bx=0.325, by=0.025, byaw=-98.0, tb=1.00),
    dict(bx=0.350, by=0.015, byaw=-80.0, tb=1.20),
)

OFFICIAL_SEED = 1415158438  # inspect-robots trial seed of eval seed 0 (the default start)
# Open-loop guard on what a replayed trajectory depends on: the clip lever's finger paddle (nothing comes off unless
# the lever is opened). A seeded start is redrawn unless its paddle lies at least GUARD_ACROSS to the side of the
# default start's paddle (across the default lever's direction; the paddle is 18 mm wide) or at least -GUARD_ALONG
# further in along it (the fingertips reach 9 mm past the paddle's centre). The fan-shroud grasp needs no guard of its
# own: with the clip locked the cooler cannot come off.
GUARD_ACROSS = 0.032
GUARD_ALONG = -0.022


def _seeded(n, stream=None):
    rng = np.random.default_rng(int(n) if stream is None else [int(n), stream])
    d = float(rng.uniform(*BENCH_D))
    a = float(rng.uniform(0.0, 2 * math.pi))
    bx, by = NOMINAL[0] + d * math.cos(a), NOMINAL[1] + d * math.sin(a)
    byaw = float(rng.uniform(*BENCH_YAW))
    tb = float(rng.uniform(*T_BREAK))
    return dict(bx=bx, by=by, byaw=byaw, tb=tb, name=f"seed {int(n)}")


def paddle_xy(spec):
    """Station (x, y) of the lever paddle's centre with the clip locked."""
    p, r = bench_pose(spec)
    return (p + r @ np.array([LEV_PIVOT[0] - LEV_LEN + LEV_TAB[0] / 2, 0.0, 0.0]))[:2]


def paddle_shift(spec, ref):
    """(across, along) shift (m) of ``spec``'s paddle from ``ref``'s, in ``ref``'s lever frame (along = outward)."""
    _, r = bench_pose(ref)
    u = (r @ np.array([-1.0, 0.0, 0.0]))[:2]
    v = np.array([-u[1], u[0]])
    d = paddle_xy(spec) - paddle_xy(ref)
    return abs(float(d @ v)), float(d @ u)


def guarded(spec, ref):
    across, along = paddle_shift(spec, ref)
    return across < GUARD_ACROSS and along > GUARD_ALONG


def setup_spec(setup):
    """Start state {'bx', 'by', 'byaw', 'tb', 'name'}. ``setup`` is a named-setup index or ('seed', n).

    Seeded starts: the bench 15-35 mm from its nominal spot in any direction, turned -90 +-12 deg, and the paste's break
    torque 0.60-1.40 N m, drawn from seed n. Open-loop guard: when the start's lever paddle would lie where the default
    (official-seed) start's trajectory grasps it (``guarded``), the bench position and turn are redrawn from a second
    seeded stream (same ranges) until it does not, so no start is close to the default one where it matters."""
    if isinstance(setup, (tuple, list)) and setup[0] == "seed":
        n = int(setup[1])
        sp = _seeded(n)
        if n == OFFICIAL_SEED:
            return sp
        ref = _seeded(OFFICIAL_SEED)
        if guarded(sp, ref):
            rng = np.random.default_rng([n, 7])
            for _ in range(2000):
                d = float(rng.uniform(*BENCH_D))
                a = float(rng.uniform(0.0, 2 * math.pi))
                cand = dict(sp, bx=NOMINAL[0] + d * math.cos(a), by=NOMINAL[1] + d * math.sin(a),
                            byaw=float(rng.uniform(*BENCH_YAW)))
                if not guarded(cand, ref):
                    sp = cand
                    break
        return sp
    k = int(setup) % len(NAMED_SETUPS)
    return dict(NAMED_SETUPS[k], name=f"setup {k}")


def bench_pose(spec):
    """Bench root pose (station): origin on the table under the socket centre."""
    return np.array([spec["bx"], spec["by"], TOP]), rz(spec["byaw"])


def ihs_station(spec):
    p, r = bench_pose(spec)
    return p + r @ np.array([0.0, 0.0, H_IHS])


def lever_pivot_station(spec):
    p, r = bench_pose(spec)
    return p + r @ (np.array([0.0, 0.0, H_IHS]) + np.array(LEV_PIVOT))


def on_mat(p, tol=0.0):
    """Station point ``p`` (x, y) over the mat."""
    return MAT[0][0] - tol < p[0] < MAT[0][1] + tol and MAT[1][0] - tol < p[1] < MAT[1][1] + tol


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


def ring_segments(r_in, r_out, z0, z1, n=32, phase=0.0):
    """Convex pieces of a tube (n sectors)."""
    out = []
    for i in range(n):
        a0 = phase + 2 * math.pi * i / n
        a1 = phase + 2 * math.pi * (i + 1) / n
        pts = []
        for a in (a0, a1):
            c, s = math.cos(a), math.sin(a)
            pts += [[r_out * c, r_out * s, z0], [r_out * c, r_out * s, z1], [r_in * c, r_in * s, z0],
                    [r_in * c, r_in * s, z1]]
        out.append(np.array(pts))
    return out
