"""Scene layout and part geometry of the bearing-press task (station frame: x forward from the arm bases, y to the
robot's left, z up; metres). Engine-independent (numpy only).

Parts (full size):
* a 1-ton bench arbor press (representative dimensions of the common cast-iron 1-ton single-lever class; no model
  number is claimed): base with a flat table, column, head, a 25.4 mm (1 in) square ram on a rack, pinion behind it,
  handle on the right of the head swinging in the vertical front-back plane (pull it toward you and down to lower
  the ram);
* an aluminium quasi-direct-drive actuator output housing (62 mm OD, 22 mm tall) with a 42 mm bearing bore 7 mm deep
  to a shoulder (37 mm through-bore below it) and a 0.8 x 45 deg lead-in chamfer; it lies loose on the press table;
* one SKF 61806-2RS1 deep-groove ball bearing, 30 x 42 x 7 mm, two rubber seals;
* two driver discs from a metric bushing-driver set: a 41 mm outer-ring driver (its face relieved inside 38 mm, so it
  bears on the outer ring only) and a 32 mm disc that bears on the inner ring (the wrong one for this job);
* a foam tool tray on the table holding the bearing and the two discs.
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475  # station table top (yam_arena TABLE_TOP["station"])


def rz(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def ry(rad):
    c, s = math.cos(rad), math.sin(rad)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


# ---------------------------------------------------------------- arbor press (fixed on the table)
RAM_XY = np.array([0.400, 0.030])  # ram axis (left of centre: the left hand works the parts, the right the handle)
PLATE_Z = TOP + 0.065  # press table top (on the base casting)
BASE_BOX = ((RAM_XY[0] - 0.068, RAM_XY[0] + 0.175), (RAM_XY[1] - 0.085, RAM_XY[1] + 0.190), (TOP, PLATE_Z))  # base casting x, y, z ranges
COLUMN_BOX = ((RAM_XY[0] + 0.105, RAM_XY[0] + 0.165), (RAM_XY[1] - 0.035, RAM_XY[1] + 0.035), (PLATE_Z, PLATE_Z + 0.250))
HEAD_BOX = ((RAM_XY[0] - 0.020, RAM_XY[0] + 0.165), (RAM_XY[1] - 0.035, RAM_XY[1] + 0.035), (PLATE_Z + 0.160, PLATE_Z + 0.250))
RAM_W = 0.0254  # 1 in square ram
RAM_LEN = 0.30
RAM_MASS = RAM_W * RAM_W * RAM_LEN * 7850.0  # 1.52 kg steel
# ram face height at the start: 4 mm above the outer-ring driver's top on a bearing started in the bore
HOUSING_H = 0.022
BEARING_B = 0.007
CHAMFER = 0.0008  # housing bore lead-in, 45 deg
START_SINK = CHAMFER  # a bearing set square in the bore mouth rests 0.8 mm deep (its chamfer in the lead-in)
DISC_T = 0.012
RAM_GAP0 = 0.004
RAM_FACE0 = PLATE_Z + HOUSING_H - START_SINK + BEARING_B + DISC_T + RAM_GAP0
RAM_TRAVEL = (0.0, 0.040)  # ram joint: 0 = start, + = down (hard limits)
# pinion and handle (joint angle theta: 0 at the start, + = handle swung toward the robot and down = ram down)
PINION_R = 0.0127  # pitch radius: ram moves 12.7 mm per handle radian
PIVOT = np.array([RAM_XY[0] + 0.030, RAM_XY[1] - 0.075, PLATE_Z + 0.200])  # handle hub centre (pinion axis along y)
HANDLE_ALPHA0_DEG = 20.0  # handle direction at the start: toward the robot (-x), 20 deg above horizontal
HANDLE_LEN = 0.280
HANDLE_R = 0.008  # 16 mm bar
KNOB_R = 0.016
GRIP_R = 0.245  # where the reference demo grips the bar (from the pivot)
HANDLE_THETA_LIM = (-0.30, 1.40)
HANDLE_FRICTION_NM = 1.2  # pinion friction (holds the handle and ram where they are left; about 5 N at the grip)
HANDLE_MASS = 0.55


def handle_dir(theta):
    """Unit vector from the pivot along the handle bar (x-z plane) at joint angle theta."""
    a = math.radians(HANDLE_ALPHA0_DEG) - theta
    return np.array([-math.cos(a), 0.0, math.sin(a)])


def handle_point(theta, r=GRIP_R):
    return PIVOT + r * handle_dir(theta)


def ram_face_z(ram_q):
    return RAM_FACE0 - ram_q


def ram_of_theta(theta):
    return PINION_R * theta


# ---------------------------------------------------------------- housing
HOUSING_R = 0.031
BORE_R = 0.02112  # 42.24 mm in the simulation (0.12 mm radial clearance; real fit: transition, see README)
THROUGH_R = 0.0185  # 37 mm through-bore below the shoulder
HOUSING_MASS = 0.11  # 6061 aluminium
SEAT_DEPTH = BEARING_B  # shoulder 7 mm below the housing top: a seated bearing is flush with the top
# start: housing on the press table to the left of the ram (from the seed: 110..130 mm to the left, +-4 mm fore-aft)
HOUSING_START_DX = (-0.004, 0.004)
HOUSING_START_DY = (0.110, 0.130)
HOUSING_NAMED = ((0.0, 0.120), (0.003, 0.112), (-0.003, 0.128), (0.002, 0.110), (-0.004, 0.130))

# ---------------------------------------------------------------- bearing (SKF 61806-2RS1)
BRG_D = 0.030
BRG_OD = 0.042
OUTER_IN_R = 0.0195  # outer ring land (face annulus 19.5..21 mm radius)
INNER_OUT_R = 0.0167  # inner ring face annulus 15..16.7 mm radius
SEAL_RECESS = 0.0002
BRG_CHAMFER = 0.0003
BRG_MASS = 0.026  # catalogue mass (not verified, see README)

# ---------------------------------------------------------------- driver discs
DISC_OUT_R = 0.0205  # 41 mm outer-ring driver
DISC_OUT_RELIEF_R = 0.019  # face relieved inside 38 mm, 1.5 mm deep
DISC_RELIEF_D = 0.0015
SPIGOT_R = 0.0146  # pilot spigot under the driver: 29.2 mm, 0.4 mm radial clearance in the bearing's 30 mm bore
SPIGOT_L = 0.004  # 4 mm long below the rim face, 1 mm x 45 deg lead chamfer
SPIGOT_CH = 0.001
DISC_IN_R = 0.016  # 32 mm plain disc: bears on the inner ring (and seals) only
DISC_BIG_R = 0.023  # 46 mm plain disc: bears on both rings at once (faces flush), so half the load goes through the balls
WRONG_DISCS = ("disc_in", "disc_big")  # one of them is in the tray each trial (seeded), the other is put away
PARK = {"disc_in": np.array([-0.45, 0.10]), "disc_big": np.array([-0.45, -0.10])}  # put-away spot, under the table
PARK_Z = TOP - 0.40
STEEL = 7850.0


def disc_mass(r):
    return math.pi * r * r * DISC_T * STEEL


# ---------------------------------------------------------------- tool tray (foam), on the table left of the press
TRAY_C = np.array([0.245, 0.175])
TRAY_SIZE = (0.16, 0.10, 0.045)  # foam-lined tool case on the table
TRAY_TOP = TOP + TRAY_SIZE[2]
TRAY_SLOTS = {"a": np.array([-0.048, 0.0]), "b": np.array([0.0, 0.0]), "c": np.array([0.050, 0.0])}
# the bearing lies in slot b; the two discs in slots a and c, which way round is drawn per seed


# ---------------------------------------------------------------- fit model (see README fidelity)
FIT_START_N = 20.0  # resistance as the outer ring enters the cylindrical bore
FIT_FULL_N = 300.0  # at full engagement (7 mm): about 5 um interference, K7 housing (estimate)
ENGAGE_LEN = SEAT_DEPTH - START_SINK  # 6.2 mm of travel from the start to the shoulder


def fit_force(d):
    """Axial force (N) needed to push the outer ring further when it is d (m) past the start depth."""
    d = min(max(d, 0.0), ENGAGE_LEN)
    return FIT_START_N + (FIT_FULL_N - FIT_START_N) * d / ENGAGE_LEN


# ---------------------------------------------------------------- start states
def _u(rng, lo_hi):
    return float(rng.uniform(lo_hi[0], lo_hi[1]))


def setup_spec(setup):
    """Start state: {'hx','hy' (housing centre offset from the ram axis, m), 'swap' (bool: 41 mm disc in slot c),
    'wrong' (the other disc in the tray: 'disc_in' 32 mm or 'disc_big' 46 mm), 'name'}. ``setup`` is a named setup
    index or ('seed', n)."""
    if isinstance(setup, tuple) and setup[0] == "seed":
        rng = np.random.default_rng(int(setup[1]))
        hx, hy = _u(rng, HOUSING_START_DX), _u(rng, HOUSING_START_DY)
        swap = bool(rng.integers(0, 2))
        wrong = WRONG_DISCS[int(rng.integers(0, 2))]
        return dict(hx=hx, hy=hy, swap=swap, wrong=wrong, name=f"seed {int(setup[1])}")
    k = int(setup) % len(HOUSING_NAMED)
    hx, hy = HOUSING_NAMED[k]
    return dict(hx=hx, hy=hy, swap=bool(k % 2), wrong=WRONG_DISCS[(k // 2) % 2], name=f"setup {k}")


def housing_start(setup):
    s = setup_spec(setup)
    return np.array([RAM_XY[0] + s["hx"], RAM_XY[1] + s["hy"], PLATE_Z + 0.0001])


def tray_xy(slot):
    return TRAY_C + TRAY_SLOTS[slot]


def part_starts(setup):
    """Start poses (pos of the part frame origin = bottom centre, rot) of the free parts."""
    s = setup_spec(setup)
    oslot, islot = ("c", "a") if s["swap"] else ("a", "c")
    out = {
        "housing": (housing_start(setup), np.eye(3)),
        "bearing": (np.r_[tray_xy("b"), TRAY_TOP + 0.0001], np.eye(3)),
        "disc_out": (np.r_[tray_xy(oslot), TRAY_TOP + SPIGOT_L + 0.0001], np.eye(3)),  # stands on its spigot
    }
    for n in WRONG_DISCS:
        if n == s["wrong"]:
            out[n] = (np.r_[tray_xy(islot), TRAY_TOP + 0.0001], np.eye(3))
        else:
            out[n] = (np.r_[PARK[n], PARK_Z + 0.0001], np.eye(3))
    return out


# ---------------------------------------------------------------- convex pieces
def annulus_cells(r0, r1, n, phase_deg=0.0):
    cells = []
    for i in range(n):
        a0 = math.radians(phase_deg + i * 360.0 / n)
        a1 = math.radians(phase_deg + (i + 1) * 360.0 / n)
        cells.append(
            np.array(
                [
                    [r0 * math.cos(a0), r0 * math.sin(a0)],
                    [r1 * math.cos(a0), r1 * math.sin(a0)],
                    [r1 * math.cos(a1), r1 * math.sin(a1)],
                    [r0 * math.cos(a1), r0 * math.sin(a1)],
                ]
            )
        )
    return cells


def prism(poly, z0, z1):
    poly = np.asarray(poly, float)
    k = len(poly)
    return np.r_[np.c_[poly, np.full(k, z0)], np.c_[poly, np.full(k, z1)]]


def frustum_cell(cell_lo, cell_hi, z0, z1):
    return np.r_[np.c_[cell_lo, np.full(len(cell_lo), z0)], np.c_[cell_hi, np.full(len(cell_hi), z1)]]


def annulus(r0, r1, z0, z1, n, phase_deg=0.0):
    return [prism(c, z0, z1) for c in annulus_cells(r0, r1, n, phase_deg)]


def cylinder(r, z0, z1, n=32, center=(0.0, 0.0)):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = lambda z: np.c_[center[0] + r * np.cos(a), center[1] + r * np.sin(a), np.full(n, z)]  # noqa: E731
    return np.r_[ring(z0), ring(z1)]


NB = 48  # sectors on the bore and the bearing's outer ring (chord sag 0.045 mm)


def housing_pieces():
    """Housing in its frame (origin at the bottom centre, z up)."""
    h = HOUSING_H
    out = annulus(THROUGH_R, HOUSING_R, 0.0, h - SEAT_DEPTH, 36)  # below the shoulder (shoulder = its top face)
    out += annulus(BORE_R, HOUSING_R, h - SEAT_DEPTH, h - CHAMFER, NB, phase_deg=180.0 / NB)  # bore wall
    lo = annulus_cells(BORE_R, HOUSING_R, NB, 180.0 / NB)
    hi = annulus_cells(BORE_R + CHAMFER, HOUSING_R, NB, 180.0 / NB)
    out += [frustum_cell(a, b, h - CHAMFER, h) for a, b in zip(lo, hi)]  # 45 deg lead-in
    return out


def bearing_pieces():
    """Bearing in its frame (origin at the bottom centre). Outer ring as full-height convex sectors with the 0.3 mm OD
    chamfers (no thin pieces: thin hulls let a loaded contact tunnel), recessed seals, inner ring (one rigid body)."""
    b, c = BEARING_B, BRG_CHAMFER
    R = BRG_OD / 2
    out = []
    lo = annulus_cells(OUTER_IN_R, R - c, NB)
    mid = annulus_cells(OUTER_IN_R, R, NB)
    for a, m in zip(lo, mid):
        out.append(np.r_[np.c_[a, np.zeros(4)], np.c_[m, np.full(4, c)], np.c_[m, np.full(4, b - c)], np.c_[a, np.full(4, b)]])
    out += annulus(INNER_OUT_R, OUTER_IN_R, SEAL_RECESS, b - SEAL_RECESS, 24)  # seals
    out += annulus(BRG_D / 2, INNER_OUT_R, 0.0, b, 24)  # inner ring
    return out


def disc_out_pieces():
    """41 mm outer-ring driver (origin at the rim face centre): full-height rim sectors (38..41 mm) that bear on the
    outer ring only, the centre of the disc 1.5 mm higher (the relief), and a 29.2 mm pilot spigot 4 mm long that
    enters the bearing's bore and keeps the driver centred on the ring (it never bears axially: the inner ring's face
    is level with the rim face)."""
    out = [cylinder(DISC_OUT_RELIEF_R + 0.0002, DISC_RELIEF_D, DISC_T, n=48)]
    out += annulus(DISC_OUT_RELIEF_R, DISC_OUT_R, 0.0, DISC_T, 36)
    a = np.linspace(0, 2 * np.pi, 32, endpoint=False)
    ring = lambda r, z: np.c_[r * np.cos(a), r * np.sin(a), np.full(32, z)]  # noqa: E731
    out.append(np.r_[ring(SPIGOT_R - SPIGOT_CH, -SPIGOT_L), ring(SPIGOT_R, -SPIGOT_L + SPIGOT_CH), ring(SPIGOT_R, DISC_RELIEF_D)])
    return out


def disc_in_pieces():
    return [cylinder(DISC_IN_R, 0.0, DISC_T, n=40)]


def disc_big_pieces():
    return [cylinder(DISC_BIG_R, 0.0, DISC_T, n=48)]


def plunger_pieces():
    """Fit-model support under the outer ring (housing frame, z = 0 at the joint): a thin annulus inside the bore."""
    return annulus(0.0170, BORE_R - 0.00025, -0.005, 0.0, 36)  # under the seals (0.2 mm recessed) and the outer ring


def tray_pocket_walls():
    """Low foam walls around each tray slot (tray frame, z = 0 at the tray top): (center, size) boxes."""
    return []
