"""Scene layout and geometry of the gpu-latch task (station frame: x forward from the arm bases, y to the robot's
left, z up; metres). Engine-independent (numpy only).

A side-open tower encloses the board on standoffs. An RTX A4000-class GPU (
241.3 x 111.15 mm full-height card, single-slot cooler) sits in the board's PCIe x16 slot; its bracket screw is
already out and its bracket flange rests on the bracket bar. The slot's retention latch at the slot's inner end hooks
the card's retention lip. An anti-static mat lies on the table to the robot's right of the benchtable.

Frames:
* slot frame S (the benchtable's frame): origin on the motherboard's top surface, on the card's mid-plane, at the
  bracket's inner face; u along the slot from the bracket toward the latch, n normal to the card toward its back
  (secondary) side = toward the robot, z up. (u, n, z) is right-handed. At yaw 0, u = +y and n = -x.
* card frame K: coincides with S when the card is fully seated.
* latch frame: at the latch pivot, S-aligned when the latch is closed; the latch turns about +n (positive = open: its
  top moves toward +u, its thumb pad goes down).
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475  # station table top (yam_arena TABLE_TOP["station"])
OFFICIAL_SEED = 1415158438  # the trial seed inspect-robots passes by default (eval seed 0)
MM = 1e-3


def rz(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def box_corners(x0, x1, y0, y1, z0, z1):
    return np.array([[x, y, z] for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)], float)


def ring(r, z, n=24, phase=0.0):
    a = phase + np.arange(n) * 2 * math.pi / n
    return np.column_stack([r * np.cos(a), r * np.sin(a), np.full(n, z)])


def ubox(u0, u1, n0, n1, z0, z1):
    """Box corners in (u, n, z) millimetres -> metres (S / K / latch frame coordinates)."""
    return box_corners(u0 * MM, u1 * MM, n0 * MM, n1 * MM, z0 * MM, z1 * MM)


# ---------------------------------------------------------------- benchtable and motherboard (S frame, mm)
BOARD_Z = TOP + 0.030  # board top above the outside of the chassis floor
BOARD = (-4.0, 240.0, -60.0, 245.0)  # ATX 244 x 305 mm: u range, n range (the CPU side is +n)
BOARD_T = 1.6
PLATE = (-30.0, 262.0, -85.0, 270.0, -11.6, -7.6)  # benchtable plate (4 mm aluminium) on four legs
BAR = (-14.0, -1.5, -80.0, 60.0, 0.0, 119.2)  # vertical bracket bar along the board's rear edge (top at 119.2)
# slot-cover guides on the bar either side of the card's bracket (0.3 mm clear), as a chassis' slot dividers locate a
# bracket whose screw is out: they keep the card upright and let it lift straight out
GUIDES = ((-1.5, 1.5, 3.5, 6.0, 95.0, 119.2), (-1.5, 1.5, -19.0, -16.5, 95.0, 119.2))

# Precision 3660 outer envelope laid on its side, mm. Board offsets and cutout
# are estimated. The removed side cover faces up. Rear opening opens to the rim.
CASE_RIM = 143.0
CASE_WALLS = {
    "floor": (-14, 406, -92, 277, -30, -28),
    "front": (404, 406, -92, 277, -28, CASE_RIM),
    "side_a": (-14, 404, -92, -90, -28, CASE_RIM),
    "side_b": (-14, 404, 275, 277, -28, CASE_RIM),
    "rear_left": (-14, -12, -90, -18, -28, CASE_RIM),
    "rear_right": (-14, -12, 5, 275, -28, CASE_RIM),
    "rear_sill": (-14, -12, -18, 5, -28, 2),
}

# PCIe x16 connector (PCI Express CEM: 89 mm housing, 1.0 mm contact pitch, 164 contacts in two rows)
SLOT_U = (45.0, 134.0)
SLOT_HALF_W = 3.75  # housing 7.5 mm wide
SLOT_TOP = 11.25
SLOT_FLOOR = 3.0
CHANNEL_HALF = 1.285  # card channel half-width (0.5 mm clear of the 1.57 mm card each side)
# The two contact rows (82 sprung contacts a side) grip the gold fingers over z 4.5-8 mm. Their grip is emulated as
# Coulomb friction on the card along the slot axis: static/kinetic limit 2 x 0.3 x 34 N = 20.4 N while the fingers are
# fully engaged, tapering to 0 once the card has risen 3-4.5 mm (contacts off the pads); stick phase as a stiff
# spring (pre-slip 2 mm at the limit) with damping.
CONTACT_RELEASE = (3.0, 4.5)  # mm of card rise: grip full up to the first value, gone at the second
CONTACT_FRICTION = 0.30  # gold on gold, lubricated (estimate)
CONTACT_NORMAL = 34.0  # N per row (estimate)
WITHDRAW_N = 2 * CONTACT_FRICTION * CONTACT_NORMAL  # 20.4 N
STICK_K = 10000.0  # N/m
STICK_C = 40.0  # N s/m


def contact_fraction(rise_m):
    """Fraction of the contact rows' grip still acting for a finger-bottom rise (m): full until the contact points
    reach the top of the gold pads (3 mm of travel), then falling to 0 over the next 1.5 mm."""
    r = rise_m / MM
    return float(min(1.0, max(0.0, (CONTACT_RELEASE[1] - r) / (CONTACT_RELEASE[1] - CONTACT_RELEASE[0]))))


# Retention latch (rocker type at the slot's inner end)
PIVOT = (136.8, 1.5)  # (u, z) of the pivot line (axis along n): under the hook, so a pull on the card only seats it
LATCH_LIMIT_DEG = (0.0, 25.0)
LATCH_TAU0 = 0.035  # N m preload of the latch's torsion spring (closed against its stop)
LATCH_K = 0.12  # N m / rad
LATCH_C = 0.0003  # N m s / rad
LATCH_MASS = 0.010
LATCH_INERTIA = 4.0e-6  # kg m^2 (raised over the real ~1e-7 for a stable explicit spring at 240 Hz; disclosed)
HOOK_CLEAR_DEG = 17.0  # the hook clears the card's lip beyond about this angle (geometry below)
LATCH_OPEN_DEG = 20.0  # demo cue: latch visibly pressed down
HOOK_PASS_RISE = 0.003  # lip top reaches the top of the closed hook

def latch_pieces():
    """Latch collision pieces in the latch frame (pivot at the origin), mm -> m: an L-shaped lever (foot under the
    card's lip through the pivot, post beside the slot's end), its hook over the lip, and the thumb pad."""
    pu, pz = PIVOT
    return {
        "foot": ubox(135.5 - pu, 143.0 - pu, -2.5, 2.5, 0.5 - pz, 3.5 - pz),
        "post": ubox(139.0 - pu, 143.0 - pu, -SLOT_HALF_W, SLOT_HALF_W, 0.5 - pz, 11.0 - pz),
        "hook": ubox(136.2 - pu, 139.0 - pu, -2.5, 2.5, 7.7 - pz, 10.5 - pz),
        "pad": ubox(143.0 - pu, 153.0 - pu, -10.0, 10.0, 9.5 - pz, 12.0 - pz),
    }


PAD_CENTER = (148.0, 6.5, 12.0)  # (u, n, z) mm: the thumb pad's exposed top beside the card's back (demo target)

# ---------------------------------------------------------------- the card (K frame, mm)
CARD_L = 241.3  # 9.5 in
CARD_TOP = 114.9  # top edge (finger bottom 3.75 + 111.15 full height)
FINGER = (45.6, 133.4, 3.75, 11.25)  # gold-finger tab: u range, z range
FINGER_Z0 = 3.75
LIP = (133.4, 138.0, 5.0, 7.5)  # retention lip ("hockey stick") under the latch hook
NOTCH_U = 135.7  # the notch's inner wall, 0.5 mm from the hook's tip: the card cannot slide off the hook
PCB_T = 1.57
CARD_MASS = 0.50  # kg (estimate for a single-slot blower card)
CARD_COM = (125.0, -5.0, 62.0)
CARD_INERTIA = (1.1e-3, 3.0e-3, 2.5e-3)  # about the K axes at the COM (u, n, z), slab estimate
SHROUD = (4.0, CARD_L, -15.3, -PCB_T / 2, 16.0, 113.0)
BACK = (4.0, CARD_L, PCB_T / 2, 2.3, 16.0, 113.0)
SHELL_FRICTION = 0.9  # gripper's rubber pads on the cooler shroud / back cover (estimate; "max" combine)
GRIP_Z = 101.0  # demo grip height on the card (mm, K frame): pads on the shroud and back faces below the top edge


def card_pieces():
    """Card collision pieces in the K frame (m)."""
    h = PCB_T / 2
    return {
        "finger": ubox(FINGER[0], FINGER[1], -h, h, FINGER[2], FINGER[3]),
        "lip": ubox(LIP[0], LIP[1], -h, h, LIP[2], LIP[3]),
        "notch": ubox(LIP[0], NOTCH_U, -h, h, LIP[3], FINGER[3]),  # inner wall of the notch the hook sits in
        "pcb_a": ubox(2.0, 139.0, -h, h, 11.25, CARD_TOP),
        "pcb_b": ubox(139.0, CARD_L, -h, h, 14.0, CARD_TOP),
        "shroud": ubox(*SHROUD),
        "back": ubox(*BACK),
        "bracket": ubox(-0.9, 0.0, -16.0, 3.0, 3.0, 120.1),
        "flange": ubox(-12.0, 0.0, -16.0, 3.0, 119.2, 120.1),
    }


def card_box():
    """Oriented bounding box of the card body (no bracket) in K: (min, max) corners, m."""
    return np.array([4.0, -15.3, 3.75]) * MM, np.array([CARD_L, 2.3, CARD_TOP]) * MM


# ---------------------------------------------------------------- anti-static mat (station frame)
MAT_CENTER = (0.400, -0.360)
MAT_HALF = (0.2285, 0.0900)  # 457 x 180 mm (cut ESD bench mat), long side along x
MAT_T = 0.002
MAT_TOP = TOP + MAT_T

# ---------------------------------------------------------------- start states
BENCH_XY = (0.200, -0.075)  # rear end of slot, in station xy
BENCH_YAW0 = 0.0  # case depth points forward along station x
JIT_X = 0.025
JIT_Y = 0.060
JIT_YAW = 5.0  # deg
# Replay guard (non-official starts), in the official slot frame, for the latch pad's offset from the official start:
GUARD_N_ROBOT = 0.045  # across the card toward the robot (the pitched pressing hand covers a wide band on that side)
GUARD_N_FAR = 0.020  # across the card away from the robot
GUARD_U = 0.050  # along the slot
GUARD_MARGIN = 1.2  # non-official starts are redrawn until they are at least 1.2 x the guard away (no boundary parking)


def _draw(rng):
    dx, dy = float(rng.uniform(-JIT_X, JIT_X)), float(rng.uniform(-JIT_Y, JIT_Y))
    yaw = float(rng.uniform(-JIT_YAW, JIT_YAW))
    return dict(dx=dx, dy=dy, yaw=yaw)


def _raw_spec(seed):
    return _draw(np.random.default_rng(int(seed)))


def bench_frame(spec):
    """(origin, rotation) of the S frame in the station frame."""
    p = np.array([BENCH_XY[0] + spec["dx"], BENCH_XY[1] + spec["dy"], BOARD_Z])
    return p, rz(BENCH_YAW0 + spec["yaw"])


def pad_point(spec):
    p, r = bench_frame(spec)
    return p + r @ (np.array(PAD_CENTER) * MM)


def pad_offset(spec):
    """(along, across) offset (m) of a start's latch pad from the official start's, in the official slot frame
    (across > 0: toward the robot)."""
    off = _raw_spec(OFFICIAL_SEED)
    _, ro = bench_frame(off)
    d = ro.T @ (pad_point(spec) - pad_point(off))
    return float(d[0]), float(d[1])


def guard_metric(spec):
    """How far the latch pad of a start is from where the official trace presses it, in guard units (>= 1: outside the
    guard): max(across / 45 mm toward the robot or across / 20 mm away from it, |along| / 50 mm)."""
    a, c = pad_offset(spec)
    gn = c / GUARD_N_ROBOT if c >= 0 else -c / GUARD_N_FAR
    return max(gn, abs(a) / GUARD_U)


def _guarded_spec(seed):
    """Seeded non-official start: draws from the seed's RNG until the start is at least GUARD_MARGIN guard units from
    the official start (so no start sits on the guard boundary)."""
    rng = np.random.default_rng(int(seed))
    for _ in range(2000):
        spec = _draw(rng)
        if guard_metric(spec) >= GUARD_MARGIN:
            return spec
    raise RuntimeError(f"no guarded start for seed {seed}")


def start_from_offset(along_mm, across_mm, dyaw_deg):
    """A start (verification aid) whose latch pad is offset (along, across) mm from the official start's pad, in the
    official slot frame, with the benchtable turned dyaw_deg more than at the official start."""
    off = _raw_spec(OFFICIAL_SEED)
    _, ro = bench_frame(off)
    yaw = off["yaw"] + float(dyaw_deg)
    target = pad_point(off) + ro @ np.array([along_mm * MM, across_mm * MM, 0.0])
    p = target - rz(BENCH_YAW0 + yaw) @ (np.array(PAD_CENTER) * MM)
    return dict(dx=float(p[0] - BENCH_XY[0]), dy=float(p[1] - BENCH_XY[1]), yaw=yaw)


SETUPS = (  # named (unseeded) setups
    dict(dx=0.0, dy=0.0, yaw=0.0),
    dict(dx=0.020, dy=0.030, yaw=4.0),
    dict(dx=-0.020, dy=-0.040, yaw=-4.0),
    dict(dx=0.015, dy=0.050, yaw=-5.0),
)


def setup_spec(setup):
    """Start spec for a named setup (int) or a seeded one (('seed', s))."""
    if isinstance(setup, (tuple, list)) and setup and setup[0] == "seed":
        s = int(setup[1])
        spec = _raw_spec(s) if s == OFFICIAL_SEED else _guarded_spec(s)
        return dict(spec, name=f"seed {s}")
    if isinstance(setup, (tuple, list)) and setup and setup[0] == "offset":  # verification aid (GPU_START)
        a, c, y = (float(v) for v in setup[1:4])
        return dict(start_from_offset(a, c, y), name=f"offset along {a} across {c} dyaw {y}")
    return dict(SETUPS[int(setup) % len(SETUPS)], name=f"setup {int(setup) % len(SETUPS)}")


# ---------------------------------------------------------------- measurements (pure geometry)
def card_in_slot(pk, rk, ps, rs):
    """Card pose relative to the slot frame: offset (m, S frame) and the card's tilt angles (deg)."""
    q = rs.T @ (np.asarray(pk, float) - ps)
    m = rs.T @ rk
    roll_u = math.degrees(math.atan2(m[2, 1], m[2, 2]))  # about u (card leaning across)
    pitch_n = math.degrees(math.atan2(-m[2, 0], math.hypot(m[2, 1], m[2, 2])))  # about n (rocking in its plane)
    return dict(du=float(q[0]), dn=float(q[1]), dz=float(q[2]), roll_u=roll_u, pitch_n=pitch_n)


def finger_bottom_rise(pk, rk, ps, rs):
    """Lowest point of the gold-finger tab and lip above the board (m, S frame z), minus its seated height."""
    pts = np.vstack([card_pieces()["finger"], card_pieces()["lip"]])
    w = (rs.T @ ((pts @ rk.T) + np.asarray(pk, float) - ps).T).T
    return float(w[:, 2].min() - FINGER[2] * MM)


SLOT_ZONE = (40.0, 145.0, 10.0, SLOT_TOP + 2.0)  # mm (S frame): u range, |n| limit, z limit of the slot's volume


def finger_points_s(pk, rk, ps, rs):
    """Corner points (mm, S frame) of the card's gold-finger tab, retention lip and notch wall."""
    pts = np.vstack([card_pieces()[k] for k in ("finger", "lip", "notch")])
    return (rs.T @ ((pts @ rk.T) + np.asarray(pk, float) - ps).T).T / MM


def in_slot(pk, rk, ps, rs):
    """True while any point of the card's edge connector (fingers, lip, notch) is inside the slot's volume: u 40-145
    mm, |n| < 10 mm, z below 2 mm above the housing top (the card is not out of the slot)."""
    w = finger_points_s(pk, rk, ps, rs)
    u0, u1, nl, zl = SLOT_ZONE
    inside = (w[:, 0] > u0) & (w[:, 0] < u1) & (np.abs(w[:, 1]) < nl) & (w[:, 2] < zl)
    return bool(inside.any())


def engaged(pk, rk, ps, rs):
    """True while the gold fingers sit between the contact rows (the tab's centre within 3 mm across and 5 mm along of
    its seated place): only then does the contact friction act."""
    q = rs.T @ (np.asarray(pk, float) + rk @ (np.array([89.5, 0.0, 7.5]) * MM) - ps) / MM
    return bool(abs(q[1]) < 3.0 and abs(q[0] - 89.5) < 5.0 and q[2] < 7.5 + CONTACT_RELEASE[1] + 1.0)


def card_corners(pk, rk):
    lo, hi = card_box()
    c = box_corners(lo[0], hi[0], lo[1], hi[1], lo[2], hi[2])
    return c @ rk.T + np.asarray(pk, float)


CARD_VERTICES = np.unique(np.vstack(tuple(card_pieces().values())), axis=0)


def card_vertices(origin, orientation):
    """Vertices of every physical card component, including the I/O bracket and its flange."""
    return CARD_VERTICES @ orientation.T + origin


def point_to_card(point, pk, rk):
    """Distance (m) from a point to the card's bounding box (0 inside)."""
    lo, hi = card_box()
    q = rk.T @ (np.asarray(point, float) - pk)
    d = np.maximum(np.maximum(lo - q, q - hi), 0.0)
    return float(np.linalg.norm(d))


def latch_angle(rl, rs):
    """Latch opening angle (rad, + = pressed open) from its rotation and the slot frame."""
    m = rs.T @ rl  # rotation about n (column 1) in S
    return float(math.atan2(m[0, 2], m[2, 2]))
