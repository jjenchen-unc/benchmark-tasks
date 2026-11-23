"""Scene layout and geometry of the edge-ring task (station frame: x forward from the arm bases, y to the robot's left,
z up; metres). Engine-independent (numpy only).

Chamber frame C: origin on the cathode axis at the ring seat (the base ring's top face), z up, x = station x. The edge
ring frame R: origin on the ring axis at its bottom face; seated, R = C rotated by the clocking angle (0 = fitted).
Representative 300 mm cathode ring-set dimensions (OEM drawings are proprietary; see README).
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475  # station table top (yam_arena TABLE_TOP["station"])


def rz(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# ---------------------------------------------------------------- chamber (cathode) geometry
C = np.array([0.405, 0.0, TOP + 0.029])  # ring seat centre in the station frame
PUCK_R = 0.149  # electrostatic chuck puck, diameter 298 mm (alumina)
PUCK_TOP = 0.0025  # chuck face 2.5 mm above the ring seat
PUCK_Z0 = -0.027  # puck body bottom (cosmetic depth)
BASE_R = (0.1493, 0.190)  # quartz base ring: bore 298.6, outer 380; top = seat (z 0)
BASE_Z0 = -0.012
LOC_PITCH_R = 0.160  # locating dowels on a 320 mm circle
LOC_ANGLES = (
    0.0,
    115.0,
    245.0,
)  # deg, chamber frame (asymmetric: only one clocking fits)
LOC_PIN_R = 0.0020  # dowel diameter 4.0 mm
LOC_PIN_H = 0.0038  # 3.8 mm proud of the seat (0.2 mm below the seated ring's top)
LOC_CHAMFER = 0.0008  # 0.8 x 45 deg lead-in: full diameter from 3.0 mm, i.e. before the bore passes the chuck face
LIFT_PITCH_R = 0.170  # lift pins on a 340 mm circle
LIFT_ANGLES = (60.0, 180.0, 300.0)
LIFT_PIN_R = 0.0015  # sapphire lift pins, diameter 3 mm
LIFT_UP = 0.025  # maintenance position: tips 25 mm above the seat
LIFT_DOWN = -0.001  # lowered: tips 1 mm below the seat
LIFT_SPEED = 0.005  # m/s
COVER_R = (0.181, 0.210)  # quartz cover ring (installed): bore 362, outer 420
COVER_TOP = 0.0040  # level with the seated edge ring's top
COVER_Z0 = -0.012
BOWL_R = (0.230, 0.245)  # lower chamber body: bore 460, 15 mm wall
BOWL_TOP = 0.127  # representative 5 in liner envelope, seat to rim
BOWL_FLOOR = TOP + 0.001 - C[2]  # floor on the table
CATHODE_R = 0.215  # cathode body below the rings (cosmetic)

# ---------------------------------------------------------------- edge ring (new)
RING_R = (
    0.1495,
    0.180,
)  # bore 299 (0.5 mm radial to the chuck), outer 360 (1.0 mm radial to the cover ring)
RING_T = 0.0040
RING_MASS = 0.29  # kg: silicon 2.33 g/cm3 x 124 cm3
RING_FRICTION = (
    0.5  # silicon on the gripper pads (the pads' own material combines with it)
)
HARD_FRICTION = 0.3  # silicon on alumina / sapphire / quartz / Delrin
# radial through-slots: 0.15 mm a side on the 4.0 mm dowels (with 0.1 mm a side the ring jammed on a dowel lead-in
# from 0.2 mm off; with 0.15 it seats up to ~0.35 mm)
SLOT_W = 0.0043
SLOT_R = (0.156, 0.164)  # 8 mm long on the 320 mm circle
SECTOR_DEG = 3.0  # collision pieces: 3 deg sectors (chord sag 0.05 mm at the bore)

# ---------------------------------------------------------------- ring stand and pendant
STAND = np.array(
    [0.120, 0.0, C[2] + BOWL_TOP + 0.014]
)  # top of the stand's arms (ring underside rests here)
STAND_POST_R = 0.020
STAND_ARM_W = 0.015
STAND_ARM_R = (0.018, 0.168)
STAND_ARM_T = 0.010
STAND_ARM_ANGLES = (45.0, 170.0, 300.0)
# named setups: ring angle on the stand (deg, 0 = fitted orientation); never near the fitted angle
SETUP_CLOCKING = (-25.0, -20.0, -15.0, 15.0, 20.0)
# seeded trials: angle uniform over these segments (deg, by length): the ring always has to be turned at least 10 deg
# Both ranges require active turning and leave access between the stand arms.
SEEDED_CLOCKING_SEGMENTS = ((10.0, 22.0), (-25.0, -10.0))
SEEDED_STAND_JITTER = 0.010  # and the stand moved by up to +-10 mm in x and y
PENDANT = np.array([0.30, -0.36, TOP])  # pendant box, bottom centre on the table
PENDANT_SIZE = (0.090, 0.060, 0.040)
KEY_SIZE = (0.020, 0.020, 0.006)
KEYS = {
    "up": np.array([-0.022, 0.0]),
    "down": np.array([0.022, 0.0]),
}  # key centres relative to the pendant (x, y)
KEY_PRESS_N = 5.0
KEY_HOLD_S = 0.10


def key_center(name):
    k = KEYS[name]
    return np.array(
        [
            PENDANT[0] + k[0],
            PENDANT[1] + k[1],
            PENDANT[2] + PENDANT_SIZE[2] + KEY_SIZE[2] / 2,
        ]
    )


def to_world_c(p_c):
    return C + np.asarray(p_c, float)


def ring_seated_pose(clock_deg=0.0):
    """(pos, rot) of the ring frame when seated at the given clocking."""
    return C.copy(), rz(clock_deg)


def ring_lift_pose(clock_deg=0.0, dx=0.0, dy=0.0):
    """Ring resting on the raised lift pins."""
    return C + np.array([dx, dy, LIFT_UP]), rz(clock_deg)


def setup_spec(setup):
    """Start state of a trial: {'angle': deg, 'dx': m, 'dy': m, 'name': str}. ``setup`` is a named setup index (int)
    or ('seed', n): angle uniform over SEEDED_CLOCKING_SEGMENTS (by length), and stand offset uniform in
    +-SEEDED_STAND_JITTER, from seed n."""
    if isinstance(setup, tuple) and setup[0] == "seed":
        rng = np.random.default_rng(int(setup[1]))
        u = float(rng.uniform(0.0, sum(hi - lo for lo, hi in SEEDED_CLOCKING_SEGMENTS)))
        for lo, hi in SEEDED_CLOCKING_SEGMENTS:
            if u < hi - lo:
                a = lo + u
                break
            u -= hi - lo
        else:
            a = SEEDED_CLOCKING_SEGMENTS[-1][1]
        dx, dy = (
            float(v)
            for v in rng.uniform(-SEEDED_STAND_JITTER, SEEDED_STAND_JITTER, size=2)
        )
        return dict(angle=a, dx=dx, dy=dy, name=f"seed {int(setup[1])}")
    k = int(setup) % len(SETUP_CLOCKING)
    return dict(angle=SETUP_CLOCKING[k], dx=0.0, dy=0.0, name=f"setup {k}")


def stand_pos(setup):
    sp = setup_spec(setup)
    return STAND + np.array([sp["dx"], sp["dy"], 0.0])


def ring_stand_pose(setup):
    sp = setup_spec(setup)
    return stand_pos(setup) + np.array([0.0, 0.0, 0.0001]), rz(sp["angle"])


# ---------------------------------------------------------------- convex piece generation
def _clip(poly, n, d):
    """Sutherland-Hodgman: keep the part of polygon (k x 2) with n . p <= d."""
    out = []
    k = len(poly)
    for i in range(k):
        a, b = poly[i], poly[(i + 1) % k]
        da, db = n @ a - d, n @ b - d
        if da <= 0:
            out.append(a)
        if (da < 0 < db) or (db < 0 < da):
            t = da / (da - db)
            out.append(a + t * (b - a))
    return np.array(out) if len(out) >= 3 else None


def _area(poly):
    x, y = poly[:, 0], poly[:, 1]
    return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def annulus_cells(r0, r1, sector_deg, phase_deg=0.0):
    """Convex quads (inner chord, outer chord) covering an annulus."""
    n = int(round(360.0 / sector_deg))
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


def cut_slots(cells, slots):
    """Remove rectangular radial slots from convex cells. slots: list of (angle_deg, r_in, r_out, width). Each cell that
    meets a slot is split into convex pieces around it (left / right of the slot, inside / outside its radial span)."""
    out = []
    for cell in cells:
        pieces = [cell]
        for ang, ra, rb, w in slots:
            a = math.radians(ang)
            t = np.array([-math.sin(a), math.cos(a)])  # tangential
            r = np.array([math.cos(a), math.sin(a)])  # radial
            nxt = []
            for p in pieces:
                u = p @ t
                v = p @ r
                if (
                    u.max() <= -w / 2
                    or u.min() >= w / 2
                    or v.max() <= ra
                    or v.min() >= rb
                ):
                    nxt.append(p)
                    continue
                for n_, d_ in ((t, -w / 2), (-t, -w / 2)):
                    q = _clip(p, n_, d_)
                    if q is not None and _area(q) > 1e-9:
                        nxt.append(q)
                mid = _clip(p, t, w / 2)
                mid = _clip(mid, -t, w / 2) if mid is not None else None
                if mid is not None:
                    for n_, d_ in ((r, ra), (-r, -rb)):
                        q = _clip(mid, n_, d_)
                        if q is not None and _area(q) > 1e-9:
                            nxt.append(q)
            pieces = nxt
        out += pieces
    return out


def prism(poly, z0, z1):
    """Vertices (2k x 3) of a vertical prism over a convex polygon."""
    poly = np.asarray(poly, float)
    k = len(poly)
    return np.r_[np.c_[poly, np.full(k, z0)], np.c_[poly, np.full(k, z1)]]


def ring_pieces():
    """Collision pieces of the edge ring in its own frame (list of vertex arrays)."""
    slots = [(a, SLOT_R[0], SLOT_R[1], SLOT_W) for a in LOC_ANGLES]
    cells = cut_slots(
        annulus_cells(RING_R[0], RING_R[1], SECTOR_DEG, phase_deg=SECTOR_DEG / 2), slots
    )
    return [prism(c, 0.0, RING_T) for c in cells]


def ring_inertia():
    m = RING_MASS
    a, b = RING_R
    izz = 0.5 * m * (a * a + b * b)
    ixx = m * (3 * (a * a + b * b) + RING_T**2) / 12
    return np.array([ixx, ixx, izz])


def base_ring_pieces():
    """Quartz base ring (chamber frame) with clearance holes for the lift pins."""
    holes = [
        (a, LIFT_PITCH_R - 0.002, LIFT_PITCH_R + 0.002, 0.004) for a in LIFT_ANGLES
    ]
    cells = cut_slots(annulus_cells(BASE_R[0], BASE_R[1], 4.0), holes)
    return [prism(c, BASE_Z0, 0.0) for c in cells]


def cylinder(r, z0, z1, n=32, chamfer=0.0, center=(0.0, 0.0)):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)

    def ring(rr, z):
        return np.c_[
            center[0] + rr * np.cos(a), center[1] + rr * np.sin(a), np.full(n, z)
        ]

    if chamfer > 0:
        return np.r_[ring(r, z0), ring(r, z1 - chamfer), ring(r - chamfer, z1)]
    return np.r_[ring(r, z0), ring(r, z1)]


def loc_pin_centers():
    return [
        np.array(
            [
                LOC_PITCH_R * math.cos(math.radians(a)),
                LOC_PITCH_R * math.sin(math.radians(a)),
            ]
        )
        for a in LOC_ANGLES
    ]


def lift_pin_centers():
    return [
        np.array(
            [
                LIFT_PITCH_R * math.cos(math.radians(a)),
                LIFT_PITCH_R * math.sin(math.radians(a)),
            ]
        )
        for a in LIFT_ANGLES
    ]


def slot_centers_world(pos, rot):
    """Slot centre points (bottom face) of the ring at pose (pos, rot), world frame."""
    out = []
    for a in LOC_ANGLES:
        p = np.array(
            [
                LOC_PITCH_R * math.cos(math.radians(a)),
                LOC_PITCH_R * math.sin(math.radians(a)),
                0.0,
            ]
        )
        out.append(pos + rot @ p)
    return out
