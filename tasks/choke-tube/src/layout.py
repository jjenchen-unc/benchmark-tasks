"""Scene layout and geometry of the choke-tube task (station frame: x forward from the arm bases, y to the robot's left,
z up; metres). Engine-independent (numpy only).

A toddler sits in a high chair pulled up to the right-hand end of the table; her silicone placemat is on the table in
front of her. On the robot's side of the table: a no-choke testing tube (16 CFR 1501 small-parts cylinder bore), three
toys and a toy storage box. Each trial's three toys come from a fixed set of nine real toys; which three, and where
they lie, is drawn from the trial seed.

Tube frame: origin on the tube axis at its bottom face, z up; the slanted floor's deep side is at the frame's -x.
Toy frames: origin at the toy's centre of mass, z up as it lies on the table.
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475  # station table top (yam_arena station.TABLE_TOP)
FLOOR_Z = -0.0125  # station floor (yam_arena station.FLOOR_Z)
TABLE_X = (-0.19, 0.91)  # station table extent
TABLE_Y = (-0.65, 0.65)


def rz(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# ---------------------------------------------------------------- no-choke testing tube
# Interior per 16 CFR 1501.4 / Fig. 1 (small parts cylinder): inside diameter 1.25 in (31.7 mm); floor slanted, depth
# below the rim 1.00 in (25.4 mm) at the shallow side to 2.25 in (57.1 mm) at the deep side. Exterior representative of
# consumer testers (translucent polypropylene): 2.5 mm wall, 3 mm under the deepest floor point, 18 g.
TUBE_ID = 0.0317
TUBE_R = TUBE_ID / 2
TUBE_WALL = 0.0025
TUBE_OR = TUBE_R + TUBE_WALL
DEPTH_SHALLOW = 0.0254
DEPTH_DEEP = 0.0571
TUBE_BASE = 0.0030  # material under the deepest floor point
TUBE_H = TUBE_BASE + DEPTH_DEEP  # rim height above the tube's bottom face: 60.1 mm
TUBE_MASS = 0.018
TUBE_FRICTION = 0.35  # polypropylene on wood / ABS (estimate)
TUBE_SECTOR_DEG = 10.0  # bore wall pieces: chord sag 0.06 mm at the bore


def floor_z(x_local):
    """Height of the slanted floor above the tube's bottom face at local x (deep side at -R)."""
    s = (float(x_local) + TUBE_R) / (2 * TUBE_R)
    return TUBE_BASE + s * (DEPTH_DEEP - DEPTH_SHALLOW)


# ---------------------------------------------------------------- the nine toys (fixed set; each trial shows three)
# fits = fits entirely inside the tube in some orientation under its own weight (the 16 CFR 1501 small-part test).
# hazard = a choking hazard for a child under 3: fits the tube, or is a ball of 1.75 in (44.45 mm) or less across
# (16 CFR 1500.18(a)(17), the small-ball rule; no ball in this set is hazardous by that rule alone).
# Dimensions: LEGO / DUPLO from the published brick module (LEGO: 8.0 mm pitch, 9.6 mm body height, studs 4.8 x 1.7 mm;
# DUPLO: 2x LEGO in plan, 19.2 mm body, studs 9.4 x 4.5 mm); painted hardwood toys from nominal inch sizes (hardwood,
# 0.70 g/cm3). Masses are estimates (see README fidelity).
SMALL_BALL_D = 0.04445
BLUE, GREEN, BRICK_RED = (0.16, 0.36, 0.78), (0.20, 0.62, 0.30), (0.78, 0.07, 0.06)
WOOD_RHO = 0.70e3


TOYS = {
    "lego2x2": dict(kind="brick", size=(0.0158, 0.0158, 0.0096), studs=(2, 0.0080, 0.0024, 0.0017), mass=0.0012,
                    fits=True, color=BRICK_RED, friction=0.35, label="LEGO 2x2 brick (3003)"),
    "duplo2x2": dict(kind="brick", size=(0.0317, 0.0317, 0.0192), studs=(2, 0.0160, 0.0047, 0.0045), mass=0.0060,
                     fits=False, color=BRICK_RED, friction=0.35, label="LEGO DUPLO 2x2 brick (3437)"),
    "cube19": dict(kind="cube", size=(0.01905,) * 3, mass=WOOD_RHO * 0.01905**3, fits=True, color=BLUE,
                   friction=0.45, label="3/4 in (19 mm) painted hardwood cube"),
    "ball25": dict(kind="ball", size=(0.0254,) * 3, mass=WOOD_RHO * math.pi / 6 * 0.0254**3, fits=True, color=BLUE,
                   friction=0.45, label="1 in (25.4 mm) painted hardwood ball"),
    "doll32": dict(kind="doll", size=(0.01905, 0.01905, 0.03175), head_d=0.0175,
                   mass=0.0, fits=True, color=GREEN, friction=0.45,
                   label="1-1/4 in (32 mm) painted wooden peg doll, 3/4 in body"),
}
for name, toy in TOYS.items():
    toy["hazard"] = name != "duplo2x2"
TOY_NAMES = tuple(TOYS)
BALL_ANG_DAMPING = 3.0  # rolling resistance stand-in for wood on a wooden table (PhysX has no rolling friction)
# Each pair includes the age-graded recipient toy and a small-parts distractor.
PAIRS = (("lego2x2", "duplo2x2"), ("cube19", "duplo2x2"), ("ball25", "duplo2x2"), ("doll32", "duplo2x2"))


def toy_half_height(name):
    return TOYS[name]["size"][2] / 2


def toy_top(name):
    """Height of the toy's highest point above its centre (studs included), lying as on the table."""
    t = TOYS[name]
    h = t["size"][2] / 2
    if t["kind"] == "brick":
        h += t["studs"][3]
    return h


def toy_inertia(name):
    t = TOYS[name]
    m = t["mass"]
    a, b, c = t["size"]
    if t["kind"] == "ball":
        i = 0.4 * m * (a / 2) ** 2
        return np.array([i, i, i])
    if t["kind"] == "stick":
        return m / 12 * np.array([b * b + c * c, a * a + c * c, a * a + b * b])
    if t["kind"] in ("cyl", "doll"):
        r = a / 2
        ixx = m * (3 * r * r + c * c) / 12
        return np.array([ixx, ixx, 0.5 * m * r * r])
    return m / 12 * np.array([b * b + c * c, a * a + c * c, a * a + b * b])


# ---------------------------------------------------------------- toddler, high chair, placemat
CHAIR = np.array([0.55, -0.80])  # high chair (tray removed, pulled up to the table's end), facing +y (the table)
CHAIR_SEAT_Z = 0.57  # seat height above the floor: the table top at her elbows
SHOULDER = np.array([0.55, -0.72, 0.84])  # toddler's shoulder centre
REACH_R = 0.42  # her reach zone: within 42 cm (horizontal) of her shoulder centre
MAT = np.array([0.42, -0.42])  # silicone placemat centre, on the table in front of her
MAT_SIZE = (0.30, 0.20, 0.003)
MAT_TOP = TOP + MAT_SIZE[2]

# ---------------------------------------------------------------- robot side: tube, toys, storage box
TUBE_XY = np.array([0.28, 0.01])  # tube start (centre of its base)
TUBE_JITTER = 0.010
TOY_REGION = ((0.29, 0.44), (-0.25, -0.08))  # toy start positions (x range, y range)
TOY_MIN_SEP = 0.075
BOX = np.array([0.33, 0.33])  # toy storage box (IKEA TROFAST 20 x 28 x 10 cm class), centre on the table
BOX_SIZE = (0.28, 0.20, 0.10)  # outer x, y, height
BOX_WALL = 0.004
PARK = np.array([-0.55, 0.0])  # unused toys rest on the floor behind the arms (not part of the trial)

# named setups (unseeded resets; real trials use these): toy names in order of their slots, slot positions fixed
NAMED_SETUPS = (
    ("ball25", "duplo2x2", "lego2x2"),
    ("cube19", "ball25", "duplo2x2"),
    ("lego2x2", "duplo2x2", "cube19"),
    ("duplo2x2", "ball25", "cube19"),
    ("duplo2x2", "lego2x2", "ball25"),
    ("doll32", "duplo2x2", "cube19"),
    ("duplo2x2", "doll32", "ball25"),
)
NAMED_SLOTS = ((0.33, -0.10), (0.41, -0.17), (0.32, -0.23))


def setup_spec(setup):
    """Start state of a trial: {'toys': [(name, x, y, yaw_deg)], 'tube': (x, y, yaw_deg), 'name': str}.

    ``setup`` is a named setup index (int) or ('seed', n). Seeded: one look-alike pair (one toy that fits the tube and
    one that does not, same material and colour) plus a third toy drawn from the other four; positions uniform in
    TOY_REGION with at least TOY_MIN_SEP between toys, random yaw; the tube moved by up to 10 mm and turned at random."""
    if isinstance(setup, tuple) and setup[0] == "seed":
        rng = np.random.default_rng(int(setup[1]))
        pair = PAIRS[int(rng.integers(len(PAIRS)))]
        rest = [n for n in TOY_NAMES if n not in pair]
        names = list(pair) + [rest[int(rng.integers(len(rest)))]]
        rng.shuffle(names)
        pts = []
        while len(pts) < 3:
            p = np.array(
                [rng.uniform(*TOY_REGION[0]), rng.uniform(*TOY_REGION[1])], float
            )
            if all(np.linalg.norm(p - q) >= TOY_MIN_SEP for q in pts):
                pts.append(p)
        yaws = rng.uniform(-45.0, 45.0, size=3)
        tj = rng.uniform(-TUBE_JITTER, TUBE_JITTER, size=2)
        tyaw = float(rng.uniform(-180.0, 180.0))
        toys = [
            (n, float(p[0]), float(p[1]), float(y)) for n, p, y in zip(names, pts, yaws)
        ]
        return dict(
            toys=toys,
            tube=(float(TUBE_XY[0] + tj[0]), float(TUBE_XY[1] + tj[1]), tyaw),
            name=f"seed {int(setup[1])}",
        )
    k = int(setup) % len(NAMED_SETUPS)
    toys = [
        (n, s[0], s[1], 0.0) for n, s in zip(NAMED_SETUPS[k], NAMED_SLOTS)
    ]
    return dict(
        toys=toys,
        tube=(float(TUBE_XY[0]), float(TUBE_XY[1]), 0.0),
        name=f"setup {k}",
    )


def toy_rest_z(name):
    return TOP + toy_half_height(name) + 0.0002


def park_pos(i):
    return np.array([PARK[0], PARK[1] + 0.08 * (i - 4.0), FLOOR_Z + 0.04])


def authored(setup):
    """Reset poses {body: (pos, rot)} of the tube and all toys (unused toys parked on the floor)."""
    sp = setup_spec(setup)
    out = {}
    tx, ty, tyaw = sp["tube"]
    out["tube"] = (np.array([tx, ty, TOP + 0.0002]), rz(tyaw))
    used = {n: (x, y, yw) for n, x, y, yw in sp["toys"]}
    for i, n in enumerate(TOY_NAMES):
        if n in used:
            x, y, yw = used[n]
            out[n] = (np.array([x, y, toy_rest_z(n)]), rz(yw))
        else:
            out[n] = (park_pos(i), np.eye(3))
    return out


def active_toys(setup):
    return [n for n, *_ in setup_spec(setup)["toys"]]


# ---------------------------------------------------------------- geometry helpers (convex pieces)
def cylinder(r, z0, z1, n=32, center=(0.0, 0.0)):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)

    def ring(z):
        return np.c_[center[0] + r * np.cos(a), center[1] + r * np.sin(a), np.full(n, z)]

    return np.r_[ring(z0), ring(z1)]


def box_pts(center, size):
    hx, hy, hz = (s / 2 for s in size)
    c = np.asarray(center, float)
    return np.array(
        [
            c + [sx * hx, sy * hy, sz * hz]
            for sx in (-1, 1)
            for sy in (-1, 1)
            for sz in (-1, 1)
        ]
    )


def tube_wall_pieces():
    """Convex prisms of the tube wall (tube frame)."""
    out = []
    n = int(round(360.0 / TUBE_SECTOR_DEG))
    for i in range(n):
        a0, a1 = (math.radians(i * 360.0 / n), math.radians((i + 1) * 360.0 / n))
        poly = [
            (TUBE_R * math.cos(a0), TUBE_R * math.sin(a0)),
            (TUBE_OR * math.cos(a0), TUBE_OR * math.sin(a0)),
            (TUBE_OR * math.cos(a1), TUBE_OR * math.sin(a1)),
            (TUBE_R * math.cos(a1), TUBE_R * math.sin(a1)),
        ]
        out.append(
            np.array([(x, y, 0.0) for x, y in poly] + [(x, y, TUBE_H) for x, y in poly])
        )
    return out


def tube_floor_piece(n=48):
    """The solid under the slanted floor (convex: the bore cylinder cut by the floor plane), tube frame."""
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    xs, ys = TUBE_R * np.cos(a), TUBE_R * np.sin(a)
    bottom = np.c_[xs, ys, np.zeros(n)]
    top = np.c_[xs, ys, [floor_z(x) for x in xs]]
    return np.r_[bottom, top]


def tube_inertia():
    m = TUBE_MASS
    izz = 0.5 * m * (TUBE_R**2 + TUBE_OR**2)
    ixx = m * (3 * (TUBE_R**2 + TUBE_OR**2) + TUBE_H**2) / 12
    return np.array([ixx, ixx, izz])


def tube_com():
    return np.array([0.0, 0.0, TUBE_H * 0.45])


def tube_rim_center(pos, rot):
    return np.asarray(pos, float) + np.asarray(rot, float) @ np.array([0.0, 0.0, TUBE_H])


def toy_corners(name):
    """Points spanning the toy's solid (toy frame) for 'fully below the rim' checks."""
    t = TOYS[name]
    a, b, c = t["size"]
    if t["kind"] == "ball":
        r = a / 2
        dirs = [
            np.array(v, float) / np.linalg.norm(v)
            for v in (
                (1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1),
                (1, 1, 1), (1, 1, -1), (1, -1, 1), (1, -1, -1),
                (-1, 1, 1), (-1, 1, -1), (-1, -1, 1), (-1, -1, -1),
            )
        ]
        return np.array([r * d for d in dirs])
    if t["kind"] in ("cyl", "doll", "stick"):
        return toy_hull_points(name)
    pts = list(box_pts((0, 0, 0), (a, b, c)))
    if t["kind"] == "brick":
        k, pitch, sr, sh = t["studs"]
        for ix in range(k):
            for iy in range(k):
                cx, cy = (ix - (k - 1) / 2) * pitch, (iy - (k - 1) / 2) * pitch
                pts.append(np.array([cx, cy, c / 2 + sh]))
    return np.array(pts)


def toy_hull_points(name, n=24):
    """Convex-hull points of an upright cylindrical toy (toy frame, origin at its middle): a cylinder ('cyl'), a
    body cylinder topped by a spherical head ('doll', as one convex hull), or a hexagonal stick lying along x ('stick')."""
    t = TOYS[name]
    if t["kind"] == "stick":  # hexagonal prism along x, flats on top and bottom
        length, flats = t["size"][0], t["size"][2]
        rc = flats / math.sqrt(3)  # circumradius
        ang = np.radians(np.arange(6) * 60.0)
        yz = np.c_[rc * np.cos(ang), rc * np.sin(ang)]  # vertices at y = +-rc: flats at z = +-flats/2
        return np.r_[np.c_[np.full(6, -length / 2), yz], np.c_[np.full(6, length / 2), yz]]
    d, _, h = t["size"]
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = lambda r, z: np.c_[r * np.cos(a), r * np.sin(a), np.full(n, z)]  # noqa: E731
    if t["kind"] == "cyl":
        return np.r_[ring(d / 2, -h / 2), ring(d / 2, h / 2)]
    hr = t["head_d"] / 2
    zc = h / 2 - hr  # head centre
    body_top = zc - 0.3 * hr
    pts = [ring(d / 2, -h / 2), ring(d / 2, body_top)]
    for el in np.linspace(0, np.pi / 2, 5)[1:]:
        pts.append(ring(hr * math.cos(el), zc + hr * math.sin(el)) if el < np.pi / 2 else np.array([[0, 0, h / 2]]))
    pts.append(ring(hr, zc))
    return np.vstack(pts)


def _hull_volume(name):
    from scipy.spatial import ConvexHull

    return float(ConvexHull(toy_hull_points(name, n=48)).volume)


for _n, _t in TOYS.items():
    if _t["kind"] in ("doll", "stick"):
        _t["mass"] = WOOD_RHO * _hull_volume(_n)
