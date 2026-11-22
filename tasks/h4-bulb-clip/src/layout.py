"""Scene layout and geometry of the h4-bulb-clip task (station frame: x forward from the arm bases, y to the robot's
left, z up; metres). Engine-independent (numpy only).

Frames
* H (housing / seat frame): origin on the bulb bore axis in the seat face of the headlamp's bulb boss (the plane the
  bulb's reference flange rests on), x_H = station +x (the bulb goes in along +x, away from the robot), y_H, z_H =
  station y, z. The three notches of the boss are fixed in H.
* B (bulb frame): origin on the bulb axis in the front face of its reference flange (the H4 "reference plane"),
  x_B toward the glass tip. Seated and fitted: B == H (roll 0). The tabs sit at TAB_ANGLES about x_B, measured from
  +y_B toward +z_B.

Bulb: Philips 12342 type (H4, 12 V 60/55 W, P43t-38 cap). The H4 type, the P43t-38 cap and the three unequal tabs are
verified from product listings; the individual dimensions below are representative values (the IEC 60061 cap sheet
and the maker's drawing could not be read on this host), each marked in README "Fidelity".
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475  # station table top (yam_arena TABLE_TOP["station"])


def rx(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def ry(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rz(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# ---------------------------------------------------------------- bulb (frame B), metres
COLLAR_R, COLLAR_X = 0.0100, (0.0, 0.0060)  # metal cup that carries the glass, in front of the flange
GLASS_R, GLASS_X = 0.00625, (0.0060, 0.0440)  # quartz/hard-glass envelope (12.5 mm), dome to GLASS_TIP
GLASS_TIP = 0.0480
FLANGE_R, FLANGE_T = 0.0185, 0.0010  # reference flange (prefocus ring): disc 37 mm, 1.0 mm steel
TAB_R = 0.0215  # tabs reach a 43 mm circle
TAB_ANGLES = (90.0, 250.0, 335.0)  # deg about x_B from +y_B toward +z_B (asymmetric: one fit only)
TAB_W = (0.0100, 0.0060, 0.0060)  # one wide and two narrow tabs
BASE_R, BASE_X = 0.0120, (-0.0160, -0.0010)  # cap body behind the flange (grip zone), 24 mm
BOSS_R, BOSS_X = 0.0080, (-0.0200, -0.0160)  # terminal boss
BLADE_X = (-0.0300, -0.0200)  # three 6.3 x 0.8 mm blades, 10 mm long
BLADE_W, BLADE_T = 0.0063, 0.0008
BLADES = ((0.0, 0.0045, 0.0), (90.0, -0.0040, 0.0040), (90.0, -0.0040, -0.0040))  # (rot deg, y, z) of each blade
BULB_MASS = 0.018  # kg (estimate)
# mass split (estimates from the part volumes): glass envelope ~4 g (12.5 x 42 mm hard glass, ~1 mm wall, plus the
# filaments); cap 14 g, flange-heavy (the 37 x 1 mm steel flange alone is ~8 g)
GLASS_MASS, GLASS_COM_X = 0.004, 0.027
CAP_COM_X = -0.004
BULB_COM = np.array([((BULB_MASS - GLASS_MASS) * CAP_COM_X + GLASS_MASS * GLASS_COM_X) / BULB_MASS, 0.0, 0.0])
BULB_FRICTION = 0.4  # metal cap / pads (PhysX averages with the pad material)

# ---------------------------------------------------------------- headlamp bulb boss (frame H)
HOLE_R = 0.0103  # bore through the reflector boss: 0.3 mm radial play on the collar (it carries the bulb's front)
HOLE_CHAMFER = 0.0008  # 45 deg lead-in on the bore at the seat face
PLATE_X = (0.0, 0.0040)  # boss plate in front of the seat face
PLATE_R = 0.0450
RIM_R = (0.0189, 0.0250)  # locating rim around the flange: 0.4 mm radial play on the 37 mm disc
RIM_X = (-FLANGE_T, 0.0)  # rim as high as the flange is thick: a seated flange is flush with the rim top
NOTCH_PLAY = 0.0005  # per side on each tab
RIM_CHAMFER = 0.0008  # 45 deg lead-in on the rim bore (entry edge), 0.2 mm of straight bore below it
LAMP_R = 0.089  # 7 in round headlamp (178 mm): back shell
SHELL_X = (0.0040, 0.0700)  # cosmetic depth of the lamp shell in front of the boss

# ---------------------------------------------------------------- wire spring clip (bail), frame H
WIRE_R = 0.0008  # 1.6 mm spring-steel wire
CLIP_HALF_W = 0.0150  # legs cross the flange 15 mm above / below the axis
HINGE_Y = 0.0260  # hinge line (z axis) 26 mm to the robot's left of the bore axis
CATCH_Y = -0.0260  # closed: the bight lies 26 mm to the right of the axis
HINGE_X = RIM_X[0] - WIRE_R  # closed legs rest on the flange / rim top
CLIP_LEN = HINGE_Y - CATCH_Y  # 52 mm from the hinge to the bight
CLIP_MASS = 0.012  # kg (real wire ~3 g; raised for solver stability, see README)
CLIP_LIMITS_DEG = (-4.0, 170.0)
CLIP_MAX_W_DPS = 360.0  # angular speed cap of the bail (solver setting against tunnelling; see README)
CLIP_DAMPING = 2.0e-4  # N m s/rad hinge drag (computed torque)
CLIP_FRICTION_NM = 0.004  # hinge friction torque (computed, Coulomb, smoothed): the bail stays where it is left
# start angle: the bail swung open toward the robot's left (deg, 0 = closed)
CLIP_OPEN_RANGE = (95.0, 105.0)
CLIP_OPEN_NAMED = (104.0, 100.0, 96.0, 102.0, 98.0)

# ---------------------------------------------------------------- catch (sprung latch, slides along y_H)
CATCH_TRAVEL = 0.0030  # m: 0 = rest (engaged position), -CATCH_TRAVEL fully cammed away (toward -y)
CATCH_PRELOAD_N = 4.0
CATCH_RATE_N_M = 1600.0  # 1.6 N/mm
CATCH_MASS = 0.010
LIP_EDGE_Y = CATCH_Y + WIRE_R + 0.0004  # lip edge 0.4 mm past the bight's far side when engaged
LIP_X = (HINGE_X - WIRE_R - 0.0022, HINGE_X - WIRE_R - 0.0002)  # 2 mm thick, 0.2 mm behind the closed bight
LIP_Z = (-0.0060, 0.0060)
CATCH_OUTER_Y = CATCH_Y - 0.0040
RAMP_DEPTH = 0.0040  # 45 deg ramp on the lip's back (the bight cams the catch toward -y)

# ---------------------------------------------------------------- placement (station frame)
H = np.array([0.400, 0.000, TOP + 0.150])  # seat centre: headlamp bulb axis 150 mm above the table
HOLDER = np.array([0.215, -0.060, TOP])  # bulb holder post, bottom centre on the table
HOLDER_TOP = 0.075  # cup rim height above the table: the flange rests on it, glass down
CUP_R = (0.0115, 0.0160)  # cup bore (1.5 mm radial play on the collar) / outer radius (< flange disc)
CUP_DEPTH = 0.055
HOLDER_JITTER = 0.010  # seeded: holder moved by up to +-10 mm in x and y
# engine-bay mock-up around the lamp (boxes: centre, size)
PANEL = (np.array([0.405 + 0.075, 0.0, TOP + 0.150]), (0.010, 0.50, 0.30))  # radiator support (cosmetic, ahead)
BATTERY = (np.array([0.300, 0.200, TOP + 0.050]), (0.160, 0.160, 0.100))  # battery box, left rear of the lamp
APRON = (np.array([0.320, -0.165, TOP + 0.055]), (0.180, 0.020, 0.110))  # inner fender apron, right of the lamp
# named setups: bulb roll in its holder (deg about its axis), clip start angle
SETUP_ROLL = (37.0, -124.0, 151.0, -58.0, 92.0)


def tab_dirs(roll_deg=0.0):
    """Unit radial directions (y, z in the B frame) of the tabs for a bulb turned by roll_deg about its axis."""
    return [
        (math.cos(math.radians(a + roll_deg)), math.sin(math.radians(a + roll_deg)))
        for a in TAB_ANGLES
    ]


def setup_spec(setup):
    """Start state: {'roll': deg, 'clip': deg, 'dx': m, 'dy': m, 'name': str}. ``setup`` is a named index (int) or
    ('seed', n): roll uniform over 0..360 deg, clip angle uniform over CLIP_OPEN_RANGE, holder offset uniform in
    +-HOLDER_JITTER, all from seed n."""
    if isinstance(setup, tuple) and setup[0] == "seed":
        rng = np.random.default_rng(int(setup[1]))
        roll = float(rng.uniform(-180.0, 180.0))
        clip = float(rng.uniform(*CLIP_OPEN_RANGE))
        dx, dy = (float(v) for v in rng.uniform(-HOLDER_JITTER, HOLDER_JITTER, size=2))
        return dict(roll=roll, clip=clip, dx=dx, dy=dy, name=f"seed {int(setup[1])}")
    k = int(setup) % len(SETUP_ROLL)
    return dict(
        roll=SETUP_ROLL[k], clip=CLIP_OPEN_NAMED[k], dx=0.0, dy=0.0, name=f"setup {k}"
    )


def holder_pos(setup):
    sp = setup_spec(setup)
    return HOLDER + np.array([sp["dx"], sp["dy"], 0.0])


# bulb in the holder: glass down, axis vertical; B -> station rotation = (x_B -> -z) then roll about the axis
R_GLASS_DOWN = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])  # = ry(90): x_B -> -z, z_B -> +x


def bulb_holder_pose(setup):
    sp = setup_spec(setup)
    p = holder_pos(setup) + np.array([0.0, 0.0, HOLDER_TOP + 0.0001])
    return p, R_GLASS_DOWN @ rx(sp["roll"])


def bulb_seated_pose(roll_deg=0.0):
    return H.copy(), rx(roll_deg)


def clip_rot(phi_deg):
    """Bail orientation in H for opening angle phi (rotation about +z_H through the hinge line; positive phi swings
    the bight back toward the robot, i.e. toward -x)."""
    return rz(-phi_deg)


def hinge_point():
    return np.array([HINGE_X, HINGE_Y, 0.0])


def bight_point(phi_deg):
    """Centre of the bight wire (mid-height) in H at bail angle phi."""
    return hinge_point() + clip_rot(phi_deg) @ np.array([0.0, -CLIP_LEN, 0.0])


def clip_angle(rot_h):
    """Bail angle (deg) from its rotation matrix in H (rotation about z)."""
    return -math.degrees(math.atan2(rot_h[1, 0], rot_h[0, 0]))


# ---------------------------------------------------------------- convex pieces
def cylinder_x(r, x0, x1, n=24, center=(0.0, 0.0)):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = np.c_[center[0] + r * np.cos(a), center[1] + r * np.sin(a)]
    return np.r_[np.c_[np.full(n, x0), ring], np.c_[np.full(n, x1), ring]]


def cylinder_z(r, z0, z1, n=24, center=(0.0, 0.0)):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = np.c_[center[0] + r * np.cos(a), center[1] + r * np.sin(a)]
    return np.r_[np.c_[ring, np.full(n, z0)], np.c_[ring, np.full(n, z1)]]


def segment(p0, p1, r, n=8):
    """Convex hull points of a round bar from p0 to p1 (octagonal section)."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    d /= np.linalg.norm(d)
    ref = np.array([0.0, 0.0, 1.0]) if abs(d[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(d, ref)
    u /= np.linalg.norm(u)
    v = np.cross(d, u)
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = np.outer(np.cos(a), u) * r + np.outer(np.sin(a), v) * r
    return np.r_[p0 + ring, p1 + ring]


def box_pts(lo, hi):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return np.array(
        [[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
    )


def glass_pts(n=24):
    """Glass envelope: cylinder plus a rounded tip (hull points, frame B)."""
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    pts = [np.c_[np.full(n, GLASS_X[0]), GLASS_R * np.cos(a), GLASS_R * np.sin(a)]]
    for t in np.linspace(0.0, 1.0, 5):
        x = GLASS_X[1] + t * (GLASS_TIP - GLASS_X[1])
        r = GLASS_R * math.sqrt(max(1.0 - t * t, 0.0))
        r = max(r, 0.0015)
        pts.append(np.c_[np.full(n, x), r * np.cos(a), r * np.sin(a)])
    return np.vstack(pts)


def annular_sector(r0, r1, a0, a1, x0, x1, n=None):
    """Hull points of an annular sector (angles deg about x, from +y toward +z) between planes x0..x1."""
    n = n or max(2, int(abs(a1 - a0) / 6) + 2)
    a = np.radians(np.linspace(a0, a1, n))
    pts = []
    for r in (r0, r1):
        for x in (x0, x1):
            pts.append(np.c_[np.full(n, x), r * np.cos(a), r * np.sin(a)])
    return np.vstack(pts)


def tab_half_angle(i, r, play=0.0):
    """Half angle (deg) subtended at radius r by tab i (plus play per side)."""
    return math.degrees(math.asin((TAB_W[i] / 2 + play) / r))


def bulb_pieces():
    """{'glass': [...], 'metal': [...], 'base': [...]} lists of hull point arrays in frame B."""
    metal = [cylinder_x(COLLAR_R, *COLLAR_X, n=24), cylinder_x(FLANGE_R, -FLANGE_T, 0.0, n=48)]
    for i, a in enumerate(TAB_ANGLES):
        h = tab_half_angle(i, TAB_R)
        metal.append(annular_sector(FLANGE_R - 0.001, TAB_R, a - h, a + h, -FLANGE_T, 0.0, n=4))
    base = [cylinder_x(BASE_R, *BASE_X, n=32), cylinder_x(BOSS_R, *BOSS_X, n=24)]
    for rot, y, z in BLADES:
        hw, ht = (BLADE_W / 2, BLADE_T / 2) if rot == 0.0 else (BLADE_T / 2, BLADE_W / 2)
        base.append(box_pts((BLADE_X[0], y - hw, z - ht), (BLADE_X[1], y + hw, z + ht)))
    return dict(glass=[glass_pts()], metal=metal, base=base)


def bulb_inertia():
    m, r, ln = BULB_MASS, 0.012, 0.078
    ixx = 0.5 * m * r * r
    iyy = m * (3 * r * r + ln * ln) / 12
    return np.array([ixx, iyy, iyy])


def rim_sector(s0, s1, n=3):
    """One convex piece of the locating rim between angles s0..s1 (deg), with the 45 deg lead-in chamfer on its bore
    at the entry face (x = RIM_X[0], the side the bulb comes from)."""
    a = np.radians(np.linspace(s0, s1, n))
    pts = []
    for r, x in (
        (RIM_R[0] + RIM_CHAMFER, RIM_X[0]),
        (RIM_R[0], RIM_X[0] + RIM_CHAMFER),
        (RIM_R[0], RIM_X[1]),
        (RIM_R[1], RIM_X[0]),
        (RIM_R[1], RIM_X[1]),
    ):
        pts.append(np.c_[np.full(n, x), r * np.cos(a), r * np.sin(a)])
    return np.vstack(pts)


def rim_pieces(sector=6.0):
    """Locating rim (frame H) with the three notches cut out, as chamfered annular sectors."""
    gaps = []
    for i, a in enumerate(TAB_ANGLES):
        h = tab_half_angle(i, RIM_R[0], NOTCH_PLAY)
        gaps.append((a - h, a + h))
    out = []
    edges = sorted(gaps)
    for k in range(len(edges)):
        a0 = edges[k][1]
        a1 = edges[(k + 1) % len(edges)][0] + (360.0 if k == len(edges) - 1 else 0.0)
        n = max(1, int(math.ceil((a1 - a0) / sector)))
        for j in range(n):
            s0 = a0 + (a1 - a0) * j / n
            s1 = a0 + (a1 - a0) * (j + 1) / n
            out.append(rim_sector(s0, s1))
    return out


def plate_pieces(sector=10.0):
    """Boss plate (frame H, x 0..4 mm) round the bore, with the bore's 45 deg lead-in at the seat face."""
    out = []
    n = int(round(360.0 / sector))
    for j in range(n):
        a = np.radians(np.linspace(j * sector, (j + 1) * sector, 3))
        pts = []
        for r, x in (
            (HOLE_R + HOLE_CHAMFER, PLATE_X[0]),
            (HOLE_R, PLATE_X[0] + HOLE_CHAMFER),
            (HOLE_R, PLATE_X[1]),
            (PLATE_R, PLATE_X[0]),
            (PLATE_R, PLATE_X[1]),
        ):
            pts.append(np.c_[np.full(3, x), r * np.cos(a), r * np.sin(a)])
        out.append(np.vstack(pts))
    return out


def notch_floor_pieces():
    """Notch floors are the seat face itself (plate): nothing extra. Kept for clarity."""
    return []


def clip_pieces():
    """Bail pieces in the bail frame (= H at phi 0, origin at the hinge point): two legs and the bight."""
    y_end = -CLIP_LEN
    legs = [segment((0.0, 0.0, s * CLIP_HALF_W), (0.0, y_end, s * CLIP_HALF_W), WIRE_R) for s in (1.0, -1.0)]
    bight = segment((0.0, y_end, -CLIP_HALF_W), (0.0, y_end, CLIP_HALF_W), WIRE_R)
    return legs + [bight]


def clip_inertia():
    m = CLIP_MASS
    return np.array([m * CLIP_LEN**2 / 3, m * CLIP_HALF_W**2, m * CLIP_LEN**2 / 3])


def catch_pieces():
    """Catch lip (frame H at rest): a 2 mm lip whose back face is a 45 deg ramp; slides along y."""
    x0, x1 = LIP_X
    z0, z1 = LIP_Z
    # prism in (x, y): lip edge at LIP_EDGE_Y, ramp from the lip's back edge out to CATCH_OUTER_Y
    poly = [
        (x1, LIP_EDGE_Y),
        (x1 - 0.0005, LIP_EDGE_Y),
        (x1 - 0.0005 - (LIP_EDGE_Y - CATCH_OUTER_Y), CATCH_OUTER_Y),  # 45 deg ramp facing -x / +y
        (x1, CATCH_OUTER_Y),
    ]
    pts = np.array([[x, y, z] for x, y in poly for z in (z0, z1)])
    stem = box_pts((x1 - 0.0060, CATCH_OUTER_Y - 0.003, z0), (x1, CATCH_OUTER_Y, z1))
    return [pts, stem]


def to_world_h(p_h):
    return H + np.asarray(p_h, float)
