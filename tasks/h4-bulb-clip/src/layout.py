"""Scene layout and geometry of the h4-bulb-clip task (station frame: x forward from the arm bases, y to the robot's
left, z up; metres). Engine-independent (numpy only).

Frames
* H (housing / seat frame): origin on the bulb bore axis in the seat face of the headlamp's bulb boss (the plane the
  bulb's reference flange rests on), x_H = station +x (the bulb goes in along +x, away from the robot), y_H, z_H =
  station y, z. The three notches of the boss are fixed in H.
* B (bulb frame): origin on the bulb axis in the front face of its reference flange (the H4 "reference plane"),
  x_B toward the glass tip. Seated and fitted: B == H (roll 0). The tabs sit at TAB_ANGLES about x_B, measured from
  +y_B toward +z_B.

The nominal keyed interface uses the P43t cap and holder sheets 7004-39-6 and 7005-39-4.
Fixture-specific spring dimensions and material approximations are documented in README.
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
COLLAR_R, COLLAR_X = 0.011, (0.0, 0.008)  # metal cup that carries the glass, in front of the flange
GLASS_R, GLASS_X = 0.00625, (0.0080, 0.0440)  # quartz/hard-glass envelope (12.5 mm), dome to GLASS_TIP
GLASS_TIP = 0.0480
PILOT_R = 0.0379 / 2  # formed shoulder diameter L, IEC cap sheet 7004-39-6
PILOT_HEIGHT = 0.0055  # OSRAM prefocus ring drawing; simplified cylindrical shoulder
FLANGE_R, FLANGE_T = 0.04295 / 2, 0.0010  # centring diameter M; flat backing proxy
TAB_R = 0.0518 / 2  # N, the three lug ends trace a larger circle than M
TAB_ANGLES = (90.0, 225.0, 315.0)  # lower two lugs 45 degrees either side of downward
TAB_W = (0.00795, 0.006, 0.006)  # widths Z and Z1
BASE_R, BASE_X = 0.0120, (-0.0160, -0.0010)  # cap body behind the flange (grip zone), 24 mm
BOSS_R, BOSS_X = 0.0080, (-0.0200, -0.0160)  # terminal boss
BLADE_X = (-0.0300, -0.0200)  # three 6.3 x 0.8 mm blades, 10 mm long
BLADE_W, BLADE_T = 0.0063, 0.0008
BLADES = ((0.0, 0.0045, 0.0), (90.0, -0.0040, 0.0040), (90.0, -0.0040, -0.0040))  # (rot deg, y, z) of each blade
BULB_MASS = 0.018  # kg (estimate)
# mass split (estimates from the part volumes): glass envelope ~4 g (12.5 x 42 mm hard glass, ~1 mm wall, plus the
# filaments); cap 14 g, flange-heavy (the prefocus ring carries much of the cap mass)
GLASS_MASS, GLASS_COM_X = 0.004, 0.027
CAP_COM_X = -0.004
BULB_COM = np.array([((BULB_MASS - GLASS_MASS) * CAP_COM_X + GLASS_MASS * GLASS_COM_X) / BULB_MASS, 0.0, 0.0])
BULB_FRICTION = 0.4  # metal cap / pads (PhysX averages with the pad material)

# ---------------------------------------------------------------- headlamp bulb boss (frame H)
HOLE_R = 0.041 / 2  # clearance for the formed shoulder; above the 40.5 mm optional relief boundary
HOLE_CHAMFER = 0.0008  # 45 deg lead-in on the bore at the seat face
PLATE_X = (0.0, 0.0040)  # boss plate in front of the seat face
PLATE_R = 0.0450
RIM_R = (0.04315 / 2, 0.0280)  # centring diameter M of holder and exterior fixture radius
RIM_X = (-0.0018, 0.0)  # 7005-39-4 X, reference plane at x=0
NOTCH_WIDTHS = (0.00810, 0.00825, 0.00825)  # holder Z and Z1
RIM_CHAMFER = 0.0002  # 45 deg lead-in on the rim bore (entry edge), 1.6 mm of straight mating surface
BOOT_INNER_R, BOOT_OUTER_R = 0.036, 0.039  # rear access opening, photograph-based approximation
BOOT_RIM_X = (-0.010, 0.0)  # side depth awaits sample measurement

# ---------------------------------------------------------------- wire spring clip (bail), frame H
WIRE_R = 0.0008  # 1.6 mm spring-steel wire
CLIP_HALF_W = 0.0150  # legs cross the flange 15 mm above / below the axis
HINGE_Y = 0.0260  # hinge line (z axis) 26 mm to the robot's left of the bore axis
CATCH_Y = -0.0260  # closed: the bight lies 26 mm to the right of the axis
HINGE_X = RIM_X[0] - WIRE_R - 0.0002  # straight legs clear the holder by 0.2 mm
CLIP_LEN = HINGE_Y - CATCH_Y  # 52 mm from the hinge to the bight
CLIP_MASS = 0.012  # kg (real wire ~3 g; raised for solver stability, see README)
CLIP_LIMITS_DEG = (-4.0, 170.0)
CLIP_MAX_W_DPS = 360.0  # angular speed cap of the bail (solver setting against tunnelling; see README)
CLIP_DAMPING = 2.0e-4  # N m s/rad hinge drag (computed torque)
CLIP_FRICTION_NM = 0.004  # hinge friction torque (computed, Coulomb, smoothed): the bail stays where it is left
# start angle: the bail swung open toward the robot's left (deg, 0 = closed)
CLIP_OPEN_RANGE = (95.0, 105.0)
CLIP_OPEN_NAMED = (104.0, 100.0, 96.0, 102.0, 98.0)

# ---------------------------------------------------------------- fixed sheet-metal tab and lateral wire compliance
CATCH_TRAVEL = 0.0030  # m: wire bend represented by a hinge support translating toward +y
CATCH_PRELOAD_N = 4.0
CATCH_RATE_N_M = 1600.0  # 1.6 N/mm
CATCH_MASS = 0.010
# Four loaded bends on the two wire legs over the 38–43 mm annulus.
PAD_Y = 0.0135
PAD_BOW = 0.0025  # straight legs stand 1 mm behind the flange at closure
PAD_RATE_N_M = 3500.0  # per bend: 21 N combined axial load at nominal closure

LIP_EDGE_Y = CATCH_Y + WIRE_R + 0.0004  # lip edge 0.4 mm past the bight's far side when engaged
LIP_X = (HINGE_X - WIRE_R - 0.0022, HINGE_X - WIRE_R - 0.0002)  # 2 mm thick, 0.2 mm behind the closed bight
BIGHT_Z = 0.0115  # offset operating end, seen in Suzuki bulletin 253 p. 9
LIP_Z = (BIGHT_Z - 0.0015, BIGHT_Z + 0.0015)
CATCH_OUTER_Y = CATCH_Y - 0.0040


# ---------------------------------------------------------------- placement (station frame)
H = np.array([0.400, 0.000, TOP + 0.150])  # seat centre: headlamp bulb axis 150 mm above the table
# Clamp the stand to the outer 4 mm of the boss plate. Its top must clear the whole bore.
STAND_TOP = -PLATE_R + 0.004
STAND_HEIGHT = H[2] - TOP + STAND_TOP
STAND_SIZE = (0.014, 0.035, STAND_HEIGHT)
STAND_CENTER = (0.008, 0.0, STAND_TOP - STAND_HEIGHT / 2)
HOLDER = np.array([0.215, -0.060, TOP])  # bulb holder post, bottom centre on the table
HOLDER_TOP = 0.075  # cup rim height above the table: the flange rests on it, glass down
CUP_R = (0.0196, 0.0213)  # formed shoulder clears the cup, centring disc rests on its rim
CUP_DEPTH = 0.055
HOLDER_JITTER = 0.010  # seeded: holder moved by up to +-10 mm in x and y
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
    return hinge_point() + clip_rot(phi_deg) @ np.array([0.0, -CLIP_LEN, BIGHT_Z])


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


def lug_arc(radius, width, centre, n=7):
    """Circle points within parallel lug sidewalls, giving the specified chord width."""
    half = math.degrees(math.asin(width / (2 * radius)))
    angles = np.radians(np.linspace(centre - half, centre + half, n))
    return np.c_[radius * np.cos(angles), radius * np.sin(angles)]


def bulb_pieces():
    """Separate convex pieces in the bulb frame; the three geometric lugs carry the indexing."""
    metal = [cylinder_x(COLLAR_R, *COLLAR_X), cylinder_x(PILOT_R, 0, PILOT_HEIGHT, n=96),
             cylinder_x(FLANGE_R, -FLANGE_T, 0.0, n=120)]
    for angle, width in zip(TAB_ANGLES, TAB_W):
        outline = np.vstack([lug_arc(r, width, angle) for r in (FLANGE_R - 0.001, TAB_R)])
        metal.append(np.vstack([np.c_[np.full(len(outline), x), outline] for x in (-FLANGE_T, 0)]))
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


def rim_pieces(sector=5.0):
    """Build the lands between three parallel-sided slots, including their entry chamfer."""
    out = []
    for i, left in enumerate(TAB_ANGLES):
        j = (i + 1) % len(TAB_ANGLES)
        right = TAB_ANGLES[j] + (360 if j == 0 else 0)
        count = int(math.ceil((right - left) / sector))
        levels = [(RIM_R[0] + RIM_CHAMFER, RIM_X[0]),
                  (RIM_R[0], RIM_X[0] + RIM_CHAMFER), (RIM_R[0], 0.0),
                  (RIM_R[1], RIM_X[0]), (RIM_R[1], 0.0)]
        profiles = []
        for r, x in levels:
            first = left + math.degrees(math.asin(NOTCH_WIDTHS[i] / (2*r)))
            last = right - math.degrees(math.asin(NOTCH_WIDTHS[j] / (2*r)))
            a = np.radians(np.linspace(first, last, count * 2 + 1))
            profiles.append(np.c_[np.full(len(a), x), r * np.cos(a), r * np.sin(a)])
        for k in range(count):
            out.append(np.vstack([profile[2*k:2*k+3] for profile in profiles]))
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


def clip_wire_path():
    """Approximate plan view of the formed holder wire in Suzuki bulletin 253 pp. 6 and 9.

    The two short hinge ends join the axle through the joint. Both legs turn outside
    the cap and converge at the offset operating end on the opposite side.
    """
    yz = [(HINGE_Y, 0.007), (0.019, 0.007), (0.018, CLIP_HALF_W),
          (CATCH_Y, CLIP_HALF_W), (CATCH_Y, 0.008), (-0.021, 0.006),
          (-0.020, -0.009), (-0.014, -CLIP_HALF_W), (0.018, -CLIP_HALF_W),
          (0.019, -0.007), (HINGE_Y, -0.007)]
    return np.array([[0.0, y - HINGE_Y, z] for y, z in yz])


def clip_pieces():
    path = clip_wire_path()
    return [segment(a, b, WIRE_R) for a, b in zip(path, path[1:])]


def flange_springs(clip_p, clip_r, bulb_p, bulb_r):
    """Return (contact position, force on bulb) for the bowed wire over the rear flange.

    Each virtual wire bend compresses along the clip normal. Only the portion of the bend
    between its base and free tip can touch the annular back face. Forces vanish on separation
    and when the wire is sideways to the flange. All positions share the station frame.
    """
    axis = clip_r[:, 0]
    normal = bulb_r[:, 0]
    if np.dot(axis, normal) <= 0.0:
        return []
    loads = []
    for y in (-PAD_Y, PAD_Y):
        for z in (-CLIP_HALF_W, CLIP_HALF_W):
            # The back of the bend lies one wire diameter behind its undeformed contact surface.
            base = clip_p + clip_r @ np.array([-WIRE_R, y - HINGE_Y, z])
            tip = base + (2 * WIRE_R + PAD_BOW) * axis
            local = bulb_r.T @ (tip - bulb_p)
            depth = local[0] + FLANGE_T
            if depth <= 0 or (base - bulb_p) @ normal > -FLANGE_T:
                continue
            radial = float(np.linalg.norm(local[1:]))
            supported_radius = float(np.clip(radial, PILOT_R, FLANGE_R))
            overhang = abs(radial - supported_radius)
            if overhang >= WIRE_R:
                continue
            # A rounded wire bend can still bear on the rim with its centre beyond the
            # annulus. Project the spherical nose onto the nearest supported edge;
            # both its usable reach and the axial component vanish continuously.
            axial_radius = math.sqrt(max(0., (WIRE_R-overhang)*(WIRE_R+overhang)))
            compression = depth - (WIRE_R-axial_radius)
            if compression <= 0:
                continue
            point = bulb_p + bulb_r @ np.r_[-FLANGE_T, local[1:] * (supported_radius/radial)]
            loads.append((point, PAD_RATE_N_M * compression * (axial_radius/WIRE_R) * normal))
    return loads


def clip_inertia():
    m = CLIP_MASS
    return np.array([m * CLIP_LEN**2 / 3, m * CLIP_HALF_W**2, m * CLIP_LEN**2 / 3])


def catch_pieces():
    """Fixed square-edged retaining tab in the housing frame."""
    x0, x1 = LIP_X
    z0, z1 = LIP_Z
    lip = box_pts((x0, CATCH_OUTER_Y, z0), (x1, LIP_EDGE_Y, z1))
    stem = box_pts((x0, CATCH_OUTER_Y - 0.003, z0), (PLATE_X[1], CATCH_OUTER_Y, z1))
    return [lip, stem]


def to_world_h(p_h):
    return H + np.asarray(p_h, float)
