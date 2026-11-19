"""timing-belt layout: every dimension and pose of the linear-axis bench and the GT2 belt (pure numpy, no simulator).

World frame = yam_arena station frame: x forward from the line between the arm bases, y to the left of their
midpoint, z up; the table top is at ``TOP``. All lengths in metres.

Belt model: a GT2 closed loop of 280 mm pitch length (140 teeth, 2 mm pitch, 6 mm wide, 1.38 mm thick) as 70 rigid
links of 4 mm (two teeth each), hinged on the pitch line. Link frame: origin on the pitch line at the link centre,
x along the belt (direction of increasing link index), y across the width, z the outward normal (belt back side);
the teeth point to -z.
"""
from __future__ import annotations

import math

import numpy as np

TOP = 0.7475  # yam_arena station table top (station.TABLE_TOP)

# ------------------------------------------------------------------------------------------------ GT2 geometry
PITCH = 0.002              # tooth pitch
TEETH_PER_LINK = 2
LINK_LEN = PITCH * TEETH_PER_LINK
N_LINKS = 70               # 140 teeth: 280 mm pitch length
BELT_LEN = N_LINKS * LINK_LEN
BELT_WIDTH = 0.006
PLD = 0.000254             # pitch line to the belt's tooth-side land
TOOTH_H = 0.00075          # tooth height (real GT2)
BACK_T = 0.00063           # backing thickness: land to back
BELT_T = TOOTH_H + BACK_T  # 1.38 mm
BACK_Z = (-PLD, BACK_T - PLD)  # backing between these local z (visual)
# Collision shapes of one link: a rigid 4 mm link wrapped on a 20T pulley has its hinges on a circle of radius
# R_HINGE = 6.47 mm (chord 4 mm at 36 deg; the real belt bends continuously on the 6.37 mm pitch circle). The backing
# collider starts 0.04 mm inside the pitch line so that the link's land rests on the pulley's tooth tips (OD) exactly
# when its hinges are on R_HINGE; the tooth collider tips are shortened to clear the groove bottom.
BACK_COLLIDE_Z = (-0.00004, BACK_T - PLD)
TOOTH_TIP_Z = -0.00075     # tooth tip of the collider (real tip at -1.004 mm; shortened by the chord sag)
TOOTH_ROOT_W = 0.0012      # tooth width at the root (local x)
TOOTH_TIP_W = 0.0008       # tooth width at the tip
TOOTH_X = (-0.001, 0.001)  # tooth centres along the link
LINK_MASS = 0.0001         # kg per link (7 g belt; a real 280 mm GT2-6 weighs a few grams)
BELT_EI = 4e-5             # (unused since rev 2d: joint drives are not honoured, the hinges have no bending spring)
TWIST_LIMIT_DEG = 10.0
EDGE_LIMIT_DEG = 2.0
BEND_LIMIT_DEG = 70.0

# ------------------------------------------------------------------------------------------------ pulleys
# Belt reduction stage of a robot arm joint: GT2 60T pulley on the joint shaft (fixed axis, left), GT2 20T pulley on
# a NEMA 17 whose plate slides in a 10 mm slot (right) and is clamped by a lever.
TEETH = {'joint': 60, 'motor': 20}


def r_pitch(kind):
    return TEETH[kind] * PITCH / (2 * math.pi)


def r_od(kind):
    return r_pitch(kind) - PLD


def r_root(kind):
    return r_od(kind) - TOOTH_H


def r_hinge(kind):
    """Hinge radius of the wrapped rigid-link belt (4 mm chord per 2 teeth)."""
    return LINK_LEN / (2 * math.sin(math.pi / (TEETH[kind] / TEETH_PER_LINK)))


R_PITCH = r_pitch('motor')   # 6.366 mm (20T)
R_OD = r_od('motor')
R_ROOT = r_root('motor')
R_HINGE = r_hinge('motor')   # 6.472 mm
PULLEY_TOOTH_TOP_W = {'joint': 0.00085, 'motor': 0.00052}   # pulley tooth width at OD
PULLEY_TOOTH_ROOT_W = {'joint': 0.00105, 'motor': 0.00078}  # at the groove bottom
TOOTH_ZONE = 0.007                       # toothed width between the flanges
FLANGE_T = 0.001
R_FLANGE = {'joint': 0.0215, 'motor': 0.008}  # 43 mm / 16 mm flanges
HUB_R, HUB_L = 0.008, 0.007              # motor pulley set-screw hub behind the rear flange
JOINT_HUB_R, JOINT_HUB_L = 0.011, 0.008  # 60T pulley hub behind its rear flange
SHAFT_R = 0.0025
JOINT_SHAFT_R = 0.004                    # 8 mm joint shaft

BELT_X = 0.372             # belt centre plane (pulley tooth-zone centre), x
AXIS_Z = TOP + 0.150       # pulley axis height
JOINT_Y = 0.070            # joint (60T) pulley axis y
MOTOR_Y = JOINT_Y          # legacy name used by older code paths: the FIXED pulley's y
C_TAUT = 0.0991681         # centre distance at which the 70-link belt is just taut (see taut_centre_distance())
C_MIN, C_MAX = C_TAUT - 0.008, C_TAUT + 0.004  # motor-plate slot (inner / outer end): 12 mm
C_START = C_MIN            # start: motor plate slackened at the inner end of its slot (15.9 mm of loop slack)
TENSION_BAND = (6.2, 12.4) # N per span: the person check is 2-4 mm of mid-span deflection under a 1 N push (99 mm span)
# Belt tension model (the 70-link chain gives the belt's shape and contacts, but a PhysX chain of 0.1 g links cannot
# carry newtons of tension without stretching millimetres, so the belt's tension is computed from geometry):
# tension per span = PLATE_K / 2 * (centre distance - C_TAUT) past taut (2 mm of belt per mm of centre distance), and
# the belt's pull 2 T acts on the motor plate (applied as an explicit force every physics step, see beltkit).
PLATE_K = 20000.0          # N/m of plate travel past taut (2 T per metre): an effective belt stiffness of EA ~ 0.7 kN,
                           # far softer than a real glass-cord GT2-6 belt (~25 kN), so that the explicitly applied belt
                           # load is numerically stable; the 6.2-12.4 N band is 0.62-1.24 mm of plate travel past taut
# Zero of the tension model: where the SIMULATED belt is just taut. The 70-link PhysX chain (joint compliance, links
# riding in the pulley grooves, its own weight) runs taut 1.4 mm of centre distance later than the real belt's
# geometric C_TAUT (measured in this simulation: below 100.6 mm the lower span still hangs after tensioning), so the
# model is zeroed there; the real-trial check (1 N deflection) is unaffected.
C_TENSION_ZERO = 0.1006
BELT_T_CAP = 30.0          # N per span: cap of the modelled tension (far above the band; numerical safety)


def idler_y(c):
    """y of the sliding (motor) pulley axis at centre distance c."""
    return JOINT_Y - c


def motor_y(c):
    return JOINT_Y - c


def span_deflection_mm(tension_n, push_n=1.0, span_m=None):
    """Mid-span deflection of a taut span of length span_m under a transverse push (small-deflection string)."""
    span = C_TAUT if span_m is None else span_m
    return 1000 * push_n * span / (4 * tension_n)


# ------------------------------------------------------------------------------------------------ frame
RAIL_X = (0.390, 0.430)    # 2040 rail lying flat (the joint housing and the motor slide are mounted on it)
RAIL_Z = (TOP + 0.095, TOP + 0.115)
RAIL_Y = (-0.150, 0.150)
POST_Y = (-0.140, 0.140)   # 2020 posts (centres)
PLATE_X = (0.390, 0.396)   # upright plates (joint housing, motor plate) on the rail's front edge
PLATE_FRONT = PLATE_X[0]
MOTOR_BOX = dict(x=(0.396, 0.436), half=0.021)  # NEMA 17 body behind the sliding motor plate (42 x 42 x 40)
IDLER_PLATE = dict(width=0.050, z=(RAIL_Z[1], AXIS_Z + 0.026))      # sliding motor plate (upright)
IDLER_FOOT = dict(x=(0.390, 0.444), z=(RAIL_Z[1], RAIL_Z[1] + 0.005))
IDLER_BLOCK_MASS = 0.060 + 0.280   # plate + NEMA 17
JOINT_PLATE = dict(width=0.060, z=(RAIL_Z[1], AXIS_Z + 0.032))     # joint housing plate (fixed)
# clamp lever: screwed down through a tab on the outer (-y) side of the slide's foot into a T-nut in the rail; handle
# points outward (-y) when loose, a quarter turn clockwise (seen from above; right-hand thread) points it forward (-x)
LEVER = dict(x=0.417, dy=-0.033, hub_r=0.008, hub_z=(RAIL_Z[1] + 0.005, RAIL_Z[1] + 0.023), arm=0.045, arm_w=0.008,
             arm_t=0.006, mass=0.025)
LEVER_TAB = dict(x=(0.407, 0.427), y=(-0.042, -0.025))  # foot tab under the lever (y relative to the motor axis)
LEVER_LOCK_DEG = -75.0     # clamped once turned this far clockwise (seen from above)
LEVER_STOP_DEG = (-95.0, 5.0)
SLIDE_FRICTION_N = 4.0     # finger-loose T-nut drag
SLIDE_CLAMPED_N = 300.0    # thread clamping once the lever is turned
LEVER_FRICTION_NM = 0.15    # finger-loose clamping lever on its M5 thread (3.3 N at the knob turns it), as in DESIGN.md
MOTOR_DETENT_NM = 0.02
JOINT_FRICTION_NM = 0.01   # joint shaft bearings
PULLEY_MASS = {'motor': 0.012, 'joint': 0.060}
AXLE_MASS = 0.005




# ------------------------------------------------------------------------------------------------ belt shapes
def link_frames(hinges, plane_x, outward=None):
    """Link poses from N closed-loop hinge points (pitch line, y-z coordinates) in the plane x = plane_x.

    Returns (centres [N,3], rotations [N,3,3]). Link i spans hinge i -> hinge i+1. The width axis (local y) is
    +world x for a loop traversed clockwise as seen from the robot (looking along +x, y to the left).
    ``outward``: optional [N,2] outward normals in (y, z); default: away from the loop's centroid side, i.e. the
    in-plane normal rotated from the tangent.
    """
    h = np.asarray(hinges, float)
    n = len(h)
    nxt = np.roll(h, -1, axis=0)
    centres2 = (h + nxt) / 2
    t2 = nxt - h
    t2 /= np.linalg.norm(t2, axis=1, keepdims=True)
    cen, rots = [], []
    for i in range(n):
        tx = np.array([0., t2[i, 0], t2[i, 1]])
        nz = np.array([0., t2[i, 1], -t2[i, 0]])  # outward for a clockwise loop (seen from the robot)
        if outward is not None and np.dot(nz[1:], outward[i]) < 0:
            nz = -nz
        wy = np.cross(nz, tx)
        rots.append(np.column_stack([tx, wy, nz]))
        cen.append(np.array([plane_x, centres2[i, 0], centres2[i, 1]]))
    return np.array(cen), np.array(rots)


def _taut_path(c, axis_z=AXIS_Z):
    """Closed path (arc-length parametrised) of the taut belt's hinges: joint pulley (left, R_hinge 60T) and motor
    pulley (right, R_hinge 20T) joined by the two external tangents; clockwise from the robot, starting at the
    joint pulley's upper tangent point."""
    yj, ym = JOINT_Y, JOINT_Y - c
    rj, rm = r_hinge('joint'), r_hinge('motor')
    phi = math.asin((rj - rm) / c)            # tangent tilt
    # work in u = -y (from the joint toward the motor) and z: circles at u = -yj (joint) and u = -ym (motor)
    uj, um = -yj, -ym
    # tangent unit direction along the top span (from joint to motor, u increasing, dropping by phi)
    t_top = np.array([math.cos(phi), -math.sin(phi)])
    n_top = np.array([math.sin(phi), math.cos(phi)])     # outward normal of the top span
    j_top = np.array([uj, axis_z]) + rj * n_top
    m_top = np.array([um, axis_z]) + rm * n_top
    n_bot = np.array([math.sin(phi), -math.cos(phi)])
    j_bot = np.array([uj, axis_z]) + rj * n_bot
    m_bot = np.array([um, axis_z]) + rm * n_bot
    span = float(np.linalg.norm(m_top - j_top))
    a_mt, a_mb = math.atan2(n_top[1], n_top[0]), math.atan2(n_bot[1], n_bot[0])  # motor wrap: from top down via +u
    wrap_m = (a_mt - a_mb) % (2 * math.pi)
    wrap_j = 2 * math.pi - wrap_m
    segs = [span, rm * wrap_m, span, rj * wrap_j]
    total = sum(segs)

    def f(s):
        s = s % total
        if s < segs[0]:
            p = j_top + t_top * s
        elif s < segs[0] + segs[1]:
            a = a_mt - (s - segs[0]) / rm
            p = np.array([um, axis_z]) + rm * np.array([math.cos(a), math.sin(a)])
        elif s < segs[0] + segs[1] + segs[2]:
            p = m_bot + (j_bot - m_bot) / span * (s - segs[0] - segs[1])
        else:
            # joint wrap: from the lower tangent point round the joint's outer (-u) side up to the top
            a = a_mb - (s - segs[0] - segs[1] - segs[2]) / rj
            p = np.array([uj, axis_z]) + rj * np.array([math.cos(a), math.sin(a)])
        return np.array([-p[0], p[1]])        # back to (y, z)
    return f, total


def taut_centre_distance():
    """Centre distance at which 70 chords of 4 mm close exactly round both pulleys (hinges on R_hinge)."""
    lo, hi = 0.06, 0.15
    for _ in range(60):
        mid = (lo + hi) / 2
        f, total = _taut_path(mid)
        _, s_end = _walk(f, total)
        if s_end < total:   # chords end before the path closes: path too long -> smaller c
            hi = mid
        else:
            lo = mid
    return (lo + hi) / 2


def taut_hinges(c=None, axis_z=AXIS_Z, phase=0):
    """Hinge points (y, z) of the belt wrapped taut round both pulleys (polygon with hinges on R_hinge circles)."""
    c = C_TAUT if c is None else c
    f, total = _taut_path(c, axis_z)
    pts, _ = _walk(f, total)
    if phase:
        pts = np.roll(pts, int(phase), axis=0)
    return pts


def _walk(path_fn, total, n=N_LINKS, chord=LINK_LEN):
    """Hinge points along a closed parametric path (arc-length parameter s, wrapping at ``total``) at exact chord
    spacing. Returns the n points and the parameter at which an (n+1)-th point would land (== total when the n
    chords close the loop exactly)."""
    pts = [np.asarray(path_fn(0.0))]
    s = 0.0
    for k in range(n):
        lo, hi = s, s + chord * 1.6
        p0 = pts[-1]
        for _ in range(60):
            mid = (lo + hi) / 2
            if np.linalg.norm(np.asarray(path_fn(mid % total)) - p0) < chord:
                lo = mid
            else:
                hi = mid
        s = (lo + hi) / 2
        if k < n - 1:
            pts.append(np.asarray(path_fn(s % total)))
    return np.array(pts), s


def belt_start():
    """Initial link poses: the belt hung over the joint (60T) pulley in the belt plane (rest shape before settling)."""
    return link_frames(draped_hinges('joint'), BELT_X)


def belt_taut():
    """Link poses of the correctly fitted, tensioned belt (unit tests)."""
    return link_frames(taut_hinges(), BELT_X)


def hinge_points(centres, rots):
    """Both hinge points (world) of every link: ([N,3] start, [N,3] end)."""
    c = np.asarray(centres, float)
    x = np.asarray(rots, float)[:, :, 0]
    return c - x * LINK_LEN / 2, c + x * LINK_LEN / 2


def draped_hinges(kind='joint', c=None):
    """Belt draped over a pulley (the task start for 'joint'): half wrap over its top at
    the wrapped-hinge radius, two vertical strands and a semicircular bottom of the same radius; clockwise from the
    robot. Strand length solved so that 70 chords close the loop."""
    yc = JOINT_Y if kind == 'joint' else motor_y(C_START if c is None else c)
    zc = AXIS_Z
    rr = r_hinge(kind)

    def make(strand):
        top = math.pi * rr
        segs = [top, strand, math.pi * rr, strand]
        total = sum(segs)

        def f(s):
            s = s % total
            if s < top:                         # over the top, from the left (+y) side to the right (-y) side
                a = s / rr
                return np.array([yc + rr * math.cos(a), zc + rr * math.sin(a)])
            s -= top
            if s < strand:                      # right strand down
                return np.array([yc - rr, zc - s])
            s -= strand
            if s < math.pi * rr:                # bottom, right -> left
                a = math.pi + s / rr
                return np.array([yc + rr * math.cos(a), zc - strand + rr * math.sin(a)])
            s -= math.pi * rr
            return np.array([yc + rr, zc - strand + s])  # left strand up
        return f, total

    lo, hi = 0.0, 0.2
    for _ in range(50):
        mid = (lo + hi) / 2
        f, total = make(mid)
        _, s_end = _walk(f, total)
        if s_end < total:
            hi = mid
        else:
            lo = mid
    f, total = make((lo + hi) / 2)
    pts, _ = _walk(f, total)
    return pts
