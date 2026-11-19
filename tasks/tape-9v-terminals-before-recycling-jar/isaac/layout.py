"""Engine-independent layout of the battery-recycle bench: dimensions, materials and reset poses (pure Python).

World frame = the yam_arena station frame: x forward from the arm bases, y to the robot's left, z up, metres.
Every body frame below is given as (centre, 3x3 rotation whose columns are the body's x, y, z axes in the world).

Real objects (sizes from IEC 60086-2 and the manufacturers' data sheets, rounded to 0.1 mm):
* 9 V alkaline battery, IEC 6LR61 (e.g. Duracell Coppertop MN1604): 48.5 x 26.5 x 17.5 mm overall, 45 g. Body frame:
  x across the 26.5 mm width, y across the 17.5 mm thickness, z along the length toward the snap terminals. The case
  is 45.3 mm long; the snaps stand 3.2 mm proud of its top face, 12.7 mm (0.5 in) apart on the centre line: the
  positive stud (round, 5.8 mm across) at +x, the negative socket (hexagonal, 8.0 mm across) at -x.
* AA alkaline cell, IEC LR6 (e.g. Energizer E91): 50.5 mm long, 14.5 mm diameter, 23 g. AAA (LR03): 44.5 x 10.5 mm,
  11.5 g. Steel can under a plastic label.
* Tape: one 66 mm strip of 19 mm (3/4 in) black vinyl electrical tape (3M Scotch Super 33+, 0.18 mm thick), its last
  3 mm at each end folded under, laid adhesive side up on a silicone baking mat: the folded-under ends cling lightly to
  the mat, so the 60 mm strip lies flat where it was put. Segment frame: x along the strip, y across its width, z = the
  adhesive side. The two end segments carry the folded-under flap: their underside is adhesive too.
* Mat: silicone baking mat (release surface) cut to 120 x 120 x 0.7 mm, under the strip, fixed to the bench.
* Jar: clear PET straight-sided wide-mouth jar (about 400 ml, 89-400 neck), lid off: 96 mm outer diameter, 1.5 mm
  wall, 75 mm tall, the top 8 mm (neck) narrowing to an 80 mm clear opening; fixed to the bench.
"""
from __future__ import annotations

import math

import numpy as np

TOP = 0.7475  # station table top (yam_arena.station.TABLE_TOP)

# ------------------------------------------------------------------------------------------------ 9 V battery
NINE_V_SIZE = (0.0265, 0.0175, 0.0453)  # case (x width, y thickness, z length)
NINE_V_MASS = 0.045
SNAP_HEIGHT = 0.0032
SNAP_SPACING = 0.0127
SNAP_RADIUS = {'positive': 0.0029, 'negative': 0.0040}
SNAP_X = {'positive': +SNAP_SPACING / 2, 'negative': -SNAP_SPACING / 2}
SNAP_TOP_Z = NINE_V_SIZE[2] / 2 + SNAP_HEIGHT  # snap top faces in the battery frame
# lying on one broad face, snaps toward the robot (-x), its width along world y
NINE_V_POS = np.array([0.250, -0.090, TOP + NINE_V_SIZE[1] / 2])
NINE_V_ROT = np.column_stack([[0., 1., 0.], [0., 0., -1.], [-1., 0., 0.]])

# ------------------------------------------------------------------------------------------------ AA cells
AA_LENGTH, AA_RADIUS, AA_MASS = 0.0505, 0.00725, 0.023
AAA_LENGTH, AAA_RADIUS, AAA_MASS = 0.0445, 0.00525, 0.0115
# two loose AA cells on the bench in front of the left arm, lying along x (cylinder axis = body z)
_ALONG_X = np.column_stack([[0., 0., -1.], [0., 1., 0.], [1., 0., 0.]])
_ALONG_Y = np.column_stack([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])
AA_CELLS = {
    'aa_1': (np.array([0.220, 0.130, TOP + AA_RADIUS]), _ALONG_X),
    'aa_2': (np.array([0.240, 0.190, TOP + AA_RADIUS]), _ALONG_X),
}

# ------------------------------------------------------------------------------------------------ jar
JAR_CENTER = np.array([0.310, 0.010])
JAR_OUTER_R = 0.048
JAR_WALL = 0.0015
JAR_INNER_R = JAR_OUTER_R - JAR_WALL           # 46.5 mm
JAR_HEIGHT = 0.075
JAR_FLOOR = 0.003                               # moulded PET base
JAR_NECK_H = 0.008
JAR_MOUTH_R = 0.040                             # 80 mm clear opening inside the 89-400 neck
JAR_SEGMENTS = 24
JAR_RIM_Z = TOP + JAR_HEIGHT


def jar_contents():
    """Eight used cells already in the jar: five AA lying side by side on the floor (along x), two AA and one AAA
    nested in the grooves on top: name -> (kind, centre, rotation). A stable packing that settles within a millimetre."""
    out = {}
    z0 = TOP + JAR_FLOOR + AA_RADIUS + 0.0002
    for i, dy in enumerate((-0.030, -0.015, 0.0, 0.015, 0.030)):
        out[f'jar_aa_{i}'] = ('AA', np.array([JAR_CENTER[0], JAR_CENTER[1] + dy, z0]), _ALONG_X)
    # second layer nested in the grooves between the bottom cells (parallel, so nothing rolls)
    for i, dy in enumerate((-0.0075, 0.0075)):
        dz = math.sqrt((2 * AA_RADIUS + 0.0003) ** 2 - 0.0075 ** 2)
        out[f'jar_aa_{5 + i}'] = ('AA', np.array([JAR_CENTER[0], JAR_CENTER[1] + dy, z0 + dz]), _ALONG_X)
    dz = math.sqrt((AA_RADIUS + AAA_RADIUS + 0.0003) ** 2 - 0.0075 ** 2)
    out['jar_aaa_0'] = ('AAA', np.array([JAR_CENTER[0], JAR_CENTER[1] + 0.0225, z0 + dz]), _ALONG_X)
    return out


# ------------------------------------------------------------------------------------------------ tape strip
TAPE_LENGTH = 0.060
TAPE_WIDTH = 0.019
TAPE_THICKNESS_REAL = 0.00018
TAPE_THICKNESS = 0.0004       # collision thickness of a segment (see README fidelity note)
TAPE_SEGMENTS = 20            # 3 mm segments
TAPE_SEG_LEN = TAPE_LENGTH / TAPE_SEGMENTS
TAPE_MASS_REAL = 0.0003       # 60 x 19 x 0.18 mm PVC + adhesive, about 0.3 g
TAPE_SEG_MASS = 0.00005       # 0.05 g per segment (1.0 g strip): solver-stability floor, see README
TAPE_TWIST_LIMIT_DEG = 20.
TAPE_BEND_LIMIT_DEG = 150.
# lying flat on the bench, adhesive side up, along world y, in front of the right arm
MAT_SIZE = (0.120, 0.120, 0.0007)          # x, y, thickness (a cut-down silicone baking mat under the strip)
MAT_CENTER = np.array([0.260, -0.180, TOP + MAT_SIZE[2] / 2])
MAT_TOP = TOP + MAT_SIZE[2]
TAPE_CENTER = np.array([0.260, -0.180, MAT_TOP + TAPE_THICKNESS / 2 + 0.0001])
TAPE_ROT = np.column_stack([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]])  # segment x = world y, z (adhesive) = up

# adhesive (rubber-resin pressure-sensitive adhesive on PVC)
BOND_PRESS_N = 0.5            # N pressing one 3 mm segment's adhesive face onto a body: a deliberate press
#                               (both snaps under ~5 segments: ~3 N in all); resting weight is < 0.1 N per segment
BOND_BREAK_N = 4.0            # N per segment: the strip's peel strength (about 2.2 N/cm adhesion to steel x 1.9 cm).
#                               A peel front loads one segment at a time, so a peel needs ~4 N; in-plane (shear) loads
#                               spread over every stuck segment and need far more, as with real tape.
BOND_BREAK_TORQUE = BOND_BREAK_N * TAPE_WIDTH / 2   # N m: the peel force acting at the strip's edge
LINER_BREAK_N = 0.3           # folded-under end flap on the silicone mat (release liner): light cling
BOND_FACE_COS = 0.7           # a press counts when its contact normal is within 45 deg of the segment's face normal
FLAP_SEGMENTS = (0, TAPE_SEGMENTS - 1)
BOND_DRIFT_M = 0.0015         # a bond whose segment has moved this far from where it stuck has let go

# friction (static = dynamic)
MU = dict(battery=0.4, cell=0.4, jar=0.35, tape_back=0.5, tape_adhesive=0.9, mat=0.8)


def tape_segment_poses(center=TAPE_CENTER, rot=TAPE_ROT):
    """Reset pose (centre, rotation) of each tape segment, segment 0 at the -x end."""
    out = []
    for i in range(TAPE_SEGMENTS):
        s = (i + 0.5) * TAPE_SEG_LEN - TAPE_LENGTH / 2
        out.append((np.asarray(center, float) + rot @ np.array([s, 0., 0.]), rot.copy()))
    return out


def snap_points(name, rim_points=8):
    """Sample points on a snap's top face in the battery frame: its centre and ``rim_points`` points 0.3 mm inside
    its rim (what a person checks: no metal of the snap shows outside the tape)."""
    r = SNAP_RADIUS[name] - 0.0003
    c = np.array([SNAP_X[name], 0., SNAP_TOP_Z])
    pts = [c]
    for k in range(rim_points):
        a = 2 * math.pi * k / rim_points
        pts.append(c + np.array([r * math.cos(a), r * math.sin(a), 0.]))
    return np.array(pts)


def mat2quat_xyzw(r):
    """Rotation matrix -> quaternion (x, y, z, w)."""
    r = np.asarray(r, float)
    t = np.trace(r)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        w, x, y, z = (r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        w, x, y, z = (r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        w, x, y, z = (r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def quat_xyzw2mat(q):
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
