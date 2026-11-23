"""Vertical residential rough-in fixture, seed layout and cable measurements (metres).
Fixture X is across the opening, +Y descends from the entry and +Z leaves the open face.
The 450 mm preformed cable descends from a service loop supported at its upstream end.
Link 0 is the free end. Each link's +X points toward that end.
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475  # station table top (yam_arena TABLE_TOP["station"])


def rz(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rx(rad):
    c, s = math.cos(rad), math.sin(rad)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


# ---------------------------------------------------------------- upright stud (2x4, actual 38 x 89 mm) in fixture coordinates
STUD_T = 0.038  # x
STUD_H = 0.089  # z
STUD_L = 0.400  # y
STUD_X = 0.389  # stud centre x (nominal): box interior centre at x 0.34 (both arms reach straight down there)
STUD_Y_OFF = 0.1515  # stud centre y relative to the box centre: the offcut ends flush with the entry wall and runs on past the far end

# Allied P-181RK engineering drawing, dimensioned in mm. Local wall thicknesses
# assume a uniform rectangular shell. The back thickness and entry are estimates.
BOX_IN_X, BOX_IN_Y = 0.0549, 0.0917
SIDE_WALL = (0.0584 - BOX_IN_X) / 2
BOX_WALL = (0.0953 - BOX_IN_Y) / 2
BACK_WALL = 0.0025
BOX_IN_Z = 0.0686 - BACK_WALL
BOSS_PITCH = 0.0834
ENTRY_X = -0.016  # one active corner entry; lateral offset estimated from front view
BOX_PROUD = 0.013  # face proud of the stud edge (1/2 in drywall)
BOX_RIM_Z = TOP + STUD_H + BOX_PROUD
BOX_FLOOR_Z = BOX_RIM_Z - BOX_IN_Z  # interior floor (the box back)
BOX_Y = 0.020  # box centre y (nominal)
# cable entry in the -y end wall: opening x-range about the box centre, z-range above the floor
ENTRY_W = 0.013
ENTRY_H = 0.0145
ENTRY_Z0 = BOX_FLOOR_Z + 0.0003  # edge of the opening nearest the box back
ENTRY_Z1 = ENTRY_Z0 + ENTRY_H
# The active tongue is rooted at the back edge and points into the box toward the face.
TAB_L = 0.0155
TAB_T = 0.0025
TAB_W = 0.0125
TAB_REST_DEG = 40.0  # Rest stop of the tab joint in the authored hinge frame.
TAB_OPEN_DEG = 88.0  # hard stop
TAB_PRELOAD_DEG = 20.0  # spring target this far below the rest stop (preload)
TAB_STIFF = 0.15  # N m / rad: with the 20 deg preload, about 10 N to start lifting the tab and 15 N at full lift
# (lever 5.2 mm: the cable's top edge below the hinge); 15 N is the proposal's estimate for a push-in clamp, not a
# measured or catalog value (see the README).
TAB_FRICTION = 0.12  # smooth moulded plastic tab on the PVC sheath (min-combined)
BOX_FRICTION = 0.35
HOLD_N = 120.0  # one-way bite: the tab edge holds the sheath against pulling back out up to this force
BITE_K = 40000.0  # N/m (stiff one-way stop, computed)
BITE_C = 40.0
BITE_MIN_IN = 0.012  # the tab edge bites only once the sheath is under it (cut end past the tab tip)

# ---------------------------------------------------------------- 14/2 NM-B cable
CABLE_W = 0.0097
CABLE_T = 0.0048
CABLE_LEN = 0.450
LEAD_LINKS, LEAD_L = 20, 0.010  # last 200 mm in 10 mm links (passes the tab, bends up out of the box)
TAIL_LINKS = 10
TAIL_L = (CABLE_LEN - LEAD_LINKS * LEAD_L) / TAIL_LINKS  # 25 mm links
CABLE_LIN_MASS = 0.090  # kg/m (two 14 AWG + bare 14 AWG copper, PVC + paper, estimate)
CABLE_PATCH_R = 0.015  # torsional (contact-patch) friction radius of the sheath against a pad
CABLE_FRICTION = 1.0  # PVC sheath (averaged with the other surface: about 0.75 on the finger pads)
# bending (per joint, computed from EI over the link length; implicit articulation drives) and plastic yield
EI_FLAT = 0.12  # N m^2 (three solid 1.63 mm copper conductors bending about their own axes)
EI_EDGE = 2.0  # N m^2 (edgewise: the conductors side by side; far stiffer)
GJ = 0.05  # N m^2 torsion
M_YIELD_FLAT = 0.18  # N m: fully plastic moment of three annealed 1.63 mm conductors plus jacket (estimate): a bend beyond it stays
M_YIELD_EDGE = 0.80
M_YIELD_TWIST = 0.06


def box_corners(x0, x1, y0, y1, z0, z1):
    return np.array([[x, y, z] for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)])


def shell_parts():
    """Axis-aligned collision solids in metres, in the active tab's hinge frame."""
    left, right = -BOX_IN_X / 2 - ENTRY_X, BOX_IN_X / 2 - ENTRY_X
    low, high = BOX_FLOOR_Z - ENTRY_Z1, BOX_RIM_Z - ENTRY_Z1
    end = BOX_IN_Y
    half = ENTRY_W / 2
    x0, x1 = left - SIDE_WALL, right + SIDE_WALL
    y0, y1 = -BOX_WALL, end + BOX_WALL
    parts = dict(
        back=(x0, x1, y0, y1, low - BACK_WALL, low),
        left=(x0, left, y0, y1, low, high),
        right=(right, x1, y0, y1, low, high),
        bottom=(left, right, end, y1, low, high),
        entry_left=(left, -half, y0, 0., low, high),
        entry_right=(half, right, y0, 0., low, high),
        entry_sill=(-half, half, y0, 0., low, -ENTRY_H),
        entry_lintel=(-half, half, y0, 0., 0., high),
    )
    # Approximate solid device bosses, centred on the published mounting axes.
    for name, y in (("upper", (end-BOSS_PITCH)/2), ("lower", (end+BOSS_PITCH)/2)):
        y_limits = (0., 2*y) if name == "upper" else (2*y-end, end)
        parts[name+"_boss"] = (-ENTRY_X-.004, -ENTRY_X+.004, *y_limits, high-.012, high)
    return parts


def tab_vertices():
    """Moving collision solid, in its zero-angle joint frame (metres)."""
    radius = TAB_T / 2
    profile = [(0., 0.), (TAB_T, 0.)]
    for angle in np.linspace(0., math.pi, 7):
        profile.append((radius*(1+math.cos(angle)), -TAB_L+radius*(1-math.sin(angle))))
    return np.array([(x,y,z) for x in (-TAB_W/2,TAB_W/2) for y,z in profile])


def link_lengths():
    return [LEAD_L] * LEAD_LINKS + [TAIL_L] * TAIL_LINKS


def link_offsets():
    """Distance from the cut end to each link's centre and to each joint (between link i and i+1)."""
    ls = np.array(link_lengths())
    ends = np.cumsum(ls)
    centres = ends - ls / 2
    return centres, ends[:-1]


# ---------------------------------------------------------------- scoring geometry
TARGET_IN = 0.150  # 150 mm of cable past the clamp (6 in of free conductor)
TOL_IN = 0.020
THROUGH_MIN = 0.020  # sheath at least 20 mm past the tab before it counts as through the clamp

# ---------------------------------------------------------------- start states
# named setups: (box dx, box dy, cable end dx, cable end gap (from the entry wall, along -y), cable yaw deg)
SETUPS = (
    (-0.016, 0.000, 0.000, 0.100, 0.0),
    (-0.015, 0.020, 0.012, 0.080, 8.0),
    (0.018, -0.025, -0.010, 0.120, -6.0),
    (-0.010, 0.010, 0.015, 0.090, -10.0),
    (-0.019, -0.012, -0.015, 0.110, 10.0),
)
CABLE_YAW0 = 20.0  # the cable lies diagonally: its tail runs off toward the far right of the bench
JITTER = dict(box_dx=0.020, box_dy=0.030, end_dx=0.020, gap=(0.075, 0.120), yaw=10.0)
BOX_DX_MIN = 0.008  # exclusion band: the box never sits within 8 mm (x) of its nominal place


OFFICIAL_SEED = 1415158438  # inspect-robots' default trial seed (eval seed 0)
REPLAY_DX_MIN = 0.007  # every other seed's box is at least this far (x) from the official seed's: an open-loop replay
# of the default-seed trace cannot find the 20 mm opening (the cable clears it within about +-5 mm)


def _raw_box_dx(seed):
    rng = np.random.default_rng(int(seed))
    bdx = float(rng.uniform(-JITTER["box_dx"], JITTER["box_dx"]))
    if abs(bdx) < BOX_DX_MIN:
        bdx = BOX_DX_MIN if bdx >= 0 else -BOX_DX_MIN
    return bdx


def setup_spec(setup):
    """Start state of a setup: int = named setup, ('seed', s) = drawn from the trial seed."""
    if isinstance(setup, (tuple, list)) and len(setup) == 2 and setup[0] == "seed":
        rng = np.random.default_rng(int(setup[1]))
        j = JITTER
        bdx = float(rng.uniform(-j["box_dx"], j["box_dx"]))
        bdy = float(rng.uniform(-j["box_dy"], j["box_dy"]))
        edx = float(rng.uniform(-j["end_dx"], j["end_dx"]))
        gap = float(rng.uniform(*j["gap"]))
        yaw = float(rng.uniform(-j["yaw"], j["yaw"]))
        if int(setup[1]) == OFFICIAL_SEED:
            bdx = _raw_box_dx(OFFICIAL_SEED)  # the default start is kept exactly as first drawn
        else:
            # redraw (own generator, so the other variables keep their draws) until the box is outside both bands:
            # |dx| >= BOX_DX_MIN from nominal and >= REPLAY_DX_MIN from the default seed's box (no point masses)
            ref = _raw_box_dx(OFFICIAL_SEED)
            rr = np.random.default_rng([int(setup[1]), 7])
            while abs(bdx) < BOX_DX_MIN or abs(bdx - ref) < REPLAY_DX_MIN:
                bdx = float(rr.uniform(-JITTER["box_dx"], JITTER["box_dx"]))
        spec = (bdx, bdy, edx, gap, yaw)
    else:
        spec = SETUPS[int(setup) % len(SETUPS)]
    bdx, bdy, edx, gap, yaw = spec
    return dict(box_dx=bdx, box_dy=0.0, end_dx=edx, gap=gap, yaw=yaw)


def box_origin(spec):
    """Box reference point: (box interior centre x, box centre y, interior floor z)."""
    return np.array([STUD_X - STUD_T / 2 - SIDE_WALL - BOX_IN_X / 2 + spec["box_dx"], BOX_Y + spec["box_dy"],
                     BOX_FLOOR_Z])


def entry_wall_y(spec, inner=True):
    """y of the entry wall's inner (or outer) face."""
    y = box_origin(spec)[1] - BOX_IN_Y / 2
    return y if inner else y - BOX_WALL


def entry_centre(spec):
    o = box_origin(spec)
    return np.array([o[0]+ENTRY_X, entry_wall_y(spec, inner=False), (ENTRY_Z0 + ENTRY_Z1) / 2])


# Fixture coordinates retain X across the box, Y from the top entry into the box, and Z out
# through its open face. Gravity, cameras, robot bases and table all remain in the station frame.
FIXTURE_ROT = rz(0) @ np.array([[0., 0., -1.], [1., 0., 0.], [0., -1., 0.]])
ENTRY_HEIGHT = 0.145
_HINGE_LOCAL = np.array([STUD_X-STUD_T/2-SIDE_WALL-BOX_IN_X/2+ENTRY_X, BOX_Y-BOX_IN_Y/2, ENTRY_Z1])
_HINGE_WORLD = np.array([0.380, 0, TOP + ENTRY_HEIGHT])


def world_point(p):
    return _HINGE_WORLD + FIXTURE_ROT @ (np.asarray(p) - _HINGE_LOCAL)


def fixture_point(p):
    return _HINGE_LOCAL + FIXTURE_ROT.T @ (np.asarray(p) - _HINGE_WORLD)


def fixture_pose(p, r):
    return fixture_point(p), FIXTURE_ROT.T @ r


DRAPE_RADIUS = 0.038
DRAPE_LEAD = 0.102


def free_end_start(spec):
    return entry_centre(spec) + [spec["end_dx"] - 0.022, -spec["gap"], 0.046]


def guide_center(spec):
    # Centre of the round crosspiece underneath the midpoint of the service loop.
    poses = cable_rest(spec)
    end = free_end_start(spec)
    # Flat section rests on the horizontal top face, independent of the lead angle.
    return np.array([end[0]-DRAPE_RADIUS, poses[-1][0][1]+DRAPE_RADIUS, end[2]])


def cable_rest(spec):
    """Cable stock lies on the upper framing support with its free lead hanging into the bay."""
    point = free_end_start(spec)
    lengths = link_lengths()
    centers, _ = link_offsets()
    poses = []
    neutral = np.column_stack(([0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]))
    for length, distance in zip(lengths, centers):
        angle = np.clip((distance - DRAPE_LEAD) / DRAPE_RADIUS, 0., math.pi/2)
        rotation = rz(spec["yaw"] * 0.35*(1-angle/(math.pi/2)) - math.degrees(angle)) @ neutral
        half_link = rotation[:, 0] * (length / 2)
        poses.append((point - half_link, rotation))
        point = point - 2 * half_link
    return poses


# ---------------------------------------------------------------- cable measurements (scorer, demo)
def cable_points(links):
    """Polyline of the cable's centre line from its cut end to its tail: [(x, y, z), ...] from the link poses."""
    ls = link_lengths()
    pts = [links[0][0] + links[0][1][:, 0] * ls[0] / 2]
    for (p, r), l in zip(links, ls):
        pts.append(p - r[:, 0] * l / 2)
    return np.array(pts)


def in_window(p, spec, margin=0.004):
    """A point lies within the entry opening's outline (x-z), with a margin."""
    c = entry_centre(spec)
    return abs(p[0] - c[0]) <= ENTRY_W / 2 + margin and ENTRY_Z0 - margin <= p[2] <= ENTRY_Z1 + margin


def inserted_length(links, spec):
    """Length of cable past the entry wall's inner face, measured along the cable from where it crosses the wall
    inside the opening to its cut end (m), or None if the cable does not pass through the opening."""
    pts = cable_points(links)
    y_in = entry_wall_y(spec, inner=True)
    acc = 0.0
    for i in range(len(pts) - 1):
        a, b = pts[i], pts[i + 1]  # a nearer the cut end
        seg = float(np.linalg.norm(b - a))
        if a[1] >= y_in > b[1]:
            t = (a[1] - y_in) / max(a[1] - b[1], 1e-9)
            q = a + t * (b - a)
            if in_window(q, spec):
                return acc + t * seg
            return None
        acc += seg
    return None


def box_contains(p, spec, margin=0.0):
    o = box_origin(spec)
    return (abs(p[0] - o[0]) <= BOX_IN_X / 2 + margin and abs(p[1] - o[1]) <= BOX_IN_Y / 2 + margin
            and o[2] - margin <= p[2] <= BOX_RIM_Z + margin)
