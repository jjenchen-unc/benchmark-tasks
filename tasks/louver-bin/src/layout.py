"""Scene layout and geometry of the louver-bin task (station frame: x forward from the arm bases, y to the robot's left,
z up; metres). Engine-independent (numpy only).

A steel louvered panel (Akro-Mils 30636 class, 36 x 19 in) is screwed to a plywood back board that stands at the far
edge of the bench, its louvered face toward the robot (-x). Each louver row is modelled as a channel open at the top:
a lip that stands proud of the panel face and a floor that joins it to the face. One AkroBin (Akro-Mils 30230,
10-7/8 x 5-1/2 x 5 in) already hangs on one row, in one half of the panel. A second, identical bin stands on the bench
in front of the panel. It must be hung on the SAME louver row, in the empty half.

Bin frame B (axis-aligned with the station when the bin hangs straight): origin on the bin's back outer face, at the
bin's mid-width (y) and at the top of the back wall (z), which is also the underside of the hanger: x points from the
front of the bin toward its back (toward the panel), z up. The hanger is a short shelf on top of the back wall with a
tab that drops behind the louver's lip.
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475  # station table top (yam_arena TABLE_TOP["station"])
IN = 0.0254


def rz(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def box_corners(x0, x1, y0, y1, z0, z1):
    return np.array([[x, y, z] for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)])


# ---------------------------------------------------------------- louvered panel (Akro-Mils 30636 class)
PANEL_W = 36 * IN  # 914 mm (y)
PANEL_H = 19 * IN  # 483 mm (z)
PANEL_T = 0.0012  # 18 ga steel sheet (visual); collision slab is thicker (PANEL_COLL_T), behind the face
PANEL_COLL_T = 0.004
PANEL_RETURN = 0.019  # side returns bent back 3/4 in (stand the panel off the board)
XP_NOM = 0.575  # panel face x (nominal; per-trial jitter)
YC_NOM = 0.000  # panel centre y (nominal; per-trial jitter)
PANEL_Z0 = TOP + 0.010  # panel bottom edge (board on the bench, panel 10 mm above the bench)
N_LOUVERS = 6
LOUVER_PITCH = 3.0 * IN  # 76.2 mm row pitch (estimate; see DESIGN.md)
LOUVER_Z1 = PANEL_Z0 + 0.050  # top of the lowest louver's lip
LOUVER_MARGIN = 0.016  # louvers stop this far inside the panel's side edges
LIP_OUT = 0.010  # lip front face this far proud of the panel face
LIP_T = 0.0015  # lip thickness (steel)
LIP_H = 0.014  # lip height = channel depth
FLOOR_T = 0.0015
# channel (pocket) in panel-face coordinates: x from the face (negative = toward the robot)
POCKET_X = (-(LIP_OUT - LIP_T), 0.0)  # -8.5 mm .. 0
STEEL_FRICTION = 0.35

# ---------------------------------------------------------------- AkroBin 30230 (polypropylene)
BIN_L = 10.875 * IN  # 276 mm front-back
BIN_W = 5.5 * IN  # 140 mm
BIN_H = 5.0 * IN  # 127 mm (back wall)
BIN_FRONT_H = 0.065  # hopper front height
BIN_FLAT = 0.150  # side walls' top edge is level for 150 mm from the back, then slopes down to the front
WALL = 0.0025
BIN_MASS = 0.27  # kg (estimate for the 30230; see DESIGN.md)
BIN_FRICTION = 0.35  # polypropylene on steel / on the bench
HANGER_T = 0.003  # hanger shelf thickness (above z = 0)
HANGER_X = 0.0070  # shelf runs from the back face to x = 7 mm
TAB_X = (0.0035, 0.0070)  # tab behind the back face (3.5 mm thick)
TAB_DROP = 0.012  # tab bottom 12 mm below the shelf underside (channel is 14 mm deep)
HANGER_W = 0.120  # hanger width (y)
BACK_GAP = LIP_OUT + 0.0005  # hung: the bin's back face 0.5 mm in front of the lips below
BIN_COM = (-0.120, 0.0, -0.070)
GRIP_X = -0.100  # grasp the side-wall rims 100 mm in front of the back face (level part of the rim)
RIM_BEAD = 0.004  # rolled rim bead width on top of the side walls (outward)

# ---------------------------------------------------------------- start states
ROWS = (4, 2, 3)  # louver row of the hanging bin (= target row), from the bottom; draw order chosen so the official seed
# (1415158438) draws the middle row 3, not an extreme row (see DESIGN.md)
SIDES = (1, -1)  # +1: the hanging bin is in the panel's +y (robot's left) half; the target is the other half
JITTER = dict(panel_dx=0.025, panel_dy=0.015, ref_dy=0.015, bin_dx=0.020, bin_dy=0.020, bin_yaw=10.0)
PANEL_DX_MIN = 0.005  # the panel never sits within 5 mm (x) of nominal (open-loop replays miss the 8.5 mm channel)
OFFICIAL_SEED = 1415158438
PANEL_DX_SEP = 0.010  # no other seeded start puts the panel within 10 mm (x) of the official seed's panel (see DESIGN.md)
REL_SEP = 0.008  # ... nor the loose bin's x relative to the panel within 8 mm of the official seed's (see DESIGN.md)
BIN_START_BACK = 0.085  # loose bin: its back face this far in front of the nominal panel face (nominal)
BIN_START_OUT = 0.0  # loose bin: centre this far outboard of the empty half's centre
SETUPS = (  # named setups: (row, side, panel dx, panel dy, ref dy, bin dx, bin dy, bin yaw)
    (3, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    (2, -1, 0.007, -0.010, 0.015, 0.015, -0.015, 6.0),
    (4, 1, -0.008, 0.012, -0.015, -0.015, 0.015, -8.0),
)


def _draw(seed):
    rng = np.random.default_rng(int(seed))
    row = int(rng.choice(ROWS))
    side = int(rng.choice(SIDES))
    j = JITTER
    vals = [float(rng.uniform(-j[k], j[k])) for k in ("panel_dx", "panel_dy", "ref_dy", "bin_dx", "bin_dy", "bin_yaw")]
    if abs(vals[0]) < PANEL_DX_MIN:
        vals[0] = PANEL_DX_MIN if vals[0] >= 0 else -PANEL_DX_MIN
    return [row, side, *vals]


def setup_spec(setup):
    """Start state of a setup: int = named setup, ('seed', s) = drawn from the trial seed."""
    if isinstance(setup, (tuple, list)) and len(setup) == 2 and setup[0] == "seed":
        spec = _draw(setup[1])
        if int(setup[1]) != OFFICIAL_SEED:
            # exclusion bands around the official seed's start (open-loop replays of its action trace): the panel never
            # sits within PANEL_DX_SEP (x) of the official seed's panel, and the loose bin's x relative to the panel
            # (bin_dx - panel_dx, what a replayed carry actually lands against) never within REL_SEP of the official
            # seed's; the second band applies to every seed, so it also covers a trace mirrored to the other half
            off = _draw(OFFICIAL_SEED)
            dx_off = off[2]
            if abs(spec[2] - dx_off) < PANEL_DX_SEP:
                up = dx_off + PANEL_DX_SEP
                spec[2] = up if up <= JITTER["panel_dx"] else dx_off - PANEL_DX_SEP
            rel_off = off[5] - off[2]
            if abs((spec[5] - spec[2]) - rel_off) < REL_SEP:
                spec[5] = spec[2] + rel_off - REL_SEP
    else:
        spec = SETUPS[int(setup) % len(SETUPS)]
    row, side, pdx, pdy, rdy, bdx, bdy, yaw = spec
    return dict(row=int(row), side=int(side), panel_dx=pdx, panel_dy=pdy, ref_dy=rdy, bin_dx=bdx, bin_dy=bdy,
                bin_yaw=yaw)


def panel_origin(spec):
    """Panel reference point: (face x, centre y, bottom edge z)."""
    return np.array([XP_NOM + spec["panel_dx"], YC_NOM + spec["panel_dy"], PANEL_Z0])


def lip_top_z(row):
    return LOUVER_Z1 + (row - 1) * LOUVER_PITCH


def row_of_z(z):
    """Louver row whose lip top is nearest height z, and the offset from it (m)."""
    k = int(round((z - LOUVER_Z1) / LOUVER_PITCH)) + 1
    return k, z - lip_top_z(k)


SLOT_Y = 0.140  # bin places used in each half: centre 140 mm from the panel's centre line (the arms' READY grippers
# stand in front of the panel at y +-305 mm, z 1.01 m: a bin hung further out on the top row would touch them)


def half_centre_y(spec, side):
    """Centre y of the bin place in the panel half on `side` (+1 = +y half)."""
    return panel_origin(spec)[1] + side * SLOT_Y


def hung_pose(spec, row, y):
    """Bin frame pose hanging straight on louver `row` at lateral position y (shelf underside on the lip top)."""
    o = panel_origin(spec)
    return np.array([o[0] - BACK_GAP, y, lip_top_z(row)]), np.eye(3)


def ref_pose(spec):
    y = half_centre_y(spec, spec["side"]) + spec["ref_dy"]
    return hung_pose(spec, spec["row"], y)


def loose_pose(spec):
    """Loose bin standing on the bench below the panel's empty half (bottom on the bench), back toward the panel."""
    r = rz(spec["bin_yaw"])
    x = XP_NOM - BIN_START_BACK + spec["bin_dx"]
    y = half_centre_y(spec, -spec["side"]) - spec["side"] * BIN_START_OUT + spec["bin_dy"]
    return np.array([x, y, TOP + BIN_H + 0.0005]), r


# ---------------------------------------------------------------- bin geometry (convex pieces in the bin frame)
def bin_pieces():
    """[(name, vertices)] of the bin's convex collision pieces in the bin frame."""
    w = BIN_W / 2
    zb = -BIN_H
    zf = zb + BIN_FRONT_H
    pieces = [
        ("bottom", box_corners(-BIN_L, 0.0, -w, w, zb, zb + WALL)),
        ("back", box_corners(-WALL, 0.0, -w, w, zb, 0.0)),
        ("front", box_corners(-BIN_L, -BIN_L + WALL, -w, w, zb, zf)),
    ]
    for s, tag in ((1, "side_l"), (-1, "side_r")):
        y0, y1 = sorted((s * w, s * (w - WALL)))
        prof = [(0.0, 0.0), (-BIN_FLAT, 0.0), (-BIN_L, zf), (-BIN_L, zb), (0.0, zb)]
        v = [[x, y, z] for (x, z) in prof for y in (y0, y1)]
        pieces.append((tag, np.array(v)))
        # rolled rim bead along the level part of the side wall's top (outward)
        b0, b1 = sorted((s * w, s * (w + RIM_BEAD)))
        pieces.append((tag + "_rim", box_corners(-BIN_FLAT, -0.004, b0, b1, -0.006, 0.0)))
    hw = HANGER_W / 2
    pieces.append(("hanger", box_corners(-WALL, HANGER_X, -hw, hw, 0.0, HANGER_T)))
    pieces.append(("tab", box_corners(TAB_X[0], TAB_X[1], -hw, hw, -TAB_DROP, 0.0)))
    return pieces


def bin_points_world(pos, rot):
    pts = np.concatenate([v for _, v in bin_pieces()])
    return pts @ np.asarray(rot).T + np.asarray(pos)


# ---------------------------------------------------------------- hang state (used by the scorer)
def hang_state(spec, pos, rot):
    """Where the bin's hanger tab is relative to the louvers, from the bin pose.

    row: louver row whose lip top is nearest the shelf underside; dz: shelf underside minus that lip top (m);
    tab_x: tab centre x relative to the panel face (m; negative = in front of the face); in_channel: the tab's centre
    line is inside the channel (between the lip's back face and the panel face) and its bottom below the lip top;
    pitch_deg / roll_deg: bin tilt (nose down / sideways), level_deg: angle of the bin's z axis from vertical;
    y_lo / y_hi: lateral extent of the bin body; y_rel: bin centre y relative to the panel centre."""
    o = panel_origin(spec)
    r = np.asarray(rot, float)
    p = np.asarray(pos, float)
    hw = HANGER_W / 2
    tab_c = p + r @ np.array([sum(TAB_X) / 2, 0.0, 0.0])
    tab_ends = [p + r @ np.array([sum(TAB_X) / 2, s * hw, -TAB_DROP]) for s in (-1, 1)]
    row, dz = row_of_z(float(tab_c[2]))
    tab_x = float(tab_c[0] - o[0])
    lip_top = lip_top_z(row)
    in_channel = all(
        POCKET_X[0] < float(e[0] - o[0]) < POCKET_X[1] and float(e[2]) < lip_top - 0.004
        and float(e[2]) > lip_top - LIP_H - 0.001 for e in tab_ends
    )
    level = math.degrees(math.acos(float(np.clip(r[2, 2], -1.0, 1.0))))
    pitch = math.degrees(math.atan2(-r[2, 0], r[2, 2]))
    roll = math.degrees(math.atan2(r[2, 1], r[2, 2]))
    ys = [float((p + r @ np.array([x, s * BIN_W / 2, 0.0]))[1]) for x in (0.0, -BIN_L) for s in (-1, 1)]
    on_panel = all(abs(float(e[1] - o[1])) < PANEL_W / 2 - LOUVER_MARGIN + 0.002 for e in tab_ends)
    return dict(row=row, dz=float(dz), tab_x=tab_x, in_channel=bool(in_channel and 1 <= row <= N_LOUVERS),
                on_panel=bool(on_panel), level_deg=level, pitch_deg=pitch, roll_deg=roll, y_lo=min(ys), y_hi=max(ys),
                y_rel=float(p[1] - o[1]))
