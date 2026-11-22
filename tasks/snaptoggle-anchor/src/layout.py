"""Scene layout and geometry of the snaptoggle-anchor task (station frame: x forward from the arm bases, y to the
robot's left, z up; metres). Engine-independent (numpy only).

Wall mock-up: a vertical section of one 16 in (406 mm) on-centre stud bay, faced with 1/2 in (12.7 mm) gypsum board
toward the robot. The board's face is the plane x = XF (setup-dependent), its back face x = XF + BOARD_T. A 1/2 in
hole is pre-drilled at a pencil cross (centre (HOLE_Y, HOLE_Z), setup-dependent).

Anchor (toggle-bolt anchor of the strap-and-channel type, 1/4-20, for a 1/2 in hole). Frames:
* S (strap frame, the articulation root "stub"): origin = the channel pivot (on the channel's centroid line, at its
  mid-length), S.x along the straps pointing OUT of the wall (toward the robot when set), S.y = the pivot axis
  (across the strap pair), S.z = S.x x S.y.
* Ch (channel): origin = pivot = channel centroid; Ch.x = length, Ch.y = width (= S.y), Ch.z = height, from the open
  (bearing) side at -CH_H/2 to the web at +CH_H/2. Channel angle phi about S.y: Ch = S * Ry(phi - 90 deg); phi = 0:
  channel across the straps (set: Ch.x = S.z, bearing side toward the straps), phi = +-90 deg: folded in line with
  the straps.
* cap: prismatic along S.x; q = S.x of the flange's outer face.
* tail: the rest of the straps with the pull tab, joined to the stub at S.x = X_BRK by a breakable fixed joint.
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


def ry(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rx(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


# ---------------------------------------------------------------- wall mock-up
BOARD_T = 0.5 * IN  # 12.7 mm gypsum board (USG Sheetrock 1/2 in)
XF0 = 0.395  # nominal board face x
BOARD_Y = (-0.2225, 0.2225)  # board section width 445 mm (bay + half a stud each side)
BOARD_Z = (TOP, TOP + 0.300)  # 300 mm tall section standing on the bench
STUD = (1.5 * IN, 3.5 * IN)  # 2x4 actual 38 x 89 mm
STUD_OC = 16 * IN  # 406 mm on centre -> 368 mm clear bay
PLATE_H = 1.5 * IN  # bottom / top plate (2x4 laid flat)
HOLE_R = 0.25 * IN  # 1/2 in drilled hole
HOLE_Z0 = TOP + 0.165  # nominal hole height
HOLE_CELL = 0.040  # half-size of the square of sector pieces around the hole
HOLE_SECTORS = 48  # 7.5 deg sectors (chord sag 0.014 mm)
GYPSUM_FRICTION = 0.5
WOOD_FRICTION = 0.5
BOARD_CRUSH_N = 40.0  # robot pushing on the board face harder than this for 0.1 s: board face crushed (veto)
BOARD_CRUSH_S = 0.10

# ---------------------------------------------------------------- anchor (1/4-20, 1/2 in hole, 3/8 to 3-5/8 in walls)
CH_L = 0.0572  # channel length (~2-1/4 in)
CH_W = 0.0095  # channel width (~3/8 in)
CH_H = 0.0056  # channel height (U section)
CH_T = 0.0010  # steel thickness (cosmetic U)
CH_END_CHAMFER = 0.0010  # 1 mm x 45 deg on the channel ends' edges (stamped ends, rounded corners)
CH_MASS = 0.010  # kg, zinc-plated steel
CH_COM_X = -0.003  # centre of mass 3 mm toward one end along Ch.x: the channel hangs (swings across) by gravity
PHI_LIMIT = 100.0  # deg, pivot limits
STRAP_T = 0.0013  # each strap 1.3 mm thick (along S.y)
STRAP_H = 0.0040  # and 4 mm tall (along S.z)
STRAP_Y = 0.0030  # strap centres at S.y = +-3 mm (inside the channel's U when folded)
STRAP_LEN = 0.160  # pivot to the end of the pull tab
TAB = (0.018, 0.0032, 0.022)  # pull tab joining the straps at their ends (S.x, S.y, S.z)
STUB_X0 = 0.0035  # stub collision starts outside the channel envelope
CAP_FLANGE_R = 0.0095  # cap flange 19 mm (~3/4 in) diameter
CAP_FLANGE_T = 0.0012
CAP_COLLAR_R = 0.0058  # collar 11.6 mm, loose in the 12.7 mm hole
CAP_COLLAR_L = 0.0115
CAP_NOSE = 0.0025  # 2.5 mm x 45 deg lead-in on the collar nose
CAP_MASS = 0.002
STUB_MASS = 0.001
TAIL_MASS = 0.003
# set geometry: channel bearing face on the back face -> pivot CH_H/2 behind the back face
BEAR_X = CH_H / 2  # S.x of the channel's bearing face when set (phi = 0)
Q_SEAT = BEAR_X + BOARD_T + CAP_FLANGE_T  # flange outer face q when the cap is seated on a snug anchor
X_BRK = Q_SEAT + 0.0005  # strap break line: 0.5 mm outside the seated cap face
Q_MIN = BEAR_X + CAP_COLLAR_L + CAP_FLANGE_T + 0.0003  # collar end meets the channel
Q_MAX = STRAP_LEN - TAB[0] - 0.002
Q_START = 0.070  # cap starts 70 mm out along the straps
# ratchet (cap teeth on the straps): free toward the channel at RATCHET_FWD_N, holds backward up to RATCHET_BACK_N
RATCHET_FWD_N = 2.0
RATCHET_BACK_N = 80.0
RATCHET_K = 20000.0  # N/m anchor drive stiffness
JOINT_ARMATURE = 0.02
CAP_PLAY = 0.0015  # the cap can shift +-1.5 mm across the straps (slot clearance, strap flex)
CAP_PLAY_K = 150.0  # N/m centring stiffness of that play
# strap break at the cap face (bending side to side snaps the straps there)
BREAK_MOMENT_NM = 0.15  # bending moment at the cap face that snaps the straps (estimate)
BREAK_HOLD_S = 0.05
BREAK_CAP_M = 0.0025  # only with the cap face within 2.5 mm of the break line (its edge is the fulcrum)
STRAP_FRICTION = 0.6  # polypropylene straps on the gripper pads (combined with the pads by PhysX averaging)
CAP_FRICTION = 0.2  # smooth nylon cap (PhysX "min" combine: 0.2 against the pads and the board)
STEEL_FRICTION = 0.25  # zinc-plated steel channel on the board's paper face / the pads
PAD_PATCH_R = 0.004  # torsional contact-patch radius where the pads pinch the channel / the tab (real pads are flat)

# ---------------------------------------------------------------- staging block (anchor start)
# Wooden staging block the anchor lies on, folded, straps toward the robot: a front pad under the channel, a rear pad
# under the straps, the cap and the pull tab hanging free between / behind them. Block frame: origin on the table
# under the start pivot; x along world x.
BLOCK_PIVOT_X = 0.315  # pivot x at the start
BLOCK_Y = -0.020
PAD_TOP = 0.075  # front pad top above the table
BLOCK_PARTS = (  # (x0, x1 relative to the pivot (world x), top above the table)
    (-0.035, 0.035, PAD_TOP),  # front pad (channel)
    (-0.135, -0.080, PAD_TOP + 0.0008),  # rear pad (straps; their underside is 0.8 mm above the channel's)
    (-0.140, 0.035, PAD_TOP - 0.012),  # base
)
BLOCK_W = 0.030
BLOCK_FRICTION = 0.5

# ---------------------------------------------------------------- start variation
SEEDED_HOLE_Y = 0.040  # wall mock-up (and so the hole centre) uniform in +-40 mm of y = 0
SEEDED_XF = 0.010  # board face +-10 mm (stand-off of the mock-up on the bench)
SEEDED_ANCHOR = 0.012  # anchor on the block +-12 mm in x and y
NAMED = (
    dict(hole_y=-0.030, hole_z=0.0, xf=0.000, ax=0.0, ay=0.0),
    dict(hole_y=0.035, hole_z=0.0, xf=0.008, ax=0.008, ay=-0.006),
    dict(hole_y=0.000, hole_z=0.0, xf=-0.008, ax=-0.008, ay=0.008),
    dict(hole_y=-0.015, hole_z=0.0, xf=0.005, ax=0.004, ay=0.010),
    dict(hole_y=0.025, hole_z=0.0, xf=-0.004, ax=-0.010, ay=-0.010),
)


def setup_spec(setup):
    """Start state: dict(hole_y, hole_z (offset from HOLE_Z0), xf (offset from XF0), ax, ay (anchor offset), name).
    ``setup`` is a named index (int) or ('seed', n)."""
    if isinstance(setup, tuple) and setup[0] == "seed":
        rng = np.random.default_rng(int(setup[1]))
        hy = float(rng.uniform(-SEEDED_HOLE_Y, SEEDED_HOLE_Y))
        hz = 0.0  # the mock-up moves as a whole on the bench: the hole height is fixed
        xf = float(rng.uniform(-SEEDED_XF, SEEDED_XF))
        ax, ay = (float(v) for v in rng.uniform(-SEEDED_ANCHOR, SEEDED_ANCHOR, size=2))
        return dict(
            hole_y=hy, hole_z=hz, xf=xf, ax=ax, ay=ay, name=f"seed {int(setup[1])}"
        )
    k = int(setup) % len(NAMED)
    return dict(NAMED[k], name=f"setup {k}")


def wall(setup):
    """(XF, hole centre (3,)) of a setup: XF = board face x; the hole centre lies on the face plane."""
    sp = setup_spec(setup)
    xf = XF0 + sp["xf"]
    return xf, np.array([xf, sp["hole_y"], HOLE_Z0 + sp["hole_z"]])


# ---------------------------------------------------------------- anchor kinematics
S_SET = rz(180.0)  # strap frame when set / at the start: S.x = -x (out of the wall), S.y = -y, S.z = +z


def channel_rot_in_s(phi_deg):
    return ry(phi_deg - 90.0)


def anchor_start(setup):
    """Start pose of the anchor: pivot position, S rotation, channel phi (deg), cap q. Folded (phi = +90), lying on
    the staging block, straps toward the robot."""
    sp = setup_spec(setup)
    pivot = np.array([BLOCK_PIVOT_X + sp["ax"], BLOCK_Y + sp["ay"], TOP + PAD_TOP + CH_H / 2 + 0.0002])
    return pivot, S_SET.copy(), 90.0, Q_START


def block_pos(setup):
    sp = setup_spec(setup)
    return np.array([BLOCK_PIVOT_X + sp["ax"], BLOCK_Y + sp["ay"], TOP])


def block_pieces():
    """Block parts (vertices) in the block frame."""
    out = []
    for x0, x1, top in BLOCK_PARTS:
        out.append(box_vertices(((x0 + x1) / 2, 0.0, top / 2), (x1 - x0, BLOCK_W, top)))
    return out


def set_pivot(setup):
    """Pivot position of a snug, set anchor (channel bearing on the back face, centred on the hole)."""
    xf, hc = wall(setup)
    return hc + np.array([BOARD_T + BEAR_X, 0.0, 0.0])


# ---------------------------------------------------------------- geometry helpers
def box_vertices(center, size, rot=None):
    hx, hy, hz = (s / 2 for s in size)
    c = np.array([[sx * hx, sy * hy, sz * hz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    if rot is not None:
        c = c @ np.asarray(rot).T
    return c + np.asarray(center, float)


def cylinder_x(r, x0, x1, n=24, center=(0.0, 0.0), nose=0.0):
    """Vertices of an n-gon cylinder along x (y, z centre given); ``nose``: 45 deg chamfer at the x0 end."""
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)

    def ring(rr, x):
        return np.c_[np.full(n, x), center[0] + rr * np.cos(a), center[1] + rr * np.sin(a)]

    if nose > 0:
        return np.r_[ring(r - nose, x0), ring(r, x0 + nose), ring(r, x1)]
    return np.r_[ring(r, x0), ring(r, x1)]


def board_pieces(xf, hole_y, hole_z):
    """Convex pieces of the gypsum board (world coordinates): a square of HOLE_SECTORS cells around the hole (inner
    side on the hole's circle, outer side on the square), and four rectangles for the rest of the section."""
    x0, x1 = xf, xf + BOARD_T
    h = HOLE_CELL
    pieces = []
    n = HOLE_SECTORS
    for i in range(n):
        a0 = 2 * math.pi * i / n
        a1 = 2 * math.pi * (i + 1) / n
        inner = [(HOLE_R * math.cos(a), HOLE_R * math.sin(a)) for a in (a0, a1)]

        def to_square(a):
            c, s = math.cos(a), math.sin(a)
            t = h / max(abs(c), abs(s))
            return (t * c, t * s)

        outer = [to_square(a0)]
        # add the square's corner if the sector spans one
        for k in range(4):
            ca = math.pi / 4 + k * math.pi / 2
            if a0 < ca < a1:
                outer.append((h * math.copysign(1, math.cos(ca)), h * math.copysign(1, math.sin(ca))))
        outer.append(to_square(a1))
        poly = inner[:1] + outer + inner[1:]
        v = []
        for (u, w) in poly:
            v.append((x0, hole_y + u, hole_z + w))
            v.append((x1, hole_y + u, hole_z + w))
        pieces.append(np.array(v))
    y0, y1 = BOARD_Y
    z0, z1 = BOARD_Z
    for (ya, yb, za, zb) in (
        (y0, hole_y - h, z0, z1),
        (hole_y + h, y1, z0, z1),
        (hole_y - h, hole_y + h, z0, hole_z - h),
        (hole_y - h, hole_y + h, hole_z + h, z1),
    ):
        pieces.append(box_vertices(((x0 + x1) / 2, (ya + yb) / 2, (za + zb) / 2), (BOARD_T, yb - ya, zb - za)))
    return pieces


def frame_pieces(xf):
    """Studs, bottom and top plates behind the board (world coordinates)."""
    xb = xf + BOARD_T
    d = STUD[1]
    out = []
    z0, z1 = BOARD_Z
    for yc in (-STUD_OC / 2, STUD_OC / 2):
        out.append(box_vertices((xb + d / 2, yc, (z0 + PLATE_H + z1 - PLATE_H) / 2),
                                (d, STUD[0], z1 - z0 - 2 * PLATE_H)))
    yw = BOARD_Y[1] - BOARD_Y[0]
    out.append(box_vertices((xb + d / 2, 0.0, z0 + PLATE_H / 2), (d, yw, PLATE_H)))
    out.append(box_vertices((xb + d / 2, 0.0, z1 - PLATE_H / 2), (d, yw, PLATE_H)))
    return out


def channel_vertices():
    """Collision hull of the channel (its frame): a box with 45 deg chamfers round both end faces."""
    a, b, c, k = CH_L / 2, CH_W / 2, CH_H / 2, CH_END_CHAMFER
    pts = []
    for sx in (-1, 1):
        for (x, y, z) in ((a, b - k, c - k), (a - k, b, c)):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    pts.append((sx * x, sy * y, sz * z))
    return np.array(pts)


def channel_inertia():
    m, a, b, c = CH_MASS, CH_L, CH_W, CH_H
    return np.array([m * (b * b + c * c) / 12, m * (a * a + c * c) / 12, m * (a * a + b * b) / 12])


def box_inertia(m, size):
    a, b, c = size
    return np.array([m * (b * b + c * c) / 12, m * (a * a + c * c) / 12, m * (a * a + b * b) / 12])
