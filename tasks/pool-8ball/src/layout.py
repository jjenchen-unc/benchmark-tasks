"""Scene layout and geometry of the pool-8ball task (station frame: x forward from the arm bases, y to the robot's left,
z up; metres). Engine-independent (numpy only).

A 7 ft bar-box pool table (playing field 39 x 78 in) stands with its near long rail at the robot; the YAM station's
bench (arm bases at z 0.75) is pushed against it, its top plate over the near rail. The table's long axis runs along y.
The cue ball and the 8-ball are placed per trial; one of the three far pockets (far-left corner, far side, far-right
corner) is called with a marker coin on the rail beside it. The house cue starts in a two-hook cue rest on the bench,
pointing at the table.
"""

from __future__ import annotations

import math

import numpy as np

IN = 0.0254
G = 9.81


def rz(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


# ---------------------------------------------------------------- table
BED_Z = 0.665  # cloth surface (station z); the floor is moved so the cushion tops are 30 in above it
FIELD_W, FIELD_L = 39 * IN, 78 * IN  # playing field (cushion nose to cushion nose) 0.9906 x 1.9812 m
# The robot stands at the foot (short) rail: the table's long axis runs along +x.
X0 = 0.120  # near (foot) cushion nose line
XF = X0 + FIELD_L  # far (head) cushion nose line
XS = (X0 + XF) / 2  # side pockets
YW = FIELD_W / 2  # long-rail cushion noses at y = +-YW
SLATE_T = 0.025  # 1 in slate
CUSHION_NOSE_H = 0.0363  # 63.5 % of the ball diameter above the bed
CUSHION_TOP = 0.0440
CUSHION_D = 0.051  # nose to rail (2 in)
RAIL_W = 0.110  # wooden top rail behind the cushion
RAIL_TOP = 0.0460
FLOOR_Z = BED_Z + CUSHION_TOP - 30 * IN  # cushion tops 30 in above the floor
CORNER_MOUTH = 0.117  # 4.6 in between the cushion noses (WPA corner 4.5-4.625 in)
SIDE_MOUTH = 0.130  # 5.1 in (WPA side 5.0-5.125 in)
CORNER_FACING_DEG = 142.0  # WPA facing angles
SIDE_FACING_DEG = 104.0
POCKET_HOLE_R = {"corner": 0.060, "side": 0.0635}
POCKET_DEPTH = 0.13  # pocket liner floor below the bed
CLOTH_FRICTION, CLOTH_RESTITUTION = 0.34, 0.07
CUSHION_FRICTION, CUSHION_RESTITUTION = 0.34, 0.57
RAIL_FRICTION = 0.4

# robot bench (replaces the station table: the arm bases and the camera stand sit on it)
BENCH_TOP = 0.7475
BENCH_X = (-0.78, 0.06)
BENCH_Y = (-0.62, 0.62)
BENCH_T = 0.03

# ---------------------------------------------------------------- balls (regulation 2 1/4 in, 6 oz)
BALL_R = 2.25 * IN / 2  # 28.575 mm
BALL_MASS = 0.170
BALL_FRICTION, BALL_RESTITUTION = 0.06, 0.93
# combined (PhysX averages both materials): ball-ball mu 0.06 e 0.93; ball-cloth mu 0.20 e 0.50; ball-cushion
# mu 0.20 e 0.75; ball-tip mu 0.60 e 0.75
ROLL_MU = 0.010  # rolling resistance coefficient on the cloth (deceleration 0.098 m/s^2)
SPIN_DECEL = 5.0  # rad/s^2: decay of spin about the vertical axis (cloth)
BALL_Z = BED_Z + BALL_R

# ---------------------------------------------------------------- cue: 48 in two-piece short cue (bar "tight spot" cue)
# The station stands at the foot rail, so its stroke hand works close to its own base: a 58 in cue's butt would hang
# 0.9 m behind the hand, over the robot. Bars keep 48-52 in short cues for exactly this (table near a wall).
CUE_L = 48 * IN  # 1.2192 m
CUE_MASS = 0.425  # 15 oz
CUE_BALANCE = 0.400  # balance point from the butt end
CUE_R_TIP = 0.0065  # 13 mm tip
CUE_R_JOINT = 0.0105
CUE_R_BUTT = 0.0140  # 28 mm butt
TIP_LEN = 0.012  # leather tip + ferrule collider
TIP_FRICTION, TIP_RESTITUTION = 1.14, 0.57
SHAFT_FRICTION = 0.5  # maple shaft on the rubber pads (combined 0.5)
BUTT_FRICTION = 0.5  # butt: same lacquered finish as the shaft
# cue frame: origin at the tip face, +x toward the tip (butt at x = -CUE_L), z up when lying level
CUE_COM = np.array([-(CUE_L - CUE_BALANCE), 0.0, 0.0])
CUE_INERTIA = (3.0e-5, 0.050, 0.050)


def cue_radius(s):
    """Cue radius at distance s (m) from the tip face."""
    s = float(np.clip(s, 0.0, CUE_L))
    half = CUE_L / 2
    if s <= half:
        return CUE_R_TIP + (CUE_R_JOINT - CUE_R_TIP) * s / half
    return CUE_R_JOINT + (CUE_R_BUTT - CUE_R_JOINT) * (s - half) / half


# cue rest: two felt-lined hooks on posts behind the foot rail; the cue lies level, pointing at the table and turned
# 12.5 deg to the robot's right so that its butt clears the overhead camera's post (and neither hand reaches across
# the other arm to take it): grip point (0.57 m from the tip) at (0.03, 0.0)
CUE_REST_Z = BENCH_TOP + 0.050  # cue axis height
CUE_REST_YAW_DEG = -12.5
CUE_REST_GRIP_XY = (0.03, 0.0)
CUE_REST_HOOK_S = (0.70, 1.10)  # hooks at these distances from the tip (balance point 0.82 m from the tip)

# ---------------------------------------------------------------- pockets
# corner point (cushion-nose lines' intersection) and outward direction (sx, sy)
POCKETS = {
    "NL": dict(kind="corner", corner=(X0, YW), sx=-1, sy=1),
    "NR": dict(kind="corner", corner=(X0, -YW), sx=-1, sy=-1),
    "LS": dict(kind="side", corner=(XS, YW), sx=0, sy=1),
    "RS": dict(kind="side", corner=(XS, -YW), sx=0, sy=-1),
    "FL": dict(kind="corner", corner=(XF, YW), sx=1, sy=1),
    "FR": dict(kind="corner", corner=(XF, -YW), sx=1, sy=-1),
}
CALLABLE = ("LS", "RS")
POCKET_NAMES = {
    "LS": "left side",
    "RS": "right side",
    "NL": "near-left corner",
    "NR": "near-right corner",
    "FL": "far-left corner",
    "FR": "far-right corner",
}


def _out(name):
    p = POCKETS[name]
    return unit([p["sx"], p["sy"]])


def pocket_hole(name):
    """(centre xy, radius) of the pocket's hole in the slate."""
    p = POCKETS[name]
    c = np.array(p["corner"], float)
    if p["kind"] == "corner":
        return c + 0.012 * _out(name), POCKET_HOLE_R["corner"]
    return c + 0.025 * _out(name), POCKET_HOLE_R["side"]


def pocket_aim(name):
    """Aim point for the object ball's centre (centre of the pocket mouth, a little inside the throat)."""
    p = POCKETS[name]
    c = np.array(p["corner"], float)
    if p["kind"] == "corner":
        return c - 0.010 * _out(name)
    return c + 0.010 * _out(name)


def pocket_jaws(name):
    """The two cushion-nose jaw points of a pocket (xy)."""
    p = POCKETS[name]
    c = np.array(p["corner"], float)
    if p["kind"] == "corner":
        a = CORNER_MOUTH / math.sqrt(2)
        return c - a * np.array([0.0, p["sy"]]), c - a * np.array([p["sx"], 0.0])
    h = SIDE_MOUTH / 2
    return c - h * np.array([1.0, 0.0]), c + h * np.array([1.0, 0.0])


def in_pocket(name, pos):
    """Ball centre has dropped below the bed inside this pocket's liner."""
    c, r = pocket_hole(name)
    return pos[2] < BED_Z - 0.5 * BALL_R and np.linalg.norm(np.asarray(pos[:2]) - c) < r + 0.03


def which_pocket(pos):
    for n in POCKETS:
        if in_pocket(n, pos):
            return n
    return None


def on_table(pos):
    return X0 - 0.002 < pos[0] < XF + 0.002 and -YW - 0.002 < pos[1] < YW + 0.002 and abs(pos[2] - BALL_Z) < 0.01


def jaw_distance(name, pos):
    """Distance (m) from the ball centre to the nearer jaw point of the pocket (horizontal)."""
    return min(float(np.linalg.norm(np.asarray(pos[:2]) - j)) for j in pocket_jaws(name))


# ---------------------------------------------------------------- cushions (segments between pockets)
def cushion_segments():
    """Six cushion segments (a, b = nose-line jaw points, n = outward normal, fa / fb = facing angles)."""
    ac = CORNER_MOUTH / math.sqrt(2)
    hs = SIDE_MOUTH / 2
    C, S = CORNER_FACING_DEG, SIDE_FACING_DEG
    segs = [
        dict(a=(X0, -YW + ac), b=(X0, YW - ac), n=(-1, 0), fa=C, fb=C),  # foot rail (at the robot)
        dict(a=(XF, -YW + ac), b=(XF, YW - ac), n=(1, 0), fa=C, fb=C),  # head rail
        dict(a=(X0 + ac, YW), b=(XS - hs, YW), n=(0, 1), fa=C, fb=S),
        dict(a=(XS + hs, YW), b=(XF - ac, YW), n=(0, 1), fa=S, fb=C),
        dict(a=(X0 + ac, -YW), b=(XS - hs, -YW), n=(0, -1), fa=C, fb=S),
        dict(a=(XS + hs, -YW), b=(XF - ac, -YW), n=(0, -1), fa=S, fb=C),
    ]
    for s in segs:
        for k in ("a", "b", "n"):
            s[k] = np.asarray(s[k], float)
    return segs


def cushion_prism(seg):
    """Convex hull points of one cushion segment (K-66-like section, jaw facings cut at the WPA angles)."""
    a, b, n = seg["a"], seg["b"], seg["n"]
    t = unit(b - a)
    # section: (u = distance behind the nose line, z above bed)
    section = [(0.012, 0.0), (0.0, CUSHION_NOSE_H), (0.004, CUSHION_TOP), (CUSHION_D, CUSHION_TOP), (CUSHION_D, 0.0)]
    pts = []
    for u, z in section:
        # facing: at depth u the end moves toward the pocket by u / tan(180 - facing)
        da = u / math.tan(math.radians(180.0 - seg["fa"]))
        db = u / math.tan(math.radians(180.0 - seg["fb"]))
        for end, sgn, d in ((a, -1.0, da), (b, 1.0, db)):
            xy = end + n * u + t * sgn * d
            pts.append([xy[0], xy[1], BED_Z + z])
    return np.array(pts)


def rail_boxes():
    """Wooden top rails as (centre, size) boxes; gaps at the pockets."""
    out = []
    u0, u1 = CUSHION_D, CUSHION_D + RAIL_W
    hz = RAIL_TOP + 0.10
    zc = BED_Z + RAIL_TOP - hz / 2
    gap = 0.075
    for x, sgn in ((X0, -1), (XF, 1)):
        out.append((np.array([x + sgn * (u0 + u1) / 2, 0.0, zc]), np.array([RAIL_W, 2 * (YW - gap), hz])))
    for y, sgn in ((YW, 1), (-YW, -1)):
        yc = y + sgn * (u0 + u1) / 2
        for xa, xb in ((X0 + gap, XS - gap), (XS + gap, XF - gap)):
            out.append((np.array([(xa + xb) / 2, yc, zc]), np.array([xb - xa, RAIL_W, hz])))
    return out


def bed_cells(min_cell=0.006):
    """Convex 2-D cells (x0, y0, x1, y1) covering the slate (to the cushion backs) minus the six pocket holes:
    one central box where no hole reaches, quadtree cells in the border strips."""
    ex = CUSHION_D + 0.02
    xa, xb, ya, yb = X0 - ex, XF + ex, -YW - ex, YW + ex
    holes = [pocket_hole(n) for n in POCKETS]
    m = 0.10  # border strip width
    cells = []
    # interior split at the side pockets' x so the side holes only touch border strips
    interior = (xa + m, ya + m, xb - m, yb - m)
    cells.append(interior)
    strips = [
        (xa, ya, xb, ya + m),
        (xa, yb - m, xb, yb),
        (xa, ya + m, xa + m, yb - m),
        (xb - m, ya + m, xb, yb - m),
    ]

    def classify(c):
        x0, y0, x1, y1 = c
        state = "out"
        for hc, r in holes:
            nx, ny = np.clip(hc[0], x0, x1), np.clip(hc[1], y0, y1)
            dmin = math.hypot(nx - hc[0], ny - hc[1])
            dmax = max(math.hypot(px - hc[0], py - hc[1]) for px in (x0, x1) for py in (y0, y1))
            if dmax <= r:
                return "in"
            if dmin < r:
                state = "cut"
        return state

    def rec(c):
        st = classify(c)
        if st == "out":
            cells.append(c)
            return
        if st == "in":
            return
        x0, y0, x1, y1 = c
        if max(x1 - x0, y1 - y0) <= min_cell:
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            if all(math.hypot(cx - hc[0], cy - hc[1]) > r for hc, r in holes):
                cells.append(c)
            return
        if x1 - x0 >= y1 - y0:
            xm = (x0 + x1) / 2
            rec((x0, y0, xm, y1))
            rec((xm, y0, x1, y1))
        else:
            ym = (y0 + y1) / 2
            rec((x0, y0, x1, ym))
            rec((x0, ym, x1, y1))

    for s in strips:
        x0, y0, x1, y1 = s
        nx = max(1, int(round((x1 - x0) / 0.2)))
        ny = max(1, int(round((y1 - y0) / 0.2)))
        for i in range(nx):
            for j in range(ny):
                rec(
                    (
                        x0 + (x1 - x0) * i / nx,
                        y0 + (y1 - y0) * j / ny,
                        x0 + (x1 - x0) * (i + 1) / nx,
                        y0 + (y1 - y0) * (j + 1) / ny,
                    )
                )
    return cells


# ---------------------------------------------------------------- shot geometry
def ghost_ball(obj, pocket):
    """Cue-ball centre at impact for a centre-to-pocket shot, and the object ball's direction."""
    u = unit(pocket_aim(pocket) - np.asarray(obj[:2]))
    return np.asarray(obj[:2]) - 2 * BALL_R * u, u


def shot(cue, obj, pocket):
    gb, u = ghost_ball(obj, pocket)
    d = unit(gb - np.asarray(cue[:2]))
    cut = math.degrees(math.acos(float(np.clip(d @ u, -1, 1))))
    return dict(
        ghost=gb,
        d=d,
        u=u,
        cut_deg=cut,
        phi_deg=math.degrees(math.atan2(d[1], d[0])),
        cue_travel=float(np.linalg.norm(gb - np.asarray(cue[:2]))),
        obj_travel=float(np.linalg.norm(pocket_aim(pocket) - np.asarray(obj[:2]))),
    )


SIDE_MIN_ANGLE_DEG = 35.0  # object-ball path to a side pocket: at least this steep to the rail


def side_entry_ok(obj, pocket):
    """The object ball's centre line to the aim point crosses the cushion-contact line (R inside the nose) between the
    jaws (with R/2 margin) at 35 deg or more to the rail: shallower side-pocket shots hit the jaw in reality too."""
    p = POCKETS[pocket]
    if p["kind"] != "side":
        return True
    o = np.asarray(obj[:2], float)
    a = pocket_aim(pocket)
    u = unit(a - o)
    sy = p["sy"]
    if abs(math.degrees(math.asin(min(1.0, abs(u[1]))))) < SIDE_MIN_ANGLE_DEG or u[1] * sy <= 0:
        return False
    yc = sy * (YW - BALL_R)
    t = (yc - o[1]) / u[1]
    xc = o[0] + t * u[0]
    return abs(xc - XS) <= SIDE_MOUTH / 2 - BALL_R / 2


def full_ball_enters(cue, obj, pocket, margin=0.0):
    """Shortcut check: would a full-ball hit (cue ball driven straight at the 8-ball's centre, ignoring the cut) send
    the 8-ball's centre line into the pocket mouth? Starts where it would are rejected."""
    p = POCKETS[pocket]
    o = np.asarray(obj[:2], float)
    u = unit(o - np.asarray(cue[:2], float))
    sy = p["sy"]
    if u[1] * sy <= 0:
        return False
    yc = sy * (YW - BALL_R)
    xc = o[0] + (yc - o[1]) / u[1] * u[0]
    return abs(xc - XS) <= SIDE_MOUTH / 2 + margin


MAST_XY = np.array([-0.2365, -0.009])
ARM_BASES = {"left": np.array([0.0, 0.305]), "right": np.array([0.0, -0.305])}
BRIDGE_LEN = 0.280  # cue tip to the bridge hand
TIP_GAP = 0.025  # tip behind the cue ball at address
GRIP_LEN = 0.290  # bridge hand to stroke hand (grip 0.57 m from the tip)
GRIP_MIN_X = 0.10  # stroke hand at least 10 cm in front of the arm bases (a tightly folded arm hits its own base)


def cue_line_ok(cue, d):
    """The cue (tip at the cue ball, butt over the bench) passes between the arm bases and clears the camera mast."""
    c = np.asarray(cue[:2])
    if abs(d[0]) < 0.5:
        return False
    y0 = c[1] - c[0] * d[1] / d[0]
    ym = c[1] + (MAST_XY[0] - c[0]) * d[1] / d[0]
    # the butt passes the camera post on the side of the rest the cue starts in (a shot to the right starts from the
    # left rest and vice versa): turning the cue across would sweep the butt through the post
    side = 1.0 if d[1] < 0 else -1.0
    return abs(y0) < 0.15 and side * (ym - MAST_XY[1]) > 0.05


def roles(d):
    """(bridge arm, stroke arm): shooting to the left the left hand bridges and the right hand strokes; to the right,
    mirrored (the arms never cross)."""
    return ("left", "right") if d[1] >= 0 else ("right", "left")


def address_points(cue, d):
    c = np.asarray(cue[:2])
    tip = c - (BALL_R + TIP_GAP) * d
    bridge = tip - BRIDGE_LEN * d
    grip = bridge - GRIP_LEN * d
    return tip, bridge, grip


# ---------------------------------------------------------------- start states
DEFAULT_SEED = 1415158438
SEED_LIMITS = dict(
    obj_x=(0.75, 1.30),
    sep=(0.15, 0.45),  # ghost-ball travel of the cue ball
    cut=(6.0, 30.0),
    phi=(-30.0, 30.0),
    margin=BALL_R + 0.035,  # ball centres at least this far inside the cushion noses
)
# named setups (unseeded resets cycle through them): (pocket, cue xy, object xy)
NAMED = [
    ("LS", (0.742, 0.182), (0.966, 0.312)),  # seed 42's layout (cut 29 deg, shooting left)
    ("RS", (0.695, -0.237), (0.987, -0.369)),  # seed 0's layout (cut 28 deg, shooting right)
    ("RS", (0.722, -0.178), (0.95, -0.303)),  # default seed's layout (cut 29 deg)
]


def _ok(cue, obj, pocket):
    lim = SEED_LIMITS
    m = lim["margin"]
    for p in (cue, obj):
        if not (X0 + m < p[0] < XF - m and -YW + m < p[1] < YW - m):
            return None
        for n in POCKETS:
            if np.linalg.norm(np.asarray(p) - pocket_hole(n)[0]) < 0.16:
                return None
    s = shot(cue, obj, pocket)
    if not (lim["sep"][0] <= s["cue_travel"] <= lim["sep"][1]):
        return None
    if not (lim["cut"][0] <= s["cut_deg"] <= lim["cut"][1]):
        return None
    if not (lim["phi"][0] <= s["phi_deg"] <= lim["phi"][1]):
        return None
    if not cue_line_ok(cue, s["d"]):
        return None
    tip, bridge, grip = address_points(cue, s["d"])
    br, gr = roles(s["d"])
    if np.linalg.norm(bridge - ARM_BASES[br]) > 0.62 or np.linalg.norm(grip - ARM_BASES[gr]) > 0.55:
        return None
    if grip[0] < GRIP_MIN_X:
        return None
    # the stroke hand stays on its own side of the centre line (no reaching across under the bridge arm)
    if (grip[1] if gr == "left" else -grip[1]) < -0.06:
        return None
    if s["obj_travel"] > 0.75:
        return None
    if not side_entry_ok(obj, pocket):
        return None
    if full_ball_enters(cue, obj, pocket):
        return None
    return s


def _canned_hits(cue, obj, canned):
    """Would the default seed's canned stroke (tip moving along its line) strike this cue ball and drive it into the
    object ball?"""
    if canned is None:
        return False
    c0, d0 = canned
    c, o = np.asarray(cue[:2]), np.asarray(obj[:2])
    rel = c - c0
    lateral = abs(rel[0] * d0[1] - rel[1] * d0[0])
    if lateral > BALL_R + CUE_R_TIP + 0.01 or rel @ d0 < -0.15:
        return False
    rel8 = o - c
    lat8 = abs(rel8[0] * d0[1] - rel8[1] * d0[0])
    return rel8 @ d0 > 0 and lat8 < 2 * BALL_R + 0.01


def _default_canned():
    sp = setup_spec(("seed", DEFAULT_SEED))
    return np.asarray(sp["cue"][:2]), np.asarray(sp["shot"]["d"])


def _draw(seed, canned):
    rng = np.random.default_rng(int(seed))
    lim = SEED_LIMITS
    pocket = CALLABLE[int(rng.integers(0, len(CALLABLE)))]
    for _ in range(100000):
        # object ball, cut angle (either side) and cue-ball travel; the cue ball follows from the ghost ball
        obj = np.array([rng.uniform(*lim["obj_x"]), rng.uniform(-YW + 0.08, YW - 0.08)])
        cut = rng.uniform(*lim["cut"]) * (1 if rng.uniform() < 0.5 else -1)
        sep = rng.uniform(*lim["sep"])
        gb, u = ghost_ball(obj, pocket)
        d = rz(cut)[:2, :2] @ u
        cue = gb - sep * d
        s = _ok(cue, obj, pocket)
        if s is None:
            continue
        if _canned_hits(cue, obj, canned):
            continue
        return dict(pocket=pocket, cue=cue, obj=obj, shot=s, seed=int(seed))
    raise RuntimeError(f"no valid start for seed {seed}")


_SPEC_CACHE: dict = {}


def setup_spec(setup):
    """Cached ``_setup_spec`` (the seeded draw is a rejection sampler; the runtime asks for the spec every step)."""
    key = tuple(setup) if isinstance(setup, (tuple, list)) else int(setup)
    if key not in _SPEC_CACHE:
        _SPEC_CACHE[key] = _setup_spec(setup)
    return _SPEC_CACHE[key]


def _setup_spec(setup):
    """Start state: {'pocket', 'cue' (xy), 'obj' (xy), 'shot', 'name'}. ``setup`` = named index (int) or ('seed', n)."""
    if isinstance(setup, (tuple, list)) and setup[0] == "seed":
        seed = int(setup[1])
        canned = None if seed == DEFAULT_SEED else _default_canned()
        sp = _draw(seed, canned)
        sp["name"] = f"seed {seed}"
        return sp
    k = int(setup) % len(NAMED)
    pocket, cue, obj = NAMED[k]
    s = shot(cue, obj, pocket)
    return dict(pocket=pocket, cue=np.array(cue), obj=np.array(obj), shot=s, name=f"setup {k}", seed=None)


def coin_pos(pocket):
    """Marker coin on the rail top beside the called pocket."""
    p = POCKETS[pocket]
    cx, cy = p["corner"]
    if p["kind"] == "corner":
        return np.array([cx - p["sx"] * 0.16, cy + p["sy"] * (CUSHION_D + 0.035), BED_Z + RAIL_TOP])
    # on the rail top just short of the pocket (robot side), centred on the rail
    return np.array([cx - 0.13, cy + p["sy"] * (CUSHION_D + 0.035), BED_Z + RAIL_TOP])


def rest_side(pocket):
    """Which of the two mirrored cue rests holds the cue: the left one for the right side pocket and vice versa."""
    return "left" if POCKETS[pocket]["sy"] < 0 else "right"


def cue_rest_pose(side="left"):
    """Cue frame (pos, rot) in a rest: 'left' turned 12.5 deg to the right (butt left of the camera post), 'right'
    mirrored."""
    sg = 1.0 if side == "left" else -1.0
    yaw = sg * CUE_REST_YAW_DEG
    a = math.radians(yaw)
    d = np.array([math.cos(a), math.sin(a)])
    g = np.array([CUE_REST_GRIP_XY[0], sg * CUE_REST_GRIP_XY[1]])
    tip = g + (BRIDGE_LEN + GRIP_LEN) * d
    return np.array([tip[0], tip[1], CUE_REST_Z]), rz(yaw)


def rest_hook_xy(s, side="left"):
    p, r = cue_rest_pose(side)
    return p[:2] - s * r[:2, 0]
