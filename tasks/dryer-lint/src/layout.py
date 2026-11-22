"""Scene layout and geometry of the dryer-lint task (station frame: x forward from the arm bases, y to the robot's
left, z up; metres). Engine-independent (numpy only).

Dryer frame D: origin on the dryer's front face at the cabinet-top height, on the cabinet's centre line; x into the
dryer (station x), y to the robot's left, z up. The dryer stands at the far end of the robot's table with its front
toward the robot; only its cabinet top region is modelled in detail (top panel with the lint-screen slot and its finger
recesses, the screen housing below the slot, the rear console) plus a Start push button on the front panel.

Lint screen: dimensions of the 53-0918 replacement lint screen as sold (14-3/4 x 5-3/4 in; the frame is 3/8 in thick at
its left end and 5/8 in at its right end, so it fits its slot one way only). Screen frame S: origin at the middle of
the screen's long axis on its flat back face, at the underside of its top grip; x = the screen's face normal pointing
out of its BACK face (toward the dryer's rear when installed), y along its length (thin end at +y), z up.
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475  # station table top (yam_arena TABLE_TOP["station"])
FLOOR = -0.0125  # station floor


def rz(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def ry(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


IN = 0.0254

# ---------------------------------------------------------------- dryer cabinet (representative 27 in dryer)
D0 = np.array([0.22, 0.0, 0.80])  # nominal dryer frame origin (front face, centre line, cabinet top: 0.81 m above the floor)
CAB_W = 0.686  # 27 in wide
CAB_D = 0.70  # front face to back
TOP_T = 0.020  # top panel thickness
CONSOLE = (0.10, 0.18)  # rear console: depth, height above the top
SLOT_X = 0.150  # slot back wall 150 mm behind the front face

# ---------------------------------------------------------------- lint screen (53-0918 dimensions)
SCREEN_L = 14.75 * IN  # 374.7 mm long
SCREEN_H = 5.75 * IN  # 146.1 mm high including the top grip
T_THIN = 0.375 * IN  # 9.5 mm at the left (+y) end
T_THICK = 0.625 * IN  # 15.9 mm at the right (-y) end
GRIP_H = 0.016  # top grip bar: height
GRIP_T = 0.034  # thickness (x): rests on the slot lips
GRIP_EAR = 0.025  # grip ears: the grip bar runs on past each frame end, over the finger recess
FRAME_BORDER = 0.008  # frame border around the mesh window
SCREEN_MASS = 0.105  # kg (listing weight 3.9 oz incl. packaging)
SCREEN_FRICTION = 0.3

# slot in the top panel (a straight back wall, the front wall following the taper)
SLOT_BACK_GAP = 0.0010  # gap behind the screen's back face
SLOT_FRONT_GAP = 0.0050  # gap in front of its front face (room for the lint mat)
SLOT_END_GAP = 0.0030  # at each end
SLOT_L = SCREEN_L + 2 * SLOT_END_GAP
POCKET_L = 0.065  # finger recess beyond each slot end
POCKET_W = 0.085  # across the slot (x)
POCKET_DEPTH = 0.030
HOUSING_DEPTH = SCREEN_H - GRIP_H + 0.012  # housing floor 12 mm below the seated screen's bottom

# ---------------------------------------------------------------- lint mat (rigid stand-in)
MAT_L = 0.345
MAT_H = 0.118
MAT_T = 0.003
MAT_MASS = 0.002
MAT_FRICTION = 1.0  # outer (fibre) face, PhysX combine "max": the pads grip the lint
MAT_BACK_FRICTION = 0.1  # face on the mesh, combine "min": torn lint slides off the mesh freely
MAT_BREAK_N = 1.5  # breakaway force of the mat from the mesh
MAT_Z0 = 0.006  # mat bottom edge above the screen bottom edge

# ---------------------------------------------------------------- Start button (front panel) and bin
BUTTON = np.array([0.045, -0.255, 0.0])  # dryer frame: on the cabinet top near its front-right corner (pressed down)
BUTTON_R = 0.012
BUTTON_PROUD = 0.006
BUTTON_PRESS_N = 4.0
BUTTON_HOLD_S = 0.5
BIN_C = np.array([-0.093, 0.0])  # bin centre relative to the dryer front face (x, y), on the table
BIN_IN = (0.180, 0.400)  # inside opening x, y
BIN_H = 0.040
BIN_WALL = 0.004

HARD_FRICTION = 0.4

# ---------------------------------------------------------------- start variation
SEED_DX_M = (0.006, 0.015)  # dryer x offset: magnitude uniform in 6..15 mm, either sign (never near nominal: the slot
#                             clearance is -1 / +4.5 mm, so a fixed trajectory cannot set the screen back in)
SEED_DY_M = 0.030  # dryer y offset uniform in +-30 mm
SEED_LINT_P = 1.0  # every trial has a lint mat (a do-nothing or start-only policy scores 0)
NAMED = (  # named check setups: (dx, dy, lint)
    (-0.010, 0.020, True),
    (0.012, -0.025, True),
    (-0.008, -0.012, True),
    (0.007, 0.028, True),
    (0.014, 0.004, True),
)


def thickness(y_s):
    """Screen frame thickness at position y_s along its length (S frame; thin end at +y)."""
    u = (y_s + SCREEN_L / 2) / SCREEN_L  # 0 at the thick (-y) end
    return T_THICK + (T_THIN - T_THICK) * float(np.clip(u, 0.0, 1.0))


def setup_spec(setup):
    """Start state: {'dx', 'dy' (m), 'lint' (bool), 'name'}. ``setup`` is a named index (int) or ('seed', n)."""
    if isinstance(setup, (tuple, list)) and setup[0] == "seed":
        rng = np.random.default_rng(int(setup[1]))
        u = float(rng.uniform(-1.0, 1.0))
        dx = math.copysign(SEED_DX_M[0] + abs(u) * (SEED_DX_M[1] - SEED_DX_M[0]), u)
        dy = float(rng.uniform(-SEED_DY_M, SEED_DY_M))
        lint = bool(rng.uniform() < SEED_LINT_P)
        return dict(dx=dx, dy=dy, lint=lint, name=f"seed {int(setup[1])}")
    k = int(setup) % len(NAMED)
    dx, dy, lint = NAMED[k]
    return dict(dx=dx, dy=dy, lint=lint, name=f"setup {k}")


def dryer_origin(setup):
    sp = setup_spec(setup)
    return D0 + np.array([sp["dx"], sp["dy"], 0.0])


def slot_back_x(dorig):
    """Station x of the slot's back wall (= the installed screen's back face + SLOT_BACK_GAP)."""
    return float(dorig[0] + SLOT_X)


def screen_seated_pose(dorig):
    """(pos, rot) of the screen frame S when fully seated (grip resting on the top), station frame."""
    p = np.array([slot_back_x(dorig) - SLOT_BACK_GAP, dorig[1], dorig[2]])
    return p, np.eye(3)


def button_center(dorig):
    return np.asarray(dorig, float) + BUTTON


def bin_center(dorig):
    return np.array([dorig[0] + BIN_C[0], dorig[1] + BIN_C[1], TOP])


# ---------------------------------------------------------------- convex pieces (all in their body frames)
def box_v(c, size):
    hx, hy, hz = (s / 2 for s in size)
    return np.array(
        [[c[0] + sx * hx, c[1] + sy * hy, c[2] + sz * hz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    )


def screen_pieces():
    """Screen (S frame): {'grip': verts, 'frame_*': verts, 'mesh': verts}. The body is a tapered slab: back face at
    x = 0, front face at x = -thickness(y). The mesh panel is flush with the frame's front face (the lint lies on it)."""
    y0, y1 = -SCREEN_L / 2, SCREEN_L / 2
    zb = -(SCREEN_H - GRIP_H)  # bottom edge
    t0, t1 = thickness(y0), thickness(y1)
    out = {}
    # body split in 4 along y (each a convex tapered prism)
    ys = np.linspace(y0, y1, 5)
    for i in range(4):
        a, b = ys[i], ys[i + 1]
        ta, tb = thickness(a), thickness(b)
        out[f"body_{i}"] = np.array(
            [[0.0, a, zb], [0.0, a, 0.0], [0.0, b, zb], [0.0, b, 0.0],
             [-ta, a, zb], [-ta, a, 0.0], [-tb, b, zb], [-tb, b, 0.0]]
        )
    tm = 0.5 * (t0 + t1)
    out["grip"] = box_v((-tm / 2, 0.0, GRIP_H / 2), (GRIP_T, SCREEN_L + 2 * GRIP_EAR, GRIP_H))
    return out


def mat_pose_in_screen():
    """(pos, rot) of the lint mat's frame (centre of its back face, on the screen's front face) in S."""
    zc = -(SCREEN_H - GRIP_H) + MAT_Z0 + MAT_H / 2
    t = thickness(0.0)
    ang = math.degrees(math.atan2(T_THICK - T_THIN, SCREEN_L))  # front face slant (rotation about z)
    return np.array([-t, 0.0, zc]), rz(-ang)


def mat_pieces():
    """Mat (its frame): a thin plate extending toward -x from its back face, as two layers: (back, front)."""
    return [box_v((-MAT_T / 4, 0.0, 0.0), (MAT_T / 2, MAT_L, MAT_H)),
            box_v((-3 * MAT_T / 4, 0.0, 0.0), (MAT_T / 2, MAT_L, MAT_H))]


def mat_inertia():
    """Principal inertia of the mat plate in its frame (x through its thickness)."""
    m = MAT_MASS
    return np.array([m * (MAT_L**2 + MAT_H**2) / 12, m * (MAT_T**2 + MAT_H**2) / 12, m * (MAT_T**2 + MAT_L**2) / 12])


def screen_mass_props():
    m = SCREEN_MASS
    t = 0.5 * (T_THIN + T_THICK)
    h, l = SCREEN_H, SCREEN_L
    ixx = m * (l * l + h * h) / 12
    iyy = m * (t * t + h * h) / 12
    izz = m * (t * t + l * l) / 12
    com = (-t / 2, 0.0, -(SCREEN_H - GRIP_H) / 2 + 0.01)
    return np.array([ixx, iyy, izz]), com


def cabinet_pieces():
    """Kinematic dryer parts in the dryer frame D: list of (name, verts, kind). kind: 'top', 'housing', 'body',
    'console', 'pocket'."""
    out = []
    w2 = CAB_W / 2
    xb = SLOT_X + 0.0  # slot back wall x (D frame)
    ye = SLOT_L / 2  # slot end |y|
    zt, zb = 0.0, -TOP_T

    def tf(yy):  # slot front wall x at position y (D frame), with the screen centred on the dryer's centre line
        return xb - SLOT_BACK_GAP - thickness(yy) - SLOT_FRONT_GAP

    # top panel: front strip (slanted back edge), back strip, end strips beyond the pockets; pocket floors
    yp = ye + POCKET_L
    xpf, xpb = xb - POCKET_W / 2 - 0.008, xb + POCKET_W / 2 - 0.008  # pocket x span (centred on the slot)
    # front strip between the front face and the slot front wall / pocket front edge
    out.append(("top_front", np.array(
        [[0.0, -w2, zb], [0.0, -w2, zt], [0.0, w2, zb], [0.0, w2, zt],
         [xpf, -w2, zb], [xpf, -w2, zt], [xpf, w2, zb], [xpf, w2, zt]]), "top"))
    out.append(("top_back", box_v(((xpb + CAB_D) / 2, 0.0, -TOP_T / 2), (CAB_D - xpb, CAB_W, TOP_T)), "top"))
    for s in (-1, 1):
        out.append((f"top_end_{'L' if s > 0 else 'R'}", box_v(
            ((xpf + xpb) / 2, s * (yp + w2) / 2, -TOP_T / 2), (xpb - xpf, w2 - yp, TOP_T)), "top"))
        # pocket floor
        out.append((f"pocket_{'L' if s > 0 else 'R'}", box_v(
            ((xpf + xpb) / 2, s * (ye + yp) / 2, -POCKET_DEPTH - 0.002), (xpb - xpf, yp - ye, 0.004)), "pocket"))
    # slot lips: between the pocket span and the slot walls along the slot length (front lip slanted)
    ys = np.linspace(-ye, ye, 5)
    for i in range(4):
        a, b = ys[i], ys[i + 1]
        out.append((f"lip_front_{i}", np.array(
            [[xpf, a, zb], [xpf, a, zt], [xpf, b, zb], [xpf, b, zt],
             [tf(a), a, zb], [tf(a), a, zt], [tf(b), b, zb], [tf(b), b, zt]]), "top"))
    out.append(("lip_back", box_v(((xb + xpb) / 2, 0.0, -TOP_T / 2), (xpb - xb, 2 * ye, TOP_T)), "top"))
    # housing below the slot (front wall slanted, back wall straight, two end walls, floor)
    hz0, hz1 = -HOUSING_DEPTH, zb
    for i in range(4):
        a, b = ys[i], ys[i + 1]
        out.append((f"housing_front_{i}", np.array(
            [[tf(a) - 0.004, a, hz0], [tf(a) - 0.004, a, hz1], [tf(b) - 0.004, b, hz0], [tf(b) - 0.004, b, hz1],
             [tf(a), a, hz0], [tf(a), a, hz1], [tf(b), b, hz0], [tf(b), b, hz1]]), "housing"))
    out.append(("housing_back", box_v((xb + 0.002, 0.0, (hz0 + hz1) / 2), (0.004, 2 * ye + 0.008, hz1 - hz0)),
                "housing"))
    for s in (-1, 1):
        out.append((f"housing_end_{'L' if s > 0 else 'R'}", box_v(
            (xb - 0.015, s * (ye + 0.002), (hz0 + hz1) / 2), (0.034, 0.004, hz1 - hz0)), "housing"))
    out.append(("housing_floor", box_v((xb - 0.015, 0.0, hz0 - 0.002), (0.034, 2 * ye + 0.008, 0.004)), "housing"))
    # console
    out.append(("console", box_v((CAB_D - CONSOLE[0] / 2, 0.0, CONSOLE[1] / 2), (CONSOLE[0], CAB_W, CONSOLE[1])),
                "console"))
    return out


def cabinet_body_box():
    """Visual cabinet shell below the top panel (front, sides): (center, size) in D, from the floor."""
    h = D0[2] - TOP_T - FLOOR
    return np.array([CAB_D / 2, 0.0, -TOP_T - h / 2]), (CAB_D, CAB_W, h)


def bin_pieces():
    """Bin walls and floor (bin frame: centre of its floor on the table)."""
    ix, iy = BIN_IN
    w, h = BIN_WALL, BIN_H
    # floor 34 mm thick, mostly below the table top (hidden inside the table): nothing thin to fall through
    out = [box_v((0.0, 0.0, (w - 0.030) / 2), (ix + 2 * w, iy + 2 * w, w + 0.030))]
    for s in (-1, 1):
        out.append(box_v((s * (ix / 2 + w / 2), 0.0, h / 2), (w, iy + 2 * w, h)))
        out.append(box_v((0.0, s * (iy / 2 + w / 2), h / 2), (ix, w, h)))
    return out
