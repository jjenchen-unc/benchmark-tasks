"""A 10-inch wooden handscrew lies flat on two wooden strips, its jaws open 8 mm wider than a pair of glued maple boards
that stand on edge between them. Each of its two spindles sets the opening at its own point, so the jaws only close
flat on the boards if both handles are turned by the right amounts.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from jointkit import Hinge, PartState, finger_points, on_reset, on_step, part, set_targets  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Clamp the two glued boards together in the wooden handscrew and leave the clamp holding them."

MAPLE = (0.86, 0.72, 0.48)
HANDLE_WOOD = (0.50, 0.30, 0.14)
STEEL = (0.62, 0.63, 0.66)
DARK = (0.12, 0.12, 0.12)
BOARD_A_COLOR = (0.93, 0.83, 0.62)   # maple
BOARD_B_COLOR = (0.70, 0.42, 0.26)   # cherry
STRIP = (0.42, 0.36, 0.30)

# -- the handscrew, in its own frame: x along the jaws (tips toward +x), y across the opening (jaw A on +y), z up; ------
# origin on the middle spindle's axis, midway between the jaws' inner faces at the start.
JAW_L, JAW_W = 0.254, 0.044          # 10 in maple jaws, 44 mm square
TIP, HEEL = 0.127, -0.127            # jaw ends; the middle spindle is 127 mm (5 in) from the tips
X1, X2 = 0.0, -0.095                 # middle and end spindle
XM = (X1 + X2) / 2
G0 = 0.046                           # opening at the start, the same at both spindles (jaws parallel)
LEAD = 0.006                         # one turn of a handle closes the jaws 6 mm at that spindle (left/right threads)
R_LEAD = LEAD / (2 * math.pi)
K_SPINDLE = 200_000.0                # N/m: give of the wooden jaws and nuts at each spindle
F_MAX = 300.0                        # N per spindle: past this the screw slips (jaw drive force limit)
HANDLE_R, HANDLE_L, FERRULE = 0.016, 0.115, 0.004
SPINDLE_R = 0.008
YA_IN, YB_IN = G0 / 2, -G0 / 2       # inner faces at the start

# -- layout (world: x forward from the arms, y left, metres) ------------------------------------------------------------
STRIP_H = 0.038
XC = 0.38                            # world x of the middle spindle
AXIS_Z = TOP + STRIP_H + JAW_W / 2   # spindle axes 60 mm above the table
BOARD_L, BOARD_T, BOARD_H = 0.200, 0.019, 0.089   # 3/4 x 3-1/2 in maple and cherry, 200 mm long
BOARD_MASS = 0.24
BOARD_X0 = 0.012                     # boards' inner ends 12 mm in front of the middle spindle (4 mm clear of it)
BOARD_CX = XC + BOARD_X0 + BOARD_L / 2
BOARD_CZ = TOP + BOARD_H / 2


def _jaw_box(y_centre, color=MAPLE):
    return ((0.0, y_centre, 0.0), (JAW_L, JAW_W, JAW_W), color)


def _handle_shapes(x, side):
    """Ferrule, wooden handle with a dark stripe along its top, and the steel spindle through both jaws (part frame)."""
    s = 1.0 if side > 0 else -1.0
    outer = s * (G0 / 2 + JAW_W)
    y_ferrule = outer + s * FERRULE / 2
    y_handle = outer + s * (FERRULE + HANDLE_L / 2)
    far = -s * (G0 / 2 + JAW_W + 0.030)
    cyl = (((x, y_ferrule, 0.0), 0.012, FERRULE, STEEL, "Y"),
           ((x, y_handle, 0.0), HANDLE_R, HANDLE_L, HANDLE_WOOD, "Y"),
           ((x, (outer + far) / 2, 0.0), SPINDLE_R, abs(outer - far), STEEL, "Y"))
    boxes = (((x, y_handle, HANDLE_R), (0.005, HANDLE_L - 0.012, 0.0015), DARK),)
    return boxes, cyl


H1_BOXES, H1_CYL = _handle_shapes(X1, +1)
H2_BOXES, H2_CYL = _handle_shapes(X2, -1)
JAWB_ANCHOR = (XM, YB_IN - JAW_W / 2, 0.0)
K_SLIDE = 2 * K_SPINDLE
K_CANT = K_SPINDLE * (X1 - X2) ** 2 / 2

CLAMP = part(
    "handscrew", pos=(XC, 0.0, AXIS_Z + 0.0005), mass=0.34, friction=0.5, color=MAPLE,
    boxes=(_jaw_box(YA_IN + JAW_W / 2),),
    hinges=(
        # jaw B: a carriage sliding across the opening, and the jaw turning on it about the vertical; their drives are
        # set every control step from the two handles' angles (see _screws)
        Hinge("slide", anchor=JAWB_ANCHOR, axis="Y", slide=True, lower=-0.050, upper=0.046, rate=K_SLIDE,
              damping=1500.0, mass=0.01, effort=2 * F_MAX),
        Hinge("jawb", anchor=JAWB_ANCHOR, axis="Z", parent="slide", boxes=(_jaw_box(YB_IN - JAW_W / 2),),
              lower=-0.45, upper=0.45, rate=K_CANT, damping=4.0, mass=0.34, effort=F_MAX * (X1 - X2)),
        Hinge("handle1", anchor=(X1, YA_IN + JAW_W, 0.0), axis="Y", boxes=H1_BOXES, cylinders=H1_CYL,
              damping=0.2, mass=0.28),
        Hinge("handle2", anchor=(X2, YB_IN - JAW_W, 0.0), axis="Y", parent="jawb", boxes=H2_BOXES, cylinders=H2_CYL,
              damping=0.2, mass=0.28),
    ),
)


def _board(name, y, color):
    return part(name, pos=(BOARD_CX, y, BOARD_CZ), mass=BOARD_MASS, friction=0.5, color=color,
                boxes=(((0.0, 0.0, 0.0), (BOARD_L, BOARD_T, BOARD_H), color),))


BOARD_A = _board("board_a", BOARD_T / 2, BOARD_A_COLOR)      # against jaw A once clamped
BOARD_B = _board("board_b", -BOARD_T / 2, BOARD_B_COLOR)
STRIPS = [Box("strip_a", pos=(XC, 0.049, TOP + STRIP_H / 2), size=(0.30, 0.050, STRIP_H), color=STRIP),
          Box("strip_b", pos=(XC, -0.050, TOP + STRIP_H / 2), size=(0.30, 0.060, STRIP_H), color=STRIP)]
OBJECTS = [*STRIPS, CLAMP, BOARD_A, BOARD_B]

# world positions a policy can read off the fixed layout: the middle of each handle, on its axis
HANDLE1_MID = (XC + X1, YA_IN + JAW_W + FERRULE + HANDLE_L / 2, AXIS_Z)
HANDLE2_MID = (XC + X2, YB_IN - JAW_W - FERRULE - HANDLE_L / 2, AXIS_Z)


# -- the screws ---------------------------------------------------------------------------------------------------------
def opening_from_joints(slide: float, cant: float):
    """Opening (m) at the middle and end spindle for jaw B's carriage position and angle."""
    c, t = math.cos(cant), math.tan(cant)
    base = G0 - slide + (JAW_W / 2) * (1 - 1 / c)
    return base - (X1 - XM) * t, base - (X2 - XM) * t


def joints_from_opening(s1: float, s2: float):
    t = (s2 - s1) / (X1 - X2)
    cant = math.atan(t)
    slide = G0 + (JAW_W / 2) * (1 - 1 / math.cos(cant)) - (s1 + s2) / 2
    return slide, cant


def opening_at(x: float, s1: float, s2: float) -> float:
    return s1 + (x - X1) * (s2 - s1) / (X2 - X1)


def commanded_opening(phi1: float, phi2: float):
    """Each spindle sets the opening at its own point. Handle 1 (on jaw A, toward the left arm) closes when turned
    clockwise seen from its end (negative about +y); handle 2 (on jaw B, toward the right arm) closes when turned
    clockwise seen from its end (positive about +y). The jaws cannot close past touching at the heel or the tips."""
    s1, s2 = G0 + R_LEAD * phi1, G0 - R_LEAD * phi2
    for x in (HEEL, TIP):
        s = opening_at(x, s1, s2)
        if s < 0.001:   # the jaw ends meet: pivot about the other spindle until they just touch
            if x == HEEL:
                s2 = s2 + (0.001 - s) * (X2 - X1) / (x - X1)
            else:
                s1 = s1 + (0.001 - s) * (X1 - X2) / (x - X2)
    return min(max(s1, 0.001), 0.15), min(max(s2, 0.001), 0.15)


_STATE: dict = {}


def _reset_state(environment=None) -> None:
    _STATE.update(force=(0.0, 0.0), failed=False, why="", clamp0=None, recent=[], released=False, finger_check=None,
                  last=None, step=0, cached=None, phi=[0.0, 0.0], raw=None)


_reset_state()


@on_reset
def _on_reset(environment) -> None:
    _reset_state()


@on_step
def _screws(environment) -> None:
    """Set jaw B's drive targets from the handles (the screws are self-locking: the jaw never turns a handle back)."""
    from yam_arena.scene import object_states

    _STATE["step"] += 1
    j = object_states(environment, ["handscrew"])["handscrew"].joints
    # the simulator reports a free joint's angle within one turn: count whole turns of each handle
    raw = (j["handle1"], j["handle2"])
    if _STATE["raw"] is not None:
        for i in range(2):
            _STATE["phi"][i] += (raw[i] - _STATE["raw"][i] + math.pi) % (2 * math.pi) - math.pi
    else:
        _STATE["phi"] = [raw[0], raw[1]]
    _STATE["raw"] = raw
    s1c, s2c = commanded_opening(*_STATE["phi"])
    slide, cant = joints_from_opening(s1c, s2c)
    set_targets(environment, "handscrew", {"slide": slide, "jawb": cant})
    s1, s2 = opening_from_joints(j["slide"], j["jawb"])
    # squeeze at each spindle: how far the jaws are held open past where the screw sets them (negative pushes apart)
    _STATE["force"] = (float(np.clip(K_SPINDLE * (s1 - s1c), -F_MAX, F_MAX)),
                       float(np.clip(K_SPINDLE * (s2 - s2c), -F_MAX, F_MAX)))


# -- hands off --------------------------------------------------------------------------------------------------------
RELEASE_GAP = 0.03


def _box_gap(p, centre, half):
    return float(np.linalg.norm(np.maximum(np.abs(np.asarray(p) - centre) - half, 0.0)))


def _clamp_boxes_local():
    """Boxes (centre, half size) round jaw A and handle 1 (clamp frame) and round jaw B and handle 2 (jaw B frame)."""
    h = np.array([HANDLE_R, (FERRULE + HANDLE_L) / 2, HANDLE_R])
    ya, yb = YA_IN + JAW_W, YB_IN - JAW_W
    a = [(np.array([0.0, YA_IN + JAW_W / 2, 0.0]), np.array([JAW_L, JAW_W, JAW_W]) / 2),
         (np.array([X1, ya + h[1], 0.0]), h)]
    b = [(np.array([0.0, YB_IN - JAW_W / 2, 0.0]), np.array([JAW_L, JAW_W, JAW_W]) / 2),
         (np.array([X2, yb - h[1], 0.0]), h)]
    return a, b


BOXES_A, BOXES_B = _clamp_boxes_local()


def jawb_to_clamp(slide: float, cant: float, p):
    """A point of jaw B's frame (its parts placed as at the start) in the clamp frame."""
    c, s = math.cos(cant), math.sin(cant)
    a = np.array(JAWB_ANCHOR) + [0.0, slide, 0.0]
    d = np.asarray(p, float) - np.array(JAWB_ANCHOR)
    return a + np.array([c * d[0] - s * d[1], s * d[0] + c * d[1], d[2]])


def clamp_to_jawb(slide: float, cant: float, p):
    c, s = math.cos(cant), math.sin(cant)
    d = np.asarray(p, float) - (np.array(JAWB_ANCHOR) + [0.0, slide, 0.0])
    return np.array(JAWB_ANCHOR) + np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], d[2]])


@on_step
def _fingers(environment) -> None:
    """Released: every finger pad of both arms at least 3 cm from the clamp (score() cannot see the robot)."""
    from yam_arena.scene import object_states

    try:
        st = object_states(environment, ["handscrew"])["handscrew"]
        sl, ca = st.joints["slide"], st.joints["jawb"]
        gaps = []
        for p in finger_points(environment):
            q = st.to_local(p)
            gaps += [_box_gap(q, c, h) for c, h in BOXES_A]
            qb = clamp_to_jawb(sl, ca, q)
            gaps += [_box_gap(qb, c, h) for c, h in BOXES_B]
        _STATE["released"] = bool(min(gaps) >= RELEASE_GAP)
        _STATE["finger_check"] = True
    except Exception as exc:  # noqa: BLE001 - report once; an unknown grip never counts as released
        if _STATE["finger_check"] is None:
            print(f"handscrew: finger check unavailable: {exc!r}", flush=True)
        _STATE["finger_check"], _STATE["released"] = False, False


# -- measuring ----------------------------------------------------------------------------------------------------------
CONTACT_GAP = 0.0005      # a 0.5 mm feeler gauge does not enter
MIN_SQUEEZE = 50.0        # N between the jaws
PARALLEL = math.radians(0.5)   # jaws parallel within 0.5 degrees
CLAMP_MOVE_LIMIT, CLAMP_TILT_LIMIT = 0.03, math.radians(10)
BOARD_FALL = math.radians(30)


def jaw_gaps(clamp: PartState, board_a: PartState, board_b: PartState):
    """For jaw A against board A's outer face and jaw B against board B's outer face: the gap (m) at the board's end
    nearest the middle spindle, at the jaw tips (or the board's far end), and the smallest, over the stretch where
    board and jaw overlap; signed (negative where the jaw is pressed into the board); None if a board is not between
    the jaws."""
    sl, ca = clamp.joints["slide"], clamp.joints["jawb"]
    zc = AXIS_Z - 0.0  # jaw mid-height in the world
    out = []
    for board, side in ((board_a, 1.0), (board_b, -1.0)):
        zl = board.to_local([board.pos[0], board.pos[1], zc])[2]
        pts = [clamp.to_local(board.to_world((u, side * BOARD_T / 2, zl))) for u in np.linspace(-BOARD_L / 2,
                                                                                                BOARD_L / 2, 81)]
        if side > 0:
            keep = [(p[0], YA_IN - p[1]) for p in pts if X1 + SPINDLE_R <= p[0] <= TIP]
        else:
            keep = []
            for p in pts:
                q = clamp_to_jawb(sl, ca, p)   # jaw B's inner face is y = YB_IN in its frame
                if X1 + SPINDLE_R <= q[0] <= TIP:
                    keep.append((q[0], q[1] - YB_IN))
        if len(keep) < 2:
            return None
        keep.sort()
        out.append((keep[0][1], keep[-1][1], min(g for _, g in keep)))   # signed: negative = pressed in
    return out


def joint_gaps(clamp: PartState, board_a: PartState, board_b: PartState):
    """The glue line: gap (m) between the two boards' glued faces at the jaw's mid-height, at both ends of the stretch
    inside the jaws (the inner end and the jaw tips); None if the boards are not between the jaws."""
    zl = board_b.to_local([board_b.pos[0], board_b.pos[1], AXIS_Z])[2]
    keep = []
    for u in np.linspace(-BOARD_L / 2, BOARD_L / 2, 81):
        w = board_b.to_world((u, BOARD_T / 2, zl))          # on board B's glued face
        x = clamp.to_local(w)[0]
        if X1 + SPINDLE_R <= x <= TIP:
            keep.append((x, -BOARD_T / 2 - board_a.to_local(w)[1]))   # board A's glued face is its -y face
    if len(keep) < 2:
        return None
    keep.sort()
    return keep[0][1], keep[-1][1]


def _tilt(s: PartState) -> float:
    return math.acos(max(-1.0, min(1.0, s.rotation()[2, 2])))


def measure(objects) -> dict:
    clamp, a, b = objects["handscrew"], objects["board_a"], objects["board_b"]
    gaps = jaw_gaps(clamp, a, b)
    rel = a.to_local(b.pos)
    joint = joint_gaps(clamp, a, b)
    m = dict(gaps=gaps, joint=joint, joint_closed=bool(joint and max(abs(g) for g in joint) <= CONTACT_GAP), squeeze=sum(_STATE["force"]), force=_STATE["force"],
             touching=bool(gaps and all(g[2] <= CONTACT_GAP for g in gaps)),
             flush=bool(gaps and all(abs(g[0]) <= CONTACT_GAP and abs(g[1]) <= CONTACT_GAP for g in gaps)),
             parallel=bool(abs(clamp.joints["jawb"]) <= PARALLEL),
             aligned=bool(abs(rel[0]) <= 0.002 and abs(rel[2]) <= 0.002),
             upright=bool(max(_tilt(a), _tilt(b)) < math.radians(10)),
             cant_deg=math.degrees(clamp.joints["jawb"]))
    return m


def _update(objects) -> tuple[float, bool]:
    """Once per control step (success() and score() are both called after each step)."""
    if _STATE["cached"] is not None and _STATE["cached"][0] == _STATE["step"]:
        return _STATE["cached"][1]
    out = _evaluate(objects)
    _STATE["cached"] = (_STATE["step"], out)
    return out


def _evaluate(objects) -> tuple[float, bool]:
    clamp, a, b = objects["handscrew"], objects["board_a"], objects["board_b"]
    if _STATE["clamp0"] is None:
        _STATE["clamp0"] = clamp.pos.copy()
    if float(np.linalg.norm(clamp.pos - _STATE["clamp0"])) > CLAMP_MOVE_LIMIT or _tilt(clamp) > CLAMP_TILT_LIMIT:
        _STATE["failed"], _STATE["why"] = True, _STATE["why"] or "clamp knocked out of place"
    if max(_tilt(a), _tilt(b)) > BOARD_FALL:
        _STATE["failed"], _STATE["why"] = True, _STATE["why"] or "a board fell over"
    m = measure(objects)
    gripped = m["touching"] and m["upright"] and m["squeeze"] >= MIN_SQUEEZE
    full = gripped and m["parallel"] and m["flush"] and m["joint_closed"] and m["aligned"] and _STATE["released"]
    recent = _STATE["recent"]
    recent.append((a.pos.copy(), b.pos.copy(), clamp.pos.copy(), full))
    del recent[:-20]
    still = len(recent) == 20 and all(f and np.linalg.norm(pa - a.pos) < 0.001 and np.linalg.norm(pb - b.pos) < 0.001
                                      and np.linalg.norm(pc - clamp.pos) < 0.001 for pa, pb, pc, f in recent)
    _STATE["last"] = m
    if _STATE["failed"]:
        return 0.0, False
    if full and still:
        return 1.0, True
    if gripped:
        return 0.5, False
    if m["touching"]:
        return 0.25, False
    return 0.0, False


def success(objects) -> bool:
    """Jaws parallel within 0.5 degrees, both flat on the boards and the glue line closed (a 0.5 mm gauge enters at
    neither end of either jaw or of the joint, no jaw edge 0.5 mm into a board), at least 50 N squeeze,
    boards flush and upright, both grippers 3 cm or more from the clamp, everything still, for 1 s."""
    return _update(objects)[1]


def score(objects) -> float:
    """1.0 success; 0.5 boards held (both jaws touching, 50 N squeeze, boards upright) but a jaw or the glue line gapes
    at one end;
    0.25 both jaws touch the boards; 0 otherwise, and 0 if the clamp was ever moved 3 cm or tilted 10 degrees or a
    board fell over."""
    return _update(objects)[0]


def demo():
    """No scripted plan for yam_scripted (the demo is demo.py); an empty plan holds both arms still."""
    return {}
