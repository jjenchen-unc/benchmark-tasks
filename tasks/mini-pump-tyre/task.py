"""On the station table: a small 11 cm pram front wheel lies flat and fixed, ahead of the arms and a little left of centre.
Its tyre is soft (0.8 bar). Its brass valve stem rises from the rim and is bent 90 degrees, so its tip points straight at
the right arm, 6 cm above the table. A black mini bike pump with no gauge lies on the table in front of the arms, along
the left-right direction: its push-on head (the end with the square opening) on the left, its sliding handle knob on the
right. The robot must push the pump's head fully onto the valve stem, hold it there while it strokes the handle in and
out, about 10 full strokes, until the tyre is at 1.5 bar (each full stroke adds about 0.08 bar at first and 0.07 bar near
the target), then pull the pump off the valve and lay it on the table.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys
import warnings

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from partkit import PartState, Slider, _np, on_reset, on_step, part  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = ("Pump the tyre up to 1.5 bar with the mini pump, then take the pump off the valve and lay it on the "
               "table. The tyre starts at 0.8 bar; the pump has no gauge, and each full stroke adds about 0.08 bar.")

BLACK = (0.08, 0.08, 0.09)
TYRE = (0.12, 0.12, 0.12)
RIM = (0.70, 0.71, 0.73)
BRASS = (0.80, 0.62, 0.25)
PUMP = (0.10, 0.10, 0.11)
HEAD = (0.85, 0.20, 0.15)
HANDLE = (0.95, 0.75, 0.10)

# -- the wheel (fixed): a 4 inch pram front wheel lying flat, with a 90 degree bent valve stem ------------------------
WHEEL_XY = np.array([0.32, 0.14])
TYRE_R, TYRE_H = 0.055, 0.028
RIM_R, RIM_H = 0.032, 0.032
STEM = 0.008                                   # square stem, 8 mm (a Schrader stem is 8 mm round)
STEM_Z = TOP + 0.062                           # height of the horizontal stem's axis
STEM_TIP_Y = WHEEL_XY[1] - TYRE_R              # the tip points at -y, level with the tyre's edge
RISER_Y = WHEEL_XY[1] - 0.025
STEM_TIP = np.array([WHEEL_XY[0], STEM_TIP_Y, STEM_Z])

WHEEL = [
    Cylinder("tyre", pos=(*WHEEL_XY, TOP + TYRE_H / 2), radius=TYRE_R, height=TYRE_H, color=TYRE),
    Cylinder("rim", pos=(*WHEEL_XY, TOP + RIM_H / 2), radius=RIM_R, height=RIM_H, color=RIM),
    Box("valve_riser", pos=(WHEEL_XY[0], RISER_Y, (TOP + RIM_H + STEM_Z + STEM / 2) / 2),
        size=(STEM, STEM, STEM_Z + STEM / 2 - TOP - RIM_H + 0.001), color=BRASS),
    Box("valve_stem", pos=(WHEEL_XY[0], (STEM_TIP_Y + RISER_Y - STEM / 2) / 2, STEM_Z),
        size=(STEM, RISER_Y - STEM / 2 - STEM_TIP_Y, STEM), color=BRASS),
]

# -- the pump, in its own frame: origin on the barrel axis at the head's opening, +y from the head toward the handle -----
# The head has a 12 x 12 mm square opening 18 mm deep (2 mm clearance round the 8 mm stem); the barrel is 24 mm across
# and 15 cm long; the handle slides 12 cm along the pump axis on a thin rod and ends in a 30 x 24 x 33 mm knob.
BORE, BORE_DEPTH, HEAD_W, HEAD_L = 0.012, 0.018, 0.026, 0.030
BARREL_R, BARREL_END = 0.012, 0.180
TRAVEL = 0.120
GRIP_Y = BARREL_END + TRAVEL + 0.010 + 0.012   # knob centre at rest: 1 cm clear of the barrel at full stroke


def _b(x0, x1, y0, y1, z0, z1, color):
    return (((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), color)


h, b = HEAD_W / 2, BORE / 2
PUMP_BOXES = [
    _b(-h, h, 0, BORE_DEPTH, b, h, HEAD),                 # head walls round the opening
    _b(-h, h, 0, BORE_DEPTH, -h, -b, HEAD),
    _b(b, h, 0, BORE_DEPTH, -b, b, HEAD),
    _b(-h, -b, 0, BORE_DEPTH, -b, b, HEAD),
    _b(-h, h, BORE_DEPTH, HEAD_L, -h, h, HEAD),           # back of the head: the opening's floor stops the stem
]
PUMP_CYLINDERS = [((0.0, (HEAD_L + BARREL_END) / 2, 0.0), BARREL_R, BARREL_END - HEAD_L, PUMP, "Y")]
HANDLE_SLIDER = Slider(
    "handle", rest=(0.0, 0.0, 0.0), axis="Y", travel=TRAVEL,
    boxes=(_b(-0.003, 0.003, BARREL_END - 0.010, GRIP_Y - 0.010, -0.003, 0.003, RIM),   # rod (into the barrel)
           _b(-0.015, 0.015, GRIP_Y - 0.012, GRIP_Y + 0.012, -0.020, 0.013, HANDLE)),   # handle knob
    rate=300.0, damping=3.0, mass=0.030)

# Start: lying on the table in front of the arms, head to the left (+y), opening facing +y.
PUMP_START = (0.20, 0.06, TOP + 0.020)            # resting on the knob and the head
PUMP_YAW = math.pi                            # pump +y -> world -y
PUMP_PART = part("pump", pos=PUMP_START, yaw=PUMP_YAW, mass=0.10, boxes=PUMP_BOXES, cylinders=PUMP_CYLINDERS,
                 sliders=(HANDLE_SLIDER,), color=PUMP, friction=0.9)

OBJECTS = [*WHEEL, PUMP_PART]

# -- air: an isothermal model of the pump cylinder and the tyre ---------------------------------------------------------
P_ATM = 1.013                    # bar, absolute
PISTON_AREA = math.pi * 0.004 ** 2   # 8 mm bore, m^2
DEAD_ML = 1.0                    # cylinder volume left at the end of a full stroke
TYRE_ML = 60.0                  # air volume of the tyre's inner tube
START_BAR = 0.8                  # gauge
TARGET_LO, TARGET_HI, SOME_AIR = 1.3, 1.7, 1.0
_STATE: dict = {}


def cylinder_ml(x: float) -> float:
    """Cylinder volume with the handle pushed in x metres."""
    return DEAD_ML + PISTON_AREA * (TRAVEL - x) * 1e6


def head_on_valve(pump: PartState) -> tuple[bool, float]:
    """(seated, depth): the stem is inside the opening, centred within 3 mm, the pump within 10 degrees of the stem's
    line and pushed on to within 3 mm of the opening's floor. depth is how far (m) the stem tip is inside the opening."""
    tip = pump.to_local(STEM_TIP)
    axis = pump.rotation()[:, 1]                     # pump +y in the world: should point along world -y
    aligned = float(-axis[1]) > math.cos(math.radians(10))
    centred = abs(tip[0]) < 0.003 and abs(tip[2]) < 0.003
    return bool(aligned and centred and tip[1] > BORE_DEPTH - 0.003), float(tip[1])


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(tyre=START_BAR + P_ATM, n_cyl=P_ATM * cylinder_ml(0.0), x=0.0, seated=False, seated_once=False,
                  step=0, scored=None, last=(0.0, False), z0=None, force=0.0)
    if environment is not None:
        _drive(environment, 0.0, 0.0)


def _handle(environment):
    base = environment.unwrapped
    asset = base.scene["pump"]
    idx = list(asset.joint_names).index("handle_slide")
    return asset, idx, float(_np(asset.data.joint_pos)[0, idx])


def _drive(environment, q: float, force: float) -> None:
    """The air pushes the handle back out with ``force`` (N): the slider's spring drive (300 N/m) is aimed past the
    handle by force / rate. With no pressure it aims at the handle itself, so the handle stays where it is left."""
    import torch

    asset, idx, _ = _handle(environment)
    target = torch.tensor([[q + force / HANDLE_SLIDER.rate]], dtype=torch.float32, device=environment.unwrapped.device)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        asset.set_joint_position_target_index(target=target, joint_ids=[idx])


@on_step
def _air(environment) -> None:
    """After each control step: compress the cylinder air to the handle's new position; it flows into the tyre when the
    head is on the valve and the cylinder pressure is above the tyre's, and out of the head when it is off the valve;
    pulling the handle back draws fresh air in past the piston cup."""
    from yam_arena.scene import object_states

    pump = object_states(environment, ["pump"])["pump"]
    _, _, q = _handle(environment)
    x = min(max(-q, 0.0), TRAVEL)
    seated, _ = head_on_valve(pump)
    v = cylinder_ml(x)
    p = _STATE["n_cyl"] / v
    if p < P_ATM:                       # drawing back: air in past the cup
        p = P_ATM
    elif not seated:                    # head open to the room
        p = P_ATM
    elif p > _STATE["tyre"]:            # valve core open: share with the tyre
        p = (p * v + _STATE["tyre"] * TYRE_ML) / (v + TYRE_ML)
        _STATE["tyre"] = p
    _STATE.update(n_cyl=p * v, x=x, seated=seated, force=(p - P_ATM) * 1e5 * PISTON_AREA)
    _STATE["seated_once"] |= seated
    _drive(environment, q, _STATE["force"])


_reset_state()


def tyre_bar() -> float:
    """The tyre's gauge pressure (bar), what a tyre gauge would read."""
    return _STATE["tyre"] - P_ATM


def _update(objects) -> tuple[float, bool]:
    if _STATE["scored"] == _STATE["step"]:
        return _STATE["last"]
    _STATE["scored"] = _STATE["step"]
    pump = objects["pump"]
    _, depth = head_on_valve(pump)
    off = depth < -0.02 or float(np.linalg.norm(pump.pos - STEM_TIP)) > 0.06
    on_table = bool(pump.low[2] < TOP + 0.01 and pump.resting(0.02, 0.5))
    bar = tyre_bar()
    if bar < SOME_AIR:
        result = (0.0, False)
    elif bar > TARGET_HI or bar < TARGET_LO:
        result = (0.25, False)
    elif off:
        done = on_table
        result = (1.0 if done else 0.75, done)
    else:
        result = (0.5, False)
    _STATE["last"] = result
    return result


@on_step
def _count_step(environment=None) -> None:
    _STATE["step"] += 1


def success(objects) -> bool:
    """Tyre at 1.3 to 1.7 bar, and the pump off the valve, lying still on the table; held 1 s."""
    return _update(objects)[1]


def score(objects) -> float:
    """All read from a tyre gauge and a photo at the end. 0 tyre below 1.0 bar; 0.25 tyre at 1.0 bar or more but outside
    1.3 to 1.7 bar (under- or over-inflated); 0.5 tyre at 1.3 to 1.7 bar; 0.75 also the pump off the valve; 1.0 also the
    pump lying still on the table."""
    return _update(objects)[0]
