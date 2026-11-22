"""Unit checks of the edge-ring scorer and geometry (numpy only): python tests/test_scoring.py"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
import layout as L  # noqa: E402
import scoring as S  # noqa: E402

FAR = [np.array([0.0, 0.4, 1.0])] * 4  # fingertips far from the ring, outside the bowl
NEAR = [
    L.C + np.array([L.RING_R[1] + 0.005, 0.0, 0.03])
] * 4  # a fingertip 5 mm outside the rim, inside the bowl
results = []


def check(name, cond):
    results.append((name, bool(cond)))
    print(("PASS" if cond else "FAIL"), name)


def run(pos, rot, lift, tips, steps=25, v=0.0, w=0.0, mon=None):
    sc = S.Scorer()
    mon = mon or S.Monitor()
    lvl = 0.0
    for _ in range(steps):
        lvl = sc.step(S.measure(pos, rot, lift, tips), v, w, mon)
    return lvl, sc


# geometry
pieces = L.ring_pieces()
area = sum(L._area(p[: len(p) // 2, :2]) for p in pieces)
ideal = math.pi * (L.RING_R[1] ** 2 - L.RING_R[0] ** 2) - 3 * L.SLOT_W * (
    L.SLOT_R[1] - L.SLOT_R[0]
)
check(
    "ring pieces cover the annulus minus the slots (within 0.2 %)",
    abs(area - ideal) / ideal < 0.002,
)
rmin = min(np.linalg.norm(p[:, :2], axis=1).min() for p in pieces)
check(
    "ring bore keeps >= 0.4 mm radial clearance to the chuck (chord sag)",
    rmin - L.PUCK_R >= 0.0004,
)
check(
    "seated ring (slot play 0.15 mm a side) keeps >= 0.25 mm to the chuck",
    rmin - L.PUCK_R - (L.SLOT_W - 2 * L.LOC_PIN_R) / 2 >= 0.00025,
)
check(
    "slot pattern has no rotational symmetry",
    all(
        sorted(((a + d) % 360) for a in L.LOC_ANGLES) != sorted(L.LOC_ANGLES)
        for d in range(1, 360)
    ),
)
check(
    "dowels engage before the ring bore reaches the chuck face",
    L.LOC_PIN_H - L.LOC_CHAMFER >= L.PUCK_TOP,
)
check(
    "ring enters the cover ring bore before the dowels (1 mm radial > 0.35 mm criterion)",
    L.COVER_R[0] - L.RING_R[1] >= S.SLOT_ACROSS_M,
)

# levels
p, r = L.ring_seated_pose(0.0)
check("seated, hands out, still 1 s -> 1.0", run(p, r, L.LIFT_DOWN, FAR)[0] == 1.0)
check(
    "seated, still, only 10 steps -> 0.5 (hold not complete)",
    run(p, r, L.LIFT_DOWN, FAR, steps=10)[0] == 0.5,
)
check("seated but a hand in the bowl -> 0.5", run(p, r, L.LIFT_DOWN, NEAR)[0] == 0.5)
check("seated but moving -> 0.5", run(p, r, L.LIFT_DOWN, FAR, v=0.01)[0] == 0.5)
check(
    "seated height but lift pins still up -> not seated (0)",
    run(p, r, L.LIFT_UP, FAR)[0] == 0.0,
)
check(
    "resting on the dowel tips (3 mm high) after lowering -> 0",
    run(p + [0, 0, 0.003], r, L.LIFT_DOWN, FAR)[0] == 0.0,
)
pl, rl = L.ring_lift_pose(0.0)
check(
    "on raised lift pins, clocked, hands off, still -> 0 (not installed)",
    run(pl, rl, L.LIFT_UP, FAR)[0] == 0.0,
)
check(
    "on lift pins 0.3 mm off -> 0",
    run(pl + [0, 0.0003, 0], rl, L.LIFT_UP, FAR)[0] == 0.0,
)
check(
    "on lift pins 0.4 mm off -> 0",
    run(pl + [0, 0.0004, 0], rl, L.LIFT_UP, FAR)[0] == 0.0,
)
check(
    "on lift pins 0.6 mm off -> 0",
    run(pl + [0, 0.0006, 0], rl, L.LIFT_UP, FAR)[0] == 0.0,
)
check(
    "on lift pins 0.4 deg off (1.1 mm at the pins) -> 0",
    run(pl, L.rz(0.4), L.LIFT_UP, FAR)[0] == 0.0,
)
check(
    "on lift pins at the stand angle -8 deg -> 0",
    run(pl, L.rz(-8.0), L.LIFT_UP, FAR)[0] == 0.0,
)
check(
    "on lift pins 1 mm off -> 0", run(pl + [0, 0.001, 0], rl, L.LIFT_UP, FAR)[0] == 0.0
)
check(
    "on lift pins, clocked, hand within 20 mm -> 0",
    run(pl, rl, L.LIFT_UP, NEAR)[0] == 0.0,
)
check(
    "on lift pins tilted 2 deg -> 0",
    run(
        pl,
        L.rz(0.0)
        @ np.array(
            [
                [1, 0, 0],
                [0, math.cos(0.035), -math.sin(0.035)],
                [0, math.sin(0.035), math.cos(0.035)],
            ]
        ),
        L.LIFT_UP,
        FAR,
    )[0]
    == 0.0,
)
ps, rs = L.ring_stand_pose(0)
check("ring on the stand -> 0", run(ps, rs, L.LIFT_UP, FAR)[0] == 0.0)
check(
    "do nothing start state -> 0",
    run(ps, rs, L.LIFT_UP, [L.C + [-0.3, 0.3, 0.3]] * 4)[0] == 0.0,
)

# vetoes
for name, kw in (
    (
        "chuck touched",
        dict(chuck_n=0.2, ring_vz=0.0, support_n=3.0, robot_on_ring_n=0.0),
    ),
    (
        "falling at 0.25 m/s, no robot contact",
        dict(chuck_n=0.0, ring_vz=-0.25, support_n=0.0, robot_on_ring_n=0.0),
    ),
    (
        "pressed 15 N for 60 ms",
        dict(chuck_n=0.0, ring_vz=0.0, support_n=18.0, robot_on_ring_n=15.0),
    ),
):
    m = S.Monitor()
    for _ in range(int(0.06 / (1 / 480)) + 1):
        m.step(1 / 480, **kw)
    check(f"{name} -> veto", m.veto and run(p, r, L.LIFT_DOWN, FAR, mon=m)[0] == 0.0)
m = S.Monitor()
for k in range(8):  # 16.7 ms of free fall (1.4 mm drop, 0.16 m/s) does not veto
    m.step(1 / 480, 0.0, -9.81 * (k + 1) / 480, 0.0, 0.0)
m.step(1 / 480, 0.0, 0.0, 3.0, 0.0)
check("free fall 16.7 ms (1.4 mm) -> no veto", not m.veto)
m = S.Monitor()
for _ in range(50):  # carried down fast (0.3 m/s) in the hands: no veto
    m.step(1 / 480, 0.0, -0.3, 0.0, 3.0)
check("lowered fast while held -> no veto", not m.veto)
m = S.Monitor()
for _ in range(
    200
):  # set down by hand: support = weight + 5 N while the robot still touches
    m.step(1 / 480, 0.0, 0.0, 2.9 + 5.0, 5.0)
check("set down with 5 N extra -> no veto", not m.veto)
m = S.Monitor()
for _ in range(
    200
):  # ring resting on pins with no robot contact: no press veto however large
    m.step(1 / 480, 0.0, 0.0, 2.9, 0.0)
check("resting, no robot contact -> no veto", not m.veto)
m = S.Monitor()
for _ in range(
    int(0.5 * 480) + 2
):  # resting on the dowel tips (bottom 3.8 mm up, base carries the weight) for 0.5 s
    m.step(1 / 480, 0.0, 0.0, 2.85, 0.0, base_n=2.85, ring_dz=0.0038)
check(
    "on the dowel tips 0.5 s -> veto",
    m.on_tips and m.veto and run(p, r, L.LIFT_DOWN, FAR, mon=m)[0] == 0.0,
)
m = S.Monitor()
for _ in range(
    int(0.5 * 480) + 2
):  # touched and turned on the tips by a hand (robot contact): still a veto
    m.step(1 / 480, 0.0, 0.0, 2.0, 1.0, base_n=2.0, ring_dz=0.0035)
check("turned on the tips by hand -> veto", m.on_tips)
m = S.Monitor()
for _ in range(
    int(0.1 * 480)
):  # sliding down a dowel lead-in while the pins lower (0.1 s): no veto
    m.step(1 / 480, 0.0, -0.005, 2.85, 0.0, base_n=1.5, ring_dz=0.0033)
for _ in range(480):  # then seated: the base ring carries it at the seat
    m.step(1 / 480, 0.0, 0.0, 2.85, 0.0, base_n=2.85, ring_dz=0.0)
check("lead-in slide then seated -> no veto", not m.veto)
m = S.Monitor()
for _ in range(480):  # held 3 mm above the seat by the hands, base not touched: no veto
    m.step(1 / 480, 0.0, 0.0, 0.0, 3.0, base_n=0.0, ring_dz=0.003)
check("held above the seat, no dowel contact -> no veto", not m.veto)
check(
    "ring distance: point on the rim surface = 0",
    S.ring_distance(p + [L.RING_R[1], 0, 0.002], p, r) < 1e-9,
)

fails = [n for n, ok in results if not ok]
print(f"{len(results) - len(fails)}/{len(results)} passed")
sys.exit(1 if fails else 0)
