"""Unit checks of the h4-bulb-clip scorer and geometry (numpy only): python tests/test_scoring.py"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
import layout as L  # noqa: E402
import scoring as S  # noqa: E402

FAR = [np.array([0.0, 0.4, 1.0])] * 4
results = []


def check(name, cond):
    results.append((name, bool(cond)))
    print(("PASS" if cond else "FAIL"), name)


def run(pos, rot, phi, catch, tips=FAR, steps=25, v=0.0, w=0.0, mon=None):
    sc = S.Scorer()
    mon = mon or S.Monitor()
    lvl = 0.0
    for _ in range(steps):
        lvl = sc.step(S.measure(pos, rot, phi, catch, tips), v, w, mon)
    return lvl, sc


seat_p, seat_r = L.bulb_seated_pose(0.0)
# geometry
check("tabs asymmetric: no rotation other than 0 maps the tab pattern onto itself",
      all(
          not all(any(abs(S.wrap180(a + d - b)) < 1.0 for b in L.TAB_ANGLES) for a in L.TAB_ANGLES)
          for d in range(5, 360, 5)
      ))
check("tabs reach past the rim bore (a misfit bulb rests on the rim)", L.TAB_R > L.RIM_R[0] + 0.002)
check("flange disc fits the rim bore with 0.4 mm radial play", abs(L.RIM_R[0] - L.FLANGE_R - 0.0004) < 1e-9)
check("closed bight sits under the catch lip", L.LIP_EDGE_Y > L.CATCH_Y + L.WIRE_R)
check("catch travel clears the bight", L.CATCH_TRAVEL > (L.LIP_EDGE_Y - (L.CATCH_Y - L.WIRE_R)))
check("bight closed position", np.allclose(L.bight_point(0.0), [L.HINGE_X, L.CATCH_Y, 0.0]))
check("clip angle round trip", abs(L.clip_angle(L.clip_rot(73.0)) - 73.0) < 1e-9)
check("legs cross the flange disc, not the cap body", L.BASE_R < L.CLIP_HALF_W - L.WIRE_R < L.FLANGE_R)
# official seed start is non-trivial: bulb has to be turned, clip open
sp = L.setup_spec(("seed", 1415158438))
check("official seed: clip open >= 95 deg", sp["clip"] >= 95.0)
# levels
lvl, _ = run(seat_p, seat_r, 0.3, 0.0)
check("seated + latched + clear + still -> 1.0", lvl == 1.0)
lvl, _ = run(seat_p, seat_r, 0.3, 0.0, steps=10)
check("needs 1 s still", lvl == 0.5)
near = [seat_p + np.array([-0.040, 0.0, 0.0])] * 4
lvl, _ = run(seat_p, seat_r, 0.3, 0.0, tips=near)
check("hand within 30 mm -> 0.5", lvl == 0.5)
lvl, _ = run(seat_p, seat_r, 4.0, 0.0)
check("clip over but not hooked -> 0.5", lvl == 0.5)
lvl, _ = run(seat_p, seat_r, 0.5, -0.0015)
check("catch held open (not snapped over) -> not latched -> 0.5", lvl == 0.5)
lvl, _ = run(seat_p, seat_r, 90.0, 0.0)
check("seated, clip open -> 0.25", lvl == 0.25)
lvl, _ = run(seat_p + np.array([-0.0010, 0.0, 0.0]), seat_r, 90.0, 0.0)
check("flange 1 mm proud (tab on the rim) -> 0", lvl == 0.0)
lvl, _ = run(seat_p, L.rx(25.0), 0.3, 0.0)
check("roll 25 deg off -> 0", lvl == 0.0)
lvl, _ = run(seat_p, L.ry(2.0), 0.3, 0.0)
check("cocked 2 deg -> 0", lvl == 0.0)
hp, hr = L.bulb_holder_pose(0)
lvl, _ = run(hp, hr, 100.0, 0.0)
check("bulb in its holder -> 0", lvl == 0.0)
m = S.Monitor()
m.step(0.002, 0.2, 0.0, 0.2)
lvl, _ = run(seat_p, seat_r, 0.3, 0.0, mon=m)
check("glass touched -> 0 (latched veto)", lvl == 0.0)
m = S.Monitor()
m.step(0.002, 0.0, -0.5, 0.0)
lvl, _ = run(seat_p, seat_r, 0.3, 0.0, mon=m)
check("dropped -> 0", lvl == 0.0)
m = S.Monitor()
m.step(0.002, 0.0, -0.5, 3.0)
check("fast downward motion while held is not a drop", not m.dropped)
lvl, _ = run(seat_p, seat_r, 0.3, 0.0, v=0.01)
check("bulb moving -> not 1.0", lvl == 0.5)
check("bulb distance zero at the axis", S.bulb_distance(seat_p, seat_p, seat_r) == 0.0)
check("clip distance at the bight", S.clip_distance(L.H + L.bight_point(30.0), 30.0) < 1e-9)
n_fail = sum(1 for _, ok in results if not ok)
print(f"{len(results) - n_fail}/{len(results)} passed")
sys.exit(1 if n_fail else 0)
