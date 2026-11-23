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
check("flange disc fits the rim bore with 0.10 mm radial play", abs(L.RIM_R[0] - L.FLANGE_R - 0.0001) < 1e-9)
check("closed bight sits under the catch lip", L.LIP_EDGE_Y > L.CATCH_Y + L.WIRE_R)
check("catch travel clears the bight", L.CATCH_TRAVEL > (L.LIP_EDGE_Y - (L.CATCH_Y - L.WIRE_R)))
check("bight closed position", np.allclose(L.bight_point(0.0), [L.HINGE_X, L.CATCH_Y, L.BIGHT_Z]))
check("flat retaining face has no entry ramp", len(set(L.catch_pieces()[0][:, 0])) == 2)
check("axial press against the outer face does not latch",
      not S.measure(*L.bulb_seated_pose(), 1.0, 0.0, FAR).latched)
check("wire still displaced cannot receive full credit",
      not S.measure(*L.bulb_seated_pose(), 0.0, -L.CATCH_TRAVEL, FAR).latched)
check("clip angle round trip", abs(L.clip_angle(L.clip_rot(73.0)) - 73.0) < 1e-9)
check("legs cross the flange disc, not the cap body", L.BASE_R < L.CLIP_HALF_W - L.WIRE_R < L.FLANGE_R)
# Check generated geometry against independent nominal values in P43t sheets.
parts = L.bulb_pieces()["metal"]
check("raw cap vertices fit the 255-vertex PhysX hull limit", all(len(v) <= 255 for v in parts))
check("43 mm centring envelope and 51.8 mm lug circle are distinct",
      np.isclose(np.linalg.norm(parts[2][:, 1:], axis=1).max(), 0.04295/2)
      and all(np.isclose(np.linalg.norm(v[:, 1:], axis=1).max(), 0.0518/2) for v in parts[3:6]))
check("lower lugs flank downward axis by 45 degrees", L.TAB_ANGLES == (90, 225, 315))
for angle, lug, width, slot_width in zip(L.TAB_ANGLES, parts[3:6], (0.00795, 0.006, 0.006),
                                        (0.0081, 0.00825, 0.00825)):
    tangent = np.array([-np.sin(np.radians(angle)), np.cos(np.radians(angle))])
    check(f"parallel-sided lug at {angle} degrees fits its holder slot",
          np.isclose(np.ptp(lug[:, 1:] @ tangent), width)
          and np.isclose(L.NOTCH_WIDTHS[L.TAB_ANGLES.index(angle)], slot_width) and width < slot_width)
# Measure the two physical slot walls at both the inner bore and outer rim.
rim = np.vstack(L.rim_pieces())
for angle, slot in zip(L.TAB_ANGLES, (0.0081, 0.00825, 0.00825)):
    tang = np.radians(angle)
    forward = rim[:, 1:] @ [np.cos(tang), np.sin(tang)]
    sideways = rim[:, 1:] @ [-np.sin(tang), np.cos(tang)]
    widths = []
    for radius in L.RIM_R:
        at_radius = (forward > 0) & np.isclose(np.linalg.norm(rim[:, 1:], axis=1), radius)
        widths.append(sideways[at_radius & (sideways > 0)].min() - sideways[at_radius & (sideways < 0)].max())
    check(f"both edges of the generated {angle}-degree slot have the specified width",
          np.allclose(widths, slot, atol=1e-8))
# The mount behind the boss previously intersected the last 2.5 mm of shoulder travel.
stand_lo = np.asarray(L.STAND_CENTER) - np.asarray(L.STAND_SIZE)/2
stand_hi = np.asarray(L.STAND_CENTER) + np.asarray(L.STAND_SIZE)/2
check("stand clears the entire insertion corridor from glass tip to base",
      stand_hi[2] < min(piece[:, 2].min() for group in L.bulb_pieces().values() for piece in group))
check("stand attaches to the plate outside the bore and reaches the table",
      -L.PLATE_R < stand_hi[2] < -L.HOLE_R and stand_lo[0] < L.PLATE_X[1] < stand_hi[0]
      and np.isclose(L.H[2]+stand_lo[2], L.TOP))
check("cup supports flange and clears the formed shoulder", L.PILOT_R < L.CUP_R[0] < L.CUP_R[1] < L.FLANGE_R)
check("rear shroud admits wire and right-side fixed tab at closure",
      np.hypot(L.CATCH_OUTER_Y - 0.003, L.LIP_Z[1]) < L.BOOT_INNER_R)
check("both operating-end and hinge joints follow the asymmetric wire",
      len(L.clip_pieces()) == 10 and L.LIP_Z[0] < L.BIGHT_Z < L.LIP_Z[1]
      and S.clip_distance(L.H + L.bight_point(0), 0) == 0)
def load_at(phi, bulb=seat_p):
    return L.flange_springs(L.H + L.hinge_point(), L.clip_rot(phi), bulb, seat_r)

loaded = load_at(0)
loads = [sum(f[0] for _, f in load_at(phi)) for phi in (0, 1, 3, 10)]
check("four contact bends give a 21 N nominal axial load", len(loaded) == 4 and np.isclose(loads[0], 21))
check("wire unwinds progressively as the bail opens", loads[0] > loads[1] > loads[2] > loads[3] == 0)
check("nominal wire forces balance about bulb centre",
      np.allclose(sum((np.cross(c-seat_p, f) for c, f in loaded)), 0, atol=1e-10))
check("no load on absent or separated bulb", not load_at(0, seat_p + [0.01, 0, 0])
      and not load_at(0, seat_p + [0, 0.1, 0]) and not load_at(0, seat_p - [0.05, 0, 0]))
check("off-axis bulb cannot score as seated",
      not S.measure(seat_p + [0, 0.0003, 0], seat_r, 0, 0, FAR).seated)
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
