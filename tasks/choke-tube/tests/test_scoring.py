"""Unit checks of the choke-tube scorer and layout (no simulator): python tests/test_scoring.py"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
import layout as L  # noqa: E402
import scoring as S  # noqa: E402

FAR_TIPS = [np.array([0.1, 0.3, 1.0])] * 4  # hands at the robot's side, out of her reach
NEAR_TIPS = [np.array([L.MAT[0], L.MAT[1], L.TOP + 0.1])] * 4


def poses_for(setup, moves=None):
    a = L.authored(setup)
    for n, p in (moves or {}).items():
        a[n] = (np.asarray(p, float), np.eye(3))
    return a


def zero_vels(poses):
    return {n: (np.zeros(3), np.zeros(3)) for n in poses}


def run(setup, moves, steps, tips=FAR_TIPS, touched=None, child_n=0.0):
    sc = S.Scorer(L.active_toys(setup))
    mon = S.Monitor()
    lv = 0.0
    for _ in range(steps):
        mon.step(1 / 480, child_n)
        p = poses_for(setup, moves)
        lv = sc.step(p, zero_vels(p), touched or {}, tips, mon)
    return lv, sc


def mat_pos(name):
    return (L.MAT[0], L.MAT[1], L.MAT_TOP + L.toy_half_height(name))


def check(cond, msg):
    if not cond:
        raise AssertionError(msg)
    print("ok", msg)


def test_geometry():
    check(abs(L.TUBE_ID - 0.0317) < 1e-6 and abs(L.DEPTH_DEEP - 0.0571) < 1e-6, "tube bore per 16 CFR 1501")
    # analytic fit: smallest cross-section circumscribed diameter vs bore, and fits below the rim
    for n, t in L.TOYS.items():
        a, b, c = t["size"]
        if t["kind"] == "ball":
            d, protrudes = a, False
        elif t["kind"] == "cube":
            d, protrudes = math.sqrt(2) * a, False
        elif t["kind"] == "stick":
            d = 2 * t["size"][2] / math.sqrt(3)  # end-on hexagon, across the corners
            protrudes = a > L.DEPTH_DEEP
        elif t["kind"] in ("cyl", "doll"):
            d = min(a, math.hypot(a, c))  # standing: its circle; the edge-on rectangle is larger
            protrudes = c > L.DEPTH_DEEP  # longer than the deepest point of the bore: it cannot sink below the rim
        else:
            d = min(math.hypot(a, b), math.hypot(a, c + t["studs"][3]))
            protrudes = False
        fits = d < L.TUBE_ID and not protrudes
        check(fits == t["fits"], f"{n}: smallest section {d * 1e3:.1f} mm, length {c * 1e3:.1f} mm -> fits={t['fits']}")
        if not protrudes:
            check(abs(d - L.TUBE_ID) >= 0.003, f"{n}: clearance margin >= 3 mm ({abs(d - L.TUBE_ID) * 1e3:.1f} mm)")
        small_ball = t["kind"] == "ball" and a <= 0.04445
        check(t["hazard"] == (t["fits"] or small_ball), f"{n}: hazard={t['hazard']} (fits or ball <= 1.75 in)")


def test_setups():
    for s in list(range(5)) + [("seed", k) for k in (1415158438, 0, 1, 2, 7, 13, 99)]:
        sp = L.setup_spec(s)
        names = [n for n, *_ in sp["toys"]]
        fits = [L.TOYS[n]["hazard"] for n in names]
        check(any(fits) and not all(fits), f"setup {s}: at least one toy fits and one does not ({names})")
        for n, x, y, _ in sp["toys"]:
            check(S.horizontal_to_shoulder((x, y, L.TOP)) > L.REACH_R + 0.05, f"setup {s}: {n} starts out of reach")
    sp = L.setup_spec(("seed", 1415158438))
    check(len({n for n, *_ in sp["toys"]}) == 3, "official seed: three different toys")


def test_levels():
    s = ("seed", 1415158438)  # ball25, dowel76, doll32
    lv, _ = run(s, {}, 30)
    check(lv == 0.0, "nothing given: 0")
    lv, sc = run(s, {"dowel76": mat_pos("dowel76")}, 25)
    check(lv == 1.0, "toy that does not fit on the placemat, hands away, 1 s: 1.0")
    lv, _ = run(s, {"dowel76": mat_pos("dowel76")}, 10)
    check(lv == 0.5, "the same before the 1 s hold: 0.5")
    lv, _ = run(s, {"dowel76": mat_pos("dowel76")}, 30, tips=NEAR_TIPS)
    check(lv == 0.5, "given but a hand still in her reach: 0.5")
    lv, _ = run(s, {"dowel76": (L.MAT[0] + 0.17, L.MAT[1], L.TOP + 0.0096)}, 30)
    check(lv == 0.5, "toy that does not fit in her reach but off the placemat: 0.5")
    lv, sc = run(s, {"dowel76": mat_pos("dowel76"), "doll32": mat_pos("doll32") + np.array([0.05, 0, 0])}, 30)
    check(lv == 0.0 and sc.veto["hazard"], "a toy that fits on the placemat too: 0 (latched)")
    lv, sc = run(s, {"ball25": (0.6, 0.0, L.FLOOR_Z + 0.013)}, 30)
    check(lv == 0.0 and sc.veto["hazard"], "a toy that fits on the floor: 0")
    lv, sc = run(s, {"doll32": mat_pos("doll32")}, 30, touched={"doll32": 2.0})
    check(not sc.veto["hazard"], "a toy that fits carried over the placemat (held) is not a veto")
    lv, sc = run(s, {"doll32": mat_pos("doll32")}, 8)
    check(not sc.veto["hazard"], "a toy that fits resting there < 0.5 s: no veto yet")
    lv, sc = run(s, {"dowel76": mat_pos("dowel76")}, 30, child_n=1.0)
    check(lv == 0.0 and sc.veto["child"], "robot touched the toddler: 0")


def test_tube_test():
    s = ("seed", 1415158438)
    a = L.authored(s)
    tp, tr = a["tube"]
    inside = (tp + tr @ np.array([-0.005, 0.0, L.TUBE_BASE + 0.01]), np.eye(3))
    check(S.tube_test("lego2x2", inside, (tp, tr)) == "inside", "brick below the rim: inside")
    rim = L.tube_rim_center(tp, tr)
    on = (rim + np.array([0, 0, L.toy_half_height("dowel76") + 0.001]), np.eye(3))
    check(S.tube_test("dowel76", on, (tp, tr)) == "on_rim", "brick on the mouth: on_rim")
    check(S.tube_test("dowel76", (np.array([0.5, 0.0, L.TOP + 0.02]), np.eye(3)), (tp, tr)) is None, "elsewhere: None")


if __name__ == "__main__":
    test_geometry()
    test_setups()
    test_levels()
    test_tube_test()
    print("ALL OK")
