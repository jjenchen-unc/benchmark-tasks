"""Unit tests of the flask-clamp layout, seat rule and scorer (numpy only; run with python -m pytest or directly)."""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

import layout as L  # noqa: E402
import numpy as np  # noqa: E402
import scoring as S  # noqa: E402

GOOD_REL = dict(dx=0.0, dy=0.0, dz=L.SEAT_Z, radial=0.0, tilt_deg=0.3)
GOOD_TH = [0.11] * 5
GOOD_N = [1.5] * 5


def m(seated=True, released=True, still=True):
    return S.Measure(seated=seated, released=released, still=still, detail={})


def test_seated_good():
    ok, d = S.seated_detail(GOOD_REL, GOOD_TH, GOOD_N)
    assert ok and d["down"] and all(d["fingers_on"])


def test_perched_is_not_seated():
    rel = dict(GOOD_REL, dz=0.049)
    ok, d = S.seated_detail(rel, [0.30] * 5, GOOD_N)
    assert not ok and not d["down"] and not any(d["fingers_closed"])


def test_tilted_or_off_axis_is_not_seated():
    assert not S.seated_detail(dict(GOOD_REL, tilt_deg=3.0), GOOD_TH, GOOD_N)[0]
    assert not S.seated_detail(dict(GOOD_REL, radial=0.004), GOOD_TH, GOOD_N)[0]


def test_one_finger_off_is_not_seated():
    n = list(GOOD_N)
    n[2] = 0.0
    assert not S.seated_detail(GOOD_REL, GOOD_TH, n)[0]
    th = list(GOOD_TH)
    th[4] = 0.30  # spread over the heel
    assert not S.seated_detail(GOOD_REL, th, GOOD_N)[0]


def test_scorer_full_sequence():
    sc = S.Scorer()
    for _ in range(S.STILL_STEPS - 1):
        assert sc.step(m()) == 0.5
    sc.step(m())
    assert sc.shaking
    for _ in range(S.shake_steps() - 1):
        assert sc.step(m()) == 0.5
    assert sc.step(m()) == 1.0
    assert sc.passed


def test_scorer_shake_knocks_out():
    sc = S.Scorer()
    for _ in range(S.STILL_STEPS):
        sc.step(m())
    sc.step(m(seated=False))
    for _ in range(S.shake_steps()):
        sc.step(m())
    assert not sc.passed and sc.shakes_failed == 1


def test_scorer_held_is_half():
    sc = S.Scorer()
    for _ in range(200):
        lv = sc.step(m(released=False))
    assert lv == 0.5 and not sc.shaking


def test_scorer_hand_back_stops_shake():
    sc = S.Scorer()
    for _ in range(S.STILL_STEPS):
        sc.step(m())
    assert sc.shaking
    sc.step(m(released=False))
    assert not sc.shaking and not sc.passed


def test_not_seated_is_zero():
    sc = S.Scorer()
    for _ in range(200):
        lv = sc.step(m(seated=False))
    assert lv == 0.0


def test_shake_amplitude_profile():
    assert S.shake_amplitude(0.0) == 0.0
    assert abs(S.shake_amplitude(0.25) - 0.5) < 1e-9
    assert S.shake_amplitude(1.0) == 1.0
    assert S.shake_amplitude(10.0) == 0.0


def test_seated_finger_geometry():
    """At the seated hinge angle the pad rests on the cone; at rest it is inside it (preload); open, the heel passes."""
    r0, z0 = L.finger_pad_radius(0.0)
    assert r0 < L.flask_radius(z0 - L.SEAT_Z)
    r1, z1 = L.finger_pad_radius(S.THETA_MAX)
    assert r1 > L.flask_radius(z1 - L.SEAT_Z)
    r2, _ = L.finger_pad_radius(math.radians(L.FINGER_LIMITS_DEG[1]))
    assert r2 > L.F_BASE_R
    assert L.TIP[0] > L.F_BASE_R  # flared tip wider than the heel: the heel lands inside the flare


def test_official_seed_and_separation():
    off = L.setup_spec(("seed", L.OFFICIAL_SEED))
    for s in range(3000):
        sp = L.setup_spec(("seed", s))
        side = L.socket_side(sp["socket"])
        assert L.FLASK_X[0] - 1e-9 <= sp["flask_x"] <= L.FLASK_X[1] + 1e-9
        assert L.FLASK_ABS_Y[0] - 1e-9 <= sp["flask_y"] * side <= L.FLASK_ABS_Y[1] + 1e-9
        # at least 30 mm from the official flask start (same free clamp) or from its mirror (other free clamp)
        ry = off["flask_y"] if sp["socket"] == off["socket"] else -off["flask_y"]
        d = math.hypot(sp["flask_x"] - off["flask_x"], sp["flask_y"] - ry)
        assert d >= L.OFFICIAL_MIN_SEP - 1e-9


def test_point_to_flask():
    pf, rf = np.zeros(3), np.eye(3)
    assert abs(L.point_to_flask([0.1, 0, 0.05], pf, rf) - (0.1 - L.flask_radius(0.05))) < 1e-9
    assert L.point_to_flask([0, 0, 0.2], pf, rf) > 0.05


if __name__ == "__main__":
    fails = 0
    for k, f in list(globals().items()):
        if k.startswith("test_"):
            try:
                f()
                print("PASS", k)
            except AssertionError as e:
                fails += 1
                print("FAIL", k, e)
    sys.exit(1 if fails else 0)
