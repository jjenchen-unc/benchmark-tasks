"""Unit checks of the gripper-sensor scorer and start-state sampler (numpy only; run with pytest or python -m pytest
from the task folder)."""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

import layout as L  # noqa: E402
import numpy as np  # noqa: E402
import scoring as S  # noqa: E402


def test_led_window():
    assert S.led(0.005, 0.005)
    assert S.led(0.0059, 0.005)
    assert S.led(0.0041, 0.005)
    assert not S.led(0.0061, 0.005)
    assert not S.led(0.0039, 0.005)


def test_levels():
    t = 0.008
    assert S.static_level(t, t, False) == 0.75
    assert S.static_level(t, t, True) == 1.0
    assert S.static_level(t + 0.0004, t, True) == 1.0
    assert (
        S.static_level(t + 0.0009, t, True) == 0.75
    )  # lit, but a 1.2 mm wider / narrower part would not light it
    assert S.static_level(t + 0.0015, t, True) == 0.0  # LED off on the part
    assert S.static_level(0.0005, t, True) == 0.0  # LED on with the jaws open
    assert S.static_level(L.STROKE, t, True) == 0.0  # end of stroke: off on the part


def test_half_credit_when_window_reaches_end_of_stroke():
    # part travel close to the full stroke: a switch on the part's side can still light at the empty position
    t = L.STROKE - 0.0015
    u = t + 0.0008
    c = S.cycle(u, t)
    assert c["on_part"] and c["off_open"] and not c["off_empty"]
    assert S.static_level(u, t, True) == 0.5


def test_locked_holds_push_unlocked_does_not():
    assert S.holds_push(True)
    assert not S.holds_push(False)


def test_scorer_needs_clear_hold():
    t = 0.006
    sc = S.Scorer()
    for _ in range(S.HOLD_STEPS - 1):
        assert sc.step(t, t, True, False, False) == 0.75
    assert sc.step(t, t, True, False, False) == 1.0
    assert sc.done
    sc = S.Scorer()
    for _ in range(3 * S.HOLD_STEPS):
        assert (
            sc.step(t, t, True, False, False, hands_clear=False) == 0.75
        )  # a hand still at the switch
    sc = S.Scorer()
    for _ in range(3 * S.HOLD_STEPS):
        assert sc.step(t, t, True, True, False) == 0.75  # override still held
    sc = S.Scorer()
    for _ in range(3 * S.HOLD_STEPS):
        assert sc.step(t, t, True, False, True) == 0.75  # still touching the switch
    sc = S.Scorer()
    for _ in range(3 * S.HOLD_STEPS):
        assert sc.step(t, t, False, False, False) == 0.75  # not locked


def test_seeded_starts_never_trivial():
    for seed in range(2000):
        sp = L.setup_spec(("seed", seed))
        assert L.PART_MIN_TRAVEL <= sp["travel"] <= L.PART_MAX_TRAVEL
        assert abs(sp["u0"] - sp["travel"]) >= L.SEEDED_GAP_M - 1e-12
        assert S.static_level(sp["u0"], sp["travel"], True) == 0.0
    sp = L.setup_spec(("seed", 1415158438))
    assert (
        abs(sp["u0"] - sp["travel"]) > 0.007
    )  # the official seed needs a long, LED-guided slide
    assert L.setup_spec(("seed", 5)) == L.setup_spec(("seed", 5))


def test_window_never_touches_open_or_empty():
    # every part travel leaves the whole window (and margin) clear of the open and the empty positions
    for t in np.linspace(L.PART_MIN_TRAVEL, L.PART_MAX_TRAVEL, 101):
        lo, hi = t - L.WINDOW / 2, t + L.WINDOW / 2
        assert lo - L.WINDOW / 2 > 0.0
        assert hi + L.WINDOW / 2 < L.STROKE + 1e-12


def test_best_fixed_position_rate():
    # open-loop: the best single switch-body position scores 1.0 on at most ~5 % of seeds (hidden sensing-point
    # offset +-10 mm and the +-0.4 mm tolerance), and not at the official seed
    sp = [L.setup_spec(("seed", s)) for s in range(4000)]
    best, best_b = 0.0, None
    for b in np.arange(-0.012, 0.024, 0.0005):
        r = float(
            np.mean(
                [S.static_level(b - p["sense"], p["travel"], True) >= 1.0 for p in sp]
            )
        )
        if r > best:
            best, best_b = r, b
    assert best <= 0.05
    o = L.setup_spec(("seed", 1415158438))
    assert S.static_level(best_b - o["sense"], o["travel"], True) == 0.0


def test_switch_body_on_groove_and_sensing_point_inside():
    for seed in range(2000):
        p = L.setup_spec(("seed", seed))
        c0 = p["u0"] + L.SW_SENSE_BASE + p["sense"] - L.SW_L / 2
        assert L.BODY_CENTER_RANGE[0] <= c0 <= L.BODY_CENTER_RANGE[1]
        assert 0.0 < L.SW_SENSE_BASE + p["sense"] < L.SW_L


def test_named_setups_nontrivial():
    for k in range(len(L.SETUPS)):
        sp = L.setup_spec(k)
        assert abs(sp["u0"] - sp["travel"]) >= L.SEEDED_GAP_M
