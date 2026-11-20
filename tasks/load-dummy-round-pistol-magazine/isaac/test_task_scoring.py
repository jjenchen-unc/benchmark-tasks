"""Unit tests of task_scoring.py on synthetic states (no simulator).  python -m pytest -q test_task_scoring.py"""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import task_scoring as ts  # noqa: E402

DT = 0.05
FWD = (1.0, 0.0, 0.0)


def seated_centre(gap=0.0003, e=0.0, dz=-0.0002):
    """Centre of a level round, bullet forward, base `gap` ahead of the rear wall, `dz` from the lip line."""
    z = ts.z_ret(e) + dz
    return (ts.x_rear(z + ts.R_CASE) + gap + ts.OAL / 2, e, z)


def m0(**kw):
    """A fully loaded state: round seated, tool parked 130 mm away, hands 138 mm away (as the demo ends)."""
    m = dict(round_c=seated_centre(), round_a=FWD, finger_round_m=0.138, tool_rel=(-0.057, 0.048, 0.108),
             tool_contact_n=0.0, through_gap=False, mag_shift_m=0.0, mag_tilt_deg=0.0, fell=False)
    m.update(kw)
    return m


def run(seq):
    """seq: list of (seconds, m). Returns the final score and the scorer."""
    sc = ts.LoadScorer()
    for secs, m in seq:
        for _ in range(max(1, round(secs / DT))):
            sc.step(DT, m)
    return sc.score, sc


def axis(yaw_deg=0.0, tilt_deg=0.0):
    y, t = math.radians(yaw_deg), math.radians(tilt_deg)
    return (math.cos(t) * math.cos(y), math.cos(t) * math.sin(y), math.sin(t))


def test_geometry_constants():
    assert ts.Z_RET0 * 1000 == pytest.approx(110.86, abs=0.01)
    assert ts.x_rear(0.110) == pytest.approx(-0.016)
    assert ts.OAL * 1000 == pytest.approx(29.16, abs=0.01)
    assert ts.Z_TIP * 1000 == pytest.approx(114.89)


def test_logged_loaded_state_is_loaded():
    # final state of a loaded development run: gap -0.57, centre (0.1, -0.16, 110.47) mm, axis (1.0, 0.001, 0.0)
    r = ts.round_state((0.0001, -0.00016, 0.11047), (1.0, 0.001, 0.0))
    assert r["loaded"]
    assert r["gap"] * 1000 == pytest.approx(-0.57, abs=0.05)
    assert r["dz"] * 1000 == pytest.approx(-0.07, abs=0.05)


def test_pass():
    s, sc = run([(2.1, m0())])
    assert s == 1.0 and sc.veto is None


def test_needs_two_seconds():
    assert run([(1.9, m0())])[0] == 0.0


def test_idle():
    s, _ = run([(5, m0(round_c=(0.12, -0.25, 0.005), round_a=(0.0, -1.0, 0.0), tool_rel=(0.06, 0.18, -0.05)))])
    assert s == 0.0


@pytest.mark.parametrize("kw", [
    dict(round_c=seated_centre(gap=0.0025)),  # base 2.5 mm from the rear wall: retained, not loaded
    dict(round_c=seated_centre(dz=-0.0012)),  # 1.2 mm below the lip line
    dict(round_a=axis(yaw_deg=6.0)),  # yaw 6 deg
    dict(round_a=axis(tilt_deg=11.0)),  # tilt 11 deg: not retained
])
def test_round_condition_fails_give_partial(kw):
    m = m0(**kw)
    r = ts.round_state(m["round_c"], m["round_a"])
    assert not r["loaded"]
    assert run([(2.1, m)])[0] == (0.25 if r["retained"] else 0.0)


def test_retained_not_seated_is_quarter():
    assert run([(2.1, m0(round_c=seated_centre(gap=0.006)))])[0] == 0.25


def test_laid_on_top_of_lips():
    c = seated_centre()
    assert run([(2.1, m0(round_c=(c[0], 0.0, ts.Z_TIP + 0.0026)))])[0] == 0.0


def test_reversed_round():
    c = seated_centre()
    assert ts.round_state(c, (-1.0, 0.0, 0.0))["reversed"]
    assert run([(2.1, m0(round_a=(-1.0, 0.0, 0.0)))])[0] == 0.0


def test_case_mouth_rule_binds_before_the_2mm_gap():
    # level centred round: the case mouth reaches the lip-ramp end at a base gap of about 0.61 mm
    assert ts.round_state(seated_centre(gap=0.0006), FWD)["loaded"]
    r = ts.round_state(seated_centre(gap=0.0007), FWD)
    assert r["mouth"] >= ts.X_RAMP1 and r["retained"] and not r["loaded"]


def test_base_ahead_of_constant_lips_gets_nothing():
    c = seated_centre(gap=0.013)  # base ahead of the constant-gap end: only on the lip ramp
    r = ts.round_state(c, FWD)
    assert r["base"][0] >= ts.X_LIP1 and not r["retained"]
    assert run([(2.1, m0(round_c=c))])[0] == 0.0


def test_through_gap_vetoes():
    c = seated_centre()
    assert ts.through_gap_now((c[0], 0.0, ts.Z_RET0 + 0.002), FWD)
    s, sc = run([(0.5, m0(through_gap=True)), (3, m0())])
    assert s == 0.0 and "lip gap" in sc.veto


def test_through_gap_not_triggered_by_normal_entry_or_carry():
    c = seated_centre()
    assert not ts.through_gap_now(c, FWD)  # seated
    assert not ts.through_gap_now((c[0] + 0.012, 0.0, ts.Z_RET0 + 0.002), FWD)  # base still ahead of the constant lips
    assert not ts.through_gap_now((c[0], 0.0, ts.Z_TIP + 0.02), FWD)  # carried above the mouth
    assert not ts.through_gap_now((c[0], 0.05, ts.Z_RET0 + 0.002), FWD)  # beside the magazine
    # tilted entry under the lips from the front (logged development run): base under the lips, centre ahead of them
    assert not ts.through_gap_now((0.011186, -0.000049, 0.11239), axis(yaw_deg=-0.2, tilt_deg=14.5))


def test_laid_on_lips_counts_as_in_the_gap():
    c = seated_centre()
    assert ts.through_gap_now((c[0], 0.0, ts.Z_TIP + 0.0026), FWD)


def test_loaded_with_tool_left_on_is_quarter():
    assert run([(2.1, m0(tool_rel=(0.0, 0.0, 0.0), tool_contact_n=30.0))])[0] == 0.25


def test_tool_touching_but_lifted():
    assert run([(2.1, m0(tool_rel=(0.0, 0.0, 0.08), tool_contact_n=1.0))])[0] == 0.25


def test_tool_free_but_too_close():
    assert run([(2.1, m0(tool_rel=(0.0, 0.0, 0.060)))])[0] == 0.25


def test_no_half_level():
    for kw in (dict(), dict(tool_rel=(0.0, 0.0, 0.0), tool_contact_n=30.0), dict(round_c=seated_centre(gap=0.006))):
        assert ts.level(m0(**kw)) in (0.0, 0.25, 1.0)


def test_gripper_still_holding():
    assert run([(5, m0(finger_round_m=0.012))])[0] == 0.0
    assert run([(5, m0(finger_round_m=0.049))])[0] == 0.0


def test_gripper_holding_unseated_round_gets_nothing():
    assert run([(5, m0(round_c=seated_centre(gap=0.006), finger_round_m=0.01))])[0] == 0.0


def test_round_moves_within_two_seconds():
    s, _ = run([(1.0, m0()), (1.5, m0(round_c=seated_centre(gap=0.006)))])
    assert s == 0.25
    s, _ = run([(2.5, m0()), (0.1, m0(round_c=(0.12, -0.25, 0.005)))])
    assert s == 0.0


def test_magazine_knocked():
    s, sc = run([(0.2, m0(mag_shift_m=0.004)), (3, m0())])
    assert s == 0.0 and "magazine" in sc.veto
    assert run([(3, m0(mag_tilt_deg=2.5))])[0] == 0.0
    assert run([(3, m0(mag_shift_m=0.0015, mag_tilt_deg=1.5))])[0] == 1.0


def test_object_fell():
    s, sc = run([(0.1, m0(fell=True)), (3, m0())])
    assert s == 0.0 and "fell" in sc.veto


def test_veto_is_latched():
    s, _ = run([(3, m0()), (0.05, m0(through_gap=True)), (3, m0())])
    assert s == 0.0
