"""Unit tests of the pure scorer (python tests/test_scoring.py from the task folder, or pytest)."""
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / 'src'))
import layout as L  # noqa: E402
import belt_scoring as S  # noqa: E402

YM = L.motor_y(L.C_TAUT)
FAR = [np.array([0.2, 0.0, L.TOP + 0.4])] * 4
T_OK = 9.0


def taut():
    c, r = L.belt_taut()
    return c.copy(), r.copy()


def level(c, r, ym=YM, lever=0.0, t=T_OK, tips=FAR, touch=None):
    return S.instant_level(S.measure(c, r, ym, lever, t, tips, touch))


def test_taut_levels():
    c, r = taut()
    m = S.measure(c, r, YM, 0.0, T_OK, FAR)
    assert m['joint']['seated'] and m['motor']['seated'], (m['joint'], m['motor'])
    assert m['spans']['top']['straight'] and m['spans']['bottom']['straight'], m['spans']
    assert level(c, r) == 0.5
    assert level(c, r, lever=-80) == 0.75
    assert level(c, r, lever=-70) == 0.5  # not yet clamped


def test_tension_band():
    c, r = taut()
    assert level(c, r, lever=-80, t=6.0) == 0.25    # too slack
    assert level(c, r, lever=-80, t=13.0) == 0.25   # over-tensioned
    assert level(c, r, lever=-80, t=6.3) == 0.75 and level(c, r, lever=-80, t=12.3) == 0.75
    assert abs(L.span_deflection_mm(6.2) - 4.0) < 0.05 and abs(L.span_deflection_mm(12.4) - 2.0) < 0.05


def test_hold_to_full_score():
    c, r = taut()
    sc = S.BeltScorer()
    m = S.measure(c, r, YM, -80, T_OK, FAR)
    for _ in range(int(2.9 * 960)):
        sc.step(1 / 960, m)
    assert sc.score == 0.75
    for _ in range(int(0.2 * 960)):
        sc.step(1 / 960, m)
    assert sc.score == 1.0
    near = S.measure(c, r, YM, -80, T_OK, [c[3] + [0, 0, .01]])
    sc.step(1 / 960, near)
    assert sc.score == 0.75 and sc.hold_s == 0.0  # a fingertip back near the belt restarts the hold
    touch = S.measure(c, r, YM, -80, T_OK, FAR, {'belt_03': 0.4})
    assert not touch['released']


def test_start_on_joint_pulley_is_zero():
    c, r = L.belt_start()
    assert level(c, r, ym=L.motor_y(L.C_START), t=0.0) == 0.0


def test_riding_tooth_not_seated():
    for kind, centre in (('motor', (YM, L.AXIS_Z)), ('joint', (L.JOINT_Y, L.AXIS_Z))):
        c, r = taut()
        d = np.linalg.norm(c[:, 1:] - centre, axis=1)
        i = int(np.argmin(np.abs(d - L.r_hinge(kind) * np.cos(np.pi / (L.TEETH[kind] / 2)))
                          + 1e3 * (np.abs(c[:, 2] - L.AXIS_Z) > .004)))  # a link mid-wrap at the outer side
        out = c[i, 1:] - centre
        c[i, 1:] += out / np.linalg.norm(out) * 0.00075
        m = S.measure(c, r, YM, -80, T_OK, FAR)
        assert not m[kind]['seated'], (kind, m[kind])
        assert level(c, r, lever=-80) == 0.25


def test_belt_in_front_of_motor_flange():
    c, r = taut()
    near = np.linalg.norm(c[:, 1:] - [YM, L.AXIS_Z], axis=1) < 0.02
    c[near, 0] -= 0.006  # belt riding in front of the motor pulley's front flange
    assert level(c, r) == 0.0


def test_belt_on_joint_flange_rim_not_seated():
    """A stretch of the belt round the 60T's lower half lifted ~3 mm off the hinge circle (standing on edge in the
    flange gap), still between the flanges by its link centres: not seated."""
    c, r = taut()
    yz = c[:, 1:] - [L.JOINT_Y, L.AXIS_Z]
    d = np.linalg.norm(yz, axis=1)
    ang = np.degrees(np.arctan2(yz[:, 1], yz[:, 0]))
    on = (d < L.R_FLANGE['joint']) & (ang > -110) & (ang < 10)
    assert on.sum() >= 8
    c[on, 1:] += yz[on] / d[on, None] * 0.003
    m = S.measure(c, r, YM, -80, T_OK, FAR)
    assert m['joint']['between'] and not m['joint']['seated'], m['joint']
    assert level(c, r, lever=-80) == 0.25


def test_teeth_out_caps():
    c, r = taut()
    r[:, :, 2] *= -1  # teeth facing out (belt inside out); hinge geometry unchanged
    r[:, :, 1] *= -1
    m = S.measure(c, r, YM, -80, T_OK, FAR)
    assert not m['motor']['facing'] and level(c, r, lever=-80) == 0.25


def test_not_round_motor():
    c, r = taut()
    assert level(c, r, ym=YM - 0.05) == 0.0


def test_fall_veto():
    c, r = taut()
    sc = S.BeltScorer()
    c2 = c.copy()
    c2[:, 2] = L.TOP - 0.3
    sc.step(1 / 960, S.measure(c2, r, YM, -80, T_OK, FAR))
    sc.step(1 / 960, S.measure(c, r, YM, -80, T_OK, FAR))
    assert sc.vetoes and sc.score == 0.0


def test_sagging_span_not_straight():
    c, r = taut()
    top = (c[:, 2] > L.AXIS_Z + 0.004) & (np.abs(c[:, 1] - (L.JOINT_Y + YM) / 2) < 0.03)
    c[top, 2] -= 0.004
    m = S.measure(c, r, YM, -80, T_OK, FAR)
    assert not m['spans']['top']['straight']


def test_tangent_line():
    n, d = S.tangent_line((0., 0.), 2., (10., 0.), 1., upper=True)
    assert abs(np.array([0., 0.]) @ n - (d - 2.)) < 1e-9 and abs(np.array([10., 0.]) @ n - (d - 1.)) < 1e-9 and n[1] > 0


def test_winding():
    sq = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], float)
    assert abs(S.winding(sq, (0.5, 0.5))) == 1 and S.winding(sq, (2, 2)) == 0




def test_tension_model():
    z = L.C_TENSION_ZERO
    assert z >= L.C_TAUT                                         # the simulated belt runs taut at or after the real one
    assert S.belt_tension(False, z + .001) == 0.0                # not on the drive: no tension
    assert S.belt_tension(True, z - .0001) == 0.0                # slack
    t = S.belt_tension(True, z + .0009)                          # 0.9 mm past taut: 10 N/mm * 0.9 mm
    assert abs(t - L.PLATE_K / 2 * .0009) < 1e-9 and 8 < t < 9.5
    assert S.belt_tension(True, z + .01) == L.BELT_T_CAP
    c, r = taut()                                                # the belt's own taut geometry (at C_TAUT): measure()
    try:                                                         # computes the tension when it is round both pulleys
        L.C_TENSION_ZERO = L.C_TAUT
        m = S.measure(c, r, L.motor_y(L.C_TAUT + .0009), -80, None, FAR)
    finally:
        L.C_TENSION_ZERO = z
    assert m['band'] and 8 < m['tension_n'] < 9.5, m['tension_n']


if __name__ == '__main__':
    for k, f in list(globals().items()):
        if k.startswith('test_'):
            f()
            print('ok', k)
