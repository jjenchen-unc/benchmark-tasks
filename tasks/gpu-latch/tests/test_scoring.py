"""Unit tests of the gpu-latch layout and scorer (numpy only; run with python -m pytest or directly)."""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

import layout as L  # noqa: E402
import numpy as np  # noqa: E402
import scoring as S  # noqa: E402


def m(out=True, on_mat=True, released=True, still=True, hook_n=0.0, fall_v=0.0, forced=False, descent=0.0):
    return S.Measure(out=out, on_mat=on_mat, released=released, still=still, hook_n=hook_n, fall_v=fall_v,
                     forced=forced, detail={}, unsupported_descent_m=descent)


def test_full_score_needs_hold():
    sc = S.Scorer()
    for k in range(S.HOLD_STEPS - 1):
        assert sc.step(m()) == 0.5
    assert sc.step(m()) == 1.0
    assert sc.step(m(out=False)) == 1.0  # passed is final (the trial ends)


def test_seated_is_zero():
    sc = S.Scorer()
    for _ in range(30):
        assert sc.step(m(out=False, on_mat=False, released=True)) == 0.0


def test_out_but_held_is_half():
    sc = S.Scorer()
    for _ in range(30):
        assert sc.step(m(on_mat=False, released=False)) == 0.5
    sc = S.Scorer()
    for _ in range(30):
        assert sc.step(m(released=False)) == 0.5  # on the mat but a hand on it


def test_load_and_supported_motion_alone_do_not_fail():
    sc = S.Scorer()
    assert sc.step(m(out=False, hook_n=100)) == 0
    assert sc.step(m(on_mat=False, released=False, fall_v=1.1)) == 0.5
    for _ in range(S.HOLD_STEPS):
        sc.step(m())
    assert sc.level == 1


def test_rear_wall_and_opening_constrain_a_low_bracket():
    def hits(u, n, z):
        return any(a <= u <= b and c <= n <= d and e <= z <= f
                   for a,b,c,d,e,f in L.CASE_WALLS.values())
    assert hits(-13, 12, 100)
    assert not hits(-13, 0, 100)
    assert not hits(-13, 12, L.CASE_RIM + 2)


def test_forced_past_closed_latch_is_zero():
    sc = S.Scorer()
    sc.step(m(out=False, forced=True))
    for _ in range(30):
        assert sc.step(m()) == 0.0


def test_dropped_is_zero_forever():
    sc = S.Scorer()
    sc.step(m(on_mat=False, released=True, fall_v=0.8, descent=0.035))
    for _ in range(30):
        assert sc.step(m()) == 0.0


def test_not_still_or_not_flat_never_full():
    sc = S.Scorer()
    for _ in range(40):
        assert sc.step(m(still=False)) == 0.5


def test_seated_card_geometry():
    spec = L.setup_spec(0)
    ps, rs = L.bench_frame(spec)
    assert abs(L.finger_bottom_rise(ps, rs, ps, rs)) < 1e-9
    assert L.in_slot(ps, rs, ps, rs) and L.engaged(ps, rs, ps, rs)
    up = ps + rs[:, 2] * 0.0096
    assert not L.in_slot(up, rs, ps, rs) and not L.engaged(up, rs, ps, rs)
    assert L.in_slot(ps + rs[:, 2] * 0.0090, rs, ps, rs)
    # the slot frame: u forward and n to the left at yaw 0
    assert np.allclose(rs[:, 0], [1, 0, 0]) and np.allclose(rs[:, 1], [0, 1, 0])


def test_hook_overlaps_lip_when_closed_and_clears_when_open():
    p = L.latch_pieces()["hook"] / L.MM  # latch frame, mm
    tip_u = p[:, 0].min() + L.PIVOT[0]
    assert tip_u < L.LIP[1]  # closed: the hook is over the lip
    th = math.radians(L.HOOK_CLEAR_DEG)
    pts = p.copy()
    u = pts[:, 0] * math.cos(th) + pts[:, 2] * math.sin(th) + L.PIVOT[0]
    assert u.min() >= L.LIP[1] - 0.05  # open past HOOK_CLEAR_DEG: clear of the lip
    assert L.LATCH_LIMIT_DEG[1] > L.HOOK_CLEAR_DEG


def test_withdrawal_force_within_reference_connector_test_range():
    # Molex TS-87715-207 section 9.2, withdrawal of a 164-contact blade.
    assert 1.95 * 9.80665 <= L.WITHDRAW_N <= 2.14 * 9.80665


def test_official_seed_unpushed_and_guard_holds():
    off = L.setup_spec(("seed", L.OFFICIAL_SEED))
    raw = L._raw_spec(L.OFFICIAL_SEED)
    assert off["dx"] == raw["dx"] and off["dy"] == raw["dy"]
    for s in range(2000):
        if s == L.OFFICIAL_SEED:
            continue
        sp = L.setup_spec(("seed", s))
        assert L.guard_metric(sp) >= L.GUARD_MARGIN - 1e-9
        a, c = L.pad_offset(sp)
        assert c <= -L.GUARD_MARGIN * L.GUARD_N_FAR + 1e-9 or c >= L.GUARD_MARGIN * L.GUARD_N_ROBOT - 1e-9 or \
            abs(a) >= L.GUARD_MARGIN * L.GUARD_U - 1e-9


def test_start_from_offset():
    for a, c, y in ((0, 30, 0), (-4, 45, 2), (5, 45, -2)):
        sp = L.start_from_offset(a, c, y)
        a2, c2 = L.pad_offset(sp)
        assert abs(a2 * 1e3 - a) < 1e-6 and abs(c2 * 1e3 - c) < 1e-6


def test_extracted_card_must_clear_case_before_sideways_transfer():
    walls = [L.ubox(*bounds) for bounds in L.CASE_WALLS.values()]

    def intersects_case(delta):
        for part in L.card_pieces().values():
            for wall in walls:
                overlap = np.minimum(part.max(0) + delta, wall.max(0)) - np.maximum(part.min(0) + delta, wall.min(0))
                if np.all(overlap > 0.0001):
                    return True
        return False

    assert not intersects_case([0, 0, 0])
    assert not intersects_case([0, 0, 0.012])
    assert intersects_case([0, -0.083, 0.012])
    assert not intersects_case([0, -0.083, L.CASE_RIM * L.MM])


def test_mat_clear_of_tower_for_entire_reset_envelope():
    # Bound every rotated floor corner, including intermediate yaw angles.
    tower_min_y = L.BENCH_XY[1] - L.JIT_Y - 0.406 * math.sin(math.radians(L.JIT_YAW)) - 0.092
    assert L.MAT_CENTER[1] + L.MAT_HALF[1] < tower_min_y

def test_placement_requires_mat_load_and_clearance_from_every_robot_link():
    vertices = np.array([[-.1, -.05, .0], [.1, .05, .02]])
    args = (vertices, vertices.mean(axis=0), np.zeros(6), np.zeros(2), np.array([.2, .1]), 0., 1.)
    assert S.placement(*args, 4.9, False, .050)[2:] == (True, 0.0, True)
    assert not S.placement(*args, 0., False, .050)[2]
    assert not S.placement(*args, 4.9, True, .050)[4]
    assert not S.placement(*args, 4.9, False, .0)[4]


def test_rotating_about_a_stationary_com_does_not_count_as_still():
    vertices = np.array([[-.1, -.05, .0], [.1, .05, .02]])
    result = S.placement(vertices, vertices.mean(axis=0), np.array([0., 0., 0., 0., 0., 1.]),
                         np.zeros(2), np.array([.2, .1]), 0., 1., 4.9, False, .05)
    assert result[2] and result[3] > S.STILL_V


def test_bracket_overhanging_mat_counts_as_outside():
    orientation = L.rz(0.0)
    body = L.card_corners(np.zeros(3), orientation)
    vertices = L.card_vertices(np.zeros(3), orientation)
    center = (body.max(0)[:2] + body.min(0)[:2]) / 2
    half = (body.max(0)[:2] - body.min(0)[:2]) / 2 + .002
    assert np.all(np.abs(body[:, :2] - center) <= half)
    assert not S.placement(vertices, np.zeros(3), np.zeros(6), center, half, 0., 1., 4.9, False, .05)[0]

if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)


