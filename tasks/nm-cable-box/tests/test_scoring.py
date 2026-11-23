"""Unit tests of the nm-cable-box geometry and scorer (numpy only): python -m pytest tests/ (from the task folder)."""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
import layout as L  # noqa: E402
import scoring as S  # noqa: E402


def chain(points):
    """Link poses along a polyline sampled from the cut end (points[0]) backward, one link per layout length."""
    pts = [np.asarray(p, float) for p in points]
    ls = L.link_lengths()

    def at(s):
        acc = 0.0
        for a, b in zip(pts[:-1], pts[1:]):
            seg = float(np.linalg.norm(b - a))
            if acc + seg >= s:
                return a + (b - a) * (s - acc) / seg, (a - b) / seg
            acc += seg
        a, b = pts[-2], pts[-1]
        return b + (b - a) / np.linalg.norm(b - a) * (s - acc), (a - b) / np.linalg.norm(b - a)

    links, s = [], 0.0
    for ln in ls:
        p, ax = at(s + ln / 2)
        w = np.cross([0.0, 0.0, 1.0], ax)
        if np.linalg.norm(w) < 1e-6:
            w = np.array([-1.0, 0.0, 0.0])
        w /= np.linalg.norm(w)
        links.append((p, np.column_stack([ax, w, np.cross(ax, w)])))
        s += ln
    return links


SPEC = L.setup_spec(0)
EC = L.entry_centre(SPEC)
Y_IN = L.entry_wall_y(SPEC)
ZC = L.ENTRY_Z0 + 0.0035


def drawn(inside, rise=True):
    """Cable through the opening with `inside` metres past the wall's inner face (horizontal 40 mm, then up)."""
    h = min(inside, 0.040)
    a = np.array([EC[0], Y_IN, ZC])
    knee = a + [0, h, 0]
    tip = knee + ([0, 0, inside - h] if rise else [0, inside - h, 0])
    tail = a + [0, -0.6, 0]
    return chain([tip, knee, a, tail])


def test_inserted_length_matches_geometry():
    for ins in (0.03, 0.10, 0.15, 0.19):
        v = L.inserted_length(drawn(ins), SPEC)
        assert v is not None and abs(v - ins) < 0.002, (ins, v)


def test_not_through_when_outside_window():
    links = chain([np.array([EC[0] + 0.03, Y_IN + 0.05, ZC]), np.array([EC[0] + 0.03, Y_IN - 0.6, ZC])])
    assert L.inserted_length(links, SPEC) is None  # crosses the wall plane outside the opening (through the wall)
    rest = L.cable_rest(SPEC)
    assert L.inserted_length(rest, SPEC) is None


def run_scorer(links, tab, tips, n=30, v=0.0):
    sc = S.Scorer()
    lv = 0.0
    for _ in range(n):
        lv = sc.step(S.measure(SPEC, links, tab, tips), v)
    return lv, sc


FAR = [np.array([0.2, 0.3, 0.9])] * 4


def test_full_score_at_150():
    lv, _ = run_scorer(drawn(0.150), 75.0, FAR)
    assert lv == 1.0


def test_band_edges():
    assert run_scorer(drawn(0.140), 75.0, FAR)[0] == 1.0
    assert run_scorer(drawn(0.160), 75.0, FAR)[0] == 1.0
    assert run_scorer(drawn(0.120), 75.0, FAR)[0] == 0.5
    assert run_scorer(drawn(0.185), 75.0, FAR)[0] == 0.5


def test_tab_must_be_lifted():
    assert run_scorer(drawn(0.150), S.TAB_THROUGH_DEG - 2.0, FAR)[0] == 0.0


def test_held_scores_zero():
    links = drawn(0.150)
    tip = L.cable_points(links)[0]
    assert run_scorer(links, 75.0, [tip] * 4)[0] == 0.0


def test_moving_scores_zero_and_hold_needed():
    assert run_scorer(drawn(0.150), 75.0, FAR, v=0.02)[0] == 0.0
    assert run_scorer(drawn(0.150), 75.0, FAR, n=S.HOLD_STEPS - 1)[0] == 0.0


def test_rest_scores_zero():
    assert run_scorer(L.cable_rest(SPEC), L.TAB_REST_DEG, FAR)[0] == 0.0


def test_seeded_box_offsets_exclusion_band():
    for s in range(2000):
        sp = L.setup_spec(("seed", s))
        assert abs(sp["box_dx"]) >= L.BOX_DX_MIN - 1e-12
        assert L.JITTER["gap"][0] <= sp["gap"] <= L.JITTER["gap"][1]
    sp = L.setup_spec(("seed", 1415158438))
    assert abs(sp["box_dx"]) >= L.BOX_DX_MIN - 1e-12


def test_service_loop_descends_to_upright_box():
    for s in range(50):
        sp = L.setup_spec(("seed", s))
        for p, r in L.cable_rest(sp):
            assert L.world_point(p)[2] > L.TOP
            assert np.allclose(L.fixture_point(L.world_point(p)),p)
            assert abs(np.linalg.det(r) - 1.0) < 1e-9
        first=L.cable_rest(sp)[0][1]
        assert (L.FIXTURE_ROT @ first[:,0])[2] < -0.99
        assert np.allclose(L.FIXTURE_ROT @ [0,0,1], [-1,0,0])
        assert math.isclose(sum(L.link_lengths()), L.CABLE_LEN)


def test_named_setups_far_from_default_box():
    ref = L.setup_spec(("seed", L.OFFICIAL_SEED))["box_dx"]
    for k in range(len(L.SETUPS)):
        assert abs(L.setup_spec(k)["box_dx"] - ref) >= 0.008, k


def test_seeded_box_dx_redrawn_no_point_mass():
    ref = L.setup_spec(("seed", L.OFFICIAL_SEED))["box_dx"]
    v = np.array([L.setup_spec(("seed", s))["box_dx"] for s in range(4000) if s != L.OFFICIAL_SEED])
    assert np.all(np.abs(v) >= L.BOX_DX_MIN - 1e-12)
    assert np.all(np.abs(v - ref) >= L.REPLAY_DX_MIN - 1e-12)
    counts, _ = np.histogram(v, bins=np.arange(-0.020, 0.0201, 0.001))
    assert counts.max() <= 0.08 * len(v)  # no point mass (uniform over the allowed 16.8 mm: about 6 % per mm)
    official = L.setup_spec(("seed", L.OFFICIAL_SEED))
    assert abs(official["box_dx"] - 0.008203509950991485) < 1e-12 and abs(official["gap"] - 0.08351342623901974) < 1e-12
