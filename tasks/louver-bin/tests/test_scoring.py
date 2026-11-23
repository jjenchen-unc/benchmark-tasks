"""Unit tests of the louver-bin geometry and scorer (numpy only; no simulator).

    python -m pytest tests/test_scoring.py   (from the task folder)
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
import layout as L  # noqa: E402
import scoring as S  # noqa: E402

FAR = [np.array([0.0, 0.0, 2.0])]


def target_pose(spec, row=None, y=None):
    y = L.half_centre_y(spec, -spec["side"]) if y is None else y
    return L.hung_pose(spec, spec["row"] if row is None else row, y)


def lvl(spec, pb, rb, tips=FAR, still=S.HOLD_STEPS):
    m = S.measure(spec, pb, rb, *L.ref_pose(spec), tips)
    return S.level_of(m, m.tip_mm >= S.CLEAR_M * 1e3, still)


def test_reference_bin_hangs_at_every_setup():
    for s in [0, 1, 2] + [("seed", k) for k in range(50)]:
        spec = L.setup_spec(s)
        st = L.hang_state(spec, *L.ref_pose(spec))
        assert S.hangs_on(st, spec["row"]), (s, st)


def test_full_score_needs_same_row_other_half_released_and_still():
    spec = L.setup_spec(0)
    pb, rb = target_pose(spec)
    assert lvl(spec, pb, rb) == 1.0
    assert lvl(spec, pb, rb, still=S.HOLD_STEPS - 1) == 0.5
    near = [pb + np.array([-0.1, 0.0, 0.0])]
    assert lvl(spec, pb, rb, tips=near) == 0.5


def test_wrong_row_scores_zero():
    spec = L.setup_spec(0)
    for d in (-1, 1):
        pb, rb = target_pose(spec, row=spec["row"] + d)
        assert lvl(spec, pb, rb) == 0.0


def test_tab_on_lip_or_in_front_scores_zero():
    spec = L.setup_spec(0)
    pb, rb = target_pose(spec)
    assert lvl(spec, pb + [-0.004, 0, L.TAB_DROP], rb) == 0.0  # tab resting on the lip's top edge
    assert lvl(spec, pb + [-0.012, 0, 0.0], rb) == 0.0  # tab in front of the lip
    assert lvl(spec, pb + [0, 0, 0.009], rb) == 0.0  # tab only 3 mm into the channel (needs 4 mm)


def test_tilted_bin_scores_zero():
    spec = L.setup_spec(0)
    pb, rb = target_pose(spec)
    from scipy.spatial.transform import Rotation

    r = Rotation.from_euler("y", 6.0, degrees=True).as_matrix()
    assert lvl(spec, pb, r) == 0.0


def test_same_half_as_reference_or_overlap_scores_zero():
    spec = L.setup_spec(0)
    pr, _ = L.ref_pose(spec)
    pb, rb = target_pose(spec, y=float(pr[1]) - spec["side"] * 0.10)  # overlaps the hanging bin
    assert lvl(spec, pb, rb) == 0.0


def test_reference_knocked_off_scores_zero():
    spec = L.setup_spec(0)
    pb, rb = target_pose(spec)
    pr, rr = L.ref_pose(spec)
    m = S.measure(spec, pb, rb, pr + [-0.02, 0, -0.05], rr, FAR)
    assert S.level_of(m, True, S.HOLD_STEPS) == 0.0


def test_official_seed_is_not_an_extreme_row():
    spec = L.setup_spec(("seed", 1415158438))
    assert spec["row"] == 3


def test_no_other_start_near_the_official_panel():
    off = L.setup_spec(("seed", L.OFFICIAL_SEED))["panel_dx"]
    for k in range(3000):
        assert abs(L.setup_spec(("seed", k))["panel_dx"] - off) >= L.PANEL_DX_SEP - 1e-12


def test_panel_never_near_nominal():
    for k in range(500):
        assert abs(L.setup_spec(("seed", k))["panel_dx"]) >= L.PANEL_DX_MIN


def test_no_other_start_near_the_official_relative_bin_x():
    off = L.setup_spec(("seed", L.OFFICIAL_SEED))
    rel_off = off["bin_dx"] - off["panel_dx"]
    for k in range(5000):
        sp = L.setup_spec(("seed", k))
        assert abs((sp["bin_dx"] - sp["panel_dx"]) - rel_off) >= L.REL_SEP - 1e-12
        assert abs(sp["bin_dx"]) <= L.JITTER["bin_dx"] + 1e-12


def test_evidence_seeds_keep_their_starts():
    # starts of the seeds used in the evidence runs (row, side, panel_dx, bin_dx), fixed when the bands were set
    want = {1415158438: (3, -1, -0.02324, 0.00379), 11: (4, 1, None, None), 1: (2, -1, None, None)}
    for seed, (row, side, pdx, bdx) in want.items():
        sp = L.setup_spec(("seed", seed))
        assert (sp["row"], sp["side"]) == (row, side)
        if pdx is not None:
            assert abs(sp["panel_dx"] - pdx) < 1e-5 and abs(sp["bin_dx"] - bdx) < 1e-5
