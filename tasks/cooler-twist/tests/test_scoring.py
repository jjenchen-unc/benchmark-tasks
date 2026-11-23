"""Unit tests for src/scoring.py and src/layout.py start states (pure Python + NumPy)."""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
import layout as L  # noqa: E402
from scoring import CLEAR_S, Scorer  # noqa: E402

DT = 0.05
NONE = dict(cooler=False, cpu=False, bench=False)
HELD = dict(cooler=True, cpu=False, bench=False)


def run(sc, n, **kw):
    lv = 0.0
    for _ in range(n):
        lv = sc.step(DT, kw.get("lift", 0.0), kw.get("parted", False), kw.get("mat", False), kw.get("contact", NONE))
    return lv


class TestScorer(unittest.TestCase):
    def test_noop_zero(self):
        self.assertEqual(run(Scorer(), 2000), 0.0)

    def test_full(self):
        sc = Scorer()
        run(sc, 40, contact=HELD)
        run(sc, 10, parted=True, contact=HELD)
        self.assertEqual(run(sc, 10, parted=True, mat=True, contact=HELD), 0.5)
        n = int(round(CLEAR_S / DT))
        self.assertEqual(run(sc, n - 1, parted=True, mat=True), 0.5)
        self.assertEqual(run(sc, 1, parted=True, mat=True), 1.0)

    def test_yank_vetoes_forever(self):
        sc = Scorer()
        run(sc, 5, lift=1.2, contact=HELD)
        self.assertEqual(sc.veto, "cpu_yanked")
        self.assertEqual(run(sc, 200, parted=True, mat=True), 0.0)

    def test_small_rise_not_yank(self):
        sc = Scorer()
        run(sc, 5, lift=0.6, contact=HELD)
        self.assertIsNone(sc.veto)

    def test_parted_not_on_mat_half(self):
        sc = Scorer()
        run(sc, 5, parted=True, contact=HELD)
        self.assertEqual(run(sc, 200, parted=False, mat=False), 0.5)  # parted latches

    def test_on_mat_needs_parted(self):
        self.assertEqual(run(Scorer(), 200, mat=True), 0.0)


class TestStarts(unittest.TestCase):
    def test_official_seed_ranges(self):
        sp = L.setup_spec(("seed", L.OFFICIAL_SEED))
        d = math.hypot(sp["bx"] - L.NOMINAL[0], sp["by"] - L.NOMINAL[1])
        self.assertTrue(L.BENCH_D[0] <= d <= L.BENCH_D[1])
        self.assertTrue(L.T_BREAK[0] <= sp["tb"] <= L.T_BREAK[1])

    def test_guard(self):
        ref = L.setup_spec(("seed", L.OFFICIAL_SEED))
        redrawn = 0
        for n in range(3000):
            sp = L.setup_spec(("seed", n))
            self.assertFalse(L.guarded(sp, ref), n)
            redrawn += sp != L._seeded(n)
        self.assertGreater(redrawn, 0)

    def test_paste_holds_more_than_zif(self):
        self.assertGreater(L.F_TENS, L.F_ZIF + 10.0)
        self.assertLess(L.T_BREAK[1] / L.PASTE_REFF, L.F_TENS)


def test_three_seconds_start_at_stable_placement():
    sc = Scorer()
    run(sc, 80, parted=True)
    for change in (dict(mat=False), dict(contact=HELD)):
        assert run(sc, 59, parted=True, mat=True) == 0.5
        run(sc, 1, **(dict(parted=True, mat=True) | change))
    assert run(sc, 59, parted=True, mat=True) == 0.5
    assert run(sc, 1, parted=True, mat=True) == 1.0


def test_board_outline_and_carrier():
    (xlo, xhi), (ylo, yhi) = L.BOARD
    assert math.isclose(xhi - xlo, 0.230)
    assert math.isclose(yhi - ylo, 0.201)
    for (tray_lo, tray_hi), (board_lo, board_hi) in zip(L.TRAY, L.BOARD):
        assert tray_lo < board_lo < board_hi < tray_hi


def test_tray_clears_mat_across_start_range():
    # At any supported bench yaw, a corner's displacement toward the mat is
    # bounded by the positive x extent plus the projected lateral extent.
    max_lateral = max(abs(y) for y in L.TRAY[1])
    cos_max = max(abs(math.cos(math.radians(yaw))) for yaw in L.BENCH_YAW)
    lowest_y = L.NOMINAL[1] - L.BENCH_D[1] - L.TRAY[0][1] - max_lateral * cos_max
    assert lowest_y > L.MAT[1][1]
