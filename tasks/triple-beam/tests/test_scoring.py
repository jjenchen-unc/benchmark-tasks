"""Unit checks of the triple-beam scorer and layout (no simulator): python -m pytest tests"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

import layout as L  # noqa: E402
import scoring  # noqa: E402

DT = 0.05


def riders(n500, n100, poise_g, off100=0.0):
    return dict(r500=L.notch_s("r500", n500), r100=L.notch_s("r100", n100) + off100, r10=L.s_of_grams("r10", poise_g))


def run(seconds, pointer, speed, r, on_pan=True, contact=None, m=373.8, sc=None):
    sc = sc or scoring.Scorer()
    lv = 0.0
    for _ in range(int(round(seconds / DT))):
        lv = sc.step(DT, pointer, speed, r, on_pan, contact or {}, m)
    return lv, sc


class TestLayout(unittest.TestCase):
    def test_lever_law(self):
        # a rider's full-scale moment equals the beam's capacity on the pan arm
        for k in L.RIDERS:
            self.assertAlmostEqual(L.RIDER_MASS[k] * L.SPAN, L.CAP[k] * L.A_PAN, places=12)

    def test_zero_balance(self):
        m = L.M_BEAM * L.X_CB + sum(L.RIDER_MASS[k] * L.S0 for k in L.RIDERS) - L.M_PAN * L.A_PAN
        self.assertAlmostEqual(m, 0.0, places=12)

    def test_reading(self):
        self.assertAlmostEqual(L.reading_g(riders(3, 7, 3.8)), 373.8, places=6)
        self.assertIsNone(L.reading_g(riders(3, 7, 3.8, off100=0.004)))
        self.assertAlmostEqual(L.imbalance_g(373.8, riders(3, 7, 3.8)), 0.0, places=6)

    def test_masses_never_fit_a_round_rule(self):
        for n in range(3000):
            m = L.setup_spec(("seed", n))["m"]
            self.assertTrue(120.0 <= m <= 480.0)
            self.assertTrue(1.0 - 1e-9 <= m % 10.0 <= 9.0 + 1e-9)
            self.assertGreaterEqual(abs(m % 10.0 - 5.0), L.FINE_MID_EXCL_G - 1e-9)

    def test_guard_against_default(self):
        ref = L.setup_spec(("seed", L.OFFICIAL_SEED))["m"]
        for n in range(5000):
            self.assertGreaterEqual(abs(L.setup_spec(("seed", n))["m"] - ref), L.GUARD_G - 1e-9)


class TestScorer(unittest.TestCase):
    def test_full(self):
        lv, sc = run(4.0, 0.1, 0.05, riders(3, 7, 3.8))
        self.assertEqual(lv, 1.0)

    def test_needs_rest_and_hands_off(self):
        lv, _ = run(1.0, 0.1, 0.05, riders(3, 7, 3.8))
        self.assertEqual(lv, 0.0)  # not yet 1.5 s at rest / 2 s clear
        lv, _ = run(5.0, 0.1, 0.05, riders(3, 7, 3.8), contact={"r10": True})
        self.assertEqual(lv, 0.0)
        lv, _ = run(5.0, 0.1, 1.5, riders(3, 7, 3.8))  # still swinging
        self.assertEqual(lv, 0.0)

    def test_off_zero(self):
        lv, _ = run(5.0, 7.0, 0.0, riders(3, 7, 0.0))  # on the stop
        self.assertEqual(lv, 0.0)
        lv, _ = run(5.0, 1.2, 0.0, riders(3, 7, 3.6))
        self.assertEqual(lv, 0.0)

    def test_unseated_is_half(self):
        lv, _ = run(5.0, 0.0, 0.0, riders(3, 7, 7.3, off100=-0.007))
        self.assertEqual(lv, 0.5)

    def test_reading_off_is_half(self):
        lv, _ = run(5.0, 0.0, 0.0, riders(3, 7, 3.5))
        self.assertEqual(lv, 0.5)

    def test_jar_off_pan(self):
        lv, _ = run(5.0, 0.0, 0.0, riders(0, 0, 0.0), on_pan=False)
        self.assertEqual(lv, 0.0)

    def test_noop_is_zero(self):
        # riders at zero with the jar on the pan: the beam is on its upper stop
        lv, _ = run(10.0, 7.0, 0.0, riders(0, 0, 0.0))
        self.assertEqual(lv, 0.0)


if __name__ == "__main__":
    unittest.main()
