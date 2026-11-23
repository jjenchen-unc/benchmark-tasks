"""Unit tests for the regulator-set scorer, pressure plant and start states (pure Python + NumPy).

    python -m pytest tests/test_scoring.py   (or: python tests/test_scoring.py)
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
import layout as L  # noqa: E402
from scoring import Scorer  # noqa: E402

DT = 0.05
NO = dict(base=False, knob=False, lever=False)


def run(scorer, seconds, **kw):
    a = dict(gauge_psi=90.0, gauge_rate=0.0, line_psi=90.0, set_psi=90.0, knob_out_mm=0.0, valve_deg=0.0,
             contact=NO)
    a.update(kw)
    lv = 0.0
    for _ in range(int(round(seconds / DT))):
        lv = scorer.step(DT, **a)
    return lv


class TestScorer(unittest.TestCase):
    def test_full_score_needs_hold(self):
        s = Scorer()
        self.assertEqual(run(s, 3.0), 0.0)  # clear 1.5 s + hold 2 s = 3.5 s
        self.assertEqual(run(s, 0.6), 1.0)

    def test_each_condition_vetoes(self):
        for kw in (dict(gauge_psi=92.5), dict(gauge_psi=87.4, line_psi=87.4, set_psi=87.4), dict(set_psi=80.0),
                   dict(knob_out_mm=3.9), dict(valve_deg=10.0), dict(gauge_rate=3.0),
                   dict(contact=dict(base=False, knob=True, lever=False)), dict(line_psi=95.0)):
            self.assertEqual(run(Scorer(), 6.0, **kw), 0.0, kw)

    def test_band_edges(self):
        self.assertEqual(run(Scorer(), 4.0, gauge_psi=91.9, line_psi=91.9, set_psi=91.9), 1.0)
        self.assertEqual(run(Scorer(), 4.0, gauge_psi=88.1, line_psi=88.1, set_psi=88.1), 1.0)
        self.assertEqual(run(Scorer(), 4.0, gauge_psi=92.1, line_psi=92.1, set_psi=92.1), 0.0)

    def test_noop_start_scores_zero(self):
        for n in range(200):
            sp = L.setup_spec(("seed", n))
            self.assertEqual(run(Scorer(), 6.0, gauge_psi=sp["p0"], line_psi=sp["p0"], set_psi=sp["p0"]), 0.0)


class TestPlant(unittest.TestCase):
    def test_non_relieving(self):
        p = 115.0
        for _ in range(200):  # knob turned down to 90, bleed closed: the line stays
            p = L.step_pressure(p, 90.0, 0.0, DT)
        self.assertAlmostEqual(p, 115.0, places=6)

    def test_bleed_brings_down_to_setting(self):
        p = 115.0
        for _ in range(200):
            p = L.step_pressure(p, 90.0, 1.0, DT)
        self.assertTrue(87.5 < p < 90.0, p)  # droop under the bleed's flow
        for _ in range(100):
            p = L.step_pressure(p, 90.0, 0.0, DT)
        self.assertAlmostEqual(p, 90.0, delta=0.05)  # lock-up at the setting once closed

    def test_rises_to_setting(self):
        p = 80.0
        for _ in range(60):
            p = L.step_pressure(p, 90.0, 0.0, DT)
        self.assertAlmostEqual(p, 90.0, delta=0.05)

    def test_valve_seal_band(self):
        self.assertEqual(L.bleed_frac(0.0), 0.0)
        self.assertEqual(L.bleed_frac(L.VALVE_CLOSED_DEG), 0.0)
        self.assertEqual(L.bleed_frac(90.0), 1.0)

    def test_gauge_angle_roundtrip(self):
        for p in (0.0, 45.0, 90.0, 160.0):
            self.assertAlmostEqual(L.psi_of_angle(L.gauge_angle(p)), p, places=9)


class TestStarts(unittest.TestCase):
    def test_ranges_and_guard(self):
        ref = L.need_turns(L.setup_spec(("seed", L.OFFICIAL_SEED)))
        for n in range(3000):
            sp = L.setup_spec(("seed", n))
            self.assertTrue(L.P0_RANGE[0] <= sp["p0"] <= L.P0_RANGE[1])
            self.assertTrue(L.K_RANGE[0] <= sp["k"] <= L.K_RANGE[1])
            self.assertGreater(sp["p0"], L.SETPOINT_PSI + L.TOL_PSI)  # every start needs the bleed
            self.assertGreaterEqual(abs(L.need_turns(sp) - ref), L.GUARD_TURNS - 1e-9)

    def test_default_trace_turn_misses_everywhere(self):
        ref = L.need_turns(L.setup_spec(("seed", L.OFFICIAL_SEED)))
        for n in range(3000):
            sp = L.setup_spec(("seed", n))
            self.assertGreater(abs(sp["p0"] + sp["k"] * ref - L.SETPOINT_PSI), L.TOL_PSI + 1.0)


if __name__ == "__main__":
    unittest.main()
