"""Unit tests of the pure scoring functions (numpy only):  python -m pytest tests/  or  python tests/test_scoring.py"""

import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
import layout as L  # noqa: E402
import scoring as S  # noqa: E402

FAR_TIPS = [np.array([0.0, 0.5, 1.2])] * 4
FAR_DRIVER = np.array([0.1, -0.3, 0.8])


def screws(setup):
    po = L.panel_origin(setup)
    return [po + np.array([L.head_under_x(0.0), y, L.SCREW_Z]) for y in L.SCREW_Y]


def installed(setup=0, dx=0.0, dy=0.0, dz=0.0, lean_deg=0.0):
    p, r = L.cover_installed_pose(setup)
    r = L.ry(-lean_deg) @ r
    # pivot about the slot tops (as when hung)
    v = np.array([-L.COVER_T / 2, 0.0, L.SLOT_TOP])
    pivot = p + v
    p = pivot - r @ v + np.array([dx, dy, dz])
    return p, r


def m_of(pose, seated=(False, False), tips=FAR_TIPS, driver=FAR_DRIVER, setup=0):
    return S.measure(pose[0], pose[1], L.panel_origin(setup), list(seated), tips, driver, screws(setup))


class TestLevels(unittest.TestCase):
    def test_full(self):
        m = m_of(installed(), (True, True))
        self.assertTrue(m["flat"] and m["hung"] and m["clear"])
        self.assertEqual(S.level_of(m), 1.0)

    def test_one_shroud_catches_even_when_cover_is_flat_and_hung(self):
        # Roll about the panel normal and translate vertically. Only the lower
        # shroud in the left column overlaps a window edge (by 0.158 mm).
        pose = L.panel_origin(0) + [0, -0.0003, 0.0013], L.rx(0.3)
        m = m_of(pose, (True, True))
        self.assertTrue(m["hung"] and m["flat"] and m["clear"])
        self.assertEqual(m["shrouds_through"], [False, True, True, True, True, True])
        for seated in ([False, False], [False, True], [True, True]):
            sc = S.Scorer()
            for _ in range(S.HOLD_STEPS + 1):
                self.assertEqual(sc.step(dict(m, seated=seated), 0.0, 0.0, S.Monitor()), 0.0)
            self.assertFalse(sc.done)

    def test_shroud_must_reach_front_face(self):
        pose = installed(dx=L.FACE_FRONT + L.COVER_T / 2)
        self.assertFalse(any(m_of(pose)["shrouds_through"]))

    def test_plate_penetrating_tub_fails_flatness(self):
        self.assertFalse(m_of(installed(dx=0.002))["flat"])

    def test_partial(self):
        self.assertEqual(S.level_of(m_of(installed(), (True, False))), 0.5)
        self.assertEqual(S.level_of(m_of(installed(), (False, False))), 0.25)

    def test_hands_on_cover_caps_half(self):
        p, r = installed()
        tip = p + r @ np.array([-0.01, 0.10, 0.0])
        m = m_of((p, r), (True, True), tips=[tip] * 4)
        self.assertFalse(m["clear"])
        self.assertEqual(S.level_of(m), 0.5)

    def test_driver_on_screw_caps_half(self):
        m = m_of(installed(), (True, True), driver=screws(0)[1] + np.array([-0.004, 0, 0]))
        self.assertEqual(S.level_of(m), 0.5)

    def test_leaning_cover_is_zero(self):
        # top standing off 3 mm (resting on a shroud / tipped open): not flat
        lean = math.degrees(math.atan2(0.003, L.COVER_H / 2 - L.SLOT_TOP))
        m = m_of(installed(lean_deg=lean), (True, True))
        self.assertFalse(m["flat"])
        self.assertEqual(S.level_of(m), 0.0)

    def test_small_gap_ok(self):
        lean = math.degrees(math.atan2(0.0008, L.COVER_H / 2 - L.SLOT_TOP))
        self.assertTrue(m_of(installed(lean_deg=lean))["flat"])

    def test_off_screws_is_zero(self):
        m = m_of(installed(dz=0.012))  # lifted off the shanks
        self.assertFalse(m["hung"])
        self.assertEqual(S.level_of(m), 0.0)
        m = m_of(installed(dy=0.003))  # beyond the slot play: shank outside the slot
        self.assertFalse(m["hung"])

    def test_slot_play_still_hung(self):
        self.assertTrue(m_of(installed(dy=0.0015))["hung"])

    def test_stand_pose_is_zero(self):
        m = m_of(L.cover_stand_pose(0))
        self.assertEqual(S.level_of(m), 0.0)


class TestScorer(unittest.TestCase):
    def test_hold_then_full(self):
        sc, mon = S.Scorer(), S.Monitor()
        m = m_of(installed(), (True, True))
        lv = [sc.step(m, 0.0, 0.0, mon) for _ in range(S.HOLD_STEPS)]
        self.assertTrue(all(v == 0.5 for v in lv[:-1]))
        self.assertEqual(lv[-1], 1.0)

    def test_moving_resets_hold(self):
        sc, mon = S.Scorer(), S.Monitor()
        m = m_of(installed(), (True, True))
        for _ in range(S.HOLD_STEPS - 1):
            sc.step(m, 0.0, 0.0, mon)
        self.assertEqual(sc.step(m, 0.01, 0.0, mon), 0.5)

    def test_drop_veto(self):
        sc, mon = S.Scorer(), S.Monitor()
        mon.step(1 / 240, -0.4, 0.0, False)
        self.assertTrue(mon.dropped)
        m = m_of(installed(), (True, True))
        self.assertEqual(max(sc.step(m, 0.0, 0.0, mon) for _ in range(30)), 0.0)

    def test_no_drop_when_held_or_supported(self):
        mon = S.Monitor()
        mon.step(1 / 240, -0.4, 1.0, False)
        mon.step(1 / 240, -0.4, 0.0, True)
        self.assertFalse(mon.dropped)


class TestGeometry(unittest.TestCase):
    def test_windows_clear_shrouds(self):
        for w in L.window_rects():
            for s in range(6):
                c = L.space_center(s)
                if w[0] < c[1] < w[1]:
                    self.assertAlmostEqual(c[1] - L.FACE_W / 2 - w[0], L.WIN_CLEAR_Y)
                    self.assertGreaterEqual(c[2] - L.FACE_H / 2 - w[2], L.WIN_CLEAR_Z - 1e-9)

    def test_slot_play_exceeds_window_clearance(self):
        self.assertGreater(L.SLOT_W / 2 - L.SHANK_R, L.WIN_CLEAR_Y)

    def test_cover_boxes_area(self):
        area = sum((b[1] - b[0]) * (b[3] - b[2]) for b in L.cover_boxes())
        holes = sum((w[1] - w[0]) * (w[3] - w[2]) for w in L.window_rects()) + sum(
            (s[1] - s[0]) * (s[3] - (-L.COVER_H / 2)) for s in L.slot_rects())
        self.assertAlmostEqual(area, L.COVER_W * L.COVER_H - holes, places=6)

    def test_seeded_specs_in_range(self):
        for n in range(50):
            sp = L.setup_spec(("seed", n))
            self.assertLessEqual(abs(sp["stand_dxy"][0]), L.SEED_STAND_DX)
            self.assertLessEqual(abs(sp["panel_dyz"][1]), L.SEED_PANEL_DZ)
            self.assertEqual(len(sp["on"]), 6)


if __name__ == "__main__":
    unittest.main()
