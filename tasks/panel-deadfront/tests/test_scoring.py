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
    p, r = L.guide_pose(lean_deg, L.panel_origin(setup))
    return p + [dx, dy, dz], r


def approach():
    mon = S.Monitor()
    for angle in range(45, -1, -5):
        mon.observe_guides(*installed(lean_deg=angle), L.panel_origin(0))
    return mon


def m_of(pose, seated=(False,), tips=FAR_TIPS, driver=FAR_DRIVER, setup=0):
    return S.measure(pose[0], pose[1], L.panel_origin(setup), list(seated), tips, driver, screws(setup))


class TestLevels(unittest.TestCase):
    def test_full(self):
        m = m_of(installed(), (True,))
        self.assertTrue(m["flat"] and m["in_guides"] and m["clear"])
        self.assertEqual(S.level_of(m), 1.0)

    def test_one_shroud_catches_even_when_cover_is_flat_and_hung(self):
        # Roll about the panel normal and translate vertically. Only the lower
        # shroud in the left column overlaps a window edge (by 0.158 mm).
        pose = L.panel_origin(0) + [0, -0.0003, 0.0013], L.rx(0.3)
        m = m_of(pose, (True,))
        self.assertTrue(m["in_guides"] and m["flat"] and m["clear"])
        self.assertEqual(m["shrouds_through"], [False, True, True, True, True, True])
        for seated in ([False], [True]):
            sc = S.Scorer()
            for _ in range(S.HOLD_STEPS + 1):
                self.assertEqual(sc.step(dict(m, seated=seated), 0.0, 0.0, approach()), 0.0)
            self.assertFalse(sc.done)

    def test_shroud_must_reach_front_face(self):
        pose = installed(dx=L.FACE_FRONT + L.COVER_T / 2)
        self.assertFalse(any(m_of(pose)["shrouds_through"]))

    def test_plate_penetrating_tub_fails_flatness(self):
        self.assertFalse(m_of(installed(dx=0.002))["flat"])

    def test_partial(self):
        self.assertEqual(S.level_of(m_of(installed(), (False,))), 0.25)

    def test_hands_on_cover_caps_half(self):
        p, r = installed()
        tip = p + r @ np.array([-0.01, 0.10, 0.0])
        m = m_of((p, r), (True,), tips=[tip] * 4)
        self.assertFalse(m["clear"])
        self.assertEqual(S.level_of(m), 0.5)

    def test_finger_on_training_knob_caps_half(self):
        p, r = installed()
        tip = p + L.grip_center(1)
        self.assertEqual(S.level_of(m_of((p,r), (True,), tips=[tip])), .5)

    def test_driver_on_screw_caps_half(self):
        m = m_of(installed(), (True,), driver=screws(0)[0] + np.array([-0.004, 0, 0]))
        self.assertEqual(S.level_of(m), 0.5)

    def test_leaning_cover_is_zero(self):
        # top standing off 3 mm (resting on a shroud / tipped open): not flat
        lean = math.degrees(math.atan2(0.003, L.COVER_H / 2 - L.SLOT_TOP))
        m = m_of(installed(lean_deg=lean), (True,))
        self.assertFalse(m["flat"])
        self.assertEqual(S.level_of(m), 0.0)

    def test_small_gap_ok(self):
        lean = math.degrees(math.atan2(0.0008, L.COVER_H / 2 - L.SLOT_TOP))
        self.assertTrue(m_of(installed(lean_deg=lean))["flat"])

    def test_one_guide_missed(self):
        pose = installed(dy=.005)
        self.assertFalse(m_of(pose)["in_guides"])

    def test_bypassing_guide_entry_is_zero(self):
        m = m_of(installed(), (True,))
        mon, sc = S.Monitor(), S.Scorer()
        mon.observe_guides(*installed(), L.panel_origin(0))
        self.assertEqual(max(sc.step(m, 0, 0, mon) for _ in range(25)), 0)

    def test_guides_must_stay_engaged(self):
        mon = approach()
        self.assertTrue(mon.guided)
        mon.observe_guides(*installed(dy=.009), L.panel_origin(0))
        mon.observe_guides(*installed(), L.panel_origin(0))
        self.assertFalse(mon.guided)

    def test_reversed_insertion_is_zero(self):
        mon = S.Monitor()
        mon.observe_guides(*installed(lean_deg=-45), L.panel_origin(0))
        self.assertFalse(mon.guided)

    def test_stand_pose_is_zero(self):
        m = m_of(L.cover_stand_pose(0))
        self.assertEqual(S.level_of(m), 0.0)


class TestScorer(unittest.TestCase):
    def test_hold_then_full(self):
        sc, mon = S.Scorer(), approach()
        m = m_of(installed(), (True,))
        lv = [sc.step(m, 0.0, 0.0, mon) for _ in range(S.HOLD_STEPS)]
        self.assertTrue(all(v == 0.5 for v in lv[:-1]))
        self.assertEqual(lv[-1], 1.0)

    def test_moving_resets_hold(self):
        sc, mon = S.Scorer(), approach()
        m = m_of(installed(), (True,))
        for _ in range(S.HOLD_STEPS - 1):
            sc.step(m, 0.0, 0.0, mon)
        self.assertEqual(sc.step(m, 0.01, 0.0, mon), 0.5)

    def test_drop_veto(self):
        sc, mon = S.Scorer(), approach()
        mon.step(1 / 240, -0.4, 0.0, False)
        self.assertTrue(mon.dropped)
        m = m_of(installed(), (True,))
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

    def test_guide_path_clears_return_and_side_tabs(self):
        # Separating-axis test of the sheet including the folded rear edges. Both side protrusions
        # are the same section in XZ. The top return spans the full plate width.
        def hits_hardware(angle, dx, dz):
            p, r = L.guide_pose(angle, np.zeros(3), dx=dx, dz=dz)
            side = np.array([[x, 0., z] for x in (-L.COVER_T, L.COVER_T)
                             for z in (-L.COVER_H/2, L.COVER_H/2)])
            plate = (p+side@r.T)[:, [0,2]]
            axes = np.vstack((np.eye(2), r[np.ix_([0,2],[0,2])].T))
            for _, lo, hi in L.upper_fit_boxes():
                box = np.array([[x,z] for x in (lo[0],hi[0]) for z in (lo[2],hi[2])])
                a, b = plate@axes.T, box@axes.T
                if np.all(a.max(axis=0)>b.min(axis=0)) and np.all(b.max(axis=0)>a.min(axis=0)):
                    return True
            return False
        # Actual stalled pose from Brev R1 (guide midpoint 14 mm forward, 15 mm high).
        self.assertTrue(hits_hardware(41.48, -.0143, .0147))
        for a, b in zip(L.ENTRY_WAYPOINTS, L.ENTRY_WAYPOINTS[1:]):
            for x, z in np.linspace(a, b, 31):
                for angle in (41.48, 45., 46.):
                    self.assertFalse(hits_hardware(angle, x, z))

    def test_mount_land_leaves_the_folded_edges_clear(self):
        # With the old full-width land, the r3 plate is stopped almost exactly where the
        # rear of each doubled side edge contacts its top edge (20 mm above the panel bottom).
        top_gap, bottom_gap = .000863, -.001265
        hem_back_at_land = bottom_gap + .020*(top_gap-bottom_gap)/L.COVER_H + L.COVER_T
        self.assertLess(abs(hem_back_at_land), .00001)
        for y0,y1,z0,z1 in L.hem_boxes():
            self.assertLess(z0, -L.COVER_H/2+.020)
            for lo, hi in L.lower_mount_boxes():
                self.assertTrue(y1 <= lo[1] or hi[1] <= y0)
        # The measured screw position stays inside the land and the pilot itself stays open.
        y,z = L.SCREW_Y[0], L.SCREW_Z
        def on_land(yy, zz):
            return any(lo[1] < yy < hi[1] and lo[2] < zz < hi[2] for lo,hi in L.lower_mount_boxes())
        self.assertFalse(on_land(y,z))
        self.assertTrue(on_land(y-.005,z) and on_land(y+.005,z))

    def test_magnetic_screw_reaches_the_pilot_only_when_aligned(self):
        axis = L.screw_axis_point(0, 0)
        rot = L.driver_rot_x(0)
        mouth = axis - [L.SHANK_L+L.HEAD_T, 0, 0]
        tip, sr = L.screw_in_driver(mouth, rot)
        np.testing.assert_allclose(tip, axis)
        self.assertAlmostEqual(np.linalg.det(sr), 1.)
        self.assertTrue(L.drive_alignment(mouth, rot, axis, 0.)[0])
        for error in ([-.003, 0, 0], [.003, 0, 0], [0, .002, 0]):
            self.assertFalse(L.drive_alignment(mouth+error, rot, axis, 0.)[0])
        self.assertFalse(L.drive_alignment(mouth, L.rz(10)@rot, axis, 0.)[0])
        self.assertGreater(L.screw_in_driver(*L.driver_holster_pose())[0][2], L.TOP)

    def test_cover_boxes_area(self):
        area = sum((b[1] - b[0]) * (b[3] - b[2]) for b in L.cover_boxes())
        holes = sum((w[1] - w[0]) * (w[3] - w[2]) for w in L.window_rects()) + sum(
            (s[1] - s[0]) * (min(s[3], L.COVER_H/2)-max(s[2], -L.COVER_H/2)) for s in L.slot_rects())
        self.assertAlmostEqual(area, L.COVER_W * L.COVER_H - holes, places=6)

    def test_seeded_specs_in_range(self):
        for n in range(50):
            sp = L.setup_spec(("seed", n))
            self.assertLessEqual(abs(sp["stand_dxy"][0]), L.SEED_STAND_DX)
            self.assertLessEqual(abs(sp["panel_dyz"][1]), L.SEED_PANEL_DZ)
            self.assertEqual(sp["on"], [False]*6)


if __name__ == "__main__":
    unittest.main()
