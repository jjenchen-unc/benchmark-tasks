"""Unit checks of the pool-8ball scorer and layout (numpy only):  python -m pytest tests/  (or python tests/test_scoring.py)"""

import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
import layout as L  # noqa: E402
import scoring as S  # noqa: E402

DT = 1 / 480


def on_bed(xy):
    return np.array([xy[0], xy[1], L.BALL_Z])


def dropped(pocket):
    c, _ = L.pocket_hole(pocket)
    return np.array([c[0], c[1], L.BED_Z - L.POCKET_DEPTH + L.BALL_R])


def hit_monitor():
    m = S.Monitor()
    for _ in range(2):
        m.step(DT, 50.0, 0, 0, 0, 0)
    for _ in range(100):
        m.step(DT, 0, 0, 0, 0, 0)
    m.step(DT, 0, 30.0, 0, 0, 0)
    return m


class TestClassify(unittest.TestCase):
    def test_made_called_pocket(self):
        lvl, f = S.classify(on_bed((0.6, 0.0)), dropped("RS"), "RS", hit_monitor(), 0.0)
        self.assertEqual(lvl, 1.0)
        self.assertEqual(f["eight_pocket"], "RS")

    def test_wrong_pocket(self):
        self.assertEqual(S.classify(on_bed((0.6, 0.0)), dropped("LS"), "RS", hit_monitor(), 0.0)[0], 0.0)

    def test_scratch(self):
        self.assertEqual(S.classify(dropped("NR"), dropped("RS"), "RS", hit_monitor(), 0.0)[0], 0.0)
        self.assertEqual(S.classify(dropped("NR"), on_bed((0.9, 0.1)), "RS", hit_monitor(), 0.0)[0], 0.0)

    def test_near_mouth_half(self):
        a, b = L.pocket_jaws("RS")
        mid = (a + b) / 2 + np.array([0.0, 2.5 * L.BALL_R])  # 2.5 R inside the mouth line
        self.assertEqual(S.classify(on_bed((0.6, 0.0)), on_bed(mid), "RS", hit_monitor(), 0.0)[0], 0.5)
        far = (a + b) / 2 + np.array([0.0, 4 * L.BALL_R])
        self.assertEqual(S.classify(on_bed((0.6, 0.0)), on_bed(far), "RS", hit_monitor(), 0.0)[0], 0.25)

    def test_legal_stroke_and_rest_required(self):
        pc = on_bed((0.6, 0.0))
        a, b = L.pocket_jaws("RS")
        near = on_bed((a + b) / 2 + [0, 2.5 * L.BALL_R])
        m = S.Monitor()
        m.step(DT, 0, 20, 0, 0, 0)
        self.assertEqual(S.classify(pc, dropped("RS"), "RS", m, 0)[0], 0)
        m = hit_monitor()
        self.assertEqual(S.classify(pc, near, "RS", m, 0.01)[0], 0.25)
        m.step(DT, 0, 0, 0, 0, 0, shaft_cb=4)
        self.assertEqual(S.classify(pc, dropped("RS"), "RS", m, 0)[0], 0)

    def test_contact_zones_on_rotated_cue(self):
        forces = np.array([3., 1., 4., 8.])
        for side in ("left", "right"):
            origin, rotation = L.cue_rest_pose(side)
            depth = np.array([0., L.TIP_LEN - 0.0001, L.TIP_LEN + 0.0001, 0.20])
            points = origin - depth[:, None] * rotation[:, 0] + 0.003 * rotation[:, 1]
            self.assertEqual(S.cue_contact_forces(points, forces, origin, rotation), (4., 12.))

    def test_no_hit(self):
        m = S.Monitor()
        m.step(DT, 50.0, 0, 0, 0, 0)
        self.assertEqual(S.classify(on_bed((0.6, 0.0)), on_bed((0.9, 0.0)), "RS", m, 0.0)[0], 0.0)

    def test_fouls(self):
        for args in ((0, 0, 1.0, 0, 0), (0, 0, 0, 1.0, 0)):
            m = hit_monitor()
            m.step(DT, *args)
            self.assertTrue(m.foul)
            self.assertEqual(S.classify(on_bed((0.6, 0.0)), dropped("RS"), "RS", m, 0.0)[0], 0.0)

    def test_resolved_gap_makes_a_second_stroke(self):
        for gap_steps in (1, 10, 19, 20):
            with self.subTest(gap_steps=gap_steps):
                mon = S.Monitor()
                for contact in [5.0] * 16 + [0.0] * gap_steps + [5.0] * 16:
                    mon.step(DT, contact, 0, 0, 0, 0)
                mon.step(DT, 0, 3, 0, 0, 0)
                self.assertEqual(mon.strokes, 2)
                self.assertEqual(mon.fouls, ["second stroke"])
                self.assertEqual(S.classify(on_bed((0.6, 0)), dropped("RS"), "RS", mon, 0)[0], 0)

    def test_second_stroke_and_push(self):
        m = hit_monitor()
        for _ in range(30):
            m.step(DT, 0, 0, 0, 0, 0)
        m.step(DT, 5.0, 0, 0, 0, 0)
        self.assertEqual(m.strokes, 2)
        self.assertIn("second stroke", m.fouls)
        m2 = S.Monitor()
        for _ in range(40):
            m2.step(DT, 5.0, 0, 0, 0, 0)
        self.assertIn("push", m2.fouls)
        self.assertEqual(m2.strokes, 1)


class TestScorer(unittest.TestCase):
    def test_hold_before_full(self):
        sc = S.Scorer()
        m = hit_monitor()
        levels = [sc.step(on_bed((0.6, 0.0)), dropped("RS"), 0.0, 0.0, "RS", m) for _ in range(S.HOLD_STEPS + 1)]
        self.assertEqual(levels[0], 0.25)
        self.assertEqual(levels[-1], 1.0)

    def test_moving_cue_ball_blocks_full(self):
        sc = S.Scorer()
        m = hit_monitor()
        for _ in range(40):
            lvl = sc.step(on_bed((0.6, 0.0)), dropped("RS"), 0.3, 0.0, "RS", m)
        self.assertEqual(lvl, 0.25)


class TestLayout(unittest.TestCase):
    def test_default_seed_nontrivial(self):
        sp = L.setup_spec(("seed", L.DEFAULT_SEED))
        self.assertGreaterEqual(abs(sp["shot"]["cut_deg"]), 6.0)
        self.assertIn(sp["pocket"], L.CALLABLE)

    def test_seeds_valid_and_reproducible(self):
        pockets = set()
        for s in range(25):
            a, b = L.setup_spec(("seed", s)), L.setup_spec(("seed", s))
            np.testing.assert_allclose(a["cue"], b["cue"])
            sh = a["shot"]
            lim = L.SEED_LIMITS
            self.assertTrue(lim["cut"][0] <= sh["cut_deg"] <= lim["cut"][1])
            self.assertTrue(lim["sep"][0] <= sh["cue_travel"] <= lim["sep"][1])
            self.assertTrue(L.cue_line_ok(a["cue"], sh["d"]))
            self.assertTrue(L.side_entry_ok(a["obj"], a["pocket"]))
            self.assertFalse(L.full_ball_enters(a["cue"], a["obj"], a["pocket"]))
            pockets.add(a["pocket"])
        self.assertEqual(pockets, set(L.CALLABLE))

    def test_canned_stroke_never_drives_into_eight(self):
        canned = L._default_canned()
        for s in range(25):
            sp = L.setup_spec(("seed", s))
            self.assertFalse(L._canned_hits(sp["cue"], sp["obj"], canned))

    def test_rests_mirror_and_clear_the_camera_post(self):
        for side in ("left", "right"):
            p, r = L.cue_rest_pose(side)
            butt = p[:2] - L.CUE_L * r[:2, 0]
            m = L.MAST_XY
            t = float(np.clip((m - p[:2]) @ (butt - p[:2]) / ((butt - p[:2]) @ (butt - p[:2])), 0, 1))
            # cue butt radius 14 mm + camera-post radius 12 mm, plus 9 mm air
            self.assertGreater(np.linalg.norm(m - (p[:2] + t * (butt - p[:2]))), 0.035)
        self.assertEqual(L.rest_side("RS"), "left")
        self.assertEqual(L.rest_side("LS"), "right")

    def test_pocket_geometry(self):
        for n in L.POCKETS:
            a, b = L.pocket_jaws(n)
            w = np.linalg.norm(a - b)
            want = L.CORNER_MOUTH if L.POCKETS[n]["kind"] == "corner" else L.SIDE_MOUTH
            self.assertAlmostEqual(w, want, places=6)
            c, r = L.pocket_hole(n)
            self.assertGreater(2 * r, 2 * L.BALL_R)

    def test_bed_cells_avoid_holes(self):
        holes = [L.pocket_hole(n) for n in L.POCKETS]
        for x0, y0, x1, y1 in L.bed_cells():
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            for c, r in holes:
                self.assertGreater(math.hypot(cx - c[0], cy - c[1]), r - 1e-9)


if __name__ == "__main__":
    unittest.main()
