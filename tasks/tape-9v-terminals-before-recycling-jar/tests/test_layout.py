"""Layout sanity checks (no simulator): python -m unittest discover -s tests (from the task folder)."""
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'isaac'))
import layout as L  # noqa: E402


class TestLayout(unittest.TestCase):
    def test_real_sizes(self):
        self.assertAlmostEqual(L.NINE_V_SIZE[2] + L.SNAP_HEIGHT, 0.0485, places=4)   # IEC 6LR61 overall height
        self.assertAlmostEqual(L.SNAP_SPACING, 0.0127, places=4)                      # 0.5 in
        self.assertAlmostEqual(L.TAPE_SEG_LEN * L.TAPE_SEGMENTS, 0.060, places=6)     # visible strip
        self.assertAlmostEqual(L.TAPE_WIDTH, 0.019, places=6)

    def test_jar_cells_inside_the_jar(self):
        for name, (kind, p, r) in L.jar_contents().items():
            ln, rad = (L.AA_LENGTH, L.AA_RADIUS) if kind == 'AA' else (L.AAA_LENGTH, L.AAA_RADIUS)
            axis = r[:, 2]
            for end in (p + axis * ln / 2, p - axis * ln / 2):
                side = np.cross(axis, [0, 0, 1.])
                side /= np.linalg.norm(side)
                for q in (end + side * rad, end - side * rad):
                    self.assertLess(np.hypot(*(q[:2] - L.JAR_CENTER)), L.JAR_INNER_R, name)
            self.assertLess(p[2] + rad, L.JAR_RIM_Z)

    def test_9v_fits_the_mouth(self):
        half_diag = np.hypot(L.NINE_V_SIZE[2] / 2 + L.SNAP_HEIGHT, L.NINE_V_SIZE[0] / 2)
        self.assertLess(half_diag, L.JAR_MOUTH_R)

    def test_mat_and_9v_do_not_overlap(self):
        half = np.array(L.MAT_SIZE[:2]) / 2
        corners = [L.NINE_V_POS + L.NINE_V_ROT @ np.array([sx * L.NINE_V_SIZE[0] / 2, 0, sz * L.NINE_V_SIZE[2] / 2])
                   for sx in (-1, 1) for sz in (-1, 1)]
        for c in corners:
            inside = np.all(np.abs(c[:2] - L.MAT_CENTER[:2]) <= half)
            self.assertFalse(inside)

    def test_strip_on_mat(self):
        for p, r in L.tape_segment_poses():
            self.assertTrue(np.all(np.abs(p[:2] - L.MAT_CENTER[:2]) < np.array(L.MAT_SIZE[:2]) / 2))
            self.assertGreater(p[2], L.MAT_TOP)
