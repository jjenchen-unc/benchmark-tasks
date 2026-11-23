"""A one-substep excursion must reset the whole second, even with equal frame endpoints."""
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
import layout
from motion import SubstepMotion
from scoring import HOLD_STEPS, Monitor, Scorer, measure

def run():
    for displacement, rotation in ((0.0001, 0.), (0., 0.1)):
        identity = np.eye(3)
        monitor = Monitor()
        scorer = Scorer()
        motion = SubstepMotion(np.array([-.004, 0., 0.]), layout.H, identity)
        seated = measure(layout.H, identity, 0., 0., [])
        def frame(with_pulse=False):
            for step in range(24):
                pulse = with_pulse and step == 8
                p = layout.H + [0, displacement if pulse else 0., 0.]
                r = layout.rx(rotation) if pulse else identity
                motion.observe(p, r, 1 / 480)
            linear, angular = motion.take(24)
            return scorer.step(seated, linear, angular, monitor)
        for _ in range(HOLD_STEPS-1):
            assert frame() == .5
        assert frame(True) == .5 and scorer.still == 0
        for _ in range(HOLD_STEPS-1):
            assert frame() == .5
        assert frame() == 1.
        assert all(np.isinf(motion.take(24)))
    print('480 Hz transient translation and rotation each reset the full hold; missing substeps fail closed.')

if __name__=='__main__':
    run()
