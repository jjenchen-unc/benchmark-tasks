"""Maximum world-space center-of-mass and rotation speed over adjacent physics poses."""
import math

import numpy as np


class SubstepMotion:
    def __init__(self, local_com, position, rotation):
        self.local_com = np.asarray(local_com, dtype=float)
        self.com = position + rotation @ self.local_com
        self.rotation = rotation
        self.linear = self.angular = 0.
        self.samples = 0

    def observe(self, position, rotation, dt):
        com = position + rotation @ self.local_com
        relative = rotation @ self.rotation.T
        skew = np.array([relative[2, 1] - relative[1, 2],
                         relative[0, 2] - relative[2, 0],
                         relative[1, 0] - relative[0, 1]])
        # atan2 remains accurate for tiny rotations and unambiguous up to 180 degrees.
        angle = math.atan2(float(np.linalg.norm(skew)), float(np.trace(relative) - 1.))
        linear = float(np.linalg.norm(com - self.com)) / dt
        angular = math.degrees(angle) / dt
        if not (math.isfinite(linear) and math.isfinite(angular)):
            linear = angular = math.inf
        self.linear = max(self.linear, linear)
        self.angular = max(self.angular, angular)
        self.com, self.rotation = com, rotation
        self.samples += 1

    def take(self, expected_samples):
        valid = self.samples == expected_samples
        maxima = (self.linear, self.angular) if valid else (math.inf, math.inf)
        self.linear = self.angular = 0.
        self.samples = 0
        return maxima
