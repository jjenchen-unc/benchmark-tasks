"""CPU regression for the captured insertion stall. Uses NumPy, SciPy and the task's YAM package."""
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
from scipy.optimize import linprog
from scipy.spatial import ConvexHull

task_dir = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(task_dir/'src'), str(task_dir/'isaac')]
import demo
import layout as L


def box(center, size):
    half = np.asarray(size)/2
    return L.box_pts(center-half, center+half)


def overlap_radius(a, b):
    """Largest common inscribed sphere in mm, ignoring disjoint bounding boxes."""
    a, b = a*1000, b*1000
    if np.any(np.minimum(a.max(0), b.max(0)) <= np.maximum(a.min(0), b.min(0)) + 1e-8):
        return 0.0
    planes = np.vstack([ConvexHull(p).equations for p in (a, b)])
    # Each convex-hull equation has a unit outward normal.
    normals = np.column_stack([planes[:, :3], np.ones(len(planes))])
    ans = linprog([0, 0, 0, -1], A_ub=normals, b_ub=-planes[:, 3],
                  bounds=[(None, None)]*3+[(0, None)], method="highs")
    if ans.status == 2:
        return 0.0
    assert ans.success, ans.message
    return ans.x[-1]


def check_corridor():
    housing = L.plate_pieces()+L.rim_pieces()+L.catch_pieces()
    housing += [box(np.array(L.STAND_CENTER), L.STAND_SIZE),
                box(np.array([L.HINGE_X+0.0025, L.HINGE_Y, 0]), (0.003, 0.006, 0.019))]
    housing += [L.annular_sector(L.BOOT_INNER_R, L.BOOT_OUTER_R, j*15, (j+1)*15, *L.BOOT_RIM_X, n=3)
                for j in range(24)]
    # The measured bail is open at ~92 degrees while the cap is inserted.
    housing += [v @ L.clip_rot(92).T+L.hinge_point() for v in L.clip_pieces()]
    moving = [p for group in L.bulb_pieces().values() for p in group]
    # Convex hull of endpoints covers every translation between x=-8 mm and the seat.
    for bulb in moving:
        swept = np.vstack([bulb- [0.008, 0, 0], bulb])
        assert all(overlap_radius(swept, fixed) < 1e-6 for fixed in housing)
    prior_stand = box(np.array([0.010, 0, -0.084]), (0.014, 0.035, 0.132))
    shoulder = L.bulb_pieces()["metal"][1]
    assert overlap_radius(shoulder-[0.00251, 0, 0], prior_stand) == 0
    assert overlap_radius(shoulder-[0.00249, 0, 0], prior_stand) > 0
    print("PASS original obstruction begins 2.5 mm before the seat; revised swept insertion corridor is clear")


def check_controller():
    class Probe:
        def __init__(self, biased=False):
            self.p = L.H + [demo.PRE_X, 0, 0]
            self.target_rot = np.eye(3)
            self.corr_p, self.corr_r = np.zeros(3), np.zeros(3)
            self.G = {"right": (np.zeros(3), np.eye(3))}
            self.right = SimpleNamespace(set_pose=self.issue, set_comp=lambda mode: None, label="")
            self.biased = biased
            self.labels = []

        def pose(self):
            return self.p, self.target_rot

        def detail(self):
            return {}

        def note(self, *args, **kwargs):
            pass

        def issue(self, p, r):
            self.labels.append(self.right.label)
            self.p += 0.2 * (p + [0.0003, 0.00045, -0.0002] - self.p)
            if self.biased:
                self.p[1] = L.H[1]+0.0006
            self.p[0] = min(self.p[0], L.H[0])

    c = Probe()
    steps = sum(1 for _ in demo.insert_bulb(c))
    assert abs(c.p[0]-L.H[0]) < 0.00035 and np.linalg.norm((c.p-L.H)[1:]) < 0.0002
    print(f"PASS feedback sequence seats in {steps} control steps in a biased first-order tracking fixture")
    class CatchOnEdge(Probe):
        def __init__(self, high_force=False):
            super().__init__()
            self.high_force = high_force
            self.seat_steps = 0
            self.progress = []

        def issue(self, p, r):
            super().issue(p, r)
            if self.right.label.startswith("seat "):
                self.seat_steps += 1
                if self.seat_steps <= 45:
                    self.p[1] = L.H[1]+0.00065
                    self.progress.append(self.p[0])

        def detail(self):
            return {"contacts": {"housing_on_bulb_n": 4.0 if self.high_force else 0.1}}

    c = CatchOnEdge()
    list(demo.insert_bulb(c, tries=1))
    assert max(c.progress) - c.progress[0] > 0.0005
    print("PASS a transient 0.65 mm lateral error with low load preserves axial feed")
    c = CatchOnEdge(high_force=True)
    try:
        list(demo.insert_bulb(c, tries=1))
    except demo.Stop:
        pass
    else:
        raise AssertionError("persistent 4 N contact did not stop insertion")
    assert max(c.progress) - c.progress[0] < 0.0005
    print("PASS sustained 4 N housing load unloads the entry face")
    c = Probe(biased=True)
    try:
        list(demo.insert_bulb(c))
    except demo.Stop:
        pass
    else:
        raise AssertionError("misaligned bulb was reported as seated")
    assert not any(label.startswith("seat ") for label in c.labels)
    print("PASS persistent 0.6 mm lateral error prevents the controller from pressing")


def check_axial_stimulus():
    import negcheck

    def sample(step, command, force, label):
        return dict(step=step, labels=[label], tcp_cmd_mm={"left": command},
                    detail={"measure": {"seated": True, "over": True, "latched": False, "catch_mm": 0.},
                            "contacts": {"robot_on_clip_force_n": force}})

    base = sample(0, [380., -24., 907.], [3., 15., 0.], "place axial pads")
    forward = sample(1, [383., -24., 907.], [4., 15., 0.], "axial press only")
    crossed = sample(1, [379.25, -27.78, 908.29], [4., 15., 0.], "axial press only")
    pinch = sample(1, [383., -24., 907.], [3., 34., 0.], "axial press only")
    for stroke, expected in ((forward, True), (crossed, False), (pinch, False)):
        valid, _ = negcheck.stimulus("neg-axial-press", [base, stroke], {})
        assert valid == expected
    print("PASS axial-control verification rejects the R5 cross-axis target and pure grip loads")


if __name__ == "__main__":
    check_corridor()
    check_controller()
    check_axial_stimulus()
