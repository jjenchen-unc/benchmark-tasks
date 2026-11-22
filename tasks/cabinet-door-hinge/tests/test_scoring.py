"""Unit tests for the cabinet-door-hinge scorer and layout (numpy only; no simulator).

    python -m pytest tests/test_scoring.py      (from the task folder)
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
import layout as L  # noqa: E402
import scoring as S  # noqa: E402

SPEC = L.setup_spec(("seed", 1415158438))


def poses_at(alpha, spec=SPEC, lift=0.0):
    dp, dr = L.door_pose_hung(spec, alpha)
    dp = dp + np.array([0.0, 0.0, lift])
    out = {"door": (dp, dr)}
    for i in range(2):
        out[f"arm{i}"] = L.arm_pose_from_door(dp, dr, spec, i, alpha)
    return out


FAR = [np.array([0.0, 0.3, 1.0])] * 4


def run(states, alpha, tips=FAR, steps=25, speeds=(0.0, 0.0), lift=0.0):
    sc = S.Scorer()
    for _ in range(steps):
        m = S.measure(poses_at(alpha, lift=lift), speeds, [alpha, alpha], states, tips, SPEC)
        sc.step(m)
    return sc


def test_full_score_needs_both_clipped_and_shut():
    assert run(["clipped", "clipped"], 0.0).level == 1.0
    assert run(["clipped", "clipped"], 2.5).level == 1.0


def test_both_clipped_open_is_half():
    assert run(["clipped", "clipped"], 90.0).level == 0.5
    assert run(["clipped", "clipped"], 10.0).level == 0.5


def test_one_clipped_is_quarter():
    assert run(["clipped", "hooked"], 0.0).level == 0.25
    assert run(["free", "clipped"], 90.0).level == 0.25


def test_hooked_only_or_free_is_zero():
    assert run(["hooked", "hooked"], 90.0).level == 0.0
    assert run(["free", "free"], 0.0).level == 0.0


def test_hands_on_door_blocks_credit():
    dp, dr = L.door_pose_hung(SPEC, 0.0)
    tip = dp + dr @ np.array([0.15, -0.005, 0.2])  # 5 mm off the outer face
    assert run(["clipped", "clipped"], 0.0, tips=[tip] * 4).level == 0.0


def test_hold_and_still_needed():
    assert run(["clipped", "clipped"], 0.0, steps=S.HOLD_STEPS - 1).level == 0.0
    assert run(["clipped", "clipped"], 0.0, speeds=(0.05, 0.0)).level == 0.0


def test_reveal_out_of_tolerance_is_not_shut():
    assert run(["clipped", "clipped"], 0.0, lift=0.003).level == 0.5


def test_seated_arm_geometry_matches_plates():
    for alpha in (90.0, 45.0, 0.0):
        p = poses_at(alpha)
        for i in range(2):
            ap, ar = p[f"arm{i}"]
            assert np.allclose(ap, L.plate_origin(SPEC, i), atol=1e-9)
            assert np.allclose(ar, np.eye(3), atol=1e-9)


def test_start_states_are_never_near_hooked():
    for seed in range(200):
        spec = L.setup_spec(("seed", seed))
        dp, dr = L.door_pose_start(spec)
        for i in range(2):
            ap, ar = L.arm_pose_from_door(dp, dr, spec, i)
            assert np.linalg.norm(ap - L.plate_origin(spec, i)) > 0.06
            # the arm channel stands outboard of the carcass's left side panel (beside it, never inside)
            corners = L.box_pts(*L.arm_pieces_a()[0][1:], rot=None) @ ar.T + ap
            assert corners[:, 1].min() > L.cabinet_origin(spec)[1] + 0.02
        assert abs(spec["yaw"]) <= L.START_YAW_DEG


def test_closed_door_covers_front():
    dp, dr = L.door_pose_hung(SPEC, 0.0)
    xf, yl = L.cabinet_origin(SPEC)
    inner = dp + dr @ np.array([0.0, L.DOOR_T, 0.0])
    assert math.isclose(inner[0], xf - 0.002, abs_tol=1e-9)
    free = dp + dr @ np.array([L.DOOR_W, 0.0, 0.0])
    assert math.isclose(free[1], yl - L.DOOR_W, abs_tol=1e-9)
