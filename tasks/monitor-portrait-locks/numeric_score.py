"""Pure numeric B5 rubric copied from frozen stand_position_score/3.

Only pure constants/helpers/score are copied; no MuJoCo observer or robot controls.
Runtime provenance and verified PhysX measurements are required by contract.py before
any value is exposed as an Arena result. Native records are not Arena evidence.
"""
from __future__ import annotations

import math

import numpy as np

SCORER_VERSION = "stand_position_score/3"

TOL_H_MM, TOL_DEG = 6.0, 3.0

HOLD_S = 20.0

STILL = 0.02                # m/s or rad/s on every stand axis (source stillness rule)

ABORT_N, ABORT_S = 80.0, 0.100

FURNITURE_N = 0.0           # v2: any screen-furniture contact vetoes (v1 used > 1 N); force is still recorded

LOCK_PEN_MM = 0.5           # pin/tooth/rack interpenetration beyond this = pin forced through a lock wall

PIN_IN_MM = 3.5             # pin retraction below this: tip still >= 0.5 mm inside the hole/tooth depth

OFF_INDEX_MM = 1.0          # arc/linear distance from the nearest index hole with the pin still in = forced axis

OVERTRAVEL_MM, OVERTRAVEL_DEG = 1.0, 1.0   # stand axes (2.9+ kg moving): forced past an end stop

BUTTON_PIN_OVERTRAVEL_MM = 1.0

BASE_MU = 0.5               # rubber foot on a desk laminate, conservative (lower = stricter)

BASE_SUSTAIN_S = 0.020      # violation must persist this long (a 2 ms contact spike cannot rotate a 7 kg stand)

GEOM_AGREE_MM, GEOM_AGREE_DEG = 0.5, 0.2

W_AXES, W_HOLD, PARTIAL_CAP = 0.60, 0.30, 0.90

TARGETS = {  # goal (height mm, pivot deg, tilt deg), axes the goal changes
    "A_portrait": ((130.0, 90.0, 0.0), "HP"),
    "landscape_shelf": ((40.0, 0.0, 0.0), "H"),
    "portrait_wall": ((130.0, 90.0, 0.0), "HP"),
    "tilt_up_shelf": ((85.0, 0.0, 15.0), "HT"),
    "portrait_side": ((130.0, 90.0, 0.0), "HP"),
}

START = (0.0, 0.0, 0.0)

AXIS_KEYS = (("H", "height_mm", TOL_H_MM), ("P", "pivot_deg", TOL_DEG), ("T", "tilt_deg", TOL_DEG))

CTX_BOX_BODIES = {"portrait_side": ("side_box",)}   # loose objects the robot must not touch

def _finite(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)

def _out(score, full, vetoed, reasons, **kw):
    return dict(score=score, full_credit=full, vetoed=vetoed, reasons=reasons, version=SCORER_VERSION, **kw)

def score(meas):
    """Pure function of the measurement record."""
    target = meas.get("target") if isinstance(meas, dict) else None
    if target not in TARGETS:
        return _out(0.0, False, True, ["unknown_or_missing_target"])
    needed = ("height_mm", "pivot_deg", "tilt_deg", "joint_height_mm", "joint_pivot_deg", "joint_tilt_deg",
              "hands_on_stand_at_end", "hold_s", "max_furniture_contact_n", "wrist_force_abort", "nonfinite",
              "samples", "disallowed_damage", "source_success_final", "furniture_contact_steps",
              "robot_box_contact_steps", "mechanism_damage", "base_tip_or_slide")
    missing = [k for k in needed if k not in meas or meas[k] is None]
    if missing:
        return _out(0.0, False, True, ["missing:" + ",".join(missing)])
    nums = [meas[k] for k in needed[:6]] + [meas["hold_s"], meas["max_furniture_contact_n"]]
    flags = [meas[k] for k in ("hands_on_stand_at_end", "wrist_force_abort", "nonfinite", "source_success_final",
                               "base_tip_or_slide")]
    counts = [meas[k] for k in ("samples", "furniture_contact_steps", "robot_box_contact_steps")]
    lists = [meas[k] for k in ("disallowed_damage", "mechanism_damage")]
    if not all(_finite(v) for v in nums) or not all(isinstance(f, bool) for f in flags) \
            or not all(isinstance(c, int) and not isinstance(c, bool) and c >= 0 for c in counts) \
            or not all(isinstance(x, (list, tuple)) for x in lists) \
            or meas["hold_s"] < 0 or meas["max_furniture_contact_n"] < 0 \
            or meas["samples"] < 1 or meas["nonfinite"]:
        return _out(0.0, False, True, ["nonfinite_or_malformed_or_unsampled_measurement"])

    reasons = []
    pose = dict(height_mm=meas["height_mm"], pivot_deg=meas["pivot_deg"], tilt_deg=meas["tilt_deg"])
    jpose = dict(height_mm=meas["joint_height_mm"], pivot_deg=meas["joint_pivot_deg"], tilt_deg=meas["joint_tilt_deg"])
    for key, agree in (("height_mm", GEOM_AGREE_MM), ("pivot_deg", GEOM_AGREE_DEG), ("tilt_deg", GEOM_AGREE_DEG)):
        if abs(pose[key] - jpose[key]) > agree:
            reasons.append(f"measurement disagreement {key}: {pose[key]:.3f} vs joint {jpose[key]:.3f}")
    damage = sorted(set(meas["disallowed_damage"]) | set(meas["mechanism_damage"]))
    if damage:
        reasons.append("damage:" + ",".join(damage))
    if meas["base_tip_or_slide"]:
        reasons.append("stand would tip or slide on the desk (freestanding foot check)")
    if meas["robot_box_contact_steps"] > 0:
        reasons.append("robot touched the box")
    if meas["wrist_force_abort"]:
        reasons.append(f"wrist force > {ABORT_N:g} N for >= {ABORT_S:g} s (episode abort)")
    if meas["furniture_contact_steps"] > 0 or meas["max_furniture_contact_n"] > FURNITURE_N:
        reasons.append(f"screen touched desk/furniture ({meas['furniture_contact_steps']} steps, "
                       f"max {meas['max_furniture_contact_n']:.2f} N)")
    goal, axes = TARGETS[target]
    err = {}
    for (ax, key, tol), g in zip(AXIS_KEYS, goal):
        err[ax] = abs(pose[key] - g)
        if ax not in axes and err[ax] >= tol:
            reasons.append(f"non-goal axis {key} moved: {pose[key]:.2f} (goal {g:g}, tol {tol:g})")
    if reasons:
        return _out(0.0, False, True, reasons, pose=pose)

    stage = {}
    for (ax, key, tol), g, s in zip(AXIS_KEYS, goal, START):
        if ax in axes:
            if err[ax] < tol:
                stage[key] = 1.0
            elif abs(pose[key] - s) < tol:      # moved less than the tolerance: not moved (pin play, sensor noise)
                stage[key] = 0.0
            else:
                stage[key] = max(0.0, min(1.0, 1.0 - err[ax] / abs(g - s)))
    within = all(err[ax] < tol for ax, _, tol in AXIS_KEYS)
    hands_off = not meas["hands_on_stand_at_end"]
    axes_credit = sum(stage.values()) / len(stage) * (1.0 if hands_off else 0.5)
    hold_credit = min(meas["hold_s"], HOLD_S) / HOLD_S if (within and hands_off) else 0.0
    partial = min(PARTIAL_CAP, W_AXES * axes_credit + W_HOLD * hold_credit)
    all_pass = within and hands_off and meas["hold_s"] >= HOLD_S - 1e-9
    src = meas["source_success_final"]
    detail = dict(stage=stage, axes_credit=axes_credit, hold_credit=hold_credit, pose=pose,
                  weights=dict(axes=W_AXES, hold=W_HOLD))
    if not hands_off:
        reasons.append("robot still touches the stand at the end; axis credit halved, no hold credit")
    for ax, key, tol in AXIS_KEYS:
        if ax in axes and err[ax] >= tol:
            reasons.append(f"{key} error {err[ax]:.2f} >= {tol:g}")
    if within and hands_off and meas["hold_s"] < HOLD_S - 1e-9:
        reasons.append(f"hands-off hold {meas['hold_s']:.2f} s < {HOLD_S:g} s")
    if all_pass and not src:
        return _out(0.0, False, True, reasons + ["scorer full pass but source predicate false; fail closed"], **detail)
    if all_pass and src:
        return _out(1.0, True, False, [], **detail)
    if src:
        reasons.append("source success latched earlier but final state no longer passes; no full credit")
    return _out(round(partial, 4), False, False, reasons, **detail)
