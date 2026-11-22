"""Stimulus + reaction check of finished demo / negative runs (reads JSON only).

    python negcheck.py <out.json> <run_dir> [<run_dir> ...]

For each trial directory (a run dir, or each trial*/ inside a multi-trial run) it reads result.json and trace.jsonl,
checks that the stimulus the stage is meant to produce actually happened (from the measured ring pose, the lift-pin
height and the contact monitor), and that the final score is the expected level.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

EXPECT = {
    "demo": 1.0,
    "idle": 0.0,
    "neg-unclocked": 0.0,
    "neg-offset": 0.0,
    "neg-drop": 0.0,
    "neg-press": 0.0,
    "neg-not-lowered": 0.0,
    "neg-hand-in": 0.5,
    "neg-touch-chuck": 0.0,
}


def load_trace(d):
    return [json.loads(line) for line in open(d / "trace.jsonl")]


def m_of(r):
    return r["detail"].get("measure", {}) or {}


def stimulus(stage, rows, res):
    """(done, evidence) from the measured trace, independent of the score."""
    end = rows[-1]["detail"] if rows else {}
    m_end = end.get("measure", {}) or {}
    veto = end.get("veto", {}) or {}
    peak = end.get("peak", {}) or {}
    keys = end.get("keys", []) or []
    ever = lambda f: any(f(m_of(r)) for r in rows)  # noqa: E731
    if stage == "demo":
        return res["success"] and not any(veto.values()), dict(
            final=m_end, keys=keys, peak=peak, veto=veto
        )
    if stage == "idle":
        moved = (
            max(
                abs(m_of(r).get("dz_mm", 0) - m_of(rows[0]).get("dz_mm", 0))
                for r in rows
            )
            if rows
            else 0
        )
        return moved < 0.5, dict(ring_moved_mm=round(moved, 3))
    if stage == "neg-unclocked":
        # on the raised pins, misclocked (> 1 deg), RING DOWN pressed, ring ends about 3 mm up on the dowel tips
        on_pins_off = ever(
            lambda m: m.get("lift_mm", 0) > 24
            and abs(m.get("dz_mm", 0) - 25) < 1.5
            and abs(m.get("clock_deg", 0)) > 1.0
        )
        return (
            on_pins_off
            and any(k["key"] == "down" for k in keys)
            and m_end.get("lift_mm", 99) < 0
            and m_end.get("dz_mm", 0) > 2.0
            and veto.get("on_tips")
        ), dict(
            on_pins_misclocked=on_pins_off,
            final_dz_mm=m_end.get("dz_mm"),
            final_clock_deg=m_end.get("clock_deg"),
            keys=keys,
            veto=veto,
            tips_s=peak.get("tips_s"),
        )
    if stage == "neg-offset":
        off = ever(
            lambda m: m.get("lift_mm", 0) > 24
            and abs(m.get("dz_mm", 0) - 25) < 1.5
            and m.get("centre_mm", 0) > 1.0
        )
        return (
            off
            and any(k["key"] == "down" for k in keys)
            and m_end.get("lift_mm", 99) < 0
            and m_end.get("dz_mm", 0) > 2.0
            and veto.get("on_tips")
        ), dict(
            offset_on_pins=off,
            final_dz_mm=m_end.get("dz_mm"),
            final_centre_mm=m_end.get("centre_mm"),
            veto=veto,
            tips_s=peak.get("tips_s"),
        )
    if stage == "neg-drop":
        return bool(veto.get("dropped")), dict(
            fall_v=peak.get("fall_v"), dropped=veto.get("dropped")
        )
    if stage == "neg-press":
        return bool(veto.get("forced")), dict(
            support_n=peak.get("support_n"), press_s=peak.get("press_s")
        )
    if stage == "neg-not-lowered":
        # placed on the raised pins (centred within 1 mm, level), hands out, RING DOWN never pressed
        return (
            m_end.get("on_lift")
            and m_end.get("centre_mm", 99) < 1.0
            and m_end.get("tip_mm", 0) > 20
            and not keys
        ), dict(final=m_end, keys=keys)
    if stage == "neg-hand-in":
        return (m_end.get("seated") and m_end.get("in_bowl")), dict(
            seated=m_end.get("seated"), in_bowl=m_end.get("in_bowl")
        )
    if stage == "neg-touch-chuck":
        return bool(veto.get("chuck")), dict(
            chuck_n=peak.get("chuck_n"), seated_end=m_end.get("seated")
        )
    return False, {}


def check(d):
    res = json.loads((d / "result.json").read_text())
    stage = res["stage"]
    rows = load_trace(d)
    done, ev = stimulus(stage, rows, res)
    want = EXPECT.get(stage)
    ok = (
        bool(done)
        and want is not None
        and abs(res["task_score"] - want) < 1e-9
        and not res.get("nonfinite")
    )
    return dict(
        dir=str(d),
        stage=stage,
        task_score=res["task_score"],
        expected=want,
        stimulus_done=bool(done),
        evidence=ev,
        steps=res["steps"],
        stop=res.get("stop"),
        nonfinite=res.get("nonfinite"),
        passed=ok,
    )


def main(out, dirs):
    rows = []
    for x in dirs:
        p = Path(x)
        trials = sorted(p.glob("trial*_*")) if not (p / "result.json").exists() else [p]
        rows += [check(t) for t in trials]
    summary = dict(passed=all(r["passed"] for r in rows), n=len(rows), rows=rows)
    Path(out).write_text(json.dumps(summary, indent=1, default=float))
    for r in rows:
        print(
            ("PASS" if r["passed"] else "FAIL"),
            r["stage"],
            r["task_score"],
            r["expected"],
            r["evidence"],
        )
    return summary["passed"]


if __name__ == "__main__":
    sys.exit(0 if main(sys.argv[1], sys.argv[2:]) else 1)
