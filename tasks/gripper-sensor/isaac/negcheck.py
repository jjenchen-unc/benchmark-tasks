"""Stimulus + reaction check of finished demo / negative runs (reads JSON only).

    python negcheck.py <out.json> <run_dir> [<run_dir> ...]

For each trial directory (a run dir, or each trial*/ inside a multi-trial run) it reads result.json and trace.jsonl,
checks that the stimulus the stage is meant to produce actually happened (from the logged switch position, valve
events, screw angle and screwdriver engagement), and that the final score is the expected level.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

EXPECT = {
    "demo": 1.0,
    "idle": 0.0,
    "replay": 0.0,
    "neg-end-of-stroke": 0.0,
    "neg-no-override": 0.0,
    "neg-no-lock": 0.75,
    "neg-light-touch": 0.75,
}
STROKE_MM = 15.0


def load_trace(d):
    return [json.loads(line) for line in open(d / "trace.jsonl")]


def stimulus(stage, rows, res):
    """(done, evidence) from the logged trace, independent of the score."""
    det = [r["detail"] or {} for r in rows]
    end = det[-1] if det else {}
    u = [d.get("u_switch_mm") for d in det if d.get("u_switch_mm") is not None]
    moved = (max(u) - min(u)) if u else 0.0
    valve_on_steps = sum(1 for d in det if d.get("override_on"))
    jaw_peak = max((d.get("jaw_travel_mm") or 0.0) for d in det) if det else 0.0
    ev = dict(
        u_start_mm=u[0] if u else None,
        u_end_mm=end.get("u_switch_mm"),
        part_travel_mm=end.get("travel_mm"),
        switch_moved_mm=round(moved, 3),
        valve_on_steps=valve_on_steps,
        jaw_peak_mm=jaw_peak,
        screw_deg=end.get("screw_deg"),
        locked=end.get("locked"),
        engage_s=end.get("engage_s"),
        engage_push_peak_n=end.get("engage_push_peak_n"),
        ovr_peak_n=end.get("ovr_peak_n"),
        outcome_level=end.get("outcome_level"),
        checks={k: end.get(k) for k in ("on_part", "off_open", "off_empty")},
    )
    if stage == "demo":
        return bool(res["success"] and end.get("locked") and valve_on_steps > 0), ev
    if stage == "idle":
        return moved < 0.01 and valve_on_steps == 0, ev
    if stage == "replay":
        return len(rows) > 100, ev
    if stage == "neg-end-of-stroke":
        # set where it lights with the jaws closed empty (end of stroke), then locked
        return bool(
            abs(end.get("u_switch_mm", 0.0) - STROKE_MM) <= 1.0 and end.get("locked")
        ), ev
    if stage == "neg-no-override":
        # the override was never worked (jaws never closed), the switch was slid by about 6 mm and locked
        return bool(
            valve_on_steps == 0
            and jaw_peak < 0.01
            and moved >= 4.0
            and end.get("locked")
        ), ev
    if stage == "neg-no-lock":
        return bool(
            end.get("outcome_level") == 0.75 and (end.get("screw_deg") or 0.0) == 0.0
        ), ev
    if stage == "neg-light-touch":
        # the screwdriver was turned at the screw without being pushed in: the screw never turned to the lock
        tips = [d.get("driver_tip_mm") for d in det if d.get("driver_tip_mm")]
        return bool(
            end.get("outcome_level") == 0.75
            and (end.get("screw_deg") or 0.0) < 90.0
            and tips
        ), ev
    return False, ev


def trials(run):
    run = Path(run)
    subs = sorted(p for p in run.glob("trial*_*") if p.is_dir())
    return subs or [run]


def main():
    out, runs = sys.argv[1], sys.argv[2:]
    rows = []
    for r in runs:
        for d in trials(r):
            res = json.loads((d / "result.json").read_text())
            stage = res["stage"]
            done, ev = stimulus(stage, load_trace(d), res)
            want = EXPECT.get(stage)
            ok = done and want is not None and abs(res["task_score"] - want) < 1e-9
            rows.append(
                dict(
                    run=str(d),
                    stage=stage,
                    score=res["task_score"],
                    expected=want,
                    stimulus_done=done,
                    passed=ok,
                    steps=res["steps"],
                    stop=res["stop"],
                    evidence=ev,
                )
            )
            print(
                f"{'PASS' if ok else 'FAIL'} {stage}: score {res['task_score']} (expected {want}), "
                f"stimulus {'done' if done else 'NOT done'}: {json.dumps(ev)}"
            )
    Path(out).write_text(
        json.dumps(dict(passed=all(x["passed"] for x in rows), rows=rows), indent=2)
    )


if __name__ == "__main__":
    main()
