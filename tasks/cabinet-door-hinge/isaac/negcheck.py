"""Stimulus + reaction check of finished demo / negative / replay runs (reads JSON only).

    python negcheck.py <out.json> <run_dir> [<run_dir> ...]

For each trial directory (a run dir, or each trial*/ inside a multi-trial run) it reads result.json and trace.jsonl,
checks that the stimulus the stage is meant to produce actually happened (from the hinge states, hinge events, door
pose and knuckle angles in the trace), and that the final score is the expected level.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

EXPECT = {"demo": 1.0, "idle": 0.0, "neg-one-clip": 0.25, "neg-high": 0.0, "neg-twist": 0.0, "neg-drop": 0.0,
          "replay": 0.0}


def load_trace(d):
    return [json.loads(line) for line in open(d / "trace.jsonl")]


def stimulus(stage, rows, res):
    """(done, evidence) from the measured trace, independent of the score."""
    end = rows[-1]["detail"] if rows else {}
    events = end.get("events", []) or []
    looks = {x["at"]: x for x in res.get("looks", [])}
    hinges_seen = [r.get("hinges") for r in rows]
    ever_hooked = [any(h and h[i] in ("hooked", "clipped") for h in hinges_seen) for i in range(2)]
    ever_clipped = [any(h and h[i] == "clipped" for h in hinges_seen) for i in range(2)]
    push = end.get("peak_rear_push_n")
    if stage == "demo":
        return res["success"], dict(events=events, final=end.get("measure"), grip_peak_n=end.get("grip_peak_n"),
                                    peak_rear_push_n=push)
    if stage == "idle":
        z0 = rows[0]["door_mm"][2] if rows else 0
        moved = max(abs(r["door_mm"][2] - z0) for r in rows) if rows else 0
        return moved < 1.0 and not any(ever_hooked), dict(door_z_moved_mm=round(moved, 3))
    if stage == "neg-one-clip":
        # both lugs hooked, the lower arm pressed and clipped, the upper never pressed (never clipped)
        return (all(ever_hooked) and ever_clipped[0] and not ever_clipped[1]), dict(
            events=events, final_hinges=end.get("hinges"), peak_rear_push_n=push)
    if stage == "neg-high":
        # lugs held above the noses (never hooked); both rears pressed (the press reached the rear: push recorded
        # by the hand's travel: see looks 'press'); nothing hooked or clipped
        presses = [x for x in res.get("looks", []) if x["at"] == "press"]
        return (not any(ever_hooked) and len(presses) == 2), dict(presses=presses, final_hinges=end.get("hinges"),
                                                                  held_high=looks.get("held high"))
    if stage == "neg-twist":
        tw = looks.get("twisted", {})
        return (all(ever_hooked) and not any(ever_clipped) and "twisted" in looks), dict(
            events=events, twisted=tw, final_hinges=end.get("hinges"))
    if stage == "neg-drop":
        z0 = rows[0]["door_mm"][2]
        zmin = min(r["door_mm"][2] for r in rows)
        return (z0 - zmin > 30.0 and not any(ever_hooked)), dict(door_z0_mm=z0, door_z_min_mm=zmin)
    if stage == "replay":
        return len(rows) > 100, dict(start=res.get("start"), final_hinges=end.get("hinges"), events=events)
    return False, {}


def check(d):
    res = json.loads((d / "result.json").read_text())
    stage = res["stage"]
    rows = load_trace(d)
    done, ev = stimulus(stage, rows, res)
    want = EXPECT.get(stage)
    ok = bool(done) and want is not None and abs(res["task_score"] - want) < 1e-9 and not res.get("nonfinite")
    return dict(dir=str(d), stage=stage, task_score=res["task_score"], expected=want, stimulus_done=bool(done),
                evidence=ev, steps=res["steps"], stop=res.get("stop"), nonfinite=res.get("nonfinite"), passed=ok)


def main(out, dirs):
    rows = []
    for x in dirs:
        p = Path(x)
        trials = sorted(p.glob("trial*_*")) if not (p / "result.json").exists() else [p]
        rows += [check(t) for t in trials]
    summary = dict(passed=all(r["passed"] for r in rows), n=len(rows), rows=rows)
    Path(out).write_text(json.dumps(summary, indent=1, default=float))
    for r in rows:
        print(("PASS" if r["passed"] else "FAIL"), r["stage"], r["task_score"], r["expected"],
              json.dumps(r["evidence"], default=float)[:400])
    return summary["passed"]


if __name__ == "__main__":
    sys.exit(0 if main(sys.argv[1], sys.argv[2:]) else 1)
