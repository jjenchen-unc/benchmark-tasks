"""Stimulus + reaction check of finished demo / negative / replay runs (reads JSON only).

    python negcheck.py <out.json> <run_dir> [<run_dir> ...]

For each trial directory (a run dir, or each trial*/ inside a multi-trial run) it reads result.json and trace.jsonl,
checks that the stimulus the stage is meant to produce actually happened (from the measured card rise, latch angle,
latch-hook load, contact friction and fingertip clearance), and that the final score is the expected level.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

EXPECT = {"demo": 1.0, "idle": 0.0, "replay": 0.0, "neg-nopress": 0.0, "neg-early": 0.0, "neg-held": 0.5}


def load_trace(d):
    return [json.loads(line) for line in open(d / "trace.jsonl")]


def meas(r):
    return (r.get("detail") or {}).get("measure") or {}


def stimulus(stage, rows, res):
    """(done, evidence) from the measured trace, independent of the score."""
    end = rows[-1]["detail"] if rows else {}
    m = end.get("measure", {}) or {}
    rise_max = max((meas(r).get("rise_mm", 0.0) for r in rows if meas(r)), default=0.0)
    latch_max = max((meas(r).get("latch_deg", 0.0) for r in rows if meas(r)), default=0.0)
    grip_max = max(((r.get("detail") or {}).get("grip_peak_n") or 0.0) for r in rows) if rows else 0.0
    cf_min = min(((r.get("detail") or {}).get("contact_f_n") or 0.0) for r in rows) if rows else 0.0
    # the pull against the closed latch: the largest hook load while the latch stayed under 5 deg
    hook_closed = max((((r.get("detail") or {}).get("hook_n") or 0.0) for r in rows
                       if meas(r) and meas(r).get("latch_deg", 99) < 5.0), default=0.0)
    base = dict(rise_max_mm=rise_max, final_rise_mm=m.get("rise_mm"), latch_max_deg=latch_max,
                hook_peak_n=end.get("hook_peak_n"), hook_closed_max_n=hook_closed, contact_f_min_n=cf_min,
                grip_peak_n=grip_max, final_tip_mm=m.get("tip_mm"), on_mat=end.get("on_mat"), out=end.get("out"),
                broken=end.get("broken"), dropped=end.get("dropped"), press_peak_n=end.get("press_peak_n"))
    if stage == "demo":
        return bool(res["success"]) and latch_max >= 17.0, base
    if stage == "idle":
        return rise_max < 2.0, base
    if stage == "replay":
        return True, base
    if stage == "neg-nopress":
        # the card was gripped and pulled (pull seen as hook load or contact friction) with no robot contact on the
        # latch ever; the card either stayed in the slot or was forced past the hook (latch broken)
        ok = (end.get("press_peak_n") or 0.0) < 1.0 and grip_max > 20.0 and (hook_closed > 3.0 or cf_min < -5.0) and (
            rise_max < 9.5 or bool(end.get("broken")))
        return ok, base
    if stage == "neg-early":
        # the latch was pressed open (>= 17 deg) and let go before the pull (the hook back under 5 deg when the pull
        # starts); the card stayed in the slot or was forced past the closed hook (latch broken)
        ok = latch_max >= 17.0 and grip_max > 20.0 and hook_closed > 3.0 and (rise_max < 9.5 or bool(end.get("broken")))
        return ok, base
    if stage == "neg-held":
        # out of the slot, still held at the end (a fingertip within 40 mm)
        ok = bool(end.get("out")) and (m.get("tip_mm") or 99) < 40.0 and not end.get("broken")
        return ok, base
    return False, base


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
        print(("PASS" if r["passed"] else "FAIL"), r["stage"], r["task_score"], r["expected"], r["evidence"])
    return summary["passed"]


if __name__ == "__main__":
    sys.exit(0 if main(sys.argv[1], sys.argv[2:]) else 1)
