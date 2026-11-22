"""Stimulus + reaction check of finished demo / negative runs (reads JSON only).

    python negcheck.py <out.json> <run_dir> [<run_dir> ...]

For each trial directory (a run dir, or each trial*/ inside a multi-trial run) it reads result.json and trace.jsonl,
checks that the stimulus the stage is meant to produce actually happened (from the measured screen and mat poses, the
mat joint state and the latched start record), and that the final score is the expected level.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

EXPECT = {
    "demo": 1.0,
    "idle": 0.0,
    "replay": 0.0,
    "neg-start-only": 0.0,
    "neg-no-clean": 0.0,
    "neg-proud": 0.0,
    "neg-screen-out": 0.0,
    "no-start": 0.25,
}


def load_trace(d):
    return [json.loads(line) for line in open(d / "trace.jsonl")]


def m_of(r):
    return (r.get("detail") or {}).get("measure") or {}


def stimulus(stage, rows, res):
    """(done, evidence) from the measured trace, independent of the score."""
    end = rows[-1]["detail"] if rows else {}
    m_end = end.get("measure", {}) or {}
    at = end.get("at_start")
    started = end.get("started")
    max_gap = max((max(m_of(r).get("end_gap_mm") or [0]) for r in rows), default=0.0)
    ever_unbonded = any(not m_of(r).get("bonded", True) for r in rows)
    ev = dict(started=started, at_start=at, max_end_gap_mm=round(max_gap, 2), final=m_end,
              button_peak_n=end.get("button_peak_n"))
    if stage == "demo":
        return bool(res["success"] and at and at["seated"] and at["clean"] and m_end.get("mat_in_bin")), ev
    if stage in ("idle", "replay"):
        return True, ev
    if stage == "neg-start-only":
        return bool(started and at and at["mat_on_screen"] and max_gap < 1.0), ev
    if stage == "neg-no-clean":
        return bool(started and at and at["mat_on_screen"] and max_gap > 130.0 and not ever_unbonded), ev
    if stage == "neg-proud":
        g = (at or {}).get("end_gap_mm") or [0, 0]
        return bool(started and at and at["clean"] and max(g) > 2.5 and m_end.get("mat_in_bin")), ev
    if stage == "neg-screen-out":
        g = (at or {}).get("end_gap_mm") or [0, 0]
        return bool(started and at and at["clean"] and min(g) > 100.0 and m_end.get("mat_in_bin")), ev
    if stage == "no-start":
        return bool(not started and m_end.get("seated") and m_end.get("mat_in_bin")), ev
    return False, ev


def trial_dirs(d):
    d = Path(d)
    subs = sorted(p for p in d.glob("trial*_*") if p.is_dir())
    return subs or [d]


def main():
    out, dirs = sys.argv[1], sys.argv[2:]
    rows_out = []
    for d in dirs:
        for t in trial_dirs(d):
            res = json.loads((t / "result.json").read_text())
            rows = load_trace(t)
            stage = res["stage"]
            done, ev = stimulus(stage, rows, res)
            exp = EXPECT.get(stage)
            rows_out.append(dict(dir=str(t), stage=stage, task_score=res["task_score"], expected=exp,
                                 stimulus_done=done, evidence=ev, steps=res["steps"], stop=res["stop"],
                                 score_ok=exp is None or abs(res["task_score"] - exp) < 1e-9))
    rep = dict(passed=all(r["stimulus_done"] and r["score_ok"] for r in rows_out), n=len(rows_out), rows=rows_out)
    Path(out).write_text(json.dumps(rep, indent=1, default=float))
    for r in rows_out:
        print(r["stage"], r["task_score"], "expected", r["expected"], "stimulus", r["stimulus_done"], r["dir"])
    print("PASSED" if rep["passed"] else "FAILED")


if __name__ == "__main__":
    main()
