"""Stimulus + reaction check of finished demo / negative runs (reads JSON only).

    python negcheck.py <out.json> <run_dir> [<run_dir> ...]

For each trial directory (a run dir, or each trial*/ inside a multi-trial run) it reads result.json and trace.jsonl,
checks that the stimulus the stage is meant to produce actually happened (from the measured cover pose, the contact
monitor and the screw model's state, independent of the score), and that the final score is the expected level.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

# neg-no-centre has no expected level: within the slots' play the cover self-centres once released (ev1: 1.0)
EXPECT = {"demo": 1.0, "idle": 0.0, "replay": 0.0, "neg-no-centre": None, "neg-no-hold": 0.0, "neg-one-screw": 0.5}


def load_trace(d):
    return [json.loads(line) for line in open(d / "trace.jsonl")]


def m_of(r):
    return (r.get("detail") or {}).get("measure", {}) or {}


def stimulus(stage, rows, res, looks):
    end = rows[-1]["detail"] if rows else {}
    m_end = end.get("measure", {}) or {}
    peaks = end.get("peaks", {}) or {}
    screws = end.get("screws", {}) or {}
    if stage == "demo":
        return bool(res["success"]), dict(final=m_end, screws=screws, peaks=peaks)
    if stage == "idle":
        moved = max(abs(a - b) for a, b in zip(m_of(rows[-1]).get("cover_mm", [0, 0, 0]),
                                                m_of(rows[0]).get("cover_mm", [0, 0, 0])))
        return moved < 1.0, dict(cover_moved_mm=round(moved, 3))
    if stage == "neg-no-centre":
        # cover hung 1.5 mm off sideways and swung shut without correction: it lands on a shroud (breaker contact,
        # corners far off) -- what happens after the hands let go is reported, not expected
        off = [lk for lk in looks if lk.get("at") == "offset"]
        dy = off[0].get("dy_mm") if off else None
        shut = [lk for lk in looks if lk.get("at") == "shut attempt"]
        gap = (shut[0].get("measure") or {}).get("worst_gap_mm") if shut else None
        return (dy is not None and abs(dy) > 1.1 and peaks.get("breaker_n", 0) > 1.0 and (gap or 0) > 5.0), dict(
            offset_dy_mm=dy, worst_gap_at_shut_attempt_mm=gap, breaker_peak_n=peaks.get("breaker_n"), final=m_end)
    if stage == "neg-no-hold":
        # hands off the cover before the screws; it is not held flat and tips open when the first screw is driven
        lg = [lk for lk in looks if lk.get("at") == "let go before screws"]
        after = [r for r in rows if lg and r["step"] > lg[0]["step"]]
        worst = max((m_of(r).get("worst_gap_mm") or 0.0) for r in after) if after else None
        return (worst is not None and worst > 5.0 and not m_end.get("flat")), dict(
            worst_gap_after_release_mm=worst, final=m_end, screws=screws)
    if stage == "neg-one-screw":
        return (screws.get("seated") == [False, True]), dict(screws=screws, final=m_end)
    if stage == "replay":
        return True, dict(final=m_end, screws=screws)
    return False, {}


def check(d):
    res = json.loads((d / "result.json").read_text())
    rows = load_trace(d)
    stage = res["stage"]
    done, ev = stimulus(stage, rows, res, res.get("looks", []))
    want = EXPECT.get(stage)
    ok = done and (want is None or (res["task_score"] == want if want != 0.0 else res["task_score"] < 1.0))
    return dict(run=str(d), stage=stage, setup=res.get("setup"), score=res["task_score"], steps=res["steps"],
                stop=res.get("stop"), expected=want, stimulus=done, ok=ok, evidence=ev)


def main():
    out = Path(sys.argv[1])
    rows = []
    for a in sys.argv[2:]:
        d = Path(a)
        trials = sorted(d.glob("trial*_*")) or [d]
        for t in trials:
            rows.append(check(t))
    out.write_text(json.dumps(rows, indent=1, default=float))
    for r in rows:
        print(r["stage"], r["score"], r["steps"], "stimulus" if r["stimulus"] else "NO-STIMULUS",
              "OK" if r["ok"] else "FAIL", r["run"])


if __name__ == "__main__":
    main()
