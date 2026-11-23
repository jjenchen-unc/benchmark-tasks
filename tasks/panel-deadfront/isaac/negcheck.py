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

EXPECT = {"demo": 1., "idle": 0., "replay": 0., "neg-straight": 0., "neg-held": .5, "neg-no-screw": .25}


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
    if stage == "neg-straight":
        approached = any(abs(m_of(r).get("guide_angle", 180)) < 8
                         and abs(m_of(r).get("cover_mm", [-1000])[0]) < 20 for r in rows)
        return bool(approached and "guided" not in end.get("monitor_first", {})), dict(final=m_end)
    if stage == "neg-held":
        return bool(end.get("guided") and m_end.get("seated") == [True] and m_end.get("tip_mm",1000)<30), dict(final=m_end)
    if stage == "neg-no-screw":
        return bool(end.get("guided") and m_end.get("flat") and m_end.get("seated") == [False]), dict(final=m_end)
    if stage == "replay":
        return True, dict(final=m_end, screws=screws)
    return False, {}


def check(d):
    res = json.loads((d / "result.json").read_text())
    rows = load_trace(d)
    stage = res["stage"]
    done, ev = stimulus(stage, rows, res, res.get("looks", []))
    want = EXPECT.get(stage)
    ok = done and res.get("stop") is None and want is not None and res["task_score"] == want
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
