"""Stimulus + reaction check of finished demo / negative runs (reads JSON only).

    python negcheck.py <out.json> <run_dir> [<run_dir> ...]

For each trial directory (a run dir, or each trial*/ inside a multi-trial run) it reads result.json and trace.jsonl,
checks that the stimulus the stage is meant to produce actually happened (from the measured poses, the press joint
angles and the contact monitor), and that the final score is the expected level.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

EXPECT = {
    "demo": 1.0,
    "idle": 0.0,
    "replay": 0.0,
    "neg-inner-disc": 0.0,
    "neg-off-centre": 0.0,
    "neg-not-home": 0.25,
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
    mpeak = end.get("monitor_peak", {}) or {}
    peak = end.get("peak", {}) or {}
    mech = end.get("mech", {}) or {}
    if stage == "demo":
        return bool(res["success"]) and not veto.get("jammed") and not veto.get("brinelled"), dict(
            final=m_end, monitor_peak=mpeak, peak=peak, mech=mech)
    if stage == "idle":
        return max(abs(m_of(r).get("theta_deg", 0.0)) for r in rows) < 0.5, dict(final=m_end)
    if stage == "replay":
        return True, dict(final=m_end, veto=veto, monitor_peak=mpeak)
    if stage == "neg-inner-disc":
        # the 32 mm disc pressed on the inner ring with >= 20 N
        return mpeak.get("inner_n", 0.0) >= 20.0 and veto.get("brinelled"), dict(
            inner_peak_n=mpeak.get("inner_n"), veto=veto, final=m_end)
    if stage == "neg-off-centre":
        return mpeak.get("ecc_pushed_mm", 0.0) > 2.0 and veto.get("jammed"), dict(
            ecc_pushed_mm=mpeak.get("ecc_pushed_mm"), veto=veto, final=m_end)
    if stage == "neg-not-home":
        deep = max(m_of(r).get("depth_mm", 0.0) for r in rows)
        return 2.0 <= deep < 6.9 and not m_end.get("seated"), dict(max_depth_mm=deep, final=m_end)
    return False, {}


def check_dir(d):
    res = json.loads((d / "result.json").read_text())
    rows = load_trace(d)
    stage = res["stage"]
    done, ev = stimulus(stage, rows, res)
    want = EXPECT.get(stage)
    ok = done and (want is None or abs(res["task_score"] - want) < 1e-9)
    return dict(dir=str(d), stage=stage, score=res["task_score"], expected=want, stimulus=done, ok=ok,
                steps=res["steps"], evidence=ev)


def main():
    out = Path(sys.argv[1])
    rows = []
    for a in sys.argv[2:]:
        d = Path(a)
        trials = sorted(d.glob("trial*_*")) or [d]
        for t in trials:
            if (t / "result.json").exists():
                rows.append(check_dir(t))
    out.write_text(json.dumps(rows, indent=1, default=float))
    for r in rows:
        print(("OK  " if r["ok"] else "FAIL"), r["stage"], r["score"], "stimulus", r["stimulus"], r["dir"])


if __name__ == "__main__":
    main()
