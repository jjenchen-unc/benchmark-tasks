"""Stimulus + reaction check of finished demo / negative runs (reads JSON only).

    python negcheck.py <out.json> <run_dir> [<run_dir> ...]

For each trial directory (a run dir, or each trial*/ inside a multi-trial run) it reads result.json and trace.jsonl,
checks that the stimulus the stage is meant to produce actually happened (from the measured anchor state, the
gripper commands and the contact forces in the trace), and that the final score is the expected level.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

EXPECT = {
    "demo": 1.0,
    "idle": 0.0,
    "replay": 0.0,
    "neg-no-tension": 0.0,
    "neg-no-snap": 0.6,
    "neg-early-pull": 0.0,
    "pinch-bend": 1.0,
}


def load_trace(d):
    return [json.loads(line) for line in open(d / "trace.jsonl")]


def m_of(r):
    return r["detail"].get("measure", {}) or {}


def stimulus(stage, rows, res):
    """(done, evidence) from the measured trace, independent of the score."""
    end = rows[-1]["detail"] if rows else {}
    m_end = end.get("measure", {}) or {}
    looks = {lk["at"]: lk for lk in res.get("looks", [])}
    if stage == "demo":
        return res["success"], dict(final=m_end, break_info=end.get("break_info"), peak=end.get("peak"))
    if stage == "idle":
        q0 = rows[0]["detail"].get("q_mm") if rows else None
        moved = max(abs((r["detail"].get("q_mm") or 0) - (q0 or 0)) for r in rows) if rows else 0
        return moved < 0.5, dict(cap_moved_mm=round(moved, 3))
    if stage == "neg-no-tension":
        # left gripper open (command >= 0.15) while the right slides the cap, and the channel left the back face
        sliding = [r for r in rows if r["labels"][1] == "slide cap"]
        left_open = bool(sliding) and all(r["cmd"][6] >= 0.15 for r in sliding)
        f_max = max(((r["detail"].get("contacts") or {}).get("channel_n", 0.0) for r in sliding), default=0.0)
        gap_max = max((max(m_of(r).get("gap_mm") or [0]) for r in sliding), default=0.0)
        return left_open and gap_max > 3.0, dict(
            left_open_while_sliding=left_open, channel_force_max_n=round(f_max, 3), gap_max_mm=round(gap_max, 2),
            final_play_mm=m_end.get("play_mm"), final=m_end)
    if stage == "neg-no-snap":
        return (not end.get("broken")) and bool(m_end.get("seated")), dict(broken=end.get("broken"), final=m_end)
    if stage == "neg-early-pull":
        phi_at_pull = looks.get("pulled early", {}).get("phi")
        return m_end.get("through") is False, dict(phi_at_pull=phi_at_pull, final=m_end)
    if stage == "pinch-bend":
        return bool(end.get("broken")) and res["success"], dict(break_info=end.get("break_info"), final=m_end)
    if stage == "replay":
        return True, dict(final=m_end)
    return False, {}


def check(d):
    res = json.loads((d / "result.json").read_text())
    rows = load_trace(d)
    stage = res["stage"]
    done, ev = stimulus(stage, rows, res)
    want = EXPECT.get(stage)
    score = res["task_score"]
    ok_score = want is None or (score >= 1.0 if want == 1.0 else score <= want + 1e-9)
    return dict(run=str(d), stage=stage, setup=res.get("setup"), score=score, steps=res["steps"], expected=want,
                stimulus=done, score_ok=ok_score, evidence=ev)


def main(out, dirs):
    rows = []
    for d in map(Path, dirs):
        trials = sorted(d.glob("trial*_*")) or [d]
        for t in trials:
            if (t / "result.json").exists():
                rows.append(check(t))
    Path(out).write_text(json.dumps(rows, indent=2, default=float))
    for r in rows:
        print(f"{r['stage']:16s} score {r['score']} steps {r['steps']} stimulus {r['stimulus']} ok {r['score_ok']}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2:])
