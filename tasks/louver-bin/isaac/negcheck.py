"""Stimulus + reaction check of finished demo / negative / replay runs (reads JSON only).

    python negcheck.py <out.json> <run_dir> [<run_dir> ...]

For each trial directory (a run dir, or each trial*/ inside a multi-trial run) it reads result.json and trace.jsonl,
checks that the stimulus the stage is meant to produce actually happened (from the measured bin hang states and the
fingertip clearance in the trace), and that the final score is the expected level.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

EXPECT = {"demo": 1.0, "idle": 0.0, "replay": 0.0, "neg-wrong-row": 0.0, "neg-on-lip": 0.0, "neg-hold": 0.5}


def load_trace(d):
    return [json.loads(line) for line in open(d / "trace.jsonl")]


def m_of(r):
    return r["detail"].get("measure", {}) or {}


def stimulus(stage, rows, res):
    """(done, evidence) from the measured trace, independent of the score."""
    ms = [m_of(r) for r in rows if m_of(r)]
    end = ms[-1] if ms else {}
    b = end.get("bin", {}) or {}
    tr = end.get("target_row")
    base = dict(target_row=tr, final_bin={k: b.get(k) for k in ("row", "dz", "tab_x", "in_channel", "level_deg")},
                ref_hangs=end.get("ref_hangs"), gap_mm=end.get("gap_mm"), final_tip_mm=end.get("tip_mm"),
                grip_peak_n=rows[-1]["detail"].get("grip_peak_n") if rows else None)
    if stage == "demo":
        return bool(res["success"] and end.get("hangs") and b.get("row") == tr and end.get("ref_hangs")), base
    if stage == "idle":
        return all(not m.get("hangs") for m in ms), base
    if stage == "replay":
        return True, base
    if stage == "neg-wrong-row":
        # the bin hangs properly (tab in a channel, level), on the row above the target, hands clear
        return bool(b.get("in_channel") and b.get("row") == tr + 1 and b.get("level_deg", 99) <= 5.0
                    and end.get("tip_mm", 0) >= 50.0 and end.get("ref_hangs")), base
    if stage == "neg-on-lip":
        # the tab was set down ON the target lip (shelf about one tab height above the lip, not in the channel) while
        # held, and after letting go the bin left that place (dropped below it or tipped more than 5 deg)
        held_on_lip = [m for m in ms if (m.get("bin") or {}).get("row") == tr
                       and 0.008 < (m.get("bin") or {}).get("dz", -1) < 0.0135
                       and not (m.get("bin") or {}).get("in_channel") and m.get("tip_mm", 99) < 50.0]
        base["steps_tab_on_lip_held"] = len(held_on_lip)
        dropped = (b.get("dz", 0) < -0.005 or b.get("row") != tr or b.get("level_deg", 0) > 5.0) and \
            end.get("tip_mm", 0) >= 50.0
        return bool(held_on_lip and dropped and not end.get("hangs")), base
    if stage == "neg-hold":
        # hung correctly on the target row, but a hand still on the bin at the end
        return bool(end.get("hangs") and b.get("row") == tr and end.get("tip_mm", 99) < 50.0), base
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
