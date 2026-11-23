"""Stimulus + reaction check of finished demo / negative runs (reads JSON only).

    python negcheck.py <out.json> <run_dir> [<run_dir> ...]

For each trial directory (a run dir, or each trial*/ inside a multi-trial run) it reads result.json and trace.jsonl,
checks that the stimulus the stage is meant to produce actually happened (from the measured pressures, knob and lever
state and the contact monitor, independent of the score), and that the final score is the expected level.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

EXPECT = {
    "demo": 1.0,
    "idle": 0.0,
    "neg-no-bleed": 0.0,
    "neg-shut-in": 0.0,
    "neg-left-open": 0.0,
    "replay": 0.0,
}


def load_trace(d):
    return [json.loads(line) for line in open(d / "trace.jsonl")]


def stimulus(stage, rows, res):
    """(done, evidence) from the measured trace, independent of the score."""
    det = [r["detail"] for r in rows if r.get("detail")]
    end = det[-1] if det else {}
    turns = [d.get("turns") or 0.0 for d in det]
    valve = [d.get("valve_deg") or 0.0 for d in det]
    ev = dict(
        final_gauge_psi=end.get("gauge_psi"),
        final_line_psi=end.get("line_psi"),
        final_set_psi=end.get("set_psi"),
        final_knob_out_mm=end.get("knob_out_mm"),
        final_valve_deg=end.get("valve_deg"),
        final_contact=end.get("contact"),
        final_cond=end.get("cond"),
        min_turns=min(turns) if turns else 0.0,
        max_valve_deg=max(valve) if valve else 0.0,
        start_line_psi=det[0].get("line_psi") if det else None,
        min_line_psi=end.get("min_line_psi"),
        peaks=end.get("peaks"),
    )
    if stage == "demo":
        done = abs((end.get("gauge_psi") or 0) - 90.0) <= 2.0 and abs((end.get("set_psi") or 0) - 90.0) <= 2.0
    elif stage == "idle":
        done = ev["max_valve_deg"] < 1.0 and abs(ev["min_turns"]) < 1e-3
    elif stage == "neg-no-bleed":
        # the knob was turned down (setting well below the line) and the bleed never opened; the line did not fall
        done = (ev["max_valve_deg"] < 3.0 and (end.get("set_psi") or 999) < (end.get("line_psi") or 0) - 5.0
                and abs((end.get("line_psi") or 0) - (ev["start_line_psi"] or -1)) < 0.5)
    elif stage == "neg-shut-in":
        # setting far below 90, line shut in above the setting, bleed closed at the end
        done = ((end.get("set_psi") or 999) < 86.0 and (end.get("line_psi") or 0) > (end.get("set_psi") or 0) + 3.0
                and (end.get("valve_deg") or 99) <= 3.0 and ev["max_valve_deg"] > 8.0)
    elif stage == "neg-left-open":
        # bleed open at the end, the rest set
        done = (end.get("valve_deg") or 0) > 45.0 and abs((end.get("set_psi") or 0) - 90.0) <= 3.0
    elif stage == "replay":
        done = len(rows) > 100
    else:
        done = None
    return done, ev


def check(d):
    res = json.loads((d / "result.json").read_text())
    rows = load_trace(d)
    stage = res["stage"]
    done, ev = stimulus(stage, rows, res)
    exp = EXPECT.get(stage)
    ok_score = exp is None or (res["task_score"] == exp)
    return dict(dir=str(d), stage=stage, setup=res.get("setup"), score=res["task_score"], expected=exp,
                score_ok=ok_score, stimulus_done=done, steps=res["steps"], stop=res.get("stop"),
                grip_peak_n=res.get("grip_peak_n"), evidence=ev)


def main(out, dirs):
    rows = []
    for x in dirs:
        p = Path(x)
        trials = sorted(t for t in p.glob("trial*") if t.is_dir()) or [p]
        for t in trials:
            if (t / "result.json").exists():
                rows.append(check(t))
    Path(out).write_text(json.dumps(rows, indent=2, default=float))
    for r in rows:
        print(r["stage"], r["setup"], "score", r["score"], "expected", r["expected"], "stimulus", r["stimulus_done"])


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2:])
