"""Stimulus + reaction check of finished demo / negative runs (reads JSON only).

    python negcheck.py <out.json> <run_dir> [<run_dir> ...]

For each trial directory (a run dir, or each trial*/ inside a multi-trial run) it reads result.json and trace.jsonl,
checks that the stimulus the stage is meant to produce actually happened (from the measured rider positions, the
pointer, the jar and the contact monitor, independent of the score), and that the final score is the expected level.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

EXPECT = {
    "demo": 1.0,
    "idle": 0.0,
    "neg-coarse-only": 0.0,
    "neg-unseated": 0.5,
    "neg-hand-on": 0.0,
    "replay": 0.0,
}


def load_trace(d):
    return [json.loads(line) for line in open(d / "trace.jsonl")]


def stimulus(stage, rows, res):
    """(done, evidence) from the measured trace, independent of the score."""
    det = [r["detail"] for r in rows if r.get("detail")]
    end = det[-1] if det else {}
    poise = [d.get("poise_g") for d in det if d.get("poise_g") is not None]
    ev = dict(
        final_pointer_mm=end.get("pointer_mm"),
        final_speed_mm_s=end.get("speed_mm_s"),
        final_notches=end.get("notches"),
        final_seated=end.get("seated"),
        final_poise_g=end.get("poise_g"),
        final_reading_g=end.get("reading_g"),
        final_err_g=end.get("err_g"),
        final_contact=end.get("contact"),
        final_rest_s=end.get("rest_s"),
        final_clear_s=end.get("clear_s"),
        jar_on_pan=end.get("on_pan"),
        poise_moved_g=(max(poise) - min(poise)) if poise else 0.0,
        riders_mm=end.get("riders_mm"),
    )
    touching = bool(end.get("contact")) and any(end["contact"].values())
    if stage == "demo":
        done = bool(end.get("on_pan")) and end.get("reading_g") is not None and abs(end["err_g"]) <= 0.2
    elif stage == "idle":
        done = ev["poise_moved_g"] < 1e-3 and not touching
    elif stage == "neg-coarse-only":
        # the notched riders were moved, the poise never was, and the beam is not on zero
        done = (ev["poise_moved_g"] < 0.01 and sum((end.get("notches") or {}).values()) > 0
                and abs(end.get("pointer_mm") or 0.0) > 0.9)
    elif stage == "neg-unseated":
        # the 100 g rider is off its notch bottom (not seated) while the beam rests near zero, hands off
        sd = end.get("seated") or {}
        done = (sd.get("r100") is False and abs(end.get("pointer_mm") or 99.0) <= 0.9 and not touching)
    elif stage == "neg-hand-on":
        # a finger is on the poise at the end of the trial
        done = bool((end.get("contact") or {}).get("r10"))
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
