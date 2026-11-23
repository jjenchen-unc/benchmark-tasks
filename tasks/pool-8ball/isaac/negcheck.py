"""Stimulus + reaction check of finished demo / negative / replay runs (reads JSON only).

    python negcheck.py <out.json> <run_dir> [<run_dir> ...]

For each trial directory (a run dir, or each trial*/ inside a multi-trial run) it reads result.json and trace.jsonl,
checks that the stimulus the stage is meant to produce actually happened (from the demo's plan / address notes, the
contact monitor's stroke count and the measured cue-ball speed), and that the final score is the expected level.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

EXPECT_MAX = {  # highest score the stage may reach
    "demo": 1.0,
    "idle": 0.0,
    "replay": 0.0,
    "neg-misaim": 0.5,
    "neg-soft": 0.5,
    "neg-full-ball": 0.5,
    "neg-double": 0.0,
    "neg-shaft": 0.0,
}
EXPECT_MIN = {"demo": 1.0}


def look(res, tag):
    return next((x for x in res.get("looks", []) if x.get("at") == tag), None)


def stimulus(stage, res, rows):
    facts = (res.get("final") or {}).get("facts") or {}
    plan = look(res, "plan") or {}
    addr = look(res, "addressed") or {}
    st = look(res, "stroke start") or {}
    cbv = (res.get("final") or {}).get("cb_speed_after")
    ev = dict(strokes=facts.get("strokes"), hit8=facts.get("hit8"), fouls=facts.get("fouls"),
              eight_pocket=facts.get("eight_pocket"), cue_on_table=facts.get("cue_on_table"),
              planned_pocket=plan.get("pocket"), aim=addr.get("err"), v_cmd=st.get("v_cmd"), cb_speed_after=cbv)
    if stage == "demo":
        return facts.get("strokes") == 1 and facts.get("eight_pocket") == plan.get("pocket"), ev
    if stage in ("idle", "replay"):
        return True, ev
    if stage == "neg-misaim":
        # the demo addressed the cue on a line rotated 3 deg off the ghost-ball line (aim error measured vs that line
        # is small) and struck once
        return facts.get("strokes") == 1 and addr.get("err") is not None, ev
    if stage == "neg-soft":
        return facts.get("strokes") == 1 and cbv is not None and plan.get("v_cueball") is not None and \
            cbv < 0.6 * plan["v_cueball"], ev
    if stage == "neg-full-ball":
        return facts.get("strokes") == 1, ev
    if stage == "neg-shaft":
        return facts.get("strokes") == 0 and "shaft or butt touched cue ball" in (facts.get("fouls") or []), ev
    if stage == "neg-double":
        first = look(res, "tap 1") or {}
        clear = look(res, "clear after tap 1") or {}
        second = look(res, "tap 2") or {}
        first_count = (first.get("facts") or {}).get("strokes")
        clear_count = (clear.get("facts") or {}).get("strokes")
        second_count = (second.get("facts") or {}).get("strokes") or 0
        gap = clear.get("tip_gap_mm")
        ev.update(first_strokes=first_count, strokes_after_retraction=clear_count,
                  first_gap_mm=gap, second_strokes=second_count)
        # The point-to-ball-surface distance must clear the entire 12 mm tip collider
        # (radius about 6.5 mm), including off-axis contact at its rim.
        return (first_count == clear_count == 1 and gap is not None and gap > 15.0
                and second_count >= 2 and "second stroke" in (facts.get("fouls") or [])), ev
    return False, ev


def check(d):
    res = json.loads((d / "result.json").read_text())
    rows = [json.loads(line) for line in open(d / "trace.jsonl")] if (d / "trace.jsonl").exists() else []
    stage = res["stage"]
    done, ev = stimulus(stage, res, rows)
    score = res["task_score"]
    ok_score = score <= EXPECT_MAX.get(stage, 1.0) + 1e-9 and score >= EXPECT_MIN.get(stage, 0.0) - 1e-9
    return dict(run=str(d), stage=stage, setup=res.get("setup"), score=score, steps=res.get("steps"),
                stimulus_happened=bool(done), score_as_expected=bool(ok_score), evidence=ev)


def main():
    out = Path(sys.argv[1])
    rows = []
    for a in sys.argv[2:]:
        d = Path(a)
        trials = sorted(d.glob("trial*/")) or [d]
        for t in trials:
            if (t / "result.json").exists():
                rows.append(check(t))
    out.write_text(json.dumps(rows, indent=2, default=str))
    for r in rows:
        print(r["stage"], r["score"], "stimulus" if r["stimulus_happened"] else "NO-STIMULUS",
              "ok" if r["score_as_expected"] else "UNEXPECTED", r["run"])


if __name__ == "__main__":
    main()
