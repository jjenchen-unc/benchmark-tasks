"""Stimulus + reaction check of finished demo / negative / replay runs (reads JSON only).

    python negcheck.py <out.json> <run_dir> [<run_dir> ...]

For each run directory it reads result.json and trace.jsonl, checks that the stimulus the stage is meant to produce
actually happened (from the measured cable / tab state in the trace), and that the final score is the expected level.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

EXPECT = {"demo": 1.0, "idle": 0.0, "replay": 0.0, "neg-blind-aim": 0.0, "neg-push-far": 0.5, "neg-overshoot": 0.5}


def load_trace(d):
    return [json.loads(line) for line in open(d / "trace.jsonl")]


def m_of(r):
    return r["detail"].get("measure", {}) or {}


def stimulus(stage, rows, res):
    """(done, evidence) from the measured trace, independent of the score."""
    end = rows[-1]["detail"] if rows else {}
    m = end.get("measure", {}) or {}
    ins = [m_of(r).get("inside_mm") for r in rows]
    ins_v = [v for v in ins if v is not None]
    base = dict(
        final_inside_mm=m.get("inside_mm"),
        max_inside_mm=max(ins_v) if ins_v else None,
        through_final=m.get("through"),
        tab_peak_deg=end.get("tab_peak_deg"),
        tab_force_peak_n=end.get("tab_force_peak_n"),
        bite_peak_n=end.get("bite_peak_n"),
        plastic_peak_deg=end.get("plastic_peak_deg"),
        grip_peak_n=end.get("grip_peak_n"),
        robot_box_peak_n=end.get("robot_box_peak_n"),
        final_tip_mm=m.get("tip_mm"),
    )
    if stage == "demo":
        return bool(res["success"]), base
    if stage in ("idle", "replay"):
        return True, base
    if stage == "neg-blind-aim":
        # the cable was pushed against the box wall (cable-box contact) and never got into the opening / under the tab
        cb = max((r["detail"].get("cable_box_n") or [0, 0])[0] for r in rows)
        base["cable_box_peak_n"] = cb
        return cb >= 2.0 and (end.get("tab_peak_deg") or 55.0) < 56.0 and not m.get("through"), base
    if stage == "neg-push-far":
        # one long push: through the clamp, released and still, length outside the 130-170 mm window
        v = m.get("inside_mm")
        base["length_outside_window"] = v is not None and not (130.0 <= v <= 170.0)
        return bool(m.get("through")) and base["length_outside_window"], base
    if stage == "neg-overshoot":
        # through, length outside the window, and a 40 mm pull back against the tab's bite left the length unchanged
        looks = {x["at"]: x for x in res.get("looks", [])}
        a, b = looks.get("pull back start", {}), looks.get("pull back end", {})
        base["pull_back_mm"] = [a.get("inserted_mm"), b.get("inserted_mm")]
        v = m.get("inside_mm")
        held = a.get("inserted_mm") is not None and b.get("inserted_mm") is not None and (b["inserted_mm"] >= a["inserted_mm"] - 2.0)
        ok = held and (end.get("bite_peak_n") or 0) > 3 and v is not None and not (130.0 <= v <= 170.0)
        return ok and bool(m.get("through")), base
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
    rows = [check(Path(x)) for x in dirs]
    summary = dict(passed=all(r["passed"] for r in rows), n=len(rows), rows=rows)
    Path(out).write_text(json.dumps(summary, indent=1, default=float))
    for r in rows:
        print(("PASS" if r["passed"] else "FAIL"), r["stage"], r["task_score"], r["expected"], r["evidence"])
    return summary["passed"]


if __name__ == "__main__":
    sys.exit(0 if main(sys.argv[1], sys.argv[2:]) else 1)
