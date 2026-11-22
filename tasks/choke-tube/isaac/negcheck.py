"""Stimulus + reaction check of finished demo / negative runs (reads JSON only).

    python negcheck.py <out.json> <run_dir> [<run_dir> ...]

For each trial directory (a run dir, or each trial*/ inside a multi-trial run) it reads result.json and trace.jsonl,
checks that the stimulus the stage is meant to produce actually happened (from the measured toy and tube poses and
the contact monitor), and that the final score is the expected level.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

EXPECT = {
    "demo": 1.0,
    "idle": 0.0,
    "replay": 0.0,
    "neg-untested": 0.0,
    "neg-ignore-test": 0.0,
    "neg-hand-stays": 0.5,
}


def load_trace(d):
    return [json.loads(line) for line in open(d / "trace.jsonl")]


def stimulus(stage, rows, res):
    """(done, evidence) from the measured trace, independent of the score."""
    end = rows[-1]["detail"] if rows else {}
    veto = end.get("veto", {}) or {}
    toys = end.get("toys", {}) or {}
    tests = end.get("tests", {}) or {}
    looks = res.get("looks", [])
    tested = [x for x in looks if x.get("at") == "tested"]
    on_mat = {n: d for n, d in toys.items() if d.get("where") == "mat"}
    fits_on_mat = [n for n, d in on_mat.items() if d.get("hazard")]
    if stage == "demo":
        fell = [x["toy"] for x in tested if x.get("result") == "inside"]
        return res["success"] and not any(veto.values()), dict(
            tested=[(x["toy"], x.get("result")) for x in tested], tipped_out=fell, on_mat=list(on_mat), tests=tests,
            toy_touch_peak_n=end.get("toy_touch_peak_n"), tube_robot_peak_n=end.get("tube_robot_peak_n"),
        )
    if stage in ("idle", "replay"):
        return True, dict(final_toys={n: d.get("where") for n, d in toys.items()}, veto=veto)
    if stage == "neg-untested":
        # a toy that fits ends on her placemat, never tested in the tube
        return bool(fits_on_mat) and not any(tests.get(n, {}).get("inside") for n in fits_on_mat) and veto.get(
            "hazard"
        ), dict(fits_on_mat=fits_on_mat, exposure=end.get("exposure"), tests=tests, veto=veto)
    if stage == "neg-ignore-test":
        # the toy tested dropped fully into the tube (fits) and still ended in her reach
        first = tested[0] if tested else {}
        exp = end.get("exposure") or []
        return first.get("result") == "inside" and any(e["toy"] == first.get("toy") for e in exp), dict(
            tested=first, exposure=exp, veto=veto
        )
    if stage == "neg-hand-stays":
        safe_on_mat = [n for n, d in on_mat.items() if not d.get("hazard")]
        return bool(safe_on_mat) and not end.get("hands_out"), dict(
            safe_on_mat=safe_on_mat, hands_out=end.get("hands_out"), min_tip_m=end.get("min_tip_to_shoulder_m")
        )
    return False, {}


def check(d):
    res = json.loads((d / "result.json").read_text())
    stage = res["stage"]
    rows = load_trace(d)
    done, ev = stimulus(stage, rows, res)
    want = EXPECT.get(stage)
    ok = (
        bool(done)
        and want is not None
        and abs(res["task_score"] - want) < 1e-9
        and not res.get("nonfinite")
    )
    return dict(
        dir=str(d),
        stage=stage,
        task_score=res["task_score"],
        expected=want,
        stimulus_done=bool(done),
        evidence=ev,
        steps=res["steps"],
        stop=res.get("stop"),
        nonfinite=res.get("nonfinite"),
        passed=ok,
    )


def main(out, dirs):
    rows = []
    for x in dirs:
        p = Path(x)
        trials = sorted(p.glob("trial*_*")) if not (p / "result.json").exists() else [p]
        rows += [check(t) for t in trials]
    summary = dict(passed=all(r["passed"] for r in rows), n=len(rows), rows=rows)
    Path(out).write_text(json.dumps(summary, indent=1, default=float))
    for r in rows:
        print(
            ("PASS" if r["passed"] else "FAIL"),
            r["stage"],
            r["task_score"],
            r["expected"],
            r["evidence"],
        )
    return summary["passed"]


if __name__ == "__main__":
    sys.exit(0 if main(sys.argv[1], sys.argv[2:]) else 1)
