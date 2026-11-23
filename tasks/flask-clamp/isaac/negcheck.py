"""Stimulus + reaction check of finished demo / negative / replay runs (reads JSON only).

    python negcheck.py <out.json> <run_dir> [<run_dir> ...]

For each trial directory (a run dir, or each trial*/ inside a multi-trial run) it reads result.json and trace.jsonl,
checks that the stimulus the stage is meant to produce actually happened (from the measured flask pose relative to
the free clamp, the finger angles and contacts, the fingertip clearance and the shake test), and that the final score
is the expected level.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

EXPECT = {"demo": 1.0, "idle": 0.0, "replay": 0.0, "neg-perched": 0.0, "neg-held": 0.5}


def load_trace(d):
    return [json.loads(line) for line in open(d / "trace.jsonl")]


def meas(r):
    return (r.get("detail") or {}).get("measure") or {}


def stimulus(stage, rows, res):
    """(done, evidence) from the measured trace, independent of the score."""
    end = rows[-1]["detail"] if rows else {}
    m = end.get("measure", {}) or {}
    rel = m.get("rel", {}) or {}
    ever_seated = any((r.get("detail") or {}).get("seated") for r in rows)
    min_dz = min((meas(r).get("rel", {}).get("dz", 1e9) for r in rows if meas(r)), default=None)
    base = dict(final_rel_mm=rel, final_theta_deg=m.get("theta_deg"), final_finger_n=m.get("finger_n"),
                final_tip_mm=m.get("tip_mm"), ever_seated=ever_seated, min_bottom_dz_mm=min_dz,
                shakes_started=end.get("shakes_started"), shakes_failed=end.get("shakes_failed"),
                grip_peak_n=end.get("grip_peak_n"), press_peak_n=end.get("press_peak_n"),
                loaded_peak_n=end.get("loaded_peak_n"))
    if stage == "demo":
        return bool(res["success"]) and (end.get("shakes_started") or 0) >= 1, base
    if stage == "idle":
        return not ever_seated, base
    if stage == "replay":
        return True, base
    if stage == "neg-perched":
        # released on the flared tips: flask upright and centred over the clamp, bottom well above the plate (the
        # heel rests on the flared tips, fingers slightly spread), hands clear, never seated
        th = m.get("theta_deg") or [0]
        ok = (not ever_seated and rel.get("radial", 99) < 6.0 and rel.get("dz", 0) > 20.0
              and rel.get("tilt_deg", 99) < 5.0 and min(th) > 1.0 and (m.get("tip_mm") or 0) >= 40.0)
        return ok, base
    if stage == "neg-held":
        # seated, a fingertip within 40 mm of the flask to the end, the shake test never started
        ok = bool(end.get("seated")) and (m.get("tip_mm") or 99) < 40.0 and not end.get("shakes_started")
        return ok, base
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
