"""Stimulus + reaction check of finished demo / negative runs (reads JSON only).

    python negcheck.py <out.json> <run_dir> [<run_dir> ...]

For each trial directory (a run dir, or each trial*/ inside a multi-trial run) it reads result.json and trace.jsonl,
checks that the stimulus the stage is meant to produce actually happened (from the measured joint state, the burner
model and the contact monitor, independent of the score), and that the final score is the expected level.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

EXPECT = {
    "demo": 1.0,
    "idle": 0.0,
    "neg-lid-closed": 0.0,
    "neg-slow-spark": 0.0,
    "neg-no-push": 0.0,
    "replay": 0.0,
}


def load_trace(d):
    return [json.loads(line) for line in open(d / "trace.jsonl")]


def stimulus(stage, rows, res):
    """(done, evidence) from the measured trace, independent of the score."""
    det = [r["detail"] for r in rows if r.get("detail")]
    end = det[-1] if det else {}
    peaks = end.get("peaks", {})
    gas_rows = [d for d in det if d.get("gas")]
    lid_at_gas = gas_rows[0]["lid_deg"] if gas_rows else None
    ev = dict(lid_at_gas=lid_at_gas, max_knob_deg=max((d.get("knob_deg", 0) for d in det), default=0),
              max_lid_deg=peaks.get("lid_deg"), max_unlit_s=end.get("max_unlit_s"),
              igniter_peak_mm=peaks.get("igniter_mm"), push_peak_mm=peaks.get("push_mm"),
              gate_peak_nm=peaks.get("gate_nm"), contact_by_link=end.get("contact_by_link"),
              lit_ever=any(d.get("lit") for d in det), veto=end.get("veto"), veto_t=end.get("veto_t"))
    if stage == "demo":
        return bool(res["success"] and lid_at_gas is not None and lid_at_gas >= 70 and not end.get("veto")), ev
    if stage == "idle":
        return bool(peaks.get("lid_deg", 0) < 1 and ev["max_knob_deg"] < 1), ev
    if stage == "neg-lid-closed":
        # gas came on with the lid down, the igniter was pressed past its switch point
        return bool(lid_at_gas is not None and lid_at_gas < 10 and (peaks.get("igniter_mm") or 0) >= 2.0), ev
    if stage == "neg-slow-spark":
        # lid open at gas-on, gas flowed unlit for more than 5 s, the igniter pressed only after that
        return bool(lid_at_gas is not None and lid_at_gas >= 70 and (end.get("max_unlit_s") or 0) > 5.0
                    and (peaks.get("igniter_mm") or 0) >= 2.0), ev
    if stage == "neg-no-push":
        # the hand turned the knob (torque on the OFF stop, contact on the knob) without pushing it past 3 mm
        knob_touch = (end.get("contact_by_link") or {}).get("knob", 0) > 0.5
        return bool(knob_touch and (peaks.get("push_mm") or 0) < 3.0 and (peaks.get("gate_nm") or 0) > 0.5
                    and ev["max_knob_deg"] < 15.0), ev
    if stage == "replay":
        return True, ev
    return False, ev


def trials(d):
    d = Path(d)
    if (d / "result.json").exists():
        return [d]
    return sorted(p for p in d.glob("trial*") if (p / "result.json").exists())


def main():
    out = Path(sys.argv[1])
    rows = []
    for d in sys.argv[2:]:
        for t in trials(d):
            res = json.loads((t / "result.json").read_text())
            stage = res["stage"]
            done, ev = stimulus(stage, load_trace(t), res)
            want = EXPECT.get(stage)
            rows.append(dict(run=str(t), stage=stage, score=res["task_score"], expected=want,
                             score_ok=want is not None and abs(res["task_score"] - want) < 1e-9,
                             stimulus_ok=done, steps=res["steps"], stop=res.get("stop"), evidence=ev))
            print(json.dumps({k: rows[-1][k] for k in ("run", "stage", "score", "score_ok", "stimulus_ok")}))
    out.write_text(json.dumps(dict(passed=all(r["score_ok"] and r["stimulus_ok"] for r in rows), rows=rows),
                              indent=2, default=float))


if __name__ == "__main__":
    main()
