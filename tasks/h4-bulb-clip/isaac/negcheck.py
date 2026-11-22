"""Stimulus + reaction check of finished demo / negative runs (reads JSON only).

    python negcheck.py <out.json> <run_dir> [<run_dir> ...]

For each trial directory (a run dir, or each trial*/ inside a multi-trial run) it reads result.json and trace.jsonl,
checks that the stimulus the stage is meant to produce actually happened (from the measured bulb pose, the clip angle,
the catch position and the contact monitor), and that the final score is the expected level.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

EXPECT = {
    "demo": (1.0,),
    "idle": (0.0,),
    "hold": (0.0,),
    "neg-noroll": (0.0,),
    "neg-unhooked": (0.5,),
    "neg-noclip": (0.25, 0.0),
    "neg-glass": (0.0,),
    "replay": (0.0,),
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
    peak = end.get("peak", {}) or {}
    ever = lambda f: any(f(m_of(r)) for r in rows)  # noqa: E731
    if stage == "demo":
        return res["success"] and not any(veto.values()), dict(final=m_end, peak=peak, kit=end.get("kit_peak"))
    if stage in ("idle", "hold"):
        return not ever(lambda m: m.get("tip_bulb_mm", 999) < 30), dict(final=m_end)
    if stage == "neg-noroll":
        tries = [x for x in res.get("looks", []) if x.get("at") == "seat try"]
        roll_off = [abs(x["err"][1]) for x in tries]
        return bool(tries) and min(roll_off) > 2.0 and not ever(lambda m: m.get("seated")), dict(
            roll_at_seat_try_deg=roll_off, axial_at_seat_try_mm=[x["err"][3] * 1e3 for x in tries], final=m_end
        )
    if stage == "neg-unhooked":
        return bool(m_end.get("over")) and not m_end.get("latched") and bool(m_end.get("seated")), dict(final=m_end)
    if stage == "neg-noclip":
        return m_end.get("clip_deg", 0) > 30 and ever(lambda m: m.get("seated")), dict(final=m_end)
    if stage == "neg-glass":
        return bool(veto.get("glass")) and peak.get("glass_n", 0) > 0.05, dict(peak=peak, veto=veto)
    if stage == "replay":
        return True, dict(final=m_end, start=res.get("setup"))
    return False, {}


def main():
    out = Path(sys.argv[1])
    report = []
    for arg in sys.argv[2:]:
        d = Path(arg)
        trials = sorted(d.glob("trial*")) or [d]
        for t in trials:
            res = json.loads((t / "result.json").read_text())
            rows = load_trace(t)
            stage = res["stage"]
            done, ev = stimulus(stage, rows, res)
            exp = EXPECT.get(stage)
            score = res["task_score"]
            ok = done and exp is not None and any(abs(score - e) < 1e-9 for e in exp)
            report.append(dict(run=str(t), stage=stage, score=score, expected=exp, stimulus=done, ok=ok,
                               steps=res["steps"], evidence=ev))
            print(("OK  " if ok else "FAIL"), t, stage, score, exp, "stimulus" if done else "NO STIMULUS", flush=True)
    out.write_text(json.dumps(report, indent=1, default=float))


if __name__ == "__main__":
    main()
