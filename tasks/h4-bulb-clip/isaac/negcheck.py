"""Stimulus + reaction check of finished demo / negative runs (reads JSON only).

    python negcheck.py <out.json> <run_dir> [<run_dir> ...]

For each trial directory (a run dir, or each trial*/ inside a multi-trial run) it reads result.json and trace.jsonl,
checks that the stimulus the stage is meant to produce actually happened (from the measured bulb pose, the clip angle,
the catch position and the contact monitor), and that the final score is the expected level.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

EXPECT = {
    "demo": (1.0,),
    "idle": (0.0,),
    "hold": (0.0,),
    "neg-noroll": (0.0,),
    "neg-unhooked": (0.5,),
    "neg-axial-press": (0.25, 0.5),
    "neg-noclip": (0.25, 0.0),
    "neg-glass": (0.0,),
    "replay": (0.0,),
}


def load_trace(d):
    return [json.loads(line) for line in open(d / "trace.jsonl")]


def m_of(r):
    return r["detail"].get("measure", {}) or {}


def axial_press_stimulus(rows, final):
    """Confirm an x-only commanded stroke and an increase in the net +x force on the clip."""
    pressed = [r for r in rows if "axial press only" in r.get("labels", [])]
    if not pressed:
        return False, dict(reason="no axial press samples", final=final)
    first = pressed[0]["step"]
    before = next((r for r in reversed(rows) if r["step"] < first), None)
    if before is None:
        return False, dict(reason="no pre-press baseline", final=final)
    sample = [before] + pressed
    target = [(r.get("tcp_cmd_mm") or {}).get("left") for r in sample]
    force = [(r["detail"].get("contacts") or {}).get("robot_on_clip_force_n") for r in sample]
    if any(v is None or len(v) != 3 for v in target + force):
        return False, dict(reason="missing commanded TCP or directional force", final=final)
    origin = target[0]
    advance = target[-1][0] - origin[0]
    lateral_cmd = max(math.hypot(p[1]-origin[1], p[2]-origin[2]) for p in target)
    peak_side = max(abs(m_of(r).get("catch_mm", 0)) for r in pressed)
    loaded = [f[0] for r, f in zip(pressed, force[1:]) if m_of(r).get("seated") and m_of(r).get("over")]
    peak_axial = max(loaded, default=0.)
    latched = any(m_of(r).get("latched") for r in rows)
    ok = (advance > 1.0 and lateral_cmd < 0.15 and peak_axial > max(0.05, force[0][0] + 0.05)
          and peak_side < 2.0 and final.get("seated") and not latched)
    return bool(ok), dict(commanded_axial_mm=advance, max_off_axis_command_mm=lateral_cmd,
                          before_axial_n=force[0][0], peak_axial_n=peak_axial,
                          peak_lateral_mm=peak_side, final=final)


def stimulus(stage, rows, res):
    """(done, evidence) from the measured trace, independent of the score."""
    end = rows[-1]["detail"] if rows else {}
    m_end = end.get("measure", {}) or {}
    veto = end.get("veto", {}) or {}
    peak = end.get("peak", {}) or {}
    ever = lambda f: any(f(m_of(r)) for r in rows)  # noqa: E731
    if stage == "demo":
        shifted = any(m_of(r).get("catch_mm", 0) <= -1.9 and m_of(r).get("over") for r in rows)
        final_axial = (end.get("contacts") or {}).get("flange_axial_n", 0)
        verified = res["success"] and shifted and 10 <= final_axial <= 60 and not any(veto.values())
        return verified, dict(final=m_end, shifted=shifted, final_flange_axial_n=final_axial, peak=peak)
    if stage in ("idle", "hold"):
        return not ever(lambda m: m.get("tip_bulb_mm", 999) < 30), dict(final=m_end)
    if stage == "neg-noroll":
        tries = [x for x in res.get("looks", []) if x.get("at") == "seat try"]
        roll_off = [abs(x["err"][1]) for x in tries]
        return bool(tries) and min(roll_off) > 2.0 and not ever(lambda m: m.get("seated")), dict(
            roll_at_seat_try_deg=roll_off, axial_at_seat_try_mm=[x["err"][3] * 1e3 for x in tries], final=m_end
        )
    if stage == "neg-axial-press":
        return axial_press_stimulus(rows, m_end)
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
    return 0 if report and all(row["ok"] for row in report) else 1


if __name__ == "__main__":
    sys.exit(main())
