"""Stimulus + reaction check of finished demo / negative runs (reads JSON only).

    python negcheck.py <out.json> <run_dir> [<run_dir> ...]

For each trial directory (a run dir, or each trial*/ inside a multi-trial run) it reads result.json and trace.jsonl,
checks that the stimulus the stage is meant to produce actually happened (from the measured lever angle, clip, paste
and processor states, the cooler's lift and the contact monitor, independent of the score), and that the final score
is the expected level.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

EXPECT = {"demo": 1.0, "idle": 0.0, "neg-straight-pull": 0.0, "neg-no-lever": 0.0, "neg-no-hooks": 0.0, "neg-short-twist": 0.0,
          "replay": 0.0}


def stimulus(stage, det, res):
    """(done, evidence) from the measured trace, independent of the score."""
    end = det[-1] if det else {}
    ev = dict(
        max_lever_deg=max((d.get("lever_deg") or 0.0 for d in det), default=0.0),
        clip_slack=any(d.get("clip") == "slack" for d in det),
        released_hooks=sorted({hook for d in det for hook in d.get("hooks_released", [])}),
        max_abs_twist_deg=max((abs(d.get("twist_deg") or 0.0) for d in det), default=0.0),
        max_slip_deg=max((d.get("slip_deg") or 0.0 for d in det), default=0.0),
        sheared=any(d.get("bond") in ("sheared", "free") for d in det),
        parted=any(d.get("bond") == "free" for d in det),
        max_bond_z_mm=max((d.get("bond_z_mm") or 0.0 for d in det), default=0.0),
        max_cpu_lift_mm=max((d.get("cpu_lift_mm") or 0.0 for d in det), default=0.0),
        cpu_yanked=any(d.get("cpu") == "yanked" for d in det),
        cooler_touched=any((d.get("contact") or {}).get("cooler") for d in det),
        peak_cooler_contact_n=max(((d.get("contact_peak_n") or {}).get("cooler", 0.0) for d in det), default=0.0),
        end_on_mat=end.get("cooler_on_mat"),
        veto=end.get("veto"),
        events=end.get("events"),
    )
    if stage == "demo":
        done = (ev["clip_slack"] and ev["released_hooks"] == ["far_hook", "near_hook"] and ev["sheared"]
                and ev["parted"] and not ev["cpu_yanked"] and ev["end_on_mat"])
    elif stage == "neg-straight-pull":  # clip released, pulled with the paste intact: processor comes out
        done = ev["clip_slack"] and ev["cpu_yanked"] and ev["max_slip_deg"] < 6.0 and ev["max_cpu_lift_mm"] > 5.0
    elif stage == "neg-no-hooks":
        looks = {row["at"] for row in res.get("looks", [])}
        done = (ev["clip_slack"] and "hooks_released" in end and not ev["released_hooks"]
                and {"shroud pinched (left)", "shroud pinched (right)"} <= looks
                and ev["cooler_touched"] and not ev["parted"] and res.get("stop") is None)
    elif stage == "neg-no-lever":  # clip never released, twist and pull tried on the cooler: nothing moves
        done = (not ev["clip_slack"]) and ev["cooler_touched"] and ev["max_bond_z_mm"] < 1.0 and ev["max_abs_twist_deg"] < 1.0
    elif stage == "neg-short-twist":  # clip released, token twist below the shear slip, then lifted: processor out
        done = ev["clip_slack"] and (not ev["sheared"]) and ev["cpu_yanked"] and ev["max_abs_twist_deg"] > 0.3
    elif stage == "idle":
        done = not ev["cooler_touched"]
    elif stage == "replay":
        done = True
    else:
        raise ValueError(f"Unknown control stage {stage!r}")
    return bool(done), ev


def trials(d):
    d = Path(d)
    if (d / "result.json").exists():
        return [d]
    return sorted(p for p in d.glob("trial*") if (p / "result.json").exists())


def main():
    out = Path(sys.argv[1])
    rows, ok = [], True
    for d in sys.argv[2:]:
        for t in trials(d):
            res = json.loads((t / "result.json").read_text())
            det = [json.loads(line).get("detail") or {} for line in open(t / "trace.jsonl")]
            stage = res["stage"]
            done, ev = stimulus(stage, det, res)
            exp = EXPECT.get(stage, 0.0)
            good = done and abs(res["task_score"] - exp) < 1e-9 if stage != "replay" else res["task_score"] == 0.0
            ok &= good
            rows.append(dict(trial=str(t), stage=stage, setup=res.get("setup"), score=res["task_score"],
                             expected=exp, stimulus=done, passed=good, steps=res["steps"], grip_peak_n=res.get("grip_peak_n"),
                             evidence=ev))
            print(f"{'PASS' if good else 'FAIL'} {t} {stage} score {res['task_score']} stimulus {done} "
                  f"lever {ev['max_lever_deg']:.0f} slip {ev['max_slip_deg']:.1f} cpu {ev['max_cpu_lift_mm']:.1f} "
                  f"yanked {ev['cpu_yanked']} parted {ev['parted']}")
    out.write_text(json.dumps(dict(passed=ok, trials=rows), indent=1, default=float))
    print("NEGCHECK", "passed" if ok else "FAILED", len(rows))


if __name__ == "__main__":
    main()
