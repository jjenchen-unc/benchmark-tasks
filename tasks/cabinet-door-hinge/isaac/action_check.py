"""Executed-vs-commanded action check of an official run (reads JSON only).

    python action_check.py <official log dir> [--out action_check.json]

Compares the actions the scripted policy commanded (policy_log.json, written by official_run.py) with the actions the
Inspect Robots CLI executed after its guardrails (actions/<run>/scene-0-e0.jsonl: URDF-bounds clamp and the per-step
delta limit of 0.05 x joint range), over the steps both cover. Also re-applies both guards to the commanded actions.
Target: 0 changed actions.
"""

import glob
import json
import sys
from pathlib import Path

import numpy as np


def main():
    d = Path(sys.argv[1])
    out = sys.argv[sys.argv.index("--out") + 1] if "--out" in sys.argv else None
    cmd = np.asarray(json.load(open(d / "policy_log.json"))["actions"], float)
    rows = [json.loads(x) for x in open(sorted(glob.glob(str(d / "actions" / "*" / "scene-0-e0.jsonl")))[0])]
    exe = np.asarray([r["action"] for r in rows if "action" in r], float)
    n = min(len(cmd), len(exe))
    diff = np.abs(exe[:n] - cmd[:n])
    changed = int(np.sum(diff.max(axis=1) > 1e-5))
    try:
        from yam_arena import ik

        lo1, hi1 = np.append(ik.LOWER, 0.0), np.append(ik.UPPER, 1.0)
        lo, hi = np.r_[lo1, lo1], np.r_[hi1, hi1]
        dmax = 0.05 * (hi - lo)
        prev, guard = None, 0
        for a in cmd:
            e = np.clip(a, lo, hi)
            if prev is not None:
                e = np.clip(e, prev - dmax, prev + dmax)
            guard += int(np.max(np.abs(e - a)) > 1e-9)
            prev = e
    except ImportError:
        guard = None
    res = dict(
        commanded=len(cmd),
        executed=len(exe),
        compared=n,
        changed_actions=changed,
        max_abs_diff=float(diff.max()) if n else None,
        guard_would_change=guard,
    )
    print(json.dumps(res))
    if out:
        Path(out).write_text(json.dumps(res, indent=1) + "\n")


if __name__ == "__main__":
    main()
