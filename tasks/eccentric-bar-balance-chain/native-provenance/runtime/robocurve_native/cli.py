from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def parser():
    ap = argparse.ArgumentParser(description="Source-native MuJoCo robot task validation (no model calls)")
    ap.add_argument("command", choices=["list", "build", "reset", "step", "reference", "render", "compile-all"])
    ap.add_argument("--module", type=Path)
    ap.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2] / "bench28")
    ap.add_argument("--target", help="Source target id; default TASK_A; 'exploration' for no scoring")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--instance", choices=["S", "T"], default="S")
    ap.add_argument("--setting", choices=["A", "S", "T-new", "T-combo"], help="Declared source setting; T-new applies transfer_parameters")
    ap.add_argument("--steps", type=int, default=1, help="Physics steps for step; no reference truncation")
    ap.add_argument("--action", help="JSON array of model.nu source-native controls; default zeros")
    ap.add_argument("--max-reference-steps", type=int, help="Explicit partial replay; cannot validate feasibility")
    ap.add_argument("--actions-out", type=Path, help="Save full source reference controls to .npy")
    ap.add_argument("--output", type=Path, help="JSON report; render writes binary PPM")
    ap.add_argument("--camera")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=360)
    ap.add_argument("--timeout", type=float, default=180, help="Per-task compile-all timeout in seconds")
    return ap


def main():
    args = parser().parse_args()
    # Set before importing MuJoCo: CPU-capable rendering, overridable per process.
    os.environ.setdefault("MUJOCO_GL", "osmesa")
    modules = sorted(p for p in args.root.glob("*/benches/*.py") if p.stem == p.parent.parent.name)
    if args.command == "list":
        print(json.dumps([dict(bench=p.stem, module=str(p.resolve())) for p in modules], indent=2))
        return
    if args.command == "compile-all":
        records = []
        for module in modules:
            cmd = [sys.executable, "-m", "robocurve_native", "step", "--module", str(module),
                   "--seed", str(args.seed), "--steps", str(args.steps)]
            start = time.monotonic()
            try:
                proc = subprocess.run(cmd, text=True, capture_output=True, timeout=args.timeout)
                try:
                    result = json.loads(proc.stdout)
                except ValueError:
                    result = {"stdout": proc.stdout[-5000:]}
                records.append(dict(bench=module.stem, passed=proc.returncode == 0, result=result,
                                    stderr=proc.stderr[-5000:], wall_seconds=time.monotonic() - start))
            except subprocess.TimeoutExpired:
                records.append(dict(bench=module.stem, passed=False, error="timeout"))
        report = dict(task_count=len(records), passed=sum(r["passed"] for r in records), results=records)
        text = json.dumps(report, indent=2, allow_nan=False)
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(text + "\n")
        print(text)
        raise SystemExit(0 if records and all(r["passed"] for r in records) else 1)
    if not args.module:
        raise SystemExit("--module is required")
    import numpy as np
    from .task import NativeTask, write_json
    task = NativeTask(args.module, seed=args.seed, target=args.target, instance=args.instance, setting=args.setting)
    try:
        result = dict(metadata=task.metadata(), status=task.status())
        if args.command == "reset":
            result["observation"] = task.observation()
        elif args.command == "step":
            action = np.asarray(json.loads(args.action)) if args.action else np.zeros(task.model.nu)
            result["observation"] = task.step(action, args.steps)
            result["status"] = task.status()
        elif args.command == "reference":
            start = time.monotonic()
            actions, result["status"] = task.reference(max_steps=args.max_reference_steps)
            result["wall_seconds"] = time.monotonic() - start
            if args.actions_out:
                args.actions_out.parent.mkdir(parents=True, exist_ok=True)
                np.save(args.actions_out, actions)
            print(write_json(result, args.output))
            raise SystemExit(0 if result["status"]["complete"] and result["status"]["valid_success"] else 2)
        elif args.command == "render":
            if not args.output:
                raise ValueError("render requires --output IMAGE.ppm")
            frame = task.render(width=args.width, height=args.height, camera=args.camera)
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_bytes(f"P6\n{args.width} {args.height}\n255\n".encode() + frame.tobytes())
            result["render"] = dict(path=str(args.output.resolve()), shape=frame.shape,
                                    min=int(frame.min()), max=int(frame.max()), mean=float(frame.mean()))
            print(write_json(result))
            return
        print(write_json(result, args.output))
    finally:
        task.close()


if __name__ == "__main__":
    main()
