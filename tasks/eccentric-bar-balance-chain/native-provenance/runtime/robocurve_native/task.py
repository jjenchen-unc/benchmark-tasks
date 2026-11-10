"""Preserve the delivered task's physical stepping and scoring contracts.

One task per process: delivered helpers have unqualified module names. Isolation
is intentional and prevents one task accidentally importing another task's robot.
"""
from __future__ import annotations

import hashlib
import importlib.util
import inspect
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

_LOADED_PATH: Path | None = None


def load_partial_scorer(module_path: Path):
    """Optional package-local evaluator plugin: <package>/scoring/partial_credit.py.

    Evaluator-only; its results go to status(), never to observation()."""
    for parent in module_path.parents:
        candidate = parent / "scoring" / "partial_credit.py"
        if not (parent / "candidate.json").is_file():
            continue                      # only the package root that owns this scene
        if candidate.is_file():
            spec = importlib.util.spec_from_file_location("_robocurve_partial_credit", candidate)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return module, candidate
        break
    return None, None


def load(path: str | Path):
    global _LOADED_PATH
    path = Path(path).resolve()
    if _LOADED_PATH is not None and _LOADED_PATH != path:
        raise RuntimeError("Use a fresh process for each task; helper names are task-local")
    _LOADED_PATH = path
    sys.path.insert(0, str(path.parent))
    spec = importlib.util.spec_from_file_location("_robocurve_native_task", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    for name in ("build", "init_state", "step", "observation", "success", "damage", "parameters"):
        if not callable(getattr(module, name, None)):
            raise TypeError(f"Task is missing required callable {name}")
    return module


def jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, (set, frozenset)):
        return sorted(value)
    raise TypeError(type(value).__name__)


def write_json(value, path=None):
    result = json.dumps(value, default=jsonable, indent=2, allow_nan=False)
    if path:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text(result + "\n")
    return result


class NativeTask:
    def __init__(self, module, *, seed=42, target=None, instance="S", setting=None):
        self.module_path = Path(module).resolve()
        self.bench = load(self.module_path)
        self.seed, self.instance = int(seed), instance
        self.target = target if target is not None else getattr(self.bench, "TASK_A", self.bench.TARGETS[0]["id"])
        if self.target == "exploration":
            self.target = None
        if self.target is not None and self.target not in [t["id"] for t in self.bench.TARGETS]:
            raise ValueError(f"Unknown target: {self.target}")
        if setting is not None:
            descriptor = next((t for t in self.bench.TARGETS if t["id"] == self.target), None)
            if not descriptor or setting not in descriptor["settings"]:
                raise ValueError(f"Target {self.target} does not declare setting {setting}")
            self.instance = instance = "T" if setting == "T-new" else "S"
        self.setting = setting
        self.params = self.bench.parameters(self.seed)
        if instance == "T":
            descriptor = next((t for t in self.bench.TARGETS if t["id"] == self.target), None)
            if not descriptor or "T-new" not in descriptor["settings"]:
                raise ValueError("T instance requires a target declaring T-new")
            self.params = self.bench.transfer_parameters(self.params, self.seed)
        self.renderer = None
        self.partial_module, self.partial_path = load_partial_scorer(self.module_path)
        self.reset()

    def reset(self):
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None
        self.model, self.data = self.bench.build(self.seed, self.target, self.params)
        self.state = self.bench.init_state(self.seed, self.target, self.params)
        self.steps, self.ever_success, self.damage_seen = 0, False, set()
        descriptor = next((t for t in self.bench.TARGETS if t["id"] == self.target), {})
        self.allowed_damage = set(descriptor.get("allowed_damage", []))
        self.partial = (self.partial_module.PrefixHoldScorer(self.bench, self.model, self.target)
                        if self.partial_module is not None else None)
        self._validate_actuators()
        self._check_state()
        return self.observation()

    def _validate_actuators(self):
        """Every action must drive robot joints, never task mechanism components."""
        self.actuators = []
        joint_types = {int(mujoco.mjtTrn.mjTRN_JOINT), int(mujoco.mjtTrn.mjTRN_JOINTINPARENT)}
        for a in range(self.model.nu):
            transmission = int(self.model.actuator_trntype[a])
            driven = int(self.model.actuator_trnid[a, 0])
            if transmission in joint_types:
                joints = [driven]
            elif transmission == int(mujoco.mjtTrn.mjTRN_TENDON):
                start, count = int(self.model.tendon_adr[driven]), int(self.model.tendon_num[driven])
                wraps = range(start, start + count)
                if any(int(self.model.wrap_type[k]) != int(mujoco.mjtWrap.mjWRAP_JOINT) for k in wraps):
                    raise ValueError(f"Actuator {a} uses a spatial tendon; manual robot-only audit required")
                joints = [int(self.model.wrap_objid[k]) for k in wraps]
            else:
                raise ValueError(f"Unsupported actuator {a}: manual robot-only audit required")
            names = [self.model.joint(j).name for j in joints]
            if not names or any(not name or not name.startswith(("left_", "right_")) for name in names):
                raise ValueError(f"Actuator {a} drives non-robot joints {names!r}")
            self.actuators.append(dict(index=a, actuator=self.model.actuator(a).name, joints=names,
                                       ctrl_limited=bool(self.model.actuator_ctrllimited[a]),
                                       ctrl_range=self.model.actuator_ctrlrange[a].tolist()))

    def _check_state(self):
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
            raise FloatingPointError("Nonfinite simulator state")
        warnings = {mujoco.mjtWarning(i).name: int(w.number) for i, w in enumerate(self.data.warning) if w.number}
        if warnings:
            raise RuntimeError(f"MuJoCo warnings: {warnings}")

    def observation(self):
        # Only the task's public robot/tactile observations; never params/state/model.
        return self.bench.observation(self.model, self.data, self.state)

    def step(self, action, count=1, *, validate_range=True):
        action = np.asarray(action, dtype=float)
        if action.shape != (self.model.nu,) or not np.isfinite(action).all():
            raise ValueError(f"Expected finite action shape ({self.model.nu},), got {action.shape}")
        if count < 1:
            raise ValueError("Physics step count must be positive")
        limited = self.model.actuator_ctrllimited.astype(bool)
        if validate_range and (np.any(action[limited] < self.model.actuator_ctrlrange[limited, 0] - 1e-9) or np.any(
                action[limited] > self.model.actuator_ctrlrange[limited, 1] + 1e-9)):
            raise ValueError("Action exceeds source actuator control range; refusing silent clipping")
        for _ in range(count):
            self.data.ctrl[:] = action
            self.bench.step(self.model, self.data, self.state)
            self.steps += 1
            self._check_state()
            self.ever_success |= bool(self.bench.success(self.model, self.data, self.state, self.target)) if self.target else False
            self.damage_seen.update(k for k, v in self.bench.damage(self.model, self.data, self.state).items() if v)
            if self.partial is not None:
                self.partial.update(self.model, self.data)
        return self.observation()

    def status(self):
        disallowed = sorted(self.damage_seen - self.allowed_damage)
        final_success = bool(self.bench.success(self.model, self.data, self.state, self.target)) if self.target else False
        return dict(bench=getattr(self.bench, "BENCH_ID", self.module_path.stem), target=self.target,
                    instance=self.instance, setting=self.setting, physics_steps=self.steps, sim_seconds=float(self.data.time),
                    source_success_ever=self.ever_success, source_success_final=final_success,
                    source_success_revoked=bool(self.ever_success and not final_success),
                    source_acceptance_success=bool(self.ever_success and not disallowed),
                    damage=sorted(self.damage_seen), disallowed_damage=disallowed,
                    valid_success=bool(final_success and not disallowed),
                    **self._score(final_success, disallowed))

    def _score(self, final_success, disallowed):
        binary = float(final_success and not disallowed)
        if self.partial is None:
            return dict(score=binary, score_rule="binary source success")
        detail = self.partial.result()
        partial = 0. if disallowed else float(detail["partial_score"])
        consistent = (partial == 1.) == (binary == 1.)
        return dict(score=partial if consistent else min(partial, binary), score_consistent_with_source=consistent, binary_source_score=binary, partial_credit=detail,
                    score_rule="package scoring/partial_credit.py (prefix-hold k/N)")

    def metadata(self):
        return dict(module=self.module_path, module_sha256=hashlib.sha256(self.module_path.read_bytes()).hexdigest(),
                    bench=getattr(self.bench, "BENCH_ID", self.module_path.stem), targets=self.bench.TARGETS,
                    nq=self.model.nq, nv=self.model.nv, nu=self.model.nu,
                    physics_timestep=float(self.model.opt.timestep), control="source-native robot actuator controls",
                    actuators=self.actuators, mujoco_version=mujoco.__version__)

    def reference(self, *, max_steps=None):
        if self.target is None:
            raise ValueError("Reference requires a deployment target")
        actions = np.asarray(self.bench.reference(self.seed, self.target, self.params), dtype=float)
        if actions.ndim != 2 or actions.shape[1] != self.model.nu or not np.isfinite(actions).all():
            raise ValueError(f"Malformed reference actions: {actions.shape}")
        self.reset()
        limit = len(actions) if max_steps is None else min(len(actions), max_steps)
        limited = self.model.actuator_ctrllimited.astype(bool)
        outside = np.any((actions[:, limited] < self.model.actuator_ctrlrange[limited, 0] - 1e-9) |
                         (actions[:, limited] > self.model.actuator_ctrlrange[limited, 1] + 1e-9), axis=1)
        for action in actions[:limit]:
            # Source references assign raw ctrl; MuJoCo performs its own native
            # actuator limiting. Preserve that behavior, including bias residuals.
            self.step(action, validate_range=False)
        return actions, dict(self.status(), reference_physics_steps=len(actions),
                             reference_rows_outside_ctrlrange=int(outside.sum()),
                             complete=limit == len(actions),
                             evidence="privileged source robot reference; not a model evaluation or agent guard audit")

    def render(self, *, width=640, height=360, camera=None):
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.model, height=height, width=width)
        if camera is None:
            p = inspect.signature(self.bench.render).parameters.get("camera") if hasattr(self.bench, "render") else None
            camera = p.default if p and p.default is not inspect.Parameter.empty else -1
        self.renderer.update_scene(self.data, camera=camera)
        frame = self.renderer.render().copy()
        if frame.shape != (height, width, 3) or not frame.any() or np.ptp(frame) == 0:
            raise RuntimeError("Renderer returned malformed, black or constant RGB image")
        return frame

    def close(self):
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None
