"""Scripted full-score demo for the handscrew task, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 1200] [--no-record]

Each arm takes one handle from above, its fingers closing across the handle, and turns it by turning its wrist about
the handle's axis, a quarter turn at a time: grip, turn, open, turn back, grip again. The left arm turns the middle
spindle's handle (on the near jaw, pointing left), the right arm the end spindle's handle (on the far jaw, pointing
right), both clockwise seen from the handle's end and in step, so the jaws close parallel. The jaws start 8 mm wider
than the two 19 mm boards and a turn closes them 6 mm, so both handles go round 1 1/3 turns (480 degrees) to bring both
jaws onto the boards. Then the middle handle turns 60 degrees more and the end handle 10 degrees less, so the jaws
end parallel and bear flat along the whole joint. Then both arms let go and back away.

The plan comes from the fixed start layout (the task's constants: where the handles are, the opening, the boards'
thickness and the spindles' lead, all given in the setup) and the arms' own commands. While it runs it looks at each
handle's stripe (how far it has turned) and where the far jaw's handle is (a camera sees both), so the right hand
follows it. It never reads
the clamp's joint angles, its forces or the task's internal state to decide what to do (the progress log prints them).

Variants that must not score 1 (DEMO_VARIANT=...): nothing (both arms stay still), middle_only (only the middle
handle is turned, 540 degrees), end_only (only the end handle, 540 degrees), equal_tight (both handles turned together
and equally, 810 degrees, as far as a hand would tighten them).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

CONTROL_HZ = 20
P_HI, P_LO = math.radians(15), math.radians(-75)   # wrist pitch range about the handle axis (+y)
TURN_S, BACK_S, GRIP_S = 1.6, 1.2, 0.5
OPEN_GRIP = 1.0


def grip_rot(p: float, flip: bool = False) -> np.ndarray:
    """Gripper rotation: from straight down with the fingers closing along x, turned by ``p`` about world +y.
    ``flip`` turns the (symmetric) gripper half a turn about its own axis, which keeps the right wrist in range."""
    base = np.array([[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]])
    if flip:
        base = base @ np.diag([-1.0, -1.0, 1.0])
    c, s = math.cos(p), math.sin(p)
    ry = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    return ry @ base


class Arm:
    """One arm: smooth Cartesian moves (IK on the URDF), each starting where the last command left it."""

    def __init__(self, name: str, base_pos):
        from yam_arena.constants import READY
        from yam_arena import ik

        self.name, self.base = name, np.asarray(base_pos, float)
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.cmd = np.r_[self.q, self.grip]
        self.worst_mm = 0.0

    def _solve(self, pos, rot):
        from yam_arena import ik

        q, pe, _ = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), self.q, base_pos=self.base, iters=80, rot_weight=1.0)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), rot
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.cmd = np.r_[q, self.grip]

    def move(self, pos, rot, duration: float, grip: float | None = None, path=None):
        """Generator: go to ``pos``/``rot`` (world TCP); ``grip`` 1 open .. 0 closed (None keeps it). ``path(s)``
        optionally overrides the position along the way."""
        from yam_arena import ik

        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            p = path(s) if path else p0 + s * (np.asarray(pos) - p0)
            self._solve(p, ik.slerp_matrix(r0, rot, s))
            yield

    def set_grip(self, grip: float, duration: float = GRIP_S):
        yield from self.move(self.pos, self.rot, duration, grip)


def wait(seconds: float):
    for _ in range(max(1, round(seconds * CONTROL_HZ))):
        yield


def both(*gens):
    gens = [iter(g) for g in gens]
    while gens:
        for g in list(gens):
            try:
                next(g)
            except StopIteration:
                gens.remove(g)
        if gens:
            yield


def script(task, ctx):
    """The demo as a generator: one yield per control step."""
    T = task
    grip_rot_ = grip_rot
    variant = os.environ.get("DEMO_VARIANT", "")
    reach = 2 * math.pi * (T.G0 - 2 * T.BOARD_T) / T.LEAD          # 480 degrees: both jaws onto the boards
    plan = {"": (reach + math.radians(60), reach - math.radians(10)),
            "middle_only": (reach + math.radians(60), 0.0),
            "end_only": (0.0, reach + math.radians(60)),
            "equal_tight": (reach + math.radians(330), reach + math.radians(330))}
    turn1, turn2 = plan.get(variant, plan[""])
    grasp_y = 0.150   # along each handle, 15 cm out from the middle of the opening (handles run 7.1 to 18.6 cm)

    def handle_point(side):
        """Where to hold a handle: on its axis, ``grasp_y`` out; for the far jaw's handle, where it is seen now."""
        if side == "left":
            return np.array([T.HANDLE1_MID[0], grasp_y, T.HANDLE1_MID[2]])
        h = ctx.link_pos("handle2")    # the handle's root, on its axis at the jaw's outer face
        return np.array([h[0], h[1] - (grasp_y - (T.G0 / 2 + T.JAW_W)), h[2]])

    def turner(A, side, total, sign):
        def grip_rot(p, _f=(side == "right")):
            return grip_rot_(p, _f)

        """Turn one handle by ``total`` (rad) clockwise seen from its end: wrist pitch ``sign`` x total about +y."""
        if total <= 0:
            return
        # left (sign -1): strokes from P_HI down to P_LO; right (sign +1): from P_LO up to P_HI (first one from -30 deg)
        start = math.radians(-30)
        first = (start - P_LO) if sign < 0 else P_HI - start
        p = start
        c = handle_point(side)
        approach = grip_rot(p)[:, 2]
        yield from A.move(c - 0.10 * approach + [0, 0, 0.03], grip_rot(p), 3.0, grip=1.0)
        yield from A.move(c - 0.04 * approach, grip_rot(p), 1.0, grip=OPEN_GRIP)
        yield from A.move(c, grip_rot(p), 1.0)
        link = "handle1" if side == "left" else "handle2"
        d = first
        for i in range(12):
            if i:
                remaining = total - sign * ctx.seen[link]       # how far the handle's stripe still has to go
                if remaining < math.radians(2):
                    break
                p_back = P_HI if sign < 0 else P_LO
                d = min(P_HI - P_LO, remaining)
                c = handle_point(side)
                yield from A.move(c, grip_rot(p_back), BACK_S)
                p = p_back
            yield from A.set_grip(0.0)
            c0 = c.copy()
            # the far jaw's handle moves toward the boards as it turns (until the jaws meet the boards): follow it
            closing = (min(d, max(0.0, reach - sign * ctx.seen[link])) * T.R_LEAD) if side == "right" else 0.0
            yield from A.move(c0 + [0, closing, 0], grip_rot(p + sign * d), TURN_S * d / (P_HI - P_LO) + 0.3,
                              path=lambda s, c0=c0, closing=closing: c0 + [0, s * closing, 0])
            p += sign * d
            c = A.pos.copy()
            yield from wait(0.2)
            yield from A.set_grip(OPEN_GRIP, 0.6)
            yield from wait(0.2)
        print(f"{side} handle stopped at {math.degrees(sign * ctx.seen[link]):.0f} deg (target "
              f"{math.degrees(total):.0f}) at step {ctx.steps}", flush=True)
        # let go: back off along the gripper's axis, then up and away
        p = math.radians(-30)                       # turn the open hand to mid-range about the handle, then back off
        yield from A.move(A.pos, grip_rot(p), 0.8, grip=1.0)
        approach = grip_rot(p)[:, 2]
        yield from A.move(A.pos - 0.08 * approach, A.rot, 1.2)
        yield from A.move(A.pos + [-0.05, 0.05 if side == "left" else -0.05, 0.10], grip_rot(math.radians(-30)), 1.5)

    if variant == "nothing":
        yield from wait(50.0)
        return
    yield from both(turner(ctx.left, "left", turn1, -1.0), turner(ctx.right, "right", turn2, +1.0))


class Ctx:
    def __init__(self, emb):
        from yam_arena import station

        self.emb = emb
        self.left = Arm("left", station.BASES["left"])
        self.right = Arm("right", station.BASES["right"])
        self.steps = 0
        self.seen = {"handle1": 0.0, "handle2": 0.0}   # each handle's turn about +y, from its stripe (unwrapped)
        self._last = None

    def look(self):
        """After each step: how far each handle has turned, from the pose of the handle (its stripe) seen by a camera."""
        base = self.emb._env.unwrapped
        a = base.scene["handscrew"]
        now = {}
        for link in self.seen:
            x, y, z, w = a.data.body_link_pose_w[0, a.body_names.index(link), 3:7].tolist()
            r02, r00 = 2 * (x * z + w * y), 1 - 2 * (y * y + z * z)
            now[link] = math.atan2(r02, r00)
        if self._last is not None:
            for k in now:
                self.seen[k] += (now[k] - self._last[k] + math.pi) % (2 * math.pi) - math.pi
        self._last = now

    def states(self):
        from yam_arena.scene import object_states

        return object_states(self.emb._env, self.emb.task.names)

    def link_pos(self, link: str) -> np.ndarray:
        """World position (relative to the env origin) of one of the clamp's parts, as a camera would see it."""
        base = self.emb._env.unwrapped
        a = base.scene["handscrew"]
        p = a.data.body_link_pose_w[0, a.body_names.index(link), :3].cpu().numpy().astype(float)
        return p - base.scene.env_origins[0].cpu().numpy()

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack

        return Action(data=pack(self.left.cmd, self.right.cmd))


def _write_mp4(frames, path: Path) -> None:
    import imageio

    imageio.mimsave(path, [np.asarray(f) for f in frames], fps=CONTROL_HZ, codec="libx264",
                    output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])


def run(task_path: str, out_dir: str, max_steps: int = 1200, record: bool = True, cam_width: int = 320,
        cam_height: int = 176, hold_s: float = 3.0) -> dict:
    from types import SimpleNamespace

    from inspect_robots import Scene
    from yam_arena.inspect_embodiment import yam_arena_embodiment
    from yam_arena.scoring import task_score

    task_path = str(Path(task_path).resolve())
    sys.path.insert(0, str(Path(task_path).parent))
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=task_path, object_jitter=0, control_hz=CONTROL_HZ, cam_width=cam_width,
                               cam_height=cam_height)
    obs = emb.reset(Scene(id="demo", instruction="demo"))
    task = emb.task.module
    ctx = Ctx(emb)
    ctx.look()
    gen = script(task, ctx)
    frames: dict[str, list] = {}

    def snap(o):
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    snap(obs)
    steps, ended, res = 0, None, None
    debug = bool(os.environ.get("DEMO_DEBUG"))
    with (out / "actions.jsonl").open("w") as log:
        while steps < max_steps:
            try:
                next(gen)
            except StopIteration:
                ended = steps if ended is None else ended
            act = ctx.action()
            res = emb.step(act)
            steps = ctx.steps = steps + 1
            ctx.look()
            log.write(json.dumps({"step": steps, "action": np.asarray(act.data, float).round(6).tolist()}) + "\n")
            snap(res.observation)
            if steps % 50 == 0 or (debug and steps % 10 == 0):
                s = ctx.states()
                j = s["handscrew"].joints
                m = task._STATE.get("last") or {}
                g = m.get("gaps")
                phi = task._STATE["phi"]
                print(f"step {steps} score {res.info.get('score')} h1 {math.degrees(phi[0]):.0f} h2 "
                      f"{math.degrees(phi[1]):.0f} slide_mm {j['slide'] * 1000:.2f} cant_deg "
                      f"{math.degrees(j['jawb']):.2f} force {np.round(task._STATE['force'], 1).tolist()} gaps_mm "
                      f"{None if g is None else np.round(np.array(g) * 1000, 2).tolist()} joint_mm "
                      f"{None if m.get('joint') is None else np.round(np.array(m['joint']) * 1000, 2).tolist()} released "
                      f"{task._STATE['released']} clamp {np.round(s['handscrew'].pos, 4).tolist()} a "
                      f"{np.round(s['board_a'].pos, 4).tolist()} b {np.round(s['board_b'].pos, 4).tolist()} "
                      f"why {task._STATE['why']!r}", flush=True)
            if res.terminated:
                break
            if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
                break
    if os.environ.get("DEMO_DEBUG") == "diag":
        st = ctx.states()["handscrew"]
        j = st.joints
        for link in ("jawb", "handle2", "slide"):
            w = ctx.link_pos(link)
            base = emb._env.unwrapped
            a = base.scene["handscrew"]
            x, y, z, ww = a.data.body_link_pose_w[0, a.body_names.index(link), 3:7].tolist()
            yaw_link = math.degrees(2 * math.atan2(z, ww))
            print(f"DIAG {link} actual_local {np.round(st.to_local(w) * 1000, 2).tolist()} world_yaw {yaw_link:.3f} "
                  f"clamp_yaw {math.degrees(2 * math.atan2(st.quat_xyzw[2], st.quat_xyzw[3])):.3f}", flush=True)
        print(f"DIAG predicted jawb anchor {np.round(task.jawb_to_clamp(j['slide'], j['jawb'], task.JAWB_ANCHOR) * 1000, 2).tolist()} "
              f"handle2 {np.round(task.jawb_to_clamp(j['slide'], j['jawb'], (task.X2, task.YB_IN - task.JAW_W, 0)) * 1000, 2).tolist()} "
              f"joints {j}", flush=True)
    scored = task_score()(SimpleNamespace(steps=[SimpleNamespace(result=res)]), None)
    print(f"task_score: {scored.value:g} ({scored.explanation}) after {steps} steps", flush=True)
    m = task._STATE.get("last") or {}
    result = {"task_score": scored.value, "explanation": scored.explanation, "steps": steps,
              "variant": os.environ.get("DEMO_VARIANT", ""), "terminated": bool(res.terminated),
              "last": {k: (np.round(np.array(v) * 1000, 2).tolist() if k in ("gaps", "joint") and v is not None else v)
                       for k, v in m.items()},
              "worst_ik_mm": {"left": round(ctx.left.worst_mm, 2), "right": round(ctx.right.worst_mm, 2)}}
    if record:
        import imageio

        cams = sorted(frames, key=lambda c: ("left" not in c, "top" not in c))   # left, top, right
        _write_mp4([np.hstack([frames[c][i] for c in cams]) for i in range(len(frames[cams[0]]))], out / "demo.mp4")
        imageio.imwrite(out / "setup.png", np.hstack([frames[c][0] for c in cams]))
    (out / "result.json").write_text(json.dumps(result, indent=2, default=str))
    print(json.dumps(result, default=str), flush=True)
    return result


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=1200)
    ap.add_argument("--no-record", action="store_true")
    a = ap.parse_args()
    run(a.task, a.out, a.max_steps, record=not a.no_record)
