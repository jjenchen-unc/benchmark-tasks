"""Scripted full-score demo for the can opener task, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 1200] [--no-record]

It is closed loop. It plans from the fixed start layout (where the lever handle and the crank knob are), and while it
runs it reads only object poses (the can's and the lid's, which a camera could see) and the arms' own commands.
The left arm presses the lever's handle down with closed fingers and holds it there. The right arm grips the crank
knob and turns the crank clockwise, and at every control step measures how far the can has turned about its axis from
the can's pose (what the white stripe on its label shows). Once the can has turned one full turn plus a 15 degree
margin, it stops cranking. Then the right arm lets go of the knob, the left arm lets the lever rise, and the right arm
pinches the lid by its seam where it is measured to be, lifts it off the can and lays it in the tray.
The episode runs in the yam_arena embodiment with the task file, and the task_score scorer scores its last step.
Writes demo.mp4 (the three camera views side by side), setup.png (the first frames), actions.jsonl (the 14 joint
commands sent at each step) and result.json to the output folder.

Variants that must not score 1 (DEMO_VARIANT=...): no_hold (crank without holding the lever down), no_crank (hold the
lever, then try to lift the lid without cranking), backwards (crank anticlockwise), half (stop when the can has
turned half a turn, then try to lift the lid), nothing (both arms stay still), knock (shove the can and lid, pry and pull the uncut lid).
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
STOP_TURN = math.radians(360 + 15)   # stop cranking once the can has turned one full turn plus this margin
CRANK_SPEED = 2.0           # rad/s at the knob, about 3 s per turn
OVERPRESS = math.radians(5) # the handle is commanded 5 degrees past the closed stop, so the arm presses on it


def _rot_y(a) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


class Arm:
    """One arm: smooth Cartesian moves (IK on the URDF), each starting where the last command left it."""

    def __init__(self, name: str, base_pos):
        from yam_arena import ik
        from yam_arena.constants import READY

        self.name, self.base = name, np.asarray(base_pos, float)
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.cmd = np.r_[self.q, self.grip]
        self.worst_mm = 0.0

    def _solve(self, pos, rot):
        from yam_arena import ik

        q, pe, _ = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), self.q, base_pos=self.base, iters=60)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), rot
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.cmd = np.r_[q, self.grip]

    def move(self, pos, rot, duration: float, grip: float | None = None):
        """Generator: go to ``pos``/``rot`` (world TCP); ``grip`` 1 open .. 0 closed (None keeps it)."""
        from yam_arena import ik

        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self._solve(p0 + s * (np.asarray(pos) - p0), ik.slerp_matrix(r0, rot, s))
            yield

    def set_grip(self, grip: float, duration: float = 0.8):
        yield from self.move(self.pos, self.rot, duration, grip)

    def aim(self, pos, rot):
        self._solve(pos, rot)


def reachable_yaw(arm, pos, yaw):
    """The grip yaw, or the same parallel grip turned 180 degrees, whichever the arm reaches more exactly."""
    from yam_arena import ik
    from yam_arena.demo import grip_rotation

    best = None
    for y in (yaw, yaw + math.pi, yaw - math.pi):
        _, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(grip_rotation(y)), arm.q, base_pos=arm.base, iters=200)
        if best is None or pe + 0.05 * re < best[0]:
            best = (pe + 0.05 * re, y)
    return best[1]


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def wait(seconds: float):
    for _ in range(max(1, round(seconds * CONTROL_HZ))):
        yield


def wait_for(flags, key):
    while not flags.get(key):
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
    from yam_arena.demo import grip_rotation

    T = task
    variant = os.environ.get("DEMO_VARIANT", "")
    flags: dict = {}
    origin = np.array([*T.OPENER_XY, T.TOP])
    pivot = origin + np.array(T.PIVOT)

    def handle(theta):
        """World point in the middle of the handle's top face with the lever at ``theta``."""
        return pivot + _rot_y(theta) @ (np.array(T.HANDLE_PRESS) - np.array(T.PIVOT))

    def knob(phi):
        c, s = math.cos(phi), math.sin(phi)
        off = np.array([s * T.CRANK_R, -c * T.CRANK_R])  # the knob starts at -y of the shaft; phi turns it about z
        return origin + np.array([T.CRANK_AXIS[0] + off[0], T.CRANK_AXIS[1] + off[1], T.KNOB_TOP - 0.010])

    def left():
        A = ctx.left
        down = grip_rotation(-math.pi / 2)            # fingers closing front to back, clear of the right hand
        tip = np.array([0, 0, 0.013])                 # TCP above the closed fingertips
        if variant == "no_hold":
            flags["lever_down"] = True
            yield from wait_for(flags, "crank_done")
            flags["lever_up"] = True
            return
        yield from A.move(handle(T.LEVER_OPEN) + tip + [-0.02, 0, 0.012], down, 3.0, grip=0.0)
        yield from A.move(handle(T.LEVER_OPEN) + tip, down, 1.0)
        n = round(2.5 * CONTROL_HZ)
        for k in range(1, n + 1):                     # press the handle down along its arc, past the stop
            s = 0.5 - 0.5 * math.cos(math.pi * k / n)
            A.aim(handle(T.LEVER_OPEN - s * (T.LEVER_OPEN + OVERPRESS)) + tip, down)
            yield
        yield from wait(0.5)
        flags["lever_down"] = True
        yield from wait_for(flags, "crank_done")
        n = round(2.0 * CONTROL_HZ)
        for k in range(1, n + 1):                     # let it rise under the hand, then lift away
            s = 0.5 - 0.5 * math.cos(math.pi * k / n)
            A.aim(handle(-OVERPRESS + s * (T.LEVER_OPEN + OVERPRESS + math.radians(3))) + tip, down)
            yield
        yield from A.move(A.pos + [-0.04, 0.04, 0.03], down, 1.5)
        flags["lever_up"] = True

    def right():
        A = ctx.right
        down = grip_rotation(math.pi / 2)
        if variant != "no_crank":
            yield from A.move(knob(0.0) + [0, 0, 0.04], down, 3.0, grip=1.0)
            yield from A.move(knob(0.0), down, 1.5)
            yield from A.set_grip(0.0, 1.0)
            yield from wait_for(flags, "lever_down")
            stop = math.pi if variant == "half" else STOP_TURN
            sign = 1.0 if variant == "backwards" else -1.0   # clockwise seen from above is negative about z
            phi, v, ramp = 0.0, 0.0, 0.5
            yaw = ctx.can_yaw()
            turned = 0.0                              # how far the can has turned, measured from its pose
            while turned < stop:                      # speed up over half a second, then turn at constant speed
                v = min(CRANK_SPEED, v + CRANK_SPEED / (ramp * CONTROL_HZ))
                phi += v / CONTROL_HZ
                A.aim(knob(sign * phi), down)
                yield
                y = ctx.can_yaw()
                turned += abs(_wrap(y - yaw))
                yaw = y
            print(f"crank stop: step {ctx.steps}, can turned {math.degrees(turned):.1f} deg (threshold "
                  f"{math.degrees(stop):.0f} deg), crank commanded {phi / (2 * math.pi):.2f} turns", flush=True)
            while v > 0:                              # slow down over half a second
                v = max(0.0, v - CRANK_SPEED / (ramp * CONTROL_HZ))
                phi += v / CONTROL_HZ
                A.aim(knob(sign * phi), down)
                yield
            yield from wait(0.3)
            yield from A.set_grip(1.0, 0.8)
            yield from A.move(A.pos + [0, 0, 0.04], down, 1.0)
        else:
            yield from wait_for(flags, "lever_down")
            yield from wait(2.0)
        flags["crank_done"] = True
        yield from wait_for(flags, "lever_up")

        # Pinch the lid across its seam (fingers closing front to back), lift it, carry it to the tray.
        lid = ctx.states()["lid"]
        p = lid.pos.copy()
        yaw = reachable_yaw(A, p, math.pi / 2)
        rot = grip_rotation(yaw)
        yield from A.move(p + [0, 0, 0.05], rot, 2.0, grip=1.0)
        yield from A.move(p + [0, 0, 0.001], rot, 1.5)
        yield from A.set_grip(0.0, 1.0)
        yield from wait(0.3)
        yield from A.move(p + [0, 0, 0.06], rot, 1.5)
        # toward the robot first, then to the right: the crank and its knob stay clear on the can's right
        drop = np.array([T.TRAY_X, T.TRAY_Y, T.TRAY_FLOOR + T.LID_H / 2 + 0.006])
        yield from A.move(np.array([T.TRAY_X, p[1], p[2] + 0.06]), rot, 1.5)
        yield from A.move(drop + [0, 0, 0.05], rot, 2.0)
        yield from A.move(drop, rot, 1.2)
        yield from A.set_grip(1.0, 0.8)
        yield from A.move(drop + [0, 0, 0.06], rot, 1.2)

    def knock():
        """Cheap strategy: shove the can and lid from the front and side with closed fingers, then pry the lid up."""
        A = ctx.right
        rot = grip_rotation(math.pi / 2)
        lid = origin + np.array([0.0, 0.0, T.CAN_TOP + T.LID_H / 2])
        for dz in (0.0, -0.03):                       # at the lid, then at the can's middle
            yield from A.move(lid + [-0.09, 0, dz + 0.04], rot, 2.0, grip=0.0)
            yield from A.move(lid + [-0.09, 0, dz], rot, 1.0)
            yield from A.move(lid + [0.02, 0, dz], rot, 1.0)   # drive 5 cm into the can
            yield from A.move(lid + [-0.09, 0, dz + 0.04], rot, 1.0)
        yield from A.move(lid + [0, -0.07, 0.0], rot, 1.5)
        yield from A.move(lid + [0, 0.02, 0.0], rot, 1.0)      # sideways, toward the lever
        yield from A.move(lid + [0, -0.07, 0.04], rot, 1.0)
        yield from A.move(lid + [-0.03, 0, 0.0], rot, 1.5, grip=1.0)   # fingertip under the front of the seam
        yield from A.move(lid + [-0.03, 0, -0.004], rot, 0.8)
        yield from A.move(lid + [-0.01, 0, 0.05], rot, 1.5)    # pry up and back
        yield from A.move(lid + [0, 0, 0.05], rot, 1.0)
        yield from A.move(lid + [0, 0, 0.001], rot, 1.0)
        yield from A.set_grip(0.0, 1.0)
        yield from A.move(lid + [0, 0, 0.08], rot, 1.5)        # pull the uncut lid straight up

    if variant == "nothing":
        for _ in range(round(65 * CONTROL_HZ)):
            yield
    elif variant == "knock":
        yield from knock()
    else:
        yield from both(left(), right())


class Ctx:
    def __init__(self, emb):
        from yam_arena import station

        self.emb = emb
        self.left = Arm("left", station.BASES["left"])
        self.right = Arm("right", station.BASES["right"])
        self.steps = 0

    def states(self):
        from yam_arena.scene import object_states

        return object_states(self.emb._env, self.emb.task.names)

    def can_yaw(self) -> float:
        """The can's rotation about its vertical axis (rad), from the can's world pose."""
        a = self.emb._env.unwrapped.scene["opener"]
        x, y, z, w = a.data.body_link_pose_w[0, a.body_names.index("can"), 3:7].tolist()
        return 2 * math.atan2(z, w)

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack

        return Action(data=pack(self.left.cmd, self.right.cmd))


def _write_mp4(frames, path: Path) -> None:
    import imageio

    imageio.mimsave(path, [np.asarray(f) for f in frames], fps=CONTROL_HZ, codec="libx264",
                    output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])


def _actual(emb) -> str:
    """Debug: where each arm's TCP really is (world, m)."""
    from yam_arena.runtime import tcp_world

    base = emb._env.unwrapped
    origin = base.scene.env_origins[0].cpu().numpy()
    out = [f"{arm[0]}_act {np.round(tcp_world(base.scene[arm]) - origin, 3).tolist()}"
           for arm in ("left_robot", "right_robot")]
    return " " + " ".join(out)


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
    gen = script(task, ctx)
    frames: dict[str, list] = {}

    def snap(o):
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    snap(obs)
    steps, ended = 0, None
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
            log.write(json.dumps({"step": steps, "action": np.asarray(act.data, float).round(6).tolist()}) + "\n")
            snap(res.observation)
            if steps % 100 == 0 or (debug and steps % 10 == 0):
                s = ctx.states()
                j = s["opener"].joints
                print(f"step {steps} score {res.info.get('score')} lever_deg {math.degrees(j['lever']):.1f} crank_deg "
                      f"{math.degrees(j['crank']):.0f} can_deg {math.degrees(ctx.can_yaw()):.0f} lid "
                      f"{np.round(s['lid'].pos, 4).tolist()} tcp_l {np.round(ctx.left.pos, 3).tolist()} "
                      f"tcp_r {np.round(ctx.right.pos, 3).tolist()}{_actual(emb) if debug else ''}", flush=True)
            if res.terminated:
                break
            if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
                break
    scored = task_score()(SimpleNamespace(steps=[SimpleNamespace(result=res)]), None)
    print(f"task_score: {scored.value:g} ({scored.explanation}) after {steps} steps", flush=True)
    result = {"task_score": scored.value, "explanation": scored.explanation, "steps": steps,
              "variant": os.environ.get("DEMO_VARIANT", ""),
              "worst_ik_mm": {"left": round(ctx.left.worst_mm, 2), "right": round(ctx.right.worst_mm, 2)}}
    if record:
        import imageio

        cams = sorted(frames, key=lambda c: ("left" not in c, "top" not in c))   # left, top, right
        _write_mp4([np.hstack([frames[c][i] for c in cams]) for i in range(len(frames[cams[0]]))], out / "demo.mp4")
        imageio.imwrite(out / "setup.png", np.hstack([frames[c][0] for c in cams]))
    (out / "result.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result), flush=True)
    return result


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=1200)
    ap.add_argument("--no-record", action="store_true")
    a = ap.parse_args()
    run(a.task, a.out, a.max_steps, record=not a.no_record)
