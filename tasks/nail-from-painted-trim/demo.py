"""Scripted full-score demo for the painted-trim nail task, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 1200] [--no-record]

Open loop from the published start layout; the only thing it looks at while running is the hammer's pose.
1. The left arm picks the scrap block off the table and lays it on the painted board just beyond the nail.
The right arm:
2. takes the hammer out of its pocket by the handle (a side grip, fingers pointing left);
3. sets the hammer's rounded claw-back on the block a little beyond the nail and slides it toward the robot, so
   the claw's slot passes either side of the nail under its head;
4. rocks the hammer on its claw-back, handle away from the robot (55 degrees commanded): the claw lifts the nail out.
It keeps holding the hammer at the end. Writes one MP4 per camera, setup.png and result.json to the output folder.

DEMO_VARIANT selects a cheap strategy that must score 0:
  nothing     the arms stay still;
  no_block    the same hammer moves with the claw-back on the bare paint (skips the block);
  pinch       pinch the nail with the fingers under its head and pull straight up;
  hook_pull   (block placed) claw under the head, then lift the hammer straight up instead of rocking it;
  knock       sweep the closed gripper through the hammer in its rack.
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
ROCK_DEG = 55.0
PRESS = 0.008  # m: how far past the support the claw-back is aimed while rocking (the arm leans on it)
PULL = 0.005  # m: how far past the shank the throat is aimed
GRIP_YAW = float(os.environ.get("DEMO_GRIP_YAW", str(math.pi)))  # turn of the grip about its approach axis (0 or pi)


def _rx(a) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(a) -> np.ndarray:
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

    def move_joint(self, pos, rot, duration: float, grip: float | None = None):
        """Generator: go to ``pos``/``rot`` by a straight line in joint space, to the IK solution (from several
        starting guesses) closest to the current joints among the accurate ones. For large re-orientations."""
        from yam_arena import ik
        from yam_arena.constants import READY

        seeds = [self.q, np.array(READY, float), np.array([0.0, 1.5, 1.5, 0.5, -1.4, 1.0]),
                 np.array([0.0, 2.0, 2.0, -0.5, 0.0, 0.0]), np.array([0.0, 2.5, 2.0, 0.6, -1.4, 1.0])]
        sols = [ik.solve(pos, ik.matrix_to_quat_xyzw(rot), s, base_pos=self.base, iters=300) for s in seeds]
        good = [r for r in sols if r[1] < 0.002 and r[2] < 0.02] or [min(sols, key=lambda r: r[1] + 0.05 * r[2])]
        q1 = min(good, key=lambda r: np.abs(r[0] - self.q).sum())[0]
        q0, g0 = self.q.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.q, self.grip = q0 + s * (q1 - q0), g0 + s * (g1 - g0)
            self.cmd = np.r_[self.q, self.grip]
            yield
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.worst_mm = max(self.worst_mm, float(np.linalg.norm(self.pos - np.asarray(pos))) * 1000)

    def follow(self, pose_at, duration: float):
        """Generator: track ``pose_at(s)`` -> (pos, rot) for s from 0 to 1 (smoothstep in time)."""
        from yam_arena import ik

        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            self._solve(*pose_at(ik.smoothstep(k / n)))
            yield

    def set_grip(self, grip: float, duration: float = 0.8):
        yield from self.move(self.pos, self.rot, duration, grip)


def wait(seconds: float):
    for _ in range(max(1, round(seconds * CONTROL_HZ))):
        yield


def script(T, ctx):
    """The demo: one yield per control step, both arms. Reads only the task's published layout."""
    left, right = _left_script(T, ctx), _right_script(T, ctx)
    done = [False, False]
    while not all(done):
        for i, g in enumerate((left, right)):
            if not done[i]:
                try:
                    next(g)
                except StopIteration:
                    done[i] = True
        yield


def _left_script(T, ctx):
    """Left arm: the scrap block from the table onto the paint just beyond the nail, long side along x."""
    from yam_arena.demo import grip_rotation

    A = ctx.left
    variant = os.environ.get("DEMO_VARIANT", "")
    if variant in ("nothing", "no_block", "pinch", "knock"):
        ctx.block_placed = True
        return
    down = grip_rotation(0.0)
    nail = np.r_[T.NAIL_XY, T.PAINT]
    b = np.array(T.BLOCK.pos)
    pick = np.r_[b[:2], T.TOP + 0.018]
    place = np.r_[nail[:2] + [T.BLOCK_SIZE[0] / 2 + 0.010, 0.0], T.PAINT + 0.020]
    yield from A.move_joint(pick + [0, 0, 0.12], down, 2.5, grip=1.0)
    yield from A.move(pick, down, 1.2)
    yield from A.set_grip(0.0, 0.8)
    yield from A.move(pick + [0, 0, 0.12], down, 1.0)
    yield from A.move(np.r_[place[:2], T.TOP + 0.09], down, 2.0)
    yield from A.move(place, down, 1.2)
    yield from A.set_grip(1.0, 0.6)
    yield from A.move(place + [0, 0, 0.05], down, 0.8)
    yield from A.move_joint(np.r_[0.20, 0.30, T.TOP + 0.20], down, 1.5)
    ctx.block_placed = True


def _right_script(T, ctx):
    """Right arm: the hammer out of its pocket, claw under the nail head with the claw-back on the block, rock."""
    from yam_arena.demo import grip_rotation

    A = ctx.right
    variant = os.environ.get("DEMO_VARIANT", "")
    if variant == "nothing":
        yield from wait(2.0)
        return
    down = grip_rotation(0.0)
    # side grip: the fingers point left (+y) and close front to back (along x) on the handle, so rocking the hammer
    # is a roll of the hand about its own pointing axis
    side = _ry(math.pi / 2) @ _rx(math.pi / 2) @ grip_rotation(GRIP_YAW)
    nail = np.r_[T.NAIL_XY, T.PAINT]

    if variant == "knock":
        h = np.r_[T.POCKET_XY, T.TOP + 0.20]
        yield from A.set_grip(0.0, 0.5)
        yield from A.move_joint(h + [0.0, -0.08, 0], side, 2.5)
        yield from A.move(h + [0.0, 0.10, 0], side, 1.5)
        yield from A.move(h + [0.08, 0.10, 0], side, 1.0)
        yield from A.move(h + [0.08, -0.08, 0], side, 1.5)
        yield from wait(2.0)
        return

    if variant == "pinch":
        # fingers either side of the shank, tips just under the head; close and pull straight up
        tcp = nail + [0, 0, T.PROUD - 0.006 - 0.004 + 0.013]
        yield from A.move_joint(tcp + [0, 0, 0.10], down, 3.0, grip=1.0)
        yield from A.move(tcp, down, 1.5)
        yield from A.set_grip(0.0, 1.0)
        yield from A.move(tcp + [0, 0, 0.06], down, 4.0)
        yield from wait(2.0)
        return

    # The hammer: fingers open above the handle, down round it, close, straight up out of the pocket.
    grip = np.array([0, 0, T.GRIP_Z])
    g0 = np.r_[T.POCKET_XY, T.TOP + T.RB] + grip
    yield from A.move_joint(g0 + [0, 0, 0.11], side, 3.0, grip=1.0)
    yield from A.move(g0, side, 1.5)
    yield from A.set_grip(0.0, 0.8)
    yield from wait(0.3)
    yield from A.move(g0 + [0, 0, 0.055], side, 1.2)  # head clear of the pocket (the arm sags about 1 cm)
    yield from A.move(g0 + [0, 0.07, 0.055], side, 1.5)  # out of the collar, to the left
    yield from A.move(g0 + [0, 0.07, 0.11], side, 1.0)
    yield from wait(0.5)
    # Where the hammer sits in the hand (it may have slipped a little): from its pose, as a camera would see it.
    cal = {}

    def calibrate():
        """Re-measure where the hammer sits relative to the commanded hand pose (grip slip and arm sag)."""
        h = ctx.hammer_pose()
        if h is None:  # dry run without a simulator
            cal["off"], cal["rel"] = A.rot.T @ -(A.rot @ (side.T @ grip)), A.rot.T @ np.eye(3)
        else:
            cal["off"], cal["rel"] = A.rot.T @ (h[0] - A.pos), A.rot.T @ h[1]

    calibrate()

    def hand(o, r_hammer=np.eye(3)):
        """Hand pose that puts the hammer's claw-back centre at ``o`` with rotation ``r_hammer``."""
        r = r_hammer @ cal["rel"].T
        return o - r @ cal["off"], r

    while not getattr(ctx, "block_placed", False):  # the left arm is still at the board
        yield

    # Claw-back down on the block (or the paint) beyond the nail; slide toward the robot under the nail head.
    support = T.PAINT + (0.0 if variant == "no_block" else T.BLOCK_SIZE[2])
    o_rest = support + T.RB - 0.001  # pressed 1 mm into the support: keeps the claw-back on it
    o_start = np.r_[nail[0] + 0.085, nail[1], o_rest + 0.004]
    o_eng = np.r_[nail[0] - T.THROAT_X + 0.002, nail[1], o_rest]
    yield from A.move(*hand(o_start + [0, 0, 0.03]), 3.0)
    yield from wait(0.6)
    calibrate()
    yield from A.move(*hand(o_start + [0, 0, 0.03]), 0.5)
    yield from wait(0.4)
    calibrate()
    yield from A.move(*hand(o_start), 1.2)
    yield from A.move(*hand(np.r_[o_start[:2], o_rest]), 0.6)
    yield from A.move(*hand(o_eng), 2.0)
    yield from wait(0.3)

    if variant == "hook_pull":
        yield from A.move(*hand(o_eng + [0, 0, 0.05]), 4.0)
        yield from wait(2.0)
        return

    # Rock on the claw-back: it rolls toward +x by RB per radian while the hammer turns, handle away from the robot.
    # Lean on it as a person does: the claw-back is pressed down into the block and the throat toward the shank
    # (targets PRESS / PULL past the contact), so the hammer turns about its claw-back instead of sliding off.
    def leaned(s):
        return hand(o_eng + np.array([-PULL * s, 0.0, -PRESS * s]))

    yield from A.follow(leaned, 1.0)

    def rocked(s):
        phi = math.radians(ROCK_DEG) * s
        o = o_eng + np.array([T.RB * phi - PULL, 0.0, -PRESS])
        return hand(o, _ry(phi))

    yield from A.follow(rocked, 8.0)
    yield from wait(1.5)


class Ctx:
    def __init__(self, emb):
        from yam_arena import station

        self.emb = emb
        self.left = Arm("left", station.BASES["left"])
        self.right = Arm("right", station.BASES["right"])

    def hammer_pose(self):
        """The hammer's pose (origin, rotation matrix): what a camera tracking the hammer would report."""
        h = self.states()["hammer"]
        x, y, z, w = h.quat_xyzw
        r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                      [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                      [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
        return np.asarray(h.pos, float), r

    def states(self):
        from yam_arena.scene import object_states

        return object_states(self.emb._env, self.emb.task.names)

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
    from inspect_robots import Scene
    from yam_arena.inspect_embodiment import yam_arena_embodiment

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
    debug = bool(os.environ.get("DEMO_DEBUG"))

    def snap(o):
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    snap(obs)
    steps, ended, success, info, trace = 0, None, False, {}, []
    while steps < max_steps:
        try:
            next(gen)
        except StopIteration:
            ended = steps if ended is None else ended
        res = emb.step(ctx.action())
        steps += 1
        info = res.info
        snap(res.observation)
        if debug or steps % 100 == 0:  # log only: nothing below feeds the script
            st = ctx.states()
            h, tr, b = st["hammer"], st["trim"], st["block"]
            row = dict(step=steps, score=info.get("score"), nail_mm=round(1000 * task.nail_lift(tr), 2),
                       hammer=np.round(h.pos, 4).tolist(), trim=np.round(tr.pos, 4).tolist(),
                       block=np.round(b.pos, 4).tolist(), touched=task.paint_touched(h),
                       tcp_cmd=np.round(ctx.right.pos, 4).tolist(),
                       tilt_deg=round(math.degrees(math.acos(min(1.0, task._rot(h.quat_xyzw)[2, 2]))), 1),
                       quat=np.round(h.quat_xyzw, 3).tolist(), grip=round(float(ctx.right.grip), 2))
            head = np.r_[task.NAIL_XY, task.PAINT + task.PROUD + task.nail_lift(tr)]
            row["head_in_hammer_mm"] = np.round(1000 * (task._rot(h.quat_xyzw).T @ (head - h.pos)), 1).tolist()
            if debug and task.paint_touched(h):
                pts = h.pos + task.HAMMER_POINTS @ task._rot(h.quat_xyzw).T
                row["low_pt"] = np.round(pts[np.argmin(pts[:, 2])], 4).tolist()
            trace.append(row)
            if steps % (10 if debug else 100) == 0:
                print(json.dumps(row), flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    score = 1.0 if success else float(info.get("score", 0.0) or 0.0)
    result = {"task_score": score, "success": success, "steps": steps, "variant": os.environ.get("DEMO_VARIANT", ""),
              "worst_ik_mm": {"left": round(ctx.left.worst_mm, 2), "right": round(ctx.right.worst_mm, 2)}}
    if record:
        import imageio

        for cam, fs in frames.items():
            _write_mp4(fs, out / f"demo_{cam}.mp4")
        top = next((c for c in frames if "top" in c), next(iter(frames), None))
        if top:
            imageio.imwrite(out / "setup.png", frames[top][0])
    (out / "trace.json").write_text(json.dumps(trace))
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
