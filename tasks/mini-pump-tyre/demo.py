"""Scripted full-score demo for the mini pump task, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 1800] [--no-record]

The left arm grips the pump's barrel just behind the head, lifts it, lines the head's opening up with the valve stem
and pushes it on, then keeps holding it. The right arm grips the handle knob and gives a fixed number of full strokes (the
pump has no gauge: the count comes from the published 0.08 bar per stroke), lets go, and the left arm pulls the pump
off the valve and lays it on the table. The plan uses only the published layout and the pump's measured pose.
Writes one MP4 per camera, setup.png (the first top-camera frame) and result.json to the output folder.

DEMO_VARIANT:
  no_seat    holds the head 1 cm short of the stem and strokes (air goes out of the head, not into the tyre)
  on_table   strokes the pump where it lies, without lifting it
  over       gives 24 strokes instead of 10
  stay       pumps but leaves the pump on the valve
  nothing    does not move
  swat       the right hand sweeps the pump across the table into the wheel, without picking it up
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
STROKES = int(os.environ.get("DEMO_STROKES", "10"))
GRIP_ON_BARREL = 0.046        # pump-frame y of the left hand's grip, right behind the head
TCP_ABOVE_AXIS = 0.004        # grip a little above the barrel's axis
GRIP_ON_KNOB_Z = -0.004       # right TCP at the knob's centre, close to the pump axis (little pitching moment)


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


def wait(seconds: float):
    for _ in range(max(1, round(seconds * CONTROL_HZ))):
        yield


def both(*gens):
    """Run several arms' generators side by side, one step each per control step."""
    gens = list(gens)
    while gens:
        for g in list(gens):
            try:
                next(g)
            except StopIteration:
                gens.remove(g)
        yield


def best_yaw(base, points, yaws):
    """The grip yaw (straight-down grip turned about the vertical) that IK reaches best at all ``points``."""
    from yam_arena import ik
    from yam_arena.constants import READY
    from yam_arena.demo import grip_rotation

    worst = {}
    for yaw in yaws:
        errs = []
        for p in points:
            _, pe, re = ik.solve(p, ik.matrix_to_quat_xyzw(grip_rotation(yaw)), np.array(READY, float), base_pos=base,
                                 iters=200)
            errs.append(pe + 0.05 * re)
        worst[yaw] = max(errs)
    return min(worst, key=worst.get), worst


def plan_points(T):
    """Where each hand has to reach, from the published layout (pump frame -> world with the pump's yaw)."""
    c, s = math.cos(T.PUMP_YAW), math.sin(T.PUMP_YAW)

    def world(origin, local):
        return np.array([origin[0] + c * local[0] - s * local[1], origin[1] + s * local[0] + c * local[1],
                         origin[2] + local[2]])

    seated = np.array([T.STEM_TIP[0], T.STEM_TIP[1] + T.BORE_DEPTH, T.STEM_Z])
    start = np.array(T.PUMP_START)
    left = [world(start, (0, GRIP_ON_BARREL, TCP_ABOVE_AXIS)), world(seated, (0, GRIP_ON_BARREL, TCP_ABOVE_AXIS)),
            world(seated, (0, GRIP_ON_BARREL, 0.08))]
    right = [world(seated, (0, T.GRIP_Y, 0.002)), world(seated, (0, T.GRIP_Y - T.TRAVEL, 0.002))]
    return seated, left, right


def script(task, ctx):
    from yam_arena.demo import grip_rotation

    T, L, R = task, ctx.left, ctx.right
    variant = os.environ.get("DEMO_VARIANT", "")
    if variant == "nothing":
        yield from wait(10.0)
        return
    if variant == "swat":
        p = ctx.states()["pump"]
        side = p.to_world((0, 0.12, 0)) + [-0.05, 0, 0.0]
        rrot0 = grip_rotation(0.0)
        yield from R.move(side + [0, 0, 0.08], rrot0, 2.0, grip=0.0)
        yield from R.move(side, rrot0, 1.0)
        yield from R.move(side + [0.10, 0.06, 0], rrot0, 1.5)
        yield from R.move(side + [0.10, 0.06, 0.10], rrot0, 1.0)
        yield from wait(2.0)
        return
    seated, lpts, rpts = plan_points(T)
    lyaw, _ = best_yaw(L.base, lpts, (math.pi / 2, -math.pi / 2))
    print(f"grip yaw left {math.degrees(lyaw):.0f} right 0", flush=True)
    ryaw = 0.0                                   # fingers close across the knob along y; wrist camera faces forward
    lrot, rrot = grip_rotation(lyaw), grip_rotation(ryaw)

    p0 = ctx.states()["pump"]
    grasp = p0.to_world((0, GRIP_ON_BARREL, 0)) + [0, 0, TCP_ABOVE_AXIS]
    # 1. Left hand: over the barrel, down, close, lift 5 cm.
    yield from L.move(grasp + [0, 0, 0.08], lrot, 2.5, grip=1.0)
    yield from L.move(grasp, lrot, 1.5)
    yield from L.set_grip(0.0, 1.0)
    yield from wait(0.3)
    yield from L.move(grasp + [0, 0, 0.05], lrot, 1.0)

    def pump_target_tcp(origin):
        """Left TCP that puts the pump's origin (the head's opening) at ``origin``, from the measured hold."""
        p = ctx.states()["pump"]
        return np.asarray(origin) + (L.pos - p.pos)

    if variant != "on_table":
        # 2. Carry the opening 3 cm short of its seated place, on the stem's line; correct twice from the measured pose.
        short = seated - [0, 0.030, 0]
        yield from L.move(pump_target_tcp(short + [0, 0, 0.04]), lrot, 2.5)
        yield from L.move(pump_target_tcp(short), lrot, 1.5)
        for _ in range(2):
            yield from wait(0.4)
            yield from L.move(pump_target_tcp(short), lrot, 0.6)
        # 3. Push the head onto the stem (2 mm past the stop, so it stays pressed on), or stop 1 cm short.
        on = seated + [0, 0.002, 0] if variant != "no_seat" else short + [0, 0.008, 0]
        yield from L.move(L.pos + (on - short), lrot, 1.5)
        yield from wait(0.5)

    # 4. Right hand on the handle knob (where the measured pump pose puts it), fingers on its two flat faces.
    p = ctx.states()["pump"]
    grip = p.to_world((0, T.GRIP_Y, 0)) + [0, 0, GRIP_ON_KNOB_Z]
    push = -p.rotation()[:, 1]                       # the handle pushes in along the pump's -y
    yield from R.move(grip + [0, 0, 0.08], rrot, 2.5, grip=1.0)
    yield from R.move(grip, rrot, 1.5)
    yield from R.set_grip(0.0, 0.8)
    yield from wait(0.3)

    # 5. Full strokes. The arm's joints are soft (the hand lags about 6 cm behind its target at 9 N), so it is aimed
    # 8 cm past the handle's stop going in and held there a moment, and 1 cm past its outer stop coming back.
    strokes = 24 if variant == "over" else STROKES
    for i in range(strokes):
        yield from R.move(grip + push * (T.TRAVEL + 0.08), rrot, 1.2)
        yield from wait(0.4)
        yield from R.move(grip - push * 0.01, rrot, 0.8)

    # 6. Let go of the handle and lift the right hand away.
    yield from R.set_grip(1.0, 0.6)
    yield from R.move(R.pos + [0, 0, 0.08], rrot, 1.0)
    yield from R.move(R.pos + [-0.05, -0.10, 0.0], rrot, 1.5)
    if variant == "stay":
        yield from wait(2.0)
        return

    # 7. Pull the pump straight off the valve, lay it on the table in front, let go.
    yield from L.move(L.pos - push * 0.05, lrot, 1.5)
    yield from L.move(L.pos + [-0.10, 0, 0], lrot, 1.5)
    held = ctx.states()["pump"]
    lower = held.pos[2] - (T.TOP + 0.020) - 0.003      # how far the pump axis is above lying on the table
    yield from L.move(L.pos - [0, 0, lower], lrot, 1.5)
    yield from L.set_grip(1.0, 0.6)
    yield from L.move(L.pos + [0, 0, 0.08], lrot, 1.0)


class Ctx:
    def __init__(self, emb):
        from yam_arena import station

        self.emb = emb
        self.left = Arm("left", station.BASES["left"])
        self.right = Arm("right", station.BASES["right"])

    def states(self):
        from yam_arena.scene import object_states

        return object_states(self.emb._env, self.emb.task.names)

    def measured_tcp(self, side):
        """Where the arm's TCP actually is (forward kinematics of the measured joints): for the log only."""
        from partkit import _np
        from yam_arena import ik

        robot = self.emb._env.unwrapped.scene[f"{side}_robot"]
        q = _np(robot.data.joint_pos)[0, :6]
        return ik.tcp_pose(q, getattr(self, side).base)[0]

    def knob_world(self):
        """The handle knob's centre as a camera would see it: the handle body's position plus the knob's offset."""
        p = self.states()["pump"]
        return self.knob() + p.rotation() @ np.array([0.0, self.emb.task.module.GRIP_Y, 0.0])

    def knob(self):
        """The handle body's origin (its rest frame sits on the pump's origin)."""
        from partkit import _np

        base = self.emb._env.unwrapped
        asset = base.scene["pump"]
        names = list(asset.body_names)
        pos = _np(asset.data.body_pos_w)[0]
        return pos[names.index("handle")] - _np(base.scene.env_origins[0])

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack

        return Action(data=pack(self.left.cmd, self.right.cmd))


def _write_mp4(frames, path: Path) -> None:
    import imageio

    imageio.mimsave(path, [np.asarray(f) for f in frames], fps=CONTROL_HZ, codec="libx264",
                    output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])


def run(task_path: str, out_dir: str, max_steps: int = 1800, record: bool = True, cam_width: int = 320,
        cam_height: int = 176, hold_s: float = 4.0) -> dict:
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
    writers: dict = {}

    def snap(o):
        if not record:
            return
        import imageio

        for cam, img in o.images.items():
            if cam not in writers:
                writers[cam] = imageio.get_writer(out / f"demo_{cam}.mp4", fps=CONTROL_HZ, codec="libx264",
                                                  output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])
                if "top" in cam:
                    imageio.imwrite(out / "setup.png", np.asarray(img))
            writers[cam].append_data(np.asarray(img))

    snap(obs)
    steps, ended, success, info = 0, None, False, {}
    while steps < max_steps:
        try:
            next(gen)
        except StopIteration:
            ended = steps if ended is None else ended
        res = emb.step(ctx.action())
        steps += 1
        info = res.info
        snap(res.observation)
        if steps % 10 == 0:                      # log only; the plan never reads these
            p = ctx.states()["pump"]
            seated, depth = task.head_on_valve(p)
            st = task._STATE
            print(f"step {steps} score {info.get('score')} tyre_bar {task.tyre_bar():.3f} handle_mm {1000 * st['x']:.1f} "
                  f"force_N {st['force']:.1f} seated {seated} depth_mm {1000 * depth:.1f} pump_pos "
                  f"{np.round(p.pos, 4).tolist()} L_tcp {np.round(ctx.left.pos, 3).tolist()} "
                  f"R_tcp {np.round(ctx.right.pos, 3).tolist()} R_meas {np.round(ctx.measured_tcp('right'), 3).tolist()} "
                  f"knobw {np.round(ctx.knob_world(), 3).tolist()}", flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    score = 1.0 if success else float(info.get("score", 0.0) or 0.0)
    result = {"task_score": score, "success": success, "steps": steps, "variant": os.environ.get("DEMO_VARIANT", ""),
              "tyre_bar": round(task.tyre_bar(), 3),
              "worst_ik_mm": {"left": round(ctx.left.worst_mm, 2), "right": round(ctx.right.worst_mm, 2)}}
    for w in writers.values():
        w.close()
    (out / "result.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result), flush=True)
    return result


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=1800)
    ap.add_argument("--no-record", action="store_true")
    a = ap.parse_args()
    run(a.task, a.out, a.max_steps, record=not a.no_record)
