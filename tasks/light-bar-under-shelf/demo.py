"""Scripted full-score demo for the light bar task, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 1200] [--no-record]

It plans from the fixed start layout (the task's constants) and, while it runs, reads only object poses a camera could
see (the monitor's and the light bar's) and the arms' own commands; never the scorer's state, contacts or forces.

1. Both arms grip the screen's side edges low down, from outside, and slide the monitor 14 cm toward the robot, out
   from under the shelf; they let go.
2. Both arms pinch the bar 8 cm either side of the clip, lift it and turn it a quarter turn about its length so the
   clip hangs counterweight-down, carry it over the measured top edge of the monitor, lower the clip onto the edge
   (front lip in front, counterweight behind) and let go.
3. Both arms grip the side edges again and slide the monitor back to where it started (its start pose, as the first
   camera frame shows it), let go and move clear.

Writes demo.mp4 (the three camera views side by side), setup.png (the first frames), actions.jsonl (the 14 joint
commands sent at each step) and result.json to the output folder.

Variants that must not score 1 (DEMO_VARIANT=...): nothing (both arms stay still), naive (hang the bar on the
monitor where it stands, under the shelf), no_return (fit the bar on the pulled-out monitor and leave it there).
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
PULL = 0.14            # how far the monitor is slid out toward the robot (m)
GRIP_Z = 0.105         # side-edge grip height above the desk (screen bottom edge at 0.089)
SIDE_PITCH = 20.0      # side grips lean 20 degrees down (a flatter hand stays clear of the shelf board)
PICK_PITCH = -20.0     # the bar is picked with the gripper leaning 20 degrees back toward the robot ...
PLACE_PITCH = 70.0     # ... and hung with it leaning 70 degrees forward: a quarter turn of the bar about its length
BAR_GRIP_Y = 0.08      # each hand pinches the bar 8 cm from the clip's middle


def _aam(axis, angle):
    from yam_arena.urdf import axis_angle_matrix

    return axis_angle_matrix(np.asarray(axis, float), angle)


def down(yaw_deg: float, pitch_deg: float = 0.0) -> np.ndarray:
    """A downward grip turned ``yaw`` about the vertical, then leaned ``pitch`` degrees forward (toward +x)."""
    from yam_arena.demo import grip_rotation

    return _aam((0, 1.0, 0), -math.radians(pitch_deg)) @ grip_rotation(math.radians(yaw_deg))


def side(arm: str, pitch_deg: float = SIDE_PITCH) -> np.ndarray:
    """A grip coming in from outside toward the middle (left arm along -y), fingers closing along x, leaned down."""
    from yam_arena.demo import grip_rotation

    s = -1 if arm == "left" else 1
    r = _aam((1.0, 0, 0), s * math.pi / 2) @ grip_rotation(math.pi / 2)
    return _aam((1.0, 0, 0), -s * math.radians(pitch_deg)) @ r


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
    T = task
    variant = os.environ.get("DEMO_VARIANT", "")
    flags: dict = {}
    top = T.TOP
    face0 = T.MONITOR_X
    edge_y = T.SCREEN_W / 2 - 0.015              # grip 15 mm in from each side edge

    def edge(arm, face_x, dy=0.0):
        s = 1 if arm == "left" else -1
        return np.array([face_x + T.SCREEN_T / 2, s * (edge_y + dy), top + GRIP_Z])

    def slide(face_from, face_to, label):
        """Both arms: grip the side edges at ``face_from``, slide the monitor to ``face_to``, let go, back off."""
        def one(A):
            R = side(A.name)
            out = 0.07
            s = 1 if A.name == "left" else -1
            # come in low from the front, outside the screen's edge, then in
            yield from A.move(np.array([face_from - 0.10, s * (edge_y + out), top + GRIP_Z + 0.03]), R, 2.5, grip=1.0)
            yield from A.move(edge(A.name, face_from, out), R, 1.2)
            yield from A.move(edge(A.name, face_from), R, 1.2)
            yield from A.set_grip(0.0, 0.8)
            flags[f"{label}_{A.name}"] = True
            yield from wait_for(flags, f"{label}_" + ("right" if A.name == "left" else "left"))
            yield from wait(0.3)
            yield from A.move(edge(A.name, face_to), R, 3.0 + 10 * abs(face_to - face_from))
            yield from wait(0.3)
            yield from A.set_grip(1.0, 0.6)
            yield from A.move(edge(A.name, face_to, out), R, 1.0)
        yield from both(one(ctx.left), one(ctx.right))

    def slide_to(face_to, label, tol=0.005, tries=3):
        """Slide until the monitor's face, measured from its pose, is within ``tol`` of ``face_to``."""
        for k in range(tries):
            face = float(ctx.states()["monitor"].pos[0])
            print(f"{label} {k}: monitor face at x {face:.4f}, target {face_to:.3f}", flush=True)
            if abs(face - face_to) < tol:
                return
            yield from slide(face, face_to, f"{label}{k}")

    def park(A):
        R = down(0.0)
        s = 1 if A.name == "left" else -1
        yield from A.move(np.array([0.18, s * 0.30, top + 0.20]), R, 2.0)

    def hang(face_x, face_y=0.0):
        """Both arms: pinch the bar either side of the clip, turn it clip-down, hang it on the edge of a screen whose
        face is at ``face_x``, let go."""
        bar = ctx.states()["light_bar"]
        # the bar's axis, measured from the bar's pose (it lies on its back: axis 17 mm beyond the clip and 6 cm up)
        axis = bar.to_world(T.BAR_C)
        target_x = face_x - 0.0005                  # clip origin: lip 1.5 mm in front of the face, counterweight 3.5 mm behind
        hang_x = target_x + T.BAR_C[0]               # where the bar's axis goes
        edge_z = top + T.SCREEN_TOP
        def one(A):
            s = 1 if A.name == "left" else -1
            yaw = -90.0 * s                          # fingers close along x, across the bar (each arm's reachable way)
            gy = s * BAR_GRIP_Y
            p = np.array([axis[0], bar.pos[1] + gy, axis[2]])
            yield from A.move(p + [-0.03, 0, 0.08], down(yaw, PICK_PITCH), 3.0, grip=1.0)
            yield from A.move(p, down(yaw, PICK_PITCH), 1.5)
            yield from A.set_grip(0.0, 1.0)
            yield from wait(0.3)
            yield from A.move(p + [0, 0, 0.10], down(yaw, PICK_PITCH), 1.5)
            # lift and turn the bar a quarter turn about its length, clip and counterweight down
            above = np.array([hang_x - 0.06, face_y + gy, edge_z + T.BAR_C[2] + 0.095])
            yield from A.move(above, down(yaw, PLACE_PITCH), 4.0)
            yield from A.move(above + [0.06, 0, 0], down(yaw, PLACE_PITCH), 2.0)
            # Look at the bar while it hovers over the edge and shift both hands so the clip's middle is over the
            # edge (the hands sag and the bar turns a little in the fingers, so the commanded pose is a few mm off).
            for k in range(2):
                flags[f"hover{k}_{A.name}"] = True
                yield from wait_for(flags, f"hover{k}_" + ("right" if A.name == "left" else "left"))
                yield from wait(0.5)
                clip = ctx.states()["light_bar"].pos           # the clip's origin: under its top plate, at the lip
                err = np.array([target_x - clip[0], face_y - clip[1], 0.0])
                print(f"{A.name} hover {k}: clip at {np.round(clip, 4).tolist()}, shift {np.round(err * 1000, 1).tolist()} mm",
                      flush=True)
                yield from A.move(A.pos + err, down(yaw, PLACE_PITCH), 1.0)
            clip = ctx.states()["light_bar"].pos
            yield from A.move(A.pos + [0, 0, edge_z + 0.003 - clip[2]], down(yaw, PLACE_PITCH), 3.0)
            yield from wait(0.5)
            yield from A.set_grip(1.0, 1.0)
            yield from wait(0.3)
            a = down(yaw, PLACE_PITCH)[:, 2]         # back out along the grip, then away from the bar
            yield from A.move(A.pos - 0.05 * a, down(yaw, PLACE_PITCH), 1.5)
            yield from A.move(A.pos + [-0.06, s * 0.05, 0.0], down(yaw, PLACE_PITCH), 1.5)
            yield from park(A)

        yield from both(one(ctx.left), one(ctx.right))

    if variant == "nothing":
        for _ in range(round(60 * CONTROL_HZ)):
            yield
        return
    if variant == "naive":
        yield from hang(face0)
        yield from wait(3.0)
        return
    yield from slide_to(face0 - PULL, "out", tol=0.03)
    yield from both(park(ctx.left), park(ctx.right))
    mon = ctx.states()["monitor"]
    face = float(mon.pos[0])                     # where the pulled monitor's face really is (it is in plain view)
    print(f"monitor face after pulling: x {face:.4f} y {mon.pos[1]:.4f} (start {face0:.3f})", flush=True)
    yield from hang(face, float(mon.pos[1]))
    if variant == "no_return":
        yield from wait(3.0)
        return
    yield from slide_to(face0, "back")
    yield from both(park(ctx.left), park(ctx.right))


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
        cam_height: int = 176, hold_s: float = 2.0) -> dict:
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
                print(f"step {steps} score {res.info.get('score')} monitor {np.round(s['monitor'].pos, 4).tolist()} "
                      f"bar {np.round(s['light_bar'].pos, 4).tolist()} shelf {np.round(s['shelf'].pos, 4).tolist()} "
                      f"tcp_l {np.round(ctx.left.pos, 3).tolist()} tcp_r {np.round(ctx.right.pos, 3).tolist()}"
                      f"{_actual(emb) if debug else ''}", flush=True)
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
