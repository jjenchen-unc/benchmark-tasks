"""Scripted full-score demo, run in the yam_arena embodiment with the task file and the task_score scorer.

    python demo.py task.py demo_out [--no-record]

Open loop from the fixed published layout. The joint targets below are inverse-kinematics solutions for the left
arm at the fixed knob and handle positions; the demo reads nothing from the simulator while it runs (no part
poses, flags, forces or parameters). It turns the knob DEMO_TURNS turns clockwise (default 1.8, the setting the
README's grip-length table shows gives a gap under 1 mm), in regrasps of at most 150 degrees of wrist turn, then
turns the handle 180 degrees clockwise to its locked stop, lets go, moves the arm away and waits.

DEMO_VARIANT: nothing (both arms stay still), no_adjust (lock without touching the knob: grip too long),
force (turn the knob DEMO_TURNS turns, default 3, then force the handle: grip too short), keep_holding (the full
demo but the gripper keeps holding the handle), press (push the lid down next to the latch and keep pressing).
Writes demo.mp4 (three cameras side by side), setup.png, actions.jsonl and result.json.
"""
import argparse, json, math, os, sys
from pathlib import Path
import numpy as np

HZ = 20
READY = [0., 1., 1., -.6, 0., 0.]
KNOB_ABOVE = [-.49, 2.316, 2.036, -1.291, 0., -1.625]
KNOB_AT = [-.49, 2.342, 1.9, -1.129, 0., -1.625]
KNOB_UP = [-.49, 2.315, 2.094, -1.35, 0.]
HANDLE_ABOVE = [-.546, 1.945, 1.512, -1.138, 0., -1.855]
HANDLE_AT = [-.546, 1.992, 1.393, -.972, 0., -1.855]
HANDLE_UP = [-.546, 1.938, 1.541, -1.173, 0.]
PARK = [-.35, 1.489, 1.307, -1.388, 0., 1.482]
PRESS = [-.546, 2.05, 1.30, -.93, 0., -1.855]
WRIST_SPAN = 2.618       # largest wrist turn per regrasp (rad)


class Left:
    def __init__(self):
        self.q = np.array(READY + [1.])

    def to(self, target, n, grip=None):
        q0 = self.q.copy(); q1 = q0.copy(); q1[:len(target)] = target
        if grip is not None: q1[6] = grip
        for k in range(1, n + 1):
            s = .5 - .5 * math.cos(math.pi * k / n)
            self.q = q0 + s * (q1 - q0)
            yield


def wait(n):
    for _ in range(n): yield


def script(arm, variant):
    if variant == 'nothing':
        yield from wait(10 ** 6)
    if variant == 'press':
        yield from arm.to(HANDLE_ABOVE, 50, grip=0.)
        yield from arm.to(PRESS, 40)
        yield from wait(10 ** 6)
    turns = float(os.environ.get('DEMO_TURNS', 3. if variant == 'force' else 0. if variant == 'no_adjust' else 1.8))
    total = 2 * math.pi * turns
    cycles = math.ceil(total / WRIST_SPAN - 1e-9)
    for _ in range(cycles):
        start = KNOB_ABOVE[5]
        yield from arm.to(KNOB_ABOVE, 60)
        yield from arm.to(KNOB_AT, 30)
        yield from arm.to(KNOB_AT, 30, grip=0.)
        yield from arm.to(KNOB_AT[:5] + [start + total / cycles], 50)
        yield from wait(10)
        yield from arm.to(arm.q[:6], 15, grip=1.)
        yield from arm.to(KNOB_UP, 30)
    yield from arm.to(HANDLE_ABOVE, 50)
    yield from arm.to(HANDLE_AT, 30)
    yield from arm.to(HANDLE_AT, 30, grip=0.)
    yield from arm.to(HANDLE_AT[:5] + [HANDLE_AT[5] + math.pi], 120)
    yield from wait(40)
    if variant == 'keep_holding':
        yield from wait(10 ** 6)
    yield from arm.to(arm.q[:6], 15, grip=1.)
    yield from arm.to(HANDLE_UP, 30)
    yield from arm.to(PARK, 50)
    yield from wait(10 ** 6)


def run(task_path, out, max_steps=3848, record=True, emb=None, on_step=None):
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from yam_arena.inspect_embodiment import yam_arena_embodiment
    from types import SimpleNamespace
    from yam_arena.scoring import task_score
    out = Path(out); out.mkdir(parents=True, exist_ok=True)
    if emb is None:
        size = (640, 360) if record else (320, 176)
        emb = yam_arena_embodiment(task=str(Path(task_path).resolve()), device='cpu', object_jitter=0, control_hz=HZ, cam_width=size[0], cam_height=size[1])
    obs = emb.reset(Scene(id='demo', instruction='demo'), seed=42)
    variant = os.environ.get('DEMO_VARIANT', '')
    arm = Left(); gen = script(arm, variant); right = np.array(READY + [1.])
    frames = {}

    def snap(o):
        if record:
            for c, im in o.images.items(): frames.setdefault(c, []).append(np.asarray(im))
    snap(obs)
    steps = 0
    with (out / 'actions.jsonl').open('w') as log:
        while steps < max_steps:
            next(gen)
            a = np.r_[arm.q, right]
            res = emb.step(Action(data=a)); steps += 1
            log.write(json.dumps({'step': steps, 'action': a.round(6).tolist()}) + '\n')
            snap(res.observation)
            if on_step: on_step(steps, res)
            if steps % 200 == 0: print(f"step {steps} score {res.info.get('score')}", flush=True)
            if res.terminated: break
    s = task_score()(SimpleNamespace(steps=[SimpleNamespace(result=res)]), None)
    print(f'task_score: {s.value:g} ({s.explanation}) after {steps} steps', flush=True)
    result = {'task_score': s.value, 'explanation': s.explanation, 'steps': steps, 'variant': variant, 'turns': os.environ.get('DEMO_TURNS', '')}
    if record:
        import imageio
        cams = sorted(frames, key=lambda c: ('left' not in c, 'top' not in c))
        imageio.mimsave(out / 'demo.mp4', [np.hstack([frames[c][i] for c in cams]) for i in range(len(frames[cams[0]]))], fps=HZ, codec='libx264', quality=None, macro_block_size=8, output_params=['-pix_fmt', 'yuv420p', '-movflags', '+faststart', '-crf', '23'])
        imageio.imwrite(out / 'setup.png', np.hstack([frames[c][0] for c in cams]))
    (out / 'result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result), flush=True)
    return result, emb


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('task'); ap.add_argument('out')
    ap.add_argument('--max-steps', type=int, default=3848); ap.add_argument('--no-record', action='store_true')
    a = ap.parse_args()
    run(a.task, a.out, a.max_steps, record=not a.no_record)
