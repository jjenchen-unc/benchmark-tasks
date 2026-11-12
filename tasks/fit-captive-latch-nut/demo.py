"""Scripted full-score demo, run in the yam_arena embodiment with the task file and the task_score scorer.

    python demo.py task.py OUT [--no-record]

The left arm acts only through the 14 joint commands. It plans from the fixed published layout (fixture.json)
and, while it runs, reads only what a camera or the robot itself can see: the robot's own joint positions and
gripper pose, the wrench poses and the nut's turn from its paint mark. It never reads the clamp or rounding state,
contacts, forces or physical parameters. It goes straight to the 1/2 in wrench (README, "How the hidden property
can be observed", shows that a trial fit tells the wrenches apart), loosens the nut in 30 degree strokes with a
refit after each, puts the wrench back on its rack blocks, swings the arm to its stop by the knob, moves clear and
waits.

DEMO_VARIANT: nothing (hold the start posture), leave (the full demo, but the wrench is put down on the table
instead of the rack), wrong (the same motions with the 13 mm wrench), push_arm (swing the arm by its knob without
touching the nut). Writes demo.mp4 (three cameras side by side), setup.png, actions.jsonl and result.json.
"""
import argparse, json, math, os, subprocess, sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
F = json.loads((HERE / 'fixture.json').read_text())
RUN_LIMIT = 3870
NUT = np.array([.12, -.02, .894])     # nut axis on the fixture (fixture frame)
TURN_DONE = 63.                       # stop loosening once the paint mark has turned this far (degrees)
OPEN_TARGET = math.radians(94.5)      # knob is driven a little past the 90 degree stop


def arr(x):
    if hasattr(x, 'torch'): x = x.torch
    return x.detach().cpu().numpy().copy() if hasattr(x, 'detach') else np.asarray(x).copy()


class Done(Exception):
    pass


class Controller:
    def __init__(self, emb, log, max_steps=RUN_LIMIT):
        self.emb = emb; self.scene = emb._base.scene; self.log = log; self.max_steps = max_steps; self.steps = 0
        self.R = np.asarray(F['rotation']); self.T = np.asarray(F['translation']) + arr(self.scene.env_origins)[0]
        self.cmd = np.array(emb._observation().state['joint_pos'], dtype=float, copy=True)
        fixture = self.scene['wrench_fixture']; self.head = fixture.body_names.index('latch_head')
        self.head0 = self.head_rotation()
        self.payload_bias = np.zeros(3); self.payload_rotation_bias = np.zeros(3)
        self.continuation_grasp = None; self.continuation_active = False
        self.result = None

    def head_rotation(self):
        s = arr(self.scene['wrench_fixture'].data.body_link_state_w)[0, self.head]
        return Rotation.from_quat(s[3:7]).as_matrix()

    def nut_turn(self):
        r = self.head_rotation() @ self.head0.T
        return math.degrees(math.atan2(r[1, 0], r[0, 0]))

    def action(self, label):
        from inspect_robots.types import Action
        if self.steps >= self.max_steps: raise Done('step limit')
        self.result = self.emb.step(Action(data=self.cmd.copy())); self.steps += 1
        self.log(self.steps, label, self.cmd)
        if self.result.terminated: raise Done(self.result.termination_reason)

    def hand(self):
        from yam_arena.constants import TCP_OFFSET
        a = self.scene['left_robot']; s = arr(a.data.body_link_state_w)[0, a.body_names.index('gripper')]
        R = Rotation.from_quat(s[3:7]).as_matrix(); return s[:3] + R @ TCP_OFFSET, R

    def tool(self, name):
        s = arr(self.scene[name].data.root_link_state_w)[0]
        return s[:3].copy(), Rotation.from_quat(s[3:7]).as_matrix()

    def jaw(self, fraction, n=20):
        initial = self.cmd[6]
        for k in range(n): self.cmd[6] = initial + (fraction - initial) * (k + 1) / n; self.action('jaw')

    def wait(self, n, label='wait'):
        for _ in range(n): self.action(label)

    def move(self, p, R, n=60, label='move', payload=None):
        from yam_arena import ik
        p0, R0 = self.hand(); rv = Rotation.from_matrix(R @ R0.T).as_rotvec()
        robot = self.scene['left_robot']; q = arr(robot.data.joint_pos)[0, :6].copy()
        base = arr(robot.data.root_pos_w)[0].copy()
        for k in range(n + (20 if payload is not None and n > 1 else 0)):
            f = min((k + 1) / n, 1.); f = f * f * (3 - 2 * f); pt = p0 + (np.asarray(p) - p0) * f; Rt = Rotation.from_rotvec(rv * f).as_matrix() @ R0
            if payload is not None:
                name, offset, relative = payload; wanted = pt + Rt @ offset
                pt = pt + self.payload_bias
                desired_tool_R = Rt @ relative
                if label in ('fit_wrench', 'refit') or self.continuation_active:
                    Rt = Rotation.from_rotvec(self.payload_rotation_bias).as_matrix() @ Rt
            q, pe, re = ik.solve(pt, ik.matrix_to_quat_xyzw(Rt), q, base_pos=base, iters=160, damping=.015)
            if pe > .003 or re > .05: raise RuntimeError(f'{label}: inverse kinematics failed ({pe} m, {re} rad)')
            local = self.R.T @ (pt - self.T)
            if np.any(local < [-.30, -.50, .825]) or np.any(local > [.50, .65, 1.27]): raise RuntimeError(f'{label}: hand target outside the workspace')
            self.cmd[:6] = q; self.action(label)
            if payload is not None:
                # Servo the held wrench to its target from its seen pose (the grasp sags under load).
                actual, actualR = self.tool(name); error = wanted - actual
                self.payload_bias = np.clip(self.payload_bias + .25 * error, -.008, .008)
                rotation_error = Rotation.from_matrix(desired_tool_R @ actualR.T).as_rotvec()
                angle = float(np.linalg.norm(rotation_error))
                if label in ('fit_wrench', 'refit') or self.continuation_active:
                    delta = .25 * rotation_error; norm = float(np.linalg.norm(delta))
                    if norm > math.radians(.5): delta *= math.radians(.5) / norm
                    candidate = self.payload_rotation_bias + delta; norm = float(np.linalg.norm(candidate))
                    if norm > math.radians(7): candidate *= math.radians(7) / norm
                    self.payload_rotation_bias = candidate
                if k >= n - 1 and np.linalg.norm(error) <= .001 and angle <= .025: break
                if k == n + 19: raise RuntimeError(f'{label}: wrench did not reach its target ({np.linalg.norm(error)} m, {angle} rad)')

    def carry(self, name, p, R, n=60, label='carry'):
        ph, Rh = self.hand(); pk, Rk = self.tool(name)
        offset = Rh.T @ (pk - ph); relative = Rh.T @ Rk
        if self.continuation_grasp is not None: offset, relative = self.continuation_grasp
        handR = R @ relative.T
        self.move(np.asarray(p) - handR @ offset, handR, n, label, payload=(name, offset, relative))

    def refit_phase(self, name, N, deg):
        # Pick a hex-equivalent wrench angle whose next 30 degree stroke is reachable.
        from yam_arena import ik
        ph, Rh = self.hand(); pk, Rk = self.tool(name)
        offset = Rh.T @ (pk - ph); relative = Rh.T @ Rk
        robot = self.scene['left_robot']; base = arr(robot.data.root_pos_w)[0].copy()
        current = arr(robot.data.joint_pos)[0, :6].copy(); candidates = []
        for j in range(-4, 5):
            psi = deg + 60 * j
            if not -180 <= psi <= 180: continue
            q = current.copy(); cost = 0.; valid = True
            for delta in np.linspace(0., 30., 13):
                R = self.R @ Rotation.from_euler('z', psi + delta, degrees=True).as_matrix(); handR = R @ relative.T
                q, pe, re = ik.solve(N - handR @ offset, ik.matrix_to_quat_xyzw(handR), q, base_pos=base, iters=220, damping=.015)
                if pe > .003 or re > .05: valid = False; break
                cost += float(np.linalg.norm(q - current))
            if valid: candidates.append((cost, psi))
        if not candidates: raise RuntimeError('no reachable refit angle')
        return min(candidates)[1]

    def pick(self, name, lift=.12):
        from yam_arena import ik
        home, homeR = self.tool(name); grasp = home + homeR @ [0, -.10, 0]
        down = homeR @ np.diag([1., -1., -1.])
        robot = self.scene['left_robot']; base = arr(robot.data.root_pos_w)[0].copy(); q = arr(robot.data.joint_pos)[0, :6].copy()
        candidates = []
        for flip in (0., math.pi):
            r = down @ Rotation.from_euler('z', flip).as_matrix()
            qq, pe, re = ik.solve(grasp + [0, 0, .08], ik.matrix_to_quat_xyzw(r), q, base_pos=base, iters=220, damping=.015)
            if pe < .003 and re < .05: candidates.append((np.linalg.norm(qq - q), r))
        if not candidates: raise RuntimeError('no top grasp')
        down = min(candidates, key=lambda a: a[0])[1]
        self.jaw(.22); self.move(grasp + [0, 0, .08], down, 90, 'rack_approach')
        self.move(grasp, down, 60, 'handle_grasp'); self.jaw(0., 30)
        self.carry(name, home + [0, 0, lift], homeR, 70, 'extract')
        return home, homeR

    def fit(self, name, psi=-28.):
        N = self.R @ NUT + self.T
        targetR = self.R @ Rotation.from_euler('z', psi, degrees=True).as_matrix()
        A = N - targetR[:, 1] * .035
        self.carry(name, A + [0, 0, .055], targetR, 100, 'nut_approach')
        self.carry(name, A, targetR, 70, 'nut_lower')
        self.carry(name, N, targetR, 100, 'fit_wrench')
        return N, psi

    def loosen(self, name, N, psi):
        ph, Rh = self.hand(); pk, Rk = self.tool(name)
        self.continuation_grasp = (Rh.T @ (pk - ph), Rh.T @ Rk); self.continuation_active = True
        for stroke in range(4):
            if self.nut_turn() > TURN_DONE: break
            for k in range(60):
                R = self.R @ Rotation.from_euler('z', psi + 30 * (k + 1) / 60, degrees=True).as_matrix()
                self.carry(name, N, R, 1, 'loosen')
                if self.nut_turn() > TURN_DONE: break
            p, R = self.tool(name); off = N - R[:, 1] * .045
            self.carry(name, off, R, 50, 'withdraw')
            self.carry(name, off + [0, 0, .055], R, 50, 'withdraw_lift')
            if self.nut_turn() <= TURN_DONE:
                ph, Rh = self.hand(); pk, Rk = self.tool(name)
                self.continuation_grasp = (Rh.T @ (pk - ph), Rh.T @ Rk)
                self.payload_bias = np.zeros(3); self.payload_rotation_bias = np.zeros(3)
                psi = self.refit_phase(name, N, self.nut_turn() + 32.)
                targetR = self.R @ Rotation.from_euler('z', psi, degrees=True).as_matrix(); A = N - targetR[:, 1] * .035
                self.carry(name, A + [0, 0, .055], targetR, 70, 'refit_above')
                self.carry(name, A, targetR, 50, 'refit_lower'); self.carry(name, N, targetR, 70, 'refit')
        if self.nut_turn() <= TURN_DONE: raise RuntimeError('nut not turned far enough')

    def put_down(self, name, p, R, label):
        self.carry(name, p + [0, 0, .12], R, 100, label + '_above')
        self.carry(name, p + [0, 0, .0008], R, 70, label); self.jaw(.22)
        self.continuation_grasp = None; self.continuation_active = False
        p, R = self.hand(); self.move(p + [0, 0, .08], R, 60, 'tool_clear')

    def open_arm(self):
        def knob(a): return self.R @ np.array([.12 - .08 * math.sin(a), -.02 + .08 * math.cos(a), .9197]) + self.T
        down = self.R @ np.diag([1., -1., -1.]); self.jaw(.4)
        self.move(knob(0) + [0, 0, .06], down, 90, 'knob_approach'); self.move(knob(0), down, 60, 'knob_grasp'); self.jaw(0., 30)
        for k in range(150): self.move(knob(-OPEN_TARGET * (k + 1) / 150), down, 1, 'open_arm')
        self.wait(10)
        self.jaw(.4); p, R = self.hand(); self.move(p + [0, 0, .08], R, 60, 'arm_clear')

    def run(self, variant=''):
        if variant == 'nothing': self.wait(10 ** 6)
        if variant != 'push_arm':
            name = 'w13' if variant == 'wrong' else 'w1_2'
            home, homeR = self.pick(name)
            N, psi = self.fit(name)
            self.loosen(name, N, psi)
            if variant == 'leave': self.put_down(name, self.R @ np.array([.10, -.12, .8295]) + self.T, homeR, 'table')
            else: self.put_down(name, home, homeR, 'return_rack')
        self.open_arm()
        self.wait(10 ** 6, 'hands_off')


class Video:
    def __init__(self, path, frame):
        import imageio_ffmpeg
        h, w, _ = frame.shape
        self.p = subprocess.Popen([imageio_ffmpeg.get_ffmpeg_exe(), '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{w}x{h}', '-r', '20', '-i', 'pipe:0',
                                   '-an', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '23', '-movflags', '+faststart', str(path)], stdin=subprocess.PIPE)
        self.add(frame)

    def add(self, frame): self.p.stdin.write(np.ascontiguousarray(frame).tobytes())

    def close(self): self.p.stdin.close(); return self.p.wait()


def side_by_side(images):
    return np.hstack([images[c] for c in ('left_cam', 'top_cam', 'right_cam')])


def run(task_path, out, max_steps=RUN_LIMIT, record=True, variant=None):
    from inspect_robots import Scene
    from types import SimpleNamespace
    from yam_arena.inspect_embodiment import yam_arena_embodiment
    from yam_arena.scoring import task_score
    out = Path(out); out.mkdir(parents=True, exist_ok=True)
    variant = os.environ.get('DEMO_VARIANT', '') if variant is None else variant
    size = (640, 360) if record else (320, 176)
    emb = yam_arena_embodiment(task=str(Path(task_path).resolve()), device='cpu', object_jitter=0, control_hz=20, cam_width=size[0], cam_height=size[1])
    obs = emb.reset(Scene(id='demo', instruction='demo'), seed=42)
    video = None
    if record:
        import imageio
        imageio.imwrite(out / 'setup.png', side_by_side(obs.images)); video = Video(out / 'demo.mp4', side_by_side(obs.images))
    log = (out / 'actions.jsonl').open('w'); ctl = None

    def on_step(step, label, cmd):
        r = ctl.result
        log.write(json.dumps({'step': step, 'phase': label, 'action': cmd.round(6).tolist()}) + '\n')
        if video: video.add(side_by_side(r.observation.images))
        if step % 200 == 0: print(f'step {step} {label} score {r.info.get("score")}', flush=True)
    ctl = Controller(emb, on_step, max_steps)
    stop = None
    try: ctl.run(variant)
    except Done as e: stop = str(e)
    except RuntimeError as e:
        stop = 'controller: ' + str(e); print(stop, flush=True)
        if variant:
            # Variants keep going after a failed motion: let go, move up and wait out the run limit.
            try:
                ctl.continuation_active = False; ctl.jaw(.4); p, R = ctl.hand(); ctl.move(p + [0, 0, .06], R, 40, 'clear'); ctl.wait(10 ** 6)
            except (Done, RuntimeError) as e2: stop += '; then ' + str(e2)
    log.close()
    res = ctl.result
    s = task_score()(SimpleNamespace(steps=[SimpleNamespace(result=res)]), None)
    print(f'task_score: {s.value:g} ({s.explanation}) after {ctl.steps} steps', flush=True)
    status = res.info.get('latch', {})
    result = {'task_score': s.value, 'explanation': s.explanation, 'steps': ctl.steps, 'variant': variant, 'stop': stop,
              'final_measurement': status.get('measurement'), 'hold_full_s': status.get('hold_full_s'), 'hold_release_s': status.get('hold_release_s'), 'events': status.get('events')}
    if video: result['video_exit'] = video.close()
    (out / 'result.json').write_text(json.dumps(result, indent=2, default=float))
    print(json.dumps(result, default=float), flush=True)
    return result, emb


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('task'); ap.add_argument('out')
    ap.add_argument('--max-steps', type=int, default=RUN_LIMIT); ap.add_argument('--no-record', action='store_true')
    a = ap.parse_args()
    run(a.task, a.out, a.max_steps, record=not a.no_record)
