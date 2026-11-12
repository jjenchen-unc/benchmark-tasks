"""Public-controller stacking episode for B4_bars:offset_stack (native dual Panda).

Policy inputs (all public): the declared fixed starting layout below (part of
the task setup / screenshot, identical for every seed), the instruction text,
and MotionController.observe() robot state: tool pose, joint angles, jaw gap,
wrist force/torque and its user tare, plus sim time. The hidden centre of mass
of each bar is identified in-task from the tared wrist torque while the bar is
lifted at its middle (offset = -tau_y / F_z). No object pose, parameter, model
field, seed or evaluator state is read by the policy. Actions go through the
public 20 Hz pose tools with 8 mm/tick, joint torque, finger, 30 N guard and
80 N/0.1 s abort enforced by motion.py; the tick budget is the task's 1896.
Evaluator status (score, partial credit) is read only after the episode ends.

This is a scripted demo, NOT a model run and NOT YAM/Dream-1.
"""
from __future__ import annotations

import argparse, hashlib, json, math, os, struct, sys, time, zlib
from pathlib import Path

import numpy as np

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG / 'runtime'))
from robocurve_native.task import NativeTask, write_json          # noqa: E402
from robocurve_native.motion import MotionController              # noqa: E402

MODULE = PKG / 'scene/B4_bars/benches/B4_bars.py'
# Declared fixed layout (metres, world frame; table top z = 0.800). Same as the setup text/screenshot.
BAR_Y = dict(A=-.30, B=-.19, C=-.08)       # bar centres at x = 0, lying along X, cap at -X
BAR_CZ = .825                              # bar centre height on the table (50 mm tube)
SUPPORT_XY = (0., .18)
SUPPORT_TOP = dict(stack_ABC=.960, stack_CBA=.960, A_stack_pedestal=.9524)   # declared heights 160/160/152.4 mm
ORDER = dict(stack_ABC='ABC', stack_CBA='CBA', A_stack_pedestal='ABC')
BAR_A = .05
GZ = .010                                  # tip 10 mm above the bar centre line when grasping
CARRY_Z = 1.15
YAW = 90.                                  # jaw axis along world Y, across the bars
OPEN, GRIP = .090, .024                    # total jaw gap commands (m)
PARK_L = (-.20, -.05, 1.15)
PARK_R = (.33, .22, 1.10)


class Policy:
    def __init__(self, C, log):
        self.C, self.log = C, log

    def obs(self):
        return self.C.observe()

    def tip(self, arm=0):
        return np.asarray(self.obs()['state']['eef_state'][7 * arm:7 * arm + 3])

    def go(self, xyz, jaw=None, arm='left', extra=6, step=.008, guard=30.):
        a = 0 if arm == 'left' else 1
        cur = np.asarray(self.C.positions[a])          # last commanded pose (public controller state)
        n = int(math.ceil(np.linalg.norm(np.asarray(xyz) - cur) / step)) + extra
        kw = dict(arm=arm, position=list(xyz), ticks=max(1, n), guard_n=guard)
        if arm == 'left':
            kw['yaw_deg'] = YAW
        if jaw is not None:
            kw['jaw_gap_m'] = jaw
        r = self.C.move(**kw)
        self.log.append(dict(cmd='move', arm=arm, xyz=list(map(float, xyz)), jaw=jaw, **r))
        return r

    def slow(self, xyz, mm_per_tick=1.5, guard=30.):
        cur = np.asarray(self.C.positions[0]); goal = np.asarray(xyz, float)
        n = max(1, int(math.ceil(np.linalg.norm(goal - cur) / (mm_per_tick / 1000))))
        for i in range(1, n + 1):
            r = self.C.move(arm='left', position=list(cur + (goal - cur) * i / n), yaw_deg=YAW, ticks=1, guard_n=guard)
            if r['stop_reason']:
                self.log.append(dict(cmd='slow', stopped_at=i, of=n, **r)); return r
        self.log.append(dict(cmd='slow', xyz=list(map(float, goal)), ticks=n)); return r

    def wait(self, n):
        r = self.C.wait(ticks=n); self.log.append(dict(cmd='wait', n=n, **r)); return r

    def identify(self, samples=20):
        w = []
        for _ in range(samples):
            self.C.wait(ticks=1)
            w.append(np.asarray(self.obs()['state']['tared_wrist_wrench'])[0])
        w = np.mean(w, 0); fz, ty = float(w[2]), float(w[4])
        off = -ty / fz if fz > 1. else 0.
        return off, fz / 9.81

    def pick(self, bar, first):
        x, y = 0., BAR_Y[bar]; gz = BAR_CZ + GZ
        self.go((x, y, CARRY_Z), OPEN, extra=10 if first else 6)
        self.go((x, y, gz + .05), OPEN)
        self.wait(10); self.C.tare()                    # hand free and still: tare the wrist sensor
        self.go((x, y, gz), OPEN, extra=8)
        self.go((x, y, gz), GRIP, extra=15)              # close (finger filter)
        self.go((x, y, gz + .10), GRIP, extra=10)
        self.wait(20)
        off, mass = self.identify()
        self.log.append(dict(cmd='identify', bar=bar, com_offset_x_m=off, mass_kg=mass))
        self.go((x, y, CARRY_Z), GRIP)
        return off

    def place(self, level, off, top, release=True):
        tx, ty = SUPPORT_XY[0] - off, SUPPORT_XY[1]
        zp = top + level * BAR_A + BAR_A / 2 + GZ
        self.go((tx, ty, CARRY_Z), GRIP)
        self.go((tx, ty, zp + .012), GRIP, extra=8)
        self.slow((tx, ty, zp + .001))
        self.wait(8)
        if not release:
            return
        self.release_start = self.C.observe()['time']     # jaw-open command issued: hands-off hold cannot start earlier
        self.go((tx, ty, zp + .001), OPEN, extra=12)
        self.slow((tx, ty, zp + .02), mm_per_tick=3.)
        self.go((tx, ty, min(zp + .08, 1.175)), OPEN)
        self.go((tx, ty, CARRY_Z), OPEN)


def png(frame, path):
    h, w, _ = frame.shape
    chunk = lambda t, b: struct.pack('>I', len(b)) + t + b + struct.pack('>I', zlib.crc32(t + b) & 0xffffffff)
    raw = b''.join(b'\x00' + row.tobytes() for row in frame)
    Path(path).write_bytes(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
                           + chunk(b'IDAT', zlib.compress(raw, 6)) + chunk(b'IEND', b''))


def run(target, setting, seed, variant, out):
    t0 = time.monotonic()
    task = NativeTask(MODULE, seed=seed, target=target, setting=setting)
    C = MotionController(task)
    log = []; P = Policy(C, log)
    frames = {}
    def snap(name):
        img = task.render(width=640, height=360); frames[name] = img
        if out: png(img, out / f'{name}.png')
    snap('00_setup')
    order = ORDER[target]
    if variant == 'wrong_order':
        order = order[1] + order[0] + order[2]
    nbars = dict(one_bar=1, two_bar=2).get(variant, 3)
    top = SUPPORT_TOP[target]
    P.go(PARK_R, arm='right')
    release_time = None
    for j, bar in enumerate(order[:nbars]):
        off = P.pick(bar, j == 0)
        if variant == 'blind_centre':
            off = 0.
        last = j == nbars - 1
        P.place(j, off, top, release=not (variant == 'hold_on' and last))
        snap(f'{j + 1:02d}_placed_{bar}')
        release_time = C.observe()['time']
    if variant != 'hold_on':
        P.go(PARK_L, OPEN)
    # truncate: cut 19 s after the last jaw-open command (an upper bound on hold onset), so < 20 s hands-off is possible
    hold_until = getattr(P, 'release_start', release_time) + 19.0 if variant == 'truncate' else release_time + 20.6
    while C.observe()['time'] < hold_until and not C.reason:
        P.wait(min(40, max(1, int(math.ceil((hold_until - C.observe()['time']) / .05)))))
    snap('99_final')
    status = task.status()
    result = dict(target=target, setting=setting, seed=seed, variant=variant,
                  evidence='public scripted controller demo (declared layout + wrist F/T); not a model run; native dual Panda, not YAM/Dream-1',
                  status=status, ticks_used=C.ticks, tick_budget=C.max_ticks, termination=C.reason,
                  last_release_time_s=release_time, last_release_command_time_s=getattr(P, 'release_start', None), metrics=C.metrics, wall_seconds=time.monotonic() - t0,
                  frames={k: dict(mean=float(v.mean()), ptp=int(np.ptp(v)),
                                  sha256=hashlib.sha256(v.tobytes()).hexdigest()) for k, v in frames.items()},
                  log=log)
    task.close()
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--target', default='stack_ABC', choices=list(ORDER))
    ap.add_argument('--setting', default=None)
    ap.add_argument('--seed', type=int, default=42)
    ap.add_argument('--variant', default='full', choices=['full', 'one_bar', 'two_bar', 'wrong_order', 'hold_on', 'truncate', 'blind_centre'])
    ap.add_argument('--out', type=Path)
    a = ap.parse_args()
    setting = a.setting or ('A' if a.target == 'A_stack_pedestal' else 'S')
    out = a.out / f'{a.target}-{setting}-{a.seed}-{a.variant}' if a.out else None
    if out: out.mkdir(parents=True, exist_ok=True)
    r = run(a.target, setting, a.seed, a.variant, out)
    text = write_json(r, out / 'result.json' if out else None)
    s = r['status']
    print(json.dumps(dict(target=a.target, setting=setting, seed=a.seed, variant=a.variant, score=s['score'],
                          binary=s.get('binary_source_score'), consistent=s.get('score_consistent_with_source'),
                          partial=s.get('partial_credit', {}).get('stage_hold_s'), ticks=r['ticks_used'],
                          term=r['termination'], wall=round(r['wall_seconds'], 1))))


if __name__ == '__main__':
    main()
