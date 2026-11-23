"""Scripted full-score demo (and negative variants) for the triple-beam task in the yam_arena embodiment.

    python demo.py task.py OUT [--max-steps N] [--no-record] [--stage demo|<variant>[@seed][,...]] [--seed N|none]

Acts ONLY through the 14 joint/gripper commands of ``YamArenaEmbodiment.step``. It reads what an operator sees on the
bench through ``balkit.bench_state()`` (read-only): the poses of the balance parts and the jar, the pointer against its
scale (and its speed), and where each rider sits on its beam (read against the beam's graduations). It never reads the
jar's mass, never writes a pose, a joint state or a score. Its joint commands are rate-limited below the CLI's
per-step delta guard and clipped to the URDF joint limits.

Weighing (right hand; the left arm stays at READY):
1. 500 g rider: bisect over its notches. Each try: pinch the rider's handle across its beam, slide it along the beam to
   the notch, let go and lift the hand clear, then watch which way the free beam swings (pointer up: the pan is still
   heavier; pointer down: the riders are). Keep the last notch that leaves the pan heavier.
2. 100 g rider: the same over its ten notches.
3. 10 g poise: slide it to mid-scale, let go, let the damped beam settle, read the pointer (5.1 mm per gram off
   balance while it is between its stops) and slide the poise by the amount read; repeat until the pointer settles
   on the zero mark.
4. Hand back to READY, clear of the balance, while the beam comes to rest.
The joint PD sags under load, so the arm adds the remaining joint error back to its commands (slow integral on its
own joint readings; pattern reused from earlier demos).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "src"))

import fastik  # noqa: E402
import layout as L  # noqa: E402

CONTROL_HZ = 20
DT = 1.0 / CONTROL_HZ
FINGER_OPEN = 0.04695  # m per finger at grip 1.0
COMP_GAIN = 0.08
RATE = 0.8  # fraction of the CLI delta guard (0.05 x joint range per step) the script may use
PAD_TIP = 0.013  # pad tips past the TCP
TIP_Z = L.HANDLE_Z0 + 0.003  # pad tips this high above the fulcrum axis when pinching a handle (4 mm over the beam)
SQUEEZE = 0.002  # fingers commanded this far inside the handle's faces: about 10 N per finger
OPEN_CLEAR = 0.009  # finger half-opening beyond the handle's half-width when open
LIFT = 0.035  # hand lifted this far above the pinch height while the beam swings
MM_PER_G = 1e-3 * L.G * L.A_PAN / L.K_SENS * L.L_POINTER * 1e3  # pointer mm per gram out of balance (5.1)
STOP_MM = L.THETA_STOP * L.L_POINTER * 1e3


def grip_of(half_opening_m):
    return float(np.clip(half_opening_m / FINGER_OPEN, 0.0, 1.0))


class Stop(Exception):
    pass


class Arm:
    """One arm: smooth Cartesian moves (URDF IK), each starting where the last command left it."""

    def __init__(self, name, base_pos):
        from yam_arena.constants import READY

        self.name, self.base = name, np.asarray(base_pos, float)
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = fastik.tcp_pose(self.q, self.base)
        self.offset = np.zeros(6)
        self.measured = self.q.copy()
        self.measured_grip = 1.0
        self.comp = True
        self.comp_band = None
        self.label = ""
        self.worst_mm = 0.0
        _, _, lo, hi, _ = fastik._chain()
        self.lo, self.hi = lo, hi
        self.last_cmd = None

    @property
    def cmd(self):
        c = np.r_[np.clip(self.q + self.offset, self.lo, self.hi), self.grip]
        lim = np.r_[RATE * 0.05 * (self.hi - self.lo), RATE * 0.05]
        if self.last_cmd is not None:
            c = np.clip(c, self.last_cmd - lim, self.last_cmd + lim)
        return c

    def commit(self, c):
        self.last_cmd = np.asarray(c, float).copy()

    def set_comp(self, mode):
        """True: joint integral (free moves and carrying); 'hold': offsets frozen (in contact); False: offsets decay."""
        self.comp = mode

    def observe(self, q):
        self.measured = np.asarray(q[:6], float)
        self.measured_grip = float(q[6])
        if self.comp == "hold":
            pass
        elif self.comp:
            self.offset = np.clip(
                self.offset + COMP_GAIN * (self.q - self.measured), -0.25, 0.25
            )
            if self.comp_band is not None:  # in contact: integral kept within a band (bounds the push force)
                self.offset = np.clip(self.offset, self.comp_band[0], self.comp_band[1])
        else:
            self.offset *= 0.85

    def relax(self):
        """Command the arm to where it is (keeping the sag offsets): releases the spring force it stores against the
        held part before the fingers open."""
        self.set_comp("hold")
        self.q = self.measured.copy()
        self.pos, self.rot = fastik.tcp_pose(self.q, self.base)

    def measured_tcp(self):
        return fastik.tcp_pose(self.measured, self.base)

    def solve(self, pos, rot):
        """IK from the current joint angles; other seeds only if that fails, and then the solution nearest the current
        angles (no jump to another arm configuration in mid-move)."""
        from yam_arena.constants import READY

        q, pe, re = fastik.solve(pos, rot, self.q, base=self.base, iters=80)
        if pe < 2e-4 and re < 3e-3:
            return q, pe, re
        q, pe, re = fastik.solve(pos, rot, q, base=self.base, iters=200)
        best = (q, pe, re)
        if pe < 5e-4 and re < 5e-3:
            return best
        for s in (np.array(READY, float), np.array([0.0, 1.6, 1.4, -1.2, 0.0, 0.0])):
            q2, pe2, re2 = fastik.solve(pos, rot, s, base=self.base, iters=200)
            cost2 = pe2 + 0.1 * re2 + 0.002 * np.abs(q2 - self.q).sum()
            if cost2 < best[1] + 0.1 * best[2] + 0.002 * np.abs(best[0] - self.q).sum():
                best = (q2, pe2, re2)
        return best

    def set_pose(self, pos, rot, tol=0.004):
        q, pe, re = self.solve(pos, rot)
        if pe > tol or re > math.radians(4):
            raise Stop(
                f"{self.name} unreachable at {self.label}: {pe * 1000:.1f} mm {math.degrees(re):.1f} deg"
            )
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), np.asarray(rot, float)

    def move(self, pos, rot, duration, grip=None, label="", tol=0.004):
        self.label = label or self.label
        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        from yam_arena import ik

        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self.set_pose(
                p0 + s * (np.asarray(pos) - p0), ik.slerp_matrix(r0, np.asarray(rot), s), tol=tol
            )
            yield

    def settle(self, tol=0.003, max_s=2.5, label=""):
        """Hold the command until the measured TCP is within ``tol`` of it (the joint rate limit and the PD lag can
        leave the arm behind after a long move)."""
        self.label = label or self.label
        for _ in range(int(max_s * CONTROL_HZ)):
            if np.linalg.norm(self.measured_tcp()[0] - self.pos) < tol:
                return
            yield

    def set_grip(self, grip, duration=0.8, label=""):
        """Open / close the fingers with the arm's joint command held (no new IK solve)."""
        from yam_arena import ik

        self.label = label or self.label
        g0 = self.grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            self.grip = g0 + ik.smoothstep(k / n) * (grip - g0)
            yield

    def move_q(self, q_target, duration, grip=None, label=""):
        """Joint-space move (smoothstep) to ``q_target`` (used to change arm configuration before a Cartesian
        approach)."""
        from yam_arena import ik

        self.label = label or self.label
        q0, q1 = self.q.copy(), np.clip(np.asarray(q_target, float), self.lo, self.hi)
        g0 = self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.q = q0 + s * (q1 - q0)
            self.grip = g0 + s * (g1 - g0)
            self.pos, self.rot = fastik.tcp_pose(self.q, self.base)
            yield

    def home(self, duration=2.5, label=""):
        from yam_arena import ik
        from yam_arena.constants import READY

        self.label = label or f"{self.name} home"
        q0, q1 = self.q.copy(), np.array(READY, float)
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            self.q = q0 + ik.smoothstep(k / n) * (q1 - q0)
            self.pos, self.rot = fastik.tcp_pose(self.q, self.base)
            yield


def wait(seconds):
    for _ in range(max(1, round(seconds * CONTROL_HZ))):
        yield


def both(a, b):
    """Run two arm generators in parallel (one control step each per yield)."""
    gens = [a, b]
    while gens:
        alive = []
        for g in gens:
            try:
                next(g)
                alive.append(g)
            except StopIteration:
                pass
        gens = alive
        if gens:
            yield






class Ctx:
    def __init__(self, emb=None):
        from yam_arena import station

        self.emb = emb
        self.left = Arm("left", station.BASES["left"])
        self.right = Arm("right", station.BASES["right"])
        self.arms = {"left": self.left, "right": self.right}
        self.log = []
        self.step = 0
        self.grip_peak = {"left": 0.0, "right": 0.0}
        self.hand = Hand()
        self.notch = {}
        self.last_sign = 0
        self.settled = (0.0, False)

    def state(self):
        import balkit

        return balkit.bench_state()

    def detail(self):
        import balkit

        return balkit.STATE["detail"]

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack

        if getattr(self, "replay", None) is not None:  # open-loop replay of a recorded command trace
            a = np.asarray(self.replay[min(self.step, len(self.replay) - 1)], float)
            self.left.commit(a[:7])
            self.right.commit(a[7:])
            return Action(data=a.copy())
        cl, cr = self.left.cmd, self.right.cmd
        self.left.commit(cl)
        self.right.commit(cr)
        return Action(data=pack(cl, cr))

    def grip_force(self):
        """Estimated squeeze per finger (N): finger drive stiffness x (measured opening - commanded opening)."""
        from yam_arena.constants import GRIPPER

        out = {}
        for n, a in self.arms.items():
            if a.last_cmd is None:
                out[n] = 0.0
                continue
            f = GRIPPER["stiffness"] * max(0.0, (a.measured_grip - a.last_cmd[6]) * FINGER_OPEN)
            out[n] = min(f, GRIPPER["effort"])
            self.grip_peak[n] = max(self.grip_peak[n], out[n])
        return out

    def note(self, tag, **kw):
        d = self.detail()
        row = dict(at=tag, step=self.step, level=d.get("level"), pointer=d.get("pointer_mm"), reading=d.get("reading_g"),
                   notches=d.get("notches"), poise=d.get("poise_g"), **kw)
        self.log.append(row)
        print("LOOK", json.dumps(row, default=float), flush=True)


def frame(app, close):
    """Gripper rotation from its approach axis (z) and finger-closing axis (x), both in the station frame."""
    z = np.asarray(app, float) / np.linalg.norm(app)
    x = np.asarray(close, float)
    x = x - (x @ z) * z
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def down(psi_deg):
    """Straight-down TCP with the fingers closing along the horizontal direction at yaw ``psi_deg``."""
    a = math.radians(psi_deg)
    return frame([0.0, 0.0, -1.0], [math.cos(a), math.sin(a), 0.0])


def feasible(arm, pos, rot, q0=None, seeds=(), tol=1.5e-3, rtol_deg=1.5):
    q, pe, re = fastik.solve(pos, rot, arm.q if q0 is None else q0, base=arm.base, iters=150)
    if pe < tol and re < math.radians(rtol_deg):
        return True, q
    for s in seeds:
        q2, pe2, re2 = fastik.solve(pos, rot, np.asarray(s, float), base=arm.base, iters=200)
        if pe2 < tol and re2 < math.radians(rtol_deg):
            return True, q2
    return False, q


# --------------------------------------------------------------------------------------------------- balance
def bal_frames(c):
    """Station rotation of the balance base (B frame) and of the beam (tilted with the beam)."""
    s = c.state()
    return s["base"], s["beam"]


def handle_point(c, k):
    """Station point between the pad tips when pinching rider k's handle (tips TIP_Z above the beam axis)."""
    s = c.state()
    p, r = s[k]
    return p + r @ np.array([0.0, 0.0, TIP_Z])


def pinch_rot(c):
    """Straight-down TCP with the fingers closing across the beams (along the knife-edge axis)."""
    (_, rb), _ = bal_frames(c)
    ax = rb[:, 1]
    return frame([0.0, 0.0, -1.0], [ax[0], ax[1], 0.0])


def tcp_for(point, rot):
    return np.asarray(point, float) - PAD_TIP * rot[:, 2]


class Hand:
    """Right-hand state for rider work: which rider it is over (None: elsewhere)."""

    def __init__(self):
        self.over = None


def to_rider(c, k):
    """Right hand: open, over rider k's handle, then down onto it (not yet closed)."""
    arm = c.right
    arm.set_comp(True)
    rot = pinch_rot(c)
    hw = L.RIDER_HANDLE[k][1] / 2
    pt = handle_point(c, k)
    above = tcp_for(pt, rot) + np.array([0.0, 0.0, LIFT])
    if c.hand.over is None:  # from READY: joint-space move to the pose above the handle
        ok, q = feasible(arm, above, rot, seeds=(np.array([0.0, 1.6, 1.4, -1.2, 0.0, 0.0]),
                                                 np.array([0.4, 1.3, 1.1, -0.5, 0.0, 0.0])), tol=1e-3, rtol_deg=1.0)
        if not ok:
            raise Stop(f"no IK above {k}")
        yield from arm.move_q(q, 2.0, grip_of(hw + OPEN_CLEAR), f"right to {k}")
    elif c.hand.over != k:
        dist = float(np.linalg.norm(arm.pos - above))
        yield from arm.move(above, rot, max(0.8, 1.2 + 4.0 * (dist - 0.1)), grip_of(hw + OPEN_CLEAR),
                            f"right over {k}", tol=0.006)
    else:
        yield from arm.move(above, rot, 0.4, grip_of(hw + OPEN_CLEAR), f"right over {k}", tol=0.006)
    yield from arm.settle(0.003, 1.0, f"settle over {k}")
    pt = handle_point(c, k)
    yield from arm.move(tcp_for(pt, rot), rot, 0.7, None, f"down onto {k}", tol=0.004)
    yield from arm.settle(0.002, 1.0, f"settle on {k}")
    c.hand.over = k


def slide_rider(c, k, s_target, label="", release=True):
    """Pinch rider k, slide it along its beam to ``s_target`` (m from the fulcrum), let go, lift clear (``release``
    False: keep pinching it)."""
    arm = c.right
    yield from to_rider(c, k)
    hw = L.RIDER_HANDLE[k][1] / 2
    yield from arm.set_grip(grip_of(hw - SQUEEZE), 0.4, f"pinch {k}")
    yield from wait(0.1)
    arm.set_comp("hold")
    for attempt in range(3):
        s_now = c.state()["riders"][k]
        d = s_target - s_now
        if abs(d) < 0.0002:
            break
        _, (_, rbm) = bal_frames(c)
        u = rbm[:, 0]  # along the beam (as tilted)
        dur = max(0.6, abs(d) / 0.08 + 0.4)
        p0 = arm.pos.copy()
        yield from arm.move(p0 + d * u, arm.rot, dur, None, f"slide {k} {label}", tol=0.006)
        yield from wait(0.25)
    if not release:
        c.note("slid, still pinching", rider=k, at_mm=round(c.state()["riders"][k] * 1e3, 2), label=label)
        return
    arm.relax()
    yield from arm.set_grip(grip_of(hw + OPEN_CLEAR), 0.4, f"let go {k}")
    yield from arm.move(arm.pos + np.array([0.0, 0.0, LIFT]), arm.rot, 0.5, None, f"lift off {k}", tol=0.008)
    arm.set_comp(True)
    s_end = c.state()["riders"][k]
    c.note("slid", rider=k, target_mm=round(s_target * 1e3, 2), at_mm=round(s_end * 1e3, 2), label=label)


def watch_sign(c, min_s=1.0, max_s=3.0):
    """Hands off: watch the free beam. Returns +1 if the pan side is heavier (pointer above zero), -1 if the riders
    are (pointer below)."""
    hist = []
    n = 0
    while True:
        st = c.state()
        hist.append(st["pointer_mm"])
        n += 1
        if n >= min_s * CONTROL_HZ:
            p = float(np.mean(hist[-6:]))
            if (abs(p) > 2.5 and st["speed_mm_s"] < 4.0) or n >= max_s * CONTROL_HZ:
                break
        yield
    p = float(np.mean(hist[-6:]))
    c.last_sign = 1 if p > 0 else -1
    c.note("watched", pointer_mm=round(p, 2), sign=c.last_sign)


def watch_settle(c, max_s=7.0):
    """Hands off: wait for the damped beam to settle; c.settled = (pointer mm, at a stop)."""
    hist = []
    n = 0
    while True:
        st = c.state()
        hist.append((st["pointer_mm"], st["speed_mm_s"]))
        n += 1
        if n >= 1.5 * CONTROL_HZ:
            recent = hist[-10:]
            if max(v for _, v in recent) < 0.4 or n >= max_s * CONTROL_HZ:
                break
            if all(abs(p) > STOP_MM - 0.3 for p, _ in recent) and n >= 2.0 * CONTROL_HZ:
                break
        yield
    p = float(np.mean([x for x, _ in hist[-6:]]))
    c.settled = (p, abs(p) > STOP_MM - 0.4)
    c.note("settled", pointer_mm=round(p, 3), at_stop=c.settled[1])


def bisect_notches(c, k):
    """Largest notch on beam k that leaves the pan side heavier (the next one up tips the beam)."""
    lo, hi = 0, L.n_notches(k) + 1  # lo: known pan-heavy (rider at zero); hi: known too heavy / past the beam
    cur = 0
    while hi - lo > 1:
        mid = (lo + hi) // 2
        yield from slide_rider(c, k, L.notch_s(k, mid), f"try notch {mid}")
        cur = mid
        yield from watch_sign(c)
        if c.last_sign > 0:
            lo = mid
        else:
            hi = mid
    if cur != lo:
        yield from slide_rider(c, k, L.notch_s(k, lo), f"back to notch {lo}")
    c.note("notch set", rider=k, notch=lo)
    c.notch[k] = lo


def fine_poise(c, max_tries=7, start_g=5.0, keep_last=False):
    """Slide the 10 g poise until the settled pointer is on the zero mark. While the beam settles on a stop, only the
    side is known: the poise's range is narrowed (bisection); once the pointer settles between the stops, the poise is
    moved by the reading (5.1 mm per gram)."""
    lo, hi = 0.0, 10.0
    g_new = start_g
    for i in range(max_tries):
        yield from slide_rider(c, "r10", L.s_of_grams("r10", g_new), f"poise try {i}")
        # hand back to READY (clear of the balance and of the overhead view) while the beam settles
        yield from both(hands_home(c), watch_settle(c))
        p, at_stop = c.settled
        g_now = L.grams_of_s("r10", c.state()["riders"]["r10"])
        if at_stop:
            span = 0.9 * STOP_MM / MM_PER_G  # at least this far out of balance
            if p > 0:
                lo = max(lo, min(10.0, g_now + span))
            else:
                hi = min(hi, max(0.0, g_now - span))
            g_new = (lo + hi) / 2
        else:
            if abs(p) < 0.25:
                c.note("poise on zero", pointer_mm=round(p, 3), poise_g=round(g_now, 3))
                return
            g_new = float(np.clip(g_now + p / MM_PER_G, 0.0, 10.0))  # pan-heavy grams -> poise further out
            if keep_last:  # negative stage: the correction read off the scale is made without letting go
                yield from slide_rider(c, "r10", L.s_of_grams("r10", g_new), "poise final, kept pinched",
                                       release=False)
                return
    c.note("poise tries used up")


def hands_home(c):
    arm = c.right
    arm.set_comp(True)
    if c.hand.over is not None:
        p = arm.pos + np.array([0.0, 0.0, 0.03])
        yield from arm.move(p, arm.rot, 0.5, None, "right up", tol=0.01)
        yield from arm.home(2.2)
    c.hand.over = None


# --------------------------------------------------------------------------------------------------- stages
def weigh_coarse(c):
    yield from wait(0.5)
    c.note("start", pointer_mm=round(c.state()["pointer_mm"], 2))
    yield from bisect_notches(c, "r500")
    yield from bisect_notches(c, "r100")


def demo(c):
    yield from weigh_coarse(c)
    yield from fine_poise(c)
    yield from hands_home(c)
    yield from wait(6.0)


def idle(c):
    yield from wait(8.0)


def neg_coarse_only(c):
    """The 100 g and 10 g riders set by the same bisection, the 10 g poise never moved: the beam stays on its upper
    stop (the jar's 1-9 g past the 10 g step are not balanced): 0."""
    yield from weigh_coarse(c)
    yield from hands_home(c)
    yield from wait(6.0)


def neg_unseated(c):
    """100 g rider left on the crest 7 mm short of its notch (not seated, as if let go between notches), then the
    poise balanced against it: the beam rests on zero but there is no valid reading: 0.5."""
    yield from weigh_coarse(c)
    k = "r100"
    s_off = L.notch_s(k, c.notch[k]) - 0.007
    yield from slide_rider(c, k, s_off, "leave short of the notch")
    yield from fine_poise(c, max_tries=7, start_g=7.0)
    yield from hands_home(c)
    yield from wait(6.0)


def neg_hand_on(c):
    """Weighed as in the demo, but the last poise correction is made without letting go: the riders read the right
    mass, but the hand stays on the poise to the end: 0."""
    yield from weigh_coarse(c)
    yield from fine_poise(c, keep_last=True)
    c.note("holding the poise")
    yield from wait(8.0)


def replay(c):
    """Open-loop check: plays the 14-D commands recorded in a finished run's trace.jsonl (path in TB_REPLAY), step by
    step, without reading anything: what a fixed trajectory scores on another start state. TB_MIRROR=1 swaps the arms
    and negates joints 1, 5 and 6 (the mirrored trace)."""
    import os

    rows = [json.loads(line) for line in open(os.environ["TB_REPLAY"])]
    cmds = [np.asarray(r["cmd"], float) for r in rows]
    if os.environ.get("TB_MIRROR") == "1":
        out = []
        for a in cmds:
            lft, rgt = a[7:].copy(), a[:7].copy()
            for arm in (lft, rgt):
                arm[[0, 4, 5]] *= -1.0
            out.append(np.r_[lft, rgt])
        cmds = out
    c.replay = cmds
    for _ in range(len(c.replay) + 40):
        yield


STAGES = {
    "demo": demo,
    "idle": idle,
    "replay": replay,
    "neg-coarse-only": neg_coarse_only,
    "neg-unseated": neg_unseated,
    "neg-hand-on": neg_hand_on,
}

def _write_mp4(frames, path):
    import imageio

    w = imageio.get_writer(
        str(path),
        fps=CONTROL_HZ,
        codec="libx264",
        quality=8,
        macro_block_size=8,
        ffmpeg_params=["-pix_fmt", "yuv420p"],
    )
    for f in frames:
        w.append_data(np.asarray(f))
    w.close()


def run(
    task_path,
    out_dir,
    max_steps=2000,
    record=True,
    stage="demo",
    cam_width=320,
    cam_height=176,
    device="cpu",
    seed=None,
):
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    t0 = time.time()
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(
        task=str(Path(task_path).resolve()),
        object_jitter=0,
        control_hz=CONTROL_HZ,
        cam_width=cam_width,
        cam_height=cam_height,
        device=device,
    )
    stages = stage.split(",")
    results, resets = [], []
    for i, st in enumerate(stages):
        tdir = out if len(stages) == 1 else out / f"trial{i}_{st.replace('@', '_seed')}"
        st_seed = seed
        if "@" in st:  # "<stage>@<seed>": this trial's reset seed ('none': named setup)
            st, sd = st.split("@")
            st_seed = None if sd == "none" else int(sd)
        res, reset_state = run_trial(emb, tdir, max_steps, record, st, t0, seed=st_seed)
        results.append(res)
        resets.append(reset_state)
    if len(stages) > 1:
        rep = reset_report([x.split("@")[0] for x in stages], resets, results)
        (out / "reset_check.json").write_text(json.dumps(rep, indent=2, default=float))
        print("RESETCHECK", json.dumps(rep["summary"]), flush=True)
    return results


def reset_state_now():
    import balkit

    rt = balkit.runtime()
    poses = rt.read()
    return dict(
        score=balkit.STATE["score"],
        setup=rt.setup,
        joints={k: round(v, 6) for k, v in rt.q().items()},
        bodies={n: np.round(p * 1000, 4).tolist() for n, (p, r) in poses.items()},
        rots={n: np.round(r, 6).tolist() for n, (p, r) in poses.items()},
    )


def reset_report(stages, resets, results):
    import layout as L

    rows = []
    for i, r in enumerate(resets):
        setup = tuple(r["setup"]) if isinstance(r["setup"], list) else r["setup"]
        spec = L.setup_spec(setup)
        gp, gr = L.balance_pose(spec)
        dev = float(np.linalg.norm(np.subtract(r["bodies"]["base"], gp * 1000)))
        sp, _ = L.sample_start(spec)
        dev = max(dev, float(np.linalg.norm(np.subtract(r["bodies"]["sample"], sp * 1000))))
        jdev = max(abs(r["joints"][k] - L.S0) for k in L.RIDERS)
        rows.append(dict(trial=i, stage=stages[i], setup=r["setup"], score_after_reset=r["score"],
                         body_dev_from_authored_mm=round(dev, 4), max_rider_dev_m=round(jdev, 5),
                         theta=r["joints"]["theta"], trial_score=results[i]["task_score"],
                         trial_stop=results[i]["stop"]))
    ok = all(x["score_after_reset"] == 0.0 and x["body_dev_from_authored_mm"] < 2.0 and x["max_rider_dev_m"] < 1e-4
             for x in rows)
    return dict(summary=dict(passed=ok, trials=len(rows), max_body_dev_mm=max(x["body_dev_from_authored_mm"] for x in rows)),
                rows=rows, reset_states=resets)


def run_trial(emb, out, max_steps, record, stage, t0, seed=None):
    import balkit
    from inspect_robots import Scene
    from inspect_robots_yam.packing import STATE_KEY

    out.mkdir(parents=True, exist_ok=True)
    t1 = time.time()
    obs = emb.reset(Scene(id="demo", instruction="demo"), seed=seed)
    print(f"BENCH reset {time.time() - t1:.0f} s", flush=True)
    reset_state = reset_state_now()
    c = Ctx(emb)
    gen = STAGES[stage](c)
    frames = {}

    def snap(o):
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    snap(obs)
    steps, ended, info, stop, success, res = 0, None, {}, None, False, None
    tracef = open(out / "trace.jsonl", "w")
    while steps < max_steps:
        if ended is None:
            try:
                next(gen)
            except StopIteration:
                ended = steps
            except Stop as e:
                stop, ended = str(e), steps
                print("STOP", stop, flush=True)
        res = emb.step(c.action())
        jp = np.asarray(res.observation.state[STATE_KEY], float).ravel()
        c.left.observe(jp[:7])
        c.right.observe(jp[7:14])
        steps += 1
        c.step = steps
        info = res.info
        snap(res.observation)
        d = balkit.STATE["detail"]
        row = dict(
            step=steps,
            score=info.get("score"),
            level=balkit.STATE["level"],
            detail=d,
            labels=[c.left.label, c.right.label],
            nonfinite=balkit.STATE["nonfinite"],
            cmd=np.round(np.r_[c.left.last_cmd, c.right.last_cmd], 4).tolist(),
            q=np.round(jp, 4).tolist(),
            grip_n={k: round(v, 2) for k, v in c.grip_force().items()},
            tcp_mm={
                n: np.round(a.measured_tcp()[0] * 1000, 2).tolist()
                for n, a in c.arms.items()
            },
            tcp_cmd_mm={
                n: np.round(a.pos * 1000, 2).tolist() for n, a in c.arms.items()
            },
            offs={n: np.round(a.offset, 3).tolist() for n, a in c.arms.items()},
        )
        tracef.write(json.dumps(row, default=float) + "\n")
        if steps % 20 == 0:
            tracef.flush()
        if steps % 25 == 0:
            print(
                f"step {steps} {c.left.label} | {c.right.label} score {info.get('score')} ptr {d.get('pointer_mm')} "
                f"spd {d.get('speed_mm_s')} reading {d.get('reading_g')} notches {d.get('notches')} poise {d.get('poise_g')} "
                f"rest {d.get('rest_s')} clear {d.get('clear_s')} contact {d.get('contact_n')} jar {d.get('jar_rel_mm')} "
                f"wall {time.time() - t1:.0f}s",
                flush=True,
            )
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= 2 * CONTROL_HZ:
            break
    score = float(info.get("score", 0.0) or 0.0)
    result = dict(
        stage=stage,
        task_score=score,
        success=success,
        steps=steps,
        stop=stop,
        termination=getattr(res, "termination_reason", None),
        setup=reset_state["setup"],
        scorer=dict(score=balkit.STATE["score"], level=balkit.STATE["level"]),
        final=balkit.STATE["detail"],
        nonfinite=balkit.STATE["nonfinite"],
        looks=c.log,
        wall_s=round(time.time() - t1, 1),
        wall_total_s=round(time.time() - t0, 1),
        worst_ik_mm=dict(
            left=round(c.left.worst_mm, 2), right=round(c.right.worst_mm, 2)
        ),
        grip_peak_n={k: round(v, 2) for k, v in c.grip_peak.items()},
        physics_dt=balkit.PHYSICS_DT,
        control_hz=CONTROL_HZ,
    )
    tracef.close()
    (out / "result.json").write_text(json.dumps(result, indent=2, default=float))
    if record and frames:
        import imageio

        for cam, fs in frames.items():
            _write_mp4(fs, out / f"demo_{cam}.mp4")
        top = next((k for k in frames if "top" in k), next(iter(frames), None))
        if top:
            imageio.imwrite(out / "setup.png", frames[top][0])
            imageio.imwrite(out / "final.png", frames[top][-1])
        for cam, fs in frames.items():
            imageio.imwrite(out / f"final_{cam}.png", fs[-1])
    print(
        "RESULT",
        json.dumps(
            {
                k: result[k]
                for k in ("stage", "task_score", "success", "steps", "stop", "setup")
            }
        ),
        flush=True,
    )
    return result, reset_state


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2000)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--stage", default="demo")
    ap.add_argument(
        "--cam-size",
        default="320x176",
        help="camera WxH (evidence runs use the default)",
    )
    ap.add_argument(
        "--device",
        default="cpu",
        help="PhysX device (cpu, or cuda:0); rendering is on the GPU either way",
    )
    ap.add_argument(
        "--seed",
        default=str(L.OFFICIAL_SEED),
        help="reset seed (seeded start state, as in inspect-robots runs; default: the official eval-seed-0 trial seed, "
        "'none': the named setups)",
    )
    a = ap.parse_args()
    cw, ch = (int(v) for v in a.cam_size.split("x"))
    sd = None if a.seed in ("none", "") else int(a.seed)
    run(a.task, a.out, a.max_steps, not a.no_record, a.stage, cw, ch, a.device, sd)
