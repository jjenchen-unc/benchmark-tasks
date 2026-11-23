"""Scripted full-score demo (and negative variants) for the pool-8ball task in the yam_arena embodiment.

    python demo.py task.py OUT [--max-steps N] [--no-record] [--stage demo|<variant>[,<variant>...]] [--seed S]

Acts ONLY through the 14 joint/gripper commands of ``YamArenaEmbodiment.step``. It reads the ball, cue and coin poses
through ``poolkit.bench_state()`` (read-only, like a player looking at the table); it never writes a pose, a joint
state or a score. Its joint commands are rate-limited below the CLI's per-step delta guard and clipped to the URDF
joint bounds.

Plan:
1. Read the coin (called pocket) and the two balls; compute the ghost-ball line (cue direction d) and the cut. The
   hand on the side the cue points to bridges, the other strokes (the arms never cross).
2. Stroke hand pinches the cue in its rest from above (grip point 0.57 m from the tip); bridge hand closes a loop
   bridge around the shaft 0.195 m from the tip (approach 30 deg below horizontal, one finger under the shaft, one over
   it). Both lift it off the hooks.
3. Carry the cue (closed loop on its measured pose) behind the cue ball, lower it to the address: tip 25 mm behind the
   cue ball on the ghost-ball line, 4 deg elevation. Fine-aim on the measured cue pose.
4. Bridge hand relaxes to a loose loop; stroke hand: 6 cm backswing, pause, one forward stroke along the cue axis at
   the speed planned from the distances (``plan_speed``), follow through 3 cm, stop.
5. Lift the cue clear of the table and wait for the balls to stop.
"""

from __future__ import annotations

import argparse
import json
import math
import os
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
S_GRIP = L.BRIDGE_LEN + L.GRIP_LEN  # grip point, m from the tip (0.57)
S_BRIDGE = L.BRIDGE_LEN  # bridge point, m from the tip
ELEV_DEG = 7.0  # butt raised to clear the foot rail with the stroke hand
BRIDGE_TILT_DEG = 30.0  # bridge approach below horizontal
GRIP_SQUEEZE = 0.004  # stroke hand: fingers 4 mm inside the shaft surface (about 20 N)
BRIDGE_CLEAR = 0.0004  # loop bridge: fingers 0.4 mm outside the shaft surface
BRIDGE_MODE = os.environ.get("POOL_BRIDGE", "loop")  # 'loop' (finger under + finger over the shaft) or 'pinch' (fingers on its sides, light squeeze)
PINCH_SQUEEZE = 0.0015
LIFT = 0.07  # cue lifted straight up out of the rest before it turns
STANDOFF = 0.06  # bridge hand starts this far back along its approach
BACKSWING = 0.06
FOLLOW = 0.10  # commanded follow-through past the contact (the arm lags ~2 control steps)
RAMP_STEPS = 2
SPEED_GAIN = 1.0  # commanded stroke speed / wanted tip speed (calibrated)
CUE_BALL_K = 1.20  # cue-ball speed / commanded tip speed (dev run d10: 0.90 m/s from 0.66 m/s commanded = 1.36; 1.2 kept, the 8-ball still arrives)
YAW_FF_DEG = float(os.environ.get("POOL_YAW_FF", "0.0"))  # aim offset (deg, anticlockwise) for the stroke's drift
ARRIVE_V = 0.30  # planned 8-ball speed at the pocket mouth (m/s)


def grip_of(half_opening_m):
    return float(np.clip(half_opening_m / FINGER_OPEN, 0.0, 1.0))


class Stop(Exception):
    pass


def _rotvec(r):
    from scipy.spatial.transform import Rotation

    return Rotation.from_matrix(r).as_rotvec()


def _rotm(v):
    from scipy.spatial.transform import Rotation

    return Rotation.from_rotvec(v).as_matrix()


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
        self.label = ""
        self.worst_mm = 0.0
        _, _, lo, hi, _ = fastik._chain()
        self.lo, self.hi = lo, hi
        self.last_cmd = None
        self.rate = RATE

    @property
    def cmd(self):
        c = np.r_[np.clip(self.q + self.offset, self.lo, self.hi), self.grip]
        lim = np.r_[self.rate * 0.05 * (self.hi - self.lo), RATE * 0.05]
        if self.last_cmd is not None:
            c = np.clip(c, self.last_cmd - lim, self.last_cmd + lim)
        return c

    def commit(self, c):
        self.last_cmd = np.asarray(c, float).copy()

    def set_comp(self, mode):
        self.comp = mode

    def observe(self, q):
        self.measured = np.asarray(q[:6], float)
        self.measured_grip = float(q[6])
        if self.comp == "hold":
            pass
        elif self.comp:
            self.offset = np.clip(self.offset + COMP_GAIN * (self.q - self.measured), -0.25, 0.25)
        else:
            self.offset *= 0.85

    def measured_tcp(self):
        return fastik.tcp_pose(self.measured, self.base)

    def solve(self, pos, rot):
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

    def set_pose(self, pos, rot):
        q, pe, re = self.solve(pos, rot)
        if pe > 0.004 or re > math.radians(8):
            raise Stop(f"{self.name} unreachable at {self.label}: {pe * 1000:.1f} mm {math.degrees(re):.1f} deg")
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), np.asarray(rot, float)

    def move(self, pos, rot, duration, grip=None, label=""):
        from yam_arena import ik

        self.label = label or self.label
        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self.set_pose(p0 + s * (np.asarray(pos) - p0), ik.slerp_matrix(r0, np.asarray(rot), s))
            yield

    def move_to(self, pos, rot, duration, grip=None, label=""):
        """Joint-space move to the IK solution of (pos, rot) (best of several seeds): for large reorientations, where a
        Cartesian slerp would pass through poses the wrist cannot reach."""
        from yam_arena import ik
        from yam_arena.constants import READY

        self.label = label or self.label
        best = None
        for seed in (self.q, np.array(READY, float), np.array([0.0, 1.6, 1.4, -1.2, 0.0, 0.0]),
                     np.array([0.0, 0.8, 0.8, -0.5, 0.0, 1.5]), np.array([0.0, 0.8, 0.8, -0.5, 0.0, -1.5])):
            q, pe, re = fastik.solve(pos, rot, seed, base=self.base, iters=300)
            cost = pe * 1000 + math.degrees(re) + 0.05 * float(np.abs(q - self.q).sum())
            if best is None or cost < best[0]:
                best = (cost, q, pe, re)
        _, q1, pe, re = best
        if pe > 0.004 or re > math.radians(8):
            raise Stop(f"{self.name} unreachable at {self.label}: {pe * 1000:.1f} mm {math.degrees(re):.1f} deg")
        q0, g0 = self.q.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.q = q0 + s * (q1 - q0)
            self.grip = g0 + s * (g1 - g0)
            self.pos, self.rot = fastik.tcp_pose(self.q, self.base)
            yield
        self.pos, self.rot = np.asarray(pos, float), np.asarray(rot, float)

    def set_grip(self, grip, duration=0.8, label=""):
        yield from self.move(self.pos, self.rot, duration, grip, label)

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


def both(*gens):
    gens = list(gens)
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
        self.G = {}
        self.corr_p, self.corr_r = np.zeros(3), np.zeros(3)
        self.plan = None
        self.replay = None

    def pose(self, name="cue"):
        """Measured pose; for the cue the frame is rebuilt from its axis alone (x toward the tip, y horizontal), so its
        free roll about its own axis (a round shaft rolls in the hooks and the hands) does not enter any target."""
        import poolkit

        p, r = poolkit.bench_state()[name]
        if name == "cue":
            x = r[:, 0] / np.linalg.norm(r[:, 0])
            y = np.cross([0.0, 0.0, 1.0], x)
            y /= np.linalg.norm(y)
            r = np.column_stack([x, y, np.cross(x, y)])
        return p, r

    def vel(self, name="cue"):
        import poolkit

        return poolkit.bench_velocities()[name]

    def detail(self):
        import poolkit

        return poolkit.STATE["detail"]

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack

        if self.replay is not None:  # open-loop replay of a recorded command trace
            a = np.asarray(self.replay[min(self.step, len(self.replay) - 1)], float)
            self.left.commit(a[:7])
            self.right.commit(a[7:])
            return Action(data=a.copy())
        cl, cr = self.left.cmd, self.right.cmd
        self.left.commit(cl)
        self.right.commit(cr)
        return Action(data=pack(cl, cr))

    def note(self, tag, **kw):
        d = self.detail()
        row = dict(at=tag, step=self.step, level=d.get("level"), facts=d.get("facts"), **kw)
        self.log.append(row)
        print("LOOK", json.dumps(row, default=float), flush=True)


# --------------------------------------------------------------------------------------------- geometry helpers
def cue_rot(d, elev_deg):
    """Cue frame rotation: x along the cue toward the tip (horizontal direction d, tip lower by elev), z up-ish."""
    e = math.radians(elev_deg)
    x = np.array([d[0] * math.cos(e), d[1] * math.cos(e), -math.sin(e)])
    y = np.cross([0.0, 0.0, 1.0], x)
    y /= np.linalg.norm(y)
    return np.column_stack([x, y, np.cross(x, y)])


def axis_point(p, r, s):
    """Point on the cue axis s metres behind the tip face."""
    return p - s * r[:, 0]


def grip_rot(cue_r, sign=1.0, roll_deg=0.0):
    """Stroke hand: approach down (rolled ``roll_deg`` about the cue axis), fingers closing across the cue."""
    a = cue_r[:, 0]
    across = np.array([-a[1], a[0], 0.0])
    across /= np.linalg.norm(across)
    x = sign * across
    z = np.array([0.0, 0.0, -1.0])
    r = np.column_stack([x, np.cross(z, x), z])
    return _rotm(math.radians(roll_deg) * a) @ r


def bridge_rot(cue_r, side, beta=BRIDGE_TILT_DEG, gamma=0.0):
    """Bridge hand: approach from its own side across the cue, tilted ``beta`` below horizontal and swung ``gamma``
    toward the tip (negative: toward the butt) about the closing axis; the closing axis lies across the cue, the
    'upper' finger over the shaft and the other under it (a loop bridge)."""
    a = cue_r[:, 0]
    h = np.array([a[0], a[1], 0.0])
    h /= np.linalg.norm(h)
    perp = np.array([-h[1], h[0], 0.0])  # to the cue's left
    toward = -perp if side == "left" else perp  # from the hand's side toward the cue
    b = math.radians(beta)
    z0 = math.cos(b) * toward + np.array([0.0, 0.0, -math.sin(b)])
    x = np.cross(z0, a)
    x /= np.linalg.norm(x)
    if x[2] < 0:
        x = -x
    w = np.cross(x, z0)
    if w @ a < 0:
        w = -w
    g = math.radians(gamma)
    z = math.cos(g) * z0 + math.sin(g) * w
    return np.column_stack([x, np.cross(z, x), z])


def bridge_half(r, loose=False):
    """Finger half-opening at the bridge for a shaft of radius r."""
    if BRIDGE_MODE == "pinch":
        # the closed bridge keeps its squeeze through the stroke: it must hold the shaft down against the cue's
        # weight behind the stroke hand (d5: a looser bridge let the tip rise 7 cm and the stroke passed over the ball)
        return r - PINCH_SQUEEZE
    return r + BRIDGE_CLEAR + (0.001 if loose else 0.0)


_IK_SEEDS = (
    np.array([0.0, 1.0, 1.0, -0.6, 0.0, 0.0]),
    np.array([0.0, 1.6, 1.4, -1.2, 0.0, 0.0]),
    np.array([0.0, 0.8, 0.8, -0.5, 0.0, 1.5]),
    np.array([0.0, 0.8, 0.8, -0.5, 0.0, -1.5]),
)


def _pose_cost(arm, p, r, q0):
    best = None
    for seed in (q0,) + _IK_SEEDS:
        q, pe, re = fastik.solve(p, r, seed, base=arm.base, iters=120)
        cost = pe * 1000 + math.degrees(re)
        if best is None or cost < best[0]:
            best = (cost, q)
        if cost < 0.05:
            break
    return best


def choose_hand_rots(c, plan):
    """Pick the bridge orientation (beta, gamma) and the stroke-hand roll that both arms can hold at the cue rest
    AND at the address (the grasp is kept between them), least wrist effort first."""
    from yam_arena.constants import READY

    pr, rr = plan["rest"]
    tip, ra = plan["address"]
    ready = np.array(READY, float)
    ba, sa = c.arms[plan["bridge"]], c.arms[plan["stroke"]]
    best_b = None
    if BRIDGE_MODE == "pinch":
        cands = [(sg, roll, grip_rot(rr, sg, roll)) for sg in (1.0, -1.0) for roll in (0, 15, -15, 30, -30, 45, -45)]
    else:
        # loop bridge: one finger under the shaft, one over it; approach as close to horizontal as reachable so the
        # finger over the shaft pushes straight down (no sideways push on the line)
        cands = [(beta, gamma, bridge_rot(rr, plan["bridge"], beta, gamma))
                 for beta in (0, 10, 20, 30) for gamma in (0, -20, 20, -40, 40, -60)]
    for beta, gamma, rot in cands:
        if True:
            rel = rr.T @ rot
            c1, q1 = _pose_cost(ba, axis_point(pr, rr, S_BRIDGE), rr @ rel, ready)
            c1 = max(c1, _pose_cost(ba, axis_point(pr, rr, S_BRIDGE) - STANDOFF * (rr @ rel)[:, 2], rr @ rel, ready)[0],
                     _pose_cost(ba, axis_point(pr, rr, S_BRIDGE) + [0, 0, LIFT], rr @ rel, ready)[0])
            c2, _ = _pose_cost(ba, axis_point(tip, ra, S_BRIDGE), ra @ rel, ready)
            cost = max(c1, c2) + 0.01 * abs(gamma) + (0.03 * beta if BRIDGE_MODE == "loop" else 0.0)
            if best_b is None or cost < best_b[0]:
                best_b = (cost, beta, gamma, rel)
    best_g = None
    for sign in (1.0, -1.0):
        for roll in (0, 15, -15, 30, -30):
            rel = rr.T @ grip_rot(rr, sign, roll)
            costs = [_pose_cost(sa, axis_point(pr, rr, S_GRIP) + [0, 0, s_up], rr @ rel, ready)[0] for s_up in (0.0, LIFT)]
            for ds in (0.0, -BACKSWING, L.TIP_GAP + FOLLOW):
                t = tip + ds * ra[:, 0]
                costs.append(_pose_cost(sa, axis_point(t, ra, S_GRIP), ra @ rel, ready)[0])
            cost = max(costs) + 0.01 * abs(roll)
            if best_g is None or cost < best_g[0]:
                best_g = (cost, sign, roll, rel)
    plan["bridge_rel"], plan["grip_rel"] = best_b[3], best_g[3]
    plan["hand_choice"] = dict(bridge=dict(cost=round(best_b[0], 2), beta=best_b[1], gamma=best_b[2]),
                               grip=dict(cost=round(best_g[0], 2), sign=best_g[1], roll=best_g[2]))
    return plan


def capture_grasp_cmd(c):
    """Grasp transforms from the hands' COMMANDED poses and the measured cue pose (right after closing, before any
    load: the commanded pose is where the fingers are)."""
    pc, rc = c.pose("cue")
    for n, arm in c.arms.items():
        c.G[n] = (rc.T @ (arm.pos - pc), rc.T @ arm.rot)
    c.corr_p, c.corr_r = np.zeros(3), np.zeros(3)


def capture_grasp(c):
    pc, rc = c.pose("cue")
    for n, arm in c.arms.items():
        pt, rt = arm.measured_tcp()
        c.G[n] = (rc.T @ (pt - pc), rc.T @ rt)
    c.corr_p, c.corr_r = np.zeros(3), np.zeros(3)


def track(c, target_fn, duration, label, gain=0.05, rot_gain=None, arms=None, until=None, max_s=None):
    """Closed loop on the cue pose: command the hands to the TCP poses that put the cue at target_fn(s) through the
    captured grasp transforms, plus slow integral corrections of the measured cue pose error."""
    from yam_arena import ik

    names = list(arms or c.arms)
    n = max(1, round(duration * CONTROL_HZ))
    nmax = n if max_s is None else max(n, round(max_s * CONTROL_HZ))
    for k in range(1, nmax + 1):
        pt, rt = target_fn(ik.smoothstep(min(1.0, k / n)))
        pm, rm = c.pose("cue")
        prev_p, prev_r = c.corr_p.copy(), c.corr_r.copy()
        c.corr_p = np.clip(c.corr_p + gain * (pt - pm), -0.05, 0.05)
        # orientation: correct the yaw only (aim); pitch follows the bridge height and roll is free
        e_r = _rotvec(rt @ rm.T)
        c.corr_r = np.clip(c.corr_r + (gain if rot_gain is None else rot_gain) * np.array([0.0, 0.0, e_r[2]]), -0.06, 0.06)
        for attempt in range(2):
            rc = _rotm(c.corr_r)
            try:
                sols = []
                for name in names:
                    arm = c.arms[name]
                    gp, gr = c.G[name]
                    arm.label = label
                    q, pe, re = arm.solve(pt + c.corr_p + rc @ rt @ gp, rc @ rt @ gr)
                    if pe > 0.004 or re > math.radians(8):
                        raise Stop(f"{name} unreachable at {label}: {pe * 1000:.1f} mm {math.degrees(re):.1f} deg")
                    sols.append((arm, q, pt + c.corr_p + rc @ rt @ gp, rc @ rt @ gr, pe))
                for arm, q, p_, r_, pe in sols:
                    arm.q, arm.pos, arm.rot = q, p_, r_
                    arm.worst_mm = max(arm.worst_mm, pe * 1000)
                break
            except Stop:
                if attempt == 1:
                    raise
                c.corr_p, c.corr_r = prev_p, prev_r  # back off this step's correction
        yield
        if until is not None and k >= n and until(c):
            break


# --------------------------------------------------------------------------------------------- shot planning
def called_pocket(coin_xy):
    return min(L.CALLABLE, key=lambda n: float(np.linalg.norm(L.coin_pos(n)[:2] - np.asarray(coin_xy[:2]))))


def _travel(v0, dist, mu_s=0.20, mu_r=L.ROLL_MU):
    """Speed after rolling ``dist`` from a sliding start at v0 (no spin): sliding until v = 5/7 v0, then rolling."""
    g = L.G
    ts = 2 * v0 / (7 * mu_s * g)
    ds = v0 * ts - 0.5 * mu_s * g * ts * ts
    if dist <= ds:
        return math.sqrt(max(0.0, v0 * v0 - 2 * mu_s * g * dist))
    vr = 5.0 / 7.0 * v0
    return math.sqrt(max(0.0, vr * vr - 2 * mu_r * g * (dist - ds)))


def plan_speed(shot, arrive=ARRIVE_V, e=L.BALL_RESTITUTION):
    """Cue-ball start speed so that the 8-ball reaches the pocket aim point at ``arrive`` m/s (1-D model)."""
    cut = math.radians(shot["cut_deg"])
    lo, hi = 0.1, 4.0
    for _ in range(60):
        v0 = 0.5 * (lo + hi)
        vi = _travel(v0, shot["cue_travel"])
        v8 = vi * math.cos(cut) * (1 + e) / 2
        va = _travel(v8, shot["obj_travel"])
        if va < arrive:
            lo = v0
        else:
            hi = v0
    return 0.5 * (lo + hi)


def make_plan(c):
    st = {n: c.pose(n) for n in ("cueball", "eightball", "coin")}
    pocket = called_pocket(st["coin"][0])
    cb, e8 = st["cueball"][0], st["eightball"][0]
    shot = L.shot(cb[:2], e8[:2], pocket)
    d = shot["d"]
    bridge_arm, stroke_arm = L.roles(d)
    d = L.rz(YAW_FF_DEG)[:2, :2] @ d  # the address line, offset for the stroke's measured drift
    r = cue_rot(d, ELEV_DEG)
    tip = np.array([cb[0], cb[1], L.BALL_Z]) - (L.BALL_R + L.TIP_GAP) * r[:, 0]
    v0 = plan_speed(shot)
    plan = dict(
        pocket=pocket,
        shot={k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in shot.items()},
        d=d,
        bridge=bridge_arm,
        stroke=stroke_arm,
        address=(tip, r),
        v_cueball=v0,
        v_tip=v0 / CUE_BALL_K,
    )
    plan["rest"] = c.pose("cue")
    choose_hand_rots(c, plan)
    c.plan = plan
    c.note("plan", hands=plan["hand_choice"], pocket=pocket, cut=round(shot["cut_deg"], 2), phi=round(shot["phi_deg"], 2), bridge=bridge_arm,
           v_cueball=round(v0, 3), v_tip=round(plan["v_tip"], 3))
    return plan


# --------------------------------------------------------------------------------------------- the script
def pick_cue(c):
    plan = c.plan
    pc, rc = c.pose("cue")
    sa, ba = c.arms[plan["stroke"]], c.arms[plan["bridge"]]
    for arm in c.arms.values():
        arm.set_comp(True)
    gp = axis_point(pc, rc, S_GRIP)
    bp = axis_point(pc, rc, S_BRIDGE)
    gr = rc @ plan["grip_rel"]
    br = rc @ plan["bridge_rel"]
    r_g, r_b = L.cue_radius(S_GRIP), L.cue_radius(S_BRIDGE)
    back = br[:, 2]  # bridge approach axis
    yield from both(
        sa.move_to(gp + [0, 0, 0.07], gr, 3.0, grip_of(r_g + 0.02), "stroke hand over grip"),
        ba.move_to(bp - STANDOFF * back, br, 3.0, grip_of(r_b + 0.025), "bridge hand to the shaft"),
    )
    yield from wait(0.3)
    pc, rc = c.pose("cue")
    gp, bp = axis_point(pc, rc, S_GRIP), axis_point(pc, rc, S_BRIDGE)
    yield from both(
        sa.move(gp, gr, 2.0, None, "stroke hand descend"),
        ba.move(bp, br, 2.0, None, "bridge hand around the shaft"),
    )
    yield from wait(0.3)
    yield from both(
        sa.set_grip(grip_of(r_g - GRIP_SQUEEZE), 1.0, "stroke hand close"),
        ba.set_grip(grip_of(bridge_half(r_b)), 1.0, "bridge close"),
    )
    yield from wait(0.5)
    c.note("gripped")
    # lift level, closed loop on the cue pose, straight up: clear of both hooks before any turn
    capture_grasp(c)  # measured hand poses (consistent with the cue as it lies)
    p0, r0 = c.pose("cue")
    yield from track(c, lambda s: (p0 + [0.0, 0.0, LIFT * s], r0), 2.0, "lift", gain=0.05)
    yield from track(c, lambda s: (p0 + [0.0, 0.0, LIFT], r0), 0.6, "lift settle", gain=0.05)
    c.note("lifted", cue=np.round(c.pose("cue")[0], 4).tolist(), corr_mm=np.round(c.corr_p * 1e3, 1).tolist())


def carry_to_address(c):
    plan = c.plan
    tip, r_addr = plan["address"]
    p0, r0 = c.pose("cue")
    d3 = r_addr[:, 0]
    pre = tip - 0.10 * d3 + np.array([0.0, 0.0, 0.05])  # 10 cm back, 5 cm up
    from yam_arena import ik

    def seg(pa, ra, pb, rb):
        return lambda s: (pa + s * (pb - pa), ik.slerp_matrix(ra, rb, s))

    yield from track(c, seg(p0, r0, pre, r_addr), 4.0, "carry", gain=0.04)
    yield from track(c, seg(pre, r_addr, tip - 0.04 * d3, r_addr), 2.0, "lower", gain=0.05)
    yield from track(c, seg(tip - 0.04 * d3, r_addr, tip, r_addr), 1.5, "address", gain=0.05)
    yield from track(c, lambda s: (tip, r_addr), 2.0, "aim", gain=0.08, rot_gain=0.08,
                     until=aimed(), max_s=8.0)
    c.note("addressed", err=aim_error(c))


def aim_error(c):
    """Lateral offset (mm) of the cue axis from the cue-ball centre and angle (deg) from the planned line."""
    pc, rc = c.pose("cue")
    cb = c.pose("cueball")[0]
    a = rc[:, 0]
    rel = cb - pc
    lat = rel - (rel @ a) * a
    d = c.plan["d"]
    ah = np.array([a[0], a[1]]) / np.linalg.norm(a[:2])
    ang = math.degrees(math.atan2(ah[0] * d[1] - ah[1] * d[0], ah @ d))
    return dict(lat_mm=round(float(np.linalg.norm(lat[:2])) * 1e3, 2), ang_deg=round(ang, 3),
                tip_gap_mm=round(float(rel @ a - L.BALL_R) * 1e3, 1), z_mm=round(float(lat[2]) * 1e3, 2))


def aimed(n=6):
    count = [0]

    def ok(c):
        e = aim_error(c)
        count[0] = count[0] + 1 if (e["lat_mm"] < 0.8 and abs(e["ang_deg"]) < 0.25) else 0
        return count[0] >= n

    return ok


STROKE_MODE = os.environ.get("POOL_STROKE", "slide")  # 'slide': cue slides through the bridge; 'both': hands move


def stroke(c, v_tip=None):
    """Backswing, then one forward stroke along the cue axis at the planned speed, follow-through, stop. 'slide': the
    stroke hand drives the cue through the bridge hand (which stays); 'both': both hands carry the cue."""
    plan = c.plan
    sa, ba = c.arms[plan["stroke"]], c.arms[plan["bridge"]]
    movers = [sa] if STROKE_MODE == "slide" else [sa, ba]
    v = plan["v_tip"] if v_tip is None else v_tip
    a = c.pose("cue")[1][:, 0]
    ba.set_comp("hold")
    sa.set_comp("hold")
    yield from both(*(m.move(m.pos - BACKSWING * a, m.rot, 1.2, None, "backswing") for m in movers))
    yield from wait(0.4)
    # re-aim at the end of the backswing (closed loop on the measured cue): the backswing itself can turn the cue
    tip0, r_addr = plan["address"]
    tip_b = tip0 - BACKSWING * r_addr[:, 0]
    for m in c.arms.values():
        m.set_comp(True)
    capture_grasp(c)
    yield from track(c, lambda s_: (tip_b, r_addr), 0.5, "re-aim", gain=0.08, rot_gain=0.10, until=aimed(n=4),
                     max_s=4.0)
    for m in c.arms.values():
        m.set_comp("hold")
    a = c.pose("cue")[1][:, 0]
    c.note("re-aimed", err=aim_error(c))
    travel = BACKSWING + L.TIP_GAP + FOLLOW
    vc = v * SPEED_GAIN
    p0 = {m.name: m.pos.copy() for m in movers}
    for m in movers:
        m.q = m.q.copy()
    for m in movers:
        m.rate = 0.95
    c.note("stroke start", v_cmd=round(vc, 3), mode=STROKE_MODE)

    def advance(ds):
        """Move the hands on along the cue axis; past the contact point an unreachable pose just ends the
        follow-through (the hands stay where they are)."""
        sols = []
        for m in movers:
            q, pe, re = m.solve(p0[m.name] + ds * a, m.rot)
            if pe > 0.004 or re > math.radians(8):
                if ds < BACKSWING + L.TIP_GAP + 0.01:
                    raise Stop(f"{m.name} unreachable at stroke: {pe * 1000:.1f} mm {math.degrees(re):.1f} deg")
                return False
            sols.append((m, q))
        for m, q in sols:
            m.q, m.pos = q, p0[m.name] + ds * a
            m.label = "stroke"
        return True

    s, k = 0.0, 0
    while s < travel:
        k += 1
        # ramp to the stroke speed over RAMP_STEPS control steps
        s = min(travel, s + vc * DT * min(1.0, k / RAMP_STEPS))
        if not advance(s):
            break
        yield
    for f in (0.6, 0.3, 0.1):  # ease out
        s += vc * DT * f
        if not advance(s):
            break
        yield
    for m in movers:
        m.rate = RATE
    yield from wait(0.6)
    c.note("stroke end", v_cue=np.round(c.vel("cue")[0], 3).tolist())


def clear_cue(c):
    """Lift the cue 12 cm straight up with both hands (the balls roll on under it), hold."""
    plan = c.plan
    for arm in c.arms.values():
        arm.set_comp("hold")
    for up in (0.08, 0.05, 0.03):
        ok = True
        for arm in c.arms.values():
            q, pe, re = arm.solve(arm.pos + [0, 0, up], arm.rot)
            ok = ok and pe < 0.004 and re < math.radians(8)
        if ok:
            try:
                yield from both(
                    *(arm.move(arm.pos + [0, 0, up], arm.rot, 1.5, None, "lift cue clear") for arm in c.arms.values())
                )
            except Stop as e:
                c.note("lift cue clear stopped", why=str(e))
            return
    c.note("lift cue clear: no reachable lift, holding")


def demo(c):
    yield from wait(0.3)
    make_plan(c)
    yield from pick_cue(c)
    yield from carry_to_address(c)
    yield from stroke(c)
    yield from clear_cue(c)
    yield from wait(25.0)


def idle(c):
    yield from wait(8.0)


def neg_full_ball(c):
    """The shortcut: drive the cue ball straight at the 8-ball's centre (no cut). Same speed plan; the start generator
    rejects layouts where this would pot the 8-ball, so it scores 0.25 or 0.5 at most."""
    yield from wait(0.3)
    plan = make_plan(c)
    cb, e8 = c.pose("cueball")[0], c.pose("eightball")[0]
    d = L.unit(e8[:2] - cb[:2])
    r = cue_rot(d, ELEV_DEG)
    plan["d"] = d
    plan["address"] = (np.array([cb[0], cb[1], L.BALL_Z]) - (L.BALL_R + L.TIP_GAP) * r[:, 0], r)
    choose_hand_rots(c, plan)
    yield from pick_cue(c)
    yield from carry_to_address(c)
    yield from stroke(c)
    yield from clear_cue(c)
    yield from wait(25.0)


def neg_soft(c):
    """Correct aim, stroke at 35 % of the planned speed: the 8-ball stops short (0.25 or 0.5)."""
    yield from wait(0.3)
    plan = make_plan(c)
    yield from pick_cue(c)
    yield from carry_to_address(c)
    yield from stroke(c, v_tip=0.35 * plan["v_tip"])
    yield from clear_cue(c)
    yield from wait(25.0)


def neg_misaim(c):
    """Aims 6 deg clockwise of the ghost-ball line (same speed): the 8-ball misses the pocket."""
    yield from wait(0.3)
    plan = make_plan(c)
    d = L.rz(-6.0)[:2, :2] @ plan["d"]
    cb = c.pose("cueball")[0]
    r = cue_rot(d, ELEV_DEG)
    plan["d"] = d
    plan["address"] = (np.array([cb[0], cb[1], L.BALL_Z]) - (L.BALL_R + L.TIP_GAP) * r[:, 0], r)
    choose_hand_rots(c, plan)
    yield from pick_cue(c)
    yield from carry_to_address(c)
    yield from stroke(c)
    yield from clear_cue(c)
    yield from wait(25.0)


def neg_shaft(c):
    """Pass the tip beside the stationary ball, then sweep the shaft into it."""
    yield from wait(0.3)
    plan = make_plan(c)
    yield from pick_cue(c)
    yield from carry_to_address(c)
    cb = c.pose("cueball")[0].copy()
    tip, rotation = plan["address"]
    side = rotation[:, 1].copy()
    if side[1] * cb[1] > 0:
        side = -side
    past_ball = cb + 0.050 * rotation[:, 0]
    for target, label in ((tip + 0.055 * side, "clear ball sideways"),
                          (past_ball + 0.055 * side, "pass beside ball"),
                          (past_ball - 0.005 * side, "sweep cue shaft")):
        start = c.pose("cue")[0].copy()
        yield from track(c, lambda s: (start + s * (target - start), rotation), 1.8, label)
    yield from wait(0.5)


def neg_double(c):
    """Touch the cue ball twice with the tip, retracting to a visible gap between taps."""
    yield from wait(0.3)
    plan = make_plan(c)
    yield from pick_cue(c)
    yield from carry_to_address(c)
    rotation = plan["address"][1]
    axis = rotation[:, 0]
    for tap in (1, 2):
        ball = c.pose("cueball")[0].copy()
        start = c.pose("cue")[0].copy()
        target = ball - (L.BALL_R - 0.009) * axis
        before = c.detail()["facts"]["strokes"]
        for _ in track(c, lambda s: (start + s * (target - start), rotation), 1.25, f"tip tap {tap}"):
            yield
            if c.detail()["facts"]["strokes"] > before:
                break
        c.note(f"tap {tap}")
        start = c.pose("cue")[0].copy()
        yield from track(c, lambda s: (start - s * L.TIP_GAP * axis, rotation), 0.35, "separate tip")
        yield from wait(0.5)
        gap = np.linalg.norm(c.pose("cueball")[0] - c.pose("cue")[0]) - L.BALL_R
        c.note(f"clear after tap {tap}", tip_gap_mm=round(float(gap) * 1000, 2))
    yield from wait(1.0)


def replay(c):
    import os

    rows = [json.loads(line) for line in open(os.environ["POOL_REPLAY"])]
    c.replay = [r["cmd"] for r in rows]
    for _ in range(len(c.replay) + 40):
        yield


_PROBE_COUNT = [0]


def probe_stroke(c):
    """DEV: pick, address, stroke at the speed in POOL_VTIP (m/s) and log the impact."""
    import os

    yield from wait(0.3)
    make_plan(c)
    yield from pick_cue(c)
    yield from carry_to_address(c)
    vs = [float(v) for v in os.environ.get("POOL_VTIP", "0.8").split(",")]
    v = vs[_PROBE_COUNT[0] % len(vs)]
    _PROBE_COUNT[0] += 1
    yield from stroke(c, v_tip=v)
    yield from clear_cue(c)
    yield from wait(15.0)


STAGES = {
    "demo": demo,
    "idle": idle,
    "replay": replay,
    "neg-full-ball": neg_full_ball,
    "neg-soft": neg_soft,
    "neg-misaim": neg_misaim,
    "neg-double": neg_double,
    "neg-shaft": neg_shaft,
    "probe-stroke": probe_stroke,
}


def _write_mp4(frames, path):
    import imageio

    w = imageio.get_writer(str(path), fps=CONTROL_HZ, codec="libx264", quality=8, macro_block_size=8,
                           ffmpeg_params=["-pix_fmt", "yuv420p"])
    for f in frames:
        w.append_data(np.asarray(f))
    w.close()


def run(task_path, out_dir, max_steps=1500, record=True, stage="demo", cam_width=320, cam_height=176, device="cpu",
        seed=None, seed_list=None):
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    t0 = time.time()
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=str(Path(task_path).resolve()), object_jitter=0, control_hz=CONTROL_HZ,
                               cam_width=cam_width, cam_height=cam_height, device=device)
    stages = stage.split(",")
    seeds = [seed] * len(stages)
    if seed_list:
        stages = [st for st in stages for _ in seed_list]
        seeds = [sd for _ in stage.split(",") for sd in seed_list]
    results, resets = [], []
    for i, st in enumerate(stages):
        tdir = out if len(stages) == 1 else out / (f"trial{i}_{st}" + ("" if not seed_list else f"_s{seeds[i]}"))
        res, reset_state = run_trial(emb, tdir, max_steps, record, st, t0, seed=seeds[i])
        results.append(res)
        resets.append(reset_state)
    if len(stages) > 1:
        rep = reset_report(stages, resets, results)
        (out / "reset_check.json").write_text(json.dumps(rep, indent=2, default=float))
        print("RESETCHECK", json.dumps(rep["summary"]), flush=True)
    return results


def reset_state_now():
    import poolkit

    rt = poolkit.runtime()
    poses = rt.read()
    return dict(
        score=poolkit.STATE["score"],
        setup=rt.setup,
        bodies={n: np.round(p * 1000, 4).tolist() for n, (p, r) in poses.items()},
    )


def reset_report(stages, resets, results):
    import poolkit

    rows = []
    for i, r in enumerate(resets):
        want = poolkit.authored(tuple(r["setup"]) if isinstance(r["setup"], list) else r["setup"])
        dev = max(float(np.linalg.norm(np.subtract(r["bodies"][n], want[n][0] * 1000))) for n in ("cueball", "eightball", "cue"))
        rows.append(dict(trial=i, stage=stages[i], setup=r["setup"], score_after_reset=r["score"],
                         max_body_dev_from_authored_mm=round(dev, 4), trial_score=results[i]["task_score"]))
    ok = all(x["score_after_reset"] == 0.0 and x["max_body_dev_from_authored_mm"] < 1.0 for x in rows)
    return dict(summary=dict(passed=ok, trials=len(rows), max_body_dev_mm=max(x["max_body_dev_from_authored_mm"] for x in rows)),
                rows=rows, reset_states=resets)


def run_trial(emb, out, max_steps, record, stage, t0, seed=None):
    import poolkit
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
        d = poolkit.STATE["detail"]
        vc = c.vel("cue")
        row = dict(
            step=steps, score=info.get("score"), level=poolkit.STATE["level"], detail=d,
            labels=[c.left.label, c.right.label], nonfinite=poolkit.STATE["nonfinite"],
            cmd=np.round(np.r_[c.left.last_cmd, c.right.last_cmd], 4).tolist(), q=np.round(jp, 4).tolist(),
            cue_v=np.round(vc[0], 4).tolist(), cue_tip=np.round(c.pose("cue")[0], 4).tolist(),
            cue_yaw=round(math.degrees(math.atan2(c.pose("cue")[1][1, 0], c.pose("cue")[1][0, 0])), 3),
            corr_mm=np.round(c.corr_p * 1000, 2).tolist(), corr_r=np.round(c.corr_r, 4).tolist(),
            offs={n: np.round(a.offset, 3).tolist() for n, a in c.arms.items()},
            tcp_cmd_mm={n: np.round(a.pos * 1000, 1).tolist() for n, a in c.arms.items()},
        )
        tracef.write(json.dumps(row, default=float) + "\n")
        if steps % 20 == 0:
            tracef.flush()
        if steps % 25 == 0:
            f = d.get("facts", {}) or {}
            print(f"step {steps} {c.left.label} | {c.right.label} score {info.get('score')} cb {d.get('cb_mm')} "
                  f"8 {d.get('eight_mm')} hit8 {f.get('hit8')} strokes {f.get('strokes')} fouls {f.get('fouls')} "
                  f"wall {time.time() - t1:.0f}s", flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= 2 * CONTROL_HZ:
            break
    score = float(info.get("score", 0.0) or 0.0)
    result = dict(stage=stage, task_score=score, success=success, steps=steps, stop=stop,
                  termination=getattr(res, "termination_reason", None), setup=reset_state["setup"],
                  scorer=dict(score=poolkit.STATE["score"], level=poolkit.STATE["level"]), final=poolkit.STATE["detail"],
                  nonfinite=poolkit.STATE["nonfinite"], looks=c.log, wall_s=round(time.time() - t1, 1),
                  wall_total_s=round(time.time() - t0, 1),
                  worst_ik_mm=dict(left=round(c.left.worst_mm, 2), right=round(c.right.worst_mm, 2)),
                  physics_dt=poolkit.PHYSICS_DT, control_hz=CONTROL_HZ)
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
    print("RESULT", json.dumps({k: result[k] for k in ("stage", "task_score", "success", "steps", "stop", "setup")},
                               default=str), flush=True)
    return result, reset_state


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=1500)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--stage", default="demo")
    ap.add_argument("--cam-size", default="320x176")
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--seeds", default=None, help="DEV: comma-separated seeds, one trial per seed and stage")
    a = ap.parse_args()
    cw, ch = (int(v) for v in a.cam_size.split("x"))
    sl = [int(v) for v in a.seeds.split(",")] if a.seeds else None
    run(a.task, a.out, a.max_steps, not a.no_record, a.stage, cw, ch, a.device, a.seed, sl)
    sys.stdout.flush()
    import os

    os._exit(0)
