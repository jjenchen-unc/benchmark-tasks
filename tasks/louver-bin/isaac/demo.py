"""Scripted full-score demo (and negative variants) for the louver-bin task in the yam_arena embodiment.

    python demo.py task.py OUT [--max-steps N] [--no-record] [--stage demo|<variant>[,<variant>...]] [--seed S]

Acts ONLY through the 14 joint/gripper commands of ``YamArenaEmbodiment.step``. It reads the bins' and the panel's
poses through ``louverkit.bench_state()`` (read-only, like a stock clerk looking at the panel, its louver rows and the
bin that already hangs there); it never writes a pose, a joint state or a score, and it never reads the start spec:
the target row and the free half come from the hanging bin's measured pose. Its joint commands are rate-limited
below the CLI's per-step delta guard and clipped to the URDF joint bounds.

Plan (each hand pinches one side wall of the bin by its rim, gripper pointing forward and down):
1. Read the target: the louver row the other bin hangs on (its hanger height), and the panel's free half (the side
   away from that bin); aim the new bin at the mirror image of the other bin's place about the panel's centre line.
2. Both hands pinch the loose bin's side walls 100 mm in front of its back, lift it, raise it level toward the panel (turning it
   square to the panel), to 35 mm in front of the panel with its hanger tab 8 mm above the target row's lip.
3. Move it back until its back wall touches the louver lips (closed loop on the measured bin and panel poses): the
   hidden tab is now over the channel behind the target lip.
4. Lower it 22 mm so the tab drops into the channel and the hanger shelf sits on the lip, relax, open both hands,
   back them out to a plane in front of every bin place and park them low and outboard.
Both arms add the remaining joint error back to their commands (slow integral on their own joint readings), because
the joint PD sags under load (pattern reused from earlier demos).
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
SQUEEZE = 0.003  # fingers commanded 3 mm inside the wall faces: about 15 N per finger (gripper 5 kN/m)
OPEN_HALF = L.WALL / 2 + L.RIM_BEAD + 0.010  # open: clears the rim bead by 10 mm a side
PAD_DEPTH = 0.013  # pad tips this far below the rim top
PITCH_DEG = 22.0  # gripper points toward the bin's back and down by this much (reach at the top row)
APPROACH = 0.060
VIA_X = 0.180  # via points in front of every hanging-bin place (bin fronts are at x 0.26 or more)
STAGE_DX = -0.035  # staging: bin back 35 mm in front of its hung place
STAGE_DZ = 0.020  # staging: hanger shelf 20 mm above the lip top (tab bottom 8 mm above it)
TOUCH_DX = 0.0004  # back wall pressed 0.1 mm onto the lips (hung pose has 0.5 mm air)
LOWER_DZ = -0.002  # lower to 2 mm below the shelf-on-lip height (the lip stops it)
ROT_CORR_MAX = 0.10
PARK = (0.220, 0.360, 0.900)  # parked TCP (left; right mirrored): low and outboard, clear of both bins and of the top
# camera's view of them (READY, at x 0.32 z 1.01, is within 5 cm of a hung bin's front)


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
        self.offset_clip = 0.25

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
                self.offset + COMP_GAIN * (self.q - self.measured), -self.offset_clip, self.offset_clip
            )
        else:
            self.offset *= 0.85

    def relax(self):
        """Command the arm to where it is (keeping the sag offsets): releases the spring force it stores against the
        rail before the fingers open."""
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

    def set_pose(self, pos, rot):
        q, pe, re = self.solve(pos, rot)
        if pe > 0.004 or re > math.radians(4):
            raise Stop(
                f"{self.name} unreachable at {self.label}: {pe * 1000:.1f} mm {math.degrees(re):.1f} deg"
            )
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), np.asarray(rot, float)

    def move(self, pos, rot, duration, grip=None, label=""):
        self.label = label or self.label
        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        from yam_arena import ik

        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self.set_pose(
                p0 + s * (np.asarray(pos) - p0), ik.slerp_matrix(r0, np.asarray(rot), s)
            )
            yield

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
        self.G = {}
        self.corr = {}

    def pose(self, name="bin"):
        import louverkit

        return louverkit.bench_state()[name]

    def detail(self):
        import louverkit

        return louverkit.STATE["detail"]

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack

        if (
            getattr(self, "replay", None) is not None
        ):  # open-loop replay of a recorded command trace
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
        row = dict(
            at=tag, step=self.step, measure=d.get("measure"), level=d.get("level"), **kw
        )
        self.log.append(row)
        print("LOOK", json.dumps(row, default=float), flush=True)


HAND_SIDE = {"left": 1.0, "right": -1.0}  # left hand on the bin's +y wall


def grasp_tcp(pb, rb, hand, flip=False):
    """TCP pose that puts the pad tips across hand's side wall, PAD_DEPTH below the rim, gripper pointing toward the
    bin's back and down by PITCH_DEG."""
    from yam_arena.constants import PAD_TIP_PAST_TCP

    s = HAND_SIDE[hand]
    a = math.radians(PITCH_DEG)
    zb = np.array([math.cos(a), 0.0, -math.sin(a)])
    xb = np.array([0.0, 1.0, 0.0]) * (1.0 if not flip else -1.0)
    rb_tcp = np.column_stack([xb, np.cross(zb, xb), zb])
    pad = np.array([L.GRIP_X, s * (L.BIN_W / 2 - L.WALL / 2), -PAD_DEPTH])
    p = pb + rb @ (pad - zb * PAD_TIP_PAST_TCP)
    return p, rb @ rb_tcp


def target_of(c):
    """(panel face x, target row, target y, lip top z) from the measured panel and hanging-bin poses."""
    po, _ = c.pose("panel")
    pr, rr = c.pose("ref")
    row, _ = L.row_of_z(float(pr[2]))
    y_t = float(po[1]) - (float(pr[1]) - float(po[1])) + getattr(c, "y_shift", 0.0)  # mirror of the other bin
    row = row + getattr(c, "row_shift", 0)
    return float(po[0]), row, y_t, L.lip_top_z(row)


def hung_at(c, dx=0.0, dz=0.0):
    xp, row, y_t, zl = target_of(c)
    return np.array([xp - L.BACK_GAP + dx, y_t, zl + dz]), np.eye(3)


def capture(c, hand):
    pb, rb = c.pose("bin")
    pt, rt = c.arms[hand].measured_tcp()
    c.G[hand] = (rb.T @ (pt - pb), rb.T @ rt)


def track(c, target_fn, duration, label, gain=0.05, free=(False, False, False), until=None, max_s=None,
          rot_gain=None):
    """Closed loop on the bin: each step command both hands to the TCP poses that put the bin at target_fn(s) through
    their frozen grasp transforms, plus slow integral corrections of the measured bin position and orientation error
    (arm sag, grasp slip). ``free``: world axes whose error is not integrated (pushing against a stop)."""
    from yam_arena import ik

    n = max(1, round(duration * CONTROL_HZ))
    nmax = n if max_s is None else max(n, round(max_s * CONTROL_HZ))
    cp, cr = c.corr
    for k in range(1, nmax + 1):
        pt, rt = target_fn(ik.smoothstep(min(1.0, k / n)))
        pm, rm = c.pose("bin")
        err = np.where(np.asarray(free), 0.0, pt - pm)
        cp = np.clip(cp + gain * err, -0.020, 0.020)
        e_r = _rotvec(rt @ rm.T)
        cr = np.clip(cr + (gain if rot_gain is None else rot_gain) * e_r, -ROT_CORR_MAX, ROT_CORR_MAX)
        rc = _rotm(cr)
        for h in ("left", "right"):
            gp, gr = c.G[h]
            arm = c.arms[h]
            arm.label = label
            arm.set_pose(pt + cp + rc @ rt @ gp, rc @ rt @ gr)
        c.corr = (cp, cr)
        yield
        if until is not None and k >= n and until(c):
            break


def bin_err(c, dx=0.0, dz=0.0):
    pt, _ = hung_at(c, dx, dz)
    pm, rm = c.pose("bin")
    return pt - pm, math.degrees(np.linalg.norm(_rotvec(rm)))


# --------------------------------------------------------------------------------------------------- the script
def pick(c):
    for arm in c.arms.values():
        arm.set_comp(True)
    pb, rb = c.pose("bin")
    tg = {}
    for h, arm in c.arms.items():
        best = None
        for flip in (False, True):
            p, r = grasp_tcp(pb, rb, h, flip)
            q, pe, re = arm.solve(p, r)
            cost = 10.0 * (pe > 0.002 or re > 0.03) + 0.2 * abs(q[5]) + 0.01 * flip
            if best is None or cost < best[0]:
                best = (cost, flip)
        tg[h] = best[1]
    c.flip = tg
    pre = {}
    for h in c.arms:
        p, r = grasp_tcp(pb, rb, h, tg[h])
        pre[h] = (p - r[:, 2] * APPROACH, r)
    # via points: straight back to x = VIA_X first, across, then forward to the pre-grasp pose (the direct line from
    # READY to the far wall would pass through the hanging bin's place)
    via1 = {h: np.array([VIA_X, arm.pos[1], arm.pos[2]]) for h, arm in c.arms.items()}
    via2 = {h: np.array([VIA_X, pre[h][0][1], pre[h][0][2] + 0.020]) for h in c.arms}
    yield from both(*(arm.move(via1[h], pre[h][1], 1.2, grip_of(OPEN_HALF), f"{h} back from ready")
                      for h, arm in c.arms.items()))
    yield from both(*(arm.move(via2[h], pre[h][1], 1.5, None, f"{h} across") for h, arm in c.arms.items()))
    yield from both(*(arm.move(pre[h][0], pre[h][1], 1.5, None, f"{h} pre-grasp") for h, arm in c.arms.items()))
    yield from wait(0.3)
    pb, rb = c.pose("bin")  # re-read (the bin does not move, but the plan uses the measured pose)
    g = {h: grasp_tcp(pb, rb, h, tg[h]) for h in c.arms}
    yield from both(*(arm.move(g[h][0], g[h][1], 2.0, None, f"{h} in") for h, arm in c.arms.items()))
    yield from wait(0.3)
    yield from both(*(arm.set_grip(grip_of(L.WALL / 2 - SQUEEZE), 1.0, f"{h} close") for h, arm in c.arms.items()))
    yield from wait(0.5)
    c.note("gripped")
    yield from both(*(arm.move(arm.pos + [0, 0, 0.050], arm.rot, 2.0, None, f"{h} lift")
                      for h, arm in c.arms.items()))
    yield from wait(0.5)
    capture(c, "left")
    capture(c, "right")
    c.corr = (np.zeros(3), np.zeros(3))
    c.note("lifted", target=target_of(c))


def carry(c):
    p0, r0 = c.pose("bin")
    ps, rs = hung_at(c, STAGE_DX, STAGE_DZ)
    yaw0 = math.degrees(math.atan2(r0[1, 0], r0[0, 0]))
    zmid = max(p0[2], ps[2]) + 0.005
    # 1: up to above the staging height and toward the panel, sideways to the aim point, turning square to it
    p1 = np.array([max(p0[0], ps[0] - 0.030), ps[1], zmid])  # rising, and back to 65 mm in front of the panel
    yield from track(c, lambda s: (p0 + s * (p1 - p0), L.rz(yaw0 * (1 - s))), 3.5, "up")
    # 2: toward the panel, down to the staging height
    yield from track(c, lambda s: (p1 + s * (ps - p1), rs), 2.0, "to the panel", gain=0.06)
    c.note("staged", err_mm=(bin_err(c, STAGE_DX, STAGE_DZ)[0] * 1e3).round(2).tolist())


def hang(c, lower=True):
    ps, rs = hung_at(c, STAGE_DX, STAGE_DZ)
    pt, _ = hung_at(c, TOUCH_DX, STAGE_DZ)
    # 3: back wall onto the louver lips (tab over the channel)
    yield from track(c, lambda s: (ps + s * (pt - ps), rs), 2.5, "back to the lips", gain=0.06,
                     until=lambda c: np.abs(bin_err(c, TOUCH_DX, STAGE_DZ)[0][[1, 2]]).max() < 0.0015, max_s=5.0)
    e, tilt = bin_err(c, TOUCH_DX, STAGE_DZ)
    c.note("at lips", err_mm=(e * 1e3).round(2).tolist(), tilt_deg=round(tilt, 2))
    if not lower:
        return
    # 4: lower: the tab drops into the channel, the shelf lands on the lip (z not integrated: the lip stops it)
    pl, _ = hung_at(c, TOUCH_DX, LOWER_DZ)
    for arm in c.arms.values():
        arm.set_comp("hold")
    yield from track(c, lambda s: (pt + s * (pl - pt), rs), 2.0, "lower onto the louver", gain=0.04,
                     free=(False, False, True))
    yield from wait(0.5)
    c.note("lowered")


def let_go(c):
    """Relax both arms (command them to where they are), open, back out along the gripper axis, park low."""
    for h in c.arms:
        c.arms[h].relax()
    yield from wait(0.4)
    yield from both(*(c.arms[h].set_grip(grip_of(OPEN_HALF), 1.0, f"{h} open") for h in c.arms))
    yield from wait(0.3)
    yield from both(*(c.arms[h].move(c.arms[h].pos - c.arms[h].rot[:, 2] * 0.070, c.arms[h].rot, 1.5, None,
                                     f"{h} out") for h in c.arms))
    for h in c.arms:
        c.arms[h].set_comp(True)
    yield from both(*(c.arms[h].move(c.arms[h].pos + [-0.120, 0, -0.040], c.arms[h].rot, 1.5, None, f"{h} back")
                      for h in c.arms))
    # out to the via plane in front of every bin place, across to the arm's own side, then down to the park pose
    v1 = {h: np.array([VIA_X, c.arms[h].pos[1], min(c.arms[h].pos[2], 0.980)]) for h in c.arms}
    v2 = {h: np.array([VIA_X, PARK[1] * HAND_SIDE[h], v1[h][2]]) for h in c.arms}
    yield from both(*(c.arms[h].move(v1[h], c.arms[h].rot, 1.2, None, f"{h} to via") for h in c.arms))
    yield from both(*(c.arms[h].move(v2[h], c.arms[h].rot, 1.5, None, f"{h} across") for h in c.arms))
    yield from both(*(park(c.arms[h], h, 1.5) for h in c.arms))
    c.note("released")


def park(arm, hand, duration=2.5):
    """Joint-space move to a parked pose low and outboard (PARK): the joint target is solved once from READY (no
    Cartesian path through the arm's reach limits)."""
    from yam_arena import ik
    from yam_arena.constants import READY

    target = np.array([PARK[0], PARK[1] * HAND_SIDE[hand], PARK[2]])
    best = None
    for rot in (arm.rot, arm.rot @ np.diag([-1.0, -1.0, 1.0])):
        q, pe, re = fastik.solve(target, rot, np.array(READY, float), base=arm.base, iters=300)
        if best is None or pe + 0.01 * re < best[1] + 0.01 * best[2]:
            best = (q, pe, re)
    arm.label = f"{hand} park"
    q0, q1 = arm.q.copy(), best[0]
    n = max(1, round(duration * CONTROL_HZ))
    for k in range(1, n + 1):
        arm.q = q0 + ik.smoothstep(k / n) * (q1 - q0)
        arm.pos, arm.rot = fastik.tcp_pose(arm.q, arm.base)
        yield


def demo(c):
    yield from wait(0.5)
    yield from pick(c)
    yield from carry(c)
    yield from hang(c)
    yield from let_go(c)
    yield from wait(3.0)


def idle(c):
    yield from wait(8.0)


def carry_only(c):
    """Development: pick, carry to the staging point, hold."""
    yield from wait(0.5)
    yield from pick(c)
    yield from carry(c)
    yield from wait(2.0)


def neg_wrong_row(c):
    """Hangs the bin properly, but on the louver row above the other bin's row: 0."""
    c.row_shift = 1
    yield from demo(c)


def neg_on_lip(c):
    """Sets the bin down with its hanger tab ON the target lip (back wall 4.5 mm short of the lips, so the tab lands on
    the lip's top edge instead of dropping behind it), then lets go: the bin tips forward and drops off: 0."""
    yield from wait(0.5)
    yield from pick(c)
    yield from carry(c)
    ps, rs = hung_at(c, STAGE_DX, STAGE_DZ)
    pt, _ = hung_at(c, -0.0045, STAGE_DZ)
    yield from track(c, lambda s: (ps + s * (pt - ps), rs), 2.5, "short of the lips", gain=0.06)
    pl, _ = hung_at(c, -0.0045, L.TAB_DROP - 0.001)
    for arm in c.arms.values():
        arm.set_comp("hold")
    yield from track(c, lambda s: (pt + s * (pl - pt), rs), 2.0, "tab onto the lip edge", gain=0.04,
                     free=(False, False, True))
    yield from wait(0.5)
    c.note("tab on lip", bin=list(c.pose("bin")[0]))
    yield from let_go(c)
    yield from wait(3.0)


def neg_hold(c):
    """Hangs the bin correctly but never lets go (hands stay on the rims): 0.5."""
    yield from wait(0.5)
    yield from pick(c)
    yield from carry(c)
    yield from hang(c)
    yield from wait(4.0)


def replay(c):
    """Open-loop check: plays the 14-D commands recorded in a finished run's trace.jsonl (path in LB_REPLAY), step
    by step, without reading anything: what a fixed trajectory scores on another start state."""
    import os

    rows = [json.loads(line) for line in open(os.environ["LB_REPLAY"])]
    c.replay = [r["cmd"] for r in rows]
    for _ in range(len(c.replay) + 40):
        yield


STAGES = {
    "demo": demo,
    "idle": idle,
    "carry-only": carry_only,
    "replay": replay,
    "neg-wrong-row": neg_wrong_row,
    "neg-on-lip": neg_on_lip,
    "neg-hold": neg_hold,
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
    max_steps=3000,
    record=True,
    stage="demo",
    cam_width=320,
    cam_height=176,
    device="cpu",
    seed=None,
    seeds=None,
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
    # per-trial seeds: --seeds a,b,... ('none' = unseeded named setup); otherwise --seed for every trial
    trial_seeds = [None if x in ("none", "") else int(x) for x in seeds.split(",")] if seeds else [seed] * len(stages)
    results, resets = [], []
    for i, st in enumerate(stages):
        tdir = out if len(stages) == 1 else out / f"trial{i}_{st}"
        res, reset_state = run_trial(emb, tdir, max_steps, record, st, t0, seed=trial_seeds[i])
        results.append(res)
        resets.append(reset_state)
    if len(stages) > 1:
        rep = reset_report(stages, resets, results)
        (out / "reset_check.json").write_text(json.dumps(rep, indent=2, default=float))
        print("RESETCHECK", json.dumps(rep["summary"]), flush=True)
    return results


def reset_state_now():
    import louverkit

    rt = louverkit.runtime()
    poses = rt.read()
    return dict(
        score=louverkit.STATE["score"],
        setup=rt.setup,
        bodies={n: np.round(p * 1000, 4).tolist() for n, (p, r) in poses.items()},
        rots={n: np.round(r, 6).tolist() for n, (p, r) in poses.items()},
    )


def reset_report(stages, resets, results):
    import louverkit

    rows = []
    for i, r in enumerate(resets):
        want = louverkit.authored(
            tuple(r["setup"]) if isinstance(r["setup"], list) else r["setup"]
        )
        dev = max(
            float(np.linalg.norm(np.subtract(r["bodies"][n], want[n][0] * 1000)))
            for n in ("bin", "ref", "panel")
        )
        rdev = float(np.abs(np.subtract(r["rots"]["bin"], want["bin"][1])).max())
        rows.append(
            dict(
                trial=i,
                stage=stages[i],
                setup=r["setup"],
                score_after_reset=r["score"],
                max_body_dev_from_authored_mm=round(dev, 4),
                max_rot_dev=round(rdev, 6),
                trial_score=results[i]["task_score"],
                trial_stop=results[i]["stop"],
            )
        )
    ok = all(
        x["score_after_reset"] == 0.0 and x["max_body_dev_from_authored_mm"] < 3.0  # the bins settle (the hanging one on its louver)
        for x in rows
    )
    return dict(
        summary=dict(
            passed=ok,
            trials=len(rows),
            max_body_dev_mm=max(x["max_body_dev_from_authored_mm"] for x in rows),
        ),
        rows=rows,
        reset_states=resets,
    )


def run_trial(emb, out, max_steps, record, stage, t0, seed=None):
    import louverkit
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
        d = louverkit.STATE["detail"]
        row = dict(
            step=steps,
            score=info.get("score"),
            level=louverkit.STATE["level"],
            detail=d,
            labels=[c.left.label, c.right.label],
            nonfinite=louverkit.STATE["nonfinite"],
            cmd=np.round(np.r_[c.left.last_cmd, c.right.last_cmd], 4).tolist(),
            q=np.round(jp, 4).tolist(),
            corr_mm=np.round(c.corr[0] * 1000, 3).tolist() if isinstance(c.corr, tuple) else None,
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
            m = d.get("measure", {})
            b = m.get("bin", {}) or {}
            fmt = lambda e: {k: (round(e[k] * 1e3, 2) if k in ("dz", "tab_x") else (round(e[k], 2) if isinstance(e.get(k), float) else e.get(k))) for k in ("row", "dz", "tab_x", "in_channel", "level_deg") if k in e}  # noqa: E731
            print(
                f"step {steps} {c.left.label} | {c.right.label} score {info.get('score')} bin {fmt(b)} "
                f"ref_hangs {m.get('ref_hangs')} gap {m.get('gap_mm')} tip {m.get('tip_mm')} "
                f"grip {d.get('contacts')} bin_mm {d.get('bin_mm')} wall {time.time() - t1:.0f}s",
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
        scorer=dict(score=louverkit.STATE["score"], level=louverkit.STATE["level"]),
        final=louverkit.STATE["detail"],
        nonfinite=louverkit.STATE["nonfinite"],
        looks=c.log,
        wall_s=round(time.time() - t1, 1),
        wall_total_s=round(time.time() - t0, 1),
        worst_ik_mm=dict(
            left=round(c.left.worst_mm, 2), right=round(c.right.worst_mm, 2)
        ),
        physics_dt=louverkit.PHYSICS_DT,
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
    ap.add_argument("--max-steps", type=int, default=3000)
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
        type=int,
        default=None,
        help="reset seed (seeded start state, as in inspect-robots runs)",
    )
    ap.add_argument(
        "--seeds",
        default=None,
        help="comma-separated reset seed per trial of a multi-stage run ('none' = unseeded named setup)",
    )
    a = ap.parse_args()
    cw, ch = (int(v) for v in a.cam_size.split("x"))
    run(a.task, a.out, a.max_steps, not a.no_record, a.stage, cw, ch, a.device, a.seed, a.seeds)
