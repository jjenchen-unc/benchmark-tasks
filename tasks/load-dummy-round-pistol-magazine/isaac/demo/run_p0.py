"""Base runner of the scripted demo (the multi-round P0 stages are unused).
Robot commands only (joint targets + gripper 0..1); the script reads simulator state to plan (allowed for demos).
The demo logic runs at HZ (10 Hz, as developed); the environment runs at ENV_HZ (20 Hz, as the official run). Each demo
command is held for ENV_HZ / HZ environment steps (zero-order hold, the same joint targets for the same 48 physics
substeps as at 10 Hz), after the same bounds clamp as the official guardrails and a per-step change limit of GUARD x
range (the official delta limit is 5 %), so an open-loop replay through `inspect-robots run` executes exactly the
recorded actions. A linear interpolation between the 10 Hz targets was tried first and changed the outcome of the
nominal episode.
"""
import argparse
import json
import math
import os
import sys
import time
import traceback
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(1, str(Path(__file__).resolve().parents[1]))
E = os.environ.get
HZ = float(E("MAG_HZ", "10"))
ENV_HZ = 20.0
SUB = round(ENV_HZ / HZ)
GUARD = 0.045
TASK = str(Path(__file__).resolve().parents[1] / "task.py")
TILT0, TILT1 = math.radians(float(E("MAG_TILT0", "60"))), math.radians(float(E("MAG_TILT1", "25")))
GX = float(E("MAG_GX", "0.008"))  # grasp point along the round from its centre toward the nose
SLIDE = float(E("MAG_SLIDE", "0.006"))  # how far the held round's base is slid under the lip front edge
BLADE_OPEN = float(E("MAG_BLADE_OPEN", "0.45"))  # gripper opening while one finger is used as a blade
BLADE_DEPTH = float(E("MAG_BLADE_DEPTH", "0.0008"))  # blade bottom below the lip underside while dragging
PLACE_DZ = float(E("MAG_PLACE_DZ", "-0.0012"))  # base-disc top vs lip underside (negative = below)
PICK_OPEN = float(E("MAG_PICK_OPEN", "0.3"))
HOLD_Z = float(E("MAG_HOLD_Z", "0.085"))
SNAP_EVERY = int(E("MAG_SNAP", "1"))


class Stop(Exception):
    pass


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_from(approach, close):
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(close, float) - z * (np.dot(close, z))
    x /= np.linalg.norm(x)
    return np.c_[x, np.cross(z, x), z]


BLADE_PHIS = [math.radians(float(v)) for v in E("MAG_BLADE_PHI", "0,15,30,40").split(",")]
HOLD_BETAS = [math.radians(float(v)) for v in E("MAG_HOLD_BETA", "0,20,35").split(",")]


def solve_side(side, p, r):
    from yam_arena import ik, station
    from yam_arena.constants import READY
    q, pe, re = ik.solve(p, ik.matrix_to_quat_xyzw(r), q_init=np.array(READY, float), base_pos=station.BASES[side], iters=600)
    m = float(np.degrees(np.minimum(q - ik.LOWER, ik.UPPER - q)).min())
    return (pe < 0.002 and re < math.radians(3)), m, q


def best(side, cands):
    """cands: [(label, [(p, r), ...])]; the candidate whose poses are all reachable with the largest joint margin."""
    out = None
    for lab, poses in cands:
        ms = [solve_side(side, p, r) for p, r in poses]
        if all(ok for ok, _, _ in ms):
            m = min(m for _, m, _ in ms)
            if out is None or m > out[1]:
                out = (lab, m, poses)
    return out


def hold_cands(mp, mr, top):
    fx = np.r_[mr[:2, 0], 0.0] / np.linalg.norm(mr[:2, 0])
    fy = np.cross([0, 0, 1.0], fx)
    g = np.array([mp[0], mp[1], top + HOLD_Z])
    # side: approach along -mag Y (from the left arm's side), fingers on the front and rear faces (tilt about the
    # magazine's Y axis is then resisted by normal forces, not by torsional pad friction); rear: approach along +X
    fam = {"side": (-fy, fx), "rear": (fx, fy)}
    out = []
    for name in E("MAG_HOLD_MODE", "side").split(","):
        ax, close = fam[name]
        for b in HOLD_BETAS:
            a = ax * math.cos(b) - np.array([0, 0, math.sin(b)])
            for s in (1, -1):
                r = rot_from(a, s * close)
                out.append((f"{name}_beta{math.degrees(b):.0f}_s{s}", [(g - a * 0.10, r), (g, r)]))
    return out


def blade_frame(mr, phi, s, opening):
    from yam_arena.demo import grip_rotation
    r = mr @ rot_y(-phi) @ grip_rotation(0.0 if s > 0 else math.pi)
    corners = [np.array([-(opening * 0.04695 + 0.0032), y, 0.0128]) for y in (0.0137, -0.0137)]
    c = min(corners, key=lambda v: (r @ v)[2])
    return r, c


def blade_cands(mp, mr, pts_m):
    out = []
    for phi in BLADE_PHIS:
        for s in (1, -1):
            r, c = blade_frame(mr, phi, s, BLADE_OPEN)
            out.append(((phi, s), [(mp + mr @ np.asarray(p) - r @ c, r) for p in pts_m]))
    return out


class Arm:
    def __init__(self, side):
        from yam_arena import station
        from yam_arena.constants import READY
        self.side, self.base = side, station.BASES[side]
        self.q = np.array(READY, float)
        self.meas = self.q.copy()
        self.integ = np.zeros(6)
        self.grip = 1.0
        self.cmd = np.r_[self.q, 1.0]
        self.ki, self.clip = float(E("MAG_KI", "0.15")), float(E("MAG_ICLIP", "0.10"))

    def solve(self, p, r, seed=None):
        from yam_arena import ik
        return ik.solve(p, ik.matrix_to_quat_xyzw(r), q_init=self.q if seed is None else seed, base_pos=self.base,
                        iters=300 if seed is None else 600)

    def command(self, q, grip):
        self.q = np.asarray(q, float)
        self.integ = np.clip(self.integ + self.ki * (self.q - self.meas), -self.clip, self.clip)
        self.grip = float(grip)
        self.cmd = np.r_[self.q + self.integ, self.grip]

    def tcp(self):
        from yam_arena import ik
        return ik.tcp_pose(self.q, self.base)

    def tcp_meas(self):
        from yam_arena import ik
        return ik.tcp_pose(self.meas, self.base)


class Runner:
    def __init__(self):
        from inspect_robots import Scene
        from yam_arena.inspect_embodiment import yam_arena_embodiment
        import demokit as magkit
        self.mk, self.Scene = magkit, Scene
        t0 = time.time()
        self.emb = yam_arena_embodiment(task=TASK, object_jitter=0.0, control_hz=ENV_HZ, cam_width=320, cam_height=176)
        self.kit_s = round(time.time() - t0, 1)
        print("KIT_START_S", self.kit_s, flush=True)

    def start(self, out, stage):
        self.out, self.stage = Path(out), stage
        self.out.mkdir(parents=True, exist_ok=True)
        self.rows, self.frames, self.steps, self.events, self.per_round = [], {}, 0, {}, []
        self.sent, self.clipped = [], 0
        t0 = time.time()
        seed = E("MAG_SEED")
        obs = self.emb.reset(self.Scene(id=f"mag-{stage}", instruction="load"), seed=None if seed is None else int(seed))
        self.reset_s = round(time.time() - t0, 1)
        self.left, self.right = Arm("left"), Arm("right")
        from yam_arena import ik
        self.lo, self.hi = ik.LOWER, ik.UPPER
        self.margin = {"left": np.full(6, 1e9), "right": np.full(6, 1e9)}
        self.snap(obs)
        self.wall0 = time.time()

    def S(self):
        return self.mk.STATE["view"].state()

    def snap(self, obs):
        if self.steps % SNAP_EVERY:
            return
        for cam, img in obs.images.items():
            self.frames.setdefault(cam, []).append(np.asarray(img))

    def step(self, label):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import STATE_KEY, pack
        lo, hi = self.emb._low, self.emb._high
        tgt = np.clip(pack(self.left.cmd, self.right.cmd), lo, hi)
        d = GUARD * (hi - lo)
        for k in range(1, SUB + 1):  # zero-order hold: the same target in every env step (as the 10 Hz development runs)
            last = self.sent[-1] if self.sent else tgt
            a = last + np.clip(tgt - last, -d, d)
            self.clipped += bool(np.any(np.abs(a - tgt) > 1e-12))
            self.sent.append(a)
            self.mk.STATE["hold"] = k < SUB
            res = self.emb.step(Action(data=a))
        jp = np.asarray(res.observation.state[STATE_KEY], float).ravel()
        self.left.meas, self.right.meas = jp[:6], jp[7:13]
        for a in (self.left, self.right):
            self.margin[a.side] = np.minimum(self.margin[a.side], np.degrees(np.minimum(a.meas - self.lo, self.hi - a.meas)))
        self.steps += 1
        self.snap(res.observation)
        st = self.mk.STATE["steps"][-1]
        row = dict(step=self.steps, label=label, grip=[self.left.grip, self.right.grip],
                   grip_meas=[round(float(jp[6]), 3), round(float(jp[13]), 3)],
                   q_left=np.round(jp[:6], 4).tolist(), q_right=np.round(jp[7:13], 4).tolist(), **st)
        self.rows.append(row)
        if self.steps % 20 == 0:
            print(json.dumps(row), flush=True)

    def wait(self, label, s):
        for _ in range(max(1, round(s * HZ))):
            self.step(label)

    def to(self, arm, p, r, grip, label):
        q, pe, re = arm.solve(p, r)
        if pe > 0.002 or re > math.radians(3):
            raise Stop(f"{label}: unreachable ({pe * 1000:.1f} mm, {math.degrees(re):.1f} deg) at {np.round(p, 4)}")
        arm.command(q, grip)

    def go(self, arm, label, duration, p1, r1, grip1=None, until=None):
        from yam_arena import ik
        p0, r0 = arm.tcp()
        g0 = arm.grip
        g1 = g0 if grip1 is None else grip1
        n = max(1, round(duration * HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            try:
                self.to(arm, p0 + s * (np.asarray(p1) - p0), ik.slerp_matrix(r0, r1, s), g0 + s * (g1 - g0), label)
            except Stop:
                if not label.endswith(("_up", "away", "_open")):
                    raise
                self.events.setdefault("soft_unreachable", []).append(label)
                arm.command(arm.q, g0 + s * (g1 - g0))
            self.step(label)
            if until is not None and until():
                return True
        return False

    def go_joint(self, arm, label, duration, p1, r1, grip1=None):
        from yam_arena import ik
        from yam_arena.constants import READY
        q1, pe, re = arm.solve(p1, r1, seed=np.array(READY, float))
        if pe > 0.002 or re > math.radians(3):
            raise Stop(f"{label}: unreachable ({pe * 1000:.1f} mm, {math.degrees(re):.1f} deg)")
        q0, g0 = arm.q.copy(), arm.grip
        g1 = g0 if grip1 is None else grip1
        n = max(1, round(duration * HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            arm.command(q0 + s * (q1 - q0), g0 + s * (g1 - g0))
            self.step(label)

    # -- geometry helpers
    def mag(self):
        s = self.S()
        return s["mag"][0], s["mag"][1]

    def down(self, yaw=0.0):
        from yam_arena.demo import grip_rotation
        return grip_rotation(yaw)

    def held(self, name):
        p, r = self.S()[name][:2]
        tp, tr = self.right.tcp_meas()
        return tr.T @ (p - tp), tr.T @ r

    def tcp_for_round(self, c_w, r_w, rel):
        p_rt, r_rt = rel
        r_t = r_w @ r_rt.T
        return c_w - r_t @ p_rt, r_t

    def finger_link(self, arm, link):
        env = self.emb._env.unwrapped
        robot = env.scene[f"{arm}_robot"]
        pose = robot.data.body_link_pose_w.torch[0].cpu().numpy()[list(robot.body_names).index(link)]
        return pose[:3] - env.scene.env_origins[0].cpu().numpy()

    # -- phases
    def hold_mag(self, lift=0.0):
        mp, mr = self.mag()
        b = best("left", hold_cands(mp, mr, self.mk.TOP))
        if b is None:
            raise Stop("left: no reachable magazine grasp")
        self.events["hold_grasp"] = dict(cand=b[0], margin_deg=round(b[1], 1))
        (ph, r), (g, _) = b[2]
        self.go_joint(self.left, "l_hover", 2.5, ph, r, 1.0)
        self.go(self.left, "l_in", 2.0, g, r)
        self.go(self.left, "l_close", 0.8, g, r, 0.0)
        self.wait("l_squeeze", 0.5)
        if lift:
            self.go(self.left, "l_lift", 1.0, g + [0, 0, lift], r)
            self.wait("l_hold_air", 0.5)

    def pick(self, name):
        mk = self.mk
        p, r = self.S()[name][:2]
        ax = r[:, 0]
        yaw = math.atan2(ax[1], ax[0])
        g = p + ax * GX
        zt = mk.TOP + 0.0015 + 0.0128
        rr = self.down(yaw)
        self.go_joint(self.right, "r_hover", 2.0, np.array([g[0], g[1], zt + 0.06]), rr, PICK_OPEN)
        self.go(self.right, "r_descend", 1.5, np.array([g[0], g[1], zt]), rr)
        self.wait("r_settle", 1.0)
        self.go(self.right, "r_close", 0.8, np.array([g[0], g[1], zt]), rr, 0.0)
        self.wait("r_squeeze", 0.3)
        self.go(self.right, "r_lift", 1.0, np.array([g[0], g[1], zt + 0.07]), rr)
        self.wait("r_lifted", 0.3)
        rel = self.held(name)
        self.events.setdefault("grasp", []).append(dict(round=name, grip_meas=self.rows[-1]["grip_meas"][1],
                                                        rel_mm=np.round(rel[0] * 1000, 1).tolist()))
        if self.rows[-1]["grip_meas"][1] < 0.02:
            raise Stop(f"{name}: grasp missed (gripper closed fully)")
        return rel

    def round_world(self, b_m, tilt):
        mp, mr = self.mag()
        a = np.array([math.cos(tilt), 0.0, math.sin(tilt)])
        c = mp + mr @ (np.asarray(b_m) + a * self.mk.OAL / 2)
        return c, mr @ rot_y(-tilt)

    def base_in_mag(self, name):
        c, a = self.mk.round_in_mag(self.S(), name)
        return c - a * self.mk.OAL / 2

    def servo(self, name, rel, label, duration, b0, b1, t0, t1, slip, gain=0.5):
        """Move the held round's base from b0 to b1 (mag frame) and its tilt from t0 to t1, correcting x/y of the base
        from the measured round pose (closed loop on simulator state)."""
        from yam_arena import ik
        corr = np.zeros(3)
        n = max(1, round(duration * HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            b = np.asarray(b0) + s * (np.asarray(b1) - np.asarray(b0))
            t = t0 + s * (t1 - t0)
            err = b - self.base_in_mag(name)
            corr[:2] = np.clip(corr[:2] + gain * err[:2], -0.006, 0.006)
            c, rw = self.round_world(b + corr, t)
            self.to(self.right, *self.tcp_for_round(c, rw, rel), self.right.grip, label)
            self.step(label)
            p, _ = self.held(name)
            slip.append(float(np.linalg.norm(p - rel[0])))
            self.rows[-1]["base_mm"] = np.round(self.base_in_mag(name) * 1000, 2).tolist()
            self.rows[-1]["base_tgt_mm"] = np.round(b * 1000, 2).tolist()
        return corr

    def insert(self, name, rel):
        """Set the held round base-first into the open front of the mouth (light contact only), then let go."""
        mk = self.mk
        bx0 = mk.X_LF + mk.R_CASE * math.sin(TILT0) + 0.0008
        slip = []
        hi = (bx0, 0.0, mk.Z_L + 0.012)
        lo = (bx0, 0.0, mk.Z_L - mk.R_CASE * math.cos(TILT0) + PLACE_DZ)  # base centre
        c, rw = self.round_world(hi, TILT0)
        self.go_joint(self.right, "r_over", 2.0, *self.tcp_for_round(c, rw, rel))
        self.servo(name, rel, "r_align", 1.0, hi, hi, TILT0, TILT0, slip)
        self.servo(name, rel, "r_place", 3.0, hi, lo, TILT0, TILT0, slip)
        self.servo(name, rel, "r_place_hold", float(E("MAG_PLACE_HOLD", "1.5")), lo, lo, TILT0, TILT0, slip)
        if SLIDE > 0:
            back = (mk.X_LF - SLIDE, 0.0, mk.Z_L - mk.R_CASE * math.cos(TILT1) + PLACE_DZ)
            self.servo(name, rel, "r_slide", 2.0, lo, back, TILT0, TILT1, slip)
            self.servo(name, rel, "r_slide_hold", 0.3, back, back, TILT1, TILT1, slip)
        st = mk.round_status(self.S(), name)
        st["base_mm"] = np.round(self.base_in_mag(name) * 1000, 2).tolist()
        self.go(self.right, "r_release", 0.6, *self.right.tcp(), BLADE_OPEN)
        p, r = self.right.tcp()
        self.go(self.right, "r_up", 0.8, p + [0, 0, 0.03], r)
        st["after_release_base_mm"] = np.round(self.base_in_mag(name) * 1000, 2).tolist()
        return dict(slip_mm_max=round(max(slip) * 1000, 2), slip_mm_end=round(slip[-1] * 1000, 2), after_insert=st)

    def blade_corner(self, phi, sgn):
        from yam_arena import ik
        mp, mr = self.mag()
        r, c = blade_frame(mr, phi, sgn, self.right.grip)
        tp, tr = self.right.tcp_meas()
        return mr.T @ (tp + tr @ c - mp)

    def blade_path(self, label, duration, pts, phi, sgn, gain=0.4):
        """Blade corner along mag-frame points (linear), re-planned every step on the current magazine pose, with
        x/y correction from the measured corner."""
        from yam_arena import ik
        corr = np.zeros(3)
        n = max(1, round(duration * HZ))
        p0, p1 = np.asarray(pts[0], float), np.asarray(pts[1], float)
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            tgt = p0 + s * (p1 - p0)
            err = tgt - self.blade_corner(phi, sgn)
            corr[:2] = np.clip(corr[:2] + gain * err[:2], -0.004, 0.004)
            mp, mr = self.mag()
            r, c = blade_frame(mr, phi, sgn, BLADE_OPEN)
            self.to(self.right, mp + mr @ (tgt + corr) - r @ c, r, BLADE_OPEN, label)
            self.step(label)

    def blade(self, name, tries=3):
        mk = self.mk
        out = []
        for t in range(tries):
            mp, mr = self.mag()
            st = mk.round_status(self.S(), name)
            if st["loaded"]:
                break
            base = self.base_in_mag(name)
            x0 = base[0] + float(E("MAG_BLADE_X0", "0.003"))
            x1 = mk.X_REAR + float(E("MAG_BLADE_END", "0.0045"))
            zd, zl = mk.Z_L - float(E("MAG_BLADE_DOWN", "0.0015")), mk.Z_L - BLADE_DEPTH
            pts = [(x0, 0.0, mk.Z_L + 0.012), (x0, 0.0, zd), (x1, 0.0, zl), (x1, 0.0, mk.Z_L + 0.03)]
            b = best("right", blade_cands(mp, mr, pts))
            if b is None:
                raise Stop("right: no reachable blade pose")
            phi, sgn = b[0]
            self.events.setdefault("blade_pose", []).append(dict(phi_deg=round(math.degrees(phi)), sign=sgn,
                                                                 margin_deg=round(b[1], 1), base_mm=np.round(base * 1000, 1).tolist()))
            self.go_joint(self.right, "b_over", 1.5, b[2][0][0], b[2][0][1], BLADE_OPEN)
            self.blade_path("b_align", 0.6, pts[0:1] * 2, phi, sgn)
            self.blade_path("b_press", 1.5, pts[0:2], phi, sgn)
            self.blade_path("b_drag", max(1.5, (x0 - x1) / 0.006), pts[1:3], phi, sgn)
            self.blade_path("b_hold", 0.3, [pts[2]] * 2, phi, sgn)
            self.blade_path("b_up", 1.0, pts[2:4], phi, sgn)
            self.wait("b_settle", 0.5)
            out.append(mk.round_status(self.S(), name))
        return out

    def load_round(self, i):
        name = self.mk.ROUNDS[i].name
        k0, t0 = len(self.rows), self.mk.STATE["sim_t"]
        rel = self.pick(name)
        ins = self.insert(name, rel)
        bl = self.blade(name)
        st = self.mk.round_status(self.S(), name)
        rows = self.rows[k0:]
        pr = dict(round=i, loaded=st["loaded"], status=st, time_s=round(self.mk.STATE["sim_t"] - t0, 2), **ins,
                  blade_tries=len(bl), push_r_peak_N=max(r["push_r_max_N"] for r in rows),
                  grip_r_peak_N=max(r["grip_r_max_N"] for r in rows), grip_l_peak_N=max(r["grip_l_max_N"] for r in rows),
                  spring_peak_N=max(r["spring_max_N"] for r in rows), spring_end_N=rows[-1]["spring_N"],
                  mag_tilt_max_deg=max(r["mag_tilt_deg"] for r in rows),
                  mag_shift_max_mm=round(max(float(np.hypot(*r["mag_xy_mm"])) for r in rows), 2),
                  margin_right_min_deg=round(float(np.min([np.degrees(np.minimum(np.array(r["q_right"]) - self.lo,
                                                                                    self.hi - np.array(r["q_right"])))
                                                            for r in rows])), 1),
                  margin_left_min_deg=round(float(np.min([np.degrees(np.minimum(np.array(r["q_left"]) - self.lo,
                                                                                   self.hi - np.array(r["q_left"])))
                                                           for r in rows])), 1))
        self.per_round.append(pr)
        print("ROUND", json.dumps(pr, default=float), flush=True)
        return st["loaded"]

    def away(self):
        from yam_arena.constants import READY
        for a, lab in ((self.right, "r_away"), (self.left, "l_away")):
            if a.grip < 0.99:
                p, r = a.tcp()
                self.go(a, lab + "_open", 0.6, p, r, 1.0)
            p, r = a.tcp()
            if lab.startswith("l"):
                self.go(a, lab, 1.0, p - [0.08, 0, 0], r)
            else:
                self.go(a, lab, 1.0, p + [0, 0, 0.06], r)
        self.wait("rest", 1.5)

    def run_load(self, two=True, lift=0.0, n=None):
        self.wait("settle", 0.5)
        if two:
            self.hold_mag(lift)
        n = len(self.mk.ROUNDS) if n is None else n
        for i in range(n):
            ok = self.load_round(i)
            if not ok and E("MAG_STOP_ON_FAIL", "1") == "1":
                self.events["stopped_after_round"] = i
                break
        self.away()

    def round_pose_reachable(self, b_m, tilts, rel, centre=None):
        for t in tilts:
            c, rw = self.round_world(b_m, t)
            if centre is not None:
                c = centre
            p, r = self.tcp_for_round(c, rw, rel)
            ok, m, _ = solve_side("right", p, r)
            if ok:
                return t, p, r
        raise Stop(f"no reachable round pose for tilts {tilts}")

    def run_drop(self):
        mk = self.mk
        self.wait("settle", 0.5)
        self.hold_mag()
        mp, mr = self.mag()
        for i, (tilt, lab) in enumerate(((0.0, "level"), (math.radians(-80), "nose_down"), (math.radians(80), "base_down"))):
            name = mk.ROUNDS[i].name
            rel = self.pick(name)
            if tilt:  # near-vertical rounds over the open front of the mouth
                cen = mp + mr @ np.array([(mk.X_LF + mk.X_FRONT) / 2, 0.0, mk.H + 0.03 + mk.OAL / 2])
                t, p, r = self.round_pose_reachable((0.0, 0.0, 0.0), [tilt, tilt * 0.8, tilt * 0.6], rel, cen)
            else:
                t, p, r = self.round_pose_reachable((mk.X_REAR + 0.001, 0.0, mk.H + 0.03), [0.0, 0.15, 0.3, 0.45, 0.6, 0.8], rel)
            self.events[f"drop_{lab}_tilt_deg"] = round(math.degrees(t), 1)
            self.go_joint(self.right, f"drop_{lab}_over", 2.0, p, r)
            p, r = self.right.tcp()
            self.go(self.right, f"drop_{lab}_open", 0.4, p, r, 1.0)
            self.go(self.right, f"drop_{lab}_up", 0.8, p + [0, 0, 0.012], r)
            self.wait(f"drop_{lab}_settle", 1.0)
            self.events[f"drop_{lab}"] = mk.round_status(self.S(), name)
        self.away()

    def run_pushtop(self):
        mk = self.mk
        self.wait("settle", 0.5)
        self.hold_mag()
        name = mk.ROUNDS[0].name
        rel = self.pick(name)
        t, p, r = self.round_pose_reachable((mk.X_REAR + 0.0005, 0.0, mk.H + mk.R_CASE + 0.004), [0.0, 0.1, 0.2, 0.3], rel)
        self.events["pushtop_place_tilt_deg"] = round(math.degrees(t), 1)
        self.go_joint(self.right, "pt_over", 2.0, p, r)
        p, r = self.right.tcp()
        self.go(self.right, "pt_open", 0.5, p, r, 1.0)
        self.go(self.right, "pt_up", 0.6, p + [0, 0, 0.012], r)
        self.wait("pt_settle", 0.5)
        c, a = mk.round_in_mag(self.S(), name)
        mp, mr = self.mag()
        top = mp + mr @ np.array([c[0], 0.0, c[2] + mk.R_CASE])
        r = None
        for phi in BLADE_PHIS:
            rr = mr @ rot_y(-phi) @ self.down(0.0)
            zt = top - rr @ np.array([0, 0, 0.0128])
            if solve_side("right", zt - [0, 0, 0.025], rr)[0] and solve_side("right", zt + [0, 0, 0.02], rr)[0]:
                r = rr
                break
        if r is None:
            raise Stop("pushtop: no reachable closed-finger press pose")
        self.go_joint(self.right, "pt_close", 1.5, zt + [0, 0, 0.02], r, 0.0)
        self.go(self.right, "pt_touch", 1.0, zt, r)
        self.go(self.right, "pt_press", 3.0, zt - [0, 0, 0.025], r)
        self.wait("pt_hold", 1.0)
        self.events["pushtop_pressed"] = mk.round_status(self.S(), name)
        self.events["pushtop_push_peak_N"] = max(rw_["push_r_max_N"] for rw_ in self.rows)
        self.go(self.right, "pt_up", 1.0, zt + [0, 0, 0.04], r)
        self.wait("pt_settle2", 1.0)
        self.events["pushtop_after"] = mk.round_status(self.S(), name)
        self.away()

    def run_stack(self):
        """Physics check (MAG_PRELOAD rounds spawned in the magazine): settle, then the left hand holds the magazine
        and the right hand's closed fingertips press the top round down between the lips and let go (thumb check)."""
        mk = self.mk
        self.wait("settle", 3.0)
        s = self.S()
        self.events["settled"] = {r.name: mk.round_status(s, r.name) for r in mk.ROUNDS[:mk.PRELOAD]}
        self.events["settled_score"] = mk.score(s)["score"]
        self.events["settle_round_vmax_last1s"] = max(r["round_vmax"] for r in self.rows[-10:])
        self.events["follower_mm"] = self.rows[-1]["follower_mm"]
        self.events["spring_N"] = self.rows[-1]["spring_N"]
        self.hold_mag()
        mp, mr = self.mag()
        top = mp + mr @ np.array([mk.X_REAR + mk.OAL / 2, 0.0, mk.H + 0.001])
        best_r = None
        for phi in BLADE_PHIS:
            rr = mr @ rot_y(-phi) @ self.down(0.0)
            r_, c = blade_frame(mr, phi, 1, BLADE_OPEN)
            p_hi, p_lo = top + [0, 0, 0.015] - r_ @ c, top - [0, 0, 0.008] - r_ @ c
            if solve_side("right", p_hi, r_)[0] and solve_side("right", p_lo, r_)[0]:
                best_r = (phi, r_, c, p_hi, p_lo)
                break
        if best_r is None:
            raise Stop("stack: no reachable press pose")
        phi, r_, c, p_hi, p_lo = best_r
        self.go_joint(self.right, "th_over", 2.0, p_hi, r_, BLADE_OPEN)
        self.go(self.right, "th_press", 3.0, p_lo, r_)
        self.wait("th_hold", 1.0)
        self.events["thumb_pressed"] = dict(follower_mm=self.rows[-1]["follower_mm"], spring_N=self.rows[-1]["spring_N"],
                                            push_peak_N=max(r["push_r_max_N"] for r in self.rows[-40:]))
        self.go(self.right, "th_up", 1.5, p_hi, r_)
        self.wait("th_settle", 2.0)
        s = self.S()
        self.events["after_thumb"] = {r.name: mk.round_status(s, r.name) for r in mk.ROUNDS[:mk.PRELOAD]}
        self.events["after_thumb_score"] = mk.score(s)["score"]
        self.events["after_thumb_follower_mm"] = self.rows[-1]["follower_mm"]
        self.away()

    def run_smoke(self):
        self.wait("idle", 2.0)
        r = self.down(0.0)
        p, _ = self.right.tcp()
        self.go_joint(self.right, "probe", 1.5, np.array([0.35, -0.1, self.mk.TOP + 0.15]), r, BLADE_OPEN)
        tl, tr = self.finger_link("right", "tip_left"), self.finger_link("right", "tip_right")
        tp, _ = self.right.tcp_meas()
        self.events["finger_y_rel_tcp_mm"] = dict(tip_left=round((tl[1] - tp[1]) * 1000, 1),
                                                   tip_right=round((tr[1] - tp[1]) * 1000, 1))
        self.wait("idle2", 1.0)

    def finish(self, err):
        import imageio
        mk = self.mk
        env = self.emb._env
        s = self.S()
        sc = mk.score(s)
        speeds = [float(np.linalg.norm(s[n][2])) for n in mk.NAMES]
        clear = mk.hands_clear(env)
        rest = max(speeds) < 0.01
        full = sc["score"] >= 1.0 and clear >= 0.03 and rest
        res = dict(stage=self.stage, error=None if err is None else repr(err), steps=self.steps, kit_s=self.kit_s,
                   reset_s=self.reset_s, step_wall_s=round((time.time() - self.wall0) / max(1, self.steps), 2),
                   sim_s=round(mk.STATE["sim_t"], 2), score=sc, full_score_conditions=dict(hands_clear_m=clear, at_rest=rest,
                                                                                          full=full),
                   per_round=self.per_round, events=self.events,
                   joint_min_margin_deg={k: np.round(v, 1).tolist() for k, v in self.margin.items()},
                   params=dict(spring=mk.SPRING, F1=mk.F1, F17=mk.F17, k_N_per_m=round(mk.K_SPRING, 1),
                               f_top_N=round(mk.F_TOP, 2), pitch_mm=round(mk.PITCH * 1000, 2), round_g=mk.ROUND_G,
                               physics_dt=mk.PHYSICS_DT, hz=HZ, grip_n=mk.GRIP_N, pad=mk.PAD_FRICTION, mu_round=mk.MU_ROUND,
                               mu_mag=mk.MU_MAG, lip_gap=mk.LIP_GAP, lip_len=mk.LIP_LEN, front_drop=mk.FRONT_DROP,
                               side_drop=mk.SIDE_DROP, w_in=mk.W_IN, tilt0=math.degrees(TILT0), tilt1=math.degrees(TILT1),
                               gx=GX, slide=SLIDE, blade_open=BLADE_OPEN, blade_depth=BLADE_DEPTH))
        (self.out / "result.json").write_text(json.dumps(res, indent=2, default=float))
        (self.out / "rows.jsonl").write_text("\n".join(json.dumps(r, default=float) for r in self.rows) + "\n")
        for cam, fs in self.frames.items():
            imageio.mimsave(self.out / f"{cam}.mp4", fs, fps=HZ / SNAP_EVERY, codec="libx264",
                            output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])
            imageio.imwrite(self.out / f"last_{cam}.png", fs[-1])
            imageio.imwrite(self.out / f"first_{cam}.png", fs[0])
        print("RESULT", json.dumps(dict(stage=self.stage, error=res["error"], score=sc["score"], loaded=sc["loaded"],
                                        full=full, per_round=[(p["round"], p["loaded"], p["time_s"]) for p in self.per_round],
                                        events=self.events, step_wall_s=res["step_wall_s"]), default=float), flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--stages", default="smoke")
    a = ap.parse_args()
    r = Runner()
    for i, st in enumerate(a.stages.split(",")):
        err = None
        try:
            r.start(Path(a.out) / f"{i}-{st}", st)
            if st == "smoke":
                r.run_smoke()
            elif st == "load":
                r.run_load()
            elif st.startswith("load"):
                r.run_load(n=int(st[4:]))
            elif st == "one":
                r.run_load(two=False)
            elif st == "air":
                r.run_load(lift=0.02, n=2)
            elif st == "stack":
                r.run_stack()
            elif st == "drop":
                r.run_drop()
            elif st == "pushtop":
                r.run_pushtop()
        except Exception as e:  # noqa: BLE001  recorded
            err = e
            traceback.print_exc()
        try:
            r.finish(err)
        except Exception:  # noqa: BLE001
            traceback.print_exc()
    os._exit(0)


if __name__ == "__main__":
    main()
