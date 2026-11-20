"""tool1 arm demo: python drv.py run_tool.py OUT
Two YAM arms load ONE dummy round into the fixture-held magazine with the printed loading tool. Robot commands only
(joint targets, gripper 0..1); the script reads simulator state to plan and servo. Nothing is teleported or edited.
P (tool arm): picks the tool from its place on the table by the rear fin, puts it on the magazine along the tube axis,
presses it down until the rails sit on the lip tips (follower down 5 mm) and holds it; later lifts it off and parks it.
N (round arm): picks the round leaning TOOL_TILT forward (so with the gripper vertical the round is nose-up), pushes it
along its own axis over the front wall and under the lips, lets go, pushes the nose with its closed fingertips to the
tongue, and after the tool is lifted pushes it to the rear wall."""
import json
import math
import os
import sys
import time
import traceback
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import plan as pl  # noqa: E402
import reach  # noqa: E402
import run_p0 as base  # noqa: E402
import tplan as tp  # noqa: E402
from run_p0 import Stop  # noqa: E402

E = os.environ.get
HZ = base.HZ
NSIDE = E("ARM_N", "right")
CAM = [int(v) for v in E("MAG_CAM", "320,176").split(",")]
GAIN = float(E("ARM_GAIN", "0.1"))
PICK_OPEN = float(E("ARM_PICK_OPEN", "0.3"))
N_HOLD = float(E("ARM_N_HOLD", "0.06"))
V_IN = float(E("TOOL_V_IN", "0.003"))
V_TOOL = float(E("TOOL_V_TOOL", "0.008"))
F_RAIL = float(E("TOOL_F_RAIL", "5"))
LEAD = float(E("TOOL_LEAD", "0.045"))  # the press command may run this far below the tool (the arm gives about 0.4 N per mm)
F_FINGER = float(E("TOOL_F_FINGER", "40"))
U_SEAT = float(E("TOOL_U_SEAT", "0.5"))  # mm above the stop that the hold controller keeps (the ring bottom sits at about 0.3)
KC = float(E("TOOL_KC", "0.00005"))  # m per N per step, sideways compliance of the tool target
NOSE_V = float(E("ARM_NOSE_V", "0.005"))
NOSE_F = float(E("ARM_NOSE_F", "18"))
NOSE_GRIP = float(E("ARM_NOSE_GAP", "0.006")) / 0.0939  # fingertips this far apart for the nose push: the round nose sits between them
F_ABORT = float(E("ARM_F_ABORT", "45"))
U = tp.U


class T(base.Runner):
    def __init__(self):
        from inspect_robots import Scene
        from yam_arena.inspect_embodiment import yam_arena_embodiment
        import demokit as magkit
        self.mk, self.Scene = magkit, Scene
        t0 = time.time()
        self.emb = yam_arena_embodiment(task=base.TASK, object_jitter=0.0, control_hz=base.ENV_HZ, cam_width=CAM[0], cam_height=CAM[1])
        self.kit_s = round(time.time() - t0, 1)
        print("KIT_START_S", self.kit_s, flush=True)
        self.name = magkit.ROUNDS[0].name
        self.phase = {}
        self.N = self.P = None

    # -- state
    def rnd(self):
        mk = self.mk
        c, a = mk.round_in_mag(self.S(), self.name)
        return dict(c=c, a=a, b=c - a * mk.OAL / 2, tilt=math.asin(float(np.clip(a[2], -1, 1))), yaw=math.atan2(a[1], a[0]))

    def gap(self, r=None):
        r = r or self.rnd()
        return float(r["b"][0] - self.mk.x_rear(r["b"][2] + self.mk.R_CASE))

    def step(self, label):
        super().step(label)
        r = self.rnd()
        row = self.rows[-1]
        row["rnd"] = np.round(np.r_[r["b"] * 1000, math.degrees(r["tilt"]), math.degrees(r["yaw"]), r["c"] * 1000], 3).tolist()
        row["gap_mm"] = round(self.gap(r) * 1000, 2)
        row["margin"] = [np.round(np.degrees(np.minimum(a.meas - self.lo, self.hi - a.meas)), 1).tolist() for a in (self.left, self.right)]
        if self.N is not None:
            mp, mr = self.mag()
            for a, k in ((self.N, "n"), (self.P, "p")):
                row[k + "_tcp"] = np.round(mr.T @ (a.tcp_meas()[0] - mp) * 1000, 2).tolist()
        if not hasattr(self, "_live"):
            self._live = open(self.out / "rows_live.jsonl", "w")
        self._live.write(json.dumps(row, default=float) + "\n")
        self._live.flush()

    def mark(self, key):
        self.phase[key] = len(self.rows)
        print("PHASE", key, len(self.rows), flush=True)

    def f(self, arm, what="rnd"):
        return np.array(self.rows[-1].get(f"f{arm.side[0]}_{what}_N", (0, 0, 0)), float)

    # -- motion primitives (generators: one yield per control step)
    def par(self, *gens):
        gens = [g for g in gens if g is not None]
        while gens:
            labs = []
            for g in list(gens):
                try:
                    labs.append(next(g))
                except StopIteration:
                    gens.remove(g)
            if labs:
                self.step("|".join(labs))

    def g_go(self, arm, label, dur, p1, r1, grip1=None):
        from yam_arena import ik
        p0, r0 = arm.tcp()
        g0 = arm.grip
        g1 = g0 if grip1 is None else grip1
        n = max(1, round(dur * HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.to(arm, p0 + s * (np.asarray(p1) - p0), ik.slerp_matrix(r0, r1, s), g0 + s * (g1 - g0), label)
            yield label

    def g_goq(self, arm, label, dur, q1, grip1=None):
        from yam_arena import ik
        q0, g0 = arm.q.copy(), arm.grip
        g1 = g0 if grip1 is None else grip1
        n = max(1, round(dur * HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            arm.command(q0 + s * (np.asarray(q1) - q0), g0 + s * (g1 - g0))
            yield label

    def g_wait(self, label, dur):
        for _ in range(max(1, round(dur * HZ))):
            yield label

    # -- plan
    def plan(self):
        mk = self.mk
        self.N, self.P = (self.right, self.left) if NSIDE == "right" else (self.left, self.right)
        mp, mr = self.mag()
        s = self.S()
        best = {}
        rp = s[self.name][0]
        ryaw = math.atan2(s[self.name][1][1, 0], s[self.name][1][0, 0])
        for key, arm, fn in (("P", self.P, lambda psi: tp.p_poses(mp, mr, s["tool"][0], s["tool"][1], psi)),
                             ("N", self.N, lambda psi: tp.n_poses(mp, mr, rp, ryaw, mk.TOP, psi))):
            for psi in ([math.radians(float(E("ARM_PSI_" + key)))] if E("ARM_PSI_" + key) else [0.0, math.pi]):
                d = fn(psi)
                b = reach.path_ik(arm.side, list(d.values()))
                if b and (key not in best or b[0] > best[key][0]):
                    best[key] = (b[0], b[1], dict(zip(d, b[2])), psi, d)
            if key not in best:
                raise Stop(f"planned path unreachable ({key})")
        self.psiP, self.psiN = best["P"][3], best["N"][3]
        self.qP, self.qN, self.dP, self.dN = best["P"][2], best["N"][2], best["P"][4], best["N"][4]
        self.events["plan"] = dict(N=self.N.side, P=self.P.side, psiN_deg=round(math.degrees(self.psiN)), psiP_deg=round(math.degrees(self.psiP)),
                                   N_margin_deg=np.round(best["N"][1], 1).tolist(), P_margin_deg=np.round(best["P"][1], 1).tolist(),
                                   tool_start=dict(xy=np.round(s["tool"][0][:2], 4).tolist(), yaw_deg=round(math.degrees(math.atan2(s["tool"][1][1, 0], s["tool"][1][0, 0])), 2)),
                                   round_start=dict(xy=np.round(rp[:2], 4).tolist(), yaw_deg=round(math.degrees(ryaw), 2)))
        print("PLAN", json.dumps(self.events["plan"]), flush=True)

    # -- P: tool
    def tool_rel(self):
        s = self.S()
        mp, mr = s["mag"][:2]
        return mr.T @ (s["tool"][0] - mp) - tp.TOOL_O, mr.T @ s["tool"][1]

    def p_cmd(self, r, label):
        mp, mr = self.mag()
        self.to(self.P, mp + mr @ (tp.TOOL_O + U * r + self.tcorr + self.gp), mr @ self.gr, 0.0, label)

    def p_seq(self):
        P, d = self.P, self.dP
        self.mark("p_pick")
        yield from self.g_goq(P, "p_hover", 2.5, self.qP["hover"], tp.P_GRIP_OPEN)
        yield from self.g_wait("p_hover_settle", 1.0)
        h, g, c = d["hover"][0], d["grasp"][0], np.zeros(3)
        for _ in range(8):  # remove the measured position error before the fingers go down over the fin
            c = c + 0.5 * (h - P.tcp_meas()[0])
            self.to(P, h + c, d["hover"][1], tp.P_GRIP_OPEN, "p_hover_trim")
            yield "p_hover_trim"
        n = round(3.0 * HZ)
        for k in range(1, n + 1):  # down along the gripper axis, sideways error corrected from the measured TCP
            t = h + (g - h) * k / n
            e = t - P.tcp_meas()[0]
            a = d["grasp"][1][:, 2]
            c = np.clip(c + 0.2 * (e - a * float(a @ e)), -0.01, 0.01)
            self.to(P, t + c, d["grasp"][1], tp.P_GRIP_OPEN, "p_descend")
            yield "p_descend"
        yield from self.g_wait("p_settle", 0.8)
        yield from self.g_go(P, "p_trim", 0.5, P.tcp()[0] + (g - P.tcp_meas()[0]), d["grasp"][1])
        yield from self.g_wait("p_trim_wait", 0.5)
        yield from self.g_go(P, "p_close", 0.8, *P.tcp(), 0.0)
        yield from self.g_wait("p_squeeze", 0.4)
        s = self.S()
        pm, rm = P.tcp_meas()
        self.gp, self.gr = s["tool"][1].T @ (pm - s["tool"][0]), s["tool"][1].T @ rm
        self.tcorr = np.zeros(3)
        self.events["tool_grasp"] = dict(tcp_in_tool_mm=np.round(self.gp * 1000, 2).tolist(), planned_mm=np.round(tp.p_tcp_tool(self.psiP) * 1000, 2).tolist(),
                                         grip_meas=self.rows[-1]["grip_meas"][0 if P.side == "left" else 1], finger_N=self.rows[-1].get(f"f{P.side[0]}_tip_N"))
        print("TOOL_GRASP", json.dumps(self.events["tool_grasp"]), flush=True)
        self.mark("p_carry")
        p0, r0 = P.tcp()
        yield from self.g_go(P, "p_lift", 2.0, np.array([p0[0], p0[1], d["lift"][0][2]]), r0)
        yield from self.g_go(P, "p_above", 2.5, *d["above"])
        rel, _ = self.tool_rel()
        if np.linalg.norm(rel - U * tp.R_OVER) > 0.03:
            raise Stop(f"tool not held (tool {np.round(rel * 1000, 1).tolist()} mm from its seat)")
        self.mark("p_insert")
        r = float(U @ rel)
        while True:  # down the tube axis; the tool target is corrected from the measured tool pose until the ring is on
            rel, rr = self.tool_rel()
            u = float(U @ rel)
            e = U * r - rel
            if u > 0.050:
                self.tcorr = np.clip(self.tcorr + 0.15 * e, -0.008, 0.008)
            else:
                self.comply()
            if r <= 0.008:
                break
            r = max(0.008, r - V_TOOL / HZ)
            self.p_cmd(r, "p_insert")
            yield "p_insert"
        self.events["tool_on"] = dict(tool_mm=self.rows[-1].get("tool_mm"), ypr_deg=self.rows[-1].get("tool_ypr_deg"), tcorr_mm=np.round(self.tcorr * 1000, 2).tolist())
        print("TOOL_ON", json.dumps(self.events["tool_on"]), flush=True)
        self.mark("p_press")
        self.r_cmd, self.p_done, self.p_seated = r, False, 0
        yield from self.g_hold()

    def comply(self):
        fm = np.array(self.rows[-1].get("tool_mag_N", (0, 0, 0)), float)
        self.tcorr = np.clip(self.tcorr + KC * (fm - U * float(U @ fm)), -0.008, 0.008)

    def g_hold(self):
        """press and hold: the command goes down the axis while the tool is above its seat, and comes back when the lip
        tips carry more than F_RAIL"""
        while not self.p_done:
            row = self.rows[-1]
            u = row["tool_u_mm"]
            fu = float(U @ np.array(row.get("tool_mag_N", (0, 0, 0)), float))
            if max(row.get(f"f{self.P.side[0]}_tip_N", (0, 0))) > F_FINGER:  # on the stop and a finger near its limit: ease the push
                self.r_cmd += 0.0003
            elif u > U_SEAT:
                self.r_cmd = max(u / 1000 - LEAD, self.r_cmd - min(0.0015, max(0.0006, 0.0005 * u)))
            self.p_seated = self.p_seated + 1 if u < U_SEAT + 0.25 else 0
            self.comply()
            self.p_cmd(self.r_cmd, "p_hold")
            yield "p_hold"

    def g_lift(self):
        self.mark("p_lift_off")
        r = self.r_cmd
        while r < tp.R_OVER:
            r = min(tp.R_OVER, r + 0.010 / HZ)
            if self.rows[-1]["tool_u_mm"] < 50:
                self.comply()
            self.p_cmd(r, "p_lift_off")
            yield "p_lift_off"
        yield from self.g_go(self.P, "p_park", 2.0, self.P.tcp()[0] + np.array([0.0, 0.10, 0.0]) + tp.PARK, self.P.tcp()[1])

    # -- N: round
    def n_pick(self):
        mk, N = self.mk, self.N
        p, r = self.S()[self.name][:2]
        yaw = math.atan2(r[1, 0], r[0, 0])
        rf = pl.rot_z(yaw)
        o = np.array([p[0], p[1], mk.TOP]) - rf @ np.array([mk.OAL / 2, 0, 0])
        g = o + rf @ pl.n_tcp((0.0, p[2] - mk.TOP), 0.0, psi=self.psiN, beta=tp.TILT)
        rr = rf @ pl.n_rot(self.psiN, tp.TILT)
        self.mark("n_pick")
        yield from self.g_goq(N, "n_hover", 2.0, self.qN["hover"], PICK_OPEN)
        yield from self.g_go(N, "n_descend", 1.5, g, rr)
        yield from self.g_wait("n_settle", 1.0)
        yield from self.g_go(N, "n_trim", 0.5, g + (g - N.tcp_meas()[0]) * [1, 1, 0], rr)
        yield from self.g_wait("n_trim_wait", 0.5)
        yield from self.g_go(N, "n_close", 0.8, *N.tcp(), 0.0)
        yield from self.g_wait("n_squeeze", 0.4)
        p0, r0 = N.tcp()
        yield from self.g_go(N, "n_lift", 1.0, p0 + [0, 0, 0.04], r0)
        yield from self.g_go(N, "n_wait_pose", 2.5, *self.dN["lift"])
        r = self.rnd()
        self.events["grasp"] = dict(grip_meas=self.rows[-1]["grip_meas"][0 if N.side == "left" else 1], tilt_deg=round(math.degrees(r["tilt"]), 2),
                                    yaw_deg=round(math.degrees(r["yaw"]), 2), base_mm=np.round(r["b"] * 1000, 1).tolist())
        print("GRASP", json.dumps(self.events["grasp"]), flush=True)
        if self.events["grasp"]["grip_meas"] < 0.02 or r["c"][2] < 0.09:
            raise Stop("pick: round not held")

    def n_cmd(self, xb, dz, label, grip=None, servo=True):
        """N holds the round with its base centre on the entry line at xb (+dz up), nose-up TILT; base x, pinch height,
        sideways position, yaw and tilt (by leaning the gripper) are corrected from the measured round pose"""
        mp, mr = self.mag()
        bt = (xb, tp.zb_line(xb) + dz)
        tcp = pl.n_tcp(bt, tp.TILT, 0.0, psi=self.psiN, beta=self.lean) + self.ncorr
        self.to(self.N, mp + mr @ tcp, mr @ pl.rot_z(self.nyaw) @ pl.n_rot(self.psiN, self.lean), self.N.grip if grip is None else grip, label)
        self.step(label)
        r = self.rnd()
        if servo:
            s = pl.L_CASE + pl.PIN_GAP + pl.TAPER * pl.PIN_D
            pin = r["b"] + r["a"] * s
            want = np.array([bt[0], 0.0, bt[1]]) + pl.axis(tp.TILT) * s
            e = np.array([bt[0] - r["b"][0], -pin[1], want[2] - pin[2]])
            self.ncorr = np.clip(self.ncorr + GAIN * e, -0.008, 0.008)
            self.nyaw = float(np.clip(self.nyaw - 0.2 * r["yaw"], -0.15, 0.15))
            self.lean = float(np.clip(self.lean + 0.2 * (r["tilt"] - tp.TILT), -0.12, 0.12))
        self.rows[-1]["ncorr"] = np.round(np.r_[self.ncorr * 1000, math.degrees(self.nyaw), math.degrees(self.lean)], 2).tolist()
        self.check(label, r)
        return r

    def check(self, label, r=None):
        r = r or self.rnd()
        if r["c"][2] < 0.095 or abs(r["c"][1]) > 0.009 or abs(r["yaw"]) > 0.5:
            raise Stop(f"{label}: round left the mouth (base {np.round(r['b'] * 1000, 1).tolist()}, yaw {math.degrees(r['yaw']):.0f} deg)")
        if np.linalg.norm(self.f(self.N)) > F_ABORT:
            raise Stop(f"{label}: force limit (N {self.f(self.N).tolist()})")

    def insert(self):
        """runs with P's hold controller: every N step also advances self.bg"""
        self.mark("n_approach")
        self.ncorr, self.nyaw, self.lean = np.zeros(3), 0.0, 0.0
        n = round(2.5 * HZ)
        for k in range(1, n + 1):  # from the waiting pose (50 mm ahead, 40 mm up) to 4 mm above the start of the entry line
            s = k / n
            xb = tp.X_IN0 + 0.05 * (1 - s)
            self.tick_bg()
            self.n_cmd(xb, 0.04 * (1 - s) + 0.004 * s - (tp.zb_line(xb) - tp.zb_line(tp.X_IN0)), "n_approach", servo=s > 0.7)
        for _ in range(round(1.2 * HZ)):
            self.tick_bg()
            self.n_cmd(tp.X_IN0, 0.004, "n_align")
        for k in range(1, 9):
            self.tick_bg()
            self.n_cmd(tp.X_IN0, 0.004 * (1 - k / 8), "n_on_line", grip=self.N.grip + (N_HOLD - self.N.grip) * k / 8)
        for _ in range(round(0.5 * HZ)):
            self.tick_bg()
            self.n_cmd(tp.X_IN0, 0.0, "n_on_line")
        r = self.rnd()
        self.events["on_line"] = dict(base_mm=np.round(r["b"] * 1000, 2).tolist(), want_mm=[tp.X_IN0 * 1000, 0, round(tp.zb_line(tp.X_IN0) * 1000, 2)],
                                      tilt_deg=round(math.degrees(r["tilt"]), 2), yaw_deg=round(math.degrees(r["yaw"]), 2),
                                      ncorr=self.rows[-1]["ncorr"], tool_u_mm=self.rows[-1]["tool_u_mm"], follower_mm=self.rows[-1]["follower_mm"])
        print("ON_LINE", json.dumps(self.events["on_line"]), flush=True)
        self.mark("n_insert")
        n = round((tp.X_IN0 - tp.X_IN1) / V_IN * HZ)
        for k in range(1, n + 1):
            self.tick_bg()
            self.n_cmd(tp.X_IN0 + (tp.X_IN1 - tp.X_IN0) * k / n, 0.0, "n_insert")
        for _ in range(round(0.4 * HZ)):
            self.tick_bg()
            self.n_cmd(tp.X_IN1, 0.0, "n_insert_hold")
        r = self.rnd()
        self.events["inserted"] = dict(base_mm=np.round(r["b"] * 1000, 2).tolist(), want_mm=[tp.X_IN1 * 1000, 0, round(tp.zb_line(tp.X_IN1) * 1000, 2)],
                                       tilt_deg=round(math.degrees(r["tilt"]), 2), yaw_deg=round(math.degrees(r["yaw"]), 2), ncorr=self.rows[-1]["ncorr"],
                                       fN_N=self.f(self.N).tolist(), tool_u_mm=self.rows[-1]["tool_u_mm"], follower_mm=self.rows[-1]["follower_mm"])
        print("INSERTED", json.dumps(self.events["inserted"]), flush=True)
        if r["b"][0] > tp.X_IN1 + 0.004:
            raise Stop("insert: round did not follow")

    def tick_bg(self):
        for g in list(self.bg_gens):
            try:
                next(g)
            except StopIteration:
                self.bg_gens.remove(g)

    def n_step(self, gen):
        """run a generator to its end while the background (P) controller keeps running"""
        while True:
            try:
                labs = [next(gen)]
            except StopIteration:
                return
            for g in list(self.bg_gens):
                try:
                    labs.append(next(g))
                except StopIteration:
                    self.bg_gens.remove(g)
            self.step("|".join(labs))

    def nose_push(self, label, stop, limit, travel):
        mk, N = self.mk, self.N
        mp, mr = self.mag()
        rd = self.rnd()
        tip = rd["c"] + rd["a"] * mk.OAL / 2
        zc = float((rd["c"] + rd["a"] * (mk.OAL / 2 - mk.R_BUL))[2])
        x0 = float(tip[0]) + 0.0015
        ny = lambda r: float((r["c"] + r["a"] * (mk.OAL / 2 - mk.R_BUL))[1])  # noqa: E731  nose centre, sideways
        self.n_step(self.g_go(N, label + "_pre", 1.2, mp + mr @ pl.nose_push_tcp(x0, zc, ny(rd), psi=self.psiN), mr @ pl.n_rot(self.psiN), NOSE_GRIP))
        x, peak = x0, 0.0
        for _ in range(round(16.0 * HZ)):
            x -= NOSE_V / HZ
            rd = self.rnd()
            zc = float((rd["c"] + rd["a"] * (mk.OAL / 2 - mk.R_BUL))[2])
            mp, mr = self.mag()
            self.tick_bg()
            self.to(N, mp + mr @ pl.nose_push_tcp(x, zc, ny(rd), psi=self.psiN), mr @ pl.n_rot(self.psiN), NOSE_GRIP, label)
            self.step(label)
            fx = -self.f(N)[0]
            peak = max(peak, fx)
            self.check(label)
            if stop() or fx > limit or x < x0 - travel:
                break
        N.q, N.integ = N.meas.copy(), np.zeros(6)  # stop pushing: hold the measured pose
        N.command(N.q, NOSE_GRIP)
        self.n_step(self.g_wait(label + "_hold", 0.3))
        rd = self.rnd()
        return dict(base_mm=np.round(rd["b"] * 1000, 2).tolist(), gap_mm=round(self.gap(rd) * 1000, 2), tilt_deg=round(math.degrees(rd["tilt"]), 2),
                    yaw_deg=round(math.degrees(rd["yaw"]), 2), travel_mm=round((x0 - x) * 1000, 2), peak_rear_N=round(peak, 2), follower_mm=self.rows[-1]["follower_mm"])

    def rule(self):
        mk = self.mk
        r = self.rnd()
        gap = self.gap(r)
        mouth = r["b"][0] + r["a"][0] * mk.L_CASE
        ok = bool(gap < 0.002 and r["c"][2] < mk.Z_RET0 + 0.0003 and mouth < mk.X_RAMP1 and abs(math.degrees(r["tilt"])) < 10
                  and abs(r["c"][1]) < mk.W_IN / 2)
        dz = r["c"][2] - mk.z_ret(r["c"][1])
        return dict(rule_ok=ok, loaded=bool(ok and abs(dz) < 0.001 and abs(math.degrees(r["yaw"])) < 5), base_gap_mm=round(gap * 1000, 2),
                    c_mm=np.round(r["c"] * 1000, 2).tolist(), dz_lip_line_mm=round(dz * 1000, 2), tilt_deg=round(math.degrees(r["tilt"]), 2),
                    yaw_deg=round(math.degrees(r["yaw"]), 2), case_mouth_x_mm=round(mouth * 1000, 2))

    def clear_mm(self):
        from partkit import finger_points
        c = self.S()[self.name][0]
        return round(min(float(np.linalg.norm(np.asarray(p) - c)) for p in finger_points(self.emb._env)) * 1000, 1)

    def run(self):
        mk, stage = self.mk, E("TOOL_STAGE", "full")
        self.par(self.g_wait("settle", 1.0))
        self.plan()
        self.bg = self.p_seq()
        self.bg_gens = [self.bg]
        self.p_seated = 0
        self.n_step(self.n_pick() if stage != "tool" else self.g_wait("n_idle", 0.1))
        for _ in range(round(45 * HZ)):  # wait until the tool is seated
            if self.p_seated >= 5:
                break
            if not self.bg_gens:
                raise Stop("tool arm stopped")
            self.n_step(self.g_wait("wait_tool", 0.1))
        s = self.rows[-1]
        self.events["tool_seated"] = dict(seated=self.p_seated >= 5, tool_u_mm=s["tool_u_mm"], tool_mm=s["tool_mm"], ypr_deg=s["tool_ypr_deg"], follower_mm=s["follower_mm"],
                                          spring_N=s["spring_N"], tool_fol_N=s.get("tool_fol_N"), tool_mag_N=s.get("tool_mag_N"),
                                          P_on_tool_N=self.f(self.P, "tool").tolist(), r_cmd_mm=round(getattr(self, "r_cmd", 9) * 1000, 2),
                                          mag_in_fix_mm=s.get("mag_in_fix_mm"))
        print("TOOL_SEATED", json.dumps(self.events["tool_seated"]), flush=True)
        if self.p_seated < 5:
            raise Stop("tool not seated")
        if stage == "tool":
            self.n_step(self.g_wait("hold", 2.0))
            self.p_done = True
            self.n_step(self.g_lift())
            return
        self.insert()
        self.mark("n_release")
        N = self.N
        self.n_step(self.g_go(N, "n_open", 0.6, *N.tcp(), 0.35))
        self.n_step(self.g_wait("n_open_wait", 0.5))
        r = self.rnd()
        self.events["released"] = dict(base_mm=np.round(r["b"] * 1000, 2).tolist(), tilt_deg=round(math.degrees(r["tilt"]), 2), yaw_deg=round(math.degrees(r["yaw"]), 2))
        print("RELEASED", json.dumps(self.events["released"]), flush=True)
        p, rr = N.tcp()
        self.n_step(self.g_go(N, "n_clear", 1.5, p + self.mag()[1] @ np.array([0.06, 0, 0.02]), rr))  # out of the way of the tool's front wall
        self.p_done = True
        self.tick_bg()
        self.bg_gens = [self.g_lift()]
        while self.bg_gens:
            self.n_step(self.g_wait("n_hold", 0.1))
        r = self.rnd()
        self.events["after_lift"] = dict(base_mm=np.round(r["b"] * 1000, 2).tolist(), gap_mm=round(self.gap(r) * 1000, 2), tilt_deg=round(math.degrees(r["tilt"]), 2),
                                         follower_mm=self.rows[-1]["follower_mm"], rule=self.rule())
        print("AFTER_LIFT", json.dumps(self.events["after_lift"]), flush=True)
        self.par(self.g_go(N, "n_close2", 0.8, *N.tcp(), NOSE_GRIP))
        self.mark("push_to_rear")
        self.events["push2"] = self.nose_push("push2", lambda: self.gap() < 0.0004, NOSE_F, float(E("ARM_NOSE_LEAD", "0.065")))
        print("PUSH2", json.dumps(self.events["push2"]), flush=True)

    def retreat(self):
        self.mark("retreat")
        mp, mr = self.mag()
        try:
            p, r = self.N.tcp()
            self.par(self.g_go(self.N, "n_away", 2.0, p + mr @ np.array([0.055, 0, 0.025]), r, 1.0))
        except Stop as e:
            self.events.setdefault("retreat_err", []).append(repr(e))
        self.mark("after")
        out = {}
        for k in ("0s", "1s", "2s"):
            out[k] = self.rule()
            if k != "2s":
                self.par(self.g_wait("after_" + k, 1.0))
        self.events["after"] = out
        self.events["hands_clear_mm"] = self.clear_mm()
        rel, _ = self.tool_rel()
        self.events["tool_end_mm"] = np.round(rel * 1000, 1).tolist()
        print("AFTER", json.dumps(out), self.events["hands_clear_mm"], flush=True)


def main():
    out = Path(sys.argv[1])
    r = T()
    err = None
    try:
        r.start(out, "tool")
        r.left.ki = r.right.ki = float(E("MAG_KI", "0.15"))
        r.left.clip = r.right.clip = float(E("MAG_ICLIP", "0.6"))
        r.run()
    except Exception as e:  # noqa: BLE001  recorded
        err = e
        traceback.print_exc()
        r.events["failure"] = repr(e)
    try:
        if r.N is not None and E("TOOL_STAGE", "full") == "full":
            r.retreat()
    except Exception:  # noqa: BLE001
        traceback.print_exc()
    try:
        mk = r.mk
        r.events["phase"] = r.phase
        r.events["params"] = dict(env={k: v for k, v in os.environ.items() if k.startswith(("MAG_", "ARM_", "FIX_", "TOOL_"))}, spring=mk.SPRING,
                                  k=mk.K_SPRING, f_top=mk.F_TOP, mu_round=mk.MU_ROUND, mu_mag=mk.MU_MAG, mu_fix=mk.MU_FIX, mu_tool=mk.MU_TOOL,
                                  tool_g=mk.TOOL_MASS * 1000, pad=mk.PAD_FRICTION, grip_n=mk.GRIP_N, phys_dt=mk.PHYSICS_DT, round_g=mk.ROUND_G,
                                  mag_xy=mk.MAG_XY.tolist(), mag_yaw_deg=math.degrees(mk.MAG_YAW))
        r.finish(err)
    except Exception:  # noqa: BLE001
        traceback.print_exc()
    os._exit(0)


if __name__ == "__main__":
    main()
