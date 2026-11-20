"""tool2 arm demo: python drv.py run_tool2.py OUT
As tool1 (run_tool.py) up to the release of the round. New: the tool arm P seats its fingertip blades on the fin's seat
lugs for the press; to take the tool off it first brings its command back to the tool (no stored push), opens, grasps the
fin again at the nominal place, lifts the tool S_PUSH up the tube axis, pulls it rearward so that the tooth under the
front wall pushes the round to the rear wall, and parks the tool (held). Robot commands only."""
import json
import math
import os
import sys
import traceback
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_tool as old  # noqa: E402
import tplan as tp  # noqa: E402
from run_p0 import Stop  # noqa: E402

E = os.environ.get
HZ = old.HZ
U = tp.U
V_LIFT = float(E("TOOL_V_LIFT", "0.010"))
V_PULL = float(E("TOOL_V_PULL", "0.004"))
DZ = float(E("TOOL_PUSH_DZ", "0.0003"))  # tooth underside above the front-wall top during the pull
F_PULL = float(E("TOOL_F_PULL", "25"))
REGRASP = E("TOOL_REGRASP", "1") == "1"
CLOSE_S = float(E("TOOL_CLOSE_S", "0.8"))
PRE_GRIP = float(E("TOOL_PRE_GRIP", "0"))  # grip command (fraction of 93.9 mm) reached before the close; 0 = off
GRASP_ONLY = E("TOOL_GRASP_ONLY", "0") == "1"
PITCH_K = float(E("TOOL_PITCH_K", "0.15"))  # lift-off and pull: the gripper command turns about mag y against the measured tool pitch
PULL_MAX = float(E("TOOL_PULL_MAX", "0.02"))
PULL_WAIT = float(E("TOOL_PULL_WAIT", "0"))  # m; 0 = off (tool3 S9)
PULL_STALL = int(E("TOOL_PULL_STALL", "0"))  # demo steps the command has waited; 0 = off
ZK, ZCLIP = float(E("TOOL_PULL_ZK", "0.2")), float(E("TOOL_PULL_ZCLIP", "0.012"))  # pull: gain and limit of the sideways/height servo  # pull command travel limit


class T(old.T):
    def p_cmd(self, r, label, off=(0.0, 0.0, 0.0)):
        mp, mr = self.mag()
        rc = old.pl.rot_y(getattr(self, "phi", 0.0))
        self.to(self.P, mp + mr @ (tp.TOOL_O + U * r + np.asarray(off) + self.tcorr + rc @ self.gp), mr @ rc @ self.gr, 0.0, label)

    def pitch_servo(self):
        self.phi = float(np.clip(getattr(self, "phi", 0.0) - PITCH_K * math.radians(self.rows[-1]["tool_ypr_deg"][1]), -0.35, 0.35))

    def g_grasp(self, label, n=20):
        """bring the open fingertips to the nominal grasp on the tool where it stands (measured tool pose, measured TCP)"""
        P, g, c = self.P, tp.p_tcp_tool(self.psiP), np.zeros(3)
        for _ in range(n):
            s = self.S()
            tgt, rt = s["tool"][0] + s["tool"][1] @ g, s["tool"][1] @ tp.g_rot(self.psiP)
            c = np.clip(c + 0.4 * (tgt - P.tcp_meas()[0]), -0.012, 0.012)
            self.to(P, tgt + c, rt, tp.P_GRIP_OPEN, label)
            yield label
        if PRE_GRIP:  # fingers nearly on the fin while the centring goes on
            g0 = P.grip
            for k in range(1, 11):
                s = self.S()
                tgt, rt = s["tool"][0] + s["tool"][1] @ g, s["tool"][1] @ tp.g_rot(self.psiP)
                c = np.clip(c + 0.4 * (tgt - P.tcp_meas()[0]), -0.012, 0.012)
                self.to(P, tgt + c, rt, g0 + (PRE_GRIP - g0) * k / 10, label + "_pre")
                yield label + "_pre"
        yield from self.g_go(P, label + "_close", CLOSE_S, *P.tcp(), 0.0)
        yield from self.g_wait(label + "_squeeze", 0.4)
        s = self.S()
        pm, rm = P.tcp_meas()
        self.gp, self.gr = s["tool"][1].T @ (pm - s["tool"][0]), s["tool"][1].T @ rm
        self.tcorr = np.zeros(3)
        ev = dict(tcp_in_tool_mm=np.round(self.gp * 1000, 2).tolist(), planned_mm=np.round(g * 1000, 2).tolist(),
                  grip_meas=self.rows[-1]["grip_meas"][0 if P.side == "left" else 1], finger_N=self.rows[-1].get(f"f{P.side[0]}_tip_N"))
        self.events[label] = ev
        print(label.upper(), json.dumps(ev), flush=True)

    def p_seq(self):
        P, d = self.P, self.dP
        self.mark("p_pick")
        yield from self.g_goq(P, "p_hover", 2.5, self.qP["hover"], tp.P_GRIP_OPEN)
        yield from self.g_wait("p_hover_settle", 1.0)
        h, g, c = d["hover"][0], d["grasp"][0], np.zeros(3)
        for _ in range(8):
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
        yield from self.g_wait("p_settle", 0.5)
        yield from self.g_grasp("tool_grasp")
        self.mark("p_carry")
        p0, r0 = P.tcp()
        yield from self.g_go(P, "p_lift", 2.0, np.array([p0[0], p0[1], d["lift"][0][2]]), r0)
        if GRASP_ONLY:
            rel, _ = self.tool_rel()
            self.events["grasp_test"] = dict(tool_in_gripper_mm=np.round((P.tcp_meas()[0] - self.S()["tool"][0]) * 1000, 2).tolist(), row=self.rows[-1].get("tool_mm"),
                                             ypr_deg=self.rows[-1].get("tool_ypr_deg"), finger_N=self.rows[-1].get(f"f{P.side[0]}_tip_N"))
            print("GRASP_TEST", json.dumps(self.events["grasp_test"]), flush=True)
            raise Stop("grasp test end")
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
            r = max(0.008, r - old.V_TOOL / HZ)
            self.p_cmd(r, "p_insert")
            yield "p_insert"
        self.events["tool_on"] = dict(tool_mm=self.rows[-1].get("tool_mm"), ypr_deg=self.rows[-1].get("tool_ypr_deg"), tcorr_mm=np.round(self.tcorr * 1000, 2).tolist())
        print("TOOL_ON", json.dumps(self.events["tool_on"]), flush=True)
        self.mark("p_press")
        self.r_cmd, self.p_done, self.p_seated = r, False, 0
        yield from self.g_hold()

    def g_remove(self):
        P = self.P
        self.mark("p_unload")
        r = self.r_cmd
        for _ in range(round(10 * HZ)):  # bring the command back up to the tool: the stored push goes, the follower lifts the tool
            u = self.rows[-1]["tool_u_mm"] / 1000
            if r >= u - 0.0005:
                break
            r = min(u, r + 0.010 / HZ)
            self.comply()
            self.p_cmd(r, "p_unload")
            yield "p_unload"
        row = self.rows[-1]
        self.events["unloaded"] = dict(tool_u_mm=row["tool_u_mm"], ypr_deg=row["tool_ypr_deg"], follower_mm=row["follower_mm"], finger_N=row.get(f"f{P.side[0]}_tip_N"),
                                       P_on_tool_N=self.f(P, "tool").tolist(), tool_mag_N=row.get("tool_mag_N"), tool_rnd_N=row.get("tool_rnd_N"))
        print("UNLOADED", json.dumps(self.events["unloaded"]), flush=True)
        if REGRASP:
            P.q, P.integ = P.meas.copy(), np.zeros(6)
            yield from self.g_go(P, "p_open", 0.6, *P.tcp(), tp.P_GRIP_OPEN)
            yield from self.g_wait("p_open_wait", 0.6)
            row = self.rows[-1]
            self.events["tool_free"] = dict(tool_u_mm=row["tool_u_mm"], ypr_deg=row["tool_ypr_deg"], follower_mm=row["follower_mm"])
            print("TOOL_FREE", json.dumps(self.events["tool_free"]), flush=True)
            self.mark("p_regrasp")
            yield from self.g_grasp("tool_regrasp")
        self.tcorr = np.zeros(3)
        self.mark("p_lift_off")
        s_push = tp.S_PUSH
        rel, _ = self.tool_rel()
        r = u0 = float(U @ rel)
        peak, stall = 0.0, 0
        while r < s_push and stall < 80:
            rel, _ = self.tool_rel()
            if r - float(U @ rel) < 0.006:  # the command waits when the tool lags more than 6 mm
                r = min(s_push, r + V_LIFT / HZ)
            else:
                stall += 1
            e = U * r - rel
            self.tcorr = np.clip(self.tcorr + 0.1 * (e - U * float(U @ e)), -0.008, 0.008)
            self.p_cmd(r, "p_lift_off")
            yield "p_lift_off"
            peak = max(peak, float(np.linalg.norm(self.rows[-1].get("tool_mag_N", (0, 0, 0)))))
        for _ in range(round(1.5 * HZ)):  # settle in the push pose: tooth underside DZ above the front-wall top
            rel, _ = self.tool_rel()
            self.tcorr = np.clip(self.tcorr + 0.2 * (U * s_push + [0, 0, DZ] - rel), -0.012, 0.012)
            self.pitch_servo()
            self.p_cmd(s_push, "p_pose", (0, 0, DZ))
            yield "p_pose"
        rel, rr = self.tool_rel()
        rd = self.rnd()
        row = self.rows[-1]
        self.events["after_lift"] = dict(tool_mm=np.round((rel - U * s_push) * 1000, 2).tolist(), ypr_deg=row["tool_ypr_deg"], lift_from_mm=round(u0 * 1000, 2), stall_steps=stall,
                                         tool_mag_peak_N=round(peak, 2), finger_N=row.get(f"f{P.side[0]}_tip_N"), base_mm=np.round(rd["b"] * 1000, 2).tolist(),
                                         gap_mm=round(self.gap(rd) * 1000, 2), tilt_deg=round(math.degrees(rd["tilt"]), 2), follower_mm=row["follower_mm"], rule=self.rule())
        print("AFTER_LIFT", json.dumps(self.events["after_lift"]), flush=True)
        if abs(rel[2] - U[2] * s_push) > 0.004:
            raise Stop("tool not lifted to the push pose")
        self.mark("p_pull")
        xt, peak, why, waited = 0.0, 0.0, "time", 0
        for _ in range(round(30 * HZ)):
            rel, _ = self.tool_rel()
            if not PULL_WAIT or (U * s_push + [xt, 0, DZ] - rel)[0] > -PULL_WAIT:  # p2 option: the command waits while it leads the tool by PULL_WAIT
                xt -= V_PULL / HZ
                waited = 0
            else:
                waited += 1
            if PULL_STALL and waited >= PULL_STALL and self.gap() < 0.001:  # p2 option: tool stalled with the round near the wall
                why = "stalled near the rear wall"
                break
            e = U * s_push + [xt, 0, DZ] - rel
            self.tcorr[1:] = np.clip(self.tcorr[1:] + ZK * e[1:], -ZCLIP, ZCLIP)  # sideways and height servo; x is the push
            self.pitch_servo()
            if len(self.rows) % 4 == 0:
                print("PULLDBG", json.dumps(dict(step=len(self.rows), xt_mm=round(-xt * 1000, 1), e_mm=np.round(e * 1000, 2).tolist(), tcorr_mm=np.round(self.tcorr * 1000, 2).tolist(),
                                                 phi_deg=round(math.degrees(getattr(self, "phi", 0.0)), 2), gap_mm=round(self.gap() * 1000, 2))), flush=True)
            try:
                self.p_cmd(s_push, "p_pull", (xt, 0, DZ))
            except Stop:
                why = "unreachable"
                break
            yield "p_pull"
            fx = float(self.rows[-1]["tool_rnd_N"][0])
            peak = max(peak, fx)
            if self.gap() < 0.0004:
                why = "at the rear wall"
                break
            if fx > F_PULL:
                why = "force limit"
                break
            if -e[0] > tp.PULL_LEAD:
                why = "lead limit"
                break
            if -xt > PULL_MAX:
                why = "travel limit"
                break
        P.q, P.integ = P.meas.copy(), np.zeros(6)  # stop pulling: hold the measured pose
        P.command(P.q, 0.0)
        yield from self.g_wait("p_pull_hold", 0.3)
        rel, _ = self.tool_rel()
        rd = self.rnd()
        row = self.rows[-1]
        self.events["push2"] = dict(stop=why, base_mm=np.round(rd["b"] * 1000, 2).tolist(), gap_mm=round(self.gap(rd) * 1000, 2), tilt_deg=round(math.degrees(rd["tilt"]), 2),
                                    yaw_deg=round(math.degrees(rd["yaw"]), 2), cmd_travel_mm=round(-xt * 1000, 2), phi_deg=round(math.degrees(getattr(self, "phi", 0.0)), 2), tool_mm=np.round((rel - U * s_push) * 1000, 2).tolist(),
                                    ypr_deg=row["tool_ypr_deg"], peak_rear_N=round(peak, 2), follower_mm=row["follower_mm"], finger_N=row.get(f"f{P.side[0]}_tip_N"))
        print("PUSH2", json.dumps(self.events["push2"]), flush=True)
        self.mark("p_park")
        mr = self.mag()[1]
        p, rr = P.tcp()
        yield from self.g_go(P, "p_back", 0.8, p + mr @ np.array([0.004, 0, 0]), rr)
        yield from self.g_go(P, "p_up", 1.5, P.tcp()[0] + mr @ (U * 0.012), rr)
        yield from self.g_go(P, "p_park", 2.5, *self.dP["park"])
        yield from self.g_wait("p_parked", 0.5)

    def run(self):
        self.par(self.g_wait("settle", 1.0))
        self.plan()
        self.bg = self.p_seq()
        self.bg_gens = [self.bg]
        self.p_seated = 0
        self.n_step(self.n_pick())
        for _ in range(round(45 * HZ)):  # wait until the tool is seated
            if self.p_seated >= 5:
                break
            if not self.bg_gens:
                raise Stop("tool arm stopped")
            self.n_step(self.g_wait("wait_tool", 0.1))
        s = self.rows[-1]
        self.events["tool_seated"] = dict(seated=self.p_seated >= 5, tool_u_mm=s["tool_u_mm"], tool_mm=s["tool_mm"], ypr_deg=s["tool_ypr_deg"], follower_mm=s["follower_mm"],
                                          spring_N=s["spring_N"], tool_fol_N=s.get("tool_fol_N"), tool_mag_N=s.get("tool_mag_N"), finger_N=s.get(f"f{self.P.side[0]}_tip_N"),
                                          P_on_tool_N=self.f(self.P, "tool").tolist(), r_cmd_mm=round(getattr(self, "r_cmd", 9) * 1000, 2),
                                          mag_in_fix_mm=s.get("mag_in_fix_mm"))
        print("TOOL_SEATED", json.dumps(self.events["tool_seated"]), flush=True)
        if self.p_seated < 5:
            raise Stop("tool not seated")
        if E("TOOL_STAGE", "full") != "tool":
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
        else:
            self.n_step(self.g_wait("hold", 2.0))
        self.p_done = True
        self.tick_bg()
        self.bg_gens = [self.g_remove()]
        while self.bg_gens:
            self.n_step(self.g_wait("n_hold", 0.1))

    def retreat(self):
        super().retreat()
        s = self.S()
        row = self.rows[-1]
        self.events["tool_end"] = dict(tool_above_table_mm=round(float(s["tool"][0][2] - self.mk.TOP) * 1000, 1), P_finger_N=row.get(f"f{self.P.side[0]}_tip_N"),
                                       P_on_tool_N=self.f(self.P, "tool").tolist(), tool_mag_N=row.get("tool_mag_N"))
        print("TOOL_END", json.dumps(self.events["tool_end"]), self.events.get("tool_end_mm"), flush=True)


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
        if r.N is not None:
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
