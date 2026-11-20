"""tool3 arm demo: python drv.py run_tool3.py OUT
As tool2 (run_tool2.py, setting S8) with two changes. (1) Tool-arm press, hold and unload: a force loop on the measured
contact force of the tool-arm fingers on the tool (axial part along the tube axis held at F_PRESS, sideways part driven
to 0): the command sits at the measured tool pose plus offsets proportional to the force errors, so the arm's joint
integrator stops winding once the force is on target (no wind-up against the fixture stop). (2) If the round base is
not deep enough after the entry, the round arm lets go and pushes the nose rearward with its closed fingertips before the
tool comes off. Robot commands only."""
import json
import math
import os
import sys
import traceback
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_tool2 as t2  # noqa: E402
from run_p0 import Stop  # noqa: E402

E = os.environ.get
HZ = t2.HZ
U = t2.U
F_PRESS = float(E("TOOL_F_PRESS", "16"))
KA = float(E("TOOL_KA", "0.0012"))  # m per N: command below the tool per N of axial force missing
KS = float(E("TOOL_KS", "0.0005"))  # m per N: command offset against the sideways force
SEAT_U = float(E("TOOL_SEAT_U", "1.6"))  # mm: seated when the tool is this low and pressing
NOSE_IF, NOSE_TO = float(E("TOOL_NOSE_IF", "-0.009")), float(E("TOOL_NOSE_TO", "-0.0102"))
NOSE_F = float(E("TOOL_NOSE_F", "18"))
NOSE_TRAVEL = float(E("TOOL_NOSE_TRAVEL", "0.016"))
LIFT_BACK = float(E("TOOL_LIFT_BACK", "0.0006"))  # m
P_HAT = np.array([math.cos(math.radians(21.0)), 0.0, -math.sin(math.radians(21.0))])


class T(t2.T):
    rel_bias = 0.0

    def tool_rel(self):  # lift-off, push pose and pull: the tool target sits LIFT_BACK behind the tube axis (the tongue face clear of the round)
        rel, rr = super().tool_rel()
        return rel + self.rel_bias * P_HAT, rr

    def fp(self):
        f = np.array(self.rows[-1].get(f"f{self.P.side[0]}_tool_N", (0, 0, 0)), float)
        return -float(U @ f), f - U * float(U @ f)

    def g_force(self, target, label):
        """command = measured tool pose, offset down the axis by KA x (axial force error) and sideways by -KS x (sideways
        force): the arm's joint integrator then winds only while the force is off target"""
        fa, fs = self.fp()
        rel, _ = self.tool_rel()
        self.r_cmd = 0.0
        self.tcorr = rel - U * float(np.clip(KA * (target - fa), -0.010, 0.012)) - np.clip(KS * fs, -0.004, 0.004)
        self.p_cmd(0.0, label)
        return fa

    def g_hold(self):
        while not self.p_done:
            fa = self.g_force(F_PRESS, "p_hold")
            self.p_seated = self.p_seated + 1 if self.rows[-1]["tool_u_mm"] < SEAT_U and fa > 0.7 * F_PRESS else 0
            yield "p_hold"

    def g_remove(self):
        self.mark("p_unload")
        n = round(1.5 * HZ)
        for k in range(1, n + 1):
            self.g_force(F_PRESS * (1 - k / n), "p_unload")
            yield "p_unload"
        for _ in range(round(1.0 * HZ)):
            self.g_force(0.0, "p_unload")
            yield "p_unload"
        self.tcorr = np.zeros(3)
        self.r_cmd = self.rows[-1]["tool_u_mm"] / 1000
        g = super().g_remove()
        for lab in g:
            if lab.startswith("tool_regrasp_squeeze"):
                self.rel_bias = LIFT_BACK
            yield lab

    def insert(self):
        try:
            super().insert()
        except Stop as e:
            if "did not follow" not in str(e):
                raise
            self.events["insert_short"] = str(e)
            print("INSERT_SHORT", str(e), flush=True)

    def nose(self):
        rd = self.rnd()
        if rd["b"][0] <= NOSE_IF:
            return
        N = self.N
        self.mark("n_nose")
        self.events["nose"] = self.nose_push("n_nose", lambda: self.rnd()["b"][0] <= NOSE_TO, NOSE_F, NOSE_TRAVEL)
        print("NOSE", json.dumps(self.events["nose"]), flush=True)
        p, rr = N.tcp()
        self.n_step(self.g_go(N, "n_clear2", 1.5, p + self.mag()[1] @ np.array([0.06, 0, 0.02]), rr, 0.35))
        if self.rnd()["b"][0] > NOSE_IF:
            raise Stop("nose push: round not deep enough")

    def run(self):
        self.par(self.g_wait("settle", 1.0))
        self.plan()
        self.bg = self.p_seq()
        self.bg_gens = [self.bg]
        self.p_seated = 0
        self.n_step(self.n_pick())
        for _ in range(round(45 * HZ)):
            if self.p_seated >= 5:
                break
            if not self.bg_gens:
                raise Stop("tool arm stopped")
            self.n_step(self.g_wait("wait_tool", 0.1))
        s = self.rows[-1]
        self.events["tool_seated"] = dict(seated=self.p_seated >= 5, tool_u_mm=s["tool_u_mm"], tool_mm=s["tool_mm"], ypr_deg=s["tool_ypr_deg"], follower_mm=s["follower_mm"],
                                          spring_N=s["spring_N"], tool_fol_N=s.get("tool_fol_N"), tool_mag_N=s.get("tool_mag_N"), finger_N=s.get(f"f{self.P.side[0]}_tip_N"),
                                          P_on_tool_N=self.f(self.P, "tool").tolist(), r_cmd_mm=round(getattr(self, "r_cmd", 9) * 1000, 2), tcorr_mm=np.round(self.tcorr * 1000, 2).tolist())
        print("TOOL_SEATED", json.dumps(self.events["tool_seated"]), flush=True)
        if self.p_seated < 5:
            raise Stop("tool not seated")
        self.insert()
        self.mark("n_release")
        N = self.N
        self.n_step(self.g_go(N, "n_open", 0.6, *N.tcp(), 0.35))
        self.n_step(self.g_wait("n_open_wait", 0.5))
        r = self.rnd()
        self.events["released"] = dict(base_mm=np.round(r["b"] * 1000, 2).tolist(), tilt_deg=round(math.degrees(r["tilt"]), 2), yaw_deg=round(math.degrees(r["yaw"]), 2))
        print("RELEASED", json.dumps(self.events["released"]), flush=True)
        p, rr = N.tcp()
        self.n_step(self.g_go(N, "n_clear", 1.5, p + self.mag()[1] @ np.array([0.06, 0, 0.02]), rr))
        self.nose()
        self.p_done = True
        self.tick_bg()
        self.bg_gens = [self.g_remove()]
        while self.bg_gens:
            self.n_step(self.g_wait("n_hold", 0.1))


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
