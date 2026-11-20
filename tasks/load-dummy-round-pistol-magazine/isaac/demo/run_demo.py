"""Scripted demo and scripted negatives of the task, through this task folder's scene and live scorer (../magkit.py,
../task_scoring.py). Bimanual YAM: tool arm = left (P), round arm = right (N). Motion code: run_tool3.py and its bases
(setting S9 of the development runs). Robot commands only; the script reads simulator state to plan.

From isaac/ (inside the Isaac Lab-Arena container, GPU; drv.py adds the development-only RTX driver-check override):
  python demo/drv.py demo/run_demo.py OUT [--stages demo,neg_idle,...] --driver-check-override
Writes per stage OUT/<i>-<stage>/: actions.json (14-D actions at 20 Hz, for official_replay.py), summary.json (the live
scorer: score, veto, first 1.0 step, finger peaks, per-substep through-gap counts), trace.jsonl (scorer per control step),
rows.jsonl (demo rows at 10 Hz with finger forces), result.json (events, phases), top/left/right MP4s.
Start pose: nominal unless MAG_START_ROUND / MAG_START_TOOL are set (test only, see ../magkit.py).
"""
import json
import math
import os
import sys
import time
import traceback
from pathlib import Path

for k, v in dict(TOOL_R_OVER="0.072", ARM_PIN_D="0.0005", TOOL_P_LEAN="40", TOOL_PULL_MAX="0.04", TOOL_PULL_ZK="0.5",
                 TOOL_PULL_ZCLIP="0.025", TOOL_PITCH_K="0", TOOL_X_IN1="-0.0125", ARM_NOSE_GAP="0", TOOL_LIFT_BACK="0",
                 MAG_KI="0.15", MAG_ICLIP="0.6").items():  # setting S9
    os.environ.setdefault(k, v)

import numpy as np  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_p0  # noqa: E402
import run_tool3 as t3  # noqa: E402
from run_p0 import Stop  # noqa: E402

E = os.environ.get
HZ = t3.HZ


class D(t3.T):
    def new_stage(self):
        for k in ("gp", "gr", "tcorr", "phi", "rel_bias", "r_cmd", "bg", "bg_gens", "psiP", "psiN", "_live"):
            self.__dict__.pop(k, None)
        self.phase, self.N, self.P, self.p_seated = {}, None, None, 0
        self.left.ki = self.right.ki = float(E("MAG_KI"))
        self.left.clip = self.right.clip = float(E("MAG_ICLIP"))

    def hold_until_full(self, max_s=6.0):
        for _ in range(round(max_s * HZ)):
            if self.mk.magkit.STATE["score"] >= 1.0:
                break
            self.par(self.g_wait("hold_end", 0.1))

    def to_insert(self):
        """run_tool3.run up to the round entry (tool pressed and held, round under the lips, round arm still holding)"""
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
        if self.p_seated < 5:
            raise Stop("tool not seated")
        self.insert()

    def p_let_go(self):
        """tool arm: force to 0 (as the unload), open, back off up the tube axis and away; the tool stays on"""
        P = self.P
        self.p_done = True
        self.tick_bg()
        n = round(1.5 * HZ)
        for k in range(1, n + 1):
            self.g_force(t3.F_PRESS * (1 - k / n), "p_unload")
            self.step("p_unload")
        P.q, P.integ = P.meas.copy(), np.zeros(6)
        self.par(self.g_go(P, "p_open", 0.6, *P.tcp(), 1.0))
        p, r = P.tcp()
        self.go(P, "p_away", 2.0, p + self.mag()[1] @ (t3.U * 0.08) + [0, 0.08, 0], r)

    def n_away(self):
        mp, mr = self.mag()
        p, r = self.N.tcp()
        self.go(self.N, "n_away", 2.0, p + mr @ np.array([0.055, 0, 0.025]), r, 1.0)

    def neg_toolon(self):
        self.to_insert()
        N = self.N
        self.n_step(self.g_go(N, "n_open", 0.6, *N.tcp(), 0.35))
        self.n_step(self.g_wait("n_open_wait", 0.5))
        p, rr = N.tcp()
        self.n_step(self.g_go(N, "n_clear", 1.5, p + self.mag()[1] @ np.array([0.06, 0, 0.02]), rr))
        self.nose()
        self.p_let_go()
        self.n_away()
        self.par(self.g_wait("hands_off", 5.0))

    def neg_hold(self):
        self.to_insert()
        self.p_let_go()
        self.par(self.g_wait("n_still_holding", 5.0))

    def neg_idle(self):
        self.par(self.g_wait("idle", 20.0))

    def neg_drop(self):
        self.par(self.g_wait("settle", 1.0))
        rel = self.pick("round0")
        p, r = self.right.tcp()
        self.events["drop_at"] = np.round(p * 1000, 1).tolist()
        self.go(self.right, "drop_open", 0.6, p, r, 1.0)
        self.go(self.right, "drop_up", 1.0, p + [0, 0, 0.05], r)
        self.wait("drop_settle", 2.0)
        self.away()
        self.wait("hands_off", 3.0)

    def neg_laylips(self):
        mk = self.mk
        self.par(self.g_wait("settle", 1.0))
        rel = self.pick("round0")
        t, p, r = self.round_pose_reachable((mk.X_REAR + 0.0005, 0.0, mk.H + mk.R_CASE + 0.001), [0.0, 0.1, 0.2, 0.3], rel)
        self.events["lay_tilt_deg"] = round(math.degrees(t), 1)
        self.go_joint(self.right, "lay_over", 3.0, p + [0, 0, 0.02], r)
        self.go(self.right, "lay_down", 2.0, p, r)
        self.wait("lay_rest", 0.5)
        self.events["lay_held"] = mk.round_status(self.S(), "round0")
        self.go(self.right, "lay_open", 1.0, p, r, 0.45)
        self.wait("lay_open_wait", 0.5)
        self.go(self.right, "lay_up", 1.5, p + [0, 0, 0.04], r)
        self.wait("lay_settle", 1.0)
        self.events["laid"] = mk.round_status(self.S(), "round0")
        self.away()
        self.wait("hands_off", 3.0)

    def neg_magknock(self):
        """round arm pinches the magazine body 6 mm under the lip tops (fingers across mag y) and pulls it 25 mm up the
        tube axis (the left-arm side grasp of run_p0.hold_mag is unreachable at this layout)"""
        mk = self.mk
        self.par(self.g_wait("settle", 1.0))
        mp, mr = self.mag()
        rr = self.down(math.atan2(mr[1, 0], mr[0, 0]))
        g = mp + mr @ np.array([0.0, 0.0, mk.H - 0.006])
        self.go_joint(self.right, "k_over", 3.0, g + [0, 0, 0.05], rr, 1.0)
        self.go(self.right, "k_down", 2.0, g, rr)
        self.go(self.right, "k_close", 1.0, g, rr, 0.0)
        self.wait("k_squeeze", 0.5)
        self.go(self.right, "k_pull", 2.0, g + mr @ (t3.U * 0.025), rr)
        self.wait("k_hold", 0.5)
        self.events["mag_after_pull_mm"] = self.rows[-1]["mag_in_fix_mm"]
        self.go(self.right, "k_open", 0.8, *self.right.tcp(), 1.0)
        self.go(self.right, "k_up", 1.0, self.right.tcp()[0] + [0, 0, 0.05], rr)
        self.away()
        self.wait("hands_off", 3.0)

    def demo(self):
        self.run()
        self.retreat()
        self.hold_until_full()

    def finish(self, err):
        import imageio
        mk = self.mk
        out = self.out
        sm = mk.magkit.summary()
        trace = mk.magkit.STATE["trace"]
        s = self.S()
        rd = self.rnd()
        res = dict(stage=self.stage, error=None if err is None else repr(err), demo_steps=self.steps, env_steps=len(self.sent),
                   guard_clipped_env_steps=self.clipped, kit_s=self.kit_s, reset_s=self.reset_s,
                   step_wall_s=round((time.time() - self.wall0) / max(1, self.steps), 3), sim_s=round(mk.STATE["sim_t"], 2),
                   official=sm, hands_clear_m=mk.hands_clear(self.emb._env),
                   round_end=dict(rule=self.rule(), base_mm=np.round(rd["b"] * 1000, 2).tolist()),
                   tool_end_mm=np.round(self.tool_rel()[0] * 1000, 1).tolist(), tool_above_table_mm=round(float(s["tool"][0][2] - mk.TOP) * 1000, 1),
                   events=self.events, phase=self.phase, joint_min_margin_deg={k: np.round(v, 1).tolist() for k, v in self.margin.items()},
                   env={k: v for k, v in os.environ.items() if k.startswith(("MAG_", "ARM_", "TOOL_"))})
        (out / "result.json").write_text(json.dumps(res, indent=1, default=float))
        (out / "summary.json").write_text(json.dumps(sm, indent=1, default=float))
        (out / "rows.jsonl").write_text("\n".join(json.dumps(r, default=float) for r in self.rows) + "\n")
        (out / "trace.jsonl").write_text("\n".join(json.dumps(r, default=float) for r in trace) + "\n")
        (out / "actions.json").write_text(json.dumps(dict(control_hz=run_p0.ENV_HZ, actions=[a.tolist() for a in self.sent])))
        for cam, fs in self.frames.items():
            imageio.mimsave(out / f"{cam}.mp4", fs, fps=HZ, codec="libx264", output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])
        print("RESULT", json.dumps(dict(stage=self.stage, error=res["error"], score=sm["score"], veto=sm["veto"], first_full=sm["first_full_step"],
                                        env_steps=len(self.sent), finger_peak=sm["finger_peak_n"], tg=sm["through_gap_substeps"],
                                        clipped=self.clipped, sim_cfg=sm["sim_cfg"], start=sm["start"], rule=res["round_end"]["rule"],
                                        hands_clear_m=res["hands_clear_m"], tool_end_mm=res["tool_end_mm"]), default=float), flush=True)


def main():
    out = Path(sys.argv[1])
    stages = sys.argv[sys.argv.index("--stages") + 1].split(",") if "--stages" in sys.argv else ["demo"]
    r = D()
    print("SIM", json.dumps(dict(low=r.emb._low.tolist(), high=r.emb._high.tolist())), flush=True)
    for i, st in enumerate(stages):
        err = None
        try:
            r.start(out / f"{i}-{st}", st)
            r.new_stage()
            getattr(r, st)()
        except Exception as e:  # noqa: BLE001  recorded
            err = e
            traceback.print_exc()
            r.events["failure"] = repr(e)
        try:
            r.finish(err)
        except Exception:  # noqa: BLE001
            traceback.print_exc()
    os._exit(0)


if __name__ == "__main__":
    main()
