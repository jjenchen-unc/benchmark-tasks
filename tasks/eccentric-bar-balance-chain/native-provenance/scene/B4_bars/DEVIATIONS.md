## B4_bars

Card: `cards/B4_bars.md`. Module: `benches/B4_bars.py`. Earlier implementation: `repo_snapshot/backend/playground/benches/b4.py`.
Evidence files are in `scratch/` (final build: audit `audit3_a.jsonl`, payoff `payoff2.jsonl`, heading test `heading.jsonl`; the pre-review files `audit2_a.jsonl`, `payoff_a/b.jsonl` belong to the rejected v1, kept as `B4_bars_v1_rejected.py`).

### Flagged at the top (card intent / guideline / team rules)
- **CARD-INTENT CHANGE (review round 1, lead direction L1): COM offset range widened from |c| 45-75 mm to 10-90 mm, random sign.**
  Why: with 45-75 mm only the sign mattered. In review, "sign + fixed 60 mm" succeeded 23/24 across the supports. Evidence after the
  change (`scratch/payoff2.jsonl`, 3 seeds, S and T): "sign + fixed 50 mm" (the middle of the range) succeeds on 6/24 deployment
  instances (edge_A 1/6, edge_B 2/6, stack_ABC 1/6, stack_CBA 2/6) and on task A 3/3, because the 51 mm riser is wide. The informed
  reference succeeds 27/27.
  To fit an offset of 90 mm on a 0.55 kg bar, the tube is now 50 x 50 x 1.0 mm (shell incl. caps 0.19 kg, was 1.5 mm / 0.27 kg).
  The 1.0 mm wall is an estimated, uncommon thin-wall gauge (standard 50 x 50 stock starts at about 1.5 mm; ASSETS.md, LEAD_DECISIONS #74).
  The slug then sits at most 137.5 mm from the middle and its end stays inside the 155 mm cap. The worst grasp moment
  (0.85 kg x 90 mm = 0.75 N m) tilts the bar 2.2 deg in the 15 N grasp, and the reference still places it (`scratch/worst.py`).
  Feasibility was re-checked on the final build (27/27 references; limits under "Limits" below). edge_B keeps its 40 mm support.
- **Review F1 fixed: heading constraint.** `complete()` now also requires each bar's long axis to lie along world X within 10 deg
  (`abs(R[1,0]) <= sin 10 deg`, either end first), i.e. across the ridge supports. The public text says so ("lie across the
  support with its long axis along the X axis (within 10 deg, either end first)"). Without it, a bar turned 90 deg to lie along
  a 100 mm ridge passed with only the sign of the offset.
  Negative test `scratch/heading_test.py`, which places the first bar turned 90 deg, with results in `heading.jsonl` plus the s42
  edge_A run:
  - On edge_A, edge_B, stack_CBA and A_stack_pedestal, the turned bar rests level and still on the support (z 0.985 m, speed 0),
    both with the true COM and with the review's sign-only 30 mm shift. It now fails in every case (0/28).
  - The across-the-support control succeeds 14/14.
  - The references end at a yaw of at most 0.7 deg.
- **[2026-10-30, lead #152/#184] The COM cells are no longer listed on the four deployment targets (metadata only; B4 becomes a
  calibration/skill bench, like cabinets).** Rule: a cell stays on a target only if the informed plan beats the best uninformed
  strategy by about 10 % of the full task time (holds included) or by a success/damage gap. The best uninformed strategy is in-task
  wrist identification (lift, settle, average, offset = torque / weight); it succeeds 27/27 (`scratch/payoff2.jsonl`) and 24/24 with
  agent-like noisy sampling (REVIEW.md R5, `/tmp/tb_review_B4_bars/r2/payoff3.jsonl`), so there is no success gap, and a
  topple has no damage consequence.
  Its time cost, as a share of the uninformed full task time including the 20 s hold (`scratch/payoff2.jsonl`, first success tick,
  3 seeds x S/T): edge_A 3.9-4.7 %, edge_B 4.3-4.6 %, stack_ABC 8.1-8.6 %, stack_CBA 7.9-8.1 % (relative to the informed time:
  4.1-5.0 % and 8.6-9.4 %). Only without the hold would it reach 11.9-14.7 %, and the hold is part of the task. No target reaches
  10 %, so every deployment target now has `cells=[]`; task A keeps com_A/B/C (exploration exposure). The cells stay sampled, varied
  in T-new, probed (check 4) and discoverable; the payoff paragraph below is kept as the record.
- Otherwise the same hidden information (COM offset per bar, mass as an intermediate), the same task A and the same four targets with
  S and T-new. No waivers: every accept.py check passes as written.
- **Sensor assumption (review F4):** the COM cells assume a real wrist 6-axis F/T sensor, as the repo robot has. The reviewer found
  identification robust with ATI Mini45-class noise. Under a Panda-class estimated wrench (1.5 N / 0.15 N m per sample) it needs
  about 2 s of averaging. This is a team-wide assumption.
- **Tool orientation: standard down-only API is enough.** Every reference grasp is top-down with a yaw about world Z (jaw across the
  bar). No rpy tool is needed.
- **Payoff is mostly time against an agent that knows how to weigh in-task (as the card says: a calibration bench).** Measured
  (`scratch/payoff.py`, 3 seeds, S and T):
  Final build, widened range (`scratch/payoff2.jsonl`):
  - informed (true COM): 27/27;
  - centre guess: 1/27. The one success is edge_B s42-T, where |c| = 17 mm happens to be within the 20 mm half-width;
  - sign + fixed 50 mm: 9/30, of which deployment 6/24 and task A 3/3;
  - stale S notes on the T bars: 0/12;
  - in-task identification from the wrist F/T (lift, settle 0.6 s, average 0.4 s, offset = torque / weight): 27/27, but
    +580..702 ticks (1.2-1.4 s) per single-bar target and +1913..2082 ticks (3.8-4.2 s) per stack. Estimation error at most
    7.3 mm.
  - So every listed COM cell, sign and magnitude, matters. Against an agent that weighs bars in-task, carrying notes saves the
    identification time.
  - [2026-10-30] That identification time is below 10 % of the full task time on every target, so the cells were dropped from the
    deployment targets (lead #152/#184, flagged item above).
- **Mass cells are declared but listed in no target** (card: "只是中間量，不單獨算"). Knowing mass gives no payoff when each bar's
  COM is placed over the support; it is only needed to turn the measured torque into an offset.

### Interface and build
1. **Self-contained module instead of the backend bench.** What: rewrote B4 in the `examples/toy_drawer.py` / bike_pump style
   (robot_root scene + own objects + scripted Driver). Why: the old `b4.py` is built on the backend workholding generator,
   `PortableEnv` and the two-hand `portable_demo.policy.balance`, none of which implement ACCEPTANCE.md §1. Evidence: accept.py PASS.
   `scripts/audit_b4_free_information.py` (imported by the old probes) is not in the snapshot.
2. **Identical bars.** What: all three bars are the same 320 x 50 x 50 mm square tube with a black end cap at -X; only the colour
   differs. The old build used a 320 x 50 x 50 box for A and 310 x 40 x 50 bars with end blocks for B/C. Why: card "外觀一樣的長條";
   the cap makes the bar's direction visible, so a note such as "heavy towards the uncapped end" transfers across the reset.
3. **Physically consistent inertia.** What: inertia from a thin-walled tube (0.19 kg incl. caps, 1.0 mm wall) plus a steel slug on the axis
   placed so that m_s x_s = M c. The old build used a fixed `diaginertia .0015 .006 .007` for every mass and offset. Why: realism;
   the slug always fits inside the tube (x_s <= 137.5 mm, slug end <= 148 mm < 155 mm).
4. **Mass range unified** to 0.55-0.85 kg for A, B and C (card revision 2026-10-28; old A was 0.60-0.68). COM offset: see the
   flagged range change at the top (now 10-90 mm, random sign).
5. **Practice pedestal is a real 2-4-6 block** on its 2 x 4 in face (top 50.8 x 101.6 mm, 152.4 mm high) instead of the old
   60 x 80 x 160 mm block. Why: real product (ASSETS.md); still "wide" against the 24-40 mm supports. A COM-free placement still
   fails on it (payoff: centre guess 0/3 on task A).
6. **Deployment supports keep the old dimensions** (24 mm ridge, 40 mm ridge, 28 x 30 mm point pedestal, 40 mm ridge pedestal; all
   100 mm long where ridged, 160 mm high). stack_CBA's ridge pedestal has the same top as edge_B's support, as in the old build.
   Supports are fixed to the world (a real narrow 160 mm pedestal would be bolted to a base plate).
7. **Layout.** Bars lie along X at x = 0, y = -0.30 / -0.19 / -0.08 m; the support (practice or target) stands at (0, 0.18) m in
   every scene. The deployment scene replaces the practice block by the target support (check 8: target objects deployment-only).
8. **Success predicate** (card: contacts in order; each upright, level, slow; hands off; 20 s):
   - contacts: bar j touches the support (j = 0) or bar j-1, and touches nothing except that and bar j+1 (no table, no robot);
   - level: long-axis tilt <= 0.06 rad (3.4 deg, the old tolerance) and a flat face down within 0.06 rad;
   - heading: long axis along world X within 10 deg, i.e. across the supports (review F1, see top);
   - still: |6-dof velocity| < 0.025 (old tolerance);
   - all of this held continuously for 20 s (card). The old build used a 2 s hold.
   - **Either way up is accepted** (the recent change mentioned by the lead). What: "upright" is read as "a flat face down", any of
     the four long faces, instead of "local +Z up" (old `xmat[2,2] > cos .06`). Why: the tube is square and the slug sits on its
     axis, so a bar rotated 90/180 deg about its long axis has exactly the same support geometry and balance; rejecting it would
     fail physically correct solutions. Checked that this does not loosen anything else: a bar on an edge (45 deg roll) or tilted
     along its length still fails, and an end-over-end flip only moves the COM, which the physics decides. Yaw is constrained
     separately (heading rule). Evidence: predicate in
     `complete()`; all references place the bars face-up anyway.
9. **No damage flags and no rules** (card: native rigid contact, "能不能救：翻了大多可以撿起來再放"). `damage()` returns `{}` and
   `RULE_TESTS` is empty (check 5 passes vacuously).
10. **Reference: one arm, top-down, no hand-over.** What: the left arm grasps each bar at its middle (tool 10 mm above the bar's
    axis, pad bottoms 15 mm above the bar's underside), carries it at Z 1.15 m, and sets it down with the true COM over the
    support centre (privileged), then releases and retreats vertically. The old reference grasped both ends with both arms.
    Why: simpler; the card allows one-arm placement ("一般單臂舉、放"). Travel happens only at carry height: a straight-line move
    from a released bar to the next pickup swept an open finger through the stack (first stack_ABC s42 run failed this way).
11. **Gentle grasp: finger target 12 mm per finger on a 50 mm bar (about 15 N per finger).** What: instead of a full close command
    (30 N per finger). Why: at 30 N the set-down contact gave 4-6 tick bursts of -2.3..-2.4 mm finger-pad penetration and 1-4
    ticks of 42-68 N untared wrist force. At 15 N: pad penetration at most -1.31 mm, untared wrist force at most 33.2 N for a single
    tick. The worst bar (0.85 kg, 90 mm, 0.75 N m) tilts 2.2 deg in the grasp and is placed successfully (`scratch/worst.py`). This answers the card's
    "Probes still needed" item 2.
12. **T-new values** are resampled from the bench's sampling ranges (mass 0.55-0.85 kg as in the card; |COM| 10-90 mm with a random sign, i.e. the widened range flagged at the top, not the card's 45-75 mm) with a guard: the COM differs from the S value by at least 25 mm (more than
    the widest support half-width) and the mass by at least 0.05 kg, so stale notes never solve T by luck (stale 0/12 measured).
    T bars look the same as the S bars (same colours and names, as in the toy example); whether the harness tells the agent the
    bars are new is a harness decision.
13. **Bar friction mu 0.6** (anodised aluminium, estimated) instead of the old 1.2-1.5; supports mu 0.5. Finger pads keep the repo's
    1.5 (MuJoCo uses the larger value per contact).
14. **Probes**: `lift_A/B/C` grasp the bar at its middle, lift 10 cm and hold 1.2 s. Statistics: tared left-wrist torque about world Y
    (COM cells) and tared vertical force (mass cells), averaged over the last 0.8 s. Probes are recorded closed-loop plans (the plan
    never reads the contrast value), as in the toy example and bike_pump.
15. **Public text in millimetres.** Positions and sizes are given in mm, so that no public number matches a hidden mass (0.55-0.85 kg)
    at 2 significant figures (e.g. "0.825 m" would have).

### Limits (all measured over the 27 references of the final build, `scratch/audit3_a.jsonl`)
See SUMMARY.md for the numbers: robot-to-object minimum distance, untared wrist force, j5-j7 torque (incl. the gravity term added by
`step()`), reach box of every commanded goal.

16. **Public text notes the HOME start pose** (review F6 minor): "the arms start at about Z 1260 mm, above the reach box".
17. **j5-j7 headroom at the COM extremes (review R3, note).** stack_CBA with three 0.85 kg / +90 mm bars reaches 11.5 N m on j5-j7
    (ctrl plus the gravity term) with the reference controller: under the 12 N m limit, with only 0.5 N m of headroom.
18. **Stock-controller replay (LEAD_DECISIONS #97, polish 2026-10-29; DOC only, no module change).** `probes/emul97/emul.py` drives the
    reference plan (same Cartesian goals, same privileged COM placement) through an emulation of the standard DualTask move tool:
    per 0.05 s tick the commanded target steps at most 8 mm / 0.08 rad (yaw) from the previous commanded target, IK from the measured
    joints (12 iterations), PD 550/45 + arm bias with the total clipped to the harness clip [80]*5 + [12]*2, jaw commands jump at tick
    boundaries, no joint-rate limiter; the right arm's joint-space park becomes a Cartesian move. Results (`probes/emul97/std_all.jsonl`):
    - success 27/27 (every bar ends level, yaw at most 3.5 deg); no stretch above 80 N; joint speed at most 0.78 of the Panda limits.
    - reference replay for comparison (`probes/emul97/ref_guard.py` -> `ref_guard.jsonl`, LEAD_DECISIONS #73): guard need median / worst
      of the 25 sampling offsets, max over seeds and S/T: A_stack_pedestal 16.9 / 18.2 N, edge_A 16.1 / 17.7, edge_B 16.8 / 18.2,
      stack_ABC 16.9 / 18.2, stack_CBA 16.9 / 18.2 N.
    - guard need (untared left wrist, one sample per tick, stop after 2 consecutive samples above the limit), max over seeds and S/T,
      median / worst of the 25 sampling offsets: A_stack_pedestal 18.6 / 26.3 N, edge_A 16.5 / 24.3, edge_B 15.6 / 22.7,
      stack_ABC 23.4 / 30.8, stack_CBA 19.9 / 31.2 N. The three worst-offset values above 30 N (30.7-31.2 N) are the set-down of the
      second bar onto the first (contact chatter between bars; `probes/emul97/std_hot.jsonl`, `phases.py`); medians stay at or below
      23.4 N, so the default 30 N guard works except at an unlucky sampling phase; 35 N covers every offset.
    - j5 total torque reaches 28-34 N m in single-sample spikes at tick boundaries during the first move from HOME (the hand turns
      90 deg in yaw while descending; IK from the measured joints jumps j5), plus a few 12.7-13.7 N m samples in stack_CBA. This is the
      stock clip (80 N m on j5; harness item LEAD_DECISIONS #88), not the plan: with j5 also capped at 12 N m (`J5CAP=1`,
      `probes/emul97/std_j5cap.jsonl`) 7/7 checked runs still succeed (task A x3, edge_A, edge_B, stack_ABC, stack_CBA). j6/j7 stay at
      the 12 N m clip.
    - pad penetration: -2.4 to -2.5 mm on bar B when the jaw command jumps closed (fast sim fingers, harness item #101); the reference
      with its ramped close stays at -1.31 mm, and the same emulation with the jaw following the plan ramp per tick gives -1.34 / -1.30 mm
      (`RAMP=1`, `probes/emul97/std_ramp.jsonl`). Other contacts: finger pads on the held bar only.
    - gripper commands used: 0.012 and 0.045 m per finger (jaw gap 24 and 90 mm), within the standard 0-0.09 m jaw gap.

### B4_bars: current module (lead #189 record, 2026-10-30)
- Current module md5 1c92ac4a84f7851529d37766029c2041 (lead #184/#187 polish: COM cells dropped from all deployment targets). Shared accept.py: PASS, no waivers (1 18/18, 3 27/27, 5 0/0, 6 13/13, 7 6/6, 8 47/48; 2/4/9 REPORT); acceptance/B4_bars/report.json md5 a836011223022208951bc0787490296a.
