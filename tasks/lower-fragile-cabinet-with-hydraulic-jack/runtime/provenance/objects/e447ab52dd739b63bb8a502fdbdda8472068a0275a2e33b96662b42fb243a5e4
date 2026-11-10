## hydraulic_jack

### Changes to card intent (flagged)
1. **J2 is S-only; J4 is the T-new cabinet target.** The card lists J2 as S + T-new and J4 as T-new with the same task, scene and success
   condition, so J2-T-new and J4 would be the same episode twice. J2 keeps the S instance and J4 the T-new instance (new red jack: valve
   and stroke re-drawn; the cabinet weight is re-drawn too because `transfer_parameters` changes every cell). Evidence: card Targets table.
2. **Exploration cabinet sits over the large (blue) jack, not the small one.** Task A (card) needs crate A on the small jack, and both
   loads cannot be on the same jack. The cabinet stays jackable in exploration (DECISIONS N3 ruling) and its weight is still felt
   through the pump handle; only the jack under it differs. J2/J4 still use the small jack, whose valve task A teaches. Evidence: card
   scene vs Task A.
3. **Practice weight box removed.** With the reach-driven layout (below) the rails have only three reachable load spots; crate A, the
   cabinet and crate B fill them. The practice box was there so the agent could practise the valve under a heavy load without risk; the
   jackable cabinet (ruling) and crate A already give heavy and light loads on both jacks. Evidence: reach map (WORKLOG step 2).
4. **J2/J4 start height 50 mm instead of 80 mm.** The cabinet starts on the small jack, whose stroke is sampled 55-75 mm (it must be
   shorter than the large jack's for J1's choice to matter, and the card's 50-90 mm range is split between the two jacks), so 80 mm is
   out of its stroke. 50 mm still gives a damaging drop at any careless valve opening (>= 25 mm/s needs only a few degrees past phi0).
5. **Cells with no payoff were dropped from targets (lead rule 3, payoff check).** J1 uses only `stroke_S` (card: both strokes and
   crate B weight); J3 drops `stroke_T`. Crate B weight does not change how to reach a height, and the large jack always has enough
   stroke for J1 (63 mm) and J3 (60 mm) because its stroke range is 75-95 mm, so knowing it saves nothing. `crate_B_weight` and
   `stroke_T` are no longer hidden cells; they stay sampled scene values (re-drawn in the T instance). Evidence for the remaining J1
   cell: `probes/j1_uninformed.py` / `.json` (uninformed = pump the small jack to its end, then lower, slide, pump the large one):
   with the 63 mm target, small stroke short (seed 42 S, seed 44 T): informed 43 s vs uninformed 88 s to success; small stroke
   long enough (the other 4 instances): same time (23 s).
   The valve cells keep a payoff, mainly in agent decisions and safety (J2/J4 strategy numbers below, review 2 N1; J3 uses the
   same valve rule).
   **Superseded by lead #189 (last section):** on the full task time the best uninformed valve procedure is as fast as the
   informed plan, so the valve cells are no longer listed on J2/J3/J4; J1 keeps `stroke_S` (re-measured against "always use the
   large jack").

6. **Crate weights are 10-30 N (card: 15-60 N)** (lead direction (a) after review 1). Keeping the crates well below the cabinet
   (70-100 N) makes the orifice law sqrt(load) matter when a valve opening learned on one load is reused on another; it also keeps
   the J1 crate push under 30 N. Evidence `probes/weight_payoff.json` (valve values known in both arms; opening chosen for 12 mm/s):
   J2, opening learned on crate A reused on the cabinet: landing 22.6-23.9 mm/s (just under the 25 mm/s limit; a 15 mm/s choice
   would crash) vs 12.1-13.0 mm/s weight-corrected. J3, opening learned on the cabinet reused on the crate: 8.2-8.8 s to lower
   50 mm vs 4.1-4.6 s weight-corrected. **Corrected after review 2 (N3): the weight payoff is a safety margin, not time or
   success.** (Lead #184, last section: the weight cells are no longer listed on J2/J3.) The reused opening still passes in J2 and even lands sooner (1.8-2.4 s vs 4.1-4.3 s), so weight knowledge pays
   only for an agent that aims faster than about 13 mm/s, or as risk margin. The J3 time comparison is not reliable: the
   weight-aware arm that saves about 4 s ends at 5.9 mm on seed 43, outside J3's 10 +- 3 mm band. (In line with lead #18.)
7. **J4 claims only the small jack's valve cells (phi0_S, v30_S)**; `cabinet_weight` was dropped because the T instance re-draws it
   and the text now calls it "a glass display cabinet" (review M2).
   **Superseded by lead #189 (last section):** the valve cells were dropped from J4 too (an uninformed continuous valve sweep
   is 7.6 % faster than the informed plan on full task time), so J4 now lists no cells (`cells=[]`).

### Review 1 fixes (REVIEW.md, 2026-10-28)
- **M1, reset pose:** both hands now start at their park poses (left (-.22, -.36, 1.12), right (.40, .30, 1.12), hand pointing
  down), set as the initial joint configuration in `build()` (IK from HOME, model kinematics only). The right arm's HOME pose was
  10.6 mm from the raised cabinet. Now the minimum robot-to-cabinet distance at reset is 192 mm in every scene, and plain straight
  right-hand moves from the reset pose to (.26, .15, 1.00), (.26, .15, 1.10), (.30, 0, 1.10) and down to the knob set no damage
  flag and never touch the cabinet. Rule test `glass_damage_reach_from_reset` (negative = straight move from reset to above the
  knob and down to it) checks this in accept.py check 5. The reference's special start route is gone (start() only holds).
- **M2, T-new texts:** all deployment texts are instance-neutral ("Two bottle jacks (a small red one and a large blue one)",
  "a sealed wooden crate", "a glass display cabinet"); nothing claims the objects are the same as in exploration.
- **M3, handle stall:** the handle-rule velocity scale is 0.004 rad/s (was 0.05). Same 4-stroke probe, 25 N crate on the small jack:
  handle force 4.77 / 4.39 / 4.31 N at 0.25 / 0.05 / 0.02 rad/s (within 10 %; with 0.05 the reviewer measured 9.08 vs 3.65 N for a
  55 N crate), lift 19.4 / 20.4 / 20.5 mm. (The first version of this item claimed a sub-threshold push creeps at most
  0.004 rad/s; review 2 N2 showed that was wrong because of 2 ms chatter, fixed in the polish item below.)
- **(b) push guard:** with 10-30 N crates the J1 push peaks at 13.7-15.7 N untared (no tick above 30 N); the default guard is enough.
- **(c) IK kicks:** the driver now ramps each joint target to the new IK solution over the IK interval, and the left hand returns
  from the large jack's handle at 0.10 m/s (at 0.20 m/s left j5 reached 13.7-20 N m for 10-20 ms on that move). Final references:
  j5-j7 at most 9.3 N m (left) and 2.5 N m (right), counting the gravity term, i.e. within the real Panda's 12 N m.
- **J1 target height 63 mm (was 62 mm)** so that no dev instance sits within 0.2 mm of the tolerance edge (the small jack tops out
  at stroke - 5 mm); the reference uses the small jack when stroke - 5.3 mm reaches the band.
- **Yaw unwrap:** with the new reset pose the driver unwraps the measured hand yaw next to the park yaw (an arctan2 +-pi flip made the
  wrist spin 2 pi and knock the cabinet over).

### Design and engineering changes (not intent)
- **Layout / reach.** Jacks at x = .05, y = .15 (small red S) and y = .45 (large blue T), under one pair of continuous steel rest rails
  (top z = .935, y -.30 .. .60). With the smoke n3 positions (y = -.15 / +.15) the left arm (base at y = -.15) could not reach the
  small jack's handle grip: IK error 115 mm with j4 at its limit (reach map in WORKLOG). Continuous rails let a load be slid from one
  jack to the other, which J1 needs ("pick a jack with enough stroke").
- **Both jacks have the same retracted height** (pad 5 mm below the rail top) because they share the rails. "Short vs tall" became
  "small vs large" (body diameter 66 vs 80 mm, red vs blue); the stroke ranges differ (S 55-75 mm, T 75-95 mm; card 50-90 mm for both).
  Jack size is a visible cue to stroke, as on real jacks; the exact stroke stays hidden.
- **Valve gain parameterised as v30** (descent speed in mm/s at phi0 + 30 deg under 50 N, log-uniform 20-300 mm/s) instead of a raw
  k_v; this is the card's own definition of the k_v range. Flow saturates 90 deg past phi0 (a real needle valve is fully open within
  about a quarter turn past cracking; without the cap the formula gives metres per second).
- **Pump handle rule.** n3 scaled the handle resistance by w/0.2 rad/s, which under-reports slow strokes (batch2 caveat). Here the
  scale is 0.004 rad/s (review 1 M3; the first version used 0.05). The handle rests on its upper stop under a light return spring (k 0.5 N m/rad, springref 0.55 rad) so that a
  released handle never creeps down and pumps; handle mass 0.07 kg, seal friction tau0 0.15 N m (n3: 0.3). With the overtravel bypass at
  the stroke end the handle feels only tau0 + spring (card: "到頂時泵桿突然沒阻力").
- **Valve and handle values from probes batch2 section 4:** knob + rod frictionloss 0.30 N m, valve turned at 0.35 rad/s with a full
  squeeze; handle stroke at 0.25 rad/s down / 0.5 rad/s up, grasp then ramp along the arc to the stroke start (no target jump).
- **Success conditions.** Task A: crate A reached >= 47 mm (50 mm - 3 mm), back on the rails, last landing <= 25 mm/s, valve closed
  (phi <= phi0), no robot contact with the crate, held 20 s. J1: crate B on either jack pad, underside 63 +- 3 mm above the rails,
  that valve closed, no contact, 20 s. J3: crate A reached >= 57 mm on the large jack, then 10 +- 3 mm, still on that pad, valve closed,
  no contact, 20 s. J2/J4: cabinet resting on the rails, landing <= 25 mm/s, no damage flag, no contact, 3 s (the card gives no hold
  time for J2).
- **Damage flags (sticky, only when the cabinet is in the scene):** `cabinet_impact` (landing on the rails faster than 25 mm/s),
  `glass_contact` (any contact on a glass panel above 15 N), `cabinet_tipped` (tilt > 10 deg).
- **"Two arms cannot lift the cabinet" was not added as a rule (card probe 2 answered by inspection, not simulated).** The cabinet has
  no handles: its sides are glass between 16 mm corner posts (a jaw cannot close around a post without pressing the glass), and the
  16 mm top slab can only be pinched vertically, which the top-down simple-gripper grasp cannot do. So a two-arm lift is not available
  to a simple gripper anyway. If an agent found a hold, the sim would allow it (two 40 N fingers with pad mu 1.5 per hand; 35-50 N per
  wrist is under the 60 N guard); the real limit is the Panda payload (3 kg per arm, 6 kg < 7.1-10.2 kg), which is stated in the
  public text. No device is added. (Open problem for the lead.)
- **Hand workspace stated in public_text:** x -0.30..0.50, y -0.50..0.65, z 0.825..1.18 m (the PortableTask default box); every
  reference pose is inside it. The cabinet top (1.32-1.37 m) is above the box but nothing needs to be done up there.
- **Hidden values are re-drawn when they would match a public number at 2 significant figures** (same rule as
  `tools/accept.py:leaked`), so the hidden-number scan in accept.py finds no match. The distribution change is negligible.
- **Valve cells are visual.** phi0 and v30 show up as the descent speed (card: "這格偏 visual"); they are declared `channel: visual`.
  Weights and strokes are force cells.
- **Pad load is the vertical contact force.** The handle resistance and the flow rule use the summed world-vertical contact force on
  the ram pad (the crate skids cross the pad edges, so summing contact normals over-read a 55 N crate as 105 N; fixed, now 55.0 N).
- **Angle-iron rails:** a 12 mm upstand on each rail's outer edge keeps a sliding load square (without it a pushed crate drifted 5 cm
  sideways and yawed, then sat off-centre on the jack pad and tilted). Inner faces are 102 mm off-centre: the cabinet plinth is
  +-100 mm, and with zero clearance it rubbed the lips and the handle read 30% more (14.6 vs 11.3 N at 72 N).
- **Crates stand on two wooden skids** (capsules across the rails, 80 mm apart so both rest inside the 110 mm pad; at 120 mm apart they
  rode the pad edges and the crate sat 3 mm low) and the skid/rail contacts use solref .02, solimp .9 .95 .001
  instead of the robot scene default (.004, .99 .999 .0001). With the stiff default a pushed crate micro-hopped (+-40 um at 2 ms)
  and the wrist read 115-148 N spikes while sliding a 28 N crate. The crate side face has mu .25 with contact priority 1 (otherwise
  the 1.5 finger-pad mu applies), so a pushing hand does not drag the crate face vertically. Push wrist force after the fix (30 mm/s, rail lips):
  peak 27-33 N untared with the old 15-60 N crates; 16-18 N with the 10-30 N crates.
- **The privileged reference creeps the hand into contact at 10 mm/s before a push and pushes at 30 mm/s** (first-touch transient was 72 N at
  50 mm/s; final reference push peaks 27-33 N untared, longest stretch above 30 N one 2 ms tick; all other reference phases stay at or below 12 N native wrist force:
  valve turning, batch2 target < 40 N, and handle grasp/strokes, batch2 74.9 N transient, are both fixed).
- **Card probe 3 (valve wrist < 40 N) and probe 4 (74.9 N handle transient)** are met by the reference: peak native wrist <= 12 N
  in every valve and pumping phase (batch2 settings + ramped handle grasp).
- **Card probe 1 (valve strategies at phi0 30-120 deg), corrected against the best uninformed strategy (review 2 N1,
  lead #53).** Informed (valve opened straight to the right angle, `probes/valve_strategies.py`): 9.5-11.6 s from grasp, 1-3
  valve commands. A blind quarter turn crashes 3 of 3 seeds (42-142 mm/s landings). The reviewer's uninformed strategies
  (`/tmp/tb_review_hydraulic_jack/r2/valve.log`) use no phi0, v30 or weight: they watch the cabinet height every 0.05 s tick
  and regulate toward about 12 mm/s with 1-1.5 deg nudges.

  | uninformed strategy (J2 S and J4 T, 3 seeds each) | J2 (s) | J4 (s) | landings (mm/s) | damage |
  |---|---|---|---|---|
  | continuous 10 deg/s sweep, stop at 0.8 mm of drop | 10.8-14.8 | 9.4-14.4 | 9.3-15.9 | none (6/6 safe) |
  | 5 deg/s sweep | 16-27 | 13.5-25.4 | (not recorded) | none |
  | discrete 3 deg steps, 0.25 s look each | 18-33 | 16-30 | (not recorded) | none |

  So knowing the valve gains only 1.0-1.3x robot time against an agent that watches continuously, and 1.6-2.8x against the
  best discrete-step agent. In agent decisions it is about 1-3 valve commands against 15-40. The valve payoff is therefore
  mainly fewer decisions and safety for step-wise agents (lead #53 accepted this). The builder's earlier step strategies
  (2 deg / 1 s wait: 57-77 s; 10 deg then close: 24-47 s) were strawmen and are no longer quoted as the payoff.
  **Superseded by lead #189 (last section):** these are valve-phase times; on the full task time a continuous sweep at the
  reference's own valve rate (20 deg/s) beats the informed plan, so the valve cells were dropped from J2/J3/J4.

### Polish 2026-10-29 (POLISH_QUEUE items, lead #54, #97)
- **N2 fix (MODULE): pump creep and chatter under a free sub-threshold push.** Review 2 measured 2 ms chatter of the handle
  (4,744 velocity sign flips per 5,000 steps at a 2.8 N push) and an oil ratchet (each down-tick added oil, the up-ticks
  removed none: +1.6 mm in 10 s while the handle rose; 9.4 mm of crate rise in 20 s at 4.5 N). Cause: the stall law
  `res * clip(-w / W_SCALE)` is a velocity feedback of res/W_SCALE (about 200 N m s/rad) on a 0.021 kg m^2 handle, explicit at
  2 ms. Fix, two parts: (a) below W_SCALE the same law is applied as implicit hinge damping (`dof_damping = 0.3 + lim /
  W_SCALE`, the implicitfast integrator treats damping implicitly); above it the constant resistance as before; (b) oil is
  added only when the handle goes below its lowest angle since the last up-stroke (`PUMP_HYST` 0.005 rad, about 0.1 mm of
  ram lost per stroke, 1.8 %). Re-run of the reviewer's probes (`probes/n2_chatter.py`, log `probes/n2_chatter.log`; 30 N
  crate, last 10 of 20 s):

  | free push at the grip | sign flips / 5,000 steps | handle speed | oil in the last 10 s |
  |---|---|---|---|
  | 2.8 N | 0 (was 4,744) | 0.0016 rad/s, down | +0.36 mm (was +1.6 mm with the handle rising) |
  | 4.5 N | 0 (was 3,607) | 0.0028 rad/s, down | +0.66 mm (crate +2.8 mm in 20 s, was 9.4 mm) |
  | 5.2 N | 0 | 0.0034 rad/s | +0.76 mm |
  | 7 N | 0 | full stroke to the stop | correct |

  The corrected claim: a free push below the stall level creeps at the stall law's rate, at most 0.004 rad/s of handle
  (0.09 mm/s of ram), smoothly and without ratcheting; a real seal would hold still, so a residual creep of up to about
  1.8 mm per 20 s of near-threshold pushing remains a documented modelling simplification. With the hand: grip held still
  20 s +0.000 mm (was +0.015); a gentle press (target 3 mm below the grip) 20 s +0.31 mm (was +0.4). Stroke-rate check
  (review M3 probe, 4 strokes, 30 N crate): lift 19.1 / 20.0 / 20.2 / 20.0 mm at 0.25 / 0.05 / 0.02 / 0.01 rad/s.
- **Lead #54 / N5 (MODULE): task A now requires every landing of crate A to be gentle.** A landing faster than 25 mm/s sets
  the sticky damage flag `crate_hard_landing` (task A only) and the success test requires it unset, so a crashed and
  re-raised crate no longer passes. The task A goal text says so ("a harder landing fails the task even if the crate is
  raised again"; no new number). New rule test `crate_hard_landing` (positive: raise crate A 50 mm, open the valve 70 deg
  past phi0; negative: the task A reference). Other crates keep the card's "crates are recoverable" (J1/J3 failures by a
  crash already fail on position).
- **Reference driver joint-target rate limit (MODULE, reference only):** after the N2 fix shifted the timeline, the left
  hand's return from the large jack's handle crossed the j2 = 0 shoulder singularity at a worse moment: the IK swung j1/j3 at
  3-6 rad/s and left j5 reached 17.2 N m for up to 16 ms (J1 on T, J3). The driver now limits each joint target to 1.5 rad/s
  (`QD_MAX`, below the Panda's 2.175 / 2.61 rad/s), as BUILD_AGENT_BRIEF asks. Re-audit numbers are in the polish summary.
- **Re-review F1 fix (MODULE):** the first polish `damage()` returned only `crate_hard_landing` in task A and dropped the
  cabinet flags, although the task A scene has the cabinet. `damage()` now builds one dict: `crate_hard_landing` in task A,
  plus `cabinet_impact` / `glass_contact` / `cabinet_tipped` whenever the cabinet is in the scene. State test
  (`probes/f1_damage_state_test.log`): with glass_contact and cabinet_tipped set in a task A state, all three cabinet flags and
  crate_hard_landing are reported. J3 keeps no hard-landing gate (lead default).
- **N5 runner note (harness, no bench change):** the reset arm pose lives in `build()`'s initial qpos (park poses, review 1
  M1). The agent runner must start from that state and must not re-home the arms to HOME (lead #55); re-homing brings back
  the round-1 cabinet topple.
- **#97 stock-controller replay (lead #97, binding): PASS 21/21, no fix needed for the controller.** `probes/emul97/std_emul97.py`
  replays each reference plan through the stock DualTask tool: per 0.05 s tick the commanded target steps at most 8 mm /
  0.08 rad from the previous commanded target, IK (12 iterations) from the measured joints, PD 550/45 + bias clipped at
  [80]*5+[12]*2, jaw commands jump at tick boundaries, no joint-rate limiter; a plan move that runs faster than 8 mm per tick
  (the reference travels at up to 0.2 m/s) ends when the commanded target has caught up, as `move_arm` does. Results
  (`probes/emul97/emul97_all.json`, final module): every target succeeds on every instance with no damage; worst-of-25-phase
  guard need: left 33.6-34.2 N on small-jack pumping (above 30 N on 1 of 25 tick phases only, at the bottom of a stroke as the
  handle turns back up; median 10.7 N), left <= 14.8 N on the large jack, right <= 22.1 N (valve and push); no 2 ms sample
  above 80 N; the stock torque clip saturates j5-j7 for single 2 ms steps at each 0.05 s target step (requested up to 96 N m,
  applied <= 12 N m by the harness clip; this is the harness's own behaviour); left j5 reaches 1.04x its 2.61 rad/s limit for
  10-22 ms when the stock IK passes the j2 = 0 shoulder singularity going to and from the large jack's handle (no rate limiter
  in the stock tool; harness note, like #88/#98). Commanded goals stay inside the PortableTask box (z <= 1.12 m), yaw -180..0
  deg, jaw gaps only 0 and 0.09 m (both inside the declared jaw range, #110 check). The public robot text now says, without a
  number, that pumping can briefly read a little above the default guard. A first emulation without the move catch-up
  failed J1 on the two pushed instances and read 61-113 N while pumping the large jack: the plan's 0.2 m/s goals outran the
  8 mm / tick commanded target (`probes/emul97/runs_v0_no_catchup/`); that was an emulation artifact, not a bench problem.

### hydraulic_jack: integration pass (2026-10-30; lead decision #165, text only)
- Robot text, joint torque limits: "80 N m on joints 1-4, 12 N m on joints 5-7"; was "80 N m (joints 1-5) and 12 N m (joints 6-7)". It now matches the #88 limits and the real Panda (j5-j7 12 N m).
  No other module change. Module md5 5ddeb02d -> f9d06d38; accept.py: PASS (shared tools/accept.py unchanged, log /tmp/tb_integ/hydraulic_jack/accept.log): 1 PASS 18/18 items pass; 2 REPORT min 4596 ticks/s; 3 PASS 24/24 items pass; 4 REPORT crate_A_weight 2161966782.51x, cabinet_weight 3559721651.18x, stroke_S 43.58x, phi0_S visual-only, v30_S visual-only, phi0_T visual-only, v30_T visual-only; 5 PASS 36/36 items pass; 6 PASS 13/13 items pass; 7 PASS 25/25 items pass; 8 PASS 41/42 items pass; 9 REPORT 6/6 renders ok. Item statuses of checks 1/3/5/6/7/8 identical to the pre-integration report; check-4 separations identical.

### hydraulic_jack: lead #184 payoff sweep (2026-10-30; MODULE metadata only)
- **`cabinet_weight` dropped from J2 and `crate_A_weight` dropped from J3.** Rule #152/#184: a cell stays on a target only if
  the informed agent beats the BEST uninformed strategy by >= ~10 % of the full task time (incl. holds) or by a success/damage
  gap. Measured on the current module, seeds 42-47 (dev + 3 non-dev; J3 in both S and T-new instances),
  `probes/payoff184/weight_payoff184.py` (results `weight_payoff184.jsonl`): the reference plan unchanged, but the closed-loop
  `Driver.lower` is seeded with a weight guess instead of the true weight (phi0/v30 stay known, so only the weight knowledge
  is removed). Full task time = all reference ticks incl. the final hold.
  | target | informed (s) | practised load reused | nominal 50 N | heavy guess 120 N | light guess 8 N |
  |---|---|---|---|---|---|
  | J2 (cabinet 85-100 N) | 19.9-22.1 | 6/6, -9.7 to -3.5 %, landing <= 21.0 mm/s | 6/6, -4.1 to +1.7 % | 6/6, +1.9 to +3.9 %, landing <= 9.7 mm/s | 3/6 (`cabinet_impact` 26-34 mm/s) |
  | J3 S (crate 14-29 N) | 76.6-78.7 | 6/6, +1.9 to +7.6 % | 6/6, +3.1 to +5.5 % | 6/6, +2.5 to +5.8 % | 6/6, -2.9 to -0.2 % |
  | J3 T-new (crate 16-27 N) | 74.8-81.8 | 6/6, +3.4 to +7.8 % | 6/6, +3.9 to +5.9 % | 6/6, +4.8 to +6.1 % | 6/6, -2.7 to -0.4 % |
  The reference lowering is closed-loop (speed read every 0.1 s, opening corrected), so a wrong seed opening costs at most a
  few seconds. Best uninformed: on J2 the conservative heavy guess (always safe, within +3.9 %; the practised reuse is even
  faster and also safe on these seeds); on J3 the light guess (faster than informed; J3 has no landing-speed damage). No
  success or damage gap for any careful choice; only the reckless light guess crashes on J2. So the earlier "safety margin"
  reading (#18, review 2 N3) is not a payoff against the best uninformed agent.
- Result: J2 cells [phi0_S, v30_S]; J3 cells [phi0_T, v30_T]. `crate_A_weight` stays in task A (exploration). `cabinet_weight`
  is now listed on no target (J4 had dropped it before, review M2); it stays a sampled, probed cell (check 4) that sets the
  physics. The valve cells (phi0, v30) are unchanged (#53: a blind quarter turn crashes 3/3; step-wise agents take
  1.6-2.8x). Check 7 loses the J3 "transfer changes crate_A_weight" items.

- Module md5 f9d06d38 -> a6683e1465f8d8758b95622fef598ecf. accept.py (shared, unchanged; log /tmp/tb_polish_hydraulic_jack/p184/accept.log): PASS, no waivers; 1 PASS 18/18; 2 REPORT min 6481 ticks/s; 3 PASS 24/24; 4 REPORT separations identical; 5 PASS 36/36; 6 PASS 13/13; 7 PASS 22/22 (was 25/25: the three J3 "transfer changes crate_A_weight" items are gone with the cell); 8 PASS 41/42; 9 REPORT 6/6. All other item statuses identical to the pre-#184 report. report.json md5 5324a9a31e178398abee11a6d75aa07f.

### hydraulic_jack: lead #189 payoff pass, strict #187 line (2026-10-30; MODULE metadata only)
- **Rule.** A cell stays on a target only if the informed plan beats the BEST uninformed strategy by >= 10 % of the full task
  time, or by a success/damage gap. Full task time = sim seconds from reset until the success predicate has held for the
  target's hold (J1/J3 20 s, J2/J4 3 s); "all ticks" (incl. release and the extra 1 s) is also logged and gives the same
  verdicts. Script `probes/payoff189/hj189.py`, results `hj189_main.jsonl` (seeds 42-47 = dev + 3 non-dev; J3 in S and T-new),
  `hj189_j3_v2.jsonl` (J3 re-run, final uninformed close), `hj189_corners_v1.jsonl` (J2 range corners), summary `summ189_main.txt`.
- **Valve cells (phi0, v30) on J2, J3, J4: dropped.** Informed = the reference (fast turn to phi0 - 4 deg, open to the computed
  delta at 0.1 rad/s, closed-loop speed correction). Uninformed = the same plan with `Driver.lower` replaced by a procedure that
  uses no phi0, v30 or weight (the review-2 strategies of #53): continuous opening at 20 / 10 / 5 deg/s (`sweepN`) or 3 deg steps
  with a 0.25 s look (`step3`), watching the load height every 0.05 s tick and stopping at 0.8 mm of drop, then regulating to the
  reference's own target speed (10 mm/s J2/J4, 5 mm/s J3; same 0.65-1.35x band) with 1.5 / 1 deg nudges every 0.1 s, then closing
  back to the starting angle. 20 deg/s is the reference's own valve rate (YAW_RATE 0.35 rad/s, wrist < 40 N).

  | target | informed (s) | sweep20 | sweep10 | sweep5 | step3 |
  |---|---|---|---|---|---|
  | J2 (S) | 15.2-17.4 | 6/6, -7.9 % (-10.6..-5.2), landing <= 13.3 mm/s | 6/6, +21.6 % | 6/6, +74 % | 6/6, +99 % |
  | J4 (T-new) | 14.2-17.6 | 6/6, -7.6 % (-11.7..-4.0) | 6/6, +19.0 % | 6/6, +67 % | 6/6, +91 % |
  | J3 S | 72.1-74.4 | stop 2 mm above centre (sweep20e): 6/6, -2.0 % (-4.1..+2.6) | 6/6, +6.1 % | 6/6, +12.7 % | 6/6, +16.3 % |
  | J3 T-new | 70.3-77.5 | sweep20e: 6/6, -0.6 % (-2.0..+2.3) | 6/6, +8.9 % | 6/6, +19.7 % | 6/6, +26.0 % |

  (+ = uninformed slower; J3 sweep5/step3 from the first run, whose close was 8 deg below the detection angle; their times are
  unaffected because those runs closed correctly.) Range corners (seed 42, v30 20 / 300 mm/s x phi0 30 / 120 deg x lightest /
  heaviest load): J2 sweep20 8/8 safe, landing <= 21.0 mm/s (limit 25; informed <= 12.6), -10.6..+2.2 %; J3 sweep20e 8/8, -0.5 %
  mean (-2.5..+2.1), sweep10 8/8 +8.3 %. Only J3 with the reference's tight 0.6 mm stop and a fast sweep fails (T-new seed 47 and
  2 slow-valve corners end 6.3-7.0 mm, just below the 7 mm band edge): an agent that does not know its closing lag aims nearer
  the band centre (sweep20e), which passes everywhere. So the informed plan is never >= 10 % faster than the best uninformed
  procedure and there is no success/damage gap: the #53 "1.0-1.3x" valve payoff was valve-phase time and disappears against a
  watcher at full valve speed. (A blind quarter turn still crashes, as before, but that is not the best uninformed strategy.)
- **J1 stroke_S: kept.** Uninformed "always use the large jack" (slide the crate to the blue jack and pump; its 75-95 mm stroke
  always reaches the 63 mm band) vs informed (small jack if its stroke reaches the band, else slide): 12 instances (seeds 42-47 x
  S/T), all succeed, no damage. Where the small jack suffices (7/12; 47.5-49.8 % of 2,000 sampled instances) informed 43.2-43.3 s
  vs 63.8-64.1 s = +47.6-48.2 %; otherwise identical (63.7-63.9 s). Mean +27.9 % (expected about +23 % over the sampled
  distribution), above 10 %. The other uninformed route (pump the small jack first, fall back) costs 88 s when it is short
  (`probes/j1_uninformed.json`), so "always large" is the best uninformed one.
- **Result:** J1 [stroke_S]; J2, J3, J4 `cells=[]` (valve-control skill targets). Task A keeps crate_A_weight, phi0_S, v30_S,
  stroke_S. phi0_T, v30_T and cabinet_weight are listed on no target; they stay sampled hidden cells (discoverable, physics).
  Pre-change copy /tmp/tb_polish_hydraulic_jack/p189/bak/hydraulic_jack_a6683e14.py.
- Module md5 a6683e1465f8d8758b95622fef598ecf -> e463ba10956c7575d731796d369fd069. accept.py (shared, unchanged; log
  /tmp/tb_polish_hydraulic_jack/p189/accept.log): PASS, no waivers; 1 PASS 18/18; 2 REPORT min 6401 ticks/s; 3 PASS 24/24; 4 REPORT
  separations identical; 5 PASS 36/36; 6 PASS 13/13; 7 PASS 10/10 (was 22/22: the 12 J3/J4 "transfer changes" items of the dropped
  valve cells are gone); 8 PASS 41/42; 9 REPORT 6/6. All other item statuses identical to the a6683e14 report.
  acceptance/hydraulic_jack/report.json md5 01a0a708cedb7cceaee080a1ef15b54d (previous acceptance/ copy:
  /tmp/tb_polish_hydraulic_jack/p189/bak/acceptance_a6683e14/).
- Doc fix (lead #192, 2026-10-30): module header comment refreshed to match HIDDEN_CELLS (7 cells; crate_B_weight and stroke_T are
  sampled scene values) and the current target cells; comment only, AST identical. Module md5 e463ba10956c7575d731796d369fd069 ->
  419c72377905ac67711352f9cd83d209. accept.py (shared, unchanged; log /tmp/tb_docfix192/hj/accept.log): PASS, no waivers; 1 PASS
  18/18; 2 REPORT min 6229 ticks/s; 3 PASS 24/24; 4 REPORT separations identical; 5 PASS 36/36; 6 PASS 13/13; 7 PASS 10/10; 8 PASS
  41/42; 9 REPORT 6/6. Every check-1/3-9 item identical to the e463ba10 report (only check-2 timings differ).
  acceptance/hydraulic_jack/report.json md5 787fe67ffd1663d882fd097599625c5c (previous acceptance/ copy:
  /tmp/tb_docfix192/bak/hj_acceptance_e463ba10/).

## Wave-15 repair (2026-11-09): jack attribution of the raising milestone (A, J3)
- Defect (independent review, CON-02): `max_lift` was updated from the crate height regardless of what held it up, so a crate
  lifted by hand to 50 mm (A) or 60 mm (J3) satisfied the "raise with the jack" milestone. Crate A weighs 1-3 kg, inside the
  stated 3 kg-per-arm payload, so this is a declared-hardware strategy, not a hypothetical.
- Change: per-jack `jack_max_lift` is raised only while the load rests on that jack's pad, no robot geom touches the load,
  the load is off the rails and its height is within 2 mm above the ram height given by the pumped oil volume
  (`lift <= V - GAP + RAM_MARGIN`). A requires the red jack's (S) value >= 47 mm and a landing that comes off the S pad with no
  hand contact on that tick or the one before (`jack_landed_after_milestone`); J3 requires the blue jack's (T) value >= 57 mm.
  Thresholds, tolerances, holds, landing-speed and damage rules are unchanged. `max_lift` is kept as a diagnostic.
- Not changed: J1/J2/J4 predicates. The cabinet (7.1-10.2 kg) exceeds the 6 kg two-arm payload; its hand-lowering bypass remains an
  unenforced-payload evidence gap (no payload guard in the native runtime).
