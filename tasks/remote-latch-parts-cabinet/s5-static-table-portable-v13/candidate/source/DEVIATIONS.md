## S5_remote_release

Waivers (kept in this top section so accept.py reads them):
- waive: no leak. Known false positive (ACCEPTANCE.md section 4). The only hit, in every public_text, is "hidden number
  40": the robot limit "each finger squeezes with at most 40 N" collides at 2 significant figures with f2_BL = 40.24 N on
  seed 42 (a sampled hidden value in 34-45 N). No cell names, no hidden strings, no hint words, observation keys clean.
  The reviewer and the lead accepted this waiver (REVIEW.md note 8); f2 is not resampled to dodge it.

### S5_remote_release: changes to card intent or the guideline (flagged)
- **RR-T3 now closes cabinet B's doors** (card: "close both doors of cabinet A"). Why: review M1. A's doors use the
  small f2 band (15-25 N), and every uninformed push latched them at every speed and guard (reviewer payoff_grid,
  48/48), so no listed cell paid off in T3. B's doors carry the large band (34-45 N) and per-door two-stage flags, so T3
  now tests f2 and two_stage with the payoff measured below. Cells: two_BL, two_BR, f2_BL, f2_BR. Right arm.
  (Lead #189: those cells are no longer listed on RR-T3, since the default agent push latches every door; see the last section.)
- **Damage flags added** (the card has none after the DECISIONS ruling): `door_slam` (door reaches the frame stop faster
  than 0.25 m/s at its knob) and `wrist_overload` (untared wrist force above 80 N held for 0.1 s, the agent's abort level).
  Why: lead direction after review (L1), so that slamming a door shut is not free. The 0.25 m/s slam limit is an
  estimate (no catalogue value found): a firm hand close of a light gasketed enclosure door; agent-style pushes reach the
  frame at 0.006-0.17 m/s (probes/logs/grid_*.jsonl), so in practice `wrist_overload` is the flag that fires
  (default-speed push with a 60 N guard: right door 6/8 cases, left door 0/4; reviewer round 2). Both flags are stated in public_text.
- **Robot base layout.** The repo's two-Panda B4 robot in its side-by-side layout (`portable_demo.env.scene(seed,
  'train_a')`: left base (-.40,-.42), right base (-.32,.30), both facing +X) instead of 'test'. Same Pandas, gains, finger
  actuators, wrist F/T and touch sensors. In the facing layout the right arm could not work a cabinet without hitting
  cabinet A or its own opening door (WORKLOG step 2). Accepted by reviewer and lead.
- **Workspace:** objects and poses reach Z 1.40 m (cabinets 0.50 m tall, knobs at 1.255 m, transit 1.365 m), above the
  default 1.18 m box. Declared in the module (`WORKSPACE`, `TOOLS` with the reach box, `OBJECT_BOX`) and in public_text.
- **Hand orientation:** every reference, probe and rule-test motion is top-down with yaw only (standard agent API).
- **Cabinet A/N placement** (lead decision #136): front-plane centre (0.10, -0.29), 0.13 m inboard of the left base's line,
  so the left arm reaches the right box without a twisted arm branch. See the "layout change" section.
- **Public-text reach hint** (lead decision #144): "The rear paddle of the -Y cabinet is near the left arm's reach; approach
  it at paddle height from the front." Skill guidance on geometry, no hidden value. See the "r5" section.

### S5_remote_release: other changes (what, why, evidence)
1. **Latch geometry (rework).** Two-stage doors: claw detent 1.4-2.8 deg (peak 8 N at the knob), first (safety) stage
   catches below 1.35 deg and holds the door 1.35 deg open (3.2 mm proud at the knob, near flush). Gasket zone is the last
   1.2 deg; the claw engages (second stage) at 0.10 deg, door then held within 0.15 deg; frame stop at -0.15 deg.
   Single-stage doors have no detent and no first stage. two_stage is drawn independently per door (two_AL/AR/BL/BR, one
   fair coin each, polish 2026-10-29, see the polish section); T-new flips A's flags; T5 takes B's swapped. Why: review
   M1/L2 (card intent: per-door yes/no, first stage must not be obviously ajar) and round-2 L1 (no coupling between doors).
2. **Success text** no longer gives the angle: "fully closed, flush with the cabinet front, so that it stays shut when its
   knob is pulled". The predicate is unchanged (second stage and angle below 0.5 deg, 20 s hands off). A door in stage 1
   is 3.2 mm proud and holds when its knob is pulled (it moves about 1 mm against the catch), so it can pass a casual pull
   test and still fail (probes/logs/lowguard.jsonl).
3. **Passive door torque rule.** Kick-off spring, claw detent and gasket are written in `step()` from the hinge angle only
   (a nonlinear passive spring element); catches change the hinge range instead of a weld. The detent is felt only while
   the claw is armed. A released claw re-arms only after the door has opened past 2.8 deg (the striker has left it);
   before that the door cannot catch again. Without re-arming, letting go of the paddle early re-latched the door.
   Because the detent now needs an armed claw, a door released from stage 1 springs open instead of sticking in the
   detent zone (review note 7a).
4. **Door/box access geometry.** Fixed round knob near the top of each door (no latch function), boxes on the cabinet
   floor with a front grab tab; cabinet 360 W x 180 D x 500 H mm, 80 mm partition. Top-down grippers collided with every
   lower knob, door-bin or narrow-compartment layout tried (WORKLOG step 2).
5. **f2 bands.** Small 15-25 N (cabinet A), large 34-45 N (cabinet B, the T-new cabinet, T5) instead of 30-45 N, defined
   at the knob (136 mm from the hinge). Payoff (measured, see table below) is speed dependent.
6. **Pop-open ~14 deg** instead of ~5 deg, so a finger can get behind the knob.
7. **Splitter maps:** the non-splitter paddle opens the right door (`LR/R`, `R/LR`), so RR-T2/RR-T5 can avoid the splitter.
8. **Hold time** 20 s as on the card.
9. **Closing probes stop at the latch** (`close_{A,B}{L,R}`): fixed slow ramp along the door arc, stop pushing when the
   door is flush (a geometric event an agent can see), hold, let go. The plan is the same for both contrast values but the
   stop time differs. Without the stop the peak is set by the frame stop and the arm stiffness (open loop 208-328 N for
   both f2 = 18 and 40 N; reviewer probe_ol), so f2 would be hidden. Reported separations are inflated by zero spread
   (deterministic sim, the seeds do not move the probe geometry); with perturbation the reviewer measured about 6x for f2
   and 3.4x for the splitter (REVIEW.md note 4).
10. **Force statistics use wrist force and torque.** A gripped knob passes part of the door torque to the hand as a couple;
    the f2 statistic uses the hinge-axis torque from wrist force + torque / 0.136 m; the two_stage statistic is the force
    drop after the first peak before the final peak (detent signature), on the same equivalent force.
11. **Robot start pose:** both arms start tucked (tips 28 cm in front of their bases at z 1.12 m).
12. **Reference controller:** IK null-space pull toward HOME; transits go straight up to 1.365 m first; joint targets
    are low-pass filtered (IK every 10 ticks) and joints 5-7 are clipped so actuator torque + gravity/bias stays within
    12 N m (real Panda). Max over all references: j5 5.7, j6 11.8, j7 3.1 N m (was 23 / 13.8 / - before; review note 6).

### S5_remote_release: feasibility, forces, clearance (reference, seeds 42/43/44)
- Every target and instance succeeds on every seed, replays identically and sets no damage flag (accept.py check 3).
- Minimum robot-to-object contact distance over all references: -1.3 mm (pads on knobs).
- Working-wrist guard need (untared peak in brackets), current layout (#136; refreshed 2026-10-30, the old line predated
  #136): A 14.6-20.6 N (16.0-22.2), T1 12.3-12.5 (12.5-12.6), T2 11.6-11.7 (12.4-13.5), T3 (B doors) 32.2-39.3 (34.1-41.8;
  >30 N for 0.09-0.19 s), T4-S 26.7-31.2 (28.2-32.8), T4-T 42.1-43.0 (43.6-45.7), T5 40.9-46.0 (42.1-54.6); untared peak at
  most 54.6 N. All below the 60 N maximum guard and the 80 N abort. Max door impact speed in references 0. Source: reference
  audit `probes/layout136/r5/ref_r5.jsonl` (r5 md5 2abc399c; same numbers as `probes/layout136/logs/ref_v10.jsonl`, REVIEW r4
  check 4: T5 guard 46.0 N, untared 54.6); later changes (#164 yaw wrap, #165 text, #189 metadata) left accept.py check-3
  items identical apart from +-1 tick (WORKLOG). Agent-controller emulation needs at most 43.0 N (#164 strict replay T4-T/T5).
- Link-guard window: in the strict emulator replay the left link6 grazes the closed, latched right door of cabinet A at
  69-74 N for one 2 ms step on task A seeds 42/44 (worst 0.1 s mean <= 5.9 N; r5 section, REVIEW r4 N3: harmless), which is
  why R6 in `integration/HARNESS_REQUIREMENTS.md` defines the non-finger link-contact guard on a 20-100 ms window, not per step.

### S5_remote_release: payoff, measured with the agent guard (probes/agent_push.py, probes/logs/grid_*.jsonl)
Guard emulation as in backend/agentic_tasks.py:398-421 (untared force, one sample per 0.05 s tick, stop after 2
consecutive samples above the limit, the stopped arm holds its measured pose). Push along the door arc from 12 deg,
commanded step v per 0.05 s tick (agent default step 8 mm). ok = stage 2, no damage; s1 = left in stage 1; open = not
caught; OVL = wrist_overload. Number = push time in s.

| f2, two-stage | guard | 8 mm | 4 mm | 2 mm | 1 mm | 0.5 mm |
|---|---|---|---|---|---|---|
| 20 N, either | 30 N | ok 0.3 | ok 0.6 | ok 1.1 | ok 1.8 | ok 3.5 |
| 34.5 N, yes/no | 30 N | ok 0.3 | ok 0.6 | ok 0.9 | ok 1.8 | s1/open 3.5 |
| 40 N, yes/no | 30 N | ok 0.3 | ok 0.5 | ok 0.9 | s1/open 1.8 | s1/open 3.4 |
| 45 N, yes/no | 30 N | ok 0.3 | ok 0.5 | s1/open 0.9 | s1/open 1.8 | s1/open 3.4 |
| 20-45 N | 60 N | OVL: right door 6/8, left 0/4 | ok 0.7-0.8 | ok 1.2-1.4 | ok 2.4-2.8 | ok 6.1-6.2 |
| 20-45 N | 30 then 60 N | ok 0.3 | ok 0.5-0.6 | ok 0.9-1.5 | ok 1.8-2.9 | ok 3.5-6.8 |

- f2: no payoff for an agent that pushes at the default speed with the default 30 N guard (it always latches, 0.3 s).
  For a cautious push (2 mm/tick or slower) a large-f2 door is not caught with a 30 N guard; the retry at 60 N costs
  0.6-3.4 s. Fast pushes with a 60 N guard overload the wrist (damage). So f2 decides guard and speed only for
  slow/cautious agents; the payoff is small (about 1-3 s per door).
- two_stage (probes/logs/lowguard.jsonl): with a low guard (12-20 N) a two-stage door stops in stage 1 and holds when its
  knob is pulled (moves ~1 mm), so the agent's own pull check passes but the door fails; a single-stage door stopped short
  follows the pull (3.7-4.1 mm) and shows it is not caught. A two-stage door with f2 = 40 failed in 7/9 low-guard cases,
  f2 = 20 in 2/9. An agent that knows the door is two-stage pushes on to the second peak. No payoff for default-speed,
  30 N pushes (they pass both stages).
- map: time only (one wrong pull about 5 s; a splitter opens both doors and T4/T5 then need an extra close), as ruled.
- **Superseded by lead #189 (last section):** measured against the default push and as % of the full task time, no cell
  clears the strict 10 % line on RR-T1..T5, so every deployment target now lists `cells=[]`.

### S5_remote_release: polish 2026-10-29 (after review round 2, ACCEPT WITH NOTES)
- **L1 (MODULE): two_stage drawn independently per door.** Before, `parameters()` coupled the doors by seed parity
  (two_AL = not two_AR, two_BL = not two_BR), so learning one door in task A gave the other away. Now each of two_AL, two_AR,
  two_BL, two_BR is an independent fair coin from its own RNG stream (`SeedSequence([seed, 5513])`), so the f2 draws (stream 5505)
  and map_A/map_B are unchanged (f2_BL is still 40.24 N on seed 42, so the check-6 waiver above still applies). Key 5513 is the
  first key from 5507 up whose dev seeds show both values of every two_* flag (accept.py check 7) and both equal and unequal door
  pairs in each cabinet. Dev values (AL, AR, BL, BR; 1 = two-stage): seed 42 (0, 1, 0, 0), seed 43 (1, 1, 1, 1), seed 44 (1, 0, 0, 1).
  T-new still flips A's two flags: they are booleans and check 7 requires every listed T-new cell to differ from S on every
  seed, so the flip is forced; an agent can use it only if it learned both of A's doors in S and guessed the generator rule.
  T5 (T-combo) still takes B's flags swapped, so it equals B whenever B's doors match (seed 42, 43).
- **Effect on references:** none (the reference pushes until stage 2 on either latch type). Re-checked: accept.py re-run
  (acceptance/), the reviewer's reference audit on all 21 references (probes/emul97/; numbers in WORKLOG "polish").
- **Docs:** the overload result is side-specific: right door 6/8, left door 0/4 (default speed, 60 N guard). Module header
  comment updated to the current latch angles (detent 1.4-2.8 deg, first stage 1.35 deg, gasket 1.2 deg, claw 0.10 deg).
- **Probe scripts:** `probes/agent_push.py` and `probes/lowguard.py` now use cabinet B (right arm), which stands open in the
  RR-T3 scene; their original logs were written with cabinet A before RR-T3 moved to B. Re-run of lowguard on B
  (`probes/logs/lowguard_B.jsonl`, 36 cases) reproduces `logs/lowguard.jsonl` exactly (same failures: f2 = 40 7/9, f2 = 20 2/9 per
  latch type; same angles and pull travel).

### S5_remote_release: polish 2 (lead decision #115): box pick-up under the agent controller (MODULE: reference plan only)
- **Problem (#97 replay):** the reference plan replayed through the stock-controller emulation succeeded on only 11/21
  target-seed runs. The lead asked for a box-tab fix. **Root cause (measured, probes/emul97/agentdiag_polish.jsonl and
  /tmp/tb_polish_S5_remote_release/tab/):** the tab was never touched. With a top-down hand the arm has one free posture: the
  wrist centre (link6 origin) circles the tool axis 88 mm away. The stock IK (no null-space term, seeded from the measured joints)
  lets it drift with the motion history; in the failing runs it pointed 28-36 deg inboard at the point in front of the box, and
  the forearm (link5) struck the CLOSED neighbouring door's front face (contact at y +17-19 mm from the seam, z 0.27 m) 30-56 mm
  before the fingers reached the tab. All 10 failures had the same posture.
- **Geometry options tried and rejected (honest attempts):**
  - box + tab moved outboard by 15, 20, 25, 30 and 40 mm: the forearm still hits the neighbouring door at the seam edge
    (it reaches about 80 mm inboard of the wrist centre), and from 30 mm on the palm also hits the open door; replay success
    barely moved (15 mm: 6/15 box runs vs 5/15 before; 25 mm, seed 42 only: 3/5, one run needing an 80 N guard);
  - a wider/taller tab cannot help: the hand stops 45-56 mm short of the tab, and the tab cannot reach past the closed-door
    plane (it must clear the door when the door is shut); a higher pick (shelf) is blocked because link7 would hit the cabinet
    top while link5 would still be below the door tops.
- **Fix (reference plan, closed loop on joint angles only):** `posture_fix()` in `take_box()`. At the low point in front of the
  compartment the plan reads the wrist-centre direction from the joint angles (forward kinematics; `joint_pos` is in the
  agent's observation, and the forearm is visible in the cameras). If it points more than 8 deg inboard (R boxes) the hand
  goes back up to a free point in front of the cabinet (cabinet frame x -0.22, y 0.06 toward the open door, z 0.595 m = world
  1.395, inside the declared workspace), turns its yaw at 0.06 rad per tick until joint 7 reaches its limit, turns about 1.1 rad
  further (joint 7 can no longer absorb it, so the arm swings the wrist centre round the tool), turns back to the working yaw,
  and comes down again. Measured: the low-point posture has two stable families, -25..-47 deg (fails) and +20..+41 deg (clear);
  one wind moves the first to the second (+33..+41 deg). Window R (-8, +45) deg; L box (task A only) (-40, +45): the reference
  sits at -26 deg there and the agent emulation at -9..-14 deg, both succeed, no failing L posture was seen.
- **The reference is unchanged:** under the reference controller (null-space pull toward HOME) `posture_fix` only reads the
  posture and returns (R boxes -0.5 deg, L box -26.1 deg on every seed: probes/emul97/ref_posture_log*.jsonl), so the recorded
  reference controls are the same as before; the reference audit after the change is identical to the one before on all 21
  references (success, guard, j5-j7, contacts, length).
- **Side effect:** during a wind the elbow (link4) brushes the top of the OPEN door (no damage, door pushed a few degrees
  further open); guard needs stay at the reference levels (below).
- **Result (agent-controller emulation, probes/emul97/):** with the harness-faithful emulator `agentctl_meas.py` (commanded
  target steps at most 8 mm / 0.08 rad from the previous commanded target per 0.05 s tick, IK seeded from the MEASURED joints with
  12 iterations, dq clip 0.08, no null-space term, PD 550/45 + bias clipped [80]*5+[12]*2, jaw command jumps at tick boundaries):
  21/21 succeed with no damage (before the fix, same emulator: see WORKLOG). The reviewer's original `agentctl.py` (same IK
  seeding, no step limiter): 21/21. Worst guard need per target (2 consecutive 0.05 s samples): A 17-23, T1/T2 11.6-11.7, T3
  31-39, T4-S 18-24, T4-T 32-34, T5 30-38 N, all inside the 60 N guard the public text allows and at the reference levels.
  Minimum contact -0.53 mm. Before the fix, same emulators: strict 12/21, reviewer's 11/21 (every failure = the forearm stop above).
- **Open (#97 joint speed, stock-IK property, not caused by the fix):** in the 6/21 runs whose history lands in the +20 deg
  posture family (no wind needed: 42 T1/T4-S/T5, 43 T5, 44 T1/T4-S) the stock IK flips the elbow while carrying the box to the
  set-down point (joint 3 up to 6.9 rad/s, 2.8-3.2x the Panda limit, about 0.2 s). The same flip is in the pre-fix successful
  runs. The harness has no joint-rate limiter, so the run completes in simulation; a real Panda would stop on its velocity
  limit. Moving the set-down point (x -0.14/-0.17/-0.20, y 0-50 % toward the box) or forcing a wind for that family did not
  remove it (the forced wind broke 5/6 runs). All other runs stay at or below 1.05x (the 1.02-1.05x peaks are single
  10-14 ms joint-4 overshoots during transits). probes/emul97/jspeed_meas_*.jsonl.
- **Payoff:** unaffected (the payoff probes push doors; they never pick a box).

### S5_remote_release: review round 3 C1 (lead decision #132): NOT resolved, needs a layout decision
- **Finding (reviewer):** the 6 "elbow-flip" runs (42 T1/T4-S/T5, 43 T5, 44 T1/T4-S) succeed under the stock controller only
  because the IK flips the elbow at about 3x the joint-3 speed limit. With a #88-style joint-rate limiter they stay in the
  alternate branch and the door closing jams: rate limit 20/21, rate limit + 12 N m j5-j7 cap 18/21.
- **Mechanism (measured):** every posture that clears the neighbouring door at the box (the +20..+41 deg family) is a twisted
  arm branch (joint 1 about 1.6-2.0 rad, joint 3 about -1.9..-2.4). The untwisted branch (joint 1 about 0.3) is the family whose
  forearm hits the closed neighbouring door. The left arm's base sits on the cabinet's centre line, so the right compartment is
  reachable low and in front only in the twisted branch. On the way to the box set-down point joint 3 reaches its lower limit
  (-2.90); the stock IK then jumps back to the untwisted branch (the 3x flip). Under a rate limiter the jump cannot happen.
- **Plan-only attempts that did not work** (scratch in /tmp/tb_polish_S5_remote_release/tab/): set-down moved to x -0.14 /
  -0.17 / -0.20 with y 0-50 % toward the box (the flip stays); free-space winds after the box at two locations, both directions
  (joint 1 moves at most 2.4 -> 1.8, joint 3 stays at its limit; 1/4 runs pass); pre-winding the +20 deg family before the pick
  (0.6 / 1.2 rad, both directions: 0-2/2 under limiter + cap, some 500+ N overloads); the other knob yaw for the closing grasp in
  the twisted branch (0/6). The module is unchanged from md5 ee28a1d1e7db9a3f90be09c9505e2f14.
- **What would fix it (beyond a plan change):** a layout change that puts each arm's base off the cabinet centre line toward
  the boxes it must reach (or deeper/wider compartments), so the forearm clears the neighbouring door in the untwisted branch;
  or a harness IK with a posture (null-space) term (the reference controller's IK has one and never needs the twisted branch).
  Either needs every box reference re-tuned and re-audited: a redesign, so stopped here as the lead instructed.
- **Superseded:** resolved by the layout change below (lead decision #136, option 1), adopted 2026-10-29.

### S5_remote_release: layout change (lead decision #136, option 1): C1 resolved, ADOPTED 2026-10-29
Module md5 ee28a1d1e7db9a3f90be09c9505e2f14 -> ddc28e8716026103ec893fce995ccf43 (pre-layout copy:
`scratch/S5_pre_layout_ee28a1d1.py`). Trial scratch: `/tmp/tb_s5_layout/`; scripts and result logs copied to
`probes/layout136/` (`rl.py` = the reviewer's #88 limiter script plus a non-finger arm-contact metric).
- **What changed (MODULE):**
  1. **Cabinet A/N placement** (`PLACE`): front-plane centre (0.10, -0.42) -> (0.10, -0.29), i.e. 0.13 m toward the middle of
     the table. The left base stays at (-0.40, -0.42), so it now sits 42 mm outboard of the right box (box R at world y
     -0.378; box L at -0.202) instead of on the cabinet centre line. A's paddle face (+Y side) is at y -0.11, 0.23 m from
     B's paddle face (y 0.12). Cabinet B, the robot, all geometry, mechanism, parameters, targets, cells and public text are
     unchanged (the text gives no cabinet coordinates; "cabinet A on the -Y side" still holds).
  2. **Reference plan, re-tuned for the new layout (plan only; every decision reads joint angles or the cameras):**
     - (a) `pull_paddle` on A/N: the hand comes at paddle height from in front of the cabinet, 0.10 m out from the side
       face (clear of both paddle tabs), moves along the side at 0.05 m/s and slides sideways onto the tab (open jaw
       straddles it along x); it leaves the same way. Why: over the cabinet top at SAFE_Z the arm is nearly stretched at the
       new paddle position (stock IK: joint 4 at 1.6x its speed limit; reference controller: sags 8 cm and its fingers
       hit the front paddle's tab, wrist_overload in 9/21 references during the trial).
     - (b) `take_box` on A/N: a first posture check at a check point 0.20 m above the cabinet floor (world z 1.00) in front of
       the compartment, then straight down to the low point and the existing low-point check. Windows (deg, + = outboard):
       check point R (-45, 8, aim 4), low point R (-45, 22, aim 0), L (-55, 45, aim 25) at both. Measured (stock-IK
       emulation, all dev runs): at the check point the R-box posture is +32 deg (on the way down the stock IK drops into the
       twisted branch, j1 > 1.5 rad, and the forearm hits the OPEN right door) or +11 deg (descends to -6 deg, forearm grazing
       the closed left door); winding into +2..+9 at the check point gives +2..+7 at the low point, untwisted and clear of
       both doors. Cabinet B keeps its old windows.
     - (c) `posture_fix` (all cabinets): the transit to the wind point no longer overshoots it by the 8 cm approach (tool z
       now at most 1.40 m; the old wind transit reached 1.475 m, above the declared workspace, a latent issue of the old agent
       replays including T2), and it rises from the low point at 0.05 m/s.
     - (d) `take_box` set-down point (all cabinets): cabinet frame x -0.17 -> -0.20 (centre front). At -0.17 the box corner
       was 3 mm outside the closing door's sweep; in the trial a closing door hit a box set down a few mm off (guard spikes
       of 50-128 N at a door angle of about 30 deg). Now about 20 mm.
- **Why:** review round 3 C1 / lead decisions #132 and #136. With the base on the cabinet centre line, every posture
  whose forearm cleared the closed neighbouring door at the right box was a twisted branch that the stock IK left by a 3x
  elbow flip, so the replay failed under a #88 joint-rate limiter (20/21; 18/21 with the 12 N m cap).
- **Evidence (strict stock-controller emulator `agentctl_meas.py`, measured-joint IK seeding, all 21 target/seed runs):**

| mode | success | damage | left-arm peak joint speed | guard need (N): A / T1 / T2 / T3 / T4-S / T4-T / T5 |
|---|---|---|---|---|
| plain (stock harness) | 21/21 | none | 0.98x | 15-22 / 12.6 / 11.6-11.7 / 31-39 / 27-31 / 40-43 / 38-43 |
| + #88 joint-rate limit | 21/21 | none | 0.99x | 15-22 / 12.6 / 11.6-11.7 / 31-39 / 27-32 / 40-43 / 38-43 |
| + rate limit + 12 N m j5-j7 cap | 21/21 | none | 0.97x | 15-22 / 12.6 / 11.6-11.7 / 31-39 / 27-31 / 40-43 / 39-43 |

  Old module, same emulator, plain: 6/21 runs needed the 3.05-3.16x elbow flip (now none). Right arm (cabinet B,
  unchanged geometry): peak 1.07-1.12x in T2 (was 0.90-1.02x), T3 unchanged. Corrected attribution (review r4 D1): the
  excess came from change (d), not (c): the box set-down at x -0.20 put the right arm in a more folded posture (peak at the
  set-down, t about 68.5 s; with B's set-down at -0.17 it was 0.90x); it is the PD overshoot under a rate-limited target
  (#138). Minimum contact -0.53 mm. Non-finger arm contacts in the emulation (5-step sampling at that time): R-box runs none
  (old module: link4/link5 on the doors in the wind runs, up to 3.7 kN); task A: link6 grazes the closed right door at 13
  and 62 N on seeds 42/44 (old: link5 on the left door 49-60 N on every seed). T2 posture wind (pre-existing, unchanged by
  the layout; per-step values from review r4): right link4 leans on B's open right door, 3.1 kN one-step impact then about
  300 N for 1.2 s, tool pushed to z 1.44 (the 330-490 N above came from 5-step sampling). Both T2 items are fixed in the
  r5 section below.
  Logs: `probes/layout136/logs/final_{plain,rl,rlcap}.jsonl`, old module `old_plain.jsonl`.
- **Reference audit** (`probes/layout136/ref_audit2.py`, reviewer's audit, `logs/ref_v10.jsonl`; run on the comment-only
  predecessor md5 4119f4df, behaviour identical): 21/21 success, no damage, no abort; guard need A 14.6-20.6, T1 12.3-12.5,
  T2 11.6-11.7, T3 32.2-39.3, T4-S 26.7-31.2, T4-T 42.1-43.0, T5 40.9-46.0 N (untared peak at most 54.6 N); j5-j7 at most
  11.86 N m; finger touch at most 22.4 N; tool z at most 1.368 m, x -0.12..0.212, y -0.579..-0.001 (inside the declared
  workspace); min contact -1.37 mm.
- **Cost / changed numbers (honest):** the closing pushes on cabinet N need more force than before (T4-S, T4-T, T5 guard needs
  +7..+10 N; reference T5 up to 46 N), still inside the 60 N guard the public text allows. Force separations of A's cells
  (accept.py check 4): f2_AL 1754 -> 530x, f2_AR 268 -> 419x, two_AL 1533 -> 1074x, two_AR 805 -> 523x; B's unchanged; none
  flagged.
- **Payoff:** cabinet B probes (RR-T3 payoff) are byte-identical to the stored logs: `lowguard` 36/36 (vs
  `probes/logs/lowguard_B.jsonl`), first-drop 24/24 (old vs new module). Extra check on cabinet N's right door with the left
  arm (`probes/layout136/pay_N.py`, the T4/T5 closing situation, old vs new module): agent-speed grid (8/4/1 mm per tick,
  guards 30, 60, 30 then 60) 35/36 identical outcomes; the same failures (1 mm/tick with a 30 N guard on f2 = 40: open on a
  single-stage door, stuck in stage 1 on a two-stage door); one extra damage case (f2 = 40 single-stage, 8 mm/tick, 60 N:
  wrist_overload, consistent with lead decision #48). Cautious low-guard pushes (12-20 N guards, 1-4 mm/tick, then the pull
  test): 12/36 succeed (old 18/36), i.e. uninformed caution fails more often, so knowing f2 / two_stage pays at least as much.
  Conclusions unchanged.
- accept.py (shared, unchanged) on the adopted module: see WORKLOG (layout136 entry).

### S5_remote_release: r5 follow-ups to review r4 (lead decision #144), 2026-10-29
Module md5 ddc28e8716026103ec893fce995ccf43 -> 2abc399c07efc5b57d1dbd6746d7c52f (predecessor: `scratch/S5_pre_r5_ddc28e87.py`).
Evidence: `probes/layout136/r5/` (`rl2.py` = strict emulator + #88 limiter options + per-mj-step non-finger arm contact log).
- **L5, public text (flagged, skill guidance only):** one sentence added to every public_text (`REACH_TEXT`): "The rear paddle
  of the -Y cabinet is near the left arm's reach; approach it at paddle height from the front." Why: review r4 N1, the A/N
  rear paddle can be reached only within about 3 cm above its own height; a top-down approach over the cabinet stalls and
  the stock IK goes singular (2/2 violent failures in the reviewer's probe). Geometry only, no hidden value; check 6 has the
  same single hit as before ("hidden number 40", waived).
- **L6, T2 posture wind (MODULE, plan only; pre-existing problem):** (1) cabinet B winds at `POSTURE_P_B` = cabinet frame
  (-0.30, 0.12 toward the closed door, z 0.585 = world 1.385) instead of (-0.22, 0.06 toward the open door, 0.595);
  (2) B's door is opened to 125 deg for the box (`TH_OPEN_BOX`, hinge stop 140 deg; A/N stay at 100 deg). Why: at the old
  point the right elbow (link4) swung into B's open right door during the wind (3.1 kN impact, ~300 N for 1.2 s, tool to z
  1.44); moving the wind point alone removed that, but the wound posture (+38 deg) then leaned its forearm (link5) on the
  open door at 60-420 N for about 1.2 s while reaching the tab (door at 100 deg). With the door at 125 deg the forearm is
  clear. Grid evidence: `t2_wind_grid_door100.jsonl` (8 wind points / aims), `t2_wind_grid_door125.jsonl` (115/125/135 deg).
- **O1, set-down (checked, B keeps -0.20):** under the new B wind the T2 peak joint speed is the same with B's set-down at
  -0.17, -0.185 and -0.20 (0.67-1.02x plain, 0.67-0.98x with limit + cap; `t2_setdown_xs_check.jsonl`); the 1.11x came
  from the old wind's posture at the -0.20 set-down. At -0.17 the set-down box ends 0.2-0.6 mm outside B's door sweeps
  (measured box pose after the replay), at -0.20 14-17 mm. So the set-down stays at -0.20 for all cabinets (one code path,
  larger margin).
- **Strict emulator replay, all 21 runs, per-step (2 ms) non-finger arm contact (sum of normal forces of all non-finger
  robot links on the world):**

| mode | success | damage | peak joint speed L / R | max non-finger arm contact | time > 50 N (longest / total) | tool z max | guard need (N): A / T1 / T2 / T3 / T4-S / T4-T / T5 |
|---|---|---|---|---|---|---|---|
| plain | 21/21 | none | 0.98x / 1.02x | 73.6 N (one step, task A 44, link6 on closed right door) | 2 ms / 4 ms | 1.397 m | 15-22 / 12.6 / 11.6-11.7 / 31-39 / 27-31 / 40-43 / 38-43 |
| + #88 rate limit | 21/21 | none | 0.99x / 0.98x | 73.6 N (same) | 2 ms / 4 ms | 1.397 m | 15-22 / 12.6 / 11.6-11.7 / 31-39 / 27-32 / 40-43 / 38-43 |
| + rate limit + 12 N m cap | 21/21 | none | 0.97x / 0.98x | 73.6 N (same) | 2 ms / 4 ms | 1.397 m | 15-22 / 12.6 / 11.6-11.7 / 31-39 / 27-31 / 40-43 / 39-43 |

  T2 (where the wind triggers, all 3 seeds): max non-finger contact 0 / 0 / 6.3 N, tool z at most 1.394 m (was 3.1 kN, ~300 N
  sustained, 1.44 m: `base_T2_rlcap.jsonl`). The only contact above 50 N in any run is task A's link6 graze (69-74 N for 2 ms on
  seeds 42/44; mean over the worst 0.1 s at most 5.9 N in any run). Left-arm runs otherwise have no non-finger contact.
- **Reference audit** (`r5/ref_r5.jsonl`, final md5): 21/21 success, no damage, no abort; guard need at most 46.0 N (T5 42);
  j5-j7 at most 11.86 N m; finger at most 22.4 N; tool z at most 1.368 m; min contact -1.37 mm.
- accept.py: see WORKLOG (r5 entry).

### S5_remote_release: commanded yaw within +-180 deg (lead decision #164, #158 audit group 2), 2026-10-29
- **What (MODULE, reference plan only; md5 2abc399c -> 06493eed5eeffc509fa920224ce1f611; backup `scratch/S5_pre_164_2abc399c.py`).**
  (1) Every yaw the plan sends is wrapped into +-180 deg (`wrap()` in `Driver.move`, `swing`, `push_arc`, `wind_shift`). Before,
  the left-arm knob grasps and closing swings in RR-T4/RR-T5 (grasp_knob > transit > goto > move; close_door > swing) commanded
  yaw 190 deg (36 endpoint goals in the #158 audit), and the right-arm wind in RR-T2 streamed 90 -> 382 -> 90 deg; the harness
  (`DualTask.parse_target`) rejects |yaw| > 180. The motion is unchanged: consecutive plan goals are at most 4 deg apart and both
  the harness waypoint and the emulator step along the wrapped difference.
  (2) `wind_shift` is now written as agent commands: turn out = one 0.06 rad command per control tick (joint 7 read after each),
  extra turn = one command (<= 1.2 rad), turn back = at least `WIND_LEGS` = 3 commands of at most 120 deg each (the turn back is
  253-292 deg on the dev seeds, and a single harness command would take the shorter way, in the wrong direction). The per-tick path
  is identical to the old uniform stream. The command yaws are logged in `WIND_CMDS` (diagnostics only). Example (RR-T2 seed 42,
  right arm, start 90 deg): out to 382 deg unwrapped, then the extra and back commands 22.2 -> -74.1 -> -173.7 -> 90.0 deg.
- **Evidence** (`probes/emul97/ws164/`): strict replay `val164.py` (= `layout136/r5/rl2.py` + a parse_target check on EVERY
  control-tick goal of both arms (position in WORKSPACE, jaw 0-0.09 m, |yaw| <= 180) + the #158 GoalLog + a check of every wind's
  command sequence). All 21 runs, plain / + #88 rate limit / + #88 + 12 N m cap: 21/21 each, no damage, success/damage identical to
  r5 (21/21), posture logs identical, 0 parse_target violations over 84,418 goals per mode, GoalLog 0 endpoint / 0 transit
  violations (old module, same script: RR-T4 seed 42 4 endpoint + 75 transit, RR-T2 117 transit; `sanity_old.jsonl`), 21 winds,
  every command < 180 deg from the previous one in the intended direction, 3 commands on each turn back. Guard needs differ from r5
  by at most 0.9 N (float rounding of the wrapped yaw; max 43.0 / 42.8 / 42.8 N, unchanged), max non-finger arm contact 73.6 N,
  peak joint speed 1.022x / 0.986x / 0.984x and tool z 1.397 m, all as in r5 (`cmp164.txt`).

### S5_remote_release: lead #189 payoff pass, strict #187 line (2026-10-30; MODULE metadata only)
- **Rule.** A cell stays on a target only if the informed plan beats the BEST uninformed strategy by >= 10 % of the full task
  time (sim seconds from reset until the success predicate has held for the 20 s hold), or by a success/damage gap.
- **f2_* / two_* (RR-T3, RR-T4, RR-T5): dropped.** The default agent push (8 mm per 0.05 s tick, default 30 N guard, emulated as in
  `backend/agentic_tasks.py`: untared wrist force, one sample per tick, stop after 2 consecutive samples above the limit;
  `probes/agent_push.py` push() unchanged) was run on every door the targets close: cabinet B left/right (RR-T3 scene, right arm,
  doors from 40 deg) and cabinet N left/right (RR-T4 scene, left arm, door released with its paddle first), f2 = 15 / 25 / 34 /
  39.5 / 45 N x single/two-stage on seed 42, plus f2 = 34 / 45 N x both latch types on seed 45 (`probes/p189/push189.py`,
  `push189.jsonl`): **56/56 doors latch in stage 2 in 0.35-0.45 s, door_slam 0, wrist_overload 0** (max frame impact 0.165 m/s
  vs the 0.25 m/s limit; the guard stops every push right after the latch, sampled peaks 33-318 N for one 0.05 s sample, never
  80 N for 0.1 s). The default-push agent therefore never trips a damage flag, and the informed reference's 4 s closing ramp is
  slower than this uninformed push, so knowing f2 or the latch type saves nothing (< 1 % of task time) and gives no
  success/damage gap. (The 60 N guard at default speed still overloads, #48; that is a careless setting, not the best uninformed
  strategy.)
- **#192/#193 R7 guard-sampling check (2026-10-30, DOC only, no module change).** The #189 "default 30 N push latches 56/56"
  holds only with the stock 2 x 0.05 s guard sampling, because it relied on the Driver's 20-40 ms stop lag. With an immediate
  per-step stop (R7) the 30 N push fails 20/56 doors with smooth motion and 40/56 with harness-like jump motion (the stage-1
  detent transient of about 31-33 N trips it). A per-step guard at 45 or 60 N latches 56/56 with no damage, costing <= 0.18 %
  of RR-T3 time and <= 0.08 % of RR-T4/T5 time; a blind 30-then-60 N retry costs <= 3.8 % / <= 1.7 %. So the f2/two cells stay
  dropped under R7 too. Evidence: `probes/p192/` (`push192.py`, `push192_all.jsonl`, `summ192.txt`).
- **map_A / map_B (RR-T1, RR-T2, RR-T4, RR-T5): dropped.** Measured with the reference plan itself (`probes/p189/map189.py`,
  `map189_final.jsonl`, `summ189.txt`): informed = the reference (pulls the paddle that releases the goal door alone); uninformed
  hN = the same plan, but it pulls paddle N first, looks whether the goal door popped (door angle > 5 deg) and otherwise pulls the
  other paddle; any extra open door is closed by the reference's own closing loop. Every map value of the target x seeds 42-47
  (T4 42-46), 168 runs, all succeed, no damage, all doors latched at the end. Gain = uninformed / informed full task time - 1:

  | target | informed (s) | front paddle first (h1): mean / worst | rear first (h2): mean / worst |
  |---|---|---|---|
  | RR-T1 (map_A 50/50) | 69.2-71.8 | +8.8 % / +17.7 % (one extra pull, 12.7-15.3 s) | +11.1 % / +22.1 % |
  | RR-T2 (map_B) | 62.6-63.5 | -0.5 % / 0 % (a splitter pull also opens the right door) | +0.9 % / +1.8 % |
  | RR-T4 (T-new map, 4 values) | 85.9-88.6 | +7.9 % / +24.4 % (6.5 % under the real T-new draw: 3 maps != map_A) | +10.4 % / +28.2 % (9.2 %) |
  | RR-T5 (combo, splitter maps) | 85.9-88.6 | +3.6 % / +7.3 % (extra close of the left door) | +6.8 % / +13.6 % |

  The best uninformed procedure (front paddle first) stays below 10 % on average on every target, with no failure or damage, so
  the map cells are dropped too. The worst single draw costs up to +17.7 % (T1) / +24.4 % (T4); recorded here for the lead in case
  a worst-case reading is preferred (one-line metadata revert).
- **Result:** RR-T1..RR-T5 list `cells=[]` (skill/control targets: paddle release, two-stage closing, box handling); task A keeps
  map_A, f2_AL, two_AL. All nine cells stay sampled, probed (check 4) and transfer-resampled, i.e. discoverable hidden behaviour.
  Module comment above TARGETS gives the numbers. Pre-change copy `scratch/S5_pre_189_3c8e05b9.py`.
- Module md5 3c8e05b9d277ec719f71a07b9ccf5855 -> 76a3ca4b343dba54bec085d523d2b95b. accept.py (shared, unchanged; log
  /tmp/tb_polish_S5_remote_release/p189/accept.log): PASS; check 6 WAIVED with the same single hit "hidden number 40" (top
  waiver); 1 PASS 21/21, 3 PASS 21/21, 5 PASS 18/18, 7 PASS 11/11 (was 26/26: the 15 RR-T4 "transfer changes" items of the dropped
  cells are gone), 8 PASS 38/39, 2/4/9 REPORT; every other item identical to the 3c8e05b9 report, separations identical.
  acceptance/S5_remote_release/report.json md5 879e7ff48a495e58fdd631511fb4de62 (previous acceptance/ copy:
  /tmp/tb_polish_S5_remote_release/p189/acceptance_backup_3c8e05b9/).

### S5_remote_release: records (lead #165, #185; added 2026-10-30 under lead #189)
- **#165 torque text (integration, text only):** ROBOT_TEXT says "Joint torque limits 80 N m on joints 1-4, 12 N m on joints 5-7"
  (was "80 N m (joints 1-5) and 12 N m (joints 6-7)"); module md5 06493eed -> 3c8e05b9, accept PASS with the same check-6 hit and
  every other item identical (WORKLOG "2026-10-30 integration").
- **#185 no harness dependency:** with the adopted layout (#136/#144/#164) the reference plan replays 21/21 through the strict
  stock-controller emulator in plain, + #88 joint-rate limit and + #88 + 12 N m cap modes (layout and r5 sections above;
  `probes/emul97/ws164/`). S5 is NOT conditional on a harness IK posture (null-space) term; the #136 "conditional" status is void.
- **Current module (after #189):** md5 76a3ca4b343dba54bec085d523d2b95b, accept PASS (check 6 waived, same hit).
