## B7_couplings

### Changes to card intent (flagged)
1. **remove_F shortened from 7 to 5 segments** (DECISIONS left this to the build agent). New F: push 12, CW30, pull 12,
   CCW60, pull out (was push 12, CW30, pull 24, CCW60, pull 12, CW60, pull out). Why: the 7-segment maze is a puzzle toy,
   not a coupling (REVIEW_A §B7.B); the 5-segment F keeps what made F a T-new target (the unseen "pull 12" value and a
   direction reversal) and is a normal two-stage J-slot. Evidence: reference solves it on every seed (report.json check 3).
   Caveat (review F4/R3): under move-until-blocked motion the only new value, "pull 12" instead of C's "pull 24", is
   invisible: the pull simply stops at the ledge, so C's move sequence (push, turn, pull, reverse turn, pull) opens F.
   remove_F therefore tests no hidden information beyond remove_C's for such an agent (it still differs in geometry).
2. **Handedness is sampled per practice coupling (polish 2026-10-29; was one mirror bit per seed).** `handedness(seed)`
   draws one independent bit for each of A, B and C (+1 = the card's direction, -1 = mirrored), so there are 8 hidden
   worlds and the dev seeds all differ: 42 = (A +, B +, C +), the card's handedness; 43 = (+, -, -); 44 = (-, +, -)
   (`HAND_SALT` = 7110 was chosen so that every bit varies over the dev seeds, A and C are equal on some dev seeds and
   opposite on others, and no two dev seeds share a world). Why: review F3 / lead decision 22: with one bit per seed only
   two worlds existed (seeds 42 and 44 were identical) and turning A once revealed every path. Real bayonets come in both
   hands. Evidence: check 7 randomization PASS; references 21/21.
3. **T-new mapping.** The checker runs every T-new target on the S instance as well, and `transfer_parameters` takes no
   target. So remove_E claims `path_B` (its T value is E's path: B-family, same direction as B, split into two 30 deg
   stages with a 24 mm pull) and remove_F claims `path_C` (T value = F's path: C-family with a 12 mm pull, C's
   direction). The S instance of remove_E / remove_F is therefore a coupling labelled E / F that carries B's / C's path (a
   checker artifact; the harness must run remove_E/F only with `transfer_parameters`, their only setting being T-new).
4. **T-combo D = C's first stage + a last turn in A's direction (polish 2026-10-29).** D's path is push 12, turn 30 in
   C's direction, pull 24, then turn 30 in A's direction, out (non-cell key `path_D`). With equal A/C bits this is the
   card's D (push 12, CW30, pull 24, CW30, out, or its mirror); with opposite bits the last turn goes back to the start
   angle (seed 43: push, CCW30, pull 24, CW30, out). Why: with independent bits D needs a rule for its last turn; tying
   it to A keeps both listed cells (`path_C`, `path_A`) relevant: C gives the first stage, A the last turn's direction
   (when the bits differ, C's own reversal also predicts it). The seat stays separated from the exit channel by 14 mm of
   wall in both forms. Evidence: references 3/3 for remove_D, wrong-model behaviour as before (jams on the wrong turn).

Hand orientation: every reference and probe pose keeps the gripper pointing straight down; the sleeve is turned by yaw
about the vertical only (the standard top-down agent API), so no rpy-orientation tool is needed.

### Other changes (what, why, evidence)
5. **Vertical shafts.** The card (and the old b7) used horizontal shafts pulled sideways; here each post stands vertical:
   push = press the sleeve down, turn = wrist yaw (joint 7), pull = lift. Why: with a parallel gripper from above the turn
   is a pure j7 rotation (no large j5/j6 torques, inside the real Panda 12 N m wrist limit), and gravity does not load the
   turn. Same mechanism and hidden information. Evidence: reference turn torques <= 0.6 N m along the correct path
   (probes p3 trace, WORKLOG).
6. **Track geometry (card: "re-do the bore-track geometry").** Track cut into the sleeve bore as 4 deg box sectors between
   r = 17 and 27 mm; two opposed pins (dia 6 mm, r 12-25 mm, tip 2 mm inside the shell) on the fixed post; channel side
   walls at +-16 deg from the channel axis (pin half-angle 10.2 deg at the inner radius, so +-5.8 deg alignment play);
   channels 10 mm high (2 mm axial play); 1.5 mm radial play between post (dia 31) and bore (dia 34). Closed 6 mm cap;
   open mouth faces the holder. Evidence: with +-14 deg / 9 mm the until-blocked search controllers caught pins on channel
   edges after 1-3 deg of arm yaw compliance; with 2.5 mm radial play the sleeve shifted sideways and pins jammed on
   walls; with 26 mm pins the tips rubbed the shell inside the 1.5 mm play and the search success dropped (notes 8/12)
   (WORKLOG). Every solid part collides with the robot and with everything else (post, pins, plunger pad, track, cap,
   shell, stand bolts); only decals (colour band, cap bars, stand tab) and the hidden plunger stem are visual-only. The
   cap (r 27.5 mm) sits inside the shell so the finger pads touch only the shell. Payoff trial 2026-10-29: the first
   junction now also has a dead-end branch with a false-seat notch (item 21).
7. **Plunger.** Spring plunger on the post top (slide joint, 15 mm preload, damping 1.5), card stiffness 320 N/m used as
   the centre of a per-seed nuisance U(280, 360) N/m (`plunger_k`, not a hidden cell; same for all couplings in an
   instance). Seated net upward force 1.8-3 N.
8. **No rules.** The mechanism is native contact geometry, so `RULE_TESTS = []` and every target has `rules = []`
   (check 5 has no items; check 7's T-combo test passes because [] is a subset of the exploration rules).
9. **Success / damage.** Success = every goal sleeve rests wholly inside the tray walls on the tray floor, not touching
   its post, slow (< 2 cm/s, < 0.1 rad/s), no robot contact, held continuously for 20 s (card). Shaft-clear is implied by
   resting in the tray. The card names no damage flags, so `damage()` returns {}.
10. **Right arm folded back** (`RIGHT_PARK`, hand at x = 0.40) instead of HOME. Why: at HOME the right hand hit left link 6
    when the left arm reached the tray (first smoke run). The right arm is idle; the task is one-armed (card).
11. **Reference controller** uses privileged state: the post axis, the sleeve pose (lateral integral servo that keeps the
    sleeve axis on the post axis, and an aim step that puts the sleeve on each path vertex), and the path. Without the
    servo the pull stick-slipped and once jammed (1065 N, seed 42 B); without the aim step one T instance (seed 44
    remove_E) entered the last turn 3.7 mm high and jammed. Polish 2026-10-29: the driver now starts from the measured
    joint pose and the first grasp move takes 700 ticks (was: the start set the IK target for the final hand yaw at once,
    so j7 spun 90 deg at 4.3x its 2.61 rad/s limit in the first 0.1 s); reference joint speeds are now <= 0.85 of the
    Panda limits and j5-j7 totals <= 4.2 N m.
12. **Reference force transients (untared, what the agent guard reads).** Untared left-wrist force norm over all 21
    reference runs: peaks of 70-90 N, never more than 3 consecutive 2 ms ticks above 80 N, so the 80 N / 0.1 s abort is
    never reached. Guard need with the agent's rule (one sample per 0.05 s tick, stop after 2 consecutive samples above
    the limit): median phase 9.5-11.8 N, worst of the 25 sampling phases 38.2 N (3 of 21 jobs above 30 N;
    probes/emul97/ref_audit.jsonl). The 38.2 N comes from two single-tick pad-chatter spikes that happen to fall 50 ms
    apart on one sampling phase; at the harness sampling point (tick end) the need is <= 15.4 N; for the agent-relevant #97 replay see item 20. Two other harness guards can
    fire on the same transients (review F5); they stop a move but do not abort: `held_object_change` (|load change| > 30 N
    between agent ticks while holding) and the default-guard `finger_contact` check (finger-force rise > 1.5 N), which
    sees the finger lunges of item 13 in about 1-2 % of held agent ticks.
13. **Robot-object penetration (rewritten 2026-10-29 after review F1/R1).** The deepest robot-object contacts over every
    reference (-2.6 to -2.9 mm) are finger-impact transients, not soft contact under torque. A pad loses contact with the
    shell for 1-2 ticks, the finger servo (40 N cap) then drives it in at about 0.85 m/s, and it lands 0.5-0.9 mm past
    the inner face of the 2 mm shell wall, on the track sectors or into a channel void; it recovers within about 3-8
    ticks. This happens 9-33 times per reference, at |wrist Tz| <= 0.14 N m (the wrist is not turning at those ticks).
    All collisions are enabled; nothing is disabled and nothing ends up on the wrong side. The excess over the team's
    ~-2 mm rule is 0.6-0.9 mm for a few ticks. Stiffening the shell cannot fix it (stiffness is not the cause; the
    earlier stiffening attempts all failed for that reason). The reviewer's scratch test with finger joint damping 80
    (repo: 20) cut the lunge speed to 0.36 m/s and the depth to -2.36 mm. The polish's finger target filter (item 17)
    slows commanded closing, not these lunges (the target is already 0), so the depth is unchanged. Payoff-trial module
    (item 21): reference audit 21/21, deepest robot-object contact -2.68 to -2.98 mm (same finger-lunge transients),
    guard need worst phase 38.2 N, joint speed <= 0.83 of the limits, j5 / j6-j7 totals <= 2.1 / 4.9 N m
    (probes/payoff_trial/results/ref_audit_v4.jsonl).
14. **Reach box.** public_text states the B4 tool reach box (X -0.30..0.35, Y -0.40..0.40, Z 0.825..1.18 m). Every
    reference tool pose lies inside it (z 0.862-1.10 m) except the repo's HOME start pose (z 1.259 m), which the first
    move leaves.
15. **Probes** are the same scripted targets for both contrast values (grasp, push 14.5 mm command = pins on the
    channel ceiling, turn 25 deg CW / CCW, or turn 30 deg CW then pull 15 mm), tracked by the robot's joint PD as in
    examples/toy_drawer.py; no aiming, so tick windows are identical for every seed and value. Statistics: turn = largest
    20-tick moving average of |dTz| during the turn (a jam loads the grip until it slips, so medians under-read it);
    pull = median |dFz| over the last half of the pull. Payoff trial 2026-10-29 (item 21): with the dead end a wrong-way
    turn is free, so the A / B probes are now push 14.5 mm, turn 60 deg CW / CCW, pull 30 mm, with the pull statistic
    (the turn-torque statistic is gone); the C probe is unchanged.
16. **Tray instead of loose table placement.** One 200 x 150 mm tray holds both task-A sleeves (slots 90 mm apart).
17. **Finger servo target filter (polish 2026-10-29, #97).** The four finger position actuators get `timeconst` 0.3 s
    (`FINGER_TAU`; first-order filter on the jaw target; repo hand: none). Why: the #97 stock-controller replay jumps the
    jaw target (the harness sets the jaw gap at once), and the repo fingers then close at about 0.85 m/s (about 10x a real
    Franka Hand, LEAD_DECISIONS #101); the first pad to arrive shoved the free 0.25 kg sleeve on its post, pressed it down
    and turned it 36 deg into its track, and the replay failed 0/21 (peaks 3-4.5 kN). With the filter a jumped command
    closes each finger at about 0.15 m/s, falling to about 0.1 m/s at contact on the 58 mm sleeve (a real Franka Hand
    moves at up to about 0.05-0.1 m/s per finger). The final grip force is the same (34.7 N per pad, same 1200 N/m servo
    and 40 N cap), but it builds up more slowly: 0.8-1.0 s to 90 % after a jumped command. The filter state is initialised
    to the open target in `build()` (`d.act[:] = d.ctrl[14:]`; review M1), so nothing moves at reset. Note for the harness:
    drop `FINGER_TAU` once the harness applies its own finger-speed limit (#128); both together would give about 0.05 m/s.
    Tried first: finger joint damping 150 (succeeds, but the damped pads chattered and lifted the guard need to 30-50 N).
    Evidence: probes/emul97/ (pre-polish replay 0/21, final replay 21/21, item 20).
18. **Cap-bar family cue (review F4/R3).** The white bars on the cap are a visible "same kind" cue that is constant
    across seeds: A and D have 1 bar, B and E 2 bars, C and F 3 bars (`MARKS`). It shows the family mapping E~B and F~C
    that the T-new targets rely on; for D it points to A (whose direction D's last turn follows, item 4), while D's first
    stage comes from C. The card allows colour marks as visible cues; it is kept as intended.
19. **Seeds.** Since the polish every dev seed has its own world (item 2); before it, seeds 42 and 44 had identical paths.

20. **#97 stock-controller replay (polish 2026-10-29; probes/emul97/std_emul.py, emul_final_jump.jsonl).** Standard
    DualTask emulation: commanded target steps <= 8 mm / 0.08 rad per 0.05 s tick from the previous commanded target,
    stock IK seeded from the measured joints (12 iterations, no null-space term), PD 550/45 + bias clipped [80]*5+[12]*2,
    no joint-rate limiter, jaw jumps, plan moves waited out. Result 21/21, no abort, never more than 2 ms above 80 N.
    Guard need: <= 15.4 N at the harness's tick-end sampling; median phase 9-13 N; worst of 25 phases 34.1 N (remove_C/F
    seed 43), so state "up to about 35 N" for an unlucky phase. j6-j7 sit at the 12 N m clip for 0.2-0.4 s per run (stock
    PD yaw steps); in task A stock-IK jumps of 0.13-0.23 rad push j5 above the real 12 N m for 16-108 ms (harness IK,
    like #98). Joint speeds <= 0.86 of the Panda limits. Jaw commands 0 / 0.09 m gap only (standard tool).
    Re-run on the payoff-trial module (item 21, md5 f5d1d429; probes/payoff_trial/results/emul_v4.jsonl): 21/21, no abort,
    never more than 2 ms above 80 N; guard need tick-end <= 15.4 N, median phase 8.9-14.0 N, worst phase 22.0 N; j6-j7 at
    the 12 N m clip as before; j5 total peak 69.8 N m (task A stock-IK jumps, as before); joint speed <= 0.86 of the limits.
    With the #88 harness limits (j5-j7 total <= 12 N m, joint targets rate-limited to the Panda limits from the previous
    limited target; `LIM88=1`, emul_v4_lim88.jsonl): 21/21, no abort, same guard numbers, j5 <= 12.0 N m.

21. **Dead-end branch at the first junction (payoff trial 2026-10-29, lead decisions #127 / #152).** Why: with the
    old track a wrong-way turn jammed at once, so knowing the path saved only 0.3-0.5 recoverable jams per run (every
    strategy 12/12, ticks within ~1.5 %), below the #152 rule (>= ~10 % over the best uninformed strategy). Change (track
    geometry only, derived from each path by `dead_ends()`; no new hidden value, no rule, no new object): at the
    push-level junction the circumferential channel also runs the other way, so the junction spans `JUNCTION_DEG` = 120
    deg: 60 deg of dead end behind a 60 deg turn (A, B), 90 deg behind a 30 deg turn (C, D, E, F). Its far end is a blind
    false-seat notch: 12 mm tall behind 60 deg (the same as the real seat pocket); 18 mm behind 90 deg, and the top of
    that notch turns 20 deg back toward the true side as a blind ledge (it looks like the true path's "pull, then turn",
    so an agent must try it). A wrong first turn is therefore free all the way to the far end; the pull there rises into
    the notch and stops (the plunger then holds the sleeve in the notch like a seat), and the only way out is a
    deliberate back-out (push down 12-18 mm, turn back 60-90 deg). Upper junctions are unchanged (a wrong turn there jams
    at once; there is no room for a dead end beside the other channels). Card intent kept: the hidden value is still each
    coupling's release path, learned by push / turn / pull probing, and the force signal is still "stalls on the wrong
    way" (now at the end of the dead end / in the notch instead of at the start of the turn). Walls between channels are
    >= 8 deg (two 4 deg sectors, about 3 mm at r = 22 mm; between F's ledge and its exit) and >= 8 mm axially; the
    track maps for every dev path are in probes/payoff_trial/results/trackmap_v4.txt.
    Probes changed with it (item 15): a wrong-way turn is now free, so the A / B probes turn the full 60 deg and then
    pull 30 mm (`PROBE_SPECS`; statistic = median |dFz| over the last half of the pull): free at the true end (6.3-16.0 N),
    blocked in the notch at the dead end (38.5-43.2 N). Separation path_A 20.0x, path_B 7.9x, path_C 15.3x (all > 3x).
    Tried first (same session, /tmp/tb_b7_payoff/): a 60 deg dead end with a 12 mm notch at every junction (8-world
    payoff of the best fixed uninformed rule 13.5-14.7 % on C / E / F, below the trial's 15 % bar); upper-junction dead
    ends of 20 deg (the until-blocked search controllers then failed often in the narrow upper notches); both dropped.
    **Outcome (narrow re-review of #163 and lead #168): the dead end does NOT make the path cells pay against the best
    uninformed strategy.** A fixed rule that knows the constant 30 deg first turn of every C-F coupling (`probe30`: turn
    30 deg the guessed way, test a short pull, if it jams turn 60 deg back) never visits the far end, so its wrong guess
    costs ~53 agent ticks instead of ~185; informed is then only 7.0-9.9 % faster over 8 worlds (4.9-7.7 % on dev seeds;
    payoff section). The geometry is kept (lead #168: physics and realism judged fine; the false seat holds the sleeve
    and blocks the pull, and it does cost ~20 % against agents that turn until blocked), but no deployment target lists a
    path cell any more (item 22). Second trial (#168, ~2 h box, analysis only, no geometry adopted):
    - a blind notch at the mirror angle of the true 30 deg exit cannot be built: its channel (edges at 14-46 deg from the
      junction start) would merge with the seat column (edges at +-16 deg), turning the push into an optional move;
      a mirror of the whole path is impossible because the mirror upper turn shares the true upper arc (opposed pins,
      180 deg periodic track);
    - a hidden first-turn size (30 / 45 / 60 deg per coupling) does not defeat probing either: the wrong side has no
      exit anywhere, so one pull test at any angle on the wrong side reveals it; estimated wrong-guess cost about 60
      agent ticks for probe30 (30 deg out, one jam, 30 + 45 deg back on average) against ~8 ticks of extra jams on the
      right side, i.e. about 9-10 % gain, plus a card change for the B family (E is B's 60 deg split in two halves);
    - in general, with the handedness bit as the only per-world variable, a wrong first guess can always be detected
      by one short pull test on the wrong side (turn out, test, turn back: 50-80 agent ticks), which caps the gain at
      about 0.5 x 80 / 360 = 11 % for the ~360-tick task. Not pursued further.

22. **Deployment targets list no hidden cells (lead #152 / #163 / #168, metadata only).** First (#163) `path_A` was dropped
    from remove_D (a wrong last-turn guess costs one jam, +12 agent ticks, about 1.7 %). After the re-review's `probe30`
    rule and the #168 strategy-family sweep (payoff section) every remove_* target is below 10 % against the best
    uninformed family (min gain over 8 worlds: C 8.3 %, D 9.9 %, E 8.2 %, F 7.0 %; #168 counts remove_D's 9.9 % as
    below), so `cells = []` for remove_C, remove_D, remove_E and remove_F: they are skill / control targets (remove
    a bayonet sleeve, recover from wrong turns), like the other skill benches (#161). Geometry, settings and
    `transfer_parameters` are unchanged (C <- path_C; D <- C's first stage + A's last-turn direction; E <- B family;
    F <- C family), and task A still lists path_A / path_B; the probes and separation check are unchanged. Check 7 now
    has no T-new transfer items for remove_E / remove_F (no cells) and the remove_D T-combo rules item still passes.
23. **WORKSPACE declared (lead #158).** `WORKSPACE = ((-.30, -.40, .825), (.35, .40, 1.18))`, the reach box already stated
    in public_text (item 14). Goal-in-workspace audit added to the #97 replay (probes/payoff_trial/std_emul.py, asserts):
    every plan move's goal (tool pose of the move's final joint target, incl. servo/aim corrections), jaw gap, yaw and
    max_force_n (30 N assumed, the default; replay guard need <= 22 N) checked as backend/agentic_tasks.py:312-314 does.
    md5 4d4ff8e9: 21/21 replays, 470 move goals, 0 violations (goals span X -0.100..0.095, Y -0.307..0.260,
    Z 0.865..1.100 m; jaw gap 0 / 0.09 m); same with the #88 limits (468 goals, 0 violations). 231 per-tick plan
    waypoints (11 per job) lie outside the box; the harness checks only goals, and the stock stepping from the HOME start
    pose (z 1.259 m, outside the box) passes through such points anyway.

### Informed vs uninformed payoff (2026-10-29: payoff trial #163, re-review, lead #168; supersedes the polish measurement below)
Measured on the dead-end track (geometry md5 4d4ff8e9 = f5d1d429 = current) in all 8 handedness worlds once each (dev
seeds 42 / 43 / 44 plus 47 / 48 / 50 / 57 / 58: every (A, B, C) bit combination once, i.e. the exact expectation over the
bench's sampling), 4 remove_* targets, `probes/payoff_trial/explore_eval.py`. Every strategy runs on the SAME low-level
controller (the reference's `follow_path` with sleeve-pose aiming) with jams charged a fixed 0.6 s, so only the
knowledge differs; all 800 runs succeed (budget 6000 agent ticks), so the payoff is time. informed = follows the path
predicted from the practice paths and the family cues (equal to the true path). Uninformed families (each with first
turn CW / CCW x second turn same / reverse as the first true turn; the best member is taken after the fact per target):
- `rule`: turn until blocked; a wrong first guess explores the dead end (far end, notch, both turns there incl. the
  blind ledge, back out) (results/explore_v4_8w.jsonl);
- `probe15/30/45/60`: turn A deg the guessed way and test a short pull; if it jams, turn back to the other side
  (probe30 = the reviewer's rule, results/probe30_reviewer.jsonl; others results/explore_families_8w.jsonl);
- `sweep15`: the reviewer's magnitude-free alternating sweep with pull tests every 15 deg.
Agent ticks (0.05 s) to the parked hand after placing; gain = informed's time saving over the best member
(family_table.py prints the full tables, incl. dev-only and non-dev-only):

| family | remove_C | remove_D | remove_E | remove_F |
|---|---|---|---|---|
| informed | 364 | 352 | 352 | 358 |
| rule | 463, 21.5 % | 455, 22.6 % | 448, 21.5 % | 450, 20.5 % |
| probe15 | 403, 9.7 % | 396, 11.2 % | 391, 9.9 % | 391, 8.6 % |
| probe30 | 397, 8.3 % | 390, 9.9 % | 384, 8.2 % | 385, 7.0 % |
| probe45 | 400, 9.2 % | 397, 11.4 % | 389, 9.4 % | 388, 7.9 % |
| probe60 | 405, 10.2 % | 398, 11.6 % | 394, 10.7 % | 394, 9.1 % |
| sweep15 | 472, 22.9 % | 464, 24.1 % | 460, 23.4 % | 456, 21.5 % |
| **min, 8 worlds** | **8.3 %** | **9.9 %** | **8.2 %** | **7.0 %** |
| min, dev seeds | 6.5 % | 7.7 % | 5.3 % | 4.9 % |
| min, non-dev seeds | 7.4 % | 8.4 % | 7.6 % | 5.8 % |

The minimum is always probe30 (30 deg is the true first-turn size of every C-F coupling in every world, a constant, so
the rule reads no hidden value). A wrong first guess costs ~53 agent ticks for probe30 and ~185 for until-blocked
turning. Result: every remove_* target is below the #152 bar of 10 %, so the path cells are dropped from all of them
(item 22). Honest summary: knowing the path saves about 20 % of the time against an agent that turns until blocked,
and 7-10 % against one that knows the constant 30 deg first turn and probes there. Cross-check with the until-blocked
search harness (probes/payoff_trial/strategies.py, results/strat_v4_8w.jsonl): informed 352-367 agent ticks, 32/32; best
uninformed search 18.7-20.2 % slower on C / E / F, but those search controllers fail in 21 % of their 256 runs (mostly
the harness giving up; partly real cocking of the sleeve after torque jams, re-review P4), so it is a cross-check only.
The old track (immediate jams) gave about 1 %.

### Informed vs source-free (polish measurement on the old track, md5 7f0e8334; kept for the record)
Re-measured on the polished module (probes/emul97/strat.jsonl, 12 deployment runs per strategy, seeds 42/43/44):
oracle 12/12, 373 agent ticks, 0 jams; literal 12/12, 368, 7; notes 12/12, 370, 9; uninformed "push, twist, pull"
(reviewer's generic prior) 12/12, 369-373, 11-13; source_free 12/12, 423, 49. Knowing the practice paths gives no
measurable time or success gain over a common-sense uninformed prior (review R2; pre-polish: about 374 vs 376); the
earlier source_free 9/12 were search-exhaustion stops after ~300 of 6000 agent ticks, not budget failures. The measurable
gain is the jam count and a few percent of ticks for a literal replay of the matching practice path.

## integration doc fixes (#184)
2026-10-30 (lead decision #184, docs only; no module or sim change; module md5 `f4647ad3`).
- **Formal penetration note (past the team's -2 mm rule; accepted by lead decision #21).** The deepest robot-object contacts are
  -2.6 to -2.9 mm over the polish references and -2.68 to -2.98 mm over the 21 references of the current track geometry (payoff
  trial, kept by #171; `probes/payoff_trial/results/ref_audit_v4.jsonl`), i.e. up to 0.98 mm past the ~-2 mm rule. Cause (item
  13): finger-lunge transients. A finger pad loses contact with the shell for 1-2 ticks, the finger servo (40 N cap) drives it
  back at about 0.85 m/s and it lands 0.5-0.9 mm past the inner face of the 2 mm shell wall; it recovers within about 3-8 ticks,
  9-33 times per reference, while the wrist is not turning (|Tz| <= 0.14 N m). All collisions are enabled and nothing ends up on
  the wrong side of a wall. Stiffening the shell does not help (four attempts, #21); finger joint damping 80 cut the depth to
  -2.36 mm in the reviewer's scratch test, which makes it a harness finger-servo item, not a bench defect. Lead decision #21
  accepted the excess (then described as soft contact under torque; the review F1/R1 analysis in item 13 replaced that cause, the
  acceptance stands). Status: accepted deviation, no waiver of an accept.py check is involved (the -2 mm figure is a team rule,
  not an accept check).
- **Integration lines (text only).** I1 (lead decision #165): ROBOT_TEXT "80 N m (joints 1-5) and a much lower limit on the two
  wrist joints" -> "80 N m on joints 1-4, 12 N m on joints 5-7"; module md5 `5a83041f` -> `aab80fb8`; accept PASS, no waivers
  (1 18/18, 3 21/21, 5 0/0, 6 13/13, 7 4/4, 8 32/33; separations identical 20.04x / 7.86x / 15.32x). #181 follow-up: ROBOT_TEXT
  adds "Working a sleeve may need a wrist force guard up to about 35 N." (the fuller 30 N default / 60 N max sentence failed
  check 6 on hidden turn angles 30 and 60) and the module header is refreshed (no remove_* target lists a cell since #171); md5
  `aab80fb8` -> `f4647ad3`; accept PASS with identical results. No physics, plan, rule or target change.
