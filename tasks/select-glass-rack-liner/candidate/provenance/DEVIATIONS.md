## C1_liners

No acceptance check is waived. Revised after review rounds 1 and 2 (REVIEW.md: round 1 M1/M2, round 2 R2-M1/R2-M2) and the lead decisions of 2026-10-29.

### Flagged at the top: changes to the card's intent, sampling or harness requirements
1. **Six liners, n_hold drawn from {2, 3} (card: four liners, n_hold {1, 2}). This is lead decision L1.**
   - What: six look-alike liners in two rows of three. Per liner set, 2 or 3 liners hold dry (equal probability), drawn from
     U(0.46, 1.0); the rest slip, drawn from U(0.15, 0.27); then shuffled. The gray zone 0.31-0.41 is never sampled.
   - Why: C1-T3 needs two holding liners at once. With the card's n_hold = 1, T3 is infeasible. With four liners and n_hold {2, 3},
     a blind pick succeeded 1/2-3/4 of the time (round-1 review). With six liners a blind pick succeeds 1/3-1/2, close to the
     card's 1/4-1/2. Dev seeds: the S sets have n_hold = 2; the T sets have 2 (seed 42) and 3 (seeds 43, 44).
2. **Wet friction is sampled stratified (card rule kept, distribution made explicit).** mu_wet = mu_dry x [0.3, 1.2], capped at
   1.0, never in the gray zone, sampled per liner (DECISIONS C1). One or two dry-holding liners (never all of them) also hold
   wet. A liner with dry mu <= 0.27 cannot reach 0.46 within x1.2, so the wet holders must be dry holders. Without this, T2 is
   often infeasible. Values that would round onto the 0.31 gray-zone edge are clipped to 0.3099 (review N2). Material reading (ASSETS): wet holders = open-cell foam; dry holders that lose grip wet = smooth rubber;
   slipping liners = vinyl film. On the dev seeds the dry-best liner fails wet on seed 44.
3. **Contact model of the glass on a liner, release behaviour, and reading mu by force (review M2, R2-M1).**
   - Contact: pair and liner geom solref 0.01, solimp 0.95 0.99 0.001 (robot scene default 0.004 / 0.99 0.999 0.0001). The glass
     sinks about 0.3 mm into a liner (plausible for rubber / foam / vinyl). History (scratch/chatter/, scratch/slap/):
     - stiff default (round 1): a dragged glass chatters (contact opens every few ms, 50-90 N single samples); the agent's
       default 8 mm/tick read is wrong;
     - pair solref 0.02 / solimp 0.9 0.95 (round 2): chatter gone, but a glass stood at the liner centre slid off holding liners
       (mu 0.46, 0.50) after the release slap; the round-2 reference hid this with a tuned 25 mm up-slope placement (R2-M1);
     - final (round 3): keeps grip on release and keeps the slow drag readable.
   - **Release grid (every sampled holder holds; scratch/slap/grid46.log, grid50.log, grid55.log).** Seed 44, liner 1, glass
     released upright from 1.5 mm, full 20 s hold. mu = 0.46 (band floor), 0.50 and 0.55, each dry and wet, glass at the liner
     centre and at +25 / -25 mm along the slope, slots 0 and 1: **36/36 succeed**, no `cup_fell`. The glass slides 21-39 mm after
     the release slap (about 12 mm of that is the kinematic shift of seating flush on the 20 deg board) and ends at most 48 mm
     below the liner centre; creep in the hold window 0.56 mm per 20 s. The reference stands the glass at the liner centre
     (`CUP_UPSLOPE` = 0); no placement knowledge is needed. Honest caveat: the reviewer's rigid-body estimate of the slap slide is
     3-5 mm at mu 0.46-0.5; MuJoCo gives about 30 mm with every contact setting tried (stiff round-1 contact: 31-41 mm).
   - **Known failure envelope of the release slide (review round 3 N1; reviewer's /tmp/tb_review_C1_liners/r3/gridA.log,
     gridB.log, seed 44).** At mu 0.46 (the band floor) the glass slides 32-53 mm relative to the liner after release (about
     12 mm of it is the kinematic seating shift, so the true slide is about 20-40 mm, against about 3-5 mm for a real glass).
     Two placements that a real glass would survive **slide off** a holding liner at mu 0.46: (a) the glass upright with its
     centre 40 mm down-slope of the liner centre (rim about at the liner's lower edge); (b) the glass 25 mm down-slope and tilted
     5 deg up-slope in the grip. Placements that hold at mu 0.46: the centre (upright, or tilted +-5 deg in the grip, or dropped
     5 mm), 25 mm down-slope upright (dry and wet, slots 0 and 1), and 25 mm up-slope with a -5 deg tilt. At mu 0.8 the 40 mm
     down-slope placement holds. So the envelope only bites holders near the band floor with an off-centre, down-slope placement;
     a centre placement always holds (lead decision #86: accepted as documented).
   - Readability at agent sampling (reviewer's `exp_c` harness: target steps at most N mm per 0.05 s tick, IK once per tick, one
     wrist sample per tick; re-grasp, 4 mm press, 30 mm drag; ratio of the means of the per-tick samples; 3 sampling phases per
     cell; logs scratch/slap/readV2.log, scratch/chatter/V2_8_05.log; the 2 mm row is widened to the round-3 reviewer's 4-phase
     result, /tmp/tb_review_C1_liners/r3/drag.log, where one phase read 0.625 for mu 0.46):

     | step per tick | calls per 30 mm drag | read of mu 0.27 | read of mu 0.46 | guard needed | held_object_change |
     |---|---|---|---|---|---|
     | 8 mm (agent default) | 4 | 0.25-0.57 | 0.30-0.97 (unreliable) | up to 54 N | 0-1 |
     | 4 mm | 8 | 0.25-0.27 | 0.21-0.53 (one misread) | up to 41 N | 0-1 |
     | 2 mm | 15 | 0.25-0.28 | 0.42-0.63 | up to 41 N | 0-1 |
     | 0.5 mm | 60 | 0.27 | 0.45 | up to 11 N | 0 |

     A drag at 2 mm per tick or slower, about 15 single-tick calls per tested liner, reads mu on the right side of a 0.41 cut-off (guard about 45 N at
     2 mm/tick, default 30 N is enough at 0.5 mm/tick). Faster drags are unreliable. The lead accepts this. The scripted
     reference probe (10 mm/s, 1 kHz average) reads 0.272 / 0.463 dry and 0.271 / 0.475 wet for 0.27 / 0.46. The ~2e8x force-separation figure in report.json only
     reflects zero run-to-run spread.
   - Rejected probes (round 1): grasp-and-drag without re-grasp (the glass pivots in the grip), and pushing the glass with the
     closed fingers (the 1.5-friction pads wedge it).
4. **Liners are 6 mm thick (card: about 4-5 mm).** A top-down parallel gripper pinches the 70 mm wide liner by its long edges, with
   the pad bottoms 2 mm above the table.
5. **Squatter tumbler with a thick base.** A uniform 68 x 80 mm glass released upright from a top-down grasp onto the 20 deg liner
   swung over on rim impact and tipped. The glass is now 74 x 70 mm with its CoM 28 mm above the base (static tip angle 53 deg).
6. **Timestep 1 ms: the agent wrapper must use 50 substeps per 0.05 s control tick (harness requirement).** The repo wrapper uses
   25 (`dual_arm_demo/env.py:197`), which would halve the control tick, the guard's 0.1 s and the hold timers. Every guard number
   here assumes 50 substeps.

### Other changes (what, why, evidence)
7. **Solver settings.** timestep 1 ms, `noslip_iterations` 20, `multiccd` enabled, cup-liner pair `solreffriction` 0.001 (clamped
   to 2 dt). Source: probes batch1 #5. With the item-3 contact a holding glass creeps 0.56 mm in 20 s. About 4,600-5,600 ticks/s.
8. **Success definition.** After the last robot contact the glass gets 0.5 s to settle. Then for 20 s it must stay within 5 mm,
   stay upright (within 15 deg of the rack normal), touch exactly one liner and neither the bare board nor the table, and have no
   robot contact on it or on that liner; that liner must touch the rack. This rules out leaning the glass against a liner edge on
   the bare board. T3: two glasses on two different liners at the same time. T2: the liner under the glass must be wet. All
   stated in `public_text`. Wave 16: the settled position is fixed once per placement; if the glass then drifts > 5 mm (or
   otherwise stops qualifying) the placement is void until renewed robot contact (no hands-off re-settling window).
9. **Damage `cup_fell` (card: "broken": falls off the rack or impact above a threshold), fixed after review M1.**
   - Sticky flag: a glass that rested on the rack *while released* ends up below the rack's low edge, or tips past 60 deg,
     while not held.
   - Holding a glass resets the "was on the rack" memory, so catching it, or picking it up and standing it back on the table,
     is not a fall. This keeps the card's recovery path.
   - New rule test `cup_fell`. Positive: blind placement on the slipperiest liner, so the glass slides off. Negative: a holding
     glass is re-grasped and stood back on the table; it must not fire and no damage is allowed. It passes on all seeds.
     Round 2 (R2-M2): the re-grasped glass stays tilted 20 deg in the grip, so the table set-down now ends 15 mm higher. All
     seeds: right wrist peak 10.4 N, guard need 8.9 N, j5-j7 9.45 N m (was up to 263 N / 77.8 N / 13.8 N m).
   - Not done (review note, optional): basing the flag on unheld fall height or impact speed. Known edge cases (review round 3
     N3, carried over): a glass dropped onto the table from height is not flagged, and neither is a glass the robot itself shoves
     off the rack (while it is touched, `held` resets the "was on the rack" memory). A glass that clips the rack's low edge from
     the table side and tips over is flagged.
   - There is no separate impact threshold, because there is no source for a breaking speed.
10. **Wet rule.** A liner whose centre is inside the pan's water volume becomes wet once. Its pair friction with both glasses
    switches to its wet value and it turns glossy (a visible clue the card allows). It stays wet. Wetting the glass is not
    modelled. T2 keeps "dip the liner" (lead).
11. **No lip on the rack; bare board mu 0.30.** A lip would stop the glass. Glass on varnished wood at 0.30 < tan 20 deg slides.
    Liner backing friction is fixed (1.0 on the rack, 1.1 on the table).
12. **Hidden cells are lists over the six liners** (`mu_dry`, `mu_wet`, object `liner_1`), with contrast 0.27 vs 0.46. The cell was
    renamed from `mu` because the leak check prefix-matches "mu" in "must".
13. **Top-down grasps only** (standard agent API). Every commanded goal lies in the default B4 reach box (declared as
    `WORKSPACE`, and stated in `public_text` in mm). The start pose is the repo HOME (tool at Z 1.26 m).
14. **Layout.**
    - Liners in two rows of three at x -0.05 / 0.05 / 0.15 m, y -0.33 (liners 1-3) and -0.14 (liners 4-6).
    - Glasses at (0.28, -0.30) and (0.28, -0.16); pan at (0.05, 0.035), inner 140 x 170 mm; rack at (0.05, 0.25).
    - The reference parks both arms first, and the left arm carries liners via a waypoint (-0.06, -0.05).
    - The reference stands the glass at the liner centre (round 2 used a tuned 25 mm up-slope offset; removed, see item 3).
    - Liner dips and the set-down after a dip end 3 mm higher than in round 1 (removes the 272 N pad-on-table transient).
15. **Practice tray** is a free flat tray; nothing depends on it.
16. **Liner materials** (review N3) are named in ASSETS: vinyl film (slips), rubber (holds dry, slips wet), open-cell foam (holds
    dry and wet).
17. **Gentle liner pinch, found by the #97 stock-controller replay (polish, 2026-10-29; MODULE change: reference plan + one
    public-text sentence, no physics change).**
    - What: the reference pinches a liner with a finger target of 20 mm (jaw gap 40 mm on the 70 mm liner, about 18 N per finger)
      instead of a full squeeze (target 0, 40 N per finger), in `place_liner`, `dip_and_return` (wet probe) and the `wet` rule
      test (`LINER_JAW`). `SCENE_TXT` now says: "The liners are thin and soft: pinch one across its 70 mm width with a light grip
      (a jaw gap of 40-60 mm, with the pads close to the table); a hard, fast squeeze can make the fingers ride over its 6 mm
      edge." (The parenthesis said "a jaw gap of about 40 mm" until the 2026-10-30 integration pass, lead decision #124.)
    - Why: replaying the round-3 reference through an emulation of the stock DualTask tool (commanded target stepped at most 8 mm
      per 0.05 s tick from the previous commanded target, stock IK from measured joints, total torque clip [80]*5 + [12]*2 incl.
      bias, no joint-rate limiter, jaw commands as one jump; probes/emul97/std_emul_c1.py) failed 18/18: the jaw jump to 0 slams
      the pads into the compliant liner edge (pads 2-6 mm up a 6 mm plate), the hand pops up 5 mm, the pads ride over the edge
      and the liner stays on the table (single-tick wrist spikes 230-337 N), so the glass then slid off the bare board. A
      tick-stepped jaw ramp, a stiff liner contact, or a jump to a 20-25 mm target all pick the liner up; a jump to 10-15 mm
      does not (/tmp/tb_polish_C1_liners/dbg2.py, dbg3.py). The plan change was chosen over re-stiffening the liner contact
      because the liner geom contact also sets the release-slide and drag-readability behaviour validated in review round 3.
    - Evidence (probes/emul97/): with the gentle pinch the stock-controller replay succeeds 18/18 in both goal modes (direct: each
      move's end goal at the 8 mm/tick cap; path: the reference path fed once per tick), no damage. Guard need (untared, 2
      consecutive 0.05 s samples, worst of all 50 sampling offsets): left at most 9.3 N, right at most 17.1 N (direct) / 26.1 N
      (path, T3 glass placement); nothing above 80 N for more than 2 ms (single-tick finger-impact spikes up to 141 N in
      direct mode at the liner pinch). The jaw-jump pinch dents the compliant liner by up to 3.4 mm for a few ms (pads on a
      gripped liner; steady -0.17 mm). Joint speeds at most 0.55 of the Panda
      limits. The stock controller's tick-boundary target steps give one-step j5 torques up to 34.5 N m (right arm, glass lift)
      and hold j6/j7 at the 12 N m clip for 70-260 ms per run; these come from the stock tool (no within-tick interpolation),
      not the bench (the reference itself stays at or below 9.9 N m), and are reported as a harness note.
    - Re-checks of the polished module: shared `accept.py` PASS with the same check counts (18/18 references, rules 12/12, leak
      13/13, settings 35/36); the round-3 reviewer audit re-run on every reference, probe and rule test
      (probes/emul97/ref_audit.jsonl): 18/18 references succeed with no damage, reference guard need at most 9.0 N, j5-j7 at most
      9.88 N m with 0 ticks above 12, 0 joint-speed ticks over a limit, pads on liners at worst -0.77 mm (was -1.12 mm with the
      full squeeze), deepest contact overall -1.54 mm (glass in the grip, drag probe; unchanged). The probes, payoff and release
      grid do not depend on the liner grip force; the blind and uninformed payoff runs in SUMMARY were made with the round-3
      module (same liner choices and hidden values).
    - Measured pinch envelope under the stock tool (review r4 N1, lead decision #124; reviewer's stock-emulator lift test,
      direct mode, jaw jump, liners 1 and 5, seed 42, `jawgrid*.jsonl`): with the pad bottoms about 2 mm above the table (the
      reference height) a jaw gap of 40, 45, 50 or 60 mm lifts the liner (60 mm: about 5.9 N per finger); 35 mm fails (pads ride
      over the edge, 233-257 N spikes) and 30 mm fails. With the pads 3 mm or more above the table every gap fails (40 mm:
      189 N); at 1 mm, 35-40 mm lift. So the public text now says "a jaw gap of 40-60 mm, with the pads close to the table"
      (text only, 2026-10-30 integration; no contact change: a stiff pad-liner contact was not adopted, and the height limit is
      mostly geometric, pads that engage only the top 3 mm of a 6 mm edge also fail with a stiff contact). The agent can find
      the table height with a guarded touch.

## integration doc fixes (#184)
2026-10-30 (lead decision #184, docs only; no module or sim change).
- **Integration (#165/#181/#182) text-only change, new md5.** I1, one module edit carrying two text changes: #124 SCENE_TXT "(a jaw
  gap of about 40 mm)" -> "(a jaw gap of 40-60 mm, with the pads close to the table)" (item 17) and #165 ROBOT_TXT "80 N m (joints
  1-5) and 12 N m (joints 6-7)" -> "80 N m on joints 1-4, 12 N m on joints 5-7". Module md5 `23d06948` -> `7f4e16db` (WORKLOG
  2026-10-30 integration). Shared accept.py: PASS, no waivers; 1 18/18, 3 18/18, 5 12/12, 6 13/13 (no hits), 7 5/5, 8 35/36; 2/4/9
  REPORT, separations identical. No physics, contact, plan, rule or target change, so the #97 replay and audits apply. No
  #181/#182 change.
