## B1_wrench

Revision 2 (2026-10-29), after the independent review REJECT (`REVIEW.md`). Items marked **[rev2]** are new or changed.

### Flagged at the top: changes to card intent or the guideline
1. **[rev2] Four listed cells no longer count for their targets; they stay as exploration facts.**
   - Dropped: `breakaway_yellow` and `breakaway_cyan` (W1, W2), `adjustable_slack` (W3), `al_gall_torque` (W4). They are still
     sampled, varied in T, probed in check 4 and discoverable in E1/E2.
   - Why (reviewer M2 evidence, re-checked):
     - Every target needs >= 0.30 N m. A fitting wrench never strips and a loose one always strips first (strip <= 0.206 N m).
       So only the fit decides, never the breakaway value; the reference never reads it and works for every S and T value.
     - The adjustable wrench rounds the nut whatever its slack (0.45-0.65 mm).
     - A careful uninformed agent (0.065 N m thread-torque cap) seats matched screws and stops on mismatched ones below the gall
       threshold, so the threshold value is never needed.
   - Redesigns considered (about 1 h, lead's 1.5 h budget):
     - "the part turns with a seized nut unless held": contradicts the card's captive nut ("no counter-hold needed");
     - "clamp torque window just under tau_dmg": the best uninformed strategy (tighten to just above the public clamp minimum) succeeds
       on every sampled tau_dmg unless the window is made artificially narrow;
     - "slack range into the safe zone": the matched fixed wrench is always available, so the adjustable never pays.
     None gives an honest payoff, so the cells are dropped from the targets instead.
   - [2026-10-30, lead #152/#184] `strip_torque` is now listed only for task A (it was also on W1/W2/W3 as the M cell of the
     rounding rule). The rule matters (a loose wrench rounds the nut, damage), but its value decides nothing on any target: every
     target needs >= 0.30 N m, a fitting wrench never strips and a loose one always strips first (strip <= 0.206 N m), so the
     informed and the best uninformed plans are the same (gain 0, no success or damage gap). B3 dropped its equivalent cell the same
     way. The rounding rule itself stays in the W1-W3 `rules`; the cell stays sampled, varied in T-new, probed (check 4) and
     discoverable in exploration.
2. **Aluminium offcut added to the exploration bench** (one M8x1.25 tapped hole). ACCEPTANCE item 8 requires each hidden cell's
   object in the exploration scene; `al_gall_torque` had none. [rev2] The text no longer says it is "the same alloy as the covers"
   (review N5: a hint at a later target).
3. **Adjustable wrench: the worm is not operable; the jaw comes pre-set to 13 mm + hidden slack (0.45-0.65 mm).** Rolling a 10 mm
   worm with a parallel-jaw hand is dexterous. Every fixture is open to every wrench, which satisfies the DECISIONS ruling.
4. **The two seized practice nuts are a mixed pair** (13 mm and 1/2 in, shuffled), like the seats. W2 uses them in S, so the W2
   flange is always mixed; the text does not say so. Lead decision (rev2): keep.
5. **Gall = stripped aluminium thread** (smoke: jam). The screw then spins without advancing and can never seat (`thread_galled`).
   A matched screw strips only above 3 tau_dmg (>= 0.36 N m); the reference seats at about 0.085 N m.
6. **[rev2/rev3] W4 "cover clamped" is enforced** (review N6, round-2 F1). Success needs, for both holes, a matched screw whose
   thread advance leaves the knob within 0.21 mm of the cover, i.e. >= 0.04 N m of seat preload on the thread (knob torque to get
   there >= 0.05 N m with the 0.01-0.03 N m running torque; stated in the public text), AND the cover in its pocket (within 1 mm in
   xy, 0.3 mm in z, tilt < 2 deg). The clamp is modelled through thread friction: at the clamped gap (0.17-0.21 mm) the knob does not
   touch the cover; the seat stiffness rule stands in for the elastic clamp. Rule test `cover_clamp` (target W4): positive = the
   reference (fires, success); negative = the same procedure with the cover lying on the table (`test_no_cover`, rule-test-only
   parameter): the screws reach the clamp gap in mid-air but the event never fires (verified seed 42; all seeds in check 5).

### Design and build changes (what, why, evidence)
- **Fresh implementation** (the old `b1*.py` depends on `mechanism_demo` internals and the pre-10-28 targets). Mechanisms follow smoke
  b13 (nut turn hinge + head-slip hinge held by an equality, stripping rule) and smoke tp (follower hinge + slide, pitch equality,
  weld on engagement, bind/gall rule).
- **Nut model**: thread friction written each tick. Seat nuts `0.02 + max(0, K_seat theta + 0.05)` (K_seat = 0.35 N m / 30 deg,
  hand-snug start); seized nuts `tau_b` until backed off 2 deg, then 0.03 N m. Nut thread lift omitted (60 deg = 0.2 mm), as b13.
- **Stripping rule** as card/b13: clearance c in [0.15 mm, 0.155 AF) and contact torque > tau_s0 (1 - c / 0.155 AF) for 5 ticks.
- **[rev2] Forced-on undersized wrench rounds the nut** (review M3). An undersized wrench (c < 0) whose centre is less than 9 mm from
  the nut axis and loads the nut with > 0.05 N m for 5 ticks crushes the corners (`nut_rounded`). Evidence:
  - `scratch/jam3.py`: the 1/2 in wrench pushed quasi-statically onto the 13 mm nut (W1 seed 44) stays blocked at 12 mm offset up to
    a 67 N two-sample hand force, above the 60 N guard; only beyond that does it jam, and then it rounds the nut.
  - `scratch/jam2.py`: driven in at speed (~45 mm/s run-up), it can wedge on at low force; turning it then rounds the nut.
  - Known: in one misuse run (reviewer's payoff2.py, 42 T, pushes above 150 N) the jammed wrench's free joint raised a MuJoCo
    QACC warning at t = 57 s, after the nut had rounded. Not reachable under the 60 N guard.
  - Stiffer contact could not be made: MuJoCo clamps solref timeconst to >= 2 dt (0.004 s), and direct-stiffness solref (-1e7/-1e8)
    made the wrench explode (36 kN). Default contact kept; the rule supplies the honest consequence of forcing.
- **W1 "latch"** = swing arm clamped by its nut (lock released once backed off 60 deg). "latch"/"spring" are accept.py hint words, so
  the texts say "swing arm" and "clamp". **W2** cover welded until both nuts are backed off 60 deg, slides out of open slots onto a
  shelf. **W3** clamp jaw linked to the nut; 90 deg free run, then task A's seat stiffness.
- **Knob screws**: square 20 mm x 30 mm knob (round knob squirts out of a parallel jaw; 14 mm slipped below the gall threshold).
  Engagement: tip 1 mm into the thread, < 0.7 mm off axis, < 3 deg tilt, 15 deg clockwise turn. Crest ring stops an unturned screw.
  Mismatch binds after 0.7-0.9 turns. 3 turns engagement to seat (real ~10; shortened). Seat stiffness 1 N m/mm inside 0.25 mm.
  [rev2] The seat gap is measured by thread advance (follower turn x pitch), not by the knob height, because the soft pitch coupling
  gave a 0.16 mm rebound when the hand stopped pressing.
- **Hidden values in mN m and um, sizes as labels**: the wrench stamps and the public tolerances are public by design; in N m the
  2-significant-figure leak check gave random collisions. Nothing physical changes; no leak waiver.
- **[rev2] Reference controller limits** (review M1): total joint torque including gravity compensation capped at 80 N m (j1-j4) and
  12 N m (j5-j7); joint-target speed capped at 2.0 rad/s (j1-j4) and 2.4 rad/s (j5-j7). Every reference picks its wrench with the
  wrist-continuous grasp-yaw choice (hand yaw psi + j pi + 2 k pi keeping joint 7 farthest from its limits at the rack and over the
  whole turning range). Before, task A drove j7 into its limit, flipped the elbow at 7.6 rad/s and hit 79 N m on j5. Audit
  (`scratch/audit.py`, all 27 references): j5-j7 max 3.5 N m, 0 ticks above 12 N m; joint speeds max 1.56 rad/s (j1-j4), 2.4 rad/s (j7).
- **[rev2] W2**: each nut gets its own wrench pick; fl2 is loosened in 45 deg strokes starting at wrench yaw -140 deg (handle away from
  the cover); engagement is checked (centre within 2 mm, height within 1.5 mm) and redone if the wrench is not seated. Before, a
  second fl2 stroke started outside the wrist range, wedged a toe flare on a corner and needed a 35.9 N guard (review M4).
- **[rev2] W2 washers are collidable** (radius 6 mm, inside the nut flats) (review N2).
- Controller details: joint PD 550/45 as the agent's tools; IK every 4 ticks, targets ramped in between; null-space posture bias;
  object-pose servo with a fixed unloaded grasp transform on a nut; screws turned at <= ~70 deg/s. All hand poses are top-down, yaw
  only (standard move tool).
- **Right arm parked** at (0.42, 0.30, 1.15) (at HOME it blocked the left arm near x = 0.12-0.18).
- **Probes** are the same scripted Cartesian plan for both contrast values (PD-tracked; targets from visible geometry; no decision on the
  probed value): seat-nut rocks +-5 deg (jaw-contact fraction), seized-nut rocks +-1 deg (median hand force), loosening ramps
  (25-tick-mean peak force), screw strokes (detrended thread-resisting wrist torque).
- **Layout**: the four exploration nuts stand in one row at x = 0.12 m; nut posts end 6 mm below the nut with a 12 mm boss.
- **[rev2] Workspace declared in the module**: `WORKSPACE` / `TOOLS` (left tip X -0.30..0.50, Y -0.50..0.65, Z 0.825..1.27 m); the
  public text states the box and the start height 1.26 m (review N1).
- **[rev2] Torque scales** (review N4): nuts s_tau ~ 0.01 of a real 13 mm nut; knob screws s_tau ~ 0.1 of a real hand-turned knob screw.

### Robot limits and force audit [rev2]
- `scratch/audit.py` (the reviewer's audit, plus joint speeds), all 27 references, every mj_step:
  - guard need, exact emulation (untared force, one sample per 25 mj_step, 2 consecutive samples, worst of 25 phases): A 22.9 N,
    W1 22.9 N, W2 23.4 N, W3 22.2 N, W4 8.2 N. The default 30 N guard suffices for every target; 80 N abort never reached.
  - untared peak: <= 24.7 N except W4 42.8 N for single ticks (knob grasp impact).
  - deepest robot-object contact -0.93 mm; tool vs collision-disabled solids >= 1.58 mm; robot vs them >= 3.2 mm.
- The stock Franka hand gives about 7 N per finger on a 12 mm handle (kp 1200 x 6 mm), enough for <= 0.5 N m at 0.1 m.

### #97 stock-controller replay (polish, 2026-10-29; DOC only, no module change)
- The reference plans were replayed through an emulation of the stock DualTask tool (`probes/emul97/std_emul_b1.py`): every
  0.05 s tick the commanded target moves at most 8 mm / 0.08 rad from the previous commanded target; stock IK (12 iterations,
  seeded from the measured joints, no null-space term); total torque clipped at [80]*5 + [12]*2 including gravity compensation;
  no joint-rate limiter (the reference's own 2.0/2.4 rad/s target limiter and its 12 N m j5 clip are removed); jaw commands jump.
  The plan logic is the reference's (its setpoints, servo on the measured wrench / bolt pose and privileged stop conditions);
  each move call returns only when the commanded target has arrived. Three goal modes: `direct` (open-loop moves get their end
  goal at once, so the target runs at the 8 mm/tick cap), `path` (the reference path fed once per tick), `path --tickservo`
  (servo loops observe and update only once per tick).
- Result, all 27 references in each mode (`direct.jsonl`, `path.jsonl`, `path_tickservo.jsonl`): **27/27 succeed, no damage,
  no engage retries**. Jaw commands used (0, 12, 16, 20, 45 mm per finger) are inside the declared `set_jaw` range.
- Guard need (untared, 2 consecutive 0.05 s samples, worst of 25 sampling offsets): at most 25.5 N (W3 disengage) in every
  mode, except W4 in `direct` mode, 48.0-48.5 N, while the 8 mm/tick lift pulls the gripped thumb bolt out of its rail pocket
  (13.9 N when the lift follows the reference's 0.6 s path, peak about 5.6 mm/tick). So W4 needs a guard of about 50 N if the
  bolt is lifted at the default 8 mm/tick; a slower lift keeps the default 30 N enough.
- Wrist force above 80 N: never longer than 2 ms (single-tick finger impacts up to 193 N when the open fingers touch the bolt
  rail at the W4 pick).
- Stock-tool artifacts (not in the reference, reported as harness notes): the tick-boundary target steps give one-step j5
  torques up to the 80 N m stock clip (the real Panda j5 limit is 12 N m) and keep j6/j7 on the 12 N m clip during wrist moves
  (about 2.7 s per W4 run, in the screw strokes); near the folded-elbow posture at the wrench rack (A return_wrench) and the
  W2 park, the stock IK's null-space drift drives j3/j5 to 1.3-1.56x the Panda speed limits. The reference itself stays at
  j5-j7 <= 3.5 N m, j1-j4 <= 1.56 rad/s and j5-j7 <= 2.4 rad/s.

### Waivers
- none

### B1_wrench: current module (lead #189 record, 2026-10-30)
- Current module md5 82d0b367aa4aa4ae97f56811622a0de9 (lead #184/#187 polish: strip_torque only on task A). Shared accept.py: PASS, no waivers (1 18/18, 3 27/27, 5 36/36, 6 13/13, 7 25/25, 8 47/48; 2/4/9 REPORT); acceptance/B1_wrench/report.json md5 e594b7f7e3784eb1d801c87e48badc7b.
