## C1_liners: independent review, round 4 (narrow re-review of the #97 polish MODULE change, 2026-10-29)

**Verdict: ACCEPT WITH NOTES.** The polish change is exactly what DEVIATIONS 17 says it is: a gentler liner pinch in the reference
(LINER_JAW = 0.020, a 40 mm jaw gap) plus one sentence in SCENE_TXT. Physics is unchanged. I reproduced the stock-controller
replay, accept.py PASS (identical numbers), the rule tests, the wet probe and the payoff. The new sentence is skill guidance. It
leaks no hidden value and does not change what exploration buys. One note: the working pinch envelope under the stock tool is
narrower than "about 40 mm" suggests (N1). A targeted contact fix exists if the lead wants to drop the sentence (Lead decisions).

Module md5 `23d0694811ca131a2314bd84b804f68f` (unchanged during the review). Scratch and logs are in `/tmp/tb_review_C1_liners/r4/`:
`accept.log` and `acc/`, `emul_direct.jsonl` and `emul_path.jsonl`, `jawgrid.py` + `jawgrid_direct.jsonl`, `jawgrid2.py` +
`jawgrid2.jsonl`, `blind_T2_44.jsonl`, `uninf_T1_43.jsonl`, `audit44.jsonl`, and `review_r3_backup.md` (the round-3 review text).

### Verified
| item | result |
|---|---|
| (1) diff vs `scratch/C1_liners_r3_accepted.py` (md5 da3a8733...) | Only these changes: the SCENE_TXT sentence (`:290-291`); `LINER_JAW = .020` + comment (`:411-414`); `0.` -> `LINER_JAW` in `place_liner` (`:429-442`), `dip_and_return` (`:515-522`) and `_rt_dip` (`:599-605`). Every other line is byte-identical: contact constants, pairs, rules, `place_cup`, `drag`, `_rt_fall`, sampling, success. The glass grasp (jaw 0) is untouched |
| (2a) emulator vs stock DualTask | `probes/emul97/std_emul_c1.py` matches `repo_snapshot/backend/agentic_tasks.py:439-466` and `dual_arm_demo/env.py:152-205` line by line: the commanded target moves at most 8 mm / 0.08 rad from the previous commanded target per tick; IK runs once per tick, seeded from the measured joints, 12 iterations, rotation weight 0.5, damping 1e-4, step clip 0.08, joint-range margin 0.01, no null-space term; total torque = clip(550/45 PD + qfrc_bias, [80]*5+[12]*2), and the bias is removed again because the bench's `step()` re-adds it; jaw targets jump when a move starts (stock `set_goal` / `grip_targets`). The only differences are documented: 50 x 1 ms substeps instead of 25 x 2 ms (harness requirement #64, DEVIATIONS 6), and no guard stops or held_object_change events are emulated (the replay reports the guard each phase would need) |
| (2b) stock replay, my runs | 6 jobs, all succeed, no damage, every number identical to the builder's jsonl. Direct mode: C1-T3:43:S, C1-T2:44:S (dip), C1-T4:43:T. Path mode: C1-T3:44:S, C1-T2:42:S (dip), A_dry_cup:43:S. Guard need, worst of 50 phases: left at most 9.3 N; right at most 17.1 N (direct) and **26.1 N** (path, T3 glass placement), under the 30 N default. Over 80 N: at most 2 ms (direct-mode jaw-impact spikes, 88-141 N). qvel at most 0.551 of the limits. Pads dent the liner by at most -3.3 mm for a few ms at the jaw jump |
| (2c) stock-tool artifacts (not bench) | Same as reported: one-step j5 torque up to 34.5 N m (right arm, `place_cup`, 52-64 ticks above 12) and 29-31 ticks in `place_liner`; j6/j7 on the 12 N m clip for 88-242 ticks per run. These come from the tool's tick-boundary target steps (#118 harness note). On the smooth reference, j5-j7 stay at or below 9.88 N m with 0 ticks over 12 |
| (4) shared `accept.py` (`r4/acc`) | **PASS**, no waivers. Compared item by item, every status and number matches the builder's `acceptance/C1_liners/report.json` except the timing fields (speed min 4,448 ticks/s). References 18/18, rules 12/12, leak 13/13 (the new sentence has no hits), randomization 5/5, settings 35/36 |
| (5) rule tests and wet probe | Wet rule test: positive fires and negative does not, on every seed (accept). My audit of seed 44 (`audit44.jsonl`, the unchanged round-3 audit script): rt:wet:1/0 guard L 7.5/7.3 N, j5-j7 at most 9.88 N m, 0 ticks over 12, held_object_change 0/50 phases, pads on the liner at worst -0.77 mm. drag_wet probe: guard R 11.0 N, 0/50 held_object_change. C1-T2 reference: L 7.5 N, R 8.9 N. All identical to the builder's `ref_audit.jsonl`. The wet-probe statistic moved slightly with the gentler lay-back (0.2713/0.4752 -> 0.2707/0.4761) and still separates cleanly. cup_fell rule test: unchanged code, PASS |
| (3a) payoff, blind | `scratch/payoff.py 44 C1-T2 S` on the new module: 1/6 liners succeed (liner 1, mu_wet 0.4605); the other five slide off (cup_fell). Identical to the round-3 log `payoff7_C1-T2_S_44.log` |
| (3b) payoff, best uninformed | `scratch/uninformed.py 43 C1-T1 S`: drag estimates 0.191 / 0.619 (same as round 3), picks liner 2, succeeds; first success at tick 60,179 (round 3: 60,192). The time cost of exploration is unchanged |

### (3) Judgement on the new SCENE_TXT sentence
"The liners are thin and soft: pinch one across its 70 mm width with a light grip (a jaw gap of about 40 mm); a hard, fast squeeze
can make the fingers ride over its 6 mm edge."
- **No hidden-value leak.** The hidden cells are the 12 cup-to-liner friction values. The sentence is the same for every seed,
  target and liner. Its numbers (70, 40, 6 mm) are public geometry and a grip command, not related to mu, and accept's leak check
  passes. "soft" is not a hint word and says nothing about friction: every liner has the same compliance.
- **Does not make a target trivial.** The difficulty is still choosing a liner that holds a glass at 20 deg. A blind pick still
  succeeds at the n_hold/6 rate (1/6 in the worst case checked), and the reference and uninformed outcomes are unchanged.
- **Payoff.** What exploration buys is unchanged: drag tests identify the holders exactly as before. The sentence only removes a
  manipulation trap caused by the sim and the tool (an instantaneous jaw jump onto a compliant edge, #101/#118). An agent no
  longer spends its budget discovering it, so slightly more of the episode measures the intended skill. I count that as a plus.
- **Realism.** A light pinch is the right way to pick up a thin rubber mat. The claim that a hard, fast squeeze rides over the edge
  is true in this sim and plausible in reality. The sentence is guidance, not a mechanism hint.

### Notes (non-blocking)
**N1: the pinch envelope under the stock tool is narrower than "about 40 mm" suggests.** I ran a stock-emulator lift test (direct
mode, jaw jump, liners 1 and 5, seed 42; `jawgrid*.jsonl`). The pad-bottom height is given relative to the reference's 2 mm
above the table.

| jaw gap | pads 1 mm | pads 2 mm (ref) | pads 3 mm | pads 4 mm |
|---|---|---|---|---|
| 60 mm | | lifts (5.9 N/finger) | | |
| 50 mm | | lifts | | fails |
| 45 mm | | lifts | | fails |
| 40 mm | lifts | lifts | **fails** (189 N) | fails |
| 35 mm | lifts | **fails** (233-257 N) | | fails |
| 30 mm | | fails | | fails |

So with the stock tool:
- A gap of 40-60 mm works when the pads are no more than about 2 mm above the table.
- A gap of 35 mm, which still counts as "about 40", fails at the reference height.
- Pads 3 mm or more above the table fail at every gap.

The height limit is mostly geometric: pads that engage only the top 3 mm of a 6 mm edge also fail with a stiff pad contact (see
Lead decisions), so a real gripper would likely struggle too. The force limit is the compliant-contact artifact. Suggested
low-cost text tweak (lead's call): "a jaw gap of 40-60 mm" instead of "about 40 mm". Also record the envelope in DEVIATIONS 17.
Agents can find the table height with a guarded touch, so the height limit does not need public text.

**N2: carried over from round 3** (N1 release slide, N2 2 mm/tick read range, N3 cup_fell edge cases). These are now written
into DEVIATIONS 3 and SUMMARY, as the WORKLOG says. Nothing in this change affects them.

### Lead decisions
- **Keep the sentence, or fix the contact and drop it.**
  - (a) Default: accept as is, with the optional N1 wording tweak.
  - (b) Add explicit `<pair>`s between the finger pads and the liners only, with stiff contact (pad values solref .004 1,
    solimp .99 .999 .0001, friction 1.5). This removes the force sensitivity: in my test a full squeeze (0 mm, 40 N) and a 30 mm
    gap both lift the liner, with peak 16-19 N (`jawgrid2.jsonl`). It leaves the cup-liner pairs and the liner-rack and
    liner-table contacts alone, so the validated release-slide and drag-readability behaviour does not change. That answers the
    #116 objection to "stiffen the liner contact".
  - Cost of (b): a MODULE change. It needs a re-run of the reference audit and accept, plus a narrow re-review. The pads-too-high
    failure stays.
  - I lean (a) now and (b) only if the harness does not adopt the #101/#118 finger-speed limit.

## History
- Round 3 (2026-10-29): **ACCEPT WITH NOTES.**
  - Both round-2 must-fixes were fixed and verified, and accept PASSED.
  - Notes: N1 sim release slide of 20-40 mm (vs 3-5 mm real), failure envelope at mu 0.46 for down-slope placements; N2 2 mm/tick
    read range 0.42-0.63; N3 cup_fell misses drops from height.
  - Lead decision: accept the release slide (option 1) or raise the holder floor.
  - Full text: `/tmp/tb_review_C1_liners/r4/review_r3_backup.md`.
- Round 2 (2026-10-29): **REJECT (must fix).**
  - R2-M1: under the round-2 compliant contact a glass stood at the liner centre slid off holders with mu 0.46-0.5, hidden by a
    tuned 25 mm up-slope reference placement.
  - R2-M2: the cup_fell negative rule test jammed the glass into the table (126-263 N).
  - Both fixed in round 3.
- Round 1 (2026-10-29): **REJECT (must fix).**
  - M1: `cup_fell` fired on a glass returned from the rack to the table.
  - M2: the docs overstated how readable mu is at agent sampling.
  - Both fixed in round 2.
