## B4_bars: independent review, round 2 (2026-10-29, reviewer `review-B4_bars`)

## Verdict (round 2): ACCEPT WITH NOTES

The round-1 must-fix (F1, no heading rule) is fixed correctly, and I could not fool the new rule. The lead-directed widening of
the COM range to 10-90 mm makes the magnitude matter. Every builder number reproduces on my own runs. The references solve every
target at the new COM extremes, which lie beyond the dev seeds. The payoff is still time only against an agent that weighs the bars
in-task, as the card intends. No must-fix items.

Evidence is in `/tmp/tb_review_B4_bars/r2/`:
- `acc/`: my accept.py run;
- `audit.jsonl`: 27 references, from `../audit.py`;
- `heading.jsonl` and `extreme.jsonl`: from `r2_tests.py`;
- `payoff3.jsonl`: from `payoff3.py`.

## History
- Round 1 (2026-10-28): **REJECT (must fix)**.
  - Must-fix F1: `complete()` did not constrain yaw, so a bar turned 90 deg to lie along a 100 mm ridge passed edge_A, edge_B and
    stack_CBA 6/6 each with only the sign of the offset.
  - Notes: F2, only the sign mattered (sign + 60 mm succeeded 23/24); F3, payoff is time only (in-task ID +1-2 s per bar); F4,
    needs a real wrist F/T sensor (Panda-class noise is marginal); F5-F8, physics, limits, level check and leaks OK.
  - Lead decisions L1-L4. The lead then directed L1 (widen the range).

## Round-2 findings, ranked

### R1 (OK). accept.py reproduces
- Verdict **PASS**, 0 WAIVED. Checks 1, 3, 5, 6, 7 and 8 PASS (check 8 is 47/48, with one REPORT item); checks 2, 4 and 9 REPORT.
- References 27/27, with identical replays.
- Separations: com_A/B/C 45.7 / 65.4 / 64.0x; mass 2549-4450x.
- Minimum speed 7852 ticks/s.
- Dev-seed |c| spans 16.9-88.5 mm after the change.

### R2 (OK). The heading rule cannot be fooled (`benches/B4_bars.py`, `YAW_TOL` and the check in `complete()`)
All runs are edge_A s42-S (24 mm ridge), placed by the privileged reference with a deliberate yaw and COM error (`heading.jsonl`):

| yaw | COM error | result |
|---|---|---|
| +9 deg | 0 | ok |
| -9 deg | 0 | ok |
| +11 deg | 0 | **fail** (rests level and still at z 0.985, rejected by the rule) |
| 9 deg | -11 mm | ok |
| 9 deg | +11 mm | fail (topples) |
| 9 deg | +13 mm | fail (topples) |
| 0 deg | +13 mm | fail (topples), the same as at 9 deg |

- A bar diagonal at 9 deg gains no balance tolerance. Along the bar axis, the contact patch grows only to 24/cos 9 deg = 24.3 mm.
- "One end lifted" is blocked by the level check (3.4 deg) together with the contact rule: only the support or the adjacent bars
  may be touched, and a bar on a flat top cannot rest tilted without a prop.
- The builder's own negative test agrees: turned bars 0/28, controls 14/14.
- The text states the rule ("across the support ... along the X axis (within 10 deg, either end first)").

### R3 (OK). Feasibility and limits at the new COM extremes, beyond the dev seeds (`extreme.jsonl`, seed 42 layout, all 5 tasks)

| set | masses (kg) | COMs (mm) | success | guard need | 2 ms peak | j5-j7 max | pad pen |
|---|---|---|---|---|---|---|---|
| X1 | all 0.85 | all +90 | 5/5 | ≤ 23.3 N | 37.3 N | **11.5** N·m | -1.39 mm |
| X2 | all 0.55 | all -90 | 5/5 | ≤ 15.6 N | 20.9 N | 10.7 N·m | -1.26 mm |
| X3 | .85/.55/.85 | +90/-90/+90 (max stack offsets) | 5/5 | ≤ 19.6 N | 29.8 N | 10.6 N·m | -1.39 mm |
| X4 | .55/.85/.70 | +10/-10/+10 | 5/5 | ≤ 18.2 N | 33.8 N | 10.8 N·m | -1.21 mm |

- All 20 runs succeed, with final yaw at most 1.2 deg.
- The only robot contacts are the finger pads on the held bar.
- The worst case is stack_CBA with three 0.85 kg / +90 mm bars. Its j5-j7 reaches 11.5 N·m, under 12 but with only 0.5 N·m of
  headroom. Its single-tick peak of 37.3 N needs only 20.2 N under the exact guard emulation.

### R4 (OK). Exact guard, j5-j7 and penetration over the 27 dev references (`audit.jsonl`, same method as round 1)
- **Guard:** untared force norm, one sample per 25 ticks, trip on 2 consecutive samples above the limit, worst of all 25 phase
  offsets (`agentic_tasks.py:398-421`):
  - the level needed is at most **18.2 N** on the left arm and 6.3 N on the right, so the default 30 N is enough;
  - the 2 ms peak is 33.2 N (stack_ABC s43-S), a single tick, as reported.
- **j5-j7** (actuator force plus the `qfrc_applied` gravity compensation, every tick): left at most 10.88 / 8.2 / 8.75 N·m, right at
  most 0.53 / 2.13 / 0.15 N·m. **0 ticks above 12.**
- **Penetration from contacts:** only finger pads on the grasped bar, worst **-1.31 mm**. No other robot-object contact.
- **Validated `mj_geomDistance`** (including collision-disabled visuals): pads at -1.30 mm or more, robot base on the table
  -0.03 mm, nothing else.
- **Workspace:**
  - goals within X -0.20..0.33, Y -0.30..0.22, Z 0.835..1.174 m;
  - the HOME start (Z about 1.26 m) is now stated in `ROBOT_TXT`.

### R5 (OK, payoff). The builder's numbers hold, and no better uninformed strategy exists (`payoff3.jsonl`, 3 seeds, S and T, deployment only)

| strategy | edge_A | edge_B | stack_ABC | stack_CBA | total |
|---|---|---|---|---|---|
| centre guess (no knowledge) | 0/6 | 1/6 (\|c\| = 17 mm) | 0/6 | 0/6 | 1/24 |
| sign + fixed 50 mm | 1/6 | 2/6 | 0/6 | 3/6 | **6/24** (round 1, sign + 60 mm: 23/24) |
| in-task wrist ID, agent-like sampling (8 x 0.05 s), noise 0.5 N / 0.03 N·m | 6/6 | 6/6 | 6/6 | 6/6 | 24/24 |

- In-task ID error is 0.9-5.2 mm. It costs +594..678 ticks (1.2-1.4 s) per single-bar target against the informed reference
  (builder: +1.2-1.4 s), and +1886..2059 ticks (3.8-4.1 s) per stack.
- **Better uninformed strategies?**
  - A sign-only tilt test now fails 18/24.
  - Laying the bar along the ridge is blocked (R2).
  - Trial and error costs a re-pick (more than 5 s) after each topple.
  - Feeling the bar on the support while still holding it gives the same information as lifting it, at the same cost.
  So in-task wrist ID is still the best uninformed strategy. The sign and the magnitude both matter now.
- Stale S notes on T: builder 0/12, guaranteed by the at-least-25 mm resampling guard (`transfer_parameters`).

### R6 (note, realism). The 1.0 mm wall is plausible but not a standard stock size
- The change is needed for |c| = 90 mm on a 0.55 kg bar. With the old 1.5 mm wall (0.27 kg shell), the slug's end would leave the
  tube for a light bar: |c| at most about 75 mm at 0.55 kg, and 90 mm needs M ≥ about 0.73 kg.
- The new physics is consistent:
  - shell 196 mm² x 2.7 g/cm³ = 0.53 kg/m;
  - slug 0.36-0.66 kg, 20-37 mm long in a 48 mm bore;
  - slug end at most 148 mm < 155 mm;
  - inertia uses the 48 mm bore.
- However, common metric catalogues (EN 755 6060/6063) list 50 x 50 square tube from about 1.5 mm wall. US stock starts at
  2 x 2 x 1/16 in (1.59 mm). A 1.0 mm wall at 50 mm (b/t = 50) is offered only by some thin-wall or decorative suppliers.
- Web access was blocked for this review (the proxy refused), so I could not confirm a specific listing. ASSETS.md marks the
  values as "computed" but does not say that 1.0 mm is an uncommon gauge.
- Suggested one-line fix, not blocking: mark the 1.0 mm wall as "estimated / uncommon thin-wall stock" in ASSETS. A realistic
  alternative is a 1.5 mm wall with a mass-dependent COM cap, or a mass range of 0.73-0.85 kg for the 90 mm draws.
- A 15-30 N finger squeeze on a 1 mm wall is harmless; the sim's rigid box is fine.

### R7 (note). Housekeeping
- DEVIATIONS flags the card-intent change (COM range 10-90 mm, lead L1) and the F1 fix at the top, with evidence.
- SUMMARY, WORKLOG and ASSETS are updated. The rejected module is kept as `scratch/B4_bars_v1_rejected.py`. The diff against it
  touches only these:
  - `SHELL_M`;
  - `COM_RANGE`;
  - `YAW_TOL` and the heading check;
  - the bore in the inertia;
  - the text.
- Minor: DEVIATIONS item 12 still says "resampled from the card ranges". It now means the widened range.

## Must-fix
- None.

## Lead decisions (carried over; nothing new)
- **L2. Payoff is time only, about 1.2-1.4 s per bar** against an agent that weighs in-task. This matches the card ("calibration
  bench"). There is no topple consequence (card: "翻了大多可以撿起來再放").
- **L3. T-new bars look identical to the S bars.** Whether the harness tells the agent they are new is a harness-wide decision.
- **L4. Sensor assumption.** The cells need a real wrist 6-axis F/T sensor. Under Panda-class estimated-wrench noise, about 2 s of
  averaging is needed. Team-wide.
- **New, minor: R6.** Accept the 1.0 mm thin-wall tube as an estimated product, or prefer a standard 1.5 mm wall with a
  mass-dependent COM cap.
