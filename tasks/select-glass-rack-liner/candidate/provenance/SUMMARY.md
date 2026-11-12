# C1_liners: summary (final, written by the team lead from the integration drafts, 2026-10-30)

| bench | accept.py result | reference success per seed (task A and each target) | force-separation flags | deviations (count + one-line gist) | open problems |
|---|---|---|---|---|---|
| C1_liners | PASS, no waivers (md5 7f4e16db): checks 1,3,5,6,7,8 PASS (compile 18/18, ref 18/18, rules 12/12, leak 13/13, rand 5/5, settings 35/36); 2,4,9 REPORT; min 3833 ticks/s | seeds 42/43/44: A_dry_cup 3/3; C1-T1 3/3; C1-T2 3/3; C1-T3 3/3; C1-T4 S 3/3, T 3/3 (18/18, replay identical, no damage) | none: mu_dry 1.9e8x, mu_wet 2.1e8x (zero spread). Agent drag readings are correct at <= 2 mm/tick and unreliable at 4-8 mm/tick | 17 items, 6 flagged. Gist: six liners with n_hold {2,3}; stratified wet sampling; retuned glass-liner contact; 6 mm liners; squatter tumbler; 1 ms timestep (50 substeps per tick); item 17 light pinch, text "jaw gap 40-60 mm, pads close to the table" (#124) | the agent must drag slowly (~15 single-tick calls per liner, guard ~41-45 N at 2 mm/tick, not stated in the text). The sim release slide is 20-40 mm vs ~3-5 mm real (#86). `cup_fell` misses drops from height. Values are estimated |

Notes
- **#97 standard tool:** 18/18 in both direct and path modes after the light-pinch fix; the round-3 reference failed 18/18 on the jaw jump.
  - #88: 18/18.
  - #145: 0 N.
  - #158: 0 violations; 360 start-descent goals from HOME z 1.256, which is noted only in a module comment.
- **Guard:** references <= 9.0 N; stock left 9.3 N, right 17.1 N (direct) or 26.1 N (path). The 30 N default holds.
- **Payoff:** strong, not a skill bench. A blind pick succeeds at the n_hold/6 rate; a blind T3 pair 1/15. The best uninformed strategy (drag-test until mu > 0.41) succeeds 12/12 at 2.0-3.5x the active time.
- **Harness:** 50 substeps per tick (#64); sensitive to jaw speed (#128). Stiff pad contact is held back as option (b).
- **Pass-through:** references >= -1.12 mm; probes >= -1.54 mm. j5-j7 <= 9.9 N m.
