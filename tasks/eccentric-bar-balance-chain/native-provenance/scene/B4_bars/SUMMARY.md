# B4_bars: summary (final, written by the team lead from the integration drafts, 2026-10-30)

| bench | accept.py result | reference success per seed (task A and each target) | force-separation flags | deviations (count + one-line gist) | open problems |
|---|---|---|---|---|---|
| B4_bars | PASS, no waivers (md5 1c92ac4a): checks 1,3,5,6,7,8 PASS (compile 18/18, ref 27/27, rules 0/0 by design, leak 13/13, rand 6/6, settings 47/48); 2,4,9 REPORT; min 10992 ticks/s | seeds 42/43/44: A_stack_pedestal 3/3; edge_A S 3/3, T 3/3; edge_B S 3/3, T 3/3; stack_ABC S 3/3, T 3/3; stack_CBA S 3/3, T 3/3 (27/27, replay identical, no damage) | none: com_A 45.73x, com_B 65.35x, com_C 64.02x, mass_A/B/C 2549-4450x | flagged header has a #184 bullet: COM cells off all deployment targets. 18 numbered items; 1 card-intent change (COM range 10-90 mm with random sign, #59). Other items: heading rule; 1.0 mm tube wall (estimated, #74); mass cells listed nowhere | CALIBRATION/SKILL BENCH: no deployment target lists a cell. In-task wrist identification succeeds 27/27 at 3.9-8.6 % of full task time including the 20 s hold (#184/#187). The COM cells assume a real F/T sensor (#60). T-new bars look identical to S. j5-j7 headroom is 0.5 N m at COM extremes |

Notes
- **Cells:** A {com_A, com_B, com_C}; edge_A/edge_B/stack_ABC/stack_CBA have cells=[].
- **Payoff record:** informed 27/27; centre guess 1/27; sign + 50 mm 6/24; stale S notes on T 0/12. Wrist identification 27/27, costing 3.9-4.7 / 4.3-4.6 / 8.1-8.6 / 7.9-8.1 % of uninformed full time. The reviewer's noisy-sampling run is 24/24.
- **Replays:** #97 standard 27/27, joint speed <= 0.78x; #88 27/27 (j5 capped from 34.0 to 12.0 N m); #145 0 N; #158 0 violations.
- **Guard:** stock worst-of-25 up to 31.2 N (bar-on-bar chatter on stacks); #88 29.4 N; reference <= 18.2 N. The 30 N default and 60 N max are stated.
- **Harness:** standard tool; jaw-jump pad penetration -2.4 to -2.5 mm (#128); sensor-noise assumption (#60).
