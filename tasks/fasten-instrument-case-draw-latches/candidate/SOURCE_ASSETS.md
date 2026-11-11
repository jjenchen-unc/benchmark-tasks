## S2_draw_latch

All geometry is built from MuJoCo primitives in `benches/S2_draw_latch.py`; there are no mesh or texture files
(`benches/S2_draw_latch_assets/` is empty). Robot meshes come from the repo's robosuite Panda (MIT license). Product pages could
not be opened from this host (the web proxy refused PDF/page fetches; search titles only), so every product value below is from
memory of the catalogue and is marked ESTIMATED unless stated.

| object | real product it is based on | values taken from the product | estimated / chosen values |
|---|---|---|---|
| toggle draw latch (lever + hook) | Southco 97-series draw latch (e.g. 97-50 medium, over-center lever, spring-loaded adjustable hook, optional secondary catch, kick-out lever); search hits: Southco "97 Draw Latch" catalogue PDF (biotek.hu mirror), RS Components Southco draw latch datasheets docs.rs-online.com/2951/0900766b80028509.pdf | over-center lever with tension built into a spring-loaded hook; secondary catch option; kick-out of the lever | lever 70 mm to the lip end, lip drooping 35 deg, pivot 24 mm below the lid seam, hook pivot 18 mm from the lever pivot at 25 deg over-center (ESTIMATED from typical medium draw latch proportions); hook preload stretch 4 mm; breakaway force 6-28 N at the lip end (case A 6-10, case B 20-28, new case 6-28) (chosen for the Panda's 60 N guard; real 97-series latches clamp several hundred N, ours pull 60-360 N) |
| keeper with three bars | adjustable / multi-position keeper (strike) used with spring draw latches | three hook positions | bars 5 mm diameter, 6 mm pitch, 6.5 mm proud (ESTIMATED) |
| secondary catch tab | Southco draw latch secondary catch (slide catch that prevents vibration opening) | slide-to-release catch beside the lever | 12 x 12 x 14 mm tab, 7 mm travel, 0.8 N slide friction (ESTIMATED, enlarged for a fingertip as the card asks) |
| instrument case | generic light instrument / tool case with two front draw latches (e.g. small ABS equipment cases) | two draw latches on the front, rear hinge | 240 x 160 x 220 mm, body 1.4 kg + lid 0.44 kg + hardware about 0.25 kg = 2.1 kg, rubber feet mu 0.7 (ESTIMATED; card: about 2 kg) |
| instrument | tall cylindrical bench instrument (e.g. a small sensor head or lab probe in its case) | none | 40 mm diameter x 160 mm, 0.30 kg (ESTIMATED; chosen to tip easily as the card asks) |
| plinth | generic wooden riser / shelf board under the cases | none | 600 x 200 x 60 mm fixed box, mu 0.7 (chosen so the horizontal hand's wrist clears the table when reaching under a lever lip) |
| gripper | Franka Emika hand (stock, repo model) | sim per-finger force 40 N, opening 0-90 mm (portable_demo/env.py) | none |

Licenses: no third-party assets; robosuite Panda meshes are MIT (repo snapshot).
