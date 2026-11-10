## B7_couplings

All geometry is built from MuJoCo primitives in `benches/B7_couplings.py` (no meshes, no textures);
`benches/B7_couplings_assets/` is empty. Robot meshes come from robosuite (MIT licence) through the repo's
`portable_demo` robot, unchanged. Web pages could not be opened from this host (proxy refused), so product values below
are from memory and catalogue conventions and are marked estimated.

| object | real product it is modelled on | values from the product / card | estimated values |
|---|---|---|---|
| coupling sleeve + shaft post (x3 practice A/B/C, x1 per deployment target C/D/E/F) | Spring-loaded pin-and-J-slot bayonet coupling, the mechanism of bayonet caps such as Minco bayonet fittings for spring-loaded sensor probes (Minco SSG01 "Bayonet fittings" sheet) and of MIL-DTL-26482 / MIL-DTL-5015 style bayonet connector couplings: two opposed radial pins on the fixed part, J-shaped track in the coupling ring, wave/coil spring pre-loading the ring into the blind pocket. Search hits: https://chart.rewindsensors.com/mod_pdf/minco_pdf/SSG01(B)_Bayonet-Fittings.pdf , https://www.jwwinco.com/en-us/products/3.2-Mounting-Positioning-Leveling-with-Retaining-Cables-Screws-Clamping-and-Supporting-El (not opened). | mechanism (push, turn, pull; pins on the shaft, track in the bore; plunger spring pushing the ring into a blind pocket); card values: plunger 320 N/m nominal, push 12 mm, turns 30/60 deg, pulls 12/24 mm, release paths A-F (F shortened; handedness sampled per practice coupling; see DEVIATIONS); payoff trial 2026-10-29: the first junction also has a dead-end branch (junction 120 deg) ending in a blind false-seat notch (12 mm, or 18 mm with a 20 deg blind ledge behind a 90 deg dead end) (a design choice for the payoff rule, not taken from a product; DEVIATIONS 21) | scaled up so a parallel gripper can hold it: sleeve OD 58 mm, length 80 mm, 6 mm closed cap, bore/track radii 17-27 mm, 0.25 kg (aluminium); post diameter 31 mm (1.5 mm radial play); pins dia 6 mm, radial 12-25 mm; track channel edges +-16 deg, axial channel height 10 mm (2 mm play); plunger preload 15 mm, stiffness sampled 280-360 N/m per seed; track friction 0.12 (greased steel), shell friction 1.0 |
| holder / stand | generic bolted base plate with a fixed shaft stub (machined block, 4 bolts) | 120 x 110 mm base (card) | 16 mm plate, post height 85 mm |
| tray | generic plastic parts tray | none | inner 200 x 150 mm, 28 mm walls |
| gripper | stock Franka Hand (repo model) | sim per-finger force 40 N, opening 0-90 mm (card / repo) | finger servo target filter time constant 0.3 s (polish 2026-10-29): closing ~0.1-0.15 m/s per finger, estimated from the real hand's speed of roughly 0.05-0.1 m/s per finger (not checked against a datasheet) |

Licences: all scene geometry is original code in this module (same licence as the TinkerBench package); robosuite Panda
meshes MIT.
