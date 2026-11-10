## B4_bars

No meshes or textures: every object is a MuJoCo primitive box built in `benches/B4_bars.py`. `benches/B4_bars_assets/` is
therefore not needed. The robot is the repo's two-Panda B4 robot (robosuite Panda meshes, MIT licence, via
`portable_demo.env.scene`). Web access was limited to a search engine that returns titles only, so product values below are
standard nominal sizes or computed from material density, and are marked as such.

| object | real product it is based on | values from the product / standard | estimated values |
|---|---|---|---|
| bars A, B, C | 6063-T5 aluminium square tube 50 x 50 x 1.0 mm (**estimated / uncommon thin-wall stock**: EN 755 metric catalogues list 50 x 50 square tube from about 1.5 mm wall and US stock from 2 x 2 x 1/16 in (1.59 mm); a 1.0 mm wall at 50 mm is offered only by some thin-wall or decorative suppliers, not confirmed by a listing; LEAD_DECISIONS #74), cut to 320 mm, two push-in end caps (a black cap marks the -X end), with a steel slug fixed inside (a common shop-made "hidden weight" bar; no single catalogue item) | 50 x 50 mm section, 1.0 mm wall: area 196 mm^2, mass 0.53 kg/m at 2.70 g/cm^3 (computed) -> tube 0.17 kg at 320 mm; shell incl. caps `SHELL_M` = 0.19 kg | slug mass = total mass - 0.19 kg (0.36-0.66 kg) and its position (x_s = M c / m_s, at most 137.5 mm from the middle, slug end inside the cap); total mass 0.55-0.85 kg (card); COM offset 10-90 mm, random sign (widened from the card, DEVIATIONS); inertia from thin-walled tube + slug (computed); aluminium friction mu 0.6 (estimated, dry anodised aluminium on steel/aluminium) |
| practice pedestal (exploration, task A) `riser_246` | machinist's 2-4-6 block (e.g. Fowler or Starrett 2-4-6 blocks; catalogue pointer: https://mtmtoolsupply.com/wp-content/uploads/2019/01/Parallel-Sets-V-Blocks-1-2-3-Blocks-FOWLER-STARRETT.pdf), standing on its 2 x 4 in face | 2 x 4 x 6 in = 50.8 x 101.6 x 152.4 mm (nominal size, by definition) | modelled as fixed to the table (a real 2-4-6 block weighs about 5-6 kg, estimated from 786 cm^3 of steel minus its holes, and does not slide or tip under 2.4 kg of bars); steel mu 0.5 (estimated) |
| `ridge_24`, `ridge_40` (edge_A, edge_B) | machined steel ridge support (a narrow flat-topped fulcrum block), no specific catalogue item | top width 24 / 40 mm from the card | 100 mm long, 160 mm high (earlier build); fixed to a base plate on the table (modelled as fixed to the world); mu 0.5 |
| `point_28` (stack_ABC) | machined steel point pedestal, no specific catalogue item | 28 x 30 mm top (earlier build) | 160 mm high; fixed; mu 0.5 |
| `ridge_ped_40` (stack_CBA) | machined steel ridge pedestal, no specific catalogue item | 40 mm wide top (earlier build) | 100 mm long, 160 mm high; fixed; mu 0.5 |
| gripper | stock Franka Hand on the repo robot | sim: per finger kp 1200, +-40 N, 0-90 mm opening, pad mu 1.5 (repo model) | real hand: 70 N continuous / 140 N peak, 80 mm stroke (card, third-party datasheet summary) |

Licences: no third-party assets added. Robot meshes: robosuite (MIT). MuJoCo: Apache-2.0.
