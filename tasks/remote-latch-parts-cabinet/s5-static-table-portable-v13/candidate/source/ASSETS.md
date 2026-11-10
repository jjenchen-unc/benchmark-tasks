## S5_remote_release

No meshes or textures are used: every object is built from MuJoCo primitive geoms in `benches/S5_remote_release.py`
(`benches/S5_remote_release_assets/` is empty). Robot meshes come from robosuite (MIT) through the repo snapshot.
Web search was not used; product facts below are from memory of the makers' catalogues and are marked as such.

| object | real product it is modelled on | values taken from the product (memory, not re-verified) | estimated values |
|---|---|---|---|
| tool cabinets A, B, N (`cab_*`) | Benchtop double-door steel cabinet / small two-door enclosure fitted with a remote-release rotary-latch kit (e.g. Southco R4 series rotary latches + AC series remote paddle actuators). No single catalogue cabinet with these exact dimensions. | two hinged doors, no handle function on the doors (released from a remote paddle), steel sheet construction | 360 W x 180 D x 500 H mm, 10 mm walls, 12 mm doors of 176 x 478 mm, 0.75 kg per door, 80 mm high partition between the two compartments. Sized to the Panda workspace. |
| door knobs (`*_door_*_knob`) | fixed cabinet pull knob (no latch function) | round steel knob, no moving part | 18 mm shaft x 40 mm standoff, 30 mm cap, 45 mm below the door top edge, 136 mm from the hinge axis |
| side paddles H1/H2 (`*_paddle1/2`) | Southco AC series paddle actuator (remote release of rotary latches through a cable; a cable splitter lets one paddle release two latches) | paddle pulled away from its housing releases the latch at the end of its stroke; a splitter paddle drives two cables | stroke 12 mm, release at 9 mm, return spring 500 N/m with 2 N preload (about 7 N at release); a splitter paddle carries two cable return springs, so 2x (card: about 2x, INFERRED) |
| rotary latch, two-stage or single-stage per door (rule, no geometry) | rotary claw latch; two-stage versions (Southco R4 family; McMaster-Carr lists safety-catch versions) have a first catch that holds the door nearly shut; single-stage versions catch once | first catch (safety stage) leaves the door slightly proud of the frame; the second stage needs the gasket pushed further | first stage at 1.35 deg (3.2 mm proud at the knob), second stage at 0.10 deg, frame stop -0.15 deg; claw detent peak 8 N at the knob over 1.4-2.8 deg, felt only while the claw is armed; kick-off spring 3 N at the knob below 12 deg (door pops ~14 deg); slam limit 0.25 m/s at the knob (estimated) |
| door gasket (rule, hidden value f2) | closed-cell EPDM door gasket compressed by the second latch stage | gasket force rises over the last few degrees of closing | second-stage force at the knob f2: 15-25 N (cabinet A, small) and 30-45 N (cabinet B, large) per the card (INFERRED); gasket zone 1.2 deg (2.8 mm at the knob) |
| boxes (`*_box_L/R`) | small boxed spare part / parts box with a front grab tab | none | 50 x 50 x 60 mm, 0.30 kg, tab 37 x 20 x 44 mm |

Placement (layout change 2026-10-29, lead decision #136; no object dimensions changed): cabinet A (and N in T4/T5)
front-plane centre at world (0.10, -0.29), 0.13 m inboard of the left Panda base's line (was (0.10, -0.42)); cabinet B at
(0.18, 0.30) as before. Boxes and paddles move with their cabinet.

Product links (from memory, not re-verified on the web; treat as estimated pointers):
- Southco rotary latches (R4 family, two-stage versions with a safety catch): https://southco.com/en_us_int/rotary-latches
- Southco remote-release actuators and cable kits (AC series paddle actuators, cable splitter): https://southco.com/en_us_int/actuators
- McMaster-Carr rotary latches, incl. remote-release and two-stage types: https://www.mcmaster.com/rotary-latches/
- McMaster-Carr remote-release paddle handles and cables: https://www.mcmaster.com/remote-release-latches/

Licenses: MuJoCo (Apache-2.0), robosuite Panda meshes (MIT). No third-party assets were added.
