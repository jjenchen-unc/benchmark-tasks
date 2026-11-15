# U-lock a bike wheel to a stand

Lock the child's 12-inch back wheel to the Sheffield stand with a full-size,
two-part bent-foot U-lock. One leg passes between spokes, the other outside the
tyre, with the stand leg between them. The loose crossbar receives the outwardly
bent foot at an angle, then pivots onto the straight end. Both hands let go.
The goal secures the wheel, not every part of the bicycle against theft.

## Revision status and interpretation

PR1805 was reviewed with: “U-lock should be modeled realistically such that an
angled entry is required on entry”. The interpretation here is **crossbar
engagement on a bent-foot U-lock**, not a requirement to tilt while threading
the wheel. The review did not identify the mechanism; this is an engineering
interpretation, not an official clarification.

Kryptonite's [manufacturer brochure](https://www.kryptonitelock.com/content/dam/kryt-us/en/documents/seller_reseller_retailer_docs/2017_Kryptonite_BrandGuidelines.pdf)
documents a bent-foot design. [US4155231A](https://patents.google.com/patent/US4155231A/en)
(summary, detailed description, Fig. 3, claim 1) explicitly describes tilted
engagement, inserting the bent end first and pivoting the crosspiece onto the
straight leg. Its detailed description gives representative half-inch rod,
roughly 8–9 inch shackle length and outward bent foot. This supports a mechanism
family; the dimensions below are authored approximations, not a measured replica
of a current Kryptonite product. [US5092142A](https://patents.google.com/patent/US5092142A/en), originally assigned to Kryptonite, also describes a sloped bent foot acting as the assembly fulcrum. The representative diagonal toe here follows that variant. Straight-ended dual-bolt U-locks also exist;
angled engagement is not claimed to be universal to all U-locks.

The published baseline used two straight ends and equal sockets, so both could
be inserted by one axial slide. This revision changes actual visible colliders:
one foot bends outward and the matching housing has a narrow entrance, a retaining
lip and an enlarged inner chamber. The sideways toe cannot translate through the
aligned mouth. Angled insertion followed by a pivot clears the same solids.
There are no extra obstacles, collision exemptions, invisible interlocks or
approach-angle/history scoring tests. Wheel, stand and robot layout are unchanged.

**Revised official-runtime evidence:** Dream-1 beta seed 1 completed all 1,200
actions with score 0.0 and zero errored trials; the scripted demo reached 1.0 in
940 actions (47 seconds). A real post-demo reset restored the parts and passive
wheel, cleared the catch and returned the score to zero. New model/demo/setup
media show this bent-foot revision. Historical baseline evidence is separately
identified in `provenance.json` and is not used for these scores.

## Files and trial commands

`task.py` defines geometry and scoring; `lockkit.py` builds rigid compound
colliders and the wheel hinge, hooks the standard embodiment and manages the
catch joint. `demo.py` uses joint commands and physical gripper grasps. No SDK
files are modified. `provenance.json` preserves published evidence separately.

In the pinned YAM/Arena environment, from this directory:

```bash
inspect-robots run --no-prompt --embodiment yam_arena --policy dream \
  -P model=dreamlabs/dream-1-beta -P rows_per_request=4 \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E device=cuda:0 -E cam_width=320 -E cam_height=176 \
  --instruction "Lock the bike's back wheel to the bike stand with the U-lock." \
  --scorer task_score --max-steps 1200 --seed 1 --store-frames --log-dir logs/u-lock-dream
python demo.py task.py demo_out
DEMO_VARIANT=straight python demo.py task.py straight_out --no-record
DEMO_RESET_CHECK=1 python demo.py task.py reset_out --no-record
```

The official `task_score` scorer takes success or the final `score()`.
`--policy yam_scripted -P hold_s=60` holds still, since `task.demo()` is empty.
No model call is needed for the scripted demo. For cluster development, launch GPU simulation in an allocated container through
Slurm.

## Robot demonstration and controls

The controller starts with both parts lying on the table. Each arm picks up its
part, the left threads the shackle through the wheel and around the stand, while
the right carries the bar behind the wheel. The bar approaches with about 47°
relative yaw around the bent end. Translation feeds the toe through its mouth;
a series of rotations brings the other socket over the straight end. The hands
then open and retreat. The initial angle is a controller choice, not a threshold
in the task or scorer.

The script plans from the fixed layout, observed simulator poses of the visible
parts and its own commands. It does not place objects, apply evaluator forces,
read contacts or use the latch state for planning. This privileged pose access
is disclosed. `trajectory.json` records actual part poses and actual issued joint
commands; videos and traces are written outside the source directory.

| Variant | Purpose | Revised physics result |
|---|---|---|
| full (default) | pick up both parts, angle, pivot, release | 1.0 at 940 controls; post-success reset passed |
| straight | former axial slide onto both ends | 0.0, no latch, 862 controls; toe blocked on housing lip; reset passed |
| nothing | do nothing | 0.0, 160 controls |
| no_crossbar | thread shackle, never attach bar | 0.25, 616 controls; no latch |
| short | bent foot entered but straight end not seated | 0.25, 870 controls; no latch |
| held | complete closure, keep both hands on | 0.5, 1150 controls; latched around both, held |
| air | attempt closure around no objects | 0.0, 538 controls; grasps failed, so this does not establish closed-air rejection |

`DEMO_RESET_CHECK=1` resets after each completed trial, advances 20 stationary
control steps, verifies both parts within 3 mm of their start positions, no latch,
no enabled catch joint and score zero. It also records rear wheel angle. Catch
joints are disabled **before** reset settling, then score/latch histories clear.
Closed-lock evaluator placements around only the stand, only the rim, both legs inside the wheel opening, and in empty space all scored zero, including 20 physics steps per fixture. These establish scorer controls only; they are not robot demonstrations. Synthetic false-seat checks also passed. A passive-wheel reset defect observed after the first demonstration was fixed and revalidated after successful closure and failed straight insertion. Geometry, latch/scoring and robot control definitions were unchanged by that reset fix.

## Setup and dimensions

The station table represents pavement. Coordinates use the SDK station world
frame; height is above the table. The rear hub is at (38, -1, 15.9) cm. The park
frame is yawed -25°; x runs across the bike, y toward its front, z up.

- Child's bicycle: 57-203 rigid rubber tyres (317 mm outer diameter), ISO 203
  aluminium rims (169–203 mm diameter), sixteen 2 mm radial steel spokes and
  a red steel frame. Wheelbase 480 mm (shorter than real 580–600 mm examples to
  fit the station); front wheel/frame fixed, back wheel 0.8 kg on a lightly damped
  axle. Initial spoke directions are 11.25° from the radial insertion line.
- Stand: fixed galvanised steel Sheffield stand, 48 mm tube, 750 mm high, legs
  700 mm apart. The rear leg is at park x=-60.5 mm, y=-142 mm; its plane is 8 mm
  ahead of the tyre. Commercial stands are often longer. No layout change in
  this revision.
- Shackle: 0.62 kg, 12.7 mm steel rod, 114.7 mm axis spacing (102 mm internal),
  approximately 220 mm closed internal depth (94.7 mm sleeve clearance); 20 mm vinyl sleeve over the bend and
  first 70 mm. In its local xy plane, legs run +x at y=±57.35 mm. The inner
  straight end's outermost point is x=198 mm. The outer leg bends from
  (183, -57.35) mm to toe centre (198, -73.35) mm. The actual diagonal segment
  has length 21.93 mm and rounded 6.35 mm radius. Initial bend centre is world
  (13, 17, 1) cm, yaw -25°.
- Crossbar: 0.70 kg nominal lumped mass, 40×40 mm steel-core housing with a black rigid polymer casing, 180 mm long, with a 50 mm
  cylindrical lock head outside one end. Socket centres y=±57.35 mm. Entrance
  plane x=-20 mm, openings 20×20 mm, lip 8 mm thick. Hook chamber extends to
  x=12 mm and y=-84 mm; straight socket bottom x=5 mm (25 mm depth).
  Initial centre (25, -30, 2) cm, yaw -25°. Only one mating orientation fits.

The exact primitive positions/materials and the remaining bicycle frame members,
seat, pedals, handlebar and kickstand are specified in `task.py` and the setup
field of `task.json`. The setup screenshot shows the separated bent-foot shackle and housing.
There is no randomisation; use `object_jitter=0`.

## Scoring

Scoring uses the final physical linkage, never the approach angle. The closed
loop follows the shackle including the bent foot and the bar joining its ends.
An odd signed crossing of the rim's spanning disc means it links the rim; an odd
crossing of the Sheffield stand opening means it links the stand.

The catch engages only when the straight end lies 21–25.5 mm inside its socket
mouth, within 3 mm of its y/z centre, **and** the bent toe is within 3 mm of its
final y/z centre with axial position strictly -5 to 6 mm in bar coordinates.
Both are bounded final assembly tests. A bar past/through the back, an offset
socket or an unengaged straight end must not latch. The fixed joint then holds
the bar at its assembled pose; key and bolt operation are simplified.

- 1.0: latched and linked around both rim and stand, every finger pad at least
  30 mm away, both parts moving less than 2 mm over the last second; success
  must hold for another second.
- 0.5: latched and linked around both, still held or moving.
- 0.25: shackle threaded around both, not closed by a latched bar.
- 0: all other outcomes, including a closed lock around only the stand, only
  the rim, nothing, or both legs inside the wheel opening.

These pre-existing score tiers and the 1,200-step/60-second limit are preserved.
Measured revised runs take roughly 4–7 minutes for full/straight trials on the development GPU, including startup and recording; shorter controls take less. The full robot sequence is 47.0 seconds at 20 Hz within the 60-second limit.

## Fidelity and real-world setup

The mechanical principle follows bent-foot locks, using authored primitive shapes
and representative clearances. Capsule ends and rectangular sockets approximate
curved/chamfered metal; there is no claim of manufacturing fidelity. The visible
housing/lip enforce the entry sequence by collision. The collision housing is an outer-envelope approximation with nominal lumped mass; internal material distribution and manufacturing wall thickness are not resolved. The fixed catch approximates
a straight-foot bolt; key turning is omitted. The tyre is rigid; sleeve/finish
friction against gripper pads is 1.0. No simulated force cap is enforced. The task
does not model cutting, breakage or lock-picking resistance.

For a physical trial use a matching bent-foot U-lock, a 12-inch spoked wheel and
48 mm stand tube, and measure actual product clearances. Use the key held in
its ready position or a push-to-lock cylinder, documenting that simplification.
Mark starting part outlines and spokes. Inspect the closed lock, then check by
hand that both rim and stand are captured. Open the cylinder and separate the
parts for reset. Do not substitute a straight-ended dual-bolt lock when testing
this angled-entry variant.

## Third-party assets and licenses

The SDK is yam-isaac-lab-arena at 0059d02a1a91bc6834c1d0846022891df0c78730, the program's official dependency,
installed by the user's environment. Nothing from it is redistributed here. Licenses are given as each source states
them.

| Asset | Used for | Source | License as stated by the source | Bundled |
|---|---|---|---|---|
| YAM arm and gripper (URDF, meshes, collision hulls, converted USD) | both arms | SDK `yam_arena/assets/yam/`, from i2rt (SOURCE_COMMIT 120c3c81400171174604e503943f8d1ebc891058) | MIT, Copyright (c) I2RT Robotics | no |
| link2/link3 meshes split into black and white parts | station arm colours | SDK `yam_arena/assets/yam/dream/`, from the dreamlabs 1.0.23 wheel | "MIT, see ../LICENSE.i2rt" (SDK `dream/SOURCE`) | no |
| Station floor and table | room | SDK `station.py`/`scene.py`: boxes with flat colours generated in code | SDK README: "This repository has no license yet: most of it is ported from YAM-inspect-Isaac by Thomas Pierson, which has none." | no |
| Top-camera stand | room | SDK `station.camera_stand`: boxes and capsules generated in code | as the row above | no |
| Dome light and lamp | lighting | SDK `scene.py`, using Isaac Lab-Arena's DomeLight and an Isaac Lab sphere light | as the row above | no |
| top_cam, left_cam, right_cam | policy RGB | SDK pinhole camera sensors | as the row above | no |
| Station fruit, bowl, plate (Arena object library) and water bottle | not used: this task's own objects replace them | - | - | no |
| Bike, cycle stand, U-lock | task scene | this folder (`task.py`, `lockkit.py`): primitive shapes built in code | the authors' own, MIT License in `LICENSE` | yes |

Isaac Lab-Arena release 0.3.1 (c7edf3a9d5d8) ships the Apache License 2.0. Isaac Sim and Isaac Lab are runtime
dependencies under their own licenses. None of them is redistributed here. Manufacturer/patent references inform mechanism design only; no drawings, logos, meshes or other third-party assets from those references are bundled. All task-specific code and geometry
(`task.py`, `lockkit.py`, `demo.py`) is the authors' own, under the MIT License in `LICENSE`.
