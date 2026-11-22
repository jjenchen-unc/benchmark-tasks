# Hang a cabinet door on clip-on hinges: hold the 1.7 kg door at the opening while the other hand clicks a hinge arm on

The task is the Isaac Lab-Arena (PhysX) `yam_arena` task in `isaac/`; `src/` holds the engine-independent modules it
imports (layout and geometry, scorer, fast IK) and `tests/` their unit checks.

## The task
A frameless wall-cabinet carcass (300 x 400 x 300 mm, 18 mm white melamine board, 8 mm back) stands on two 2x4 offcuts
laid flat on the bench, its open front toward the robot. Two clip-on cruciform mounting plates (Blum CLIP 173H7100
class, 0 mm) are screwed to the inside of its left side panel on the 37 mm line, 80 mm from the top and the bottom of
the door. The door, 298 x 398 mm of 19 mm MDF (1.69 kg), carries two clip-on 110 deg full-overlay cup hinges (Blum CLIP
top 71B3550 class) in 35 mm cup bores; it stands upright in the groove of a third 2x4 offcut on the bench beside the
cabinet, roughly in its open attitude. The robot must hang the door: carry it to the opening, hold it there with both
hinge arms' front lugs over the plates' noses and the arms' rears standing off the plates, drop both lugs on together,
clip the arms onto their plates (the reference policy presses each arm's rear until its latch clicks), then swing the door shut and take its hands off.

Why it is hard: one hook cannot hold the door (its moment rolls a single lug off its plate), so both lugs, 238 mm
apart and seen only obliquely through the cabinet's open front, must be brought over their plates' noses and dropped
on together while the door is held at the right height (the carcass stands on blocks, so the door cannot rest on the
bench). Each arm's rear then needs about 22 N to click, and turning the open door square does not click it (the hinge
pivots slip), so at least one arm must be pressed while the other hand keeps the hooked door up and square. Once one
arm is clipped, swinging the door shut also seats the other, still hooked arm (a verification run that pressed only
the lower arm and then swung the door shut scored 1.0).

Putting a flat-pack kitchen cupboard together often leaves the door until last. The mounting plates are already inside the cabinet. The person fitting the door supports its weight while hooking both hinge arms onto the plates, then presses their rear latches and checks the closed door's top edge. The same operation lets a household replace a door or put it back after moving the cabinet.

## Scoring (`src/scoring.py`, primary score 0..1, value at the end of the trial)
- 1.0 (ends the trial): both hinges clipped onto their plates (each arm's rear latch closed), the door shut (opening at
  most 3 deg, its top edge level with the carcass top within 1.5 mm), every fingertip 30 mm or more from the door and
  the hinges, door still for 1 s.
- 0.5: both hinges clipped, door not shut (same hands-off and still conditions).
- 0.25: exactly one hinge clipped. For every positive score, all fingertips must be at least 30 mm clear of the door and hinges and the door must remain still for one second.
- 0: no hinge clipped (door on the bench, in its groove or still held).
Real trial: pull the door gently at each hinge (a clipped arm stays on), look at each arm's rear (flat on its plate),
look at the shut door's gaps against the carcass; the video shows the hands off.

## Dream-1 evaluation

Dream-1 scored 0.0 on seed 0 at the 2,400-step limit. Scripted reference policy: official CLI run 1.0 (1,067 of 2,400 steps, default seed); direct demo 1.0 at the default seed (twice, identical traces) and seed 7; no-op 0; open-loop replay at seed 7: 0; door twisted, not pressed: 0.

From the task's `isaac/` directory in the Isaac Sim 6.1 and Isaac Lab-Arena environment, run:

```bash
instruction=$(python -c 'import json; print(json.load(open("../task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
    -P model=dreamlabs/dream-1-beta -P rows_per_request=10 \
    -E task=task.py -E object_jitter=0 -E device=cpu -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
    --scorer task_score --seed 0 --max-steps 2400 --store-frames --log-dir ./dream-output
```

## Run and score a trial
Inside the Isaac Sim 6.1 + Isaac Lab-Arena container, working directory `isaac/` (`isaac/run_frozen.sh` does this on a
frozen copy of `isaac/`, `src/` and `task.json`, writes `source-sha256.txt` and logs to `<out>/<run>/run.log`; see its
header for the environment variables it needs):

```
# official Inspect Robots run of the scripted reference demo (registered closed-loop policy cabinet_door_scripted)
python official_run.py -- run --embodiment yam_arena --policy cabinet_door_scripted -E task=task.py -E object_jitter=0 \
    -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 \
    --instruction "$(python -c 'import json;print(json.load(open("../task.json"))["instruction"])')" \
    --scorer task_score --max-steps 2400 --store-frames --log-dir <out>/official
inspect-robots video <out>/official/<log>.json --out <out>/official/video     # H.264 video of every camera

# plain do-nothing run (task.py demo() gives the yam_scripted policy no waypoints: both arms hold READY)
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py ... --max-steps 2400

# direct demo / negative stages (trace.jsonl, result.json, demo_<camera>.mp4, setup.png, final.png)
python demo.py task.py <out>/<run> --stage demo --seed 1415158438   # the default inspect-robots trial seed
python demo.py task.py <out>/<run> --stage neg-twist      # both lugs hooked, door twisted square and past instead of pressing the arms: 0
python demo.py task.py <out>/<run> --stage neg-drop       # one hand lets go before the door is hooked: 0
python demo.py task.py <out>/<run> --stage neg-one-clip   # both hooked, lower arm clipped only; the script stops on an unreachable exit pose with the left hand still on the door (scores 0, not the intended 0.25)
python demo.py task.py <out>/<run> --stage look --cam-size 1280x704   # media: wrist camera on the plates
CDH_REPLAY=<run>/trace.jsonl python demo.py task.py <out>/<run2> --stage replay --seed <s>   # open-loop replay
python negcheck.py negcheck.json <run> [<run> ...]       # stimulus + reaction check
python action_check.py <official log dir>                # executed vs commanded actions
```
Unit checks (no simulator): `python -m pytest tests/test_scoring.py` (or run the test functions directly).
Tiled evidence video (top + left wrist + right wrist, same timing):
`isaac/tile.sh <out_dir> <name> demo_top_cam.mp4 demo_left_cam.mp4 demo_right_cam.mp4`.

## Recorded media

`model_run.mp4` shows the full seed 0 Dream-1 trial (2,400 steps, final task score 0.0).

- `official_tiled_top_left_right.mp4`: the official Inspect Robots run of the scripted reference policy at the default
  seed, score 1.0 after 1,067 of 2,400 control steps (H.264, 960 x 176).
- `setup_overhead_hires.png`: the default-seed start state from the overhead camera at 1280 x 704 (carcass on its
  blocks, door with its two hinges standing in the groove block).

Start states: every trial reset with a seed (every `inspect-robots run` passes one) draws the carcass position and the
door's start pose in its groove (see `task.json` "setup"). Unseeded resets (direct `demo.py` runs) use the five named
setups in turn (`CDH_SETUP` picks the first; set, it also forces named setups on seeded resets): carcass offset
(dx, dy) mm and door start (in front, outboard) mm, turn deg: (0, 0) (30, 100) 0; (12, -8) (38, 92) -3; (-10, 6)
(22, 112) 2.5; (6, 10) (35, 108) 3.5; (-14, -6) (25, 95) -2.

## Fidelity (full statement)
- Engine: Isaac Sim 6.1 / Isaac Lab-Arena `yam_arena`, PhysX on the CPU, physics 480 Hz, control 20 Hz, cameras
  320 x 176. All parts are full size.
- Door: 298 x 398 x 19 mm MDF slab (750 kg/m^3, 1.69 kg), lacquered surface (friction 0.8 against the finger pads).
  Hinges: representative geometry of a clip-on 110 deg full-overlay cup hinge of the Blum CLIP top class (35 mm cup,
  22.5 mm cup centre from the hinge edge, a 17 mm wide arm about 60 mm long reaching back onto the plate); plates:
  cruciform 0 mm plates of the Blum 173H7100 class on the 37 mm system line. The part numbers name the class of
  hardware; the dimensions are our estimates from the published system dimensions (35 mm cup, 37 mm line) and could not
  be checked against a drawing from our build host.
- The door is one floating PhysX articulation: the slab (with the cups) and the two hinge arms, each on a revolute
  joint (0 to 105 deg of opening). The hinge's four-bar linkage is modelled as a single pivot at the door's outer
  hinge-edge corner: the open (90 deg) and shut door positions are those of a full-overlay door; the swing path in
  between is simplified. Pivot resistance: a damping drive on each pivot (1 N m s/rad, capped at 0.8 N m; viscous, no static
  friction). Self-closing spring: 1.2 N m per hinge below 35 deg of opening; soft-close: the pivot damping rises to
  2 N m s/rad below 20 deg (estimates; the real hinge's integrated spring and damper are not published as numbers).
- Plates and arms (computed contact model, ours; the plates are drawn but have no collision shape, the arm links do
  not collide with the carcass, everything else uses PhysX contacts): an arm's front lug hooks only when it comes down onto its plate's nose within 3 mm of the seat
  along the plate and against the panel, with the arm's rear standing 1.5 to 12 mm off the plate; the hooked lug is
  held by a stiff spring-damper (1.5e4 N/m) and rolls off if pulled 4 mm (about 60 N), if the rear swings more than
  14 mm off, or if the arm tilts 10 deg. The rear: a stop at the plate plus a latch detent that pushes it back with
  22 N over the last 2 mm; pushed to within 0.2 mm of seated, the latch clicks and the arm is held on the plate by three
  stiff point springs (clipped). The click force is an estimate (no published figure); the release lever is not
  modelled (a clipped hinge stays on). A light touch never clicks a hinge, and a hinge cannot click unless hooked. The soft-close damping acts
  between the door and the arm, so a door swinging shut can drag a hooked arm's rear onto its plate and click it
  (observed with the other arm already clipped).
- The hinge arm's inertia is raised to 2e-4 kg m^2 (from about 4e-5) for the stability of the explicit plate springs;
  the arm is light and moves with the door through its pivot, so this has no visible effect.
- Carcass, plates, the 2x4 blocks and the groove block are kinematic (the cabinet would be standing or clamped; nothing
  in the task moves it). Friction (static; dynamic 0.9x): wood and melamine 0.6, steel 0.3, door 0.8; the gripper pads
  use PhysX defaults combined by averaging.
- Reference demo grip: about 20-24 N per finger on the door's free edge (two hands); press: a 40 N fingertip push
  toward the panel on each arm's rear.
- Nothing welds, snaps or teleports during a trial; the scripted demo only sends the 14 joint / gripper commands and
  reads poses, joint angles and hinge states. The scorer only reads.

## Real rig
A frameless wall-cabinet carcass 300 (w) x 400 (h) x 300 (d) mm of 18 mm white melamine particleboard with an 8 mm
hardboard back (any 30 x 40 cm wall-cabinet carcass, e.g. a flat-pack kitchen wall cabinet), two Blum CLIP cruciform
mounting plates 173H7100 (0 mm) screwed on the inside of the left side panel on the 37 mm line, 80 mm from the top and
the bottom of the carcass (System 32 holes); a 298 x 398 mm, 19 mm MDF door with two 35 mm cup bores (5 mm boring
distance, 80 mm from its top and bottom edges) and two Blum CLIP top 110 deg full-overlay hinges (71B3550) screwed in;
three 2x4 offcuts (38 x 89 mm): two 320 mm long laid flat under the carcass sides, one 260 mm long with a 19.5 mm wide,
12 mm deep groove along it, laid flat beside the carcass, the door standing in its groove. Hinges adjusted to their
middle settings.

## Third-party assets and licenses
| Asset | Source | License |
|---|---|---|
| YAM arm model (URDF, meshes) | i2rt, via the Isaac Lab-Arena YAM package | MIT |
| Station scene, cameras, `yam_arena` embodiment | Isaac Lab-Arena YAM package | as distributed with the package (no license file at the pinned revision) |
| Isaac Sim 6.1 / Isaac Lab-Arena | NVIDIA | NVIDIA Isaac Sim license / Apache-2.0 |
| Carcass, plates, door, hinges, blocks geometry | generated in code by this task (`src/layout.py`) | MIT (this repository, `LICENSE`) |
| `src/fastik.py` (YAM URDF kinematics) | this task's authors | MIT |
"Blum", "CLIP", "CLIP top", "173H7100" and "71B3550" name the real hardware the replica represents; no manufacturer
asset (CAD, texture, logo) is used.
