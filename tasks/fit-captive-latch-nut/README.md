# Fit a wrench, release a captive latch nut and open its arm

A steel latch is fixed on the table: a swing arm with a black knob, clamped shut by a captive hex nut with a
yellow paint mark. The nut is unmarked; its size (12.70 mm across flats, 1/2 in) is not stated. A rack beside it
holds five open-end wrenches: 12 mm, 1/2 in, 13 mm, 14 mm and an adjustable one preset to 13.55 mm. The clamp lets
the arm go once the nut has turned more than 60 degrees. Only the 1/2 in wrench turns the nut; the looser ones
round its corners under load (the nut then turns bright and only spins, and the clamp never lets go), and the
12 mm does not go on. The robot has to find the wrench that fits, loosen the nut, swing the arm to its stop, put
the wrench back in its own place on the rack and let go of everything.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects and `score()` |
| `kit.py` | Spawns the fixture and wrenches in Isaac Lab-Arena, sets 2 ms physics, runs the mechanism every physics step |
| `physics.py` | The nut, clamp and arm mechanism and the measurements the scorer uses |
| `scoring.py` | The rounding rule and the scoring rules (all thresholds are named constants at the top) |
| `contacts.py`, `motor_limits.py` | Contact reads; the arms' total joint torque cap (27 / 7 N m including gravity) |
| `fixture.json`, `assets/*.usda` | Geometry, masses, joints and rack positions of the fixture and the five wrenches |
| `demo.py` | Scripted full-score demo and the cheap-strategy variants, with video recording |
| `probe.py` | Produces the trial-fit table below |
| `LICENSE`, `LICENSE.i2rt` | This task's license; the YAM arm model's license (see the last section) |

## Run a trial

In the Isaac Lab-Arena environment, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream -P model=dreamlabs/dream-1-beta -P rows_per_request=4 \
  -E task=task.py -E device=cpu -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$(python -c 'import json;print(json.load(open("task.json"))["instruction"])')" \
  --scorer task_score --max-steps 3870 --seed 42 --store-frames --log-dir out-dream
```

`task_score` gives 1.0 when the episode ends in success, otherwise `score()` at the last step. The layout is fixed
(`object_jitter=0`) and the seed does not change the scene. Physics runs on the CPU at 2 ms steps (25 per action);
the cameras need an RTX GPU. A full 3,870-step trial takes about 25 minutes of simulation plus model time.

## Scoring

The score is the live output of `score()`; everything it uses can be checked on real hardware with a protractor,
a ruler and the video. No force or torque is measured.

- Nut released: the paint mark has turned more than 60 degrees counter-clockwise (seen from above) from its start,
  and the nut is not rounded (a rounded nut shows bright corners and spins freely).
- Hands off: no part of either robot and no wrench touches the nut or the arm.
- Wrench back in its rack position: it lies flat on its own two rack blocks, that is its jaw centre is within
  10 mm horizontally of its marked rack position and within 3 mm of its rack height, its long axis is within
  10 degrees of the rack direction (handle toward +x), its face is within 5 degrees of horizontal (either face
  up), and no part of either robot touches any wrench. The rack blocks are 32 mm wide and the wrench handle 12 mm,
  so 10 mm off-centre still leaves the handle on the blocks; a wrench whose handle end has slipped off its back
  block tilts by about 12 degrees.
- 1.0: nut released, hands off, arm open at least 85 degrees (the stop is at 90) and all five wrenches back in
  their own rack positions, all continuously for 20 s. The episode ends then with success.
- 0.5: nut released and hands off continuously for the last 20 s of the episode, with the arm at any angle or a
  wrench not back in its rack position (for example the used wrench left on the table).
- 0 otherwise. A rounded nut never counts as released, so rounding scores 0 for the rest of the episode. A state
  that cannot be read scores 0.

The simulator checks these every 2 ms physics step, so any touch or any wrench off its place during the 20 s
restarts the count. A robot may pick up and try any wrench; only the final state counts.

What changed from the first version: the full score now requires every wrench back in its rack position (before,
returning the tools was only instructed). Removed: the wrist and arm-link force limits that ended the episode
(they cannot be checked with simple tools and are not what the task tests), the internal record of rounding
(rounding is permanent and visible, so the final state shows it) and the "tool out of bounds" veto (a wrench
dropped off the table is not in its rack position, so it already blocks the full score).

## How the hidden property can be observed

Which wrench fits is found by a trial fit. `probe.py` picks each wrench from the rack with the demo's motions,
pushes it onto the nut, then turns it slowly (0.25 degrees per step) up to 20 degrees and records what a camera
sees: whether the head goes onto the nut, how far the wrench turns before the nut's paint mark moves, and whether
the nut turns bright (rounded). The wrench turn before anything happens is the visible play of the fit.

| Wrench (jaw opening) | Goes on? | Wrench turn before the nut moves or rounds | After a 20 degree turn |
|---|---|---|---|
| 12 mm (12.05 mm) | no: the jaws stop 13.3 mm short of the nut's axis | - | nut untouched |
| 1/2 in (12.75 mm) | yes | 1.1 degrees, then the nut turns with it | nut turned 19.5 degrees, not rounded |
| 13 mm (13.05 mm) | yes | 3.0 degrees, then the nut rounds | rounded (bright), spins: paint mark 17.3 degrees, clamp still locked |
| 14 mm (14.05 mm) | yes | 12.7 degrees, then the nut rounds | rounded, paint mark 7.2 degrees |
| adjustable (13.55 mm) | yes | 6.8 degrees, then the nut rounds | rounded, paint mark 13.1 degrees |

So the 12 mm visibly does not fit, and among the wrenches that go on, the free turn before the jaws bear on the
nut grows with the clearance (1.1, 3.0, 6.8, 12.7 degrees). The 1/2 in has almost no play; the 13 mm's 3 degrees is
the smallest visible difference, about 8 mm of movement at the handle end, 160 mm from the nut. Seating a wrench
does no harm; turning a loose wrench past its play against the nut rounds it, so the play has to be judged before
the wrench is loaded, which is the skill the task is about. In the `wrong` run (13 mm, full strokes) the nut rounded at step 554 and the clamp stayed
locked. The 12 mm row is from `python probe.py task.py out w12` (the rack's far end needs a lower lift), the
others from `python probe.py task.py out`; this script, not the demo, also reads the simulator's rounding state
to confirm the colour change.

## Scripted demo

```bash
python demo.py task.py demo_out                                   # demo.mp4, setup.png, actions.jsonl, result.json
DEMO_VARIANT=nothing python demo.py task.py out --no-record
DEMO_VARIANT=leave python demo.py task.py out --no-record         # full demo, wrench left on the table
DEMO_VARIANT=wrong python demo.py task.py out --no-record         # the same motions with the 13 mm wrench
DEMO_VARIANT=push_arm python demo.py task.py out --no-record      # swing the arm by its knob without loosening the nut
```

The demo acts only through the 14 joint commands and plans from the fixed published layout (`fixture.json`). While
it runs it reads only what a camera or the robot itself could see: its own joint positions and gripper pose, the
wrench poses, and the nut's turn from its paint mark. It never reads the clamp or rounding state, contacts, forces
or physical parameters. It goes straight to the 1/2 in wrench (the table above shows that a trial fit tells the
wrenches apart), seats it on the nut, loosens the nut in 30 degree strokes with a refit after each until the paint
mark has turned 63 degrees, puts the wrench back on its rack blocks, swings the arm to its stop by the knob, moves
clear and waits.

Measured results (Isaac Sim, CPU physics, live scorer):

| Run | Score | Steps | Final state |
|---|---|---|---|
| Demo (A6000 development GPU) | 1.0 | 2,396 | nut turned 64.0 degrees, arm at 90 degrees, all five wrenches in place (returned 1/2 in wrench 0.0 mm off) |
| Demo (evaluation machine, L40S; the video) | 1.0 | 2,388 | nut turned 63.8 degrees, arm at 90 degrees, all five wrenches in place (returned 1/2 in wrench 0.4 mm and 0.5 degrees off) |
| Dream-1 (`dreamlabs/dream-1-beta`, the command above, evaluation machine) | 0.0 | 3,870 (run limit) | episode ended at the step limit |
| `nothing` | 0 | 3,870 (run limit) | nut not turned, arm clamped |
| `leave` (wrench left on the table) | 0.5 | 3,870 (run limit) | nut 64.0 degrees, arm at 90 degrees, 1/2 in wrench on the table 230 mm from its rack position |
| `wrong` (13 mm wrench forced) | 0 | 3,870 (run limit) | nut rounded at step 554 before it had turned; arm still clamped |
| `push_arm` (arm pushed by its knob, nut not loosened) | 0 | 3,870 (run limit) | arm did not move from its clamp |

## Physical setup

Coordinates are metres, x forward, y left, z up. The standard station table top is at z = 0.7475; the robot bases
are at (0, +0.305, 0.75) and (0, -0.305, 0.75), both arms in the ready pose with grippers open.

- Latch fixture, fixed: steel pillar 22 x 22 x 84 mm at (0.28, 0.12); captive steel hex nut 12.70 mm across flats,
  6.8 mm tall, unmarked, yellow paint mark, bearing face at z = 0.8375; steel swing arm (bar 18 x 98 x 4.4 mm,
  0.09 kg; black knob 12 mm diameter, 36 mm tall, 80 mm from the pivot) on a vertical pivot on the nut's axis at
  z = 0.833. The arm starts pointing toward -y; its open stop is at 90 degrees, pointing toward -x.
- Clamp: the arm cannot move until the nut has turned more than 60 degrees counter-clockwise (seen from above).
  The nut needs 0.35 N m to break free and 0.03 N m once it has turned 2 degrees.
- Rack: ten fixed steel blocks 16 x 32 x 35 mm, two per wrench, 155 mm apart along x, friction 1.0.
- Wrenches, free steel bodies, 5 mm thick, friction 0.2, lying flat across their blocks with handles toward +x,
  jaw centres at (0.30, y, 0.785) for y = 0.50, 0.45, 0.40, 0.35, 0.30: 12 mm (jaw opening 12.05 mm), 1/2 in
  (12.75 mm), 13 mm (13.05 mm), 14 mm (14.05 mm), each 160 mm from jaw centre to handle end and 0.11 kg; adjustable
  preset to 13.55 mm, 200 mm, 0.24 kg.
- Rounding: with 0.15 mm or more clearance the nut rounds when the turning torque stays above 0.20 N m (13 mm),
  0.14 N m (adjustable) or 0.08 N m (14 mm) for 10 ms, all below the 0.35 N m breakaway; a too-small wrench forced
  onto the nut rounds it above 0.05 N m. The 1/2 in (0.05 mm clearance) never rounds it.
- Arms: the platform's standard YAM arms; total joint torque including gravity capped at 27 N m (joints 1 to 3)
  and 7 N m (joints 4 to 6), 40 N per finger. Physics at 2 ms, 25 steps per 20 Hz action.

`fixture.json` and `assets/` give every shape, pose, mass and joint. Reset restores the fixed layout, the nut
(unturned, not rounded) and the clamped arm.

## Fidelity

All parts are rigid; physics runs on the CPU at 2 ms. The nut, wrench openings and clearances have real sizes, but
the torques are scaled down: the nut breaks free at 0.35 N m and turns at 0.03 N m after 2 degrees, while a real
nut of this size is tightened to roughly 10 to 25 N m; the arms' wrist joints give only 7 N m and the grip on a
5 mm wrench handle would slip long before. No thread is simulated: the nut turns on a hinge and the clamp lets go
at a 60 degree turn. Rounding is modelled as an event: a wrench with 0.15 mm or more clearance (or a too-small one
forced on) under a turning torque above a threshold that falls with clearance (0.20 N m for the 13 mm, 0.08 N m for
the 14 mm) for 10 ms rounds the nut; it then turns bright and spins freely on its stud. The adjustable wrench's
worm is visual. The arms are the platform's standard YAM arms on their standard bases, with total joint torque
(gravity included) capped at 27 / 7 N m and 40 N per finger.

## Real-world build

A steel plate with a 22 mm square pillar, a captive 1/2 in hex nut on a stud with a paint mark, and a swing arm
clamped under the nut that is free once the nut has turned 60 degrees; a 90 degree stop. A rack of ten blocks for
five open-end wrenches (12 mm, 1/2 in, 13 mm, 14 mm, an adjustable one set to 13.5 mm); mark each wrench's rack
position on its blocks. Judge with a protractor ring around the nut (60 degrees) and on the arm pivot (85 degrees),
a ruler on the rack marks, and the video for the 20 s hold and any touch. Replace a rounded nut before the next
trial.

## Relevance and related tasks

Industrial machinery mechanics repair and maintain equipment
([O*NET 49-9041](https://www.onetonline.org/link/details/49-9041.00)). Correct tool fit protects fastener corners;
the manufacturer's open-end wrench design discusses corner-rounding prevention
([Snap-on hand-tool catalogue](https://www.snapon.com/display/901/WWW/Downloads/Files/HandTools/Catalogs/Hand_Tools_Part_3.pdf)).
This task combines close metric/imperial fit selection, irreversible clearance-sensitive rounding, a clamp release
followed by hands-off proof, and returning the tools. Existing Robocurve wrench tasks include the pipe-union backup
wrench TK-CW7VYS, the click-torque wrench TK-NP6A85 and the confined ratchet TK-GMVZWK; ordinary wrench use alone
is not a novelty claim. LIBERO, SimplerEnv and RoboCasa include drawer, knob and articulated-object tasks; the
task lists we inspected do not include choosing a close-fitting tool for an unmarked fastener where a wrong fit
causes permanent damage.

## Third-party assets and licenses

The SDK is yam-isaac-lab-arena at 0059d02a1a91bc6834c1d0846022891df0c78730, the program's official dependency,
installed in the user's environment. Nothing from it is redistributed here except its i2rt MIT notice
(`LICENSE.i2rt`). Licenses are given as each source states them.

| Asset | Used for | Source | License as stated by the source | Bundled |
|---|---|---|---|---|
| YAM arm and gripper (URDF, meshes, collision hulls, converted USD) | both arms | SDK `yam_arena/assets/yam/`, from i2rt (https://github.com/i2rt-robotics/i2rt, SOURCE_COMMIT 120c3c81400171174604e503943f8d1ebc891058) | MIT, Copyright (c) I2RT Robotics | notice only (`LICENSE.i2rt`) |
| Station floor and table | room | SDK `station.py` / `scene.py`: boxes with flat colours generated in code; no mesh or texture file loaded | SDK README: "This repository has no license yet" (Placeholder); it is mostly ported from YAM-inspect-Isaac (https://github.com/ttpierson/YAM-inspect-Isaac), which has none | no |
| Top-camera stand (foot, mast, arm, housing) | room | SDK station code: boxes and capsules generated in code | as the row above | no |
| Dome light and sphere lamp | lighting | SDK `scene.py`, using Isaac Lab-Arena's dome light and an Isaac Lab sphere light | as the row above | no |
| top_cam, left_cam, right_cam | policy RGB | SDK pinhole camera sensors at the calibrated poses; no camera mesh | as the row above | no |
| Station object library (fruit, bowl, plate, bottle) | not used: this task's own objects replace them | - | - | no |
| Latch fixture, nut, swing arm, rack and five wrenches | task objects | this folder (`fixture.json`, `assets/*.usda`, `task.py`, `kit.py`) | the authors' own procedural geometry, MIT (`LICENSE`) | yes |

Isaac Lab-Arena release/0.3.1 (c7edf3a9d5d8276565208f72422f8f8a0806d6cf) ships the Apache License 2.0 in its
LICENSE.md. Isaac Sim and Isaac Lab are runtime dependencies under their own licenses. None of them is
redistributed here. Everything in this folder other than `LICENSE.i2rt` (fixture, nut, arm, rack and wrench
geometry and mechanics, scorer, demo, probe) is the authors' own work, released under the MIT License (`LICENSE`);
it uses no third-party models or textures.
