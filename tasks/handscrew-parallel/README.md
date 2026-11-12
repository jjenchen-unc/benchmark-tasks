# Clamp a glue-up in a wooden handscrew

A 10-inch wooden handscrew lies flat on two wooden strips, one strip under each jaw. Between the strips, inside the
open jaws, a maple board and a cherry board stand on edge with their glued faces together. The jaws start parallel,
46 mm apart; the boards are 38 mm thick together. The robot is told only to clamp the boards in the handscrew and leave
the clamp holding them.

A handscrew has two screws. The middle spindle comes out of the left jaw, the end spindle out of the right jaw, and each
one sets the opening only at its own point along the jaws. Turn only the middle handle and the tips close first: they
pinch the front of the boards while the back of the joint gapes. Turn only the end handle and the jaws close behind the
boards and never touch them. Crank both handles equally hard and, because both screws sit behind the boards, the jaws
end out of parallel and press only the back edge of the joint. The jaws bear flat along the whole joint, with no gap at either end, only if the two
handles are turned together and each is finished by its own amount. This is how woodworkers set a handscrew. The
instruction gives none of it.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: the clamp, the boards and strips, the screw coupling, `success()` and `score()` |
| `jointkit.py` | Rigid parts built from boxes and cylinders, with hinged and sliding links and drives, for yam_arena task files |
| `demo.py` | The scripted full-score demo and the cheap strategies (`DEMO_VARIANT`) |

`jointkit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena. The task runs
with the standard `yam_arena` embodiment and the `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`), from this
folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Clamp the two glued boards together in the wooden handscrew and leave the clamp holding them." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/handscrew-dream
```

`task_score` prints the trial's score: 1.0 on success (held 1 s), otherwise `score()` at the end of the episode. The
layout is fixed (`object_jitter=0`), so no seed changes the start. A trial of 1,200 steps takes about 6 to 7 minutes
on one RTX A6000, including about 1 minute of start-up.

## Scripted demo

```bash
python demo.py task.py demo_out                          # full score: writes demo.mp4, setup.png, result.json
DEMO_VARIANT=middle_only python demo.py task.py out      # cheap strategies, see below
```

Each arm takes its handle from above, fingers closing across the handle 15 cm out from the middle of the opening, and
turns it by turning the wrist about the handle's axis. A wrist reaches about a quarter turn this way, so the hand
grips, turns, opens, turns back and grips again. The left arm turns the middle handle and the right arm the end handle,
both clockwise seen from the handle's end and in step, so the jaws stay parallel as they close. The jaws must close
8 mm and a turn closes them 6 mm, so both handles turn 480 degrees until both jaws meet the boards. Then the middle
handle turns 60 degrees more and the end handle stops 10 degrees short, so the jaws end parallel and bear flat along
the joint. Both hands let go and back away.

The demo plans from the fixed layout and the facts in the setup (where the handles are, the opening, the boards'
thickness, the screws' lead). While it runs it looks at two things a camera sees: the dark stripe on each handle,
which shows how far that handle has turned, and where the far jaw's handle is, so the right hand follows it as the jaw
closes. Each hand keeps turning until its stripe has gone round the planned angle, because the fingers slip a few
degrees per stroke. The demo never reads the clamp's joint angles, its forces or the task's internal state.

The demo reaches 1.0 in 887 steps (44 s), well inside the 1,200-step limit. The right hand's stripe reads 469 degrees
at step 668 and the left's 538 degrees at step 823. At the end the jaws are parallel to 0.01 degrees, every gap is
within 0.1 mm and the squeeze is 110 N.

## Scoring

Measured at the end with a 0.5 mm feeler gauge, a protractor, a ruler, a 4.5 kg weight and a photo:

- **Gaps**: between each jaw's inner face and the board against it, and between the two boards (the glue line), at the
  jaw's mid-height, at the boards' end nearest the middle spindle and at the jaw tips.
- **Held**: with the clamp lifted by its jaws and a 4.5 kg weight hung from the boards, neither board slips (ruler).
- **Touch**: somewhere along each jaw the gauge does not enter.

| Score | Condition |
|---|---|
| 0 | at any point the clamp was moved more than 3 cm or tilted more than 10 degrees, or a board tipped more than 30 degrees |
| 1.0 | jaws parallel within 0.5 degrees, both bearing flat on the boards and the glue line closed (the gauge enters at neither end of either jaw or of the glue line, and neither end of a jaw is pressed 0.5 mm or more into a board), boards held, upright (within 10 degrees) and flush (ends and top edges within 2 mm), both grippers at least 3 cm from every part of the clamp, boards and clamp still (moving less than 1 mm over the last second), all for 1 s |
| 0.5 | both jaws touch the boards and the boards are held and upright, but another 1.0 condition fails |
| 0.25 | both jaws touch the boards |
| 0 | otherwise |

The simulation computes the lift test from the modelled screws (see below): `task.py` keeps the squeeze each spindle puts
on the boards (how far the jaw is held open past where its screw sets it, times the jaw stiffness) and counts the boards
as held when the two together give at least 50 N (`MIN_SQUEEZE`). With friction 0.5 on both outer faces that carries
50 N, the boards (0.48 kg) plus a 4.6 kg weight. The jaw parallelism is the right jaw's angle to the left jaw; the gaps
are signed distances between the jaw faces and the board faces at the two ends of the overlap (`jaw_gaps`,
`joint_gaps`). The finger check uses points on both arms' finger pads against boxes around the jaws and handles.
Jaws closed behind the boards never touch them and score 0 (the `end_only` row below).

## Cheap strategies

Each run with `DEMO_VARIANT=<name> python demo.py task.py out`, same scene, robot, limits and scorer:

| Strategy | Score | What happens |
|---|---|---|
| `nothing`: both arms stay still | 0 | the jaws stay 4 mm off the boards |
| `middle_only`: only the middle handle, 540 degrees | 0.25 | the tips close first; jaws 1.4 degrees out of parallel, the back gapes 0.8 and 1.9 mm, and the two screws work against each other, so there is no net squeeze |
| `end_only`: only the end handle, 540 degrees | 0 | the jaws close behind the boards; the tips gape 19.6 mm |
| `equal_tight`: both handles together, 810 degrees each | 0.5 | the boards are held (200 N), but the jaws end 1.6 degrees out of parallel and press only the back edge of the joint |
| full demo | 1.0 | jaws parallel, flat on the boards, 110 N |

Turning one handle and then the other is not penalised by itself: what counts is how the jaws end. A robot that
closes the jaws canted can still recover by turning the other handle, as a woodworker would.

## The mechanism, and what is simplified

The far (right) jaw hangs from the clamp on a carriage that slides across the opening and a pivot about the vertical,
midway between the spindles. Every control step `task.py` turns the two handles' angles into the opening each screw
sets at its own point. Each handle turn, clockwise seen from its end, closes the jaws 6 mm there. From the two openings
it sets the carriage's and the pivot's drive targets. The drives are stiff, so each spindle gives 1 mm per 200 N, as
the wooden jaws and nuts would, and they slip at 300 N per spindle. The screws are self-locking: the jaw never turns a
handle back. Simplifications: threads are not modelled; the handles turn with the same small effort (damping
0.2 N m s/rad) however tight the clamp is, unlike a real screw; glue is not simulated (the board faces have friction
0.5). All bodies are rigid boxes and cylinders, at real size and mass: a 10-inch handscrew has 254 mm maple jaws about
44 mm square, a 127 mm (5 in) throat and weighs about 1.2 kg; the boards are 3/4 x 3 1/2 in stock.

The simulator reports a free joint's angle within one turn, so `task.py` counts whole turns of each handle itself.

Both arms are used: the middle handle points left, to the left arm, and the end handle points right, to the right arm.
The end handle rides on the moving jaw, so it moves as the jaws close. A reach check (inverse kinematics on the arm
model, the grasp across the handle used by the demo) found the left arm can also hold the end handle over about 60
degrees of wrist turn and the right arm the middle handle over only about 15 degrees, against about 90 degrees for each
arm on its own handle. One arm working both handles in turn is therefore not impossible, but it needs roughly twice the
strokes of the two-arm demo.

## Third-party assets and licenses

The SDK is yam-isaac-lab-arena at 0059d02a1a91bc6834c1d0846022891df0c78730, the program's official dependency,
installed by the user's environment. Nothing from it is redistributed here. Licenses are given as each source states
them.

| Asset | Used for | Source | License as stated by the source | Bundled |
|---|---|---|---|---|
| YAM arm and gripper (URDF, meshes, collision hulls, converted USD) | both arms | SDK `yam_arena/assets/yam/`, from i2rt (SOURCE_COMMIT 120c3c81400171174604e503943f8d1ebc891058) | MIT, Copyright (c) I2RT Robotics (SDK `LICENSE.i2rt`) | no |
| link2/link3 meshes split into black and white parts | station arm colours | SDK `yam_arena/assets/yam/dream/`, from the dreamlabs 1.0.23 wheel | "MIT, see ../LICENSE.i2rt" (SDK `dream/SOURCE`) | no |
| Station floor and table | room | SDK `station.py`/`scene.py`: boxes with flat colours generated in code; no mesh or texture file loaded | SDK README: "This repository has no license yet" (Placeholder) | no |
| Top-camera stand (foot, mast, arm, housing) | room | SDK `station.camera_stand`: boxes and capsules generated in code | as the row above | no |
| Sky dome light (uniform colour, no HDR image) and lamp | lighting | SDK `scene.py`, using Isaac Lab-Arena's DomeLight and an Isaac Lab sphere light | as the row above | no |
| top_cam, left_cam, right_cam | policy RGB | SDK pinhole camera sensors at the calibrated poses; no camera mesh | as the row above | no |
| Station poses, sizes and colours | numbers in the SDK code | SDK README: "Dream Labs' calibrated YAM station, from Dream's MuJoCo replica (dreamlabs 1.0.23, `yam_mujoco`) and the calibration it follows (yamlab at `ec0455d`)" | license not stated by the SDK | no |
| Station fruit, bowl, plate (Arena object library) and water bottle | not used: this task's own OBJECTS replace them | - | - | no |
| Handscrew, boards, strips | task objects | this folder (`task.py`, `jointkit.py`): boxes and cylinders generated in code | authors' own, MIT License (`LICENSE`) | yes |

Isaac Lab-Arena release/0.3.1 (c7edf3a9d5d8276565208f72422f8f8a0806d6cf) ships the Apache License 2.0 in its
LICENSE.md. Isaac Sim and Isaac Lab are runtime dependencies under their own licenses. None of them is redistributed
here. All task-specific code and geometry in this folder (`task.py`, `jointkit.py`, `demo.py`) is the authors' own,
under the MIT License in `LICENSE`.
