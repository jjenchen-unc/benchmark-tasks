# Slide a monitor out from under a desk shelf to fit a clip-on light bar, then slide it back

A 21.5-inch desktop monitor stands pushed back under a free-standing pine desk shelf: there are only 4 cm between the
top edge of its screen and the underside of the shelf. A monitor light bar (a 45 cm bar on a clip whose counterweight
hangs behind the screen) lies on its back on the desk in front. The instruction is only the goal: "Fit the light bar
on top of the monitor and leave the monitor where it started."

Not stated: the clip's counterweight hangs 6.5 cm below the clip's top plate, so the bar cannot be put on while the
monitor is under the shelf. The monitor has to be slid forward out from under the shelf, the bar hung on its top edge
with the counterweight behind the screen, and the monitor slid back (the fitted bar stands only 2.8 cm above the edge,
so it passes under the shelf). Skipping this, the bar or the hands strike the shelf and the clip never seats.

The monitor is 2.67 kg on a narrow stand and tips forward if it is pulled high up; the 45 cm bar turns lengthwise in
a single grip near one end. Both arms are used: together they slide the monitor out and back by its side edges, and
they carry the bar one hand either side of the clip.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: shelf, monitor and light bar, `success()` and `score()` |
| `barkit.py` | Rigid parts built from several boxes (optionally tilted) and cylinders, for yam_arena task files |
| `demo.py` | The scripted full-score demo and the cheap-strategy variants |

`barkit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task runs
with the standard `yam_arena` embodiment and `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`), from this
folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fit the light bar on top of the monitor and leave the monitor where it started." \
  --scorer task_score --max-steps 1600 --store-frames --log-dir ~/yam-eval/light-bar-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.
`--policy yam_scripted` plays the task file's `demo()`, which holds both arms still (a "do nothing" check); the
full-score demo needs tilted grips that `yam_scripted` waypoints cannot express, so it is `demo.py`.

## Scripted demo

```bash
python demo.py task.py demo_out                        # full score: writes demo.mp4, setup.png, result.json
DEMO_VARIANT=naive python demo.py task.py naive        # hang the bar with the monitor still under the shelf
DEMO_VARIANT=no_return python demo.py task.py noret    # fit the bar on the pulled-out monitor and leave it there
DEMO_VARIANT=nothing python demo.py task.py nothing    # both arms stay still
```

1. Both arms come in low from the front, grip the screen's side edges 10.5 cm above the desk (near the bottom of
   the screen, so the monitor slides instead of tipping) and slide the monitor 14 cm toward the robot, out from under
   the shelf. They let go, look at where the monitor is and repeat the slide if it is more than 3 cm short.
2. Both arms pinch the bar 8 cm either side of the clip, lift it and turn it a quarter turn about its length so the
   clip hangs counterweight-down, and carry it over the top edge of the pulled-out monitor. Hovering there, they look
   at the bar's pose twice and shift so the clip's opening is centred on the edge, then lower the clip onto the edge
   (front lip in front, counterweight behind) and let go.
3. Both arms grip the side edges again and slide the monitor back to its start, repeating the slide until the
   monitor is within 5 mm of its start or three slides have been made; then they let go and move clear.

The demo reaches 1.0 (success) after 1,085 steps (54 s).

The demo uses only what a policy could see: the fixed layout in the first camera frame (the task's constants, with
`object_jitter=0`), the poses of the monitor and the light bar (both in plain view of the cameras) and its own arm
commands. It never reads the scorer's state, contacts or forces.

## Scoring

Evaluated every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`).

- 0 if at any time the monitor tipped more than 20 degrees from upright or the shelf moved more than 1 cm from its
  start (a struck shelf stays failed even if it settles back).
- 1.0 (success, which must hold for 1 s): the light bar sits on the monitor's top edge (the clip's top plate from
  3 mm below to 6 mm above the edge, the inside of the front lip within 7 mm in front of the screen's face, so the
  counterweight is behind the screen; the middle of the clip within 3 cm of the middle of the edge; the bar level
  within 5 degrees; the clip upright within 15 degrees; the bar running along the edge within about 25 degrees); the
  monitor is back within 2 cm of its start and turned less than 5 degrees; every finger pad of both arms is at least
  3 cm from the light bar and 2 cm from the monitor; and over the last second neither the bar nor the monitor moved
  more than 1 mm and both grippers stayed clear of them.
- 0.5: the bar sits on the edge in the same way, let go and still, but the monitor is not back (for example left
  pulled out).
- Otherwise 0: the bar on the desk or dropped, held in the hands, or resting on the edge on its front lip.

In a real trial: measure the monitor's stand against its taped outline and the shelf's end panels against theirs with
a ruler, and check the clip's seat, the bar's level and the counterweight's side with a protractor and front and side
photos.

## Results

Printed by the live scorer (`task_score`) in the yam_arena embodiment, fixed layout:

| Run | Score | Steps |
|---|---|---|
| Scripted demo (`demo.py`) | 1.0 (success) | 1,085 |
| Dream-1 (`dreamlabs/dream-1-beta`), command above | 0.0 (the bar, the monitor and the shelf ended where they started) | 1,600 |
| `DEMO_VARIANT=naive`: hang the bar with the monitor still under the shelf | 0 (the bar strikes the shelf, shelf moved 30 mm, bar falls) | 623 |
| `DEMO_VARIANT=no_return`: fit the bar on the pulled-out monitor and leave it there | 0.5 | 910 |
| `DEMO_VARIANT=nothing`: both arms stay still | 0 | 1,240 |

## Runtime and seeds

About 10 to 15 minutes of wall time per 1,600-step trial on one A6000 GPU (the simulation runs slower than real time), plus about 1 minute for Isaac Sim to start. The layout is fixed (`-E object_jitter=0`); there are no seeds to choose.

## Fidelity

Rigid bodies built from boxes and one cylinder at real size and mass (monitor 50.4 x 38.4 x 17.9 cm, 2.67 kg; light
bar 45 cm, 0.53 kg; shelf 4.6 kg); every shape is defined in `task.py`. The monitor's tilt hinge is not modelled: the
screen is fixed upright on its stand. The clip is a rigid U with a fixed 2.1 cm opening around the 1.6 cm edge and no
spring, so the fitted bar is held on the edge by gravity and its counterweight behind the screen, not by a clamping
force. The bar's centre of mass is set where its counterweight puts it (9 mm behind the screen's face). A bar resting
on the edge on its front lip or with the counterweight in front sits at least 11 mm too high or is turned around, and
scores 0. No contact-force cap is enforced; the shelf is free and moves if struck.

## Real-world build

A 21.5-inch monitor on its fixed stand (Dell SE2222H: 50.4 x 38.4 x 17.9 cm with stand, 2.67 kg), a clip-on monitor
light bar (BenQ ScreenBar: 45 cm bar, 0.53 kg, clip for 1-3 cm thick screens), and a pine desk shelf 120 x 25 cm on
two end panels with 42.4 cm clear underneath (about 4.6 kg). Mark the start outlines of the monitor's stand, the
shelf's end panels and the light bar on the desk with tape.

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
| Desk shelf, monitor, light bar | task objects | this folder (`task.py`, `barkit.py`): boxes and one cylinder generated in code | authors' own, MIT License (`LICENSE`) | yes |

Isaac Lab-Arena release/0.3.1 (c7edf3a9d5d8276565208f72422f8f8a0806d6cf) ships the Apache License 2.0 in its
LICENSE.md. Isaac Sim and Isaac Lab are runtime dependencies under their own licenses. None of them is redistributed
here. All task-specific code and geometry in this folder (`task.py`, `barkit.py`, `demo.py`) is the authors' own,
under the MIT License in `LICENSE`.
