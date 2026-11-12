# Set a beaker of water on a ring stand without tipping the stand

A laboratory ring stand stands in front of the arms: a light aluminium base plate with a rod near its right end, and a
support ring with a square of wire gauze clamped to the rod 7 cm above the table. The ring's clamp is snug, so the ring
turns around the rod under a light push and stays where it is left. At the start the ring points straight at the
robot and sticks out 12 cm past the front edge of the base. A 400 mL beaker of water stands on the table to the left.
The robot must set the beaker on the gauze and leave the stand standing.

Ring stands hold beakers and flasks over burners and under funnels in every teaching and research lab, and a stand loaded off its base is a classic cause of lab spills. The instruction only
names the goal. Loaded where it starts, the ring overhangs the base and the stand tips over as soon as the beaker is let
go, so the robot has to see the hazard in the layout and first turn the ring over the base. The task tests reasoning
about statics from the scene and setting up a fixture before using it, not just a pick and place.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()`, `score()` and the scripted `demo()` |
| `standkit.py` | A loose rigid part built from boxes and cylinders with one swinging arm on a damped hinge, for yam_arena task files |
| `record_demo.py` | Plays `demo()` through the same embodiment and scorer and records it to MP4 |

`standkit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task runs
with the standard `yam_arena` embodiment and `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Set the beaker of water on the wire gauze of the ring stand, and leave the stand standing." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/ring-stand-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

## Scripted demo

The same command with `--policy yam_scripted` plays `demo()` (open loop, planned only from the fixed start layout).
To record it:

```bash
python record_demo.py task.py demo_out                               # full score: demo_*_cam.mp4, setup.png, result.json
DEMO_VARIANT=no_swing python record_demo.py task.py out --no-record   # load the ring where it starts
DEMO_VARIANT=tip python record_demo.py task.py out --no-record        # press the ring down and knock the stand over
DEMO_VARIANT=swing_only python record_demo.py task.py out --no-record # turn the ring and stop
DEMO_VARIANT=swing:30 python record_demo.py task.py out --no-record   # turn the ring only 30 degrees, then load it
```

1. The left arm closes its fingers, lowers them beside the gauze's edge and pushes the ring round the rod, 92 degrees
   clockwise seen from above, until it points along the base plate. It backs off and lifts.
2. It picks up the beaker, carries it over the gauze, lowers it onto the gauze and opens.
3. It slides straight back toward itself (the open fingers pass either side of the beaker) and lifts away.

The demo reaches 1.0 (success) in 516 steps (26 s) through `inspect-robots run`.

## Scoring

Evaluated every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`).

- 0 if at any time the stand tilts more than 5 degrees or its base slides more than 3 cm, or the beaker tilts more
  than 30 degrees (spilled). This holds even if the stand rocks back upright afterwards.
- 1.0 (success, which must hold for 2 s): the beaker stands upright (within 10 degrees) on the gauze, its centre within
  3 cm of the ring's centre; the stand is upright (within 3 degrees); and every point on both grippers is at least
  2 cm from the beaker and the stand, so nothing is holding them up.
- 0.5: the ring has been turned at least 45 degrees about the rod from where it started, and the beaker has been lifted
  3 cm off the table.
- 0.25: the ring has been turned at least 45 degrees.
- Otherwise 0.

All of it can be judged from the video and a photo of the end with a ruler and a protractor. Measured in simulation:

| Strategy | Score |
|---|---|
| Scripted demo | 1.0 (success at step 516) |
| Do nothing (1,200 steps) | 0 |
| Load the ring where it starts | 0: the stand tips and the beaker falls to the table |
| Press the overhanging ring down | 0: the stand tips |
| Turn the ring only, leave the beaker | 0.25 |
| Turn the ring 30 degrees, then load it | 0: the stand tips |
| Turn the ring 45 degrees, then load it | 1.0: it stays up |

Holding the stand down while loading it does not help: once the grippers let go, the stand tips, and while they hold
it the hands-off condition fails.

## Runtime and seeds

About 1 minute per trial for the full 1,200 steps on an RTX A6000, plus about 1.5 minutes for Isaac Sim to start. The
layout is fixed (`-E object_jitter=0`); there are no seeds to choose.

## Fidelity

The parts are rigid bodies built from boxes and cylinders. The stand (base plate and rod) is one free body; the ring,
its bar, clamp and gauze are a second body on a hinge about the rod, with light rotational damping (0.3 N m s/rad) and
no spring, standing in for a snug thumbscrew; it cannot slip down the rod. The beaker and its water are one rigid
cylinder (no sloshing), so a tilt past 30 degrees counts as a spill. The base (0.62 kg) is lighter than a usual
cast-iron one (1.5 to 3 kg), so that a 0.48 kg beaker (a 400 mL beaker, the largest that fits between the YAM's
open fingers) tips it. With the beaker on the ring where it starts, the load's overturning moment about the base's
front edge is about 0.70 N m against 0.43 N m holding the stand up; with the ring turned over the base there is none.
Friction under the base is 0.6.
Glass breakage is not simulated.

## Real-world build

A small ring stand with a 24 x 12 cm aluminium base and a 12 mm rod 30 cm tall, a 10 cm support ring on a bar with a
boss clamp, and a 12.5 cm square of wire gauze. Tighten the clamp's thumbscrew until the ring stays where it is put
but turns with a light push, with the bar 7 cm above the table and pointing at the robot. A 400 mL Griffin beaker holds
350 mL of water. Mark the base and the beaker's spot on the table with tape.
