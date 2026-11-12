# Bench can opener: one hand holds the sprung cutter lever down while the other cranks; the cut itself is invisible

A bench can opener is fixed to the table in front of the arms, with a food can already standing in its seat. Its
cutter lever, sprung up about 20 degrees, points toward the robot on the left of the can; its crank, with a wooden
knob that spins freely, is on the right. A wooden tray stands to the front right. The robot must hold the lever down
so the cutter bites the lid, turn the crank clockwise until the can has turned once all the way round under the
cutter, let the lever up, then lift the cut lid off the can and put it in the tray.

Commercial kitchens open cans this way all day. The task needs two arms
doing different things at once: one keeps a steady push on a sprung lever while the other moves a knob round a
circle, turn after turn, against the crank's resistance. Let go of the lever and the cutter lifts and stops cutting;
turn the crank backwards and nothing is cut; and the lid stays fixed to the can until the whole rim has been cut. The
cut itself cannot be seen, so progress has to be judged from the can turning: a white stripe on its label faces the
robot at the start and comes back after one full turn of the can (three crank turns). The
freed lid then has to be picked off the can past the cutter head and carried to the tray without hitting the crank.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, the cut, `success()` and `score()` |
| `hingekit.py` | Rigid parts built from boxes and cylinders with hinged links (springs, dampers), and a releasable fixed joint, for yam_arena task files |
| `demo.py` | The scripted full-score demo: closed loop, run in the same embodiment with the same scorer, with video recording |

`hingekit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task runs
with the standard `yam_arena` embodiment and `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Open the can with the bench can opener and put the lid in the wooden tray." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/can-opener-dream
```

`task_score` prints the trial's score: 1.0 on success (held for 1 s), otherwise `score()` at the end of the episode.

## Scripted demo

The demo is closed loop, so it is not a `demo()` plan for `--policy yam_scripted` (that policy only plays a fixed
list of waypoints). `demo.py` runs it instead, in the same `yam_arena` embodiment with the same task file, start
layout (`object_jitter=0`, 20 Hz, 320 x 176 cameras) and limits as a trial. It sends 14 joint commands per step,
like a policy, and scores the last step with the same `task_score` scorer, which it prints. From this folder, in
the same environment:

```bash
python demo.py task.py demo_out                               # demo.mp4 (three cameras side by side), setup.png,
                                                              # actions.jsonl (commands sent), result.json
DEMO_VARIANT=no_hold python demo.py task.py out --no-record     # crank without holding the lever down
DEMO_VARIANT=no_crank python demo.py task.py out --no-record    # hold the lever, then try to lift the lid uncut
DEMO_VARIANT=backwards python demo.py task.py out --no-record   # hold the lever and crank anticlockwise
DEMO_VARIANT=half python demo.py task.py out --no-record        # stop when the can has turned half a turn
DEMO_VARIANT=nothing python demo.py task.py out --no-record     # both arms stay still
DEMO_VARIANT=knock python demo.py task.py out --no-record       # shove the can and lid, pry and pull the uncut lid
```

The demo uses the fixed start layout (where the lever's handle and the crank knob start) and, while it runs, the
poses of the can and the lid, which a camera can see. It reads no internal state of the cut, no forces and no
physical parameters.

1. The left arm brings its closed fingers onto the lever's handle and pushes it down along its arc, 5 degrees past
   the stop, so it keeps pressing.
2. The right arm grips the crank knob and, once the lever is down, moves it round the crank's circle clockwise at
   about one turn every 3 seconds. At every control step it measures how far the can has turned about its axis
   from the can's pose (what the white stripe shows). When the can has turned one full turn plus 15 degrees, it
   slows the crank to a stop.
3. The right arm lets go of the knob and rises; the left arm lets the lever rise under its fingers and moves away.
4. The right arm pinches the lid across its seam where it is measured to be, lifts it 6 cm, carries it toward the
   robot and then right to the tray, lowers it onto the tray floor and lets go.

Both grippers keep their fingers closing front to back, so their bodies don't meet while one presses the lever and
the other circles the knob next to it.

## Scoring

Evaluated every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`).

- 1.0 (success, held for 1 s): the lid lies on the tray floor, flat within 20 degrees and still, its centre inside
  the walls.
- 0.75: the cut lid was lifted at least 3 cm above where it sat on the can, but is not in the tray.
- 0.5: the lid was cut free (the whole rim passed under the cutter) but never lifted 3 cm.
- 0.25: at least half of the rim was cut.
- Otherwise 0.

The cut: the crank turns the can one third of a turn per crank turn. The rim is tracked in 1-degree steps; a step is
cut when it passes under the cutter while the lever is within 3 degrees of horizontal and the can turns clockwise
seen from above. Turning back over a cut part, or turning with the lever up, cuts nothing. When every step is cut,
the fixed joint holding the lid to the can is released.

Cheap strategies score 0 (measured with the demo variants in Isaac Sim): doing nothing for the whole 1,200 steps;
cranking a full turn of the can without holding the lever; holding the lever without cranking and then pulling at the
lid; holding the lever and cranking anticlockwise a full turn of the can; shoving the can and lid from the front and
side, prying under the seam and pulling the uncut lid (the lid did not move). Stopping once the can has turned half a
turn scores 0.25, as intended (half of the rim cut). The opener is fixed and the lid is held to the can until the
whole rim is cut, so the can can't be knocked over or the lid pried off.

## Runtime and seeds

The demo reaches 1.0 in about 720 steps (36 s simulated); it stops cranking when the can has turned about 375 to 385
degrees, after about 3.1 to 3.2 crank turns. A full 1,200-step trial takes about 2 minutes on a recent GPU plus about
1 to 2 minutes for Isaac Sim to start. The layout is fixed (`-E object_jitter=0`); there are no seeds to choose. The
demo is deterministic apart from small differences in the physics between machines, which change the exact step at
which it stops cranking.

## Fidelity

All parts are rigid bodies built from boxes and cylinders. The opener's base is fixed to the world; the can, lever,
crank and knob are hinged links of one articulation. The lever has a spring drive (2.1 N m per rad toward 22 degrees,
which with its weight rests at about 19 degrees and needs about 5 N at the handle to hold horizontal) and a stop at
horizontal. The crank has a damper of 0.1 N m per rad/s (about 0.2 N m at one turn every 3 s); the knob spins freely.
The gears are not modelled: a stiff drive turns the can to one third of the crank's angle. Metal cutting is not
simulated: the cut is tracked as described above and a fixed joint holds the lid to the can until the rim is cut;
then the lid rests loose on the can and can only be lifted once the lever is up, as the cutter head overlaps its
edge. The lever spring and crank effort are lighter than on a typical bench opener, so the YAM arms (27 N m shoulder
joints, 7 N m wrist joints) can hold and turn them.

## Real-world build

A commercial bench can opener (or any lever-and-crank opener mounted on a base) clamped to the table, a short food
can (about 68 mm wide and 75 mm tall) with a paper label marked with a vertical stripe, and a wooden tray 12 x 12 cm
inside with 2.5 cm walls taped to the table. A sharp real cutter needs more force than here; a softened lever spring
or an opener with a lighter action matches the simulated effort.
