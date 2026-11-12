# Refit a loaded drawer on its roller slides, carried in above riding height so the rear hooks clear, then pull-test it

A small wooden drawer cabinet stands on the table with its drawer taken out. The drawer, loaded with a 0.8 kg box of
parts, stands on the table between the arms and the cabinet. The robot must put the drawer back on its slides, close
it, and then pull it open again by about 5 cm to show that it runs.

The cabinet's slides are roller slides. Each side wall carries a steel rail with a small roller at its front end;
each side of the drawer carries a steel runner with an orange hook plate under its rear half. A seated drawer rides
on the rollers (under the runners) and on the rails (under the hook plates). To put it in, the hooks must pass over
the rollers, so the loaded drawer has to be held level, about a centimetre or two higher than where it will ride,
carried in until the hooks are past the rollers, and set down. Pulled out, the hooks stop the drawer against the
rollers after 75 mm, so a seated drawer cannot be pulled out by accident.

The common mistake is to push the drawer in low: it then slides along the cabinet floor under the slides and also
closes, but it is not on its slides. The instruction only names the goal ("on its slides", then "pull it open about
5 cm"); it does not say how to get past the hooks or how to tell the two cases apart.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()`, `score()` and the scripted `demo()` |
| `drawerkit.py` | Rigid parts built from several boxes (free or fixed, each box with its own friction) for yam_arena task files |
| `record_demo.py` | Plays `demo()` through the same embodiment and scorer, prints the drawer's state and records MP4s |

`drawerkit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task
runs with the standard `yam_arena` embodiment and `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the drawer back into the cabinet on its slides and close it. Then pull it open again by about 5 cm and leave it there." \
  --scorer task_score --max-steps 1600 --store-frames --log-dir ~/yam-eval/drawer-refit-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

## Scripted demo

The same command with `--policy yam_scripted` plays `demo()`: open loop, planned only from the fixed start layout;
it never reads the simulator's state. To record it:

```bash
python record_demo.py task.py demo_out                                  # full score: demo_*_cam.mp4, setup.png
DEMO_VARIANT=floor python record_demo.py task.py out --no-record        # push it in low, under the slides
DEMO_VARIANT=short python record_demo.py task.py out --no-record        # lift too little: hooks meet the rollers
DEMO_VARIANT=insert_only python record_demo.py task.py out --no-record  # seat it and stop
DEMO_VARIANT=close_only python record_demo.py task.py out --no-record   # seat it, close it, stop
DEMO_VARIANT=pull:12 python record_demo.py task.py out --no-record      # pull 12 cm instead of 5
DEMO_VARIANT=floor,pull:12 python record_demo.py task.py out --no-record  # variants combine with commas
```

1. Both arms grip the drawer's side walls from above at its centre of mass (fingers closing across the wall) and
   lift it level.
2. They carry it into the opening with the hook plates above the rollers until the hooks are 15 mm past them, set it
   down on the rails 6 cm short of closed, and let go.
3. Closed fingers push the front panel until it is shut; the arms back off and wait.
4. Each arm grips the top edge of the front panel, pulls it 5 cm toward the robot, lets go and moves away.

The demo reaches 1.0 (success) at step 566 (28 s), both through `inspect-robots run --policy yam_scripted` and in
`record_demo.py`.

## Scoring

Evaluated every control step; the state at the end of the episode is scored (`SCORE_OVER = "end"`).

- **On its slides**: the drawer bottom 66 +/- 4 mm above the table (measured at the front panel's foot), level
  within 2.5 degrees, square to the cabinet within 3 degrees and centred within 12 mm, and open no more than 78 mm
  (it stops at 75 mm when the hooks meet the rollers). Pushed in under the slides the drawer sits at 12 mm.
- **Closed**: the front panel within 5 mm of the cabinet face. **Hands off**: every point of both grippers at least
  2 cm from the drawer and the box.
- 0 if the box ever leaves the drawer or the drawer falls off the table.
- 1.0 (the trial ends as a success once this has held for 1 s): earlier the drawer was closed on its slides with hands off for 1 s; now it is on its
  slides, open 30 to 80 mm, the box inside, hands off.
- 0.5: it was closed on its slides with hands off for 1 s, and it is still on its slides.
- 0.25: it is on its slides.
- Otherwise 0.

All of it can be checked with a ruler on the video and a photo of the end.

Measured in simulation (scores as printed by the live scorer; `record_demo.py` with the given `DEMO_VARIANT`):

| Strategy | Score |
|---|---|
| Scripted demo (`inspect-robots run --policy yam_scripted`) | 1.0 (success at step 566) |
| Do nothing (1,600 steps, `inspect-robots run`) | 0 |
| Push it in low (`floor`): lifted just over the cabinet floor, it slides in under the slides, is pushed shut (8 mm from the face) and pulled out 33 mm | 0: front panel foot at 12 mm, not on its slides |
| Lift too little (`short`): the hooks meet the rollers, the drawer jams, tips up and falls back onto the table | 0 |
| Seat it on its slides and stop (`insert_only`) | 0.25 |
| Seat it, close it, let go and stop (`close_only`) | 0.5 |
| Full demo but pull 12 cm (`pull:12`): the drawer stops at its hooks | 1.0 (75 mm open is inside 30 to 80 mm) |

Pulling the drawer open without having closed it on its slides first scores at most 0.25.

## How the hidden property can be observed

Once the drawer is in, the cameras cannot see whether its hooks and runners sit on the rails: they are behind the
drawer's side walls. Two things show it:

- the height of the front panel: on its slides its foot is 66 mm above the table, under the slides 12 mm;
- a pull test: pulled by its front panel, a drawer on its slides runs out level at the same height and stops when
  the hooks meet the rollers; a drawer under the slides drags out along the floor with no stop.

Measured with `record_demo.py` (it prints the drawer's state every 20 steps: `height_mm` is the front panel's foot
above the table, `opened_mm` the distance from closed, `tilt` in degrees), after the drawer was put in and closed:

| Drawer state | Closed: front panel foot above the table | Pulled 5 cm by the front panel | Pulled 12 cm by the front panel |
|---|---|---|---|
| On its slides (demo, `pull:12`) | 66.0 mm | runs out 49.6 mm, still 66.0 mm high, tilt 0.0 deg | stops at 75.0 mm (hooks on the rollers), 66.0 mm high, tilt 0.0 deg |
| Under the slides (`floor`, `floor,pull:12`) | 12.0 mm | drags out 33.0 mm at 12.0 mm | no stop: comes out 107.4 mm and its front drops to the table (0.2 mm, tilt 5.4 deg) |

```bash
DEMO_VARIANT=pull:12 python record_demo.py task.py probe --no-record
DEMO_VARIANT=floor python record_demo.py task.py probe --no-record
DEMO_VARIANT=floor,pull:12 python record_demo.py task.py probe --no-record
```

## Runtime and seeds

About 30 s of simulated time for the demo; a full 1,600-step trial takes about 2 minutes on an RTX A6000 plus about
1.5 minutes for Isaac Sim to start. The layout is fixed (`-E object_jitter=0`); there are no seeds to choose.

## Fidelity

All parts are rigid boxes. The rollers and ball bearings of real roller slides are modelled as low-friction (0.05)
sliding contacts and a fixed 6 mm step at the front of each rail; there is no self-closing spring or soft-close
damper. The drawer (1.0 kg) and its load (0.8 kg) are lighter than a typical full kitchen drawer so that two YAM arms
(27 N m at the large joints, 7 N m at the wrist, about 40 N per finger) can carry it level by its side walls. The
cabinet has no worktop, so the arms can follow the drawer into the opening from above.

## Real-world build

A plywood drawer box 300 mm wide, 202 mm deep and 90 mm tall with an 18 mm front 390 x 100 mm; a 40 cm wide, 26 cm
deep carcass without a top. Bottom-mount roller slides (cabinet member with a front roller, drawer member with a rear
hook) fixed 90 mm above the bottom of the carcass, or the equivalent built from aluminium angle and a 6 mm step.
A box of parts weighing 0.8 kg in the drawer. Mark the drawer's start position on the table with tape.
