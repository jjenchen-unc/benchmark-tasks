# Find a compression latch's hidden grip length by trial, then lock it so the enclosure lid stays sealed

A grey steel equipment enclosure is fixed on the table in front of the two arms. Its lid lies on a rubber gasket
and is closed with an adjustable compression latch: a black wing handle that turns a cam, and a brass knob that
sets the latch's grip length (1 mm per turn). The pawl that clamps sits under the flange, so the right grip length
cannot be seen. At the start setting the locked lid still has a 2.5 mm gap; with the grip set too short the handle
jams part way and the cam is overloaded. The knob only turns while the latch is unlocked. The robot has to lock
the latch, see that the lid is not sealed, unlock it, turn the knob, lock it again, and let go.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects and the scorer (`score()`, `success()`) |
| `latch.py` | The enclosure and latch: loads `latch.usda`, runs the gasket, cam, threaded knob and cam stripping every 2 ms, and records the measured quantities for the scorer |
| `latch.usda` | Geometry, masses and joints of the enclosure and latch |
| `demo.py` | Scripted full-score demo (open loop) with the cheap-strategy variants and video recording |
| `validate.py` | Produces the grip-length table below |
| `LICENSE.i2rt` | License of the YAM arm model (see the last section) |

## Run a trial

In the Isaac Lab-Arena environment, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream -P model=dreamlabs/dream-1-beta -P rows_per_request=4 \
  -E task=task.py -E device=cpu -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$(python -c 'import json;print(json.load(open("task.json"))["instruction"])')" \
  --scorer task_score --max-steps 3848 --store-frames --log-dir out-dream
```

`task_score` gives 1.0 when the episode ends in success, otherwise `score()` at the last step. The layout is fixed
(`object_jitter=0`), so there are no seeds to choose. Physics runs on the CPU at 2 ms steps; the cameras need an RTX
GPU. A full 3,848-step trial takes about 20 to 25 minutes of simulation plus model time.

## Scoring

Everything the scorer uses can be checked on real hardware with a feeler-gauge set, a protractor (or a mark on
the handle plate at 175 degrees) and the video. No force or torque is measured.

- Held: the handle is at 175 degrees or more (its locked stop is 180) and no part of either robot touches the
  enclosure, lid, handle or knob.
- 1.0: the gap between lid and flange next to the latch stays under 1 mm (a 1 mm feeler gauge does not enter) for
  20 s in a row while held. The episode ends there with success.
- Otherwise, at the end of the episode: 0.5 if the gap has stayed under 2 mm for the last 20 s while held, 0.25 if
  it has stayed under 5.5 mm, else 0. The lid resting on the gasket alone shows 6.33 mm.
- Forcing the latch scores 0 for the rest of the episode: the handle turned past 172 degrees (the cam's over-centre
  point) while the knob is more than 2 turns clockwise from its start mark (counted from the knob's white mark on
  the video).
- A measurement that cannot be read scores 0.

The simulator checks these every 2 ms physics step, so a gripper still on the handle or pressing the lid at any
moment of the 20 s restarts the count. A stripped cam holds nothing, so the lid rises back to 6.33 mm and cannot
score.

## How the hidden property can be observed

The grip length shows as the gap after locking. `validate.py` runs the demo's motions with different numbers of
knob turns, then locks the handle, lets go and waits 20 s. Two columns are internal simulator values (read only
by `validate.py`) that the previous version of this task scored on: pawl clamp force above 20 N, and cam torque
above 3 N m for 0.1 s strips the cam. "Old score" applies that previous rule to the same run.

| Knob turns (measured) | Handle (deg) | Gap (mm) | Score | Pawl clamp (N) | Peak cam torque (N m) | Old score |
|---|---|---|---|---|---|---|
| 0.00 | 180 | 2.51 | 0.25 | 126 | 0.62 | 0.25 |
| 0.48 | 180 | 2.03 | 0.25 | 144 | 0.69 | 0.25 |
| 0.98 | 180 | 1.54 | 0.5 | 160 | 0.77 | 0.5 |
| 1.47 | 180 | 1.05 | 0.5 | 177 | 0.84 | 0.5 |
| 1.76 (demo) | 180 | 0.76 | 1.0 | 188 | 0.88 | 1.0 |
| 1.95 | 180 | 0.56 | 1.0 | 193 | 1.00 | 1.0 |
| 2.20 | jams at 32, springs back | 6.33 | 0 | 0 | 5.25 | 0 |
| 2.45 | jams at 32, springs back | 6.33 | 0 | 0 | 5.20 | 0 |
| 2.69 | jams at 39, springs back | 6.33 | 0 | 0 | 5.91 | 0 |
| 2.93 | jams at 32, springs back | 6.33 | 0 | 0 | 5.15 | 0 |

Each knob turn closes the locked gap by about 1 mm, so trying a setting and looking at the gap tells which way
and how far to turn. The new rule gives the same score as the old one in every run:

- Clamp: every locked latch clamps 126 N or more, and an unclamped lid rests at 6.33 mm, so "gap under 5.5 mm while
  locked" corresponds to the old "clamp above 20 N" (20 N of clamp corresponds to a gap of about 5.8 mm).
- Torque: settings up to 1.95 turns stay at or below 1.0 N m. From 2.2 turns the handle jams near 30 to 40
  degrees with brief torque peaks of 5.2 to 5.9 N m (above the old 3 N m limit, but each shorter than 0.1 s, so the
  cam did not strip) and the arm cannot turn it further. The 2-turn knob limit separates the two groups, so a latch
  forced past over-centre with a too-short grip scores 0 even if a stronger robot could do it.

Command used (A6000 GPU, CPU physics; the 2.25 to 3 settings in a second run):
`python validate.py task.py val 0 0.5 1 1.5 1.8 2`

## Scripted demo

```bash
python demo.py task.py demo_out                                   # demo.mp4, setup.png, actions.jsonl, result.json
DEMO_VARIANT=nothing python demo.py task.py out --no-record
DEMO_VARIANT=no_adjust python demo.py task.py out --no-record     # lock without adjusting: grip too long
DEMO_VARIANT=force python demo.py task.py out --no-record         # knob 3 turns, then force the handle: grip too short
DEMO_VARIANT=keep_holding python demo.py task.py out --no-record  # full demo, gripper stays on the handle
DEMO_VARIANT=press python demo.py task.py out --no-record         # press the lid down next to the latch and keep pressing
```

The demo is open loop from the fixed layout. Its joint targets are inverse-kinematics solutions for the left arm at
the fixed knob and handle positions. While it runs it reads nothing from the simulator: no part poses, latch
state, forces or physical parameters. It turns the knob 1.8 turns clockwise in regrasps of at most 150 degrees
(the knob slips slightly in the fingers and ends at 1.76 turns), turns the handle 180 degrees clockwise onto its
stop, lets go, moves the arm away and waits. It goes straight to a good setting instead of trying the start
setting first; the table above shows that the setting can be found from the gap.

Measured results (Isaac Sim, CPU physics):

| Run | Score | Steps |
|---|---|---|
| Demo (on an A6000 and on the evaluation machine's L40S) | 1.0 | 1,803 |
| Dream-1 (`dreamlabs/dream-1-beta`, the command above) | 0.0 | 3,848 (step limit) |
| `nothing` | 0 | 3,848 |
| `no_adjust` (grip too long; the 0-turn row of the table) | 0.25 | 805 |
| `force` (grip too short; the 3-turn row of the table, handle jams near 32 degrees) | 0 | 2,605 |
| `keep_holding` | 0 | 3,848 |
| `press` | 0 | 3,848 |

## Fidelity

All parts are rigid. The gasket is a spring (33.4 N/mm, eight times stiffer over its last 0.5 mm) on the lid's
straight vertical slide, not a deformable body, and the lid does not swing open. The cam and the threaded knob are
stiff compliant constraints computed every 2 ms; the knob lock is a stiff restraint while the pawl presses on the
flange; stripping removes the cam force until reset. The arms are the platform's standard YAM arms (27 N m
shoulder and 7 N m wrist joints, 50 N grippers), raised 150 mm on supports so they reach the latch.

## Real-world build

A steel wall-mount enclosure about 300 x 400 x 130 mm with a 7 mm sponge-rubber gasket and an adjustable-grip
compression latch (wing handle, grip-adjusting screw of about 1 mm per turn), fixed to the table. Paint a mark on
the knob and on the handle plate at 175 degrees. Judge with feeler gauges of 1 mm, 2 mm and 5.5 mm next to the
latch, and the video for the 20 s hold, the knob turns and robot contact.

## Third-party assets and licenses

- YAM arm and gripper model: from i2rt (https://github.com/i2rt-robotics/i2rt), MIT license; copy in
  `LICENSE.i2rt`. It is loaded from the installed SDK, not copied here.
- Station scene: loaded from the installed yam-isaac-lab-arena SDK
  (https://github.com/kairosiann/yam-isaac-lab-arena, revision 0059d02a1a91bc6834c1d0846022891df0c78730). For a
  task file its scene and station modules build, from primitive shapes and lights: the floor and table (boxes), a
  dome light and a sphere lamp, and the top-camera stand (a foot and a housing as boxes, a mast and an arm as capsules). No object-library
  models are spawned in this task. The SDK's README says the repository has no license yet ("Placeholder"): it is
  mostly ported from YAM-inspect-Isaac (https://github.com/ttpierson/YAM-inspect-Isaac), which has none. Only its
  YAM model carries a license (MIT, above).
- Everything in this folder other than `LICENSE.i2rt` (enclosure, gasket, lid and latch geometry and mechanics,
  scorer, demo) is the authors' own work, submitted with this task. It uses no third-party models or textures.
