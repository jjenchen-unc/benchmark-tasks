# Set a strap toggle anchor in drywall: keep both straps taut so the hidden channel stays flat while the cap goes flush

The task is the Isaac Lab-Arena (PhysX) `yam_arena` task in `isaac/`; `src/` holds the engine-independent modules it
imports (layout and anchor geometry, scorer, fast IK) and `tests/` their unit checks.

## The task
A 1/2 in hole has been drilled at a pencil cross in a 1/2 in gypsum board on a stud-bay mock-up. A strap toggle anchor
(1/4-20, for a 1/2 in hole; the TOGGLER SnapToggle BA type) lies on a wood block in front of it: a steel channel folded
in line with its two plastic straps, the straps joined at the end in a pull tab, and a plastic cap on the straps. The
robot sets it the way the maker's instructions describe:
1. push the folded channel end first through the hole;
2. let the channel swing across behind the board;
3. pull the straps until the channel bears on the back of the board;
4. keep the straps taut with one hand while the other slides the cap along them into the hole until its flange is
   flush on the board (the cap's teeth ratchet on the straps and keep it there);
5. bend the straps sideways at the cap until they snap off.

Why two hands, and why it is hard: the channel is behind the board, out of sight. Only strap tension holds it on the
back face. If the straps go slack while the cap slides, the cap's teeth drag the straps in with it and the channel
leaves the back face; the cap then ratchets into place on a loose anchor, and once the straps are snapped off the slack
can never be taken up. The cues the robot can see: the channel through the hole before it swings, the straps stopping
(going taut) when the channel bears, the cap flange against the board, the strap stubs at the cap.

Why it matters: setting hollow-wall anchors to mount shelves, grab bars, handrails, signs, dispensers and brackets on
drywall is routine work for general maintenance and repair workers and installers (see `task.json` context).

## Scoring (`src/scoring.py`, primary score 0..1, value at the end of the trial)
- 1.0 (ends the trial): the anchor is set and finished: channel through the hole and across it on the back face (both
  ends within 1 mm of the back face); cap flange no more than 1.5 mm proud of the board; the anchor snug (the ratchet
  holds the cap within 1 mm of the seated position, so the channel cannot move more than 1 mm off the back face);
  straps snapped off at the cap (stub no more than 2 mm) and the strap tail away from the hole; both hands 100 mm or
  more from the cap; still for 1 s.
- 0.6: set as above but the straps not snapped off.
- 0.3: channel through and across the hole within 3 mm of the back face, but the anchor not set (cap proud, or more
  than 1 mm of play).
- 0: otherwise (channel not through the hole, still folded, dangling in the cavity), or the board face crushed (real
  trial: the board face visibly dented or cracked at the robot's contact; sim: the robot pushed on the board with more
  than 40 N for 0.1 s).
Real trial: ruler at the cap flange (proud) and the stub; push on the cap with a finger (play, 1 mm); a light tug;
thread the 1/4-20 bolt through a fixture into the channel (it only catches on a channel that is across the hole).

## Dream-1 evaluation

Dream-1 scored 0.0 on seed 0 at the 2,400-step limit. Scripted demo: official Inspect Robots run 1.0 (default seed, 1,156 of 2,400 steps); direct runs 1.0 twice, identical; seed 7: 1.0; snap-off by rocking the pinched tab: 1.0. No-op: 0. Open-loop replay: 0. Straps left on: 0.6.

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
# official Inspect Robots run of the scripted reference demo (registered closed-loop policy snaptoggle_scripted)
python official_run.py -- run --embodiment yam_arena --policy snaptoggle_scripted -E task=task.py -E object_jitter=0 \
    -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 \
    --instruction "$(python -c 'import json;print(json.load(open("../task.json"))["instruction"])')" \
    --scorer task_score --max-steps 2400 --store-frames --log-dir <out>/official
inspect-robots video <out>/official/<log>.json --out <out>/official/video     # H.264 video of every camera

# plain do-nothing run (task.py demo() gives the yam_scripted policy no waypoints: both arms hold READY)
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py ... --max-steps 2400

# direct demo / negative stages (trace.jsonl, result.json, demo_<camera>.mp4, setup.png, final.png)
python demo.py task.py <out>/<run> --stage demo --seed 1415158438   # the official default start
python demo.py task.py <out>/<run> --stage demo                     # named setup 0 (SNAP_SETUP=k for setup k)
python demo.py task.py <out>/<run> --stage neg-no-tension           # straps let go before the cap slides: 0
python demo.py task.py <out>/<run> --stage neg-no-snap              # set, straps left on: 0.6
python demo.py task.py <out>/<run> --stage neg-early-pull           # straps pulled back before the channel is pushed fully through: 0
python demo.py task.py <out>/<run> --stage pinch-bend               # snap-off by rocking the pinched tab with the strap hand: 1.0
SNAP_REPLAY=<trace.jsonl> python demo.py task.py <out>/<run> --stage replay --seed 7   # open-loop replay
```
Unit checks (no simulator): `python tests/test_scoring.py`.
Tiled evidence video (top + left wrist + right wrist, same timing):
`isaac/tile.sh <out_dir> <name> demo_top_cam.mp4 demo_left_cam.mp4 demo_right_cam.mp4`.

## Start states
Every trial reset with a seed (every `inspect-robots run` passes one) draws, from the seed: the wall mock-up's lateral
position (hole centre y uniform in +-40 mm), its stand-off (board face x 395 +-10 mm) and the anchor's place on its
block (+-12 mm in x and y). The default inspect-robots seed (1415158438) gives hole y +16.4 mm, face x 385.7 mm, anchor
offset (+7.6, -7.5) mm. Unseeded resets (direct `demo.py` runs without `--seed`) use the named setups in turn
(`SNAP_SETUP=k` picks the first and, when set, forces named setups):

| setup | hole y (mm) | face x (mm) | anchor dx, dy (mm) |
|---|---|---|---|
| 0 | -30 | 395 | 0, 0 |
| 1 | +35 | 403 | +8, -6 |
| 2 | 0 | 387 | -8, +8 |
| 3 | -15 | 400 | +4, +10 |
| 4 | +25 | 391 | -10, -10 |

## Recorded media

`model_run.mp4` shows the full seed 0 Dream-1 trial (2,400 steps, final task score 0.0).

- `official_tiled_top_left_right.mp4`: the official Inspect Robots run of the scripted reference policy at the default
  seed, score 1.0 (H.264, 960 x 176).
- `setup_overhead_hires.png`: the default start at 1280 x 704 (board with the hole at the pencil cross, anchor on the
  block).

## Fidelity (full statement)
- Engine: Isaac Sim 6.1 / Isaac Lab-Arena `yam_arena`, PhysX on the CPU, physics 480 Hz, control 20 Hz, cameras
  320 x 176. All parts are full size.
- Anchor dimensions are approximate (from the product as remembered; the maker's drawing was not available): channel
  57.2 x 9.5 x 5.6 mm, 10 g; two straps 1.3 x 4 mm, 160 mm from the channel to the end of the pull tab; pull tab
  18 x 22 x 3.2 mm; cap flange 19 mm, collar 11.6 mm x 11.5 mm with a 2.5 mm x 45 deg lead-in on its nose. The
  channel collides as a box with 1 mm x 45 deg chamfers round both end faces (stamped ends); its U section is visual,
  and the straps lie inside the U when folded. Pads pinching the channel or the pull tab have a 4 mm torsional contact
  patch (flat pads resist twisting).
- Anchor mechanics: one PhysX articulation. The channel turns on a free pin at the straps (limits +-100 deg, tiny pin
  friction); its centre of mass is 3 mm off the pin so it swings across by gravity once through the hole (real straps
  are flexible and let the channel drop the same way). The cap slides on a prismatic joint; its teeth are modelled as a
  ratchet: a stiff joint drive (20 kN/m) anchored elasto-plastically, slipping at 2 N toward the channel and holding up
  to 80 N the other way. The cap can shift +-1.5 mm across the straps (two short spring-centred slides, 150 N/m:
  the slot clearance and the straps' flex). Within the stub (the straps from the channel to the cap face) the straps
  are rigid.
- Strap snap-off: the rest of the straps with the tab is a separate rigid body on a fixed joint at the break line,
  0.5 mm outside the seated cap face. The joint is removed (the straps snap) when the robot bends the tail with a moment
  of 0.15 N m or more for 50 ms once the cap is seated (its ratchet teeth hold the flange within 1.5 mm of the seated
  position, so its edge is at the break line and acts as the fulcrum); before that, bending does not snap them (real straps just flex; in sim the
  tail is rigid). Moment, per arm: the sideways part (perpendicular to the straps) of that arm's net contact force on
  the tail x the distance along the straps from the break line to its nearest fingertip (finger pose, not contact
  points). A pure pinch cancels in the net force; pinching the tab and rocking it (the maker's side-to-side method) or
  pushing the tail sideways both count. Real straps snap after one or two bends; the 0.15 N m threshold is an estimate.
- Board: 1/2 in gypsum board as kinematic convex pieces (48 sectors round the 12.7 mm hole), studs and plates as
  kinematic boxes; the whole mock-up is placed per trial. Friction 0.5 (gypsum, wood), 0.25 (zinc-plated steel),
  0.6 (plastic straps), 0.2 (nylon cap, taken as the minimum against any other surface); estimates. Board crush is a veto (robot on the board > 40 N for 0.1 s), not a material model.
- Nothing welds, snaps into place or teleports the anchor; the scripted demo only sends the 14 joint / gripper commands
  and reads poses and contact forces. The scorer only reads. On an environment reset (between trials only) the kit
  puts the anchor back on its block, re-joins the strap tail and places the mock-up.

## Real rig and replica
- Anchor: TOGGLER SnapToggle BA (1/4-20 bolt, 1/2 in hole, walls 3/8 to 3-5/8 in), sold with bolts; any strap toggle
  of this type works the same way.
- Wall: a 16 in on-centre stud-bay mock-up: two 2x4 studs (38 x 89 mm) between 2x4 plates, 300 mm tall, faced with a
  445 mm wide piece of 1/2 in gypsum board (e.g. USG Sheetrock 1/2 in). A fresh board (or a fresh area of it) per trial:
  drill the 1/2 in hole at a pencil cross 165 mm above the bench. The mock-up stands on the bench with its face 395 mm
  in front of the arms' base line (+-10 mm per trial) and its hole within +-40 mm of the arms' centre line.
- Staging block: a wood block 175 mm long, 30 mm wide: a front pad 75 mm high under the channel and a rear pad
  0.8 mm higher under the straps, lower between and behind them so the cap and the pull tab hang free.

## Third-party assets and licenses
| Asset | Source | License |
|---|---|---|
| YAM arm model (URDF, meshes) | i2rt, via the Isaac Lab-Arena YAM package | MIT |
| Station scene, cameras, `yam_arena` embodiment | Isaac Lab-Arena YAM package | as distributed with the package (no license file at the pinned revision) |
| Isaac Sim 6.1 / Isaac Lab-Arena | NVIDIA | NVIDIA Isaac Sim license / Apache-2.0 |
| Wall mock-up, anchor, block geometry | generated in code by this task (`src/layout.py`) | MIT (this repository, `LICENSE`) |
| `src/fastik.py` (YAM URDF kinematics) | this task's authors | MIT |
