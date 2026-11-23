# Hang a parts bin on a louvered panel: hook its hidden back lip into the same louver row as the bin beside it

The task is the Isaac Lab-Arena (PhysX) `yam_arena` task in `isaac/`; `src/` holds the engine-independent modules it
imports (layout and geometry, hang state and scorer, fast IK) and `tests/` their unit checks.

## The task
A steel louvered panel (simulated 36 x 19 in panel, six rows, based on Akro-Mils 30636) is screwed to a plywood board standing at
the back of the bench, louvers toward the robot. One AkroBin (Akro-Mils 30230, 10-7/8 x 5-1/2 x 5 in, red) already
hangs on one of its rows, in one half of the panel. An identical bin (blue) stands on the bench below the panel's empty
half. The robot pinches the blue bin's side walls with both hands, lifts it, brings its back to the panel with the
hanger lip above the louver it wants, moves it back until the bin's back wall meets the louvers, lowers it so the lip
drops behind the louver, and lets go. The bin must hang on the same louver row as the red bin, in the empty half.

Why it matters: louvered panels with hook-on bins are standard storage at assembly benches, in stockrooms and in
maintenance shops; restocking and rearranging them is routine work for stockers, order fillers and assemblers.

What makes it hard:
- The hook is hidden. The hanger lip is at the top of the bin's back, behind the bin; it must go behind a louver's
  front edge, which stands 10 mm proud of the panel, into an 8.5 mm channel (the lip is 3.5 mm thick). The robot sees
  the louver rows and the panel face, not the lip.
- Success shows only on letting go: nothing latches. A bin whose lip is set on the louver's top edge instead of behind
  it tips forward and drops when released; a bin brought in too low hits the louver edge with its lip.
- Two hands: the bin is wider (140 mm) than the gripper opens (94 mm), so each hand pinches one side wall by its rim
  and the two carry it level; the hang height is near the top of the arms' reach.
- The row is a judgement: six identical louver rows; the target is the row the red bin hangs on (row 2, 3 or 4 from
  the bottom, per trial), read from the red bin's hanger height. A bin on the row above or below scores 0.

## Scoring (`src/scoring.py`, primary score 0..1, value at the end of the trial)
- 1.0 (ends the trial): the blue bin hangs (its hanger lip down in the louver channel along its full width, the bin
  level within 5 deg) on the same row as the red bin, in the panel's other half, not overlapping it; the red bin still
  hangs on its row; every fingertip 5 cm or more from both bins; both bins still for 2 s.
- 0.5: the same, but a hand is still on or within 5 cm of a bin at the end (not let go).
- 0 otherwise: on another row, lip resting on the louver edge or fallen, the red bin knocked off its row, or the two
  bins overlapping.
Real trial: the row from a photo of the panel (count louvers, or compare the two bins' tops); level from the photo; let
go and wait 2 s; a bin not hooked drops.

## Dream-1 evaluation

Run from `isaac/` in the configured YAM environment. The seed-0 trial recorded 1,600 executed actions, each with 14 values.

```bash
inspect-robots run --embodiment yam_arena --policy dream \
    -P model=dreamlabs/dream-1-beta -P rows_per_request=10 \
    -E task=task.py -E object_jitter=0 -E device=cpu -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 \
    --instruction 'Hang the loose bin on the panel next to the other bin, on the same louver row, and let go.' --scorer task_score --seed 0 --max-steps 1600 --store-frames --log-dir ./dream-output
```

## Run and score a trial
Inside the Isaac Sim 6.1 + Isaac Lab-Arena container, working directory `isaac/` (`isaac/run_frozen.sh` does this on a
frozen copy of `isaac/`, `src/` and `task.json`, writes `source-sha256.txt` and logs to `<out>/<run>/run.log`; see its
header for the environment variables it needs):

```
# official Inspect Robots run of the scripted reference demo (registered closed-loop policy louver_bin_scripted)
python official_run.py -- run --embodiment yam_arena --policy louver_bin_scripted -E task=task.py -E object_jitter=0 \
    -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 \
    --instruction "$(python -c 'import json;print(json.load(open("../task.json"))["instruction"])')" \
    --scorer task_score --max-steps 1600 --store-frames --log-dir <out>/official
inspect-robots video <out>/official/<log>.json --out <out>/official/video     # H.264 video of every camera

# plain do-nothing run (task.py demo() gives the yam_scripted policy no waypoints: both arms hold READY)
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py ... --max-steps 1600

# direct demo / negative stages (trace.jsonl, result.json, demo_<camera>.mp4, setup.png, final.png)
python demo.py task.py <out>/<run> --stage demo --seed 1415158438   # the default inspect-robots seed (row 3)
python demo.py task.py <out>/<run> --stage neg-on-lip --seed 1415158438    # lip set on the louver edge, let go: 0
python demo.py task.py <out>/<run> --stage neg-wrong-row --seed 1415158438 # hung properly one row higher: 0
python demo.py task.py <out>/<run> --stage neg-hold --seed 1415158438      # hung, never let go: 0.5
LB_REPLAY=<run>/trace.jsonl python demo.py task.py <out>/<run2> --stage replay --seed 3942   # open-loop replay
```
Unit checks (no simulator): `python -m pytest tests/test_scoring.py`.
Tiled evidence video (top + left wrist + right wrist, same timing):
`isaac/tile.sh <out_dir> <name> top.mp4 left_wrist.mp4 right_wrist.mp4`.

## Submission media

- `model_run.mp4` shows the complete Dream-1 trial at seed 0, score 0.0 in 1,600 steps.
- `scripted_run.mp4` shows the official reference at seed 0, score 1.0 in 585 steps. Both videos show overhead, left wrist and right wrist views at 20 Hz.
- `setup_overhead_hires.png` shows the proposed setup from the overhead camera.

## Start states
Every trial reset with a seed (every `inspect-robots run` passes one) draws the red bin's row (2, 3 or 4 from the
bottom), its half (left or right), its sideways place in that half (up to 15 mm about a place 140 mm from the panel's
centre line), the panel's place (x 5 to 25 mm either way of nominal; y up to 15 mm), and the blue bin's place on the
bench below the empty half (x and y up to 20 mm, yaw up to 10 deg). For every seed but the default, the panel is never
within 10 mm (x) of the default seed's panel place, and the blue bin's x relative to the panel is never within 8 mm of
the default seed's (exclusion bands, so that the default run's action trace replayed open loop, or mirrored to the
other half, misses the louver at every other start). The default
inspect-robots seed gives row 3 with the red bin in the right half. Unseeded resets (direct `demo.py` runs) use the
named setups 0-2 in turn (`src/layout.py` SETUPS; `LB_SETUP` picks the first and, when set, also forces named setups on
seeded resets).

## Fidelity (full statement)
- Engine: Isaac Sim 6.1 / Isaac Lab-Arena `yam_arena`, PhysX on the CPU, physics 240 Hz, control 20 Hz, cameras
  320 x 176. All parts are full size.
- Bins: Akro-Mils AkroBin 30230 outline (276 x 140 x 127 mm, sloped side walls, 65 mm hopper front) built from convex
  pieces (bottom, back, front, side walls with a rim bead, hanger shelf and tab), 2.5 mm walls, polypropylene friction
  0.35. The hanger is modelled as a full-width shelf on the back wall's top with a 3.5 mm tab dropping 12 mm behind the
  back face; mass 0.27 kg (estimate). The real bin's hanger is moulded with ribs; the modelled shelf and tab keep its
  function (a lip that hooks behind a louver).
- Panel: 914 x 483 mm steel sheet with side returns on a 3/4 in plywood board (one kinematic body). Six louver rows on
  a 3 in (76.2 mm) pitch (estimate). Each louver is modelled as a lip 10 mm proud of the panel face (1.5 mm steel) with
  an 8.5 mm channel 14 mm deep behind it, open at the top; real louvers are stamped hoods of the same function.
- Bins hang, tip and fall by contact and gravity only: no joints, no computed forces, no welds, no snapping. A bin set
  with its lip on the louver's top edge tips forward and drops off when released (checked in the simulator).
- Contact offset 0.5 mm, rest offset 0; bodies never sleep. Friction (static = dynamic): bins 0.35, steel 0.35,
  gripper pads the station's own material (PhysX averages the pair).
- Nothing teleports a bin during a trial; the scripted demo only sends the 14 joint / gripper commands and reads poses.
  The scorer only reads. On an environment reset (between trials only) the kit puts the panel, the red bin (hung) and
  the blue bin (on the bench) at the next start state.
- Grip: the reference demo closes its fingers on the 2.5 mm side wall (the finger drive stops on the wall); the
  measured contact force between a gripper and the bin peaks at 17.4 N in the official run (held while carrying),
  under the YAM's 40 N per finger. At seed 11 (row 4) a finger landing on the rim gave one single-physics-step contact
  spike of 162 N, and the cross-reaching arm brushed the hanging bin once (33 N, it stayed hung).
- The bins are empty (real bins hold parts; a few hundred grams more would not change the hook geometry).

O*NET includes stocking bins and racks among stocker duties. This task models rehanging a removable parts bin during replenishment. The occupation data cover all storage systems. [O*NET, Stockers and Order Fillers](https://www.onetonline.org/link/summary/53-7065.00)

## Real rig

Use the Akro-Mils 30636 steel panel and two empty 30230 bins, red and blue. The manufacturer lists the panel at 35.75 × 19 in and each bin at 10.875 × 5.5 × 5 in. The compatibility guide shows nine rows of twelve stamped louvers on this panel and pairs it with the 30230. The simulated panel has six rows on a 3 in pitch. [Panel specifications](https://akro-mils.com/product/louvered-wall-system-30636/) · [Bin specifications](https://akro-mils.com/product/akrobins-plastic-storage-bin-30230/) · [Manufacturer compatibility guide](https://akro-mils.com/wp-content/uploads/dlm_uploads/2024/12/Louvered-Panel-Bin-Combination-Guide.pdf)

Mount the panel to a 1000 × 540 mm plywood board on two angle feet, with its bottom 10 mm above the bench. For each trial, choose the physical row nearest the simulated support height measured from the bench, and record its height and row number counted from the bottom. Hang the red bin on that row. Set the blue bin in the opposite half with its back 85 mm from the panel. Judge the final placement against the actual row supporting the red bin. The tab and channel clearances and empty-bin mass still require physical measurement.

## Third-party assets and licenses
| Asset | Source | License |
|---|---|---|
| YAM arm URDF and meshes | [I2RT Robotics model source](https://github.com/i2rt-robotics/i2rt/tree/120c3c81400171174604e503943f8d1ebc891058) | MIT. The installed runtime records the model commit in `yam_arena/assets/yam/SOURCE_COMMIT`. |
| Station, cameras and YAM embodiment | [YAM Arena runtime used on Brev](https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730#license) | Installed dependency. Its README reports that the integration code has no declared license. |
| Isaac Sim 6.1 and Isaac Lab-Arena | NVIDIA and [Isaac Lab-Arena](https://github.com/isaac-sim/IsaacLab-Arena/blob/c7edf3a9d5d8276565208f72422f8f8a0806d6cf/LICENSE.md) | NVIDIA Isaac Sim license and Apache-2.0 for the Arena package, respectively. |
| Panel, louvers, board and bin geometry | generated in code by this task (`src/layout.py`, `isaac/louverkit.py`) | MIT (this repository, `LICENSE`) |
| `src/fastik.py` (YAM URDF kinematics) | this task's authors | MIT |
