# Sink the 8-ball in the coin-marked pocket of a 7 ft bar table: one cue stroke, cut angle and speed judged, no scratch

Scripted reference 1.0 in 542 control steps. Dream-1 (seed 0) scored 0.0 at the 1,500-step limit. Shaft contact moved the cue ball, recorded zero legal strokes and scored 0.0. In a separate two-tap trial the cue retracted to a measured 35.87 mm tip-centre clearance after the first contact. The second tip contact registered a double hit and scored 0.0.

The task is the Isaac Lab-Arena (PhysX) `yam_arena` task in `isaac/`; `src/` holds the engine-independent modules it
imports (table layout and start states, scorer, fast IK) and `tests/` their unit checks.

## The task
A 7 ft bar-box pool table (78 x 39 in playing field) stands with its foot rail at the robot. On the cloth are the cue
ball and the 8-ball; a brass coin on the rail marks the called pocket (the left or the right side pocket). A 48 in short
cue lies in a two-hook cue rest beside the robot, pointing at the table. The robot picks the cue up with both hands,
guides the shaft 0.28 m from the tip with one gripper as the bridge (a loop: one finger under the shaft, one over
it, the shaft sliding between them) and pinches the shaft 0.57 m from the tip with the other, which drives the stroke,
lines the cue up behind the cue ball on the line that sends the 8-ball into the marked pocket, and strikes the
cue ball once at a suitable speed. The 8-ball must drop in the marked pocket while the cue ball stays on the table.

The judgement is closed-loop: the called pocket (from the coin), the ball positions, the cut angle (6-30 deg, either
side) and the distances change every trial, so the cue line (the ghost-ball line), the hand roles (the hand on the
side the shot goes bridges, so the arms never cross) and the stroke speed all have to be worked out from the scene.
Every positive level requires one legal leather-tip stroke and subsequent cue-ball contact with the 8-ball. Shaft or butt contact with the cue ball fouls the trial.

Why it matters: pool is one of the most widely played bar and home games; a clip of a robot lining up and sinking the
8-ball is recognisable to almost anyone.

## Scoring (`src/scoring.py`, primary score 0..1, value at the end of the trial)
- 1.0 (ends the trial): the 8-ball has dropped in the called pocket and the cue ball is still on the table, both at
  rest for 1 s.
- 0.5: the 8-ball rests on the table within one ball width of the called pocket's mouth (its edge within 57 mm of the
  line between the two cushion jaw points).
- 0.25: the cue ball struck the 8-ball, but neither of the above.
- 0: no hit; scratch (the cue ball dropped or left the table); the 8-ball in the other pocket or off the table; any foul:
  a second cue-tip contact on the cue ball, a push (tip on the ball for more than 60 ms), the cue touching the 8-ball,
  or the robot touching a ball.
Real trial: use the synchronized 240 fps contact views described below (which pocket, the cue ball on the table, one tip contact, no hand on a
ball); the 0.5 band is measured with a ruler from the jaw line.

## Dream-1 evaluation command

From the task's `isaac/` directory in the Isaac Sim 6.1 and Isaac Lab-Arena environment, run:

```bash
instruction=$(python -c 'import json; print(json.load(open("../task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
    -P model=dreamlabs/dream-1-beta -P rows_per_request=10 \
    -E task=task.py -E object_jitter=0 -E device=cpu -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
    --scorer task_score --seed 0 --max-steps 1500 --store-frames --log-dir ./dream-output
```

## Run and score a trial
Inside the Isaac Sim 6.1 + Isaac Lab-Arena container, working directory `isaac/` (`isaac/run_frozen.sh` does this on a
frozen copy of `isaac/`, `src/` and `task.json`, writes `source-sha256.txt` and logs to `<out>/<run>/run.log`; see its
header for the environment variables it needs):

```
# official Inspect Robots run of the scripted reference demo (registered closed-loop policy pool_8ball_scripted)
python official_run.py -- run --embodiment yam_arena --policy pool_8ball_scripted -E task=task.py -E object_jitter=0 \
    -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 \
    --instruction "$(python -c 'import json;print(json.load(open("../task.json"))["instruction"])')" \
    --scorer task_score --max-steps 1500 --store-frames --log-dir <out>/official
inspect-robots video <out>/official/<log>.json --out <out>/official/video     # H.264 video of every camera

# plain do-nothing run (task.py demo() gives the yam_scripted policy no waypoints: both arms hold READY)
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py ... --max-steps 1500

# direct demo / negative stages (trace.jsonl, result.json, demo_<camera>.mp4, setup.png, final.png)
python demo.py task.py <out>/<run> --stage demo --seed 1415158438   # the default inspect-robots start state
python demo.py task.py <out>/<run> --stage demo --seed 42           # another seeded start state
python demo.py task.py <out>/<run> --stage neg-misaim       # cue line 6 deg off the ghost-ball line: misses
python demo.py task.py <out>/<run> --stage neg-soft         # right line, 35 % of the planned speed: stops short
python demo.py task.py <out>/<run> --stage neg-full-ball    # shortcut: straight at the 8-ball's centre (no cut)
python demo.py task.py <out>/<run> --stage neg-shaft        # pass beside the ball and sweep the shaft into it
python demo.py task.py <out>/<run> --stage neg-double       # tap, retract clear of the ball, then tap it again
POOL_REPLAY=<run>/trace.jsonl python demo.py task.py <out>/<run2> --stage replay --seed 7  # open-loop replay
```
Unit checks (no simulator): `python tests/test_scoring.py`.
Tiled evidence video (top + left wrist + right wrist, same timing):
`isaac/tile.sh <out_dir> <name> demo_top_cam.mp4 demo_left_cam.mp4 demo_right_cam.mp4`.

## Start states
Every trial reset with a seed (every `inspect-robots run` passes one) draws, from the seed: the called pocket (left or
right side pocket, about 45 / 55 %: 135 / 165 of seeds 0-299), the 8-ball
position (x 0.75-1.30 m), the cut angle (up to 30 deg, either side) and the cue ball's distance to the ghost-ball spot
(0.15-0.45 m). Layouts are rejected unless the robot can reach the address, the cue clears the arm pedestals and the
camera post, the 8-ball's path enters the side pocket at 35 deg or more to the rail, a straight full-ball hit would
NOT pot the 8-ball, and the default seed's recorded stroke would not drive the new cue ball into the 8-ball. The
default inspect-robots seed gives: right side pocket, cue ball (0.722, -0.178), 8-ball (0.950, -0.303), cut 29 deg.
Unseeded resets (direct `demo.py` runs without `--seed`) cycle through three named setups (`layout.NAMED`; `POOL_SETUP`
picks the first and, when set, forces named setups): 0 = left pocket, cue ball (0.742, 0.182), 8-ball (0.966, 0.312);
1 = right pocket, cue ball (0.695, -0.237), 8-ball (0.987, -0.369); 2 = the default seed's layout; real trials use these, with the ball spots marked by paper rings.
Run time: about 30 s of robot time per trial (about 550 control steps at 20 Hz); run limit 1500 steps.

## Original submission only — Reference demo results (frozen source, CPU PhysX)
- Default inspect-robots seed, official Inspect Robots run: 1.0 (success at step 542 of 1500); the 542 executed
  actions equal the policy's commands (no guard clamping).
- Direct demo at the default seed, twice: 1.0 both times, identical command traces.
- Other starts (one process, four seeds): seed 42 (left pocket) 1.0, seed 1 1.0, seed 2 1.0, seed 0 0 (the stroke
  pushed the cue ball: tip contact longer than 60 ms, a foul). The scripted stroke is sensitive to the arm's dynamic
  lag; a policy that watches the wrist cameras can correct it.
- Negatives at the default seed: cue line 6 deg off 0.25 (8-ball hit, missed); stroke at 35 % speed 0 (cue ball stops
  short); straight full-ball hit (the shortcut) 0.25; doing nothing 0. Open-loop replay of the default-seed commands at
  seed 7: 0 (the recorded stroke passes the moved cue ball: no tip contact).

## Recorded media for this revision

The media package contains `model_run.mp4` (Dream-1), `official_tiled_top_left_right.mp4` (scripted reference) and `setup_overhead.png` (the original overhead frame before the first reference action). The videos show overhead, left wrist and right wrist at 20 Hz in a single 960 × 176 frame.

## Fidelity (full statement)
- Engine: Isaac Sim 6.1 / Isaac Lab-Arena `yam_arena`, PhysX on the CPU, physics 480 Hz, control 20 Hz, cameras
  320 x 176. All parts are full size.
- Table: a generic 7 ft bar-box table built to the dimensions in this task: playing field 78 x 39 in (1.981 x 0.991 m), 1 in slate, cushion nose 36.3 mm above the cloth
  (63.5 % of the ball diameter, K-66-style section), cushion tops 30 in above the floor, corner / side pocket mouths
  4.6 / 5.1 in with 142 / 104 deg jaw facings, pocket holes 120 / 127 mm, liners 13 cm deep. The cushions are rigid
  convex prisms (no rubber compression); a restitution of 0.75 stands in for the rubber. The robot stands at the foot
  rail; the station's work table is removed (it would be inside the pool table): the arm bases sit on two pedestals
  whose plates reach over the foot rail, 8.5 cm above the cloth, and the overhead camera is on its own post. The slate is
  a set of coplanar convex cells around the six holes; cushions, rails, liners, pedestals and the cue rest are kinematic
  bodies (contact forces readable). The cabinet, apron and legs are cosmetic (no collision).
- Balls: regulation 2 1/4 in (57.15 mm), 170 g (6 oz) free spheres with solid-sphere inertia; the cue ball is white
  with a small red dot and the 8-ball black with white spots (they show the spin). They never sleep.
- Cloth: PhysX has no rolling resistance, so every physics step a torque against each resting ball's rolling, of
  constant size 1.4 mu_r m g R (mu_r = 0.010), is applied, which slows a rolling ball at 0.098 m/s^2, and a small
  torque damps spin about the vertical. These are applied torques (no velocity is written). Sliding friction ball on
  cloth 0.20 (a struck ball skids, then rolls at 5/7 of its speed), as in the physics literature.
- Contact constants (static = dynamic friction; PhysX averages the two materials): ball-ball friction 0.06,
  restitution 0.93; ball-cushion 0.20 / 0.75; ball-cloth 0.20 / 0.50; cue tip on ball 0.60 / 0.75; cue shaft on the
  gripper pads 0.50. The bounce threshold is lowered from Isaac Lab's 0.5 m/s to 0.02 m/s so that slow ball impacts
  stay elastic. Values are literature-typical estimates, not measurements of a particular table.
- Cue: 48 in (1.219 m) two-piece short cue (the bars' "tight spot" cue), 425 g (15 oz), 13 mm tip, 21 mm at the joint,
  28 mm butt, balance point 400 mm from the butt; one rigid body with three convex colliders (tip, shaft, butt). No shaft flex, squirt or
  tip compression is modelled; the tip-ball impulse comes from PhysX contact with the restitution above.
- Robot technique in the reference demo: both hands lift the cue level out of its rest, the hand on the shot's side
  makes a loop bridge (one finger under the shaft, one over it), the other pinches the shaft 0.57 m from the tip; the
  cue is carried to the address with closed-loop correction on its measured pose (yaw only), the butt raised 7 deg to
  clear the foot rail; after a 6 cm backswing the demo re-aims, then strokes at a speed planned from the distances.
  The demo reads poses from the simulator (allowed for scripted demos); a learned policy has the camera images.
- Contact rules read force matrices by collider identity every physics step (cue on cue ball, cue ball on 8-ball,
  cue on 8-ball, robot links on a ball), cue–ball contact points are transformed to the cue frame to distinguish leather tip from shaft or butt. Pocketed = ball centre below the cloth inside a
  pocket liner.
- Nothing welds, snaps or teleports a ball or the cue during a trial; the scripted demo only sends the 14 joint /
  gripper commands and reads poses. On an environment reset (between trials only) the kit places the balls, the cue
  and the coin.

## Recording the shot and sourcing the table

Mark the leather tip and the ferrule boundary in different colours. Place a calibrated side camera close to cue-ball height and a synchronized oblique overhead camera covering the predicted contact point and ball path. Use 240 fps or faster with exposure short enough to resolve separation. At 240 fps the 60 ms push threshold spans 14.4 frame intervals. Bracket first and last contact by timestamp. Two contacts separated by visible daylight count as two strikes. At 480 Hz the scorer starts a new strike on contact after any complete physics sample below or equal to the contact threshold (0.02 N). A continuous contact is a push only when it exceeds 60 ms. Repeat with a clearer view or higher frame rate if a possible separation falls between frames, the 60 ms threshold lies inside the timing uncertainty, or the tip is occluded. A trial with unresolved contacts cannot establish a legal single stroke. A shaft or butt contact with the cue ball fouls the shot. Award half credit only after a legal tip contact, a subsequent object-ball hit and the 8-ball coming to rest at the called mouth (<2 mm/s).

A used, locally collected seven-foot Valley coin-op table is one low-cost sourcing route. Captain’s Auction Warehouse recorded a $250 winning bid for lot 316. At the published 15% on-site premium and 8% sales tax, that bid comes to $310.50 before collection or repairs ($318.60 at the 18% online premium). For a $350 all-in table budget, a $250 on-site bid leaves $39.50 for collection and repairs. Count those costs before setting the bid limit. Before purchasing, measure the nose-to-nose playing surface, both jaw points of the called pocket, nose height and slate drop with the dimensions below and in `src/layout.py`. Reject a table whose mouths cannot be matched by adjusting or replacing facings. The auction establishes a historical price, not current availability. [Auctioneer's sale record](https://bid.captainsauctionwarehouse.com/lot-details/index/catalog/33/lot/8936/POOL-TABLE-7-VALLEY-SLATE-TOP-COIN-OP)

For temporary access, The Break Room publishes a seven-foot table rate of $10/hour ($40 for four hours). Arrange access for the arm stands and cameras and measure the bed and the called pocket before booking. Record the table ID, jaw positions and arm-base marks so a later run can use the same station. The advertised rate covers table time only. [Venue rates](https://www.breakroomscv.com/billiards)

## Real rig
- Table: any 7 ft bar-box (coin-op or home) pool table with a 78 x 39 in playing field and WPA-range pockets (corner
  mouth about 4.5-4.6 in, side about 5.0-5.1 in), cushion tops 29.25-31 in above the floor. No particular brand or model
  is required or claimed.
- Balls: a regulation 2 1/4 in set (156-170 g); only the cue ball and the 8-ball are used. Coin-op tables with an
  oversize or magnetic cue ball should use a standard cue ball for this task.
- Cue: a 48 in two-piece short cue (sold for tight spots, e.g. 14-16 oz, 13 mm tip), chalked.
- Robot: the two YAM arms on pedestals (or a stand) at the foot rail, arm bases about 8.5 cm above the cloth, 61 cm
  apart, centred on the table's long axis; the overhead camera on a post behind them.
- Cue rest: two padded hooks on floor posts behind the foot rail holding the cue level 13 cm above the cloth, pointing
  at the table, its tip turned 12.5 deg toward the called pocket, tip 47 cm in from the foot cushion (two cable hooks
  on posts); a mirrored second rest serves the other side pocket (the cue lies in the one away from the called pocket).
- Marker: a coin on the rail top beside the called pocket. Ball spots: paper reinforcement rings at the setup's
  positions, removed before the trial.

## Third-party assets and licenses
| Asset | Source | License |
|---|---|---|
| YAM arm model (URDF, meshes) | i2rt, via the Isaac Lab-Arena YAM package | MIT |
| Station scene, cameras, `yam_arena` embodiment | Isaac Lab-Arena YAM package | as distributed with the package (no license file at the pinned revision) |
| Isaac Sim 6.1 / Isaac Lab-Arena | NVIDIA | NVIDIA Isaac Sim license / Apache-2.0 |
| Pool table, balls, cue, cue rest, coin geometry | generated in code by this task (`src/layout.py`, `isaac/poolkit.py`) | MIT (this repository, `LICENSE`) |
| `src/fastik.py` (YAM URDF kinematics) | this task's authors | MIT |
