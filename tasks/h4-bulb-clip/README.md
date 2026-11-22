# Change an H4 headlamp bulb from behind the lamp: tabs into notches by eye, glass untouched, then hook its wire clip

The task is the Isaac Lab-Arena (PhysX) `yam_arena` task in `isaac/`;
`src/` holds the engine-independent modules it imports (layout and part geometry, scorer, fast IK) and `tests/` their
unit checks.

## The task
The rear of a 7 in round H4 headlamp sits in an engine-bay mock-up on the table: the lamp's back shell (178 mm) with
its bulb boss facing the robot, the radiator support panel around it, a battery box to the left rear and the inner
fender apron to the right. The old bulb is already out, the plug is off and the rubber dust boot is off (both go back on
after the task). The boss has a 20.6 mm bore with a locating rim round it; three notches in the rim (one wide, two
narrow, unequally spaced) take the three tabs of the bulb's reference flange, so the bulb fits one way only. A U-shaped
wire spring clip hangs open on its hinge to the left of the bore. The new bulb (H4, 12 V 60/55 W, P43t-38 cap, e.g.
Philips 12342) stands glass-down in an orange foam holder on the table, its flange resting on the holder's rim, turned
to a different angle every trial.

The robot must take the bulb by its cap (never the glass: skin oil on a halogen envelope shortens its life and a
fingerprint is cleaned off with alcohol before fitting), turn it so the tabs face their notches, push it home so the
flange sits flat in the rim, keep it there while the clip is brought round, and hook the clip's end under its catch.
The cue for the turn is visual: the tab pattern round the flange (top and right wrist cameras) and the notch pattern
round the bore. A bulb with a tab on the rim stands 1 mm proud and can turn; the clip will not then reach its catch.

Two hands: one hand picks the bulb by its cap at the grasp angle that the tab pattern calls for and seats it; the other
swings the U-shaped clip round its hinge and pushes its end past the catch's spring (the catch snaps over only under a
push of roughly 8-12 N at the clip's end, measured in simulation; a light push does not hook it). In this model a
seated, unclipped bulb that is let go cleanly may tip slightly in its rim under its own weight (flange up to about 0.5 mm back,
under 1.5 deg) with its tabs still in their notches, and the closing clip pushes it flat again; a bulb disturbed while
letting go (tab out of its notch) is not seated and fails. The reference demo lets go of the bulb before the clip is
swung, because the U clip sweeps round the cap and would hit a hand that grips the cap. Both arms finish to the
sides, so the overhead camera's last frame shows the closed clip.

## Scoring (`src/scoring.py`, primary score 0..1, value at the end of the trial)
- 0 for the whole trial if at any time the robot touched the glass envelope (any contact force between a robot link
  and the glass, a separate collider), or the bulb was dropped (moving down faster than 0.3 m/s, the speed after a
  4.6 mm free fall, with no robot contact).
- 0.25: bulb seated (below), clip open.
- 0.5: bulb seated and the clip swung over the flange (within 6 deg of closed) but not hooked under its catch, or hooked
  with a hand still within 30 mm of the bulb or the clip.
- 1.0 (ends the trial): bulb seated, clip hooked under the catch (bail within 1.2 deg of closed, catch back over the
  wire), both hands 30 mm or more from the bulb and the clip, bulb still for 1 s.
- Otherwise 0 (bulb in its holder, in a hand, cocked on a tab or out).
Seated = the flange's front face within 0.4 mm of the boss seat, tilted less than 1 deg, roll within 4 deg of the
fitted roll; only possible with all three tabs down in their notches. Real trial: the flange is flush with the rim all
round and the bulb cannot be turned by hand (seated); the clip's end is under the catch and a light pull on it does not
free it (hooked); the video shows the hands never touched the glass and the bulb never fell.

## Dream-1 evaluation

The Inspect Robots run used model `dreamlabs/dream-1-beta` and returned a task score of 0.0 at the 2,400-step limit. The scripted reference run scored 1.0.
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
frozen copy of `isaac/`, `src/` and `task.json`; see its header for the environment variables it needs):

```
# official Inspect Robots run of the scripted reference demo (registered closed-loop policy h4_bulb_scripted)
python official_run.py -- run --embodiment yam_arena --policy h4_bulb_scripted -E task=task.py -E object_jitter=0 \
    -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 \
    --instruction "$(python -c 'import json;print(json.load(open("../task.json"))["instruction"])')" \
    --scorer task_score --max-steps 2400 --store-frames --log-dir <out>/official
inspect-robots video <out>/official/<log>.json --out <out>/official/video     # H.264 video of every camera
python action_check.py <out>/official/policy_log.json                         # commanded vs guard-clamped actions

# plain do-nothing run (task.py demo() gives the yam_scripted policy no waypoints: both arms hold READY)
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py ... --max-steps 2400

# direct demo / negative stages (trace.jsonl, result.json, demo_<camera>.mp4, setup.png, final.png)
python demo.py task.py <out>/<run> --stage demo            # named setup 0; H4_SETUP=k for setup k
python demo.py task.py <out>/<run> --stage demo --seed 7   # a seeded start state, as in inspect-robots runs
python demo.py task.py <out>/<run> --stage hold --seed 1415158438 --max-steps 2400   # full-length do-nothing: 0
python demo.py task.py <out>/<run> --stage neg-noroll      # grasp at a fixed angle, ignoring the tabs: tab on the rim, 0
python demo.py task.py <out>/<run> --stage neg-unhooked    # clip swung but not pushed past its catch
H4_REPLAY=<demo run>/trace.jsonl python demo.py task.py <out>/<run> --stage replay --seed 7   # open-loop replay: 0
python negcheck.py negcheck.json <run> [<run> ...]         # stimulus + reaction check
```
Unit checks (no simulator): `python tests/test_scoring.py`.

Reference demo coverage: the scripted demo reaches 1.0 at the default seed (two direct runs and the official CLI run)
and at seed 7 (direct run, one trial per process). The left hand swings the clip one way round
to 40 deg, sets down again the other way round for the rest and the press, and sets down again if it slips off the
wire.

## Recorded media

- `model_run.mp4` shows the full Dream-1 trial (seed 0, 2,400 steps, final task score 0.0). The three views run at 20 Hz in order from left to right: overhead, left wrist, right wrist.
- `official_tiled_top_left_right.mp4`: the official Inspect Robots run of the scripted reference policy at the default
  seed, score 1.0 after 1,220 control steps (H.264, 960 x 176).
- `setup_overhead_hires.png`: the start state at 1280 x 704 (default seed).

## Fidelity (full statement)
- Engine: Isaac Sim 6.1 / Isaac Lab-Arena `yam_arena`, PhysX on the CPU, physics 480 Hz, control 20 Hz, cameras
  320 x 176. All parts are full size.
- Sources: the H4 type, the P43t-38 cap and the three unequal tabs (one fit only) are those of the named bulb. The
  individual dimensions are representative values (the IEC 60061 cap sheet and the maker's drawing were not available
  to us): reference flange disc 37 mm x 1.0 mm with tabs to a 43 mm circle (one 10 mm wide, two 6 mm, at 90 / 250 /
  335 deg; the real angles and widths differ but the pattern is likewise asymmetric), collar 20 mm x 6 mm, glass
  12.5 mm reaching 48 mm in front of the flange, cap body 24 mm x 15 mm, three 6.3 x 0.8 mm blades, 18 g. Lamp boss:
  20.6 mm bore (0.3 mm radial play on the collar, 0.8 mm lead-in), locating rim 37.8 mm bore (0.4 mm radial play on the flange disc, 0.8 mm lead-in), 1.0 mm high, notches
  with 0.5 mm play a side on each tab. To check on a real lamp and bulb before trials.
- Bulb: the cap (flange, tabs, collar, cap body, blades; 14 g, centre of mass 4 mm behind the flange face) is one free rigid body of convex pieces; the glass
  envelope (4 g) is a second rigid body held to it by a fixed joint (no contact between the two), so robot contact
  with the glass is read by collider identity.
- Lamp, rim, engine-bay mock-up (radiator support, battery box, fender apron) and the holder are kinematic rigid bodies:
  they do not move during a trial and PhysX reports the contact forces on them. The engine-bay parts are lower than in
  a real bay (battery box top 100 mm, apron top 110 mm above the table) so the arms reach over them.
- Wire clip: one rigid body (1.6 mm wire, legs 30 mm apart, 52 mm from hinge to bight) on a revolute joint with hard
  limits; its mass is raised from about 3 g to 12 g and its angular speed capped at 360 deg/s, so the thin wire cannot
  tunnel through the 1 mm flange (solver settings; a hand-swung clip moves far slower), and swept (CCD) contact is on
  for the clip and the catch (scene CCD enabled; the run logs show `enable_ccd=True`).
  Hinge drag and friction (0.004 N m) are torques computed every physics step (drives on non-articulation joints do
  not act in this engine build). The wire's bending preload on the flange is not modelled: hooked, the rigid bail holds
  the flange with up to 0.2 mm play.
- Catch: a 10 g plastic latch on a prismatic joint (3 mm travel, hard limits), pushed toward its rest stop by a spring
  force computed every physics step (4 N preload + 1.6 N/mm; estimate). Its 45 deg ramp is cammed aside by the bight
  and snaps back over it. Real H4 clips usually hook under a fixed tab with a press-and-slide; the sprung catch stands
  in for that sideways spring of the wire. Hooking needs a real push on the bight (several newtons), not a touch.
- Friction (static = dynamic): metal cap and wire 0.4 / 0.3, glass 0.3, catch 0.3, foam holder 0.6; PhysX averages with
  the gripper pads' material. Estimates. Contact offset 0.1 mm, rest offset 0.
- Vetoes from contact forces read every physics step: any robot-link force on the glass body (0.05 N); a downward bulb
  speed above 0.3 m/s with no robot contact. Contact rules use forces and collider identity, never contact positions.
- Nothing welds, snaps or teleports a part; the scripted demo only sends the 14 joint / gripper commands and reads poses.
  The scorer only reads. On an environment reset (between trials only) the kit puts the bulb back in its holder, the
  clip at its start angle and the catch at rest.
- The reference demo grips the cap with about 10 N per finger (fingers commanded 2 mm inside the cap's faces against the
  5000 N/m finger drive), far under the YAM's 40 N.

## Third-party assets and licenses
| Asset | Source | License |
|---|---|---|
| YAM arm model (URDF, meshes) | i2rt, via the Isaac Lab-Arena YAM package | MIT |
| Station scene, cameras, `yam_arena` embodiment | Isaac Lab-Arena YAM package | as distributed with the package (no license file at the pinned revision) |
| Isaac Sim 6.1 / Isaac Lab-Arena | NVIDIA | NVIDIA Isaac Sim license / Apache-2.0 |
| Headlamp, bulb, clip, catch, holder and engine-bay geometry | generated in code by this task (`src/layout.py`) | MIT (this repository, `LICENSE`) |
| `src/fastik.py` (YAM URDF kinematics) | this task's authors | MIT |
