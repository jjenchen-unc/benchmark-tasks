# Refit a load centre dead-front: hang it on the started screws, swing every breaker through its window, drive the screws

The task is the Isaac Lab-Arena (PhysX) `yam_arena` task in `isaac/`; `src/` holds the engine-independent modules it
imports (layout and geometry, scorer, fast IK) and `tests/` their unit checks.

## The task
A small wall-mounted residential load centre (6 spaces, Square D Homeline HOM612L100S class) has had its dead-front
cover off. All six spaces hold single-pole breakers; their handle shrouds stand 9 mm proud of the tub face (the handles 16 mm). The cover
(16 gauge steel, 250 x 340 mm, 1.0 kg, hemmed side edges) stands in a grooved stand on the bench in front of the panel.
The two lower cover screws (#10-32 hex washer head) are started in the tub's bottom flange with their heads standing
3.5 mm proud of where the cover's front face will be; the cover's lower holes are open-bottom slots that hang on them. A
stubby ratcheting 5/16 in nut driver stands in a holster at the right.

The robot lifts the cover with both hands, hangs its two slots on the started screws (the plate has to pass between
the screw heads and the tub flange with its top leaning out, so the shrouds stay clear), swings it shut about the
screws so that every shroud goes through its window, holds it flat with one hand, and with the other sets the
driver's socket on each screw head and turns it (4.4 turns, in ratchet strokes) until the head is down on the cover;
then both hands let go of the cover. The openings clear the shrouds by 1.0 mm a side (the cue: the shroud edges against the opening
edges). Depending on the starting pose and contact trajectory, a cover hung slightly off-centre can settle into place or remain caught on a shroud.
Unscrewed, the cover does not stay flat on the screws (let go, it stands off the tub and tips open once a screw is
driven), so it must be held flat until both screws are down. The cover's top two screws go in after this step and are not part of the trial.

Why it matters: refitting the dead-front ends every job that opens a panel (adding or changing a circuit, replacing a
breaker, inspection); electricians (about 760,000 in the US) do it with their ordinary hand tools.

## Scoring (`src/scoring.py`, primary score 0..1, value at the end of the trial)
- Earn 0.25 when the cover hangs on both lower screws, all four back-face corners are within 1 mm of the tub face
  in either direction, and all six breaker shrouds pass through the cover. Three shrouds share each column opening.
  The scorer checks the full rectangular section of each shroud against its opening at both faces of the steel plate.
- 0.5: + one screw head down on the cover.
- 1.0 (ends the trial): + both screw heads down, every fingertip 30 mm or more from the cover, the driver's socket off
  the screws, the cover still for 1 s.
- 0 otherwise: cover not on both screws (still in the stand, dropped, lifted off), any shroud missing its opening, or not flat (resting on a shroud,
  tipped open, crooked), or dropped at any time (moving down faster than 0.25 m/s with no hand on it: latched).
Real trial: a 1 mm feeler gauge at the four corners; a screw is down when its head touches the cover and the driver
will not turn it further with normal hand force; hands and driver from the video.

## Alignment validation and Dream-1 evaluation

The official scripted reference scored 1.0 in 2,221 control steps on CLI seed 0 with the revised scorer. At policy step 862 the cover hung on both screws and its four corner errors were within 0.200 mm, but only five shrouds passed. The scorer returned 0 at that observation. The reference subsequently reached full alignment and completed the installation.

A direct PhysX run of `neg-no-centre` on named setup 0 ended at step 1,152 with a score of 0.0. The cover hung with five shrouds through from step 618 to the end. In two controlled, mirrored poses with both screws marked seated, clear hands and a still, flat, hung cover, one shroud overlapped the edge by 0.15825 mm. Both poses scored 1.0 under the original scorer and 0.0 under the revised scorer. A centered positive control scored 1.0 under both.

Dream-1 scored 0.0 on the revised task (CLI seed 0, all 3,600 control steps, termination reason `max_steps`). Two earlier attempts stopped at step 32 when the inference service refused the request.

From the task's `isaac/` directory in the Isaac Sim 6.1 and Isaac Lab-Arena environment, run:

```bash
instruction=$(python -c 'import json; print(json.load(open("../task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
    -P model=dreamlabs/dream-1-beta -P rows_per_request=10 \
    -E task=task.py -E object_jitter=0 -E device=cpu -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
    --scorer task_score --seed 0 --max-steps 3600 --store-frames --log-dir ./dream-output
```

## Run and score a trial
Inside the Isaac Sim 6.1 + Isaac Lab-Arena container, working directory `isaac/` (`isaac/run_frozen.sh` does this on a
frozen copy of `isaac/`, `src/` and `task.json`, writes `source-sha256.txt` and logs to `<out>/<run>/run.log`; see its
header for the environment variables it needs):

```
# official Inspect Robots run of the scripted reference demo (registered closed-loop policy panel_deadfront_scripted)
python official_run.py -- run --embodiment yam_arena --policy panel_deadfront_scripted -E task=task.py \
    -E object_jitter=0 -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 \
    --instruction "$(python -c 'import json;print(json.load(open("../task.json"))["instruction"])')" \
    --scorer task_score --seed 0 --max-steps 3600 --store-frames --log-dir <out>/official
inspect-robots video <out>/official/<log>.json --out <out>/official/video     # H.264 video of every camera

# plain do-nothing run (task.py demo() gives the yam_scripted policy no waypoints: both arms hold READY)
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py ... --max-steps 3600

# direct demo / negative stages (trace.jsonl, result.json, demo_<camera>.mp4, setup.png, final.png)
python demo.py task.py <out>/<run> --stage demo --seed 1415158438   # the default inspect-robots seed
python demo.py task.py <out>/<run> --stage demo --seed 7            # another seeded start
python demo.py task.py <out>/<run> --stage neg-no-centre   # swing shut with a 1.5 mm lateral offset command
python demo.py task.py <out>/<run> --stage neg-no-hold     # let go before the screws: the cover tips open: 0
python demo.py task.py <out>/<run> --stage neg-one-screw   # only one screw driven: 0.5
python demo.py task.py <out>/<run> --stage replay          # PANEL_REPLAY=<trace.jsonl>: open-loop replay
```
From the task root, run the unit checks with `python tests/test_scoring.py`.
Tiled evidence video (top + left wrist + right wrist, same timing):
`isaac/tile.sh <out_dir> <name> top.mp4 left.mp4 right.mp4`.

## Earlier runs with the original scorer (CPU PhysX)
- Official Inspect Robots run at the default seed: 1.0 after 2,221 control steps; every executed action equals the
  commanded one (no bound or delta-guard clamping).
- Direct demo at the default seed: 1.0 twice, identical traces; at seed 7: 1.0 (2,245 steps).
- Doing nothing: 0 (plain CLI no-op and a full-length 3,600-step no-op).
- The default-seed command trace replayed open-loop at seed 7: 0 (the cover ends hung but tipped open).
- Both hands let go before the screws: 0 (the cover tips open when the first screw is driven).
- Only one screw driven: 0.5.
- Swung shut 1.5 mm off-centre (inside the slots' play): it first lands on a shroud, then self-centres once the right hand
  lets go and ends 1.0, so sideways centring is not claimed as a difficulty.

## Recorded media

The task board model video shows the complete seed-0 Dream-1 trial (3,600 steps, score 0.0). The demo video shows the scripted reference (2,221 steps, score 1.0). Both show the overhead and two wrist cameras at 20 Hz. The setup image is the overhead camera frame at reset from the scored Dream-1 trial.

## Fidelity (full statement)
- Engine: Isaac Sim 6.1 / Isaac Lab-Arena `yam_arena`, PhysX on the CPU, physics 240 Hz, control 20 Hz, cameras
  320 x 176. All parts are full size.
- Load centre: a Square D Homeline HOM612L100S class panel (6 spaces / 12 circuits, 100 A main lugs, surface mount). The
  catalogue drawings could not be checked from the work host, so the enclosure (200 x 320 x 95 mm), the breaker shroud
  size (46 x 21 mm, 9 mm proud; handles 16 mm proud), the cover (250 x 340 mm, 16 gauge, hemmed side edges, open-bottom
  hanging slots 8 mm wide, one opening per breaker column with 1.0 / 1.5 mm clearance a side) and the screw positions
  are representative of this size class, not copied from the product. The slotted lower holes and the hemmed edges
  are representative features (many load-centre covers have them; not verified for this catalogue number).
- The panel is mounted low (tub bottom 135 mm above the bench, tub face 38 cm in front of the arm bases) so the arms
  can reach it; a real panel hangs higher.
- Bodies: the cover (1.02 kg, 11 convex plate pieces + 2 hem strips) and the nut driver (0.16 kg) are free rigid bodies;
  the tub, breakers, handles, screws, cover stand and driver holster are kinematic (so the contact forces on them can be
  read). The handles do not toggle (the breakers' states are not scored).
- Screw model (the only rule-based element): each started #10-32 screw (hex washer head, 5/16 in hex) is a kinematic
  body on its axis. It turns only while the driver's socket is over its hex (axes within 8 deg and 1 mm, hex at least
  2 mm inside the socket), only in the tightening direction (the driver is a ratcheting driver: turning back does
  nothing), and advances 0.794 mm per turn. It stops when its head reaches the cover's front face (it can never push a
  seated cover into the tub), or the tub face if there is no cover under it. There is no torque model: "down" means
  the head is on the cover's front face (no interference), which traps the cover between the head and the tub flange.
  The screw bodies translate but do not spin (a spinning kinematic shank would drag the cover by friction).
- Contacts: no welds, snaps or proximity triggers anywhere. Friction (estimates): steel on steel 0.35, the cover's hem
  against the finger pads 0.8 (rubber on steel; PhysX combine mode max), rubber stand 0.8.
- Grip forces: the hands pinch the 3 mm hem fully closed (about 7.5 N per finger commanded at the gripper's 5000 N/m drive; peak estimate 14.5 N); the
  driver handle is squeezed 5 mm (about 25 N per finger). Both are under the 40 N per finger limit.
- The scripted demo reads the cover, driver, panel and screw state from the simulator (allowed for scripted demos); a
  person would use the shroud and window edges, the screw heads and the cover's edge against the tub.

## Real rig (parts a person can buy or make)
| Part | Real item | Sim model |
|---|---|---|
| Load centre | Square D Homeline HOM612L100S class (6 spaces / 12 circuits, 100 A main lugs, surface mount), enclosure about 20 x 32 x 9.5 cm (representative) | kinematic tub with front flanges, mounting pan (visual) |
| Breakers | six single-pole Homeline breakers (HOM115 / HOM120 class), 1 in per space | kinematic bodies with 46 x 21 mm handle shrouds 9 mm proud; handles 16 mm proud, ON or OFF |
| Dead-front cover | the load centre's steel cover, 16 gauge, with slotted lower holes; side edges hemmed (representative) | free rigid body, 1.02 kg |
| Cover screws | 2 x #10-32 hex washer head, 5/16 in hex, started in the bottom flange | kinematic screws (screw model above) |
| Nut driver | stubby ratcheting 5/16 in nut driver (or a ratcheting screwdriver with a 5/16 in nut setter), 40 mm shaft | free rigid body, 0.16 kg |
| Wall stub | 19 mm plywood, 42 x 44 cm, screwed to the bench | kinematic |
| Cover stand | plywood block with a 3.5 mm groove and a backrest (4 deg lean) | kinematic |
| Driver holster | 5 x 5 x 6 cm block with a 15.5 mm square hole | kinematic |
Real trial (de-energised panel or a training panel with no supply): a 1 mm feeler gauge at the cover's four corners
and a hand check of the screws.

## Third-party assets and licenses
| Asset | Source | License |
|---|---|---|
| YAM arm model (URDF, meshes) | i2rt, via the Isaac Lab-Arena YAM package | MIT |
| Station scene, cameras, `yam_arena` embodiment | Isaac Lab-Arena YAM package | as distributed with the package (no license file at the pinned revision) |
| Isaac Sim 6.1 / Isaac Lab-Arena | NVIDIA | NVIDIA Isaac Sim license / Apache-2.0 |
| Load centre, breakers, cover, screws, stand, nut driver, holster geometry | generated in code by this task (`src/layout.py`) | MIT (this repository, `LICENSE`) |
| `src/fastik.py` (YAM URDF kinematics) | this task's authors | MIT |
