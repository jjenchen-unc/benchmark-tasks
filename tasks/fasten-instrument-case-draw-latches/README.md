# Fasten an instrument case with spring draw latches

A task file for the `yam_arena` embodiment (Isaac Lab-Arena, station scene): [task.py](task.py). The scene,
mechanism and scorer code are in [candidate/](candidate/); [candidate/task.py](candidate/task.py) defines the
objects and `score()`, and [candidate/scoring.py](candidate/scoring.py) holds the scoring rules.

Close the open blue-grey case, seat each of its two separate spring hooks on one of the three keeper bars on
the lid, pull each lever flat to tension its hook, slide both safety catches home, and keep both hands off the
case and its instrument for 20 s. The green case stays closed. Unlike a toolbox latch that hooks itself when the
lever is closed, each hook here is a loose link that must be placed on a keeper by hand before the lever can
tension it; a flat lever on an unseated hook earns nothing for that latch.

## Run a trial

Dream-1 (from the repository root):

```bash
inspect-robots run --embodiment yam_arena --policy dream -P model=dreamlabs/dream-1-beta -P rows_per_request=4 \
  -E task=tasks/fasten-instrument-case-draw-latches/task.py -E device=cpu -E control_hz=20 -E object_jitter=0 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$(python -c 'import ast,sys;print(ast.get_docstring(ast.parse(open(sys.argv[1]).read())))' tasks/fasten-instrument-case-draw-latches/task.py)" \
  --scorer task_score --max-steps 7964 --seed 42 --store-frames
```

`task.py` clips both gripper columns of every Dream-1 reply to the valid range [0, 1] before the standard
decoder (arm values are not changed); see "Results".

Scripted demo (runs the demo controller [candidate/reference.py](candidate/reference.py), records the three cameras,
and writes the actions it sent to `<out>/demo-actions.jsonl` and its pose checks to `<out>/checks.jsonl`):

```bash
cd tasks/fasten-instrument-case-draw-latches
python run_demo.py --plan --out /tmp/s2-demo
```

Replay of the recorded demo actions, open loop: `python run_demo.py --actions demo-actions.jsonl --out /tmp/s2-replay`.

No-op check (holds the start posture): `python run_demo.py --noop 400 --out /tmp/s2-noop`. Add
`--rtx-driver-check-override` on hosts whose GPU driver is older than the one Isaac Sim checks for.
`run_demo.py` prints the live score every 200 steps and a final line `FINAL {...}` with `live_score`,
`steps`, `hold_s` and `termination_reason`; the per-step record is in `<out>/journal.jsonl` and the camera
videos are `<out>/top_cam.mp4`, `left_cam.mp4` and `right_cam.mp4`.

## Runtime and seeds

- Physics runs on the CPU at 500 Hz (25 substeps per 20 Hz action); RTX rendering needs a GPU.
- Demo: 3,951 steps (197.55 s simulated), about 22 minutes on one L40S including start-up.
- Dream-1: 6,979 steps (348.9 s simulated) before the episode ended; about 35 minutes including model calls.
- `object_jitter=0` keeps the layout fixed; the physical layout and properties use seed 42. The Inspect seed is
  passed through unchanged and does not change the scene.

## Physical setup

Coordinates are metres, X forward, Y left, Z up. The standard YAM table top is at Z = 0.7475; the robot bases
are at (0, +0.305, 0.75) and (0, -0.305, 0.75). Both arms start in the standard ready posture with joint 1 at
+1.2 rad (left) and -1.2 rad (right) and both grippers open.

- Fixed wooden plinth: 200 x 600 x 60 mm, centre (0.35, -0.04, 0.7775).
- Two loose rigid plastic cases: 240 x 160 x 180 mm bodies with 40 mm hinged lids and 6 mm walls, 1.82 kg each
  including lid and steel hardware. Blue-grey case A at (0.34, 0.10, 0.8075), green case B at
  (0.34, -0.18, 0.8075), both rotated -90 degrees about Z.
- Each case has two steel draw latches at ±65 mm from its centre. Each latch is a lever on the case body, a
  separate hook on a spring link, and three 5 mm keeper bars at 6 mm pitch on the lid. The hook spring has a
  4 mm preload. Case A has 12 x 12 x 14 mm safety-catch tabs that slide sideways 7 mm; case B's tabs are fixed.
- Each case holds one upright steel instrument stand-in: a 40 mm diameter, 160 mm tall cylinder with a 42 mm x
  12 mm cap, 0.30 kg, centred at (0.35, 0.10, 0.894) and (0.35, -0.18, 0.894).
- Start state: case A's lid is open 1.85 rad, both levers are open 1.9 rad, both hooks hang free and both catches
  are out (7 mm). Case B is closed with both hooks on their keepers.
- Friction: case and plinth 0.7, instruments 1.0, latch hardware 1.0 to 1.2.

[SCENE_INVENTORY.md](SCENE_INVENTORY.md) lists all 95 colliders with their sizes and materials;
[candidate/fixture.json](candidate/fixture.json) gives every pose, mass, inertia, joint and spring constant.
The case and instrument models are in [candidate/assets/](candidate/assets/); their origin is in
[SOURCE_ASSETS.md](SOURCE_ASSETS.md). The simulator settles for 0.5 s with the springs active before the first
counted step. Reset restores both cases, all latch parts and both instruments.

## Scoring

`score()` reports the current score after every step:

- 0 if the robot is touching case A or its instrument at the end.
- 0 if case A's lid is open more than 2 degrees.
- Lid closed: 0.25, plus 0.25 for each fastened latch. A latch is fastened when its hook is attached to any
  keeper bar, the hook spring is stretched at least 1.5 mm, the lever is within 0.05 rad of flat and the
  safety catch is within 1.5 mm of home. The maximum before the hold is complete is 0.75.
- 1.0 once the lid is closed and both latches are fastened with no robot contact for 20 s in a row; the episode
  ends there. Any break restarts the 20 s count.
- 0 for the rest of the episode if either case tilts more than 15 degrees or either instrument tilts more than
  20 degrees at any moment, or if a wrist force stays above 80 N for 100 ms; the episode ends there.

Arm protection (not part of the score): a wrist force above 60 N for 20 ms, or more than 50 N averaged over
20 ms on the arm links, stops that arm for the rest of the current action.

A hook counts as attached after it has touched the keeper bar and stayed within 2.5 mm of it for 150 ms; the
simulator then joins hook and bar with a ball joint, which releases when the spring goes slack with the lever
opened past 105 degrees (70 degrees once the hook has been tensioned).

## Results

- Demo: 1.0 after 3,951 steps; the 20 s hands-off hold completes there and the episode ends with success.
  Peak instrument tilt 0.23 degrees, peak case tilt 0.17 degrees.
- No-op (hold the start posture for 400 steps): 0.0; the lid stays open.
- Dream-1: 0.0 after 6,979 steps. The instrument in case A tilted 20.08 degrees, past the 20 degree limit, at
  348.93 s; the lid was down but neither hook was attached. Dream-1 sometimes returns gripper commands slightly
  outside the valid range [0, 1]; in this run all 16,218 such values were clipped to 0 or 1 before the standard
  decoder, and all arm joint values were passed through unchanged. Two earlier Dream-1 attempts stopped with
  errors and were not scored: in the first, before clipping was added, the standard decoder rejected a gripper
  value outside [0, 1] at step 3,436; the second ended at step 1,488 with a model-service authorization error.

## How the demo works

The demo controller [candidate/reference.py](candidate/reference.py) moves the left arm through fixed phases
planned from the published layout and geometry: lift the lid past upright and let it fall shut, lower each lever,
push each hook onto the lowest keeper bar, press each lever flat, slide each safety catch home, then move both
arms clear and wait. It acts only through the 14 joint commands. To decide when a phase is done it reads object
poses and mechanism angles (lid, levers, catches, hooks), as a camera could see them, and the robot's own joint
positions. It never reads the hook-attached state, contact or force readings, spring or friction constants, or
the score. [demo-actions.jsonl](demo-actions.jsonl) is the action log of the scored run.

## What a policy can observe

Every step of the task is visible as a pose. The values below are from the scored demo run (checks.jsonl):

| step | visible sign that it is done | threshold | demo value |
|---|---|---|---|
| lid closed | lid angle | at most 2 degrees | 0.00 degrees |
| hook seated | hook tip within 2.5 mm of the keeper bar for 0.6 s while pushed, then still there with the hand backed off 8 mm | 2.5 mm | within 2.5 mm for 1.0 s (A0) and 1.2 s (A1); 0.52 and 0.53 mm after back-off |
| hook tensioned | lever angle, with the hook tip still on the bar | lever at most 0.05 rad; tip within 2.5 mm | 0.038 and 0.031 rad; 0.14 mm |
| catch home | catch slide position, steady | at most 1.5 mm, moving less than 1 mm/s | 0.75 and 0.34 mm |
| ready for hands-off | all of the above, instruments and cases upright | tilt under 20 / 15 degrees | 0 degrees |

A hook that only touches the bar, without being seated, springs back when the hand backs off. In development
runs, backing off after a shorter dwell left the tip 10 to 13 mm from the bar; in the scored run the tip first
stayed within 2.5 mm for 0.6 s on the tenth push of each hook, and the back-off test then passed.
## Fidelity

All cases, hooks and instruments are free rigid bodies with the masses and joints above; passive springs and
joint friction act on the latch parts. Hook seating is a contact-plus-proximity rule that switches on a ball
joint, not a model of steel deformation. The YAM arms have 27 / 7 Nm total effort limits and 40 N grippers. The
case is a rigid stand-in, not a measured product, and no transport or vibration test is included.
