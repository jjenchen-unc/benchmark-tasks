# Pull a nail from painted trim: first lay a scrap block down as the claw hammer's fulcrum so the paint is never marked

A white-painted pine trim board is clamped flat on the table with one nail standing 2.4 cm proud of its painted face.
A claw hammer stands head down in a wooden pocket to the right; a plywood scrap block lies on the bare table to the
left. The robot must pull the nail out with the hammer without the hammer ever touching the paint.

A claw hammer pulls a nail by rocking on the rounded back of its head, and that rocking head dents and scuffs whatever
it rests on. Carpenters and painters working on finished trim therefore lay a scrap of wood beside the nail first and
rock on the scrap. Nothing in the scene or the instruction says this: the robot has to see that the hammer's fulcrum
would land on the paint and put the scrap there first. The nail is too tight to pull straight up (the arm alone, or
the claw lifted straight up, does not move it), so the lever is needed, and the lever's fulcrum is the thing to
protect.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, the nail's hold in the wood, `score()` and `success()` |
| `partkit.py` | Rigid parts built from boxes and cylinders, with sliders, for yam_arena task files |
| `demo.py` | The scripted full-score demo and the cheap-strategy variants |

`partkit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task runs
with the standard `yam_arena` embodiment and `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment with the YAM embodiment installed, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pull the nail out of the white board with the claw hammer without letting the hammer touch the paint." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/painted-trim-dream
```

`task_score` prints the trial's score: 1.0 on success (the run ends there), otherwise `score()` at the end of the run.

## Scripted demo

```bash
python demo.py task.py demo_out                                # full score: demo_*_cam.mp4, setup.png, result.json
DEMO_VARIANT=no_block  python demo.py task.py out --no-record  # same hammer moves, rocking on the bare paint
DEMO_VARIANT=hook_pull python demo.py task.py out --no-record  # block placed, claw under the head, lifted straight up
DEMO_VARIANT=pinch     python demo.py task.py out --no-record  # fingers under the nail head, pulled straight up
DEMO_VARIANT=knock     python demo.py task.py out --no-record  # sweep the closed gripper through the hammer
DEMO_VARIANT=nothing   python demo.py task.py out --no-record  # arms stay still
```

The demo is open loop from the published start layout. The only thing it looks at while running is the hammer's pose
(as a camera would see it), to correct for where the hammer ended up in the hand; it never reads the nail's hold or
any other internal state.

1. The left arm picks the scrap block off the table and lays it on the paint just beyond the nail.
2. The right arm takes the hammer by its grip (fingers closing front to back across the handle) and lifts it out of
   its pocket and collar.
3. It sets the hammer's rounded claw-back on the block 8.5 cm beyond the nail and slides it toward the robot, so the
   claw's slot passes either side of the nail under its head.
4. It rocks the hammer on its claw-back, handle away from the robot, leaning down on it: the claw lifts the nail out.

The demo reaches 1.0 (success) at step 559 (28 s at 20 Hz), the same in two runs, with IK within 0.1 mm (right
arm) and 3.3 mm (left arm) of every target. While rocking, the hammer turns only about 25 degrees of the 55 commanded
before the nail starts to rise, because the arm is soft (it lags its command until it pushes hard enough).

Cheap strategies, measured with the variants above in Isaac Sim:

| Variant | Nail raised (max) | Score |
|---|---|---|
| full demo | out (17 mm) | 1.0 |
| `no_block`: same hammer moves on the bare paint | 12.3 mm | 0.0 (paint touched) |
| `hook_pull`: block placed, claw lifted straight up | 0.55 mm | 0.25 (block only) |
| `pinch`: fingers under the head, pulled straight up | 0.37 mm | 0.0 |
| `knock`: gripper swept through the hammer | 0 mm | 0.0 |
| `nothing` (also through `inspect-robots run` with the arms held still) | 0 mm | 0.0 |

## Scoring

Evaluated every control step.

- 0 if at any moment any part of the hammer comes within 1.5 mm of the painted face of the board. A human judge sees
  this in the video as the hammer's head (or any other part) resting on the white face instead of on the brown scrap
  block, and on a real board as a dent or scuff in a photo afterwards. This cannot be undone later in the run.
- Otherwise 1.0 (success) if the nail is out of the wood: raised 17 mm, its point above the paint.
- 0.5 if the nail is raised at least 5 mm (ruler from the paint to the underside of the head).
- 0.25 if the scrap block lies flat on the board with its nearest edge within 2 cm of the nail (ruler), i.e. where
  the hammer's claw-back would rest. A block put down elsewhere on the board earns nothing.
- Otherwise 0.

Touching the scrap block, the bare table, the pocket or the board's edges with the hammer is allowed. Trying to pull
the nail by hand first is allowed and not punished.

## How the nail's tightness can be observed

The nail does not move under a pull below 80 N; the wood only gives elastically (up to 2 mm), and the nail springs back
when let go. A robot can see this by trying: pulling with the fingers or lifting the hooked claw straight up leaves
the nail where it is.

Measured in Isaac Sim with a constant upward pull on the nail from a fresh start (`nail_hold.py`, held 2 s, then
released for 1 s):

| Pull | Lift after 1 s | Lift after 2 s | 1 s after release |
|---|---|---|---|
| 40 N | 1.0 mm | 1.0 mm | 0 mm (springs back) |
| 60 N | 1.5 mm | 1.5 mm | 0 mm |
| 75 N | 1.9 mm | 1.9 mm | 0 mm |
| 90 N | 6.8 mm | 11.5 mm | 9.5 mm (slipped) |
| 120 N | 19 mm (out) | 19 mm | 18.8 mm |

```python
# nail_hold.py (run inside the yam_arena environment, from this folder)
import numpy as np, torch
from inspect_robots import Scene
from inspect_robots.types import Action
from inspect_robots_yam.packing import pack
from yam_arena.inspect_embodiment import yam_arena_embodiment
from yam_arena.constants import READY

emb = yam_arena_embodiment(task="task.py", object_jitter=0, control_hz=20, cam_width=64, cam_height=64)
env = emb._env.unwrapped
hold = Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0]))
for F in [40, 60, 75, 90, 120]:
    emb.reset(Scene(id="hold", instruction="hold"))
    trim, lift = env.scene["trim"], []
    for i in range(60):  # 2 s pulled, 1 s released
        trim.set_joint_effort_target(torch.full((1, 1), float(F if i < 40 else 0), device=env.device))
        emb.step(hold)
        lift.append(round(1000 * float(trim.data.joint_pos[0, 0]), 2))
    print(F, lift[19], lift[39], lift[-1])
```

## Runtime and seeds

The layout is fixed (`-E object_jitter=0`); there are no seeds to choose. The scripted demo is deterministic. A full
1,200-step trial takes about 2 minutes on an RTX A6000 plus 2 to 3 minutes for Isaac Sim to start.

## Fidelity

Rigid bodies built from boxes and cylinders; the board, the pocket and the collar are fixed. The hammer's claw-back is
a real cylinder of 1.6 cm radius and its split claw a pair of prongs with a 4.4 mm slot, so the claw engages the nail
head through contact alone. The nail is a slider. Its hold in the wood is a stick-slip model computed each step: the
nail hangs on a stiff spring (40 N per mm) toward an anchor point that follows the nail only when the pull exceeds
80 N, and nothing holds it once its point is out. A real 6d common nail in pine holds roughly 100 to 250 N; 80 N is
chosen so that the weak YAM arm (27 N m at the big joints) cannot pull it straight up but can through the hammer's
lever of about 6 to 1. The nail body weighs 50 g in simulation (a real nail weighs about 3 g) so the solver keeps the
claw from pressing through its head. Paint marking is judged geometrically from the hammer's surface points.

## Real-world build

A length of primed and painted pine trim (14 x 24 x 2 cm) clamped to the table, a 6d common nail driven 17 mm into it
(head 24 mm proud), a standard 16 oz claw hammer standing head down in a wooden pocket with a collar, and a plywood
offcut (10 x 5 x 1.5 cm). Inspect the paint for dents or scuffs and measure the nail with a ruler.
