# Restrain a spring-loaded tote lid with a loose elastic strap

Choose one of four loose elastic straps, move both hooks to different anchor
bars, and restrain an overfilled tote lid after releasing the hooks and lid.
The complete fixed setup, instruction, numeric scoring and approximations are
in submission-fields.json, setup-inventory.json and task.py. All dimensions are metres unless stated.

The two YAM arms use genuine14D joint_pos controls at20Hz. There are4000
controls (200 simulated seconds). Total arm motor effort includes gravity
inside27Nm shoulder and7Nm wrist caps; each finger is capped22N. A28N hook
load,80N wrist force for0.1s, snap-back or source geometry/measurement failure
permanently vetoes credit. Exact partial/full bands are in d1r_arena/scoring.py.
The final continuous hold counts; prior success does not survive a later veto.

## Runtime and commands

Use the existing official Python3.12/IsaacSim6.1/Arena/YAM/Inspect environment.
Target official pins and measured reference versions/hashes are in runtime-bindings.json. The environment supplies
numpy, scipy, PyTorch, Pillow, ffmpeg (or imageio-ffmpeg) and robot/station assets.
No sibling task, planner service, environment installation or downloaded task
asset is required. Use CPU PhysX with an allocated RTX-capable GPU for cameras.

From this folder, choose a fresh output directory outside the source:

    python -B run_trial.py --out /absolute/path/to/fresh-demo-output

Require result.json full_goal_reached=true, final_score=1.0,
recording.encoder_returncode=0, recording.frames=steps and no error field.
The development reference reached1.0 in2291 actions (114.55 simulated seconds).
The packaged controller uses measured robot/hook state and kinematic planning;
its actual action count is checked against4000. Every change is an ordinary14D
joint target. It never writes object poses, fixture forces, scores or hold timers.
The two bounded planning pauses advance no physics or scoring time. Both fresh
plans must pass independent source/fixture/cord/FK checks before execution.
This is a disclosed state-aware scripted feasibility demo, not a model.

task.py demo() registers a neutral SDK hold; use run_trial.py for the full
reference. Model observations remain the actual top_cam,left_cam,right_cam
RGB at320x176 and actual14D robot state. The recorder composes the native
three cameras into1280x720 H.264 at20fps, without extra camera actors, added
hold frames or team labels. It writes setup.png and initial/final camera PNGs.

## Dream-1 and submission status

Two parallel grippers require an actual Dream-1 run through Inspect Robots.
Use the exact instruction from submission-fields.json (task.json must match
after publication metadata is added), seed42, object_jitter0,20Hz and4000 steps.
A model score0 is valid with a separate verified full-score reference;
an API/runtime error is not a measured zero-score trial. The official existing
continuity/authentication workflow must be used. The verified Dream-1 trial scored 0.0 after 4000 controls with no API or runtime
error. The independent scripted trial scored 1.0 after 2291 controls.

Run only a source/argument preflight without contacting the model:

    python -B run_model.py --out /absolute/path/to/fresh-model-output --preflight-only

In the configured supported runtime, run the model entry with a fresh output path:

    python -B run_model.py --out /absolute/path/to/fresh-model-output

This invokes the official Dream-1 policy through Inspect at4000 steps and4
rows/request. The continuity log records only hashed server-session identifiers;
missing or changed sessions reject the episode. Require actual Inspect completion
and numeric scorer output, with no API/runtime error. The recorded model episode used seed 42 and zero object jitter; the derived
Inspect trial seed does not change this literal fixed geometry. Its 4001 native
observation frames per camera were encoded at 20 fps (200.05 seconds including
the initial frame). Model wall time was about 1018 seconds; scripted wall time
also includes two bounded planning pauses and depends on hardware.

## Reproducibility and limits

Restore all eight hooks to their marked tray slots and both tote lids to their
upper stops. Simulator reset restores the fixed layout and clears permanent
failure/hold history. This is one fixed setup; no multi-seed claim is made.
Totes/hooks/lids are rigid primitives. The cord is an analytic massless elastic
path over lid edges; robot-cord collision and slack shape dynamics are omitted.
Temporary cord/gripper contact is possible. Overfilled contents are a spring-
loaded sliding lid, not deformable packed goods. See the complete fidelity text
and THIRD_PARTY_NOTICES.md. No physical-world trial is claimed.
