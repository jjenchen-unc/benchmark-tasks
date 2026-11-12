# Remove a loaded drawer from ball-bearing slides by pressing both disconnect levers at once

The target drawer runs on side-mounted slides modelled on full-extension
ball-bearing drawer slides. Each slide has a red spring-loaded disconnect lever.
The slide stop lets the drawer out only while both levers are pressed at the
same time, so the robot must keep both levers pressed while pulling, then carry
the loaded drawer out and set it level on the table without damaging either
glass jar. There is no lift-over catch on this drawer: only the two levers
disable its stop. The scene is one fixed layout; the episode seed does not
change it. The scripted reference demo reads simulator state.

The instruction given to the policy is:

> Take the red-tab drawer completely out of the chest and set it level on the
> table. Keep its sealed box inside and both jars intact. The target jar may
> stand upright on the table outside cabinets. Release the drawer for one second.

Running
-------

Use an existing IsaacLab-Arena / YAM / Inspect / Dream runtime with its USD and
URDF assets and your own Dream credentials. No installation or SDK patch is
performed. Activate that environment, then from this folder:

```bash
python -B run.py prepare --cache /absolute/drawers-review-cache --preflight-only
python -B run.py setup --cache /absolute/drawers-review-cache --out /absolute/new-setup
python -B run.py replay --cache /absolute/drawers-review-cache --out /absolute/new-demo
python -B run.py model --cache /absolute/drawers-review-cache --out /absolute/new-model
```

`--cache` and each new `--out` must be outside this folder, and outputs must be
outside the cache. `run.py` downloads the pinned evidence archive (30,856,983
bytes, SHA256 fixed in `run.py`), or uses an offline copy given with
`--archive /absolute/archive.tgz`. It extracts it into the cache, checks every
member hash and checks that the submitted `candidate/` and `runtime/` sources
match the payload that is executed. The member list is in
`evidence/verification-cache-inventory.json`.

- `prepare`: CPU source checks only.
- `setup`: zero actions; captures the reset state (typically 20-40 s).
- `replay`: runs the scripted reference demo through the public YAM entry, with
  no object pose writes (about 7.5 min wall time for the recorded demo).
- `model`: runs Dream-1 through Inspect Robots in one continuous episode,
  seed 42, object_jitter = 0, 20 Hz, at most 3,586 steps (179.3 simulated
  seconds), four actions per request. SDK warmup actions count toward the same
  budget. The recorded run took 27.5 min wall time; wall time depends on
  inference latency. An episode restart or model error fails verification; it
  is not reported as a score of 0.

`model` keeps the Inspect summary and record, the three camera frame streams,
the action log and the source registration, and its verifier checks the
completed episode, task score, clocks, instruction and frame identities before
encoding the three policy views. Do not use the exit status alone as evidence
of success.

Scene
-----

`candidate/inventory.json`, `candidate/fixture.json`, the USD files and
`candidate/assets.json` give every primitive's geometry, material, mass, centre
of mass, inertia and reset pose. The fixture has two loaded drawers, two
wooden carcasses and an empty adjacent slot. The target drawer has the red
disconnect levers (6 mm travel, 700 N/m); its stop is disabled only while both
are pressed more than 4 mm. The other drawer runs on bottom-mount runners with
orange pinch locks and a rear hook. Each drawer holds a sealed ballasted box
and a glass jar.

The cabinet's own frame, used in scoring, maps to the world frame by a
180-degree rotation about Z followed by a translation of (0.42, -0.17, -0.05) m.
The YAM bases are at (0.05, +/-0.30, 0.93) m with identity rotation. Each sits
on a rigid steel floor mount: an 80 x 220 x 920 mm post centred at
(-0.15, +/-0.30, 0.46) m and a 280 x 220 x 20 mm beam centred at
(-0.05, +/-0.30, 0.92) m. `candidate/mount.json` is authoritative for the
mounts; the robot-base entry in the inventory is older and superseded by it.
Geometry and dynamics are not scaled to make the task easier.

The policy commands the 14 YAM joint targets directly. Targets are checked
against joint limits and the Inspect clamp and delta guards, and total motor
torque including gravity is capped at 27/27/27/7/7/7 N m with 40 N per finger.
The record of the controller interface change is in
`candidate/proofs/v21-interface-derivative.json`.

This is a rigid geometric approximation without hardware or FEM validation.
Jar damage is a contact-force, impact-speed and tilt threshold, not simulated
fracture or spill. Feet and torsional friction are approximated. Contact with
the gripper finger pads (the distal-tip collision solids) is exempt from the
jar force and impact limits; every other contact counts.

Scoring
-------

Every physics substep is evaluated (240 Hz, 12 substeps per 20 Hz step). After
the first 0.3 s, the score drops to 0 for good if either jar sees more than
15 N of non-pad force, a closing speed above 0.3 m/s along the contact normal,
or a tilt above 30 degrees; it also drops to 0 if an object leaves the work
area or either wrist force stays above 80 N for 0.1 s.

Otherwise the current state scores 0.25 when every part of the target drawer,
including runner ends and levers, is in front of the cabinet face
(cabinet-frame x < 0.12 m), the sealed box is still inside, and the target jar
is inside the drawer or upright on the table and not touched by the robot.
0.5 additionally requires the drawer centre within 6 mm of cabinet-frame height
0.835 m and at cabinet-frame x < 0. 1.0 additionally requires drawer tilt under
3 degrees and no robot contact with the drawer for 1 s. The episode ends on
success, a zero condition or the step limit; earlier progress does not replace
a final state that no longer meets the conditions.

Results
-------

- Demo: 1.0 in 1,043 steps (52.15 simulated seconds), drawer released for
  1.04 s at the end.
- Dream-1: 0.0 in 3,586 steps (179.3 simulated seconds), 889 requests in one
  session with no transport retries. The arms moved above the fixture without
  disconnecting the drawer, and the episode ended at the time limit.

All recorded videos decode fully at 20 fps (H.264). The 640 x 360 overview
camera is for viewing only; Dream receives only the top_cam, left_cam and
right_cam 320 x 176 images. `candidate/task.draft.json` is a frozen runtime
input; `task.json` records the submitted results.

A new independent physics review can be run with
`python -B run.py fresh-physical --cache /absolute/cache --out /absolute/new-physical`,
which runs the scripted reference with the validation channels left
unapproved; approving its evidence is a separate manual step.
