# Pick the shelf liner that stops a glass sliding on a 20-degree drying rack

Six grey liners, numbered by white dots, look alike but grip glass very
differently; only testing reveals which ones hold. The robot must choose a
liner, lay it on a fixed rack sloped at 20 degrees, stand one glass on it and
release both. The glass must then stay within 5 mm of where it settled, and
within 15 degrees of the rack normal, for 20 seconds after 0.5 seconds of
settling. The rack has no retaining lip and a bare glass slides on it. A liner
without enough grip cannot solve the task, and continued robot support,
including through chains of movable objects, earns no placement credit.

The scene also has a pan of water, a second glass and a loose practice board.
They are optional: the robot may use them, for example to try a liner before
committing. Dipping a liner in the pan makes it wet, which lowers the grip of
some liners. This task needs only one glass and does not require wetting. The
layout is fixed (physical seed 42). The full object list and scoring are in
`task.json` and `candidate/task.py`; `candidate/c1_public.py` holds the exact
instruction. Other scene variants kept in `candidate/provenance` are source
history and are not separately evaluated here.

Results (same scene, scorer and limits):

| Trial | Score | Steps | Notes |
| --- | ---: | ---: | --- |
| Dream-1 through Inspect Robots | 0.0 | 2,268 | One completed episode without errors; ended at the time limit; all nine measurement channels available |
| Scripted demo (recorded-action replay) | 1.0 | 1,469 | 73,450 one-millisecond samples; glass held hands-off for 25.274 s |

The model used four actions per request in one continuous session, with 559
successful requests and 12 logged transient transport retries; there was no
policy fallback or episode restart. Its camera streams have 2,269 frames
(113.45 s at 20 fps). The model episode took 1,195.90 s of wall time on an
L40S GPU, including model requests; wall time depends on the service and
hardware. The demo uses 73.45 simulated seconds plus its reset observation
(1,470 video frames, 73.50 s). Both fit the 2,268-step (113.4 s) limit,
including settling and hold. The scripted planner used simulator state and the
hidden material values to produce its actions; the replay changes the scene
only through 14-D robot joint commands.

## Running

Use an existing Python 3.12.16 YAM / Arena / Inspect environment with CPU PhysX
and GPU cameras. Nothing is installed or modified. With that environment
active, set `B` to this task folder and `R` to a new output directory outside
`B`:

```bash
python -I -B "$B/package_checks.py"
python -I -B "$B/run_phase.py" replay --out "$R/replay"
python -I -B "$B/fresh_review_builder.py" --candidate "$B/candidate" --trial "$R/replay"
ffmpeg -i "$R/replay/top_cam.mp4" -frames:v 1 "$R/setup.png"
```

`package_checks.py` is an offline integrity check of this folder. The replay
records the reset observation and every action result through the three
cameras, with no extra actions, render warmups or object-state writes. The
supplied setup image is the first reset-observation frame of the full-score
demo video.

The model phase only starts after a person has checked a fresh replay. The
builder writes an unapproved review template (`*-UNAPPROVED.json`) in `$R`.
After checking the replay trace (score 1, the full 20-second hold, no robot
support through contact chains, valid contact, force and motor data, matching
source hashes), save a copy as `$R/fresh-review-approved.json` with
`root_approved` set to `true`; the model entry refuses to run otherwise. Then:

```bash
python -I -B "$B/run_phase.py" model --fresh-review "$R/fresh-review-approved.json" --preflight-only --out "$R/model"
python -I -B "$B/run_phase.py" model --fresh-review "$R/fresh-review-approved.json" --out "$R/model"
python -I -B "$B/tools/verify_model.py" --candidate "$B/candidate" --out "$R/model" --fresh-review "$R/fresh-review-approved.json"
python -I -B "$B/tools/encode_model.py" "$R/model"
```

The verifier checks the Inspect success, error and score records, the exact
instruction and 2,268-step budget, the frozen source, the physical and reset
seeds, one continuous model session, the 14-D actions, the 1 ms sample counts,
all nine measurement channels and all camera frame streams. The encoder uses
only saved RGB observations. Inspect or model errors are not turned into a
score of 0.

The model helper that was executed had two spaces on an otherwise empty line;
this package removes them to pass git whitespace checks (the code is
unchanged). The executed and published source maps and the original scores are
kept under `evidence/`.

## Fidelity

Rigid liners and cylinder glass approximations, estimated friction values and
spring compliance, pair-only collision filters, visual water with a one-time
wet-material switch, and a conservative Coulomb bound on non-finger contact
force. The force-measurement check reuses eight earlier Y/Z load cases plus
four new X cases; it is not hardware calibration or a cross-simulator
comparison. Detailed limitations are in `candidate/DEVIATIONS.md`.
