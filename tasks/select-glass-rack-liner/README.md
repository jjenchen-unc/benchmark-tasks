# Select a liner that holds a glass on a 20 degree rack

Choose a suitable sample from six gray, numbered liners, lay it on the fixed
inclined rack, place one glass on it, and release both. The glass must remain
within 5 mm of its settled position and 15 degrees of the rack normal for
20 seconds after 0.5 seconds settling. The rack has no retaining lip. A liner
that cannot provide enough friction does not solve the task; continuing robot
support, including through movable contact chains, cannot earn placement credit.
This fixed public case is C1-T1, physical seed 42. Wetting and a second glass
placement are not required. The complete object inventory and numeric rubric
are in task.json and candidate/task.py; candidate/c1_public.py holds the exact
instruction. Extra native variants in candidate/provenance are preserved as
source history and are not claimed as separately evaluated public tasks.

Actual final Brev results on the same scene, score and limits:

| Trial | Source score | Controls | Evidence |
| --- | ---: | ---: | --- |
| Dream-1 through Inspect Robots | 0.0 | 2268 | One completed, non-error episode; time limit, all nine measurement channels available |
| Scripted recorded-action replay | 1.0 | 1469 | 73,450 one-millisecond samples; 25.274 s final hands-off hold |

The model used four rows per request and one continuous server session, with
559 successful requests and 12 logged SDK transient transport retry events.
There was no policy fallback or episode restart. Its complete 2269-frame camera
streams span 113.45 s at 20 fps. The model episode took 1195.90 s wall time on
the tested L40S runtime, including model requests; wall time varies by service
and hardware. The demo uses 73.45 simulated seconds plus its reset observation
(1470 video frames, 73.50 s). Both fit the 2268-control, 113.4-s native limit,
including settling and hold. The scripted planner used simulator state and
hidden material parameters to obtain its recorded actions; the replay changes
the scene only through genuine 14D robot controls.

Use the existing pinned Python 3.12.16 YAM/Arena/Inspect environment, CPU PhysX
and GPU station cameras. No SDK, image, driver or environment installation or
modification is part of the task. From a shell with that environment active,
set B to this task folder and R to a fresh output directory outside B:

```bash
python -I -B "$B/package_checks.py"
python -I -B "$B/run_phase.py" replay --out "$R/replay"
python -I -B "$B/fresh_review_builder.py" --candidate "$B/candidate" --trial "$R/replay"
ffmpeg -i "$R/replay/top_cam.mp4" -frames:v 1 "$R/setup.png"
```

Replay records the reset observation and every actual action result through
all three existing cameras, without extra actions, render warmups or object
state writes. The supplied setup image is the actual first reset-observation
frame from the full-score demo video, before its first Action. The historical
auxiliary setup.py failed on a missing pre-first-sample status key; it is not
the source of the submitted screenshot and is not a required reproduction step.

Review the fresh measured trace for native and public score 1, the complete
20-second hold, no robot/contact-chain support, valid contact/force/motor data,
and matching source/media receipts. The builder writes an UNAPPROVED template;
save an approved sibling as ROOT-fresh-public-review-APPROVED.json only after
actually inspecting that evidence. A reviewer attestation is not a signature.
Then run:

```bash
python -I -B "$B/run_phase.py" model --fresh-review "$R/ROOT-fresh-public-review-APPROVED.json" --preflight-only --out "$R/model"
python -I -B "$B/run_phase.py" model --fresh-review "$R/ROOT-fresh-public-review-APPROVED.json" --out "$R/model"
python -I -B "$B/tools/verify_model.py" --candidate "$B/candidate" --out "$R/model" --fresh-review "$R/ROOT-fresh-public-review-APPROVED.json"
python -I -B "$B/tools/encode_model.py" "$R/model"
```

The verifier checks actual Inspect success/error/score records, the exact
instruction and 2268-control budget, original frozen source, physical/reset
seeds, one continuous model session, actual 14D actions, every 1-ms sample count,
all nine channels and all camera frame streams. The encoder uses only saved
actual RGB observations. Inspect/model errors are not converted into score 0.

The executed v9 model helper had two spaces on an otherwise empty line. This
publication removes those spaces to satisfy git whitespace checks; its AST is
identical. Only that helper's formatting and its receipt/manifest bindings
differ inside candidate. The executed and published source maps, original three
changed files, and original actual scores are retained under evidence. No
physics/controller/action/scorer/limit changes or new model run are attributed
to that formatting. For a fresh reproduction, source freezing and review use
the published package normally. The historical model verification identifies
the actual executed v9 source fingerprint, not an invented byte-identical hash.

Contact fidelity is scoped: rigid liners and cylinder glass approximations,
estimated material coefficients and spring compliance, pair-only collision
filters, visual water with a passive material-change rule, and a conservative
Coulomb nonfinger-force estimate. The actual reviewed channel receipt reuses
eight signed Y/Z load cases with four new X cases; it is not twelve new cases,
hardware calibration or MuJoCo/PhysX solver equivalence. Detailed limitations
and original notices remain in candidate/DEVIATIONS.md and provenance snapshots.
