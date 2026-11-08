# Hold a spring-return safety loop while opening a bottle

Open the bottle and keep it upright on the table. The safety loop returns toward
its raised locking position when released. Maintain its release while pressing
the separate rocker; the lid then opens under its passive spring. The free
bottle must remain upright and supported by the tabletop. Only robot contact
may manipulate it. `task.json` specifies the fixed setup and numeric scoring.

The active entry point is `yam/task.py`; root `task.py` forwards the same task.
All custom geometry, runtime, scoring, controllers and evaluation helpers are
inside this folder. No sibling task or repository-level helper is imported.

## Results

The scripted reference reaches1.0 at309control steps (15.45simulated seconds),
seed0. One hand grips the original loop crown and holds it near80degrees; the
upper finger of the other20mm-open gripper presses the lower rocker while its
palm remains below the holding hand. The lid reaches114.24degrees, maximum
sampled wrist force is8.316N and maximum tilt2.735degrees. All samples satisfy
geometric tabletop support. This is one fixed-layout success, not a robustness
estimate. `yam/verification.json` records the independently recomputed result.

The matched release-then-press comparison scores0.0 after500steps/25s:
the same crown approach lowers the loop, then releases it and braces the body
before the same rocker press. The loop returns near8degrees; the rocker moves
5.994degrees, but the lid stays below0.735degrees. All samples remain supported
and upright, with no force veto. This targeted comparison supports the
sustained-release distinction; it is not an exhaustive alternate-strategy search.
`yam/sequential-verification.json` records this result. Dream-1 beta scored0.0 over2400steps/120simulated seconds (34.1wall minutes
including startup), seed0, using592requests in one continuous session with
zero expiry/retries. The arms approached but did not open the loop or lid.
Official action guards modified874steps; remaining interface/model effects are
not isolated. `yam/dream1-verification.json` records the independent result.
The submitted model video includes2401frames, and the demo310frames, at20fps.

## Environment and reproduction

Use the official simulation launchable's activated Isaac Lab-Arena environment.
Tested on Isaac Sim6.1.0.0, Arena0.3.1, one NVIDIA L40S48GB, official YAM revision
`0059d02a1a91bc6834c1d0846022891df0c78730`. `provenance.json` records source pins.
The existing Arena dependencies and robot assets are required; this task does
not replace installed SDK code. Run commands from this task directory:

```bash
source /data/IsaacLab-Arena/yam.env
inspect-robots list embodiments
python -m unittest discover -s . -p 'test_*.py'
(cd yam && python -m unittest test_contract test_continuity)
```

Choose fresh output directories; preserve the final JSON and measurement trace.
The full task budget is2400steps/120simulated seconds at20Hz. The scripted
reference normally finishes well before the600step diagnostic cap:

```bash
out_dir="$PWD/artifacts/scripted-check"
mkdir -p "$out_dir"
export BOTTLE_TRACE="$out_dir/measurements.jsonl"
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=yam/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction 'Open the bottle and keep it upright on the table.' \
  --scorer task_score --max-steps 600 --seed 0 --store-frames \
  --log-dir "$out_dir" > "$out_dir/run.log" 2>&1
python yam/analyze_evaluation.py "$out_dir"
```

The controller commands only the14robot joint/gripper values. It cannot set
bottle root poses or joint positions. Source freezing and trace analysis are
independent of the success callback; a process exit0 alone is insufficient.

Configure Dream credentials privately using the official launchable. The runner
checks presence without printing keys. These commands make paid model calls:

```bash
# Six-request continuity preflight, then the full budget in a fresh directory.
bash yam/evaluate.sh "$PWD/artifacts/continuity-check" 56 4
bash yam/evaluate.sh "$PWD/artifacts/dream-check" 2400 4
```

The checkpoint is `dreamlabs/dream-1-beta`. Four rows execute between requests,
which avoided observed session expiry in the slow1kHz bottle simulation. This
changes request cadence, not model outputs, physics or observation content.
The wrapper aborts before executing a chunk if its server session is missing or
changes, and independently checks the completed log and continuity trace.
`validation.exit` must be0. No gateway lifetime or robustness guarantee is made.
The first related task's full evaluation took approximately35wall minutes;
startup plus this scripted reference takes approximately4minutes.

```bash
# Offline; no simulator or model call.
python yam/analyze_evaluation.py artifacts/dream-check
python -m spring_bottle.scoring artifacts/dream-check/measurements.jsonl
# Use the generated final adhoc_*.json filename, excluding *.live.json.
inspect-robots video artifacts/dream-check/FINAL_LOG.json
# Same fold and press plan, but release the loop and brace the body first.
python yam/audit_sequential.py --out "$PWD/artifacts/sequential-check" \
  > "$PWD/artifacts/sequential-launch.log" 2>&1
cp artifacts/sequential-launch.log artifacts/sequential-check/run.log
python yam/analyze_evaluation.py artifacts/sequential-check
```

The official video exporter writes each camera stream. Submitted videos combine
top and right-wrist streams at original20fps, including the reset observation.
The policy receives official RGB camera views and robot state; evaluator-only
bottle angles, table support and wrist reaction measurements are not exposed.

## Mechanism, scoring and limits

The four rigid links, three hinges, U-loop exterior and73collision pieces are
preserved from the related detented-loop bottle. No side handle, fixture,
container contents or extra tool is added. The loop instead has a passive
zero-rest torsion spring0.012N m/rad and damping0.002N m s/rad. It receives no
detent torque, gravity-compensation torque or software release rule. Lid and
rocker have passive springs. Internal whitelist contacts retain the geometric
hook/receiver constraint; there are no policy-dependent contact filters or
post-reset pose corrections. PhysX integrates at1kHz,50substeps per control
tick,64position and8velocity solver iterations.

At20Hz, each threshold requires8consecutive ticks: loop>=60degrees gives0.25,
lid>15degrees gives0.5, lid>60degrees gives1.0. The current terminal score counts,
not the maximum earlier score. The loop threshold marks its displaced release
position rather than the first task's fully folded150degree detent position.
Full score or invalidation ends the trial; timeout keeps current partial credit.

A bottle-axis/world-up dot<=0.90, baseZ<0.7375m, nonfinite robot/bottle state,
unavailable/nonfinite wrist readings, or a wrist reaction norm>100N for4ticks
permanently vetoes credit. Use the larger wrist norm, including gravity. The
base disc's lowest edge (radius36mm, accounting for tilt) must remain within
10mm of tabletopZ=.7475m, with centreX[-.154,.874]m andY[-.614,.614]m. This is
geometric support proximity, not a table contact-force sensor. Missing support
measurements fail closed. Tilt, force and support gates apply throughout.

This is an estimated rigid-body mechanism: dimensions, inertia, friction and
springs are approximate. The force units inherit10N/20N static-load validation
of the common adapter, not new dynamic or hardware calibration. Cosmetic rings,
seals and spout surfaces are visual; no fluids, seal deformation, breakage or
hardware execution is modeled. The force ceiling is an evaluator safeguard,
not a measured real-bottle opening specification.

## Difference from related tasks

The task board was reviewed on2026-11-08 (1101visible accepted/pending/planned
items). The related detented bottle permits sequential loop release followed
by rocker operation. Here the zero-rest spring requires maintaining loop
release during the separate press; the exterior is deliberately unchanged.
The comparison tests this behavioral distinction rather than object appearance.
Other nearby tasks use axial push-and-turn medicine caps, opening-and-pouring,
a case with opposing prismatic latches and a detached lid, or holding a
spring-return drawer while retrieving an object. This bottle combines a hinged
safety loop, separate rocker/hook contacts and upright free-body manipulation.
Reviewers determine whether the distinction merits a separate task.

`yam/submission-audit.json` records the actual validation scope. Neither one
successful demo nor a targeted negative comparison is an exhaustive exploit
search, hardware validation or random-layout success-rate estimate.
