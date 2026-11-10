# Remove a loaded drawer using paired slide releases

Disconnect the red-tab drawer from its slides, carry it out with its contents,
and leave it level on the table without damaging either jar. The two independent
spring-loaded release paddles, retained load and fragile-contact constraints
make supporting the drawer during disconnection consequential. The fixed case
is native DR-T1/S, physical seed42; different Inspect episode seeds do not
randomize this fixture. The scripted reference reads simulator state.

The actual public instruction is:

> Take the red-tab drawer completely out of the chest and set it level on the
> table. Keep its sealed box inside and both jars intact. The target jar may
> stand upright on the table outside cabinets. Release the drawer for one second.

Use an existing matching IsaacLab-Arena/YAM/Inspect/Dream runtime, its existing
USD/URDF assets and your own Dream credentials. No installation or SDK patch is
performed. Activate that runtime and its established environment first. From
this folder, with its Python:

```bash
python -B run.py prepare --cache /absolute/drawers-review-cache --preflight-only
python -B run.py setup --cache /absolute/drawers-review-cache --out /absolute/new-setup
python -B run.py replay --cache /absolute/drawers-review-cache --out /absolute/new-demo
python -B run.py model --cache /absolute/drawers-review-cache --out /absolute/new-model
```

`--cache` and each fresh `--out` must be outside the submitted source directory;
outputs must also be outside the evidence cache. An offline copy of the exact
review archive can be provided with `--archive /absolute/archive.tgz`.
`run.py` pins the public release URL, 30,856,983-byte size and SHA256, extracts
only bounded regular files into an atomic external cache, validates every
member SHA and compares every submitted candidate/runtime source with the
payload actually executed. Existing cache files are rechecked. The cache holds
the original complete physical-review journals and source maps; those raw
journals and generated media are excluded from git. The full member inventory
is in `evidence/verification-cache-inventory.json`.

`prepare` is CPU source validation only. `setup` takes zero Actions and captures
the actual reset state. `replay` runs the genuine native reference through the
credited public YAM entry, with no object motion writes. `model` uses actual
Dream-1 through Inspect Robots: one continuous episode/session, harness seed42,
object_jitter=0, 20Hz and at most3,586 genuine14D controls (179.3 simulated
seconds), four returned actions per request. SDK warmup actions count toward the
same budget. Official transient transport/server retries stay within that
session; expired/replaced sessions or episode restarts fail verification.
An actual model error is not a model score0. Wall time depends on live inference
latency; the own-process supervisor used for the recorded trial allowed105min,
without any VM shutdown deadline. Setup typically takes20–40s and the recorded
credited demo took 7.5min of wall time.

`model` preserves Inspect summary, record, original three-camera NPY frames,
actual action/status journal, request continuity, seed contract and source
registration. Its external verifier checks a clean completed episode, actual
task score, control/substep clocks, all channels, exact instruction and full
frame identities before encoding the three original policy views. Do not use
exit status alone as evidence of success.

Scene and physical fidelity
---------------------------

`candidate/inventory.json`, `fixture.json`, USD files and `assets.json` give the
complete primitive geometry, materials, mass/COM/inertia and reset layout.
The fixture contains two260×200×70mm loaded drawers, two carcasses and an empty
adjacent slot; spring-loaded red release tabs, orange pinch plates, runners,
channels, stops and hooks; and a sealed ballasted box and jar per drawer.
The chosen task concerns the red-tab drawer. Native coordinates are transformed
by Rz(pi) and translation(.42,-.17,-.05)m. The actual YAM bases are
(0.05,+/-0.30,0.93)m with identity rotation, shifted(+0.05,0,+0.18)m from the
original tabletop defaults. Each rigid steel floor mount has an80x220x920mm
post centred(-0.15,+/-0.30,0.46)m and a280x220x20mm beam centred
(-0.05,+/-0.30,0.92)m. `candidate/mount.json` is authoritative for these
mounts. The embedded inventory's older robot-base description predates this
mount; its fixture geometry/mass entries remain the preserved source inventory. Geometry and task dynamics are not scaled to make
the robot succeed. Reset restores object poses/velocities, releases, spring and
stop state, contact/safety history and hold counters.

The original Panda controller exposed XYZ+yaw and internally constructed a
downward tool orientation. The YAM adaptation exposes its genuine14D joint
target interface instead. It retains finite-target/actualjointlimit validation,
the official Inspect Clamp+Delta guards and actual hardware total PD+gravity
caps27/27/27/7/7/7Nm with40N per finger. An earlier port incorrectly imposed an
extra requested-FK2degree/workspace rejection on arbitrary YAM joint actions;
the first real Dream output was rejected before application. That trial remains
ERROR. The repaired v21 removes only that API restriction and its instruction
clause, with no target projection or automatic orientation solver. Native
scorer, physical history, geometry, springs, contacts, reference/controller,
motor caps and budget remain byte-identical. The exact old/current source
derivative is recorded in `candidate/proofs/v21-interface-derivative.json`.

This is a rigid geometric functional approximation, without hardware/FEM
validation. Jar damage is an irreversible numerical contact/tilt predicate,
not simulated fracture or liquid spill. The inherited soft-foot and torsional
friction approximations remain. Pad exemptions use the finite union of the
actual distal-tip collision solids clipped at tip-local z≤−0.0372m with0.5µm
tolerance, using each measured moving jaw pose. They exempt only specified jar
force/normal-impact measurements; other contacts, including content-touch and
drawer hand contact, remain included. No all-layout or hardware validation is
claimed.

Scoring and evidence
--------------------

Every physical substep is evaluated (240Hz;12substeps per20Hz Action). An
irreversible zero applies if either jar experiences non-pad normal force>15N,
normal closing speed>0.3m/s, tilt>30degrees, an out-of-bounds drop, or wrist
force>80N continuously for0.1s. The impact metric is closing speed along the
contact normal, not tangential sliding speed.

Otherwise the current physical state gives0.25 when every part of the target
drawer—including runner noses/tails and tabs—has native maxX<0.12m, the sealed
box remains inside, and the target jar is either inside or upright on the table
outside cabinets and not touched by the robot. Score0.5 additionally requires
native centreZ within6mm of0.835m, native centreX<0, and every part in front of
every cabinet face. Full1.0 additionally requires drawer tilt<3degrees and
no drawer-hand contact for1s. These numbers are in the declared native fixture
frame, not unlabeled SDK world coordinates. Success terminates only after the
actual hold; damage, measurement failure or budget exhaustion also terminates.
Latched progress does not replace invalid current-state predicates.

The source-bound physical review includes the original1092-control affected
replay (13,104substeps, native1 and3.491667s hold; deliberately uncredited
public0), measured contact ownership, moving-jaw pad classification and exact
normal force/speed reconstruction. It remains scoped to that fixed case and
reused selected positive API evidence. Original launchers, journals and source
hashes are preserved. The strict derivative checker accepts only the reviewed
interface removal and rejects changed physical sources, budgets or instructions;
the actual public v21 demo and model are independently executed.

The actual v21 demo achieved public/native1 in1,043controls (12,516substeps,
52.15simulatedseconds), with1.041667s released hold and all10channels validated.
All four recorded videos decode fully:1,044frames,20fps,H264/yuv420p. The
selected640×360overview shows reset plus every actualAction; its extra renders
verify unchanged robot/task state and counters. It adds no physics/hold credit
and is not a fourth Dream observation. Dream receives only the original
top_cam/left_cam/right_cam320×176 images. Eye(1.05,1.15,1.65)m,
target(.40,-.17,.95)m,18mm focal length and clipping(.05,20)m are declared in the
recording helper. Setup uses the same actual reset frame.

The original actual v21 Dream-1 run completed with score0.00 in3586controls (179.30simulatedseconds),889requests in one continuous session and0 logged transient retries. Inspect summary/record status is success, error is null, and all measured channels/frame identities/source bindings passed. The selected actual model overview contains3587frames at20fps. The task ended at its genuine time_limit with no native failure. The recorded model trial took 27.5min wall time; the arms moved above the fixture without disconnecting the drawer, consistent with score0. Original records/raw frames remain in the complete SHA-verified evidence backups; no failed earlier trial is relabeled as score0. Frozen candidate/task.draft.json retains archival runtime metadata; task.json records this submission's verified results.

For a new independent uncredited physical review, `run.py fresh-physical
--cache /absolute/cache --out /absolute/new-physical` executes the exact native
reference with channels unpromoted. `runtime/build_fresh_review.py` in the
verified payload copies the complete evidence but never approves it. A reviewer
must explicitly review the actual physical/channel evidence before approving a
replacement receipt in a separately assembled and refrozen bundle. Keep the shipped immutable cache unchanged. Source-only or numeric checks do not confer model credit.
