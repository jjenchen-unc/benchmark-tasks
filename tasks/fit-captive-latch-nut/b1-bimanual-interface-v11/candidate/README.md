# B1 captive-nut wrench service: Arena preparation

This is an executable development port of **W1_latch_release**, not a validated
task or a submission. All runtime credit channels are gated. No YAM physics,
genuine robot full score, Dream-1 run, review media or upload is claimed.

Select and fit an open-end wrench to an unmarked captive latch nut, loosen it
without rounding it, swing the arm open, return the tool and leave all hands and
tools off the nut and arm for 20 seconds. Matching a near-size tool under load is
part of the goal. The adjustable tool is present with its source preset jaw.

`task.draft.json` contains every submission field, with unrun scores and null
media stated explicitly. It is deliberately not `task.json`.

## Execution

Use the existing pinned Arena environment recorded in `dependency-versions.json`;
do not install or modify a host, SDK or image. From this folder:

```bash
python -B verify_cpu.py
python -B verify_assets.py
python -B preflight_ik.py --out /absolute/output/nominal-ik.json
python -B foundation.py --mode foundation --out /absolute/new/output/foundation
python -B foundation.py --mode reference --out /absolute/new/output/reference
python -B foundation.py --mode negative --out /absolute/new/output/negative
```

Only the first three commands are pure/static. The remaining commands require
root's serial GPU allocation for RTX cameras; they use CPU PhysX, one GPU, four
CPUs and 32 GB, with a maximum allocation of 30 minutes. Never run physics on the
login node. `--legacy-rtx-override` is an optional, explicit process-only
AppLauncher flag. It never edits a host/driver configuration.

Every runtime command copies the **whole task** to `out/executed-source`, records
its hashes and command, and re-executes that copied runner before importing the
physics SDK. The byte-identical copied `provenance/w1_scoring.py` actually executes
before those imports. Runtime code imports no MuJoCo, native helper or hidden
workspace. Each run writes `result.json`, tracebacks on failure, action logs and
raw per-physics measurements. A failed run remains evidence, not a retry success.

The Inspect task-file/scoring route, after runtime validation, is:

```bash
inspect-robots run --embodiment yam_arena \
  -E task=/absolute/path/to/this/task.py -E device=cpu \
  -E object_jitter=0 -E control_hz=20 --scorer task_score \
  --instruction 'Fit a wrench, loosen the captive latch nut without rounding it, open the arm and leave all hands and tools off it for 20 seconds.' \
  --policy <actual-authorized-policy>
```

This command documents the route; no policy/model call was made. The final actual
Dream-1 run is required for this dual parallel-gripper robot. Native full-score
history supplies design evidence, never an exemption or a YAM result.

The run limit is **3,870 genuine 14D Actions at 20 Hz**, or 193.5 simulated
seconds, with 25 physical 2 ms samples per Action. Wall runtime is unmeasured.
The complete state-aware teacher in `reference.py` is an unvalidated attempt.
It reads actual robot/tool poses and the fixed source case to plan SDK IK, and
changes the world solely through `emb.step(Action(14D))`. It aborts on IK failure,
workspace violation, safety termination or failure to release the actual nut.
`evaluator.py` is separate: its direct setup placements and external known loads
are diagnostic privileges and can never count as robot feasibility.

## Scoring, safety and reset

The original W1 numeric rubric executes unchanged. Its inputs come from actual
joint/body state, filtered contacts and wrist sensors on every physical sample.

| Score | Conditions, continuously for the final 20 s |
| --- | --- |
| 1.0 | Nut backed off at least 60 degrees, clamp released, arm opened at least 85 degrees, nothing touching nut or arm |
| 0.5 | Same released nut and contact-free hold, with any arm opening |
| 0 | All other outcomes; any rounded nut, invalid measurement or safety veto |

Rounding is sticky for the episode. It opens the head's passive slip joint and
changes its visible colour, as in source. An oversized wrench uses the source
clearance-dependent threshold; an undersized wrench forced within 9 mm and over
0.05 Nm also rounds. Either needs five consecutive 2 ms samples. Thread breakaway
is 0.3524 Nm until more than 2 degrees loosening, then 0.03 Nm running. The clamp
unlocks only after **strictly more than 60 degrees**, preserving the source
mechanism's comparison. It never sets an angle or applies a task-driving torque.

Safety: wrists above 80 N for 100 ms veto the trial. Default wrist guard is 30 N
for 20 ms; a nonfinger moving link above 50 N over a 20 ms window also guards.
Unlike the native tool-call API, the continuous joint API has no tool-call end:
these guards terminate the port episode at the reporting control boundary. This
is stricter than native, explicitly disclosed, and still requires feasibility
under the retained limits. Nonfinger force reduction sums normal and tangential
magnitudes conservatively, pending calibration. No safety threshold is raised.
Tools leaving the transformed source object box also terminate with zero credit.
There is a finite 14D Action check and a no-step-after-termination guard.

Reset restores five **free** wrenches, all task joint positions/velocities, the
locks, original nut appearance, sticky damage, event history and hold clocks.
The physical scene is the single declared **source S/42 case** for every Inspect
seed. Harness seeds identify episodes; they are not rejected or used to leak a
secret parameter into observations. `object_jitter=0` is required. The public
Observation remains three RGB images and the genuine 14D robot proprioception;
the observer's private source state is not added to it. Native W2/W3/W4 and other
settings remain in the read-only originals, with no additional task count.

## Physical inventory and layout

`physical-inventory.json` lists **all 56 original task geometries**, shapes,
dimensions, local poses, colours, masses, friction and collision masks. The five
tools are independent rigid bodies. The fixture has three real joints: nut turn,
head slip and arm swing. The stand and rack are fixed supports as in the source;
no free tool was fixed, resized or attached to the robot.

The **entire task scene**, including rack and all tools, is transformed by
`world = Rz(180 degrees) * native + (0.40, 0.10, -0.0525) m`.
Every relative object distance and orientation is preserved. The native tabletop
at 0.80 m coincides with the station tabletop at 0.7475 m. Robot bases, camera
stand and table geometry are unmodified; no pedestal or fake clearance is used.
The rack jaw centres are at x=0.30 m, y=0.50/0.45/0.40/0.35/0.30 m, z=0.785 m;
handles point +x. The captive nut bearing face is (0.28,0.12,0.8375) m, 12.70 mm
across flats and 6.8 mm tall. Its size is not stated in the policy instruction.
The arm pivot is at (0.28,0.12,0.833) m, with an 80 mm lever and 12 mm knob.
It starts toward -y and opens toward -x. Table/robot/furniture clearance has
**not** been established by the nominal endpoint IK checks.

Steel tool/fixture dimensions are retained from source. Wrench jaw friction is
0.2, handle friction 1.0, nut friction 0.15. Nut thread torques are deliberately
about 1/100 real fastener torque. This models a light practice fixture, not a
full-torque M8 maintenance operation. The preset adjustable worm is visual only.
The nut is captive on a hinge; it has no helical translation or material fracture.
Rounding is a passive conditional slip rule rather than deforming mesh geometry.
Tiny numerical masses on otherwise empty fixed links are disclosed in inventory.

YAM uses actual **27/7 Nm TOTAL including gravity**, with the verified B5
gravity-inside-drive algorithm copied task-locally. Its actual SDK finger ceiling
is 50 N; this port retains the source's lower **40 N per finger**. Native Panda
arm limits were 80/12 Nm. No higher capacity is faked; loaded YAM feasibility
still needs proof. The native down/yaw public motion API becomes the official
six-joint-per-arm YAM interface; the reference uses top-down tool grasps. The
right arm is kept parked as required by the original representative task.

## Repairs and evidence limits

Two concrete source issues are repaired only in the new port:

1. Source prose says tool sizes are stamped, but geometry had only coloured
   bands. The new USD adds visible numeric size decals without collision/mass
   changes. Screen readability still needs actual camera inspection.
2. Repeated floating additions can report 19.999999999999794 s after 10,000
   2 ms samples. Integer sample counters now give exactly 20 s; the copied
   scorer and threshold are unchanged. A retained counterexample verifies this.

The first nominal layout failed actual SDK IK at two poses and is retained in
the readiness artifacts. Only whole-scene translation changed; the selected
layout passes 16 endpoint IK checks. This is neither a swept clearance audit
nor a manipulation result. Pure scoring and USD checks do not activate any
runtime credit channel. Root must validate drive ceilings, contact force/moment
sign, passive friction, correct/wrong tool fitting, rounding, clamp release,
reset and force guards before any full-goal/model evidence is eligible.

## Provenance and overlap

`provenance/B1_wrench.py` and `provenance/w1_scoring.py` are byte-identical copies
of the owned read-only native source. Geometry is compiled offline by executing
the original geometry functions, with no native simulator imports. Runtime uses
only local JSON/USD and helpers. All task objects are procedural primitives and
one generated hex mesh; no Panda asset is bundled. Actual source ASSETS,
DEVIATIONS and provenance records are preserved. The installed SDK supplies its
YAM robot, whose upstream notice and source revision are copied in provenance.

The cached 1,296-record primary board comparison is stored in the readiness
artifacts with complete selected records and a cache hash. TK-CW7VYS has pre-fitted
union/backup wrenches, while W1 requires loose near-size fitting and damage-safe
latch release. TK-LA8TPH uses a pre-mounted tap wrench, TK-GMVZWK a fixed socket
and ratchet clutch, and TK-5D2MTC a gauge and spoked locknut. None of those inspected
instructions covers all original W1 abilities. This is a focused cached
comparison, **not current ownership clearance**. Refresh primary ownership and
comparison before publication. No competitor code was executed.
