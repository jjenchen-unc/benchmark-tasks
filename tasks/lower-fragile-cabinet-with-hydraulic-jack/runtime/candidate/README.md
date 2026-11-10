Hydraulic jack Arena preparation

This is a concrete, unvalidated portable Arena port of the original representative
`J2_lower_cabinet`, setting S, fixed physical seed42. It is not submission ready.
Actual Dream-1, successful genuine robot execution, physics and media are unverified.
No native scores transfer. Other original goals remain in the read-only native
packages; `variant-preservation.json` lists them without adding task counts.

Run from this directory, using the pinned installed YAM/Arena stack recorded in
`dependency-versions.json`. Runtime imports no MuJoCo, native workspace or Panda
assets. `extract_geometry.py` is an optional offline authoring tool requiring the
immutable native source and MuJoCo3.3.7; it is never imported by the runtime.
Generated USDs and JSON are shipped. USD authored with pxr25.5 in the existing
IsaacSim6.1/Arena0.3 environment; installed metadata reports isaaclab17.0.2.
These versions are recorded as found, without installing or editing dependencies.

On an allocated GPU runtime (CPU PhysX and GPU RTX cameras):

```
python -B foundation.py --mode foundation --out /absolute/new/output
python -B foundation.py --mode reference --out /absolute/new/reference
python -B foundation.py --mode time-limit --out /absolute/new/termination-check
python -B evaluator.py --out /absolute/new/evaluator-negative
```

The optional `--legacy-rtx-override` on foundation.py disables only the current
process's RTX driver-version check through AppLauncher. It edits no host state.
`gpu-request.json` in the delivery directory requests a first root-serial-only
allocation; authors have submitted no compute job. Runtime wall time per trial
is unmeasured. The full trial budget is160.5 simulated seconds. Never run physics
on the login node. The reference reads simulator state and fixed hydraulic
parameters, but sends only genuine14D20Hz joint-position robot actions. It has
no task pose/joint/force/history/score write calls. Evaluator-only direct valve
opening is confined to evaluator.py and does not demonstrate robot feasibility.

An ordinary Inspect route, once task channels are actually validated, is:

```
inspect-robots run --embodiment yam_arena -E task=task.py -E device=cpu -E object_jitter=0 -E control_hz=20 --policy dream --instruction "Lower the cabinet onto the rails gently and let go for3s." --scorer task_score
```

The `dream` entry point was checked in the SDK pyproject; the authorized model session must be verified by the
root before invoking any model. This author has made no inference call. The
required final actual Dream-1 route is not waived by the native Panda morphology.
The state-aware teacher is a separate driver, not `yam_scripted` and not a model.
`score(objects)` exposes only gated task score; private history is not added to
policy observations. StepResult diagnostic info and local evaluator output are
for verification; model observations remain station RGB plus14D joint_pos.

Setup and physical inventory

`physical-inventory.json` enumerates every source primitive, local pose,
quaternion, dimensions, RGBA, mass, inertia, material friction, joint, and fixed
physical parameter. `fixture.json` is the runtime geometry source. The native
scene is translated by(+0.12,-0.30,-0.0525)m with no rotation or scaling. Standard
station table top is0.7475m; original table top was0.80m. No robot placement or
support adaptation is made. The complete rail/support/anti-slide-lip geometry
and both jacks remain. Bases and rail supports sit on the physical station table.
The cabinet is a free rigid body, never fixed to a jack or the world.

Two900x30x30mm steel rails run from y-0.60 to+0.30m, centered x0.085/0.255m,
tops0.8825m. Six24x24x105mm support posts hold their undersides. Outer12mm lips
retain the source lateral fit. Jacks are centered(0.17,-0.15)/(0.17,+0.15)m,
base120x100x12mm, cylinder diameters66/80mm, saddle110mm square. Each handle is
180mm with a60mm free sleeve; each40mm release knob is on its original extension
rod. The cabinet plinth starts at(0.17,-0.15,0.9325)m, exactly50mm above the rails.
Walnut frame, four glass side panels, two glass shelves and four ornaments are
retained as listed in inventory. Its sampled weight/mass is in fixture.json.
Only the representative red jack initially has55mm ram extension.

Source rails use sliding friction0.35, pad1.2, knob2.0, cabinet plinth0.4;
remaining source inherited coefficients are exported per primitive. The new
backend uses equal static/dynamic coefficients and restitution0, not source
MuJoCo contact softness/torsional/rolling equivalence. The standard robot uses
its genuine SDK assets and gains, with40N fingers (below SDK default50N), and
27Nm on joints1-3/7Nm on4-6 TOTAL including gravity. The task-local adapter
puts gravity inside the capped implicit drive; actual get_dof_max_forces and
gains must agree or it raises. Native Panda80/12Nm capability is not preserved
by pretending YAM is stronger. Feasibility under the lower capacity is unproven.

The first nominal placement failed valve IK. Both the failed and translated
results remain in readiness artifacts. Representative approach and42 sampled
valve-waypoint IK checks pass on the actual SDK URDF. Some blue-jack regression
waypoints fail. No collision clearance, reachable full motion, load stability or
torque feasibility is inferred from IK. No dimensions or limits were relaxed.

Reset and scoring

Every Inspect seed uses the same explicitly fixed seed42 physical case and
unchanged starting properties/layout; object_jitter must be0. Episode identifiers
and seed routing do not secretly change hydraulic parameters or expose them to
model observations. Reset restores actual jack joint defaults and free cabinet
root through Arena reset; the foundation driver checks two harness seeds.
A step after termination is rejected before physics. Nonfinite/wrong-size actions
are rejected;3210 controls end the episode. Contacts/clock/history sample at2ms.

`scoring.py` implements the numeric rubric in task.draft.json. Safety failures
remain permanent. No partial progress is earned by hand-lowering the cabinet.
10/30mm descent stages require actual hands-off pad support and measured oil
flow; gentle jack-mediated landing earns0.75;3s released rail support earns1,
with linear speed<=1mm/s and angular speed<=1degree/s throughout that hold.
These explicit rest thresholds tighten the source contact-only hold predicate;
they do not provide new native success evidence.
Landing speed is the last pre-contact downward root velocity. The conservative
soft30N wrist guard terminates the episode rather than silently continuing;
the source80N/0.1s abort is also retained in history. Damage, tipping, dropping,
and nonfinite states invalidate all earlier credit. The initial Boolean native
scorer is not represented as partial-credit evidence.

`validated_channels.json` is empty. Static checks, historical source claims and
SDK API reuse cannot enable channels. Root must supply matching actual sensor,
mechanism and motor calibration evidence before any credit can be activated.
The separate `diagnostic_candidate_score` is deliberately not the official
score and cannot be used as a measured model result.

For external judging, use ruler marks on the rail and plinth to measure10/30mm
descent, side video with a millimetre scale and frame timestamps for landing
speed<=25mm/s, a timer for3s release, and a10degree template for tilt. Glass
normal-force threshold requires an instrumented15N witness/force gauge on the
panels, sampled fast enough to catch impact. This is a proposed measurement
procedure, not a claim of hardware calibration. Scoring remains practical but
requires force instrumentation for the force condition; a photo alone cannot
establish it.

Verification and fidelity

```
python -B verify_static.py
python -B verify_guards.py
python -B verify_motor.py
python -B verify_usd.py
python -B check_reference_ik.py
```

These are pure math, static adapter checks, SDK URDF IK, and pxr asset audits;
none executes physics. The cylinder/ram model is conditional hydraulics, as in
the native source: measured valve angle and pad load regulate an oil-volume
target; measured pump motion adds oil with source hysteresis. Ram K200000N/m,
D400Ns/m approximate the stiff native equality. Handle resistance is a smoothed
Coulomb approximation around0.004rad/s; numerical stall behavior is not yet
calibrated. There is no FEM/fluid solver or glass fracture. Contact points within
an expanded glass panel count conservatively as glass; frame boundary contacts
may overcount. All forces, geometry, reset and safety channels need actual tests.

Original geometry provenance and notices are preserved in SOURCE_ASSETS.md,
SOURCE_PROVENANCE.json and SOURCE_DEVIATIONS.md. Product specifications there
were explicitly approximate and not independently verified. Primitive geometry
needs no invented external mesh licence. No Panda meshes are bundled; SDK YAM
assets remain external dependencies under their original notices.

Cached primary neighbors TK-UUSVB7 (jack/stands/wrench, pending) and TK-BUD2WM
(ratchet staged lowering, pending) share abilities but do not explicitly cover
the representative fragile cabinet's15N glass and25mm/s landing conditions.
This is a concrete comparison, not novelty clearance. Refresh primary ownership,
PR status and benchmark comparison before publication. No task.json, media ID,
model score, upload, commit, PR, publication or official acceptance is claimed.
