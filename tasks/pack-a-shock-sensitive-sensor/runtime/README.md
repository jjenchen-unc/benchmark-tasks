# D3 Arena integration candidate

This is unfinished task development. The real Arena/YAM scene contains four
floating hinged cartons, compliant pad faces and both tagged sensor boxes.
The task's scoring callback is deliberately gated while actual contact, safety,
tuck/drop and independent scoring observers are integrated and verified.

The retained material and rigid-carton components were checked separately in
evaluator-placed physics tests. Those tests are not a robot full-goal run, a
model run or a submission.

Use the pinned image in ../dependency-pins.json. No dependency, SDK, image or
driver changes. YAM joint_pos actions have 14 entries at 20 Hz. The task-local
motor adapter preserves the total 27/7 N m limits including gravity and the
40 N finger limit. The initial layout is fixed, seed 42, object_jitter=0. No
task.json, media or model score is claimed. All runtime helpers and assets for
this candidate are here.

The passive pad top is a rigid face with global compression, tilt and shear,
four-millisecond loss, and nonlinear densification. Independent corner warping
and foam side collisions are omitted. The physical 30 g pad mass is split into
15 g moving and 15 g fixed; numeric native armature is not treated as physical
mass.
