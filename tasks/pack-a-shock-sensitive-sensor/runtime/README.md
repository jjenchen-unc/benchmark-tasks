# D3 Arena integration candidate

This is unfinished task development. The real Arena/YAM scene contains four
floating hinged cartons, compliant pad faces and both tagged sensor boxes.
The task's scoring callback is deliberately gated while actual contact, safety,
tuck/drop and independent scoring observers are integrated and verified.

The retained material/rigid-carton components have scoped physics evidence under
artifacts/submission-readiness/d3-arena-preparation-20261109 on the canonical server.
Those evaluator-placed tests are not a robot fullgoal, model run or submission.

Use the existing pinned UNC image. No dependency, SDK, image or driver changes.
Genuine YAM joint_pos actions have fourteen entries at20Hz. Task-local motor
adapter preserves total27/7Nm including gravity and40N finger ceilings.
The initial layout is fixed, seed42, object_jitter=0. No task.json/media/model
score is claimed. All runtime helpers/assets for this candidate are here.

The passive pad top is a rigid face with global compression, tilt and shear,
four-millisecond loss, and nonlinear densification. Independent corner warping
and foam side collisions are omitted. Physical30g pad mass is split15g moving
and15g fixed; numeric native armature is not treated as physical mass.
