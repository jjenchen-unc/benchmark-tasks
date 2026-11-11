# Backend changes and evidence limits

DL-T2's final goal, individual fastening tiers, damage vetoes,7964action
limit,20second continuous final hold, and40N source fingers are preserved.
Source full scores and historical review are design evidence only.

- Source MuJoCo task-only XML was compiled offline and evaluated by forward
  kinematics, without mj_step. Geometry/masses/axes/initial joint coordinates
  were exported into fixture.json and task-only USD. This is not PhysX proof.
- A single proper rigid transform moves all task objects and their plinth into
  the standard YAM station. No retained robot pedestal, size changes, fixed
  cases, artificial motor increase or lower tension threshold is used.
- Native80/12Nm Panda becomes actual SDK27/7Nm YAM. Total PD plus gravity
  is bounded by the existing drive through G/K.40N source finger force remains
  below SDK50N. Lower capacity and six rather than seven arm DOF are real
  feasibility gaps; they are not justified by source success.
- Springs,400N/m bilateral slide term, tension-only spring deadband and damping
  are passive task effort laws in SI units. Native absolute dry friction now uses the pinned PhysX tensor API's static
  and dynamic friction EFFORT slots, both set to the original frictionloss
  (0.04 Nm levers and 0.8 N sliding catches); viscous slot is zero because
  the original SI damping remains applied separately. The earlier explicit
  tanh approximation is removed. Actual16-slot property readback, passive5second settling and
  low/high additional joint-effort component tests pass on the pinned backend;
  this is not an exact breakaway calibration or full-task proof.
  Native torsional/rolling friction and MuJoCo contact solver compliance are
  not mapped exactly into PhysX's Coulomb friction/contact offsets. Calibrate
  hook force/closure, catches, free-case motion and contacts before enabling
  any credit channel. Do not lower stiffness/limits to obtain success.
- Native conditional hook connections become preauthored spherical joints
  excluded from the reduced-coordinate articulation. Hook capture requires
  actual hook/lid contact, all native shape masks and2.5mm bar proximity for
 150ms; rearm/release and catch-limit rules are retained. Runtime USD enable
  notifications/loop stability and keeper contact identity remain unvalidated.
- Primitive sizes, poses and physical masses include the original instrument
  cap colliders. Both free instrument roots and both floating cases remain
  physically free. Fixed catch tabs on the no-catch case B use fixed joints
  to its case root, reflecting their native welded relation.
- Compiled source case mass1.82kg conflicts with old rough2.1kg prose;
  the port uses compiled1.82kg. No approximation is presented as product data.
- Wrist force uses actual JointWrenchSensor gripper-body channels. Target
  hands-off uses actual filtered tensor contacts. Nonfinger guards use a
  conservative sum of normal magnitude and friction magnitude, then the
  maximum per-arm body, averaged20ms. Compared with native per-contact
  vector norms this may stop sooner. Runtime known-load/contact/clock/drive
  audits are required; no calibration is inferred from API existence.
- SDK reset includes0.5s physical settling. Springs and initially connected
  B hooks act during settling; history/control count begin afterward. The
  resulting start layout needs actual comparison. The fixed physical seed42
  accepts arbitrary Inspect reset seeds/episode IDs with jitter0; it does
  not reject all seeds except42 or encode private parameters in observations.
- Native push-on-lid orientations failed nominal SDK IK. The private robot
  teacher instead grasps the unchanged lip with a30degree upward approach,
  pulls to60degrees and opens while lifting120mm before retreat. Source COM
  analysis rejects the old75degree gravity-release assumption. Bounded IK,
  local adaptive subdivision and finite individual collision-part checks are
  conditional controller improvements; actual grasp/contact/stability and the
  remaining whole sequence still need fresh physical validation. Robot bases
  and the loose fixture layout are unchanged; outward reset joint1 targets
  remove the measured reset fingertip/lid overlap in CPU geometry only.
- Physics/registration/reference/evaluator scripts are executable candidates,
  not validated backend evidence. VALIDATED_CHANNELS is deliberately empty.
  Model RGB/robot observations do not receive private task state; rich
  diagnostic info is opt-in only in the evaluator/reference runner.

- Actual v24 capture telemetry found the YAM right fingertip simultaneously
  loaded the lever tab (6.19 N in the closest-pin sample) while applying only
  0.036 N to the hook. A 55-degree tilted push changes only the robot teacher
  approach: individual convex-part checks at four recorded mechanism states
  avoid that lever overlap and the keeper bars, whereas the previous vertical
  orientation overlaps the lever. This is conditional geometry evidence, not
  contact/dynamics/capture success. All physical task geometry and rules remain.

- The actual55-degree approach reduced unintended lever loading to5positive
  contact rows and reached2.985mm tip-pin distance, but stopped1459controls
  before capture on the unchanged robot/keeper-bar clearance check. New75-degree
  teacher tilt is selected by realURDF geometry at four actual poses and four
  explicitly predicted native link/slide inverse poses: it clears all8 queried
  poses with a critical task-surface lower bound285um. Nearby73/79-degree
  alternatives collide or fail the same guard. These hypothetical poses and
  CPU checks are not physical capture, trajectory or task-success evidence.
  Native mechanism, geometry, tolerances, forces, friction and budget unchanged.

- The private teacher clearance classifier now permits left fingertip contact
  with only the three keeper bars of the latch being captured. The native task
  does not forbid this operating contact. V26 stopped on a conservative finite
  geometry bound near a bar at1459controls while native force/tilt guards were
  clear; the hook had not captured. All other scene/self/arm checks remain, and
  keeper-bar collisions, forces and native hook/lid-contact+dwell predicates
  remain physically active. This is not collision-filter, geometry, damage,
  tolerance, force-cap or scoring relaxation, and does not prove capture.

- Native DL-T2 seed42 has no catch on case B. The previous port incorrectly
  set B lever upper stops to2.6degrees and marked them locked. B levers now
  retain their native fixture range[0,1.9rad] in both the authored asset and
  reset. A latch/catch rules, controls, springs, friction, capture masks,
  geometry, scoring and7964-control budget are unchanged. New B-limit runtime
  readback and a matching full replay remain required; no success is claimed.

- Private catch finishing controller draft: retain last public arm targets
  for up to40 settling actions, withdraw outward with unchanged jaw, then
  open jaw while holding both arms. Check native1.5mm catch home after
  settling/withdrawal/parking. All18N teacher stops, native7964 budget,
  mechanics, collision masks and score thresholds remain. This source has
  NOT yet passed realIK/caller/whole-budget CPU review or actual fullgoal.

- V30 private egress target: after catch jaw opening, request fingertips3mm
  farther above the measured lid envelope before yaw/fold. V29 nominal100um
  target left28um commanded bottom margin and rejected a77.8um yaw gap.
  The finite-geometry100um requirement, native goal, physics, forces, joints,
  controllers before this retreat and7964 budget remain unchanged. This is
  a planning margin, not a relaxed task rule or proof of dynamic tracking.
