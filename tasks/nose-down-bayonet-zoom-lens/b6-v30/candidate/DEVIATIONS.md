B6 port deviations and unresolved evidence

- Source: standalone B6_camera--lens_exchange_zoom, representative nose_down_zoom,
  seed42 fixed physical case. Other native goals are preserved read-only. One
  task count. Final official policy is robocurve-official-v2.1; genuine Dream-1
  remains required for two parallel-gripper arms.
- Geometry: no resized parts or frozen free lenses. Every task shape receives
  the same translation (+0.25,0,-0.0525)m. Both real YAM bases are raised to0.90m
  on disclosed 0.20×0.20×0.1525m pedestals. All support, self, cross-arm and fixture
  clearance is unverified. Failed original endpoint guesses are retained.
- Embodiment: native two7DOFPanda/40N hands -> genuine two6DOFYAM/40N hands.
  Native total80/12Nm capability is reduced to27/7Nm including gravity; source
  successes cannot establish feasibility at lower YAM capacity. SDK finger max
  is50N; this task deliberately retains40N. No motor capacity is increased.
- Action route: native500Hz residual torque/position servo and pose tools ->
  actual14D absolute joint/mean-jaw actions held20Hz, with500Hz physics. No model
  exemptions are inferred. The privileged reference uses actual SDK IK and
  stops at30N; all policies retain the80N100ms sticky episode abort. Native
  optional force_guard settings up to60N are not exposed by this14D interface.
- Passive capture/lock: source weld equalities -> physical conditional USD fixed
  constraints at current frames, with source capture geometry/contact checks,
  click/button/release rules and bounded centring. Dynamic enablement, closed
  articulation loops, lock reaction basis and contact-force sign need actual
  PhysX checks. No body pose writes occur in policy or passive step logic.
- Source visual lugs, neck/index and ring details remain visual. Only original
  feet/face/shell contact masks become colliders. This is a surrogate bayonet,
  not a fully geometric interlocking-lug model, in both native and new port.
- Zoom source tight linear equality -> balanced finite-stiffness helix potential,
  K=20000N/m,C=20Ns/m and h=.02/(pi/2)m/rad. Native joint ranges, masses,
  friction/damping/armature are retained. Native torsional friction and PhysX
  friction schema behavior require calibration; no equivalence claim.
- Pull-out contact force currently omits tangential friction (normal contact
  data only). Gravity and source filtering/thresholds are retained. Nose-down
  index release, off-index retention and false seating are not proven.
- Static fixed bodies have1e-6kg numerical mass where needed by articulation or
  kinematic rigid APIs; no free task mass is changed. Default solver and rest/
  contact-offset differences are disclosed in build_usd.py and require evidence.
- Source validation bug repaired only in this new port: negative radial/tilt/
  lock-angle-error magnitudes previously passed full score. Original copied
  scorer actually returns1.0 for each counterexample; new scorer returns0.0.
  Native files are untouched. Valid0/.5/1 outcomes and thresholds are unchanged.
- State/history are private evaluator channels, sampled every actual scene update.
  Required runtime-channel validation starts empty. Raw rubric cannot activate
  official task credit. Runtime step-after-termination and derived-seed reset
  are implemented, but need actual foundation execution.
- No Arena physics, full-goal reference, model call, media, upload, PR or live
  publication clearance was performed by this author. Historical native full
  score/reviews are design evidence only. Root must validate actual final bytes.
