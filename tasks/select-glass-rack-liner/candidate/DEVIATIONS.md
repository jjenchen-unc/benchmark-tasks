# Fidelity and unresolved mechanisms

The original dimensions, masses,20degree slope, material distributions, wet
volume,0.5/10/20second timing,5mm/15degree hold tolerances,30mm drop/60degree
veto, two-glass distinctness and2268control budget are unchanged. No source
physics was edited. The proper rigid world transformation is Rz(-90degrees),
translation(0.40,0,-0.0525)m; native table0.80m becomes station0.7475m. There is
no geometric scaling. Native rack Rx20degrees is composed with that rotation.

The robot is the installed dual6joint YAM with parallel grippers, rather than
native dual7joint Panda. Its actual27/7Nm total caps and40N fingers are declared
and must be checked on this scene. The nativePanda80/12Nm capability is NOT
claimed for YAM. Reference planning may use either YAM when the preferred left
liner route is unreachable. Task/source scoring has no arm-owner predicate.
Native tool point is25mm above finger bottoms; YAM TCP is13mm above them. The
reference explicitly adds12mm world-up to the YAM TCP before checking the
original source tool workspace; it does not lower the physical finger reach
bound. Top-down grasps retain free yaw. Source jaw0..90mm is implemented by saturating public commands within
YAM's actual0..93.9mm travel, resetting the open pose to90mm and retaining
the0.3s filter/0.15m/s total gap rate. All six YAM joint speeds are capped at
the conservative native2.175rad/s bound (native distal joints allowed2.61).
The arm command slew cap is0.85times2.175rad/s as in the native wrapper.

MuJoCo explicit cup/liner pairs are represented by two coincident colliders on
each SAME rigid body. Main cup(.30) and rubber liner(1.0) shapes are excluded
against the opposite class; pair-only cup(1.0) and liner(mu) use min combine.
Pair-only shapes exclude every other collider/robot. Thus each external pair
has exactly one active collision shape pair; inertia/mass are explicitly set,
not inferred twice from duplicated volumes. This exact shape-filter mapping
must be verified in root's actual PhysX run: presence of USD relationships alone
is insufficient. If the backend cannot support shape exclusions as authored,
that is an unresolved mechanism; no averaged friction substitute is authorized.

The source describes approximately0.3mm compression under a0.2kg glass, using
MuJoCo solref0.01/solimp0.95,0.99,0.001. PhysX uses a force spring6540N/m and
critical damping72.33Ns/m on liner materials, derived from that compression and
glass mass. This is an explicit approximation, NOT a MuJoCo solver-equivalence
claim. Real PhysX slip/hold at the source coefficient bands, impact/release and
20second creep are unresolved pending the supplied evaluator friction suite.
The cup is the same solid collision cylinder approximation with0.8times solid
cylinder inertia and CoM7mm below centre. Water is visual volume plus the same
irreversible centre-entry material rule, not fluid simulation.

Contact safety is sampled at1ms through actual raw PhysX actor identities.
The nonfinger link observer sums normal contact magnitudes plus tangential
magnitudes per link before its20ms window; native sums per-contact vector
magnitudes. This is conservative but not exact when normal and tangential
components coexist. The50N bound is not increased. A matching vector-magnitude
aggregation/calibration remains unresolved and keeps the link channel unverified.
Wrist force uses incoming gripper sensor-force norm, whose coordinate rotation
does not change magnitude; it still needs actual C1 known-load/abort checks.
No geometry-distance-only contacts, fake wrist readings, fitted hold clocks or
policy-owned state changes substitute for missing measurements.

No port physics, robot fullgoal or model score is proven in this bounded coding
pass. Native43394 score1/1598controls/25.407second hold is source evidence only.
Global board ownership, Dream-1, model media, blind review and publication are
root-owned later gates under robocurve-official-v2.1.

## v2 refinement (supersedes v1 mounting and force-aggregation notes)
Task objects retain the original rigid Rz(-90degree),offset(0.40,0,-0.0525)m
layout. Only the robots move: left base(0.36,0.20,0.8975)m,yaw-90degrees;
right base(0.40,-0.36,0.8975)m,yaw+90degrees. Each is bolted to a real
240x120mm world-aligned pedestal on the unchanged station table, with height
148.898888mm. Pedestal top equals the actual SDK base collision-mesh bottom
(base local Z=-1.101112mm). The202.201x72.202mm rotated base footprint fits
that support. No unsupported floating base or invisible support is introduced.
These are declared candidate mounts; actual C1 measured mount/FK and safety
remain pending the first foundation, not inferred from prior D3 measurements.
Pickup yaw180degrees is an equivalent downward grasp across the same liner
width; held objects rotate horizontally to delivery yaw0. Right retreat is
100mm toward-X,80mm toward-Y and60mm up to keep the two-glass route reachable.

The nonfinger observer now bounds the native sum of contact-vector magnitudes
by sqrt(1+mu_bound²)*sum(normal_magnitudes), before the unchanged20ms/50N
comparison. mu_bound is derived from actual stage static/dynamic coefficients,
including a conservative bound for multiply combine and the installed default.
Under the solver Coulomb constraint |t_i|<=mu_bound*N_i, the bound follows
per contact and is preserved by summing/time averaging. Friction anchors have
no established1:1 map to normal contacts; no exact vector norm or force
calibration is claimed. Opposing tangents cannot cancel this bound. It may stop
motion earlier than native, and that approximation is explicit; no threshold
is raised and no averaged-friction fallback is used.

All runtime runners freeze and verify the complete source before task/SDK
imports. The proven AppLauncher Kit argument disables only the current process's
RTX driver-version check. No driver/image/installed SDK is edited. Only a
first foundation allocation is requested:1GPU,4CPU,32GB,maximum30minutes.

Private parking-controller-v5: one right-arm parking insertion after original
control169, holding the observed left precontact pose; no changes to task bodies,
materials, dynamics, timestep, caps, force thresholds, scoring or total budget.
Geometry assumes the measured8450 held-liner transform for sampled planning;
physical grasp retention and later full-goal execution remain unproven. The
state-aware planner retains the disclosed hidden-friction choice from the full
reference. Generic-case execution is not supported by this private DATA path.

The original20Hz remaining rack-hover targets are temporally subdivided after
parking, retaining0.045rad/action and reaching the unchanged target rather than
accumulating clipping lag. The selected right park is[.60,-.30,1.11]m and its destination is
inside the original transformed workspace. The rejected[.55,-.25,1.12]m path
is retained outside the candidate as failed planning evidence.

Private runtime-workspace-v6 corrects an added per-waypoint gate which rejected
its own measured HOME-derived start before any parking action. The native
snapshot declares a Cartesian tool reach box but never reads WORKSPACE after
its definition; its documented HOME starts above it. Trial.move checks the
Cartesian destination, not every intermediate FK point. Parking now uses that
same destination predicate with unchanged bounds and additionally rejects any
worsening component of the inherited excursion or failure to end inside.
The identical201-point path starts at source-tool Z1.218804m, enters the box
at point157 and finishes at Z1.174500m. No trajectory or other phase changes.
The12mm world-Z virtual-tool offset is the existing port convention for its
source downward tool; it is not an orientation-dependent pad reconstruction
for the tilted parked wrist. Rigid native/world A rotation is not wrist rotation.
CPU fake-action/FK and finite STL checks are conditional planning/control-flow
evidence only; they do not establish physical retention or full-goal success.
The reference now bootstraps its directory explicitly for the chosen python-I
launcher before importing c1_launch. All reference function ASTs are unchanged.
