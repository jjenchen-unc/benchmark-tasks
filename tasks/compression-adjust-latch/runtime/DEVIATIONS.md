# Fidelity and unresolved evidence

Source: standalone design-lab S4 compression-adjust/CL-T1 snapshot, seed42.
All native files remain unchanged. No native simulation is embedded in Arena.

Geometry, shape masses/inertias, source joint axes/ranges/armature, gasket
stiffness33416.5N/m, damping200Ns/m, 6.4mm cam lead to172deg, 0.15mm overcenter
backoff, 1mm/turn screw, 3Nm persistent cam damage, gasket bottoming8x over the
last0.5mm, clamp20N, gap1/2mm and20s hands-off retention are retained. Source
collision masks are represented by pair exclusions; pawl touches only flange.
A rigid translation(+0.36,+0.15,-0.0525)m maps the native0.8m table to the actual
Arena station0.7475m table. This does not scale geometry or change axes.

The native dynamically relinearized equality is replaced with a balanced,
finite compliant generalized-force constraint, computed each2ms physics step.
The scalar backward-Euler denominator uses disclosed approximate effective
inverse inertias (including original numerical armature), not the live full
articulation mass matrix. No pose/velocity is overwritten. Error stiffness1e7
and damping1e4 retain the source nominal equality coefficients but are not a
proof of equivalent loaded behavior. Contact, cam reaction, overcenter retention,
load-dependent screw lock and damage must be checked in the actual backend.
Screw locking is a stiff finite restraint, not an ideal equality. Knob position
is captured by the environment only upon measured clamp>2N, released below0.5N.
Cam stripping removes only the passive cam force, irreversibly until reset.
Stop reaction is excluded from the rated cam torque, as in the source.

PhysX contact uses50um contact offset/rest0, Coulomb static/dynamic coefficients
from the source. Native solver compliance/impedance and torsional/rolling
friction are not reproduced by equivalent MuJoCo parameters; these remain
backend approximations. The source visual gasket is noncolliding, with spring
and nonlinear bottoming supplying its force in both routes. Source armature is
numerical generalized inertia, not added physical body mass.

Native Panda(7 joints/arm;80/12Nm) and left2F85(up to235N/pad) are replaced by
standard YAM(6 joints/arm;27/7Nm including gravity;left50N andright40N fingers).
These are lower capacities, different pad shapes and94mm jaw range. The task
physics is not weakened to offset them. The native235N ceiling is not evidence
that a YAM50N grasp can transmit the needed torque. Exact native robot/hardware
capacity preservation is therefore unresolved; full-score YAM demonstration is
required and could fail. No fake235N YAM actuator is claimed. The reference uses
source free-move tip speed<=0.25m/s and yaw<=1.2rad/s; turn rates<=40deg/s(handle)
and90deg/s(knob). It may stop if IK/tracking/physical part-following fails.

No model/public-observation solvability claim is made from privileged planning.
No component calibration or evaluator-prepared hold is robot feasibility proof.
Scoring remains gated without source-bound runtime calibration. Official
robocurve-official-v2.1 gates still require matching actual full-score demo,
Dream-1 evidence, honest metadata/media, standalone runtime and live ownership.


## V3 reach and foundation correction (2026-11-09)

The prior fixed root(0.48,0,0.7475), identity yaw and table-level mounts left
assigned poses unreachable. Installed SDK FK is float64: nine initial-contact
IK seeds bottomed the wrist joint and best position error was31.605mm; this is
not a float32 or SDK defect claim. Broader assigned-pose diagnostics also reject
the old teacher. The new scene applies only a whole-fixture rigid transform to
root(0.28,0,0.7475), yaw-105deg, preserving all relative task geometry, source
materials/masses/joints/scoring/3Nm100ms damage/20s retention. Both actualYAM
mounts rise150mm on fixed240x120mm physical supports; their tops meet base mesh
bottomZ=-1.101112mm. Gravity and the source joint axes are unchanged in local
coordinates. Nominal installed-URDF checks cover89poses (including approach,
turn, retreat and park), with all policy assigned poses reachable. These are
not clearance, loaded tracking, contact, retention or fullgoal evidence.

The evaluator damage stimulus now applies a finite4Nm joint torque to a
prepared short-grip handle. Previously releasing a170deg handle let it backdrive,
so it never sustained the source3Nm100ms threshold. Threshold/mechanism code is
unchanged; no damage flag/state/score is written. The contact probe uses a
reachable local(-0.10,+0.10,+0.1508)m pose and actual same rotated frame. Both
interventions are evaluator-only, never a scripted feasibility demonstration.

Inspect's derived trial seed is accepted while the fixture remains source42;
object_jitter must be zero. Eleven actual integration-wrapper checks pass.
Actual V3 foundation/fullgoal/model results remain pending; no calibration was
promoted by these nominal checks. Old43533 source/results remain immutable.
