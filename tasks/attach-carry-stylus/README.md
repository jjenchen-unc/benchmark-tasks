# Attach a stylus and pick up a tablet lying flat on a desk

Actual Dream-1 beta: **0.0**, 2400 control steps, one continuous session and no
execution errors. Scripted demonstration: **1.0**, 1685 steps / 84.25 simulated
seconds. Both use the same initial scene, physics, robot limits and scoring.

Place the stylus's flat magnetic face against the tablet's designated magnetic
side, release the stylus, then lift the entire tablet at least 50 mm above the
desk and retain it for 5 seconds. The pen may not receive any hand or external
support. Pushing the tablet to the table edge is one reference strategy, not a
scoring requirement. Either arm may carry the tablet.

The everyday goal is to prepare the tablet and stylus for carrying from a desk.
The tablet starts flush against the desktop, with no spacer, handle or initial
overhang. Its 6.3 mm thickness and lack of access underneath make acquiring a
grasp part of the task. The reference uses the same left arm to slide the tablet
toward the desk edge and later pick it up. Creating that access is coupled with
aligning the stylus's magnetic face and retaining the attachment when both
objects leave the desk. Other pickup strategies remain valid under the scorer.

Fixed initial setup: no case; a 195.4 x 134.8 x 6.3 mm, 293 g tablet approximation
and a 166 x 8.9 mm, 18.2 g flat-sided stylus approximation, based on iPad mini 6
Wi-Fi and Apple Pencil 2 dimensions. Tablet centre (-.04,.10,.75065) m, yaw180°;
stylus centre (.20,-.17,.75055) m, roll90°, on the YAM station, desk top
z=.7475 m. The fixed top camera keeps its calibrated optical pose, supported by
a collidable rear-table metal cantilever: 160 x 160 x 16 mm foot centred at
(.80,0,.7555), 40 mm diameter mast, 32 mm diameter overhead arm, and
18 mm diameter short drop to the camera housing (mount.py). The mast and arm
meet 80 mm above the optical centre. All mount parts remain physical obstacles.
Two YAM arms with dual parallel grippers; original27/7Nm and50N effort limits and SDK speed limits. Shoulder/wrist position gains are tuned8x on the left and4x on the right, damping scaled by square root, with gravity compensation. Tablet metal/glass
chassis and stylus polymer shell use uniform estimated friction0.8. Exact source
geometry and reset poses are in task.py, runtime.py and magnet.py. The screen is
visual only. A supplementary fixed camera records the demonstration; the policy retains the standard three camera inputs. Initial layout uses object_jitter=0; units are metres/kg/seconds.

Current scorer: the flat face and long axis must be within15° of the tablet's
magnetic edge, vertical centre difference<=4mm, surface gap between-0.5 and1mm,
and the pen may extend no more than2mm past either end of that edge. Physical
pen/tablet contact and absence of other pen contact are required. The pen must
first remain attached without hand or fixture support for0.1s while the tablet's lowest
corner is at most2mm above the desk. The desk may support the stylus before lifting because the stylus is thicker than the tablet; desk support is forbidden during either scored carry hold. Any subsequent loss of seating or new hand/fixture support clears this state.
0.5 credit requires all tablet corners at least20mm above the desk for1s;
1.0 requires all corners at least50mm for5s. The tablet must contact a gripper
and not the environment or any arm/base link outside the grippers. Timing is consecutive, and any failed condition resets
the affected timer; score is the current end score, never a historical maximum.
The embodiment ends a successful trial once the5s criterion is met.

Physics: CPU PhysX at2000Hz,20Hz actions, GPU RTX cameras; finite conservative
softened dipoles evaluated on every physics step, with equal/opposite forces
and balanced total torque. No attachment joints, pose snaps or capture flags.
The dipole magnitude/layout, flat section, mass distribution and friction are
estimates, not calibrated Apple measurements. Convex mesh stylus and box tablet
omit small cosmetic features. This simulation tests estimated magnetic retention,
not charging, electronics, real hardware force ratings or real-world reliability.

Commands in the pinned project Arena/Inspect environment:

    python demo.py OUTPUT --stage smoke --reset-check
    python demo.py OUTPUT --stage all --physical-controls --reset-check

A legacy RTX driver requires the explicitly disclosed, process-local
compatibility wrapper `python entry.py demo.py OUTPUT ...`. It does not change
host drivers. The evaluator saves the full robot demonstration before its separate interventions:
remove magnetic force, reset the pen onto the screen, and reset it on the wrong
face. These interventions are never demonstration or model actions. A same-instance
reset follows the controls. The model entry point uses the installed official
Dream adapter, stored frames and a session-continuity gate:

    python model.py MODEL_OUTPUT --driver-check-override
    python analyze_model.py MODEL_OUTPUT

Default: dreamlabs/dream-1-beta, 2400 actions, four rows per request, seed0.
The model sees standard camera images and joint observations only. The audit
observer records physical measurements separately; it never modifies actions.
The actual model run used one continuous session (592 requests), reached the
2400-step limit, and scored 0.0; independent measurement replay agrees.
Reference planning uses simulator object poses and robot joint feedback; it moves objects exclusively through robot action commands.

Provenance: the magnetic model and reference trajectory originate in the local
Pencil/iPad prototype. The task-local Arena spawner/import-hook pattern follows
the project's monitor and bottle adapters, without editing their published
snapshots or the installed SDK. YAM URDF, SDK and station assets remain supplied
by the pinned external runtime. The asset and runtime licenses are listed below.

Task-specific code, procedural object geometry and controller are MIT-licensed
under LICENSE. No manufacturer CAD, logo, photograph or manual is bundled.
Runtime dependencies supply the robot and station: i2rt YAM model revision
120c3c81400171174604e503943f8d1ebc891058 (MIT), yam-isaac-lab-arena revision
0059d02a1a91bc6834c1d0846022891df0c78730 (SDK README states no repository license
yet; separate YAM model MIT license), and Isaac Lab-Arena revision
c7edf3a9d5d8276565208f72422f8f8a0806d6cf (Apache2.0). None is bundled here.
The installed official Dream adapter is used unchanged for model evaluation.

The scripted route pushes the tablet 150 mm toward the desk edge, grasps the
stylus near its blunt end, rotates and seats it with bounded pose feedback,
releases and retracts, then approaches the tablet from above and outside its
overhang before grasping and lifting. Every demonstration movement is a public
14-dimensional robot action; evaluator resets and counterexamples are separate.

Validated runtime versions: CPython3.12.16, Isaac Sim6.1.0.0, IsaacLab17.0.2,
Arena0.3.0, Inspect Robots1.0.0, Inspect Robots Dream0.1.0, NumPy2.3.1 and
SciPy1.17.0. Physics runs on the CPU; an NVIDIA L40S renders RTX cameras.
The tested host driver is565.57.01 with the process-local compatibility flag.

Validation: the complete demonstration starts from reset, releases the stylus
while the tablet is on the desk, and ends with the entire tablet at least
63.703 mm above the desk throughout the final5 seconds. The stylus remains
correctly seated and contacts only the tablet; a gripper supports the tablet.
An independent replay of recorded poses and contacts agrees with the score.
The recorded demonstration stops before the separately logged evaluator controls.
Removing magnetic force then causes detachment and score0. Screen and wrong-face
placements remain unseated and score0 in every recorded control sample. The same
simulation instance subsequently resets both object positions and scorer history.
Results are in OUTPUT/result.json, physical-controls.json and reset-check.json;
The initial frame, the full video and action/measurement trace are retained alongside them.

Run limit:2400 control steps at20Hz. The full demonstration plus evaluator
controls/reset took approximately11.2 wall-clock minutes on the tested host;
the120-second simulated model trial took approximately20 wall-clock minutes.
These are observed timings, not performance guarantees. Seed0, fixed layout,
object_jitter=0. No multi-seed success-rate or real-hardware reliability is claimed.

Render the initial setup from a supplementary camera:

    python entry.py setup_view.py SETUP_OUTPUT

Use SETUP_OUTPUT/high.png. This moves only the review camera after the same
initial reset, with no subsequent physics steps; an exact before/after object
transform check is written to setup-render.json. It does not alter the standard
three Dream input cameras. The selected review eye is(.55,0,1.65)m, looking at
(.08,0,.76)m. The demonstration retains its original fixed overview camera.
