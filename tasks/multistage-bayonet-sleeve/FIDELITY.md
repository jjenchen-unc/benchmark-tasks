# Fidelity and limitations

This models a spring-loaded two-pin bayonet coupling sleeve with a two-stage
track, not an exact commercial part. The original estimates are kept: 58 mm
sleeve outside diameter, 80 mm length, 6 mm cap, 17-27 mm bore and track radii,
31 mm shaft diameter, 6 mm pins and a 250 g sleeve. The track keeps the
original dead end, false seat, blind ledge and final reversal. No stage is
removed and the goal is not eased.

A single translation of (0.23, 0.14, -0.0525) m places every task object,
including the tray, on the station table at 0.7475 m. No object rotation,
scaling, relative displacement or score tolerance is changed. The robots sit on
fixed 15 cm extensions, base height 0.90 m; this is a setup change, not a
joint-limit change. Wrist cameras move with the robot; the top camera is the
station camera. The reset image and videos show what is visible.

The sleeve's 290 box and cylinder primitives are built by the original geometry
helper, and all 306 task shapes are exported as separate colliders. No convex
hull closes the bore, and there is no scripted latch or pose servo on task
parts. Only the plunger spring and damper act on the task parts; the plunger
has a 32 mm stroke, a 25 g body and a 0.001 kg numerical slide armature that is
not added to its physical mass. How the two physics engines treat slide
inertia has not been compared. The plunger cylinder's mass and inertia are kept
analytically, including its 3.3 mm centre-of-mass offset.

Materials are estimates: aluminium, greased steel and a plastic tray. Sliding
friction is set per shape. The original MuJoCo torsional and rolling friction,
contact softness and 0.3 s finger-target filter have no exact PhysX/YAM
equivalent; the YAM reference uses gradual jaw targets and the stock hardware
velocity limits. The kept measurement checks cover this fixed case only; they
do not show identical contact dynamics across engines. All relevant contacts
between task parts and the robot are enabled; the stand and its plunger do not
collide with each other. The tray floor and walls are separate fixed bodies so
that floor support can be told apart from wall contact; their positions and
shapes are unchanged.

Rigid parts cannot break or deform, and no damage is modelled. There is no FEM
or hardware calibration. Static checks alone do not calibrate wrist forces; the
kept foundation run applies known loads and checks prepared states. The demo
replay scored 1.0 and the Dream-1 run scored 0.0; see README.md for the
measurements, the earlier failed connection and the reproduction commands.
