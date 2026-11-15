# Prototype mechanism specification

Prototype specification, not measured W90 construction. Native and Dream validation
are recorded in README.md and VALIDATION.json.

The tablet is a rigid180×240×7mm body of480g. Its local+X is width,+Y is portrait
up, and+Z points out of the screen. Each support insert is a rigid134×150×1.3mm
panel of35g. These are an estimated laminated-panel approximation; diagonal
origami creases, flexible skins, adhesive peeling and material damage are omitted.
The first hinge is pre-attached to the tablet; installation is outside the task.

Coordinates below are metres. Both revolute axes are localX. The tablet-side
first hinge is(0,.030,−.006); its panel-side anchor is(0,.075,0). The second hinge
is at(0,−.075,+.0016) relative to panel1's CENTER, and(0,.075,0) relative to panel2's
CENTER. Relative to panel1's first hinge, the second hinge is(0,−.150,+.0016).
All three task-body pairs have collisions enabled. The positive second-hinge
normal offset is essential; a superseded negative offset let the leaf cross its
parent and must not be restored.

Physical angles q1=0,q2=−180° describe the folded stack; the deployed reference
is approximately q1=60°,q2=−120°. Native joint2 uses a closed-zero coordinate
instead, with its local reference rotated180°, to avoid the reset wrap ambiguity.
The native stops are0–143.239° for fold1 and0–159.946° for fold2's reparameterized
coordinate. The joints have zero drive stiffness, damping0.002N·m·s/rad (converted
to native degree units), and PhysX friction coefficient0.006. This coefficient is
not a MuJoCo Coulomb torque. These values are estimates, not manufacturer data.

Two endpoint attraction pairs use panel2-center local(±.045,−.075,0) and tablet
local(±.045,−.120,−.006). Each pair has Gaussian potential
U=−E exp(−distance²/(2 sigma²)), sigma=.006m and E=1.5 sigma exp(.5) joules.
The maximum force is1.5N per pair. Equal and opposite forces and their moments
are applied to the actual bodies. There is no latch, snap, target drive or weld.
This is an estimated magnet approximation, not a claim about W90 internals.

Task-body static/dynamic friction is0.8 with minimum coefficient combination.
The frozen45616 runtime/demo defaults to1ms, matching the selected validation
configuration. The optional0.5ms diagnostic setting is a different configuration.
Actions are20Hz. Shoulder/wrist servo stiffness is
scaled×8 LEFT and×4 RIGHT; damping is scaled by the square root of the respective
factor. Finger actuators and original robot effort/velocity limits are retained.
These settings must travel with the source; changing them requires affected
validation. Exact authoritative definitions are in mechanism.py and runtime.py.

The initial tablet yaw is−90°, centerX=−.04,Y=.10, with its real folded stack
resting on the tabletop atZ=.7475m. Its center is6.65mm above the table. No added
spacer or pre-opened grasp tab is used. The camera housing/support are collidable
and cannot count as valid support. The final screen faces world−X, toward the user.

For real-world scoring, observe the tablet resting on the desk through its bottom
edge and deployed support after both hands withdraw. Measure screen angle with
an inclinometer, portrait/facing alignment from reference views, hand clearance
with a ruler, and five-second pose stability from a fixed camera with scale and
orientation references. Force/contact telemetry is a simulator diagnostic; there
is no hidden fold-angle or magnetic endpoint-distance scoring requirement.

The current full score requires50–70° screen tilt, portrait roll<=15°, facing
error<=30°, hands>=30mm clear, and five seconds within5mm translation/3° rotation
of the first pose in that final interval. Partial credit0.5 covers25–<50° with
the same support, release, alignment and stability conditions. The authoritative
implementation is scoring.py; all other outcomes score0.

For descriptive setup, the tablet is a glass/aluminium analogue, the inserts
polymer-laminate analogues, and the desktop wooden. These are prototype material
labels; physics uses the stated rigid mass, inertia, friction and joint models,
not a calibrated composite-material simulation. The fixed camera foot is
160×160×16mm; mast/arm/drop are capsules of20/16/9mm radius respectively, and
the housing is50×42×22mm. Their endpoints/poses derive from the pinned station
camera pose in mount.py, and they cannot provide valid final support.
