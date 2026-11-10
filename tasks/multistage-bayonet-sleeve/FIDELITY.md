# Fidelity and limitations

This models a custom machined two-pin multistage bayonet sleeve, not an exact
commercial SKU. Original source estimates are retained:58mm sleeve OD,80mm
length,6mm cap,17–27mm bore/track radii,31mm post diameter,6mm pins and250g
sleeve. The physical path includes the original dead end, false seat, blind
ledge and final reversal. No mechanism stage is removed or goal eased.

A single translation (0.23,0.14,-0.0525)m maps every task object, including the
tray, onto the existing station table at0.7475m. No object rotation, scaling,
relative displacement or score tolerance is changed. The robot itself sits on
physical15cm extensions, base height0.90m. That is a documented setup change,
not a joint-limit change. Wrist cameras move with the robot; top camera remains
the station camera, the submitted reset and actual videos document visibility.

The native sleeve's290box/cylinder primitives execute through the frozen
source helper, and all306task shapes are exported individually. There is no
convex hull closing the bore, policy-written latch or pose servo on task parts.
Only the source plunger spring/damper supplies task effort; it has a32mm stroke,
25g body and source numerical slide armature0.001kg. Native
slide armature is numerical and not added to physical mass. Exact slide-unit
inertia handling across engines remains unverified. Source plunger cylinder
mass/inertia is preserved analytically, including its .0033m COM offset.

Materials are estimated aluminium/greased steel and plastic tray. Sliding
friction coefficients are copied per shape. MuJoCo torsional/rolling friction,
contact softness and its0.3s finger-target filter are not equivalent PhysX/YAM
mechanisms; the YAM reference uses gradual jaw targets and stock hardware
velocity limits. The retained foundation evidence checks scoped fixed-case measurement channels;
it does not establish identical contact dynamics across engines. Source contact masks permit all relevant interbody task/robot
contacts; parent-child stand/plunger collision remains disabled. Tray floor
and walls are separated into world-fixed rigid bodies solely to distinguish
actual floor support from wall contact. Their positions/shapes are unchanged.

Rigid parts cannot break or plastically deform. Original native damage()={}
is retained. No FEM, hardware calibration campaign or public-only teacher is
claimed or required. Static source checks alone do not calibrate wrist reactions. The retained
foundation performs known loads and prepared-state diagnostics. The fresh Brev
replay scored 1.0 and the complete Dream-1 trial scored 0.0; see README.md for
measurement scope, preserved infrastructure failure and reproduction commands.
