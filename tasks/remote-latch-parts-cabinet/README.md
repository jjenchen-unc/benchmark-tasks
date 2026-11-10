# Retrieve a parts box from a remote-release cabinet

Open cabinet A's right door and leave the blue parts box upright on the table outside it, released and stable for20seconds. Cabinet A is the negative-Y cabinet. This submission uses one fixed physical case, source42, for every harness seed.

Actual results on the frozen source: Dream-1 scored **0.0** after3,646 controls,904 requests in one continuous session; the independent scripted demo scored **1.0** after2,433 controls and20.04seconds of released stable hold. Both completed without evaluation error. The model did not release a cabinet latch. All task physics, scoring, actuator limits and native control budget are shared. The full raw model backup and independent payload verification are retained separately; no model frame archive is committed.

The frozen core is `s5-static-table-portable-v13`. Its internal draft metadata and development annotations preserve history; the current public specification is this README and task.json. The scripted controller reads simulator state and acts only through genuine14D YAM controls.

## Reproduce

Use the pinned IsaacLab/YAM environment, source its yam.env, and run with its Python interpreter. Provide Dream credentials outside this task directory. A GPU is needed for the three RTX cameras; rigid dynamics use the supported CPU backend.

```sh
python -I -B run.py prepare --cache /absolute/external/s5-review-cache
python -I -B run.py setup --cache /absolute/external/s5-review-cache --out /absolute/fresh/s5-setup
python -I -B run.py demo --cache /absolute/external/s5-review-cache --out /absolute/fresh/s5-demo
python -I -B run.py model --cache /absolute/external/s5-review-cache --out /absolute/fresh/s5-model
```

The loader pins the immutable external physical-review archive, every archive member, the approved receipt and all shipped core/entry bytes before SDK initialization. Use --archive with the exact release asset for offline operation. Outputs and cache must be outside the source tree. Each run output must be fresh. The model command runs its strict offline verification and finalizes the three videos automatically; model-verification.json reports its measured result. The actual native replay supporting the receipt remained publicly uncredited; enabling its reviewed channels requires this source-bound receipt.

Controls are absolute six-joint targets plus normalized parallel-gripper opening for each arm,20Hz, with25 physics steps of2ms per action. The limit is3646controls/182.3simulated seconds. Allow roughly30minutes wall time per trial on the tested RTX environment; rendering and model latency vary. Dream uses dreamlabs/dream-1-beta, four action rows per request, one uninterrupted episode, CLI seed20261109 (Inspect derives actual reset seed3938811775, passed unchanged to the SDK), no reset recovery or session restart. All three native RGB views are320x176 and are stored.

## Physical setup

The table is1.10m along X by1.30m along Y, centreX=.36m, topZ=.7475m. The robot bases are fixed upright at left(-.08,-.50,1.00)m and right(0,-.305,1.10)m. Steel-like120x120mm pedestals rise252.5mm and352.5mm above the table, respectively. Both arms actually reset to their READY posture with open fingers; the teacher then commands the left parking posture through real actions. The initial left parked pose in the preserved internal draft is historical and does not describe the actual reset.

Cabinet front-plane centres are A(.25,-.29,.7475)m and B(.33,.30,.7475)m, facing-X. Each rigid steel-like enclosure is360mm wide,180mm deep and500mm tall with two176x478x12mm doors, cylindrical pull handles and two spring-return side paddles. There are two loose50x50x60mm,.31kg tabbed boxes in each cabinet. A's right blue box is the target; its settled initial centre is approximately(.315,-.378,.7875)m. The projecting tab makes its total envelope87mm along X. The complete primitive sizes, local poses, masses, friction, visual materials and joint data are in candidate/fixture.json and candidate/inventory.json. Their source body positions receive the declared translation(+.15,0,-.0525)m when instantiated; support coordinates are already world coordinates.

Paddles have12mm travel and release their mapped latch at9mm. In this fixed case A's front/first paddle releases its left door, and its rear/second paddle releases its right door. B's first paddle releases its right door and its second releases both. A-right is a two-stage latch; the other three are single-stage. Latch state, kick-off spring, recatch and gasket/detent torque are passive environment mechanics. The native task starts with every door stage2 latched.

Measured drive ceilings are27/27/27/7/7/7Nm per arm, including gravity, and40N per finger. The task-local adapter puts gravity inside the existing capped implicit PD target and applies zero separate joint effort. It checks the installed arm caps/gains before acting; the credited demo additionally records all eight caps on both arms.

## Numeric score and reset

Score the current state, with permanent zero for a frame-stop impact faster than.25m/s at the door knob or measured untared wrist force above80N continuously.1s. Otherwise:

- .25: the target rear-paddle release has occurred and A-right is at least7degrees open.
- .5: the target box is supported by actual table contact, no robot body touches it, its full box/tab envelope lies over the table, its centre lies outside the cabinet exclusion rectangle, centre height777.5+/-4mm, upright within15degrees and combined linear/angular speed norm at most.05.
- 1.0: every box condition holds continuously20seconds.
- Other outcomes:0.

The exclusion rectangle uses localX(-.02,.20)m and |localY|<.20m relative to(.25,-.29). The speed norm combines m/s and rad/s as in the preserved native rubric. Overload/slam vetoes are sticky; loss of any box condition resets the hold. A moved or assisted box cannot earn full score merely by retaining an earlier success.

Reset restores all object/joint poses and velocities, latch and damage state, histories and counters. Passive settling precedes the evaluator clock. Nothing is consumed. Episode seeds pass unchanged through the SDK; source42 physical properties and layout remain fixed.

## Fidelity and relevance

Rigid primitives approximate the cabinets, boxes, contact/friction, springs and gasket response. A conditional hinge-limit latch represents remote cable release and recatch; neither a cable nor rotary-claw geometry, fracture or gasket deformation is simulated. Mechanism/force estimates are not a manufacturer's exact product model. Simulation channel review is not a physical hardware calibration. The elevated arm supports and state-aware teacher are explicit adaptations. Model and demo use the same geometry, limits and scorer.

Remote rotary-latch systems connect an actuator to a separate latch by cable, with flexible placement and single/multiple actuation points ([Southco system overview](https://media.southco.com/media/static/documents/brochures/20200717-BR-Rotary-WEB.pdf), [actuator applications](https://southco.com/en_any_int/latches/push-to-close-latches/remote-actuated/ac-actuators)). The task represents maintenance access to a parts compartment using that dependency. Cabinet retrieval, two-hand latch operation and remote release already appear in other tasks; the narrower mechanism comparison and its limits are in importance-review.md. Final importance and ownership remain reviewer judgments.

The recorded demo/setup views are visibly grainy; the model views are cleaner. All three cameras use the native320x176 RGB resolution. The left wrist provides a closer view of the box; parked wrist views can show only table, and the top view is partly occluded. Submitted media combines all three synchronized observations with a static final-result panel; no camera frames are fabricated, repeated or trimmed.
