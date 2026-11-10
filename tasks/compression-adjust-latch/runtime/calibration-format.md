# Runtime calibration receipt format

The current receipt is bound to the actual43544foundation and43552per-substep load/contact evidence. It validates measurement channels, not robot feasibility.
`observer.py` accepts a local `calibration.json` only after the responsible root
has reviewed actual measurements for this implementation. This is a task-local
measurement gate, not an additional universal official submission requirement.

Receipt fields: `passed: true`, `physics_device: "cpu"`, `physics_dt: 0.002`,
`control_hz: 20`, `source_sha256` exactly matching every file in
`observer.RUNTIME_FILES`, and `checks` containing all seven keys in
`observer.CALIBRATION_CHECKS`, each the Boolean `true`. Preserve measured raw
traces and exact commands as the evidence for those checks; include evidence
paths/hashes as additional receipt fields. A source change invalidates the receipt.

The foundation driver supplies actual per-substep traces for reset, no-contact,
prepared retained clamp/gap, delivered grip negative, loaded/unloaded screw,
short-grip stripping, sticky postdamage failure, applied120N wrist abort and
prepared robot/enclosure contact. Its checks may fail; it does not issue a
receipt. Review the remaining cam-force/wrench sign agreement, actual arm/finger
force ceilings and repeated loaded/damaged resets before promoting channels.
Prepared-state and known-load results remain evaluator-only at every score.
A source-bound receipt never converts those interventions into robot feasibility.
