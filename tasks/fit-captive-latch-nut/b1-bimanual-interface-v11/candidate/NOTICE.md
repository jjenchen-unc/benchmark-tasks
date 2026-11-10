This task's procedural fixture and tool geometry is derived from the project's
standalone B1_wrench snapshot. The original task source, source rubric, ASSETS,
DEVIATIONS and provenance are preserved byte-for-byte in `provenance/`.
`native-sourcehashes.json` records the read-only inputs used for this preparation.
No native robot meshes, Panda assets, native support modules or competitor assets
are included or needed by the Arena runtime.

YAM robot assets remain supplied by the installed SDK. Their actual upstream
notice is reproduced as `provenance/YAM_LICENSE.i2rt`, and its source-commit record
as `provenance/YAM_SOURCE_COMMIT`. No different license is invented for these
assets. Source task generated primitives and the generated hex are not third-party
asset downloads.

The gravity-inside-drive adapter follows the locally verified B5 implementation
from actual contact/motor verification 43386. The task-scoped USD primitive
authoring and contact reduction helpers were adapted from local port code, with
their own new B1 checks. That reuse is implementation provenance, not evidence
that B1's backend channels or robot feasibility are validated.

Current runtime versions and source-stack pins are in `dependency-versions.json`.
