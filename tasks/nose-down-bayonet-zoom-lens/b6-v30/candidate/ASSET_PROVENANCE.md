All seven USD files are generated task-specific primitives from the actual
compiled B6 native scene, with no physics stepping. `fixture.json` records the
source scene SHA256, exact source seed42 parameter draw, original body/shape
poses, mass/inertia and joints. `build_usd.py` is the reproducible offline author.
The original task scene has75body shapes plus2world bracket shapes. No Panda,
Robotiq, competitor mesh/code, texture or native runtime is bundled/imported.

Source lineage: ports/design-lab/fit_tool_service/prototypes/
B6_camera--lens_exchange_zoom/scene/B6_camera/benches/B6_camera.py, independent
snapshot of tinkerbench_build_28benches_2026-10-31. Exact original ASSETS.md,
DEVIATIONS.md, PROVENANCE.json and scorer/bench copies, all source hashes and
static export command are preserved in the owned readiness source-snapshot.
Those files are provenance/design evidence, never runtime dependencies. The
source-native build (MuJoCo3.3.7/numpy2.0.2/scipy1.13.1) was only statically
compiled to extract primitives; no original robot assets are redistributed.

Runtime uses the existing installed genuine YAM SDK and its robot/USD assets,
under their existing notices and LICENSE.i2rt. This package neither vendors nor
modifies those assets. Station geometry/cameras are provided by that SDK. The
installed container versions are recorded in dependency-versions.json.

Task-local contacts.py adapts the verified B5 read-only contact API helper;
motor_limits.py adapts the verified B5/D3 total-effort G/K representation. They
are copied locally to avoid cross-task runtime dependencies. No calibration
status or historical model/robot score transfers with those helper methods.
These helpers are project-authored code; no new third-party license is invented.
