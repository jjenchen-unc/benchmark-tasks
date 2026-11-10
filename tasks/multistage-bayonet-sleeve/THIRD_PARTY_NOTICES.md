# Provenance and license status

The frozen native_source.py is an exact copy of the current task-local B7
snapshot, SHA256 recorded in source-provenance.json. Its native ASSETS.md states
that original task geometry has the same license as the TinkerBench package,
but a complete original geometry license text was NOT found in the inspected
snapshot. Redistribution license clearance is therefore UNVERIFIED; do not
claim MIT for that original task code or publish until that is resolved.
Original product references in native ASSETS are estimated context, not copied
meshes/textures. The complete preserved source path and document hashes are in
the provenance file. The native package is read-only and remains unchanged.

The source's MuJoCo/Panda imports are NOT executed: source_geometry.py executes
only exact AST-selected pure constants/geometry/scoring helpers from the frozen
file. No native robot XML/mesh/support package is bundled or loaded. Both USD
assets are authored from those original primitives.

The task-local contact reader, episode guard, frozen-process driver and total
motor-effort adapter derive from local S4/B5/D1 implementations; exact inputs
are recorded. Their existing validation does not transfer to B7. New Arena
integration/reference/evaluator code is authored in this development package.

Installed YAM robot assets are external (i2rt MIT origin), as are IsaacLab,
IsaacSim/USD/PhysX, Arena and Inspect. Preserve those dependencies' installed
license notices and use the existing pinned environment. No dependency,
image, driver, SDK or environment has been edited by this package.
