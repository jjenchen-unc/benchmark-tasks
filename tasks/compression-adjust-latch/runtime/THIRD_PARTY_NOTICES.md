# Provenance and dependencies

The task-only fixture is derived from the preserved S4_compression_latch design-
lab snapshot in this repository. `source-provenance.json` records exact source
hashes. No native Panda, Franka or Robotiq robot mesh/XML assets are bundled or
loaded by the runtime. The native source is not modified.

The task-local contact reader derives from this project's B5 PhysX observer;
the total effort limiter derives from the D1 bounded-motor pilot. Those task
sources remain read-only; their validation results do not transfer to S4.

Runtime robot geometry is supplied by the installed YAM/Arena dependency, not
copied here. YAM assets originate in i2rt under its MIT license. Use the pinned
project dependency/environment and retain its installed license notices.
MuJoCo3.3.7 is needed only to rebuild the static fixture compilation, not for
Arena runtime. USD/PhysX, IsaacLab/Arena and Inspect are external dependencies
provided by the existing environment; this directory changes none of them.
