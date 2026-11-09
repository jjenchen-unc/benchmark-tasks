# Asset provenance

All task cartons, pads, cargo, tags and indicator visuals are generated primitives. The native D3 ASSETS.md explicitly records that no third-party task mesh or texture is used. The four local USD assets are procedural derivatives of that task geometry. Native robot/runtime helpers and robosuite/Panda assets are not bundled or required.

YAM robot geometry is supplied externally by the pinned runtime. Its I2RT MIT notice is reproduced in LICENSE.i2rt. Isaac Sim, Isaac Lab/Arena, Inspect Robots, NumPy, SciPy and PyTorch remain external dependencies under their own licenses. No SDK code, proprietary binaries, environment, robot meshes or textures are redistributed here.

Task-specific scene/mechanics/scoring/replay code and generated primitives are this contribution; original native scene and adaptation provenance are identified in runtime/source-provenance.json.
