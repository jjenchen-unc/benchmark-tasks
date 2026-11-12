# Installed asset dependency

The task uses the installed official yam-isaac-lab-arena SDK at revision
0059d02a1a91bc6834c1d0846022891df0c78730. Robot meshes are supplied by that
dependency, rather than bundled in this task. Its YAM asset directory includes
an I2RT Robotics MIT notice, reproduced verbatim in LICENSE.i2rt (SHA256
7cd0a0224a287ebd6c45703eb6623d264c7810f8958d65de8f99e87ccd9f4891).
This notice covers that identified component; it is not a claim that the entire
simulator or SDK uses the same license.

The task runs in the SDK's `station` scene. In this task that scene consists of
the SDK's floor and table boxes, its top-camera stand (boxes and capsules), a
uniform-colour dome light and a sphere lamp, all generated in SDK code with flat
colours; no mesh or texture file is loaded for them. The cameras are pinhole
sensors without a camera mesh. The SDK README's License section states that the
repository "has no license yet" (Placeholder). Per the SDK README, the station
replicates "Dream Labs' calibrated YAM station, from Dream's MuJoCo replica
(dreamlabs 1.0.23, `yam_mujoco`) and the calibration it follows (yamlab at
`ec0455d`)"; the SDK does not state a license for those numbers. The station
arms are coloured with I2RT's link2/link3 meshes split into black and white
parts (SDK `yam_arena/assets/yam/dream/`), which the SDK's SOURCE file marks
"MIT, see ../LICENSE.i2rt". The station's Arena object-library fruit, bowl and
plate and its water bottle are not spawned: this task's own objects replace
them. Isaac Lab-Arena release/0.3.1 ships the Apache License 2.0 in its
LICENSE.md; Isaac Sim and Isaac Lab are runtime dependencies under their own
licenses. Nothing from the SDK, Arena, Isaac Lab or Isaac Sim is redistributed
in this folder except LICENSE.i2rt.

Task fixture/pedestal geometry is procedural primitives,
with no downloaded task meshes, textures or external USD references.
