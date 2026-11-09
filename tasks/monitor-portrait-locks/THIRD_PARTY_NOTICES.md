# Installed asset dependency

The task uses the installed official yam-isaac-lab-arena SDK at revision
0059d02a1a91bc6834c1d0846022891df0c78730. Robot meshes are supplied by that
dependency, rather than bundled in this task. Its YAM asset directory includes
an I2RT Robotics MIT notice, reproduced verbatim in LICENSE.i2rt (SHA256
7cd0a0224a287ebd6c45703eb6623d264c7810f8958d65de8f99e87ccd9f4891).
This notice covers that identified component; it is not a claim that the entire
simulator or SDK uses the same license. Standard station geometry comes from
the installed SDK. Task fixture/pedestal geometry is procedural primitives,
with no downloaded task meshes, textures or external USD references.
