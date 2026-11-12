# Asset sources

All case, latch, instrument and plinth geometry is made of simple primitives (boxes and cylinders) written by
[candidate/build_usd.py](candidate/build_usd.py) into [candidate/assets/](candidate/assets/). There are no mesh,
texture or third-party asset files, so no external licence applies. The YAM robot and station come from the
standard `yam_arena` package and are not bundled.

The dimensions follow common products but were estimated, not measured:

| object | based on | values used |
|---|---|---|
| draw latch (lever + hook) | over-centre spring draw latch with adjustable hook and secondary catch (for example Southco 97-series) | separate spring-loaded hook, 4 mm hook preload, lever on the case body |
| keeper with three bars | multi-position keeper used with spring draw latches | 5 mm bars at 6 mm pitch |
| safety catch tab | slide catch beside the lever that stops vibration opening | 12 x 12 x 14 mm tab, 7 mm travel |
| instrument case | small rigid plastic equipment case with two front draw latches and a rear hinge | 240 x 160 x 180 mm body, 40 mm lid, 1.82 kg with hardware |
| instrument | tall cylindrical bench instrument | 40 mm diameter x 160 mm, 0.30 kg |
| plinth | wooden riser board | 600 x 200 x 60 mm, fixed |
