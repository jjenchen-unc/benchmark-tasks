## B1_wrench

All geometry is built procedurally in `benches/B1_wrench.py` from MuJoCo primitives (boxes, cylinders, one generated hex
mesh per nut). `benches/B1_wrench_assets/` is empty: no meshes, textures or third-party files are used, so there are no asset
licenses to track beyond the repo robot (robosuite Panda meshes, MIT, already in `repo_snapshot`). Web search returned only
link titles (no page text), so the numbers below come from the standards and well-known catalogue values, and are marked as
such. "Estimated" means no source was checked.

| object | real product it is based on | values taken from the spec | estimated values |
|---|---|---|---|
| hex nuts (seat, practice, W1-W3) | ISO 4032 M8 hex nut (s = 13 mm, m = 6.8 mm) and ASME B18.2.2 5/16-18 hex nut (F = 1/2 in = 12.70 mm, H = 17/64 in = 6.75 mm) | across-flats 13.00 / 12.70 mm, height 6.8 mm | mass 6 g; steel friction 0.15; all torques (scaled, see DEVIATIONS) |
| open-end wrenches 12 mm, 1/2 in, 13 mm, 14 mm | single-size open-end wrench, e.g. a combination spanner's open end (e.g. Stanley/Facom/Gedore class, ISO 3318 / ISO 691 openings) | nominal openings 12.00 / 12.70 / 13.00 / 14.00 mm | opening +0.05 mm above nominal (ISO 691 allows roughly +0.04..+0.2 mm; 0.05 is the b13 smoke value); length 160 mm; head thickness 5 mm; jaw width 9 mm; mass ~0.12 kg; 15 deg toe lead-in; box end omitted |
| adjustable wrench | Bahco 8070 (8 in / 200 mm adjustable wrench) | length 200 mm (catalogue) | jaw pre-set to 13 mm + hidden slack 0.45-0.65 mm (card value); mass 0.25 kg (catalogue lists about 0.28 kg, unverified); worm modelled visually only |
| practice seats, posts | workshop practice fixture | - | 22 mm square posts, nut bearing face at z = 0.89 m |
| W1 swing arm | swing latch / hasp clamped by a captive nut | - | 80 mm steel arm, quarter-turn travel to a stop |
| W2 cover | inspection cover with open-ended slots and captive nuts | M8 stud, 2 mm washer | 50 x 170 x 4 mm aluminium plate, 0.17 kg; 10 mm slots; grip tab |
| W2 shelf | bench shelf | - | 100 x 200 mm, top at 0.84 m |
| W3 clamp | screw-driven clamp | - | jaw linked to the nut (0.1 rad jaw per rad of nut), 90 deg free run then seat |
| knob screws (thumb bolts) | knurled-knob clamping screw with threaded stud, M8 and 5/16-18 (e.g. Kipp K0153 / Ganter GN 5337 class) | thread major diameter 8.00 mm (M8) / 7.94 mm (5/16 in); pitch 1.25 mm / 1.411 mm (18 TPI) | square 20 mm x 30 mm knob (a round knob slips out of a parallel jaw, smoke tp; 14 mm slipped below the gall threshold); stud 9.25 mm under head; mass 16 g |
| steel tapped block | workshop thread-check block with labelled M8x1.25 and 5/16-18 holes | thread sizes | 70 x 50 mm block, 4 mm clearance counterbore + 12 mm tapped bore |
| aluminium offcut, W4 base and cover | 6061-class aluminium plate | M8 clearance hole (ISO 273 medium about 9 mm) | square holes: clearance 8.6 mm, tapped bore 8.3 mm (modelling choice); cover 71 x 49 x 4 mm |
| tightening torques | real M8 8.8 nut: ~25-35 N m; real knob screw by hand: ~1-2 N m | - | two scales (estimated): nuts s_tau ~ 0.01 (window 0.30-0.40 N m = 30-40 N m real, breakaway 0.35-0.50, strip 0.18-0.25); knob screws s_tau ~ 0.1 (aluminium gall 0.12-0.14 N m = 1.2-1.4 N m real, thread running 0.01-0.03, seat preload ~0.05-0.2) |
