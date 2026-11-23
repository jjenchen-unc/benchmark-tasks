# Build the dryer-screen handling fixture

## Appliance and benchmark geometry

Maytag's LNC7764A71 parts list, “Door (Series 11),” identifies item 9 as lint filter 53-0918 and item 6 as collector duct kit 12001324. The filter sits at the front door opening. PartsReadyOnline lists a used 53-0918 at US $17 before shipping, with a half-moon outline, 14½ × 6 in envelope, 1¼ in handle width, and ends ⅜ and ⅝ in thick. The price is for the filter alone. [Manufacturer parts diagram and list](https://www.maytag.com/content/dam/global/documents/201206/parts-list-LNC7764A71.pdf) · [Filter listing with dimensions and photographs](https://www.partsreadyonline.com/item.php?ItemID=451)

For scored robot trials, build the screen **and its matching slot** below. The bench cartridge uses the listed envelope, grip width and asymmetric taper. The rectangular-backed mesh, two grip extensions and horizontal deck are adaptations that give the arms access without a door or a powered appliance. A stock filter has a different bottom profile. Use the bench cartridge for both the real trial and the simulated comparison. The Start signal comes from an instrumented button on the fixture.

## Fabrication and alignment

The three DXFs are 1:1 in mm. Import the closed outlines on layer CUT. REFERENCE marks locate parts or identify orientation. Confirm the mat rectangle measures 345 × 118 mm before cutting. In deck plan the robot faces the front edge with the thin end to its left.

| Part | Fabrication and inspection |
|---|---|
| Deck and guides | Cut the 686 × 700 mm deck from 20 mm plywood using the drawing. The straight back edge of the slot lies 150 mm behind the deck front. Fit a straight 4 mm back wall and four equal-width 4 mm front-wall segments flush with the corresponding cut edges. Continue the central opening down to a floor 148.4 mm below the deck surface. Close the guide ends at the ends of the 374.3 mm central opening. Fit 85 × 65 mm pocket floors outside the guides, 30 mm below the surface. Keep fasteners outside the clearance envelope. [Deck plan](deck.dxf) |
| Screen body | Cut a rigid 368.3 × 136.4 mm foam-core blank. Bond a thin rigid skin on each face and fine nylon insect mesh on the front; finish the combined thickness to a linear taper of 9.525 mm at the left to 15.875 mm at the right. Keep the back flat. Bond the mesh over the backing with no loose edges. Use a light timber or hollow grip, 418.3 × 31.75 × 16 mm. Centre it over the span, 25 mm overhang per end. In depth its centreline is 6.35 mm forward of the body's flat back. Trim or ballast internally to 105 ± 5 g. Aim for centre of mass on the midline, 58.2 mm below the grip underside. All dimensions refer to the finished exterior. [Body, taper gauge and grip](cartridge.dxf) |
| Deck and cartridge fit | With cartridge centred, check 1 mm gap at back, 5 mm at bare front face, 3 mm at both ends and 12 mm below the body. Pocket and grip dimensions within 0.5 mm; gap targets within 0.3 mm. A loaded cartridge must slide in by hand. The underside of the grip must rest on the lips, with a 2.5 mm feeler excluded at both ends. Measure the narrow opening against the finished thick end: nominal 15.525 mm opening versus 15.875 mm body. Require at least 0.2 mm interference and check that the bare reversed cartridge fails to seat in a gentle hand insertion. |
| Start and collection | At the marked 24 × 24 mm position fit a cap on a small load cell, 6 mm proud. Latch a lamp or a timestamp after force reaches 4 N continuously for 0.5 s. Check using known weights. Put a tray of inside opening 180 × 400 mm against the front, rim 12.5 mm below the deck. Set the deck 52.5 mm above the robot table. Use the recorded offsets for each setup. The fixture uses no heater or drum. |

## Making consistent lint mats

Use Vlieseline Low Loft Batting 249, a soft sew-in polyester sheet specified at 47 g/m². A 345 × 118 mm rectangle has a nominal mass of 1.91 g. Use one dry ply from the same roll with its long edge along the roll. Materials for reset are the batting, a reusable cutting template, scissors or rotary cutter, and one fixed hook spot on the screen. [Manufacturer material specification](https://www.vlieseline.com/en/product/412/249) · [Cutting template](mat.dxf)

**Commission each stock lot and screen once.** Cut five spare rectangles to ±1 mm. Weigh each (target 1.8–2.2 g). For thickness, place a flat 50 × 50 mm, 10 g card on the centre and read the gap after five seconds (target 2.5–3.5 mm). Seat a small disk of low-profile nylon hook tape in the front skin, hooks exposed flush with the mesh at the centre of the mat outline. Start with a 6 mm disk. Set each spare with the procedure below. With the screen clamped, pinch a 20 mm strip at the centre of the short right-hand edge. Pull horizontally outward along the long axis at about 25 mm/s with a 0–5 N spring gauge. Require each peak release force to be 1.2–1.8 N and each piece to detach intact. Adjust the exposed hook area and repeat with five new pieces if needed. Record the lot, accepted diameter and readings. This bench test sets a comparable stripping load; the simulated joint also responds to moments and represents the entire mat as one rigid body.

**For each trial (design target 2–3 minutes).**

1. Cut a fresh rectangle along the template, or take a flat pre-cut mat. Check size and mass and discard any curled or torn piece. Clear stray fibres from the hook spot.
2. Lay the cartridge mesh up. Centre the mat side to side, with its lower edge 6 mm above the body bottom (11.65 mm side margins). Use the REFERENCE rectangle on the body drawing as the placement gauge. Apply a flat 25 × 25 mm foot carrying 200 g total for two seconds over the hook spot; leave the perimeter free for the robot's pinch.
3. Stand the screen upright for five seconds. If the mat slips more than 2 mm, detaches, or curls into the guide clearance, replace it and check the hook spot. Insert the cartridge mesh toward the robot, seat both grip ends, empty the bin, reset the Start indicator and place the fixture on the trial's offset marks.

The elapsed reset time, coupon release forces, physical slot fit and button calibration are acceptance measurements for the first build. The simulation records rigid-body release and disposal; it does not resolve individual fibres or local peeling.
