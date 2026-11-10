## hydraulic_jack

All geometry is built from MuJoCo primitives in `benches/hydraulic_jack.py` (no meshes or textures; `benches/hydraulic_jack_assets/`
holds only a README). The robot is the repo's two-Panda B4 robot (robosuite Panda meshes, MIT licence, as in the repo snapshot).
A web search found the product listing and manual (Amazon https://www.amazon.com/dp/B0002H3364, manual PDF
https://cdn4.servls.com/view/2191/2191634-big_red_t90203.pdf) but the pages could not be opened from this host (proxy refused), so
every product value below is from memory of the public listings and is marked approximate/estimated. Values used in the sim are about 1/2 scale of the real product, as the card says.

| object | real product it is based on | values taken from the product (approximate, from memory) | estimated / scaled values |
|---|---|---|---|
| small red jack `jack_S` | Torin Big Red T90203, 2-ton hydraulic bottle jack (Torin / Big Red, model T90203 / T90203B, listing https://www.amazon.com/dp/B0002H3364) | Red bottle-jack form: base plate, cylinder body, ram with serrated saddle, pump handle socket at the side, release valve turned by the handle's slotted end (counter-clockwise opens). Real minimum height about 181 mm, real hydraulic stroke about 80-120 mm, 2 t capacity | Body 1/2 scale (base 120x100 mm, body diameter 66 mm). Stroke sampled 55-75 mm (hidden). Ram lift per handle stroke 5.9 mm (0.023 m/rad, from n3 smoke). Pad 110x110 mm, rubber mu 1.2. Valve knob 40 mm on a 21 cm extension rod (reach compromise, card) |
| large blue jack `jack_T` | Torin Big Red T90403 / 4-ton class bottle jack (larger bore, longer stroke) | Same form, larger body diameter | Body diameter 80 mm; same pad height as the small jack (both sit under the same rails, see DEVIATIONS). Stroke sampled 75-95 mm (hidden). Same pump ratio |
| pump handles | the jacks' two-piece steel handle | steel tube, inserted in the side socket | 18 cm long, slanted to join the 60 mm grip sleeve at its end, 0.07 kg, return spring 0.5 N m/rad, seal friction 0.15 N m (estimated) |
| release valves | the jacks' needle release valve | counter-clockwise opens; flow grows with opening and with load pressure | Opening angle phi0 30-120 deg (card), gain v30 = descent speed at phi0+30 deg under 50 N, 20-300 mm/s (card, INFERRED), flow ~ sqrt(load) (orifice law), saturates 90 deg past phi0; knob friction 0.30 N m (batch2 setting) |
| steel rest rails | workshop jack-stand / cribbing rails (generic) | steel | Two 30x30 mm steel bars, 0.9 m long, top 135 mm above the table, mu 0.35 against wood (estimated) |
| wooden crates `crate_A`, `crate_B` | generic small sealed wooden storage crate (no specific product) | wood | 180x180x120 mm body on two 12 mm wooden skids, weight 10-30 N (hidden; card 15-60 N, narrowed per lead after review 1), skid-steel mu 0.35, side face mu 0.25 (estimated) |
| glass display cabinet `cabinet` | generic tabletop glass display case with walnut frame (no specific product found from memory) | glass panels, wooden frame | 200x200x386 mm, 70-100 N (hidden, card), walnut plinth and frame, four glass side panels, two glass shelves with ornaments |

Licences: all scene geometry is original to this bench (primitives), no third-party assets. Robot meshes come from robosuite (MIT).
