# Bench fixture and loading tool

Two printed parts. The fixture holds the empty magazine upright on the bench. The tool is put on the magazine to hold
the follower down while the round is slid in, and its tooth pushes the round back to the rear wall afterwards.
STATUS: neither part has been printed or test-fitted on a real magazine. Both were designed on a community CAD model of
the magazine, not on measurements of a real one.

## Parts
| Item | Qty | Size / part |
|---|---|---|
| Magazine | 1 | 9 mm, 17-round factory magazine MF17017 (10-round MF10017 has the same body) |
| Dummy rounds | 1 per trial | inert 9x19 aluminium snap caps (for example A-Zoom 15116, 5-pack) |
| Fixture | 1 | `magazine_fixture.stl` / `.step` from `make_fixture.py`; PLA or PETG; 166.7 cm3 (207 g at 100 % PLA) |
| Keeper pin | 1 | steel pin, dia 5 mm, at least 75 mm long |
| Bench screws or clamps | 4 screws or 2 clamps | M5 through the base holes, or any bench clamps |
| Loading tool v3 | 1 | `loading_tool3.stl` / `.step` from `make_tool.py`; PLA, 60.2 cm3 (74.6 g at 100 %) |

## Fixture drawing (mm)
- Base 120 x 80 x 6 with four dia 5.5 holes at (7, +-33) and (113, +-33). Tower 75 x 46.6 x 50 on the base.
- Channel for the magazine tube raked 21 deg, 37.27 long x 23.1 wide (0.3 clearance per side), open on one side.
- Groove for the floor plate 41 x 28.8 x 6.6. Keeper-pin hole dia 5.2, 30 above the base top, closing the open side.
- `fixture_check.png`: section views of the magazine in the fixture (from `check_fixture.py`).

## Tool drawing (mm, tool seated and pressed on the magazine)
- Ring inside 35.6 x 23.3 (0.4 clearance front and sides, 1.2 rear), walls 2.5. Roof 3 thick, 1.0 above the lip tips.
- Tongue 6.5 x 2.5 on the follower (v3; v2.1 had 5.0). Stop: the ring bottom sits on the fixture tower top; in the
  simulation the force-held tool pushes the follower 5.1-5.7 mm down.
- Front-wall relief 6.7. Nose-pusher tooth 4 wide x 5 thick at the bottom, face lean 15 deg.
- Fin 80 long x 7.0 thick, finger-seat lugs 4.5 proud of each fin face. Size in use 112.7 x 28.3 x 90.9.
- `section_fig.py` draws the side section, push pose and tongue section (needs trimesh with manifold3d; the v3
  drawing has not been regenerated).
- Print upside down (roof on the bed, tube axis vertical), no supports; downward faces 45 deg or steeper except 1 mm
  flats; the front wall is a 23 mm bridge.

## Regenerate
```sh
python make_fixture.py                 # CadQuery: magazine_fixture.step / .stl
python make_tool.py MAG_STL            # CadQuery + trimesh: loading_tool3.step / .stl, ../isaac/tool/tool_hulls.json,
                                       # overlap checks (MAG_STL: the community magazine STL, inches; NOT bundled,
                                       # no license; the checks against the simulation collider run without it)
python section_fig.py                  # drawing
```

## Assembly
1. Test-fit the magazine in the fixture before use (the fixture is designed on a mirror-symmetric body).
2. Screw or clamp the fixture base to the bench. Slide the empty magazine in from the open side, floor plate in the
   groove. Push the keeper pin through.
3. Stand the tool on its ring at its tape outline.

## Checks before trials
- Fixture: pushing the magazine with about 30 N down and 30 N sideways does not move the fixture (ruler at a tape mark).
- Tool: it slides onto the magazine and the ring reaches the tower top; pressed fully, the follower moves about 5-6 mm.
- Magazine: the follower returns to its stop when released; the magazine is empty.

## Reset
Press the round forward out from under the lips with a finger (or turn the fixture out and the magazine over), check
the follower returns, put the tool and the round back on their outlines.
