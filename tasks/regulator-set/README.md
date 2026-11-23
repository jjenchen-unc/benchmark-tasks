# Turn a non-relieving shop-air regulator down to 90 psi: one hand bleeds the line while the other sets the knob

The task is the Isaac Lab-Arena (PhysX) `yam_arena` task in `isaac/`; `src/` holds the engine-independent modules it
imports (layout, pressure plant and start states, scorer, fast IK) and `tests/` their unit checks.

## The task
A shop-air drop is mounted on a plywood drop board on the bench in front of the robot: a 1/4 in general-purpose
regulator of the non-relieving kind (modelled on the SMC AR20 non-relieving option, see "Real-world setup
(reproducible)"; note that the common Norgren R07 is a relieving regulator) with a 36 mm push-to-lock adjusting knob on
top, the supply
coming in on the robot's left, and on the outlet side (robot's right) a tee with a 2.5 in (63 mm) 0-160 psi gauge
facing up, then a 1/4 in lever ball valve that vents the line through a muffler (the bleed). The regulator was left
set high: the gauge reads 104-122 psi. The instruction is only "Set the shop air regulator to 90 psi."

What makes it hard:
1. The regulator is non-relieving: turning its knob down does not lower the outlet pressure. The gauge stays where it
   is until air is let out downstream. The robot has to open the bleed with one hand while it turns the knob with the
   other (the gauge then follows the knob), shut the bleed, and finish the adjustment upward onto 90 psi, which is the
   maker's and the trade's method.
2. The knob is locked: it has to be pulled out (about 18 N against its detent, the orange band shows) before it turns,
   and pushed back in afterwards.
3. How far a turn moves the pressure (12-18 psi per turn) and the start pressure change every trial and show nowhere
   but on the gauge, whose needle lags the line by a fraction of a second; the band is +-2 psi (one minor division).
4. A line shut in at 90 psi with the knob set lower is not "set": the gauge drops to the regulator's setting at the
   first tool draw. The scorer checks the regulator's own setting as well as the gauge.
Cues the robot can see: the gauge needle against its graduations (2 psi minor, 10 psi, 20 psi major), the knob's
orange lock band, the knob's white index mark, the ball-valve lever (across the pipe = closed, along it = open).

## Scoring (`src/scoring.py`, primary score 0..1, value at the end of the trial)
- 1.0 (ends the trial), when all of these hold together for 2 s: the gauge within 2 psi of 90 and steady (needle
  slower than 1 psi/s); the regulator's setting within 2 psi of 90 (the gauge stays there when air is drawn); the knob
  pushed in (locked); the bleed closed (lever within 3 deg of its closed stop); no robot contact with any part of the
  drop for at least 1.5 s.
- 0 otherwise. There is no partial credit: every intermediate state (knob unlocked, bleed open, near 90) can also be
  reached by a blind policy.
Person check: the video shows the needle on 90 with the hands away, the lever across the pipe and the knob pushed in;
cracking the bleed or running the tool briefly shows the gauge settling back on 90 (the setting).

## Dream-1 evaluation

On Brev, Dream-1 scored 0.0 in 1,800 of 1,800 control steps at evaluation seed 0. The Inspect Robots log contains 1,800 actions of 14 values each. Run the following from `isaac/` inside the configured YAM environment.

```bash
inspect-robots run --embodiment yam_arena --policy dream \
    -P model=dreamlabs/dream-1-beta -P rows_per_request=10 \
    -E task=task.py -E object_jitter=0 -E device=cpu \
    -E control_hz=20 -E cam_width=320 -E cam_height=176 \
    --instruction 'Set the shop air regulator to 90 psi.' --scorer task_score --seed 0 --max-steps 1800 --store-frames --log-dir ./dream-output
```

## Run and score a trial
Inside the Isaac Sim 6.1 + Isaac Lab-Arena container, working directory `isaac/` (`isaac/run_frozen.sh` does this on a
frozen copy of `isaac/`, `src/` and `task.json`, writes `source-sha256.txt` and logs to `<out>/<run>/run.log`; see its
header for the environment variables it needs):

```
# official Inspect Robots run of the scripted reference demo (registered closed-loop policy regulator_set_scripted)
python official_run.py -- run --embodiment yam_arena --policy regulator_set_scripted -E task=task.py \
    -E object_jitter=0 -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 \
    --instruction "Set the shop air regulator to 90 psi." --scorer task_score --max-steps 1800 --store-frames \
    --log-dir <out>/official
inspect-robots video <out>/official/<log>.json --out <out>/official/video     # H.264 video of every camera

# plain do-nothing run (task.py demo() gives the yam_scripted policy no waypoints: both arms hold READY)
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py ... --max-steps 1800

# direct demo / negative stages (trace.jsonl, result.json, demo_<camera>.mp4, setup.png, final.png);
# --seed defaults to the official eval-seed-0 trial seed (1415158438); "stage@seed" sets one trial's seed
python demo.py task.py <out>/<run> --stage demo
python demo.py task.py <out>/<run> --stage neg-no-bleed    # knob turned down, bleed never opened: gauge stays: 0
python demo.py task.py <out>/<run> --stage neg-shut-in     # knob down 3 turns, bleed cracked to ~90, shut: 0
python demo.py task.py <out>/<run> --stage neg-left-open   # set to 90 under flow, bleed left open: 0
python demo.py task.py <out>/<run> --stage replay@30039    # RG_REPLAY=<trace.jsonl>: open-loop replay (RG_MIRROR=1: mirrored)
```
Unit checks (no simulator): `python -m pytest tests`.
Tiled evidence video (top + left wrist + right wrist, same timing):
`isaac/tile.sh <out_dir> <name> top.mp4 left.mp4 right.mp4`.

## Submission media

- `model_run.mp4` shows the full Dream-1 trial. The views are overhead, left wrist and right wrist at 20 Hz.
- `scripted_run.mp4` shows the independently rerun official scripted policy, which scored 1.0 in 1,164 steps.
- `setup_overhead_hires.png` — Overhead view of the reference setup.

Start states: every trial reset with a seed (every `inspect-robots run` passes one) draws the start pressure (the
regulator's setting and the line pressure, 104-122 psi), the spring rate (12-18 psi per knob turn), moves the drop up
to 15 mm in x and y and turns it up to 6 deg. Any start whose required knob turn (90 - start) / rate would lie within
0.3 turn of the default start's gets a new pressure and rate, so the default start's knob turn leaves every other
start at least 3.6 psi off. Unseeded resets use three named setups. The scene is built in the first trial's start
state; a later trial in the same process gets its own pressures, but the drop stays where the first trial put it. One
trial per process is the intended use (the official CLI runs one).

## Fidelity (full statement)
- Engine: Isaac Sim 6.1 / Isaac Lab-Arena `yam_arena`, PhysX on the CPU, physics 480 Hz, control 20 Hz, cameras
  320 x 176.
- Parts at estimated real size, not checked against the named parts (see "Known sim/real differences"): regulator body 44 mm
  square, bonnet 40 mm, knob 36 mm x 24 mm with 4 mm push-to-lock travel; 1/4 NPT nipples (13.7 mm), tee, 2.5 in
  (63 mm) centre-back-mount gauge, 0-160 psi over 270 deg (numerals not modelled); 1/4 in ball valve with an 80 mm lever
  and a 14 x 12 mm vinyl sleeve; 18 mm plywood drop board. The board stands for the shop wall, at bench height so the
  arms can reach the knob from above.
- Knob (PhysX joints on the fixed drop): a prismatic travel held at either end by a detent (stiff drive capped at
  18 N, estimate), then a revolute knob. Pushed in: the lock ring holds the knob's turn up to 4 N m. Pulled out: the
  adjusting screw's friction, 0.12 + 0.0022 N m per psi of setting (0.32 N m at 90 psi; estimate from the spring load
  on a ~7 cm2 diaphragm and the screw's friction). Clockwise raises the setting by 12-18 psi per turn (seeded).
- Bleed valve: the lever turns on the stem (revolute) between hard stops at 0 (closed) and 90 deg (open) against
  0.45 N m of operating friction (estimate for a 1/4 in PTFE-seat ball valve); sealed below 8 deg, flow rising to full
  at 90 deg.
- Pressure: a lumped model, not a gas-dynamics simulation. The regulator valve admits supply air (150 psi) only when
  the outlet is below the setting, proportionally within 4 psi; there is no relief path (non-relieving). The bleed
  vents the downstream volume (time constant about 1.7 s fully open). Wide open, the outlet droops about 1.2 psi below
  the setting. The gauge follows the line with a 0.3 s lag (snubber); its needle is a PhysX joint driven to that
  reading. No tool, hose or air consumption other than the bleed is modelled.
- Forces in the reference demo (official seed; PhysX): finger squeeze up to about 19 N per finger on the knob and
  21 N on the lever sleeve (estimated from the finger drives); the knob needs 18 N to pull out or push in and
  0.3-0.4 N m to turn; the lever 0.45 N m. The demo pulls the knob out by its grip but pushes it back in with its
  closed fingertips on the knob's top (pad friction alone slips below the 18 N detent). At the push-in click the knob
  hits its stop with the fingertips on it: the PhysX contact force between fingertip and knob shows a one-physics-step
  (2 ms) spike of 174-194 N in the evidence runs; the sustained press is about 18-22 N.
- Robot-on-part contact (for "hands off") is read from PhysX contact forces between the robot's links and the knob,
  the lever and the rest of the drop (collider identity, not contact positions).
- Nothing welds, snaps or teleports anything during a trial; the scripted demo only sends the 14 joint / gripper
  commands and reads state (the gauge reading, the knob's travel and turn, the lever angle; never the hidden setting
  or spring rate). On an environment reset (between trials only) the kit sets the parts and pressures for the next
  start.

## Real-world setup (reproducible)
The tabletop assembly needs a left-to-right regulator with the knob on top and a separate downstream gauge. For the SMC AR20-B, the ordering-table configuration is `AR20-N02B-NYZ-B`. The first N selects NPT, 02 selects 1/4 inch, B after the port selects the bracket, and the suffix options N, Y and Z select non-relieving operation, upward knob and psi markings. This code is derived from the manufacturer’s table. Availability and delivered options require supplier confirmation. [SMC AR20-B ordering guide](https://www.smcworld.com/assets/overseas/international/en-jp/iso/pdf/iso-25_ar20-en.pdf)

| Item | Part (example source) | Approx. price |
|---|---|---|
| Non-relieving regulator, 1/4 NPT | SMC AR20-B configured as above, separate gauge, standard adjustment range 0.05–0.85 MPa | Obtain a quote for the configured unit |
| Pressure gauge, 2-1/2 in, 0-160 psi, 1/4 NPT centre back mount | WIKA 111.12 2.5 in 0-160 psi CBM (or any 2-1/2 in 0-160 psi CBM gauge with 2 psi minor divisions) | USD 15-30 |
| Bleed valve, 1/4 in threaded bronze ball valve | Apollo 70-101-01 (7010101) | USD 41.19 list in the May 2026 [Apollo price list](https://aalberts-ips.us/wp-content/uploads/2024/12/BVPL-0526-1.pdf) |
| Muffler, 1/4 NPT sintered bronze | any 1/4 NPT male sintered-bronze breather muffler | USD 3-8 |
| Fittings | 1/4 NPT brass street tee, three 1/4 NPT x close / 1-1/2 in hex nipples, 1/4 NPT industrial (M-style) quick-connect plug on the inlet, PTFE tape | USD 15 |
| Mounting | 18 mm plywood drop board about 40 x 20 cm on a foot, SMC AR20 bracket (included in the specified B option), two wood screws, a pipe clamp under the valve | USD 15 |
| Air supply | a shop compressor whose outlet regulator is set to 125 psi or less, e.g. DeWalt DWFP55126 6 gal pancake (its own regulator turned to 125 psi), 1/4 in ID air hose with M-style coupler | USD 0 (shop) / about USD 180 |

For the physical rig, use a measured 125 psi regulator inlet and start at 104–106 psi. The series manual limits the outlet setting to 85% of inlet pressure (106.25 psi at 125 psi). The gauge also has a fluctuating-service limit of two thirds of full scale (106.7 psi on a 160 psi dial). Use a calibrated reference at the outlet to score 88–92 psi. The 160 psi WIKA dial is the robot’s visible cue. Its nominal 2% midscale accuracy allows 3.2 psi error, wider than the scoring band. [WIKA gauge accuracy](https://shop.wika.com/en-us/111_12.WIKA) The remaining simulated starts, up to 122 psi, are outside this fixture’s documented operating range. [SMC installation manual, §2.3](https://static.smc.eu/binaries/content/assets/smc_global/product-documentation/installationmaintenance-manuals/en/im_ar-b_tf2z394en.pdf) · [WIKA 111.12 operating limits](https://www.wika.com/en-us/111_12.WIKA)

The downstream volume is only the tee, nipples and valve body (about 10-15 mL), so the bleed empties it in a second
or two and nothing stores much energy. Wear safety glasses; the muffler points down, away from the robot and people.

A reproducible physical example is layout seed 3. The frozen layout gives 105.54 psi, a board shift of +9.04 mm forward and +2.46 mm left, and yaw −4.87°. Set approximately 105.5 psi on the calibrated reference. Evaluation seed 0 maps to layout seed 1415158438 and starts at 116.69 psi in simulation. The published 125 psi fixture cannot reproduce that specific start within SMC’s operating guidance.

Setting a seeded start (about 1 minute):
1. Supply on (125 psi). Open the ball valve (lever along the pipe), pull the regulator knob up (unlocked).
2. Turn the knob counter-clockwise until the gauge reads well below the start pressure, then clockwise until the gauge
   reaches the selected physical start (104–106 psi, approached from below), shut the valve (lever across the
   pipe), check the gauge holds, push the knob down (locked).
3. The knob's rotational position follows from the pressure (there is no other knob setting to make); its knurl phase
   is cosmetic. Set the board on its bench marks (position and yaw marks every 5 mm / 2 deg) for the trial's pose.

Reset after a trial using steps 1–3 with the next supported physical start pressure.

Real-trial scoring checklist (keep the dial photo, the calibrated pressure reading and the contact video):
- the gauge needle reads 88–92 psi on the dial photo 2 s after the hands leave, with the calibrated outlet reading also in that band;
- the regulator really set: crack the valve for 1 s and shut it; the calibrated outlet reading must settle back within 2 psi of 90;
- the knob pushed down (the orange/coloured lock band of the AR20 knob is not visible, or the knob will not turn);
- the valve lever across the pipe (closed);
- no robot contact with the drop at the end (video).
Score 1.0 if all hold, else 0.

## Known sim/real differences
- Dimensions are estimates, not measured on the SMC AR20 (body 44 mm square, bonnet 40 mm, knob 36 x 24 mm, 4 mm lock
  travel); the real AR20 knob diameter and travel must be measured and the sim geometry updated.
- Spring rate: the sim seeds 12-18 psi per knob turn; the maker publishes no psi-per-turn figure, and one real unit has
  one fixed rate. The real unit's rate must be measured (count turns from 30 to 120 psi) and either the sim range
  centred on it or several units used; the official-seed rate (12.2 psi/turn) may not match a given unit.
- Supply: the sim uses a 150 psi supply; the documented real rig uses a measured 125 psi inlet and the 104–106 psi subset of simulated starts.
- Pressure is a lumped model (proportional regulator valve, 1.2 psi droop with the bleed wide open, 1.7 s bleed time
  constant, 0.3 s gauge lag), not gas dynamics; real droop, bleed speed and needle lag must be measured.
- The knob detent (18 N), knob friction (0.12 + 0.0022 N m per psi), lock holding torque (4 N m) and valve torque
  (0.45 N m) are estimates, not measured on the named parts.
- The gauge needle is a driven joint showing the computed pressure (the real needle is a Bourdon tube); numerals are
  not drawn on the sim dial.
- The sim's knob carries a coloured lock band; check the bought knob shows one (if not, the lock check is "the knob
  will not turn").
- The drop board stands for the shop wall at bench height so the arms reach the knob from above.
- The scorer's "setting" check reads the regulator's spring setting directly; the real check is the 1 s bleed test
  above.

## Third-party assets and licenses
| Asset | Source | License |
|---|---|---|
| YAM arm URDF and meshes | [I2RT Robotics model source](https://github.com/i2rt-robotics/i2rt/tree/120c3c81400171174604e503943f8d1ebc891058) | MIT. The installed runtime records the model commit in `yam_arena/assets/yam/SOURCE_COMMIT`. |
| Station, cameras and YAM embodiment | [YAM Arena runtime used on Brev](https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730#license) | Installed dependency. Its README reports that the integration code has no declared license. |
| Isaac Sim 6.1 and Isaac Lab-Arena | NVIDIA and [Isaac Lab-Arena](https://github.com/isaac-sim/IsaacLab-Arena/blob/c7edf3a9d5d8276565208f72422f8f8a0806d6cf/LICENSE.md) | NVIDIA Isaac Sim license and Apache-2.0 for the Arena package, respectively. |
| Regulator, gauge, valve, pipes, board geometry | generated in code by this task (`src/layout.py`, `isaac/regkit.py`); simplified convex pieces at estimated real sizes, no manufacturer CAD | MIT (this repository, `LICENSE`) |
| `src/fastik.py` (YAM URDF kinematics) | this task's authors | MIT |
