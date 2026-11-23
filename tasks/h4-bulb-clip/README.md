# Change an H4 headlamp bulb from behind the lamp: tabs into notches by eye, glass untouched, then hook its wire clip

The revised fixture uses a fixed headlamp bracket, a laterally compliant wire and a removable contact shoe on the left YAM gripper.

The task is the Isaac Lab-Arena (PhysX) `yam_arena` task in `isaac/`;
`src/` holds the engine-independent modules it imports (layout and part geometry, scorer, fast IK) and `tests/` their
unit checks.

## Physical setup and task

Use the rear bulb interface from a 2018 US Suzuki GSX250R L8 headlamp assembly (35100-20K30, service replacement 35100-20K31) on a bench. The Suzuki L8 owner’s manual specifies a 12 V 60/55 W H4. Suzuki service bulletin 253 shows the keyed bulb, the holder wire hinged on the left, its bracket on the opposite side and the technician’s finger at the upper right. The separate keeper plate at the hinge stays installed during this task. The US dealer listed the original assembly at $324.99 when checked. A used OEM example was advertised at $155 with mounting hardware excluded and was marked sold out. Check the rear spring and bracket when sourcing a used assembly. [Suzuki owner’s manual](https://www.suzuki.at/service/dokumente/bedienungsanleitungen/motorrad/GSX-250R%20L8%20DE%20ab%202018.pdf) · [Suzuki bulletin, pp. 6–9](https://static.nhtsa.gov/odi/rcl/2020/RCMN-20V612-5758.pdf) · [OEM parts fiche](https://www.revzilla.com/oem/suzuki/2018-suzuki-gsx250r/headlamp) · [Dealer](https://www.oneidasuzuki.com/head-lamp-gsx250r-2018.html) · [Used OEM listing](https://candscustom.com/products/2018-2019-suzuki-gsx250r-headlight)

Clamp the unpowered housing with the wide slot up, rear face towards the arms and seat axis horizontal, 400 mm ahead of the arm bases, centred between them, 150 mm above the worktop. Remove the coupler, rubber cap and old bulb before reset. Place the replacement glass-down in the foam cup at x 215, y −60 mm. Named setups use bulb rolls of 37, −124, 151, −58 and 92 degrees and open clip angles of 104, 100, 96, 102 and 98 degrees. Seeded setups vary roll, open angle (95–105 degrees) and cup position (±10 mm).

Pick up the bulb by its base, align the asymmetric lugs with the openings and seat the flange. Swing the clip over the cap and shift its upper-right end clear of the fixed bracket, press inward and release the wire under the lip. The robot withdraws both hands. For a physical trial, check the flange sits flush, the tabs stay indexed, and a light pull on the wire leaves it under the bracket.

The left gripper carries a split 6061-aluminium collar on the fixed Ø57 mm boss and a steel tool with a 3 mm-wide contact face. The stock moving jaws remain available for swinging the open wire. To finish the operation, the shoe reaches past the rear rim, supports the wire axially and laterally, and withdraws clear of the seated bulb. Its attachment, three part STEP files, assembly STEP, dimensions, mass check and rebuild script are supplied in the [tool CAD package](retainer-tool-CAD.zip). The proposed tool still requires a fit and load check on physical hardware.

## Scoring

Any contact beyond 0.05 N between a robot link and the glass, or a downward bulb speed above 0.3 m/s with no robot contact above 0.05 N, vetoes the trial. A seated flange has axial error at most 0.4 mm, lateral error at most 0.2 mm, tilt at most 1 degree and roll error at most 4 degrees. Lateral tolerance includes the model’s 0.10 mm radial clearance and 0.10 mm solver allowance.

| Score | End condition |
|---|---|
| 0 | Veto or bulb outside seated tolerances. |
| 0.25 | Seated, clip open. |
| 0.5 | Seated and clip within 6 degrees of closed. |
| 1.0 | Seated, wire within 1.2 degrees of closed and its returned operating end physically behind and under the fixed lip. All four fingertips and the mounted shoe tip clear the bulb and wire by at least 30 mm, with the bulb still for 1 s. |

Stillness uses the largest centre-of-mass displacement speed and the largest shortest-arc angular speed over the 24 adjacent 480 Hz integration intervals in each 20 Hz control frame. Each of the 20 consecutive frames must stay below 3 mm/s and 3°/s. A single brief movement resets the hold even if the bulb returns within the same frame. The drop veto uses the native vertical velocity and robot contacts at every physics substep. The glass veto uses the measured force on the dedicated glass collider.

## Recorded validation

The official reference on the revised mechanism reached 1.0 in 1,248 steps. It moved the operating end sideways 2.735 mm to clear the fixed tab. After release the flange retained 17.404 N of axial preload. At the finish, the nearest fingertip or shoe point was 37.62 mm from the bulb and 80.10 mm from the wire. All 480 consecutive physics intervals in the final second met the original stillness limits. In the axial-only check the tool pushed 3 mm, increasing axial contact force from 1.473 to 2.935 N, with zero lateral wire travel. The bulb stayed seated and the spring stayed unlatched, for 0.25 in 1,314 steps.

A separate axial-only force check reached 13.118 N, above the 11.918 N axial peak in the successful trial. The wire had zero lateral travel and stayed outside the tab.

Dream-1 scored 0.0 in 2,400 steps on seed 0 with the revised wire, fixed bracket and gripper shoe. The task-board media show the full three-camera reference and model trials plus the initial overhead frame.

## Dream-1 evaluation command

From the task's `isaac/` directory in the Isaac Sim 6.1 and Isaac Lab-Arena environment, run:

```bash
instruction=$(python -c 'import json; print(json.load(open("../task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
    -P model=dreamlabs/dream-1-beta -P rows_per_request=10 \
    -E task=task.py -E object_jitter=0 -E device=cpu -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
    --scorer task_score --seed 0 --max-steps 2400 --store-frames --log-dir ./dream-output
```

## Run and score a trial
Inside the Isaac Sim 6.1 + Isaac Lab-Arena container, working directory `isaac/` (`isaac/run_frozen.sh` does this on a
frozen copy of `isaac/`, `src/` and `task.json`; see its header for the environment variables it needs):

```
# official Inspect Robots run of the scripted reference demo (registered closed-loop policy h4_bulb_scripted)
python official_run.py -- run --embodiment yam_arena --policy h4_bulb_scripted -E task=task.py -E object_jitter=0 \
    -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 \
    --instruction "$(python -c 'import json;print(json.load(open("../task.json"))["instruction"])')" \
    --scorer task_score --max-steps 2400 --store-frames --log-dir <out>/official
inspect-robots video <out>/official/<log>.json --out <out>/official/video     # H.264 video of every camera
python action_check.py <out>/official/policy_log.json                         # commanded vs guard-clamped actions

# plain do-nothing run (task.py demo() gives the yam_scripted policy no waypoints: both arms hold READY)
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py ... --max-steps 2400

# direct demo / negative stages (trace.jsonl, result.json, demo_<camera>.mp4, setup.png, final.png)
python demo.py task.py <out>/<run> --stage demo            # named setup 0; H4_SETUP=k for setup k
python demo.py task.py <out>/<run> --stage demo --seed 7   # a seeded start state, as in inspect-robots runs
python demo.py task.py <out>/<run> --stage hold --seed 1415158438 --max-steps 2400   # full-length do-nothing: 0
python demo.py task.py <out>/<run> --stage neg-noroll      # grasp at a fixed angle, ignoring the tabs: tab on the rim, 0
python demo.py task.py <out>/<run> --stage neg-unhooked    # clip swung but not pushed past its catch
H4_REPLAY=<demo run>/trace.jsonl python demo.py task.py <out>/<run> --stage replay --seed 7   # open-loop replay: 0
python demo.py task.py <out>/<run> --stage neg-axial-press # seated, contactful straight press, no latch
python negcheck.py negcheck.json <run> [<run> ...]         # stimulus + reaction check
```
From the task root, run `python tests/test_scoring.py`, `python tests/test_wire_support.py` and `python tests/test_substep_motion.py`. With the pinned YAM package on PYTHONPATH, `python tests/test_scene.py` checks scene construction and `python tests/test_insertion.py` exercises the insertion controller and axial-only checks.

## Dimensions and contact model

| Feature | Simulated value | Basis |
|---|---|---|
| P43t-38 cap | 42.95 mm centring diameter, 37.9 mm shoulder, 51.8 mm lug circle, 22 mm cup diameter. Lug widths 7.95, 6.0, 6.0 mm at 90, 225 and 315 degrees in the rear-view frame. | OSRAM’s P43t-38 drawing and IEC cap sheet 7004-39-6, dimensions M, L, N, Z, Z1 and angle α. |
| Holder | 43.15 mm centring diameter, 41 mm relief, 1.8 mm locating wall, straight-sided slots 8.10, 8.25 and 8.25 mm wide. The outer ends open into the surrounding fixture. | P43t holder sheet 7005-39-4. The relief diameter, slot ends and 0.20 mm entry chamfer are fixture choices. |
| Rear headlamp interface | 72 mm clear opening at the dust-boot landing, 10 mm estimated recess, side hinge and fixed bracket, profiled 1.6 mm wire with a 52 mm reach and an operating end centred 11.5 mm above the axis. | Approximation from the location of the spring in Suzuki bulletin 253 pp. 6 and 9, scaled qualitatively against the 43 mm cap. The photos show the planar layout. The lip overlap, thickness, recess depth, wire diameter and travel require direct measurement. |
| Axial flange force | Four compliant bow contacts over the rear annulus, 3.5 N/mm each. With the cap nominally seated and the wire closed, 1.5 mm compression at each contact gives 21 N net axial force. | The holder sheet specifies 10–60 N of axial force on the installed cap. The chosen nominal force lies inside that range. The spring’s equal and opposite forces and moments act on the cap and wire during the trial. A round 0.8 mm wire radius gives continuous contact while a bow crosses the cap edge. |
| Lateral action at the operating end | One-sided travel up to 3 mm against a 4 N preload plus 1.6 N/mm, giving 8.8 N at the travel limit. About 2 mm of shift clears the lip. | Provisional model of wire bending. Suzuki’s bulletin gives the hooking procedure and no measured force–travel curve. The installed axial force specification governs a different direction and load case. |

[OSRAM lamp-cap drawing, PDF p. 5, left-hand P43t-38 diagram](https://www.lamptech.co.uk/Documents/Components/Osram%20-%20Lamp%20Components%20-%20Caps%20-%201980%20DE.pdf) · [Cap sheet 7004-39-6, IEC adoption](https://meganorm.ru/mega_doc/norm/gost_gosudarstvennyj-standart/18/gost_IEC_60061-1-2014_mezhgosudarstvennyy_standart_tsokoli_i.html) · [Holder sheet 7005-39-4, IEC adoption](https://meganorm.ru/mega_doc/norm/gost_gosudarstvennyj-standart/15/gost_IEC_60061-2-2017_mezhgosudarstvennyy_standart_tsokoli_i.html)

The accessible seat, boot landing, wire and bracket form the local service fixture. A stand supports the outer 4 mm of the boss plate, leaving the insertion bore open for the formed shoulder. Cosmetic fairings, optics, plug and rubber boot are omitted. The bulb mass is estimated at 18 g, including 4 g in a separate fixed-jointed glass collider. The rigid wire segments carry a one-sided contact model for wire bowing. The hinge support can move laterally under load, while the bracket remains part of the fixed housing. The wire’s 12 g solver mass, 360 degree/s angular cap and CCD limit tunnelling. Friction coefficients are estimates (metal 0.4, wire/glass 0.3 and foam 0.6). CPU PhysX runs at 480 Hz and the controller at 20 Hz.

To close the hardware comparison, measure the chosen assembly’s wire diameter, hinge-to-bracket span, lip undercut, recess and release travel from orthogonal views with a scale. With a seated bulb, record bight axial and lateral reaction versus displacement at 0.5 mm intervals through five complete engage/release cycles. Record installed axial retention separately. Compare this with the simulator’s contact-force and displacement traces, including the release point. [Hardware identification and force record](hardware-record.md)

## Asset provenance

| Component used by the task | Origin and license |
|---|---|
| Bench, floor, diffuse light, camera housing, headlamp, wire, cup and left-hand tool | Procedural geometry and materials authored in this task, MIT under the included LICENSE. The H4 room is assembled in `isaac/h4kit.py`. |
| YAM meshes and URDF underlying the installed embodiment, including the split arm visuals | I2RT Robotics, MIT at the model source commit and the Dream redistribution. [I2RT upstream](https://github.com/i2rt-robotics/i2rt/tree/120c3c81400171174604e503943f8d1ebc891058) · [Dream third-party notices](https://github.com/dreamlabs-co/dreamlabs/blob/ba8ad9850dd2e783b3c10fc079b0b5f78d7df441/dream_library/THIRD-PARTY-LICENSES.md) |
| Arm mounts, overhead and wrist camera calibration underlying the station rig | ARISE-Initiative yamlab at ec0455d2, MIT. Dream’s released example also supplies camera/framing parameters. [Calibration](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/yamlab/configs/robot/yam.yaml) · [MIT license](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE) · [Dream YAM example](https://github.com/dreamlabs-co/dreamlabs/tree/ba8ad9850dd2e783b3c10fc079b0b5f78d7df441/dream_library/examples/yam_mujoco) · [Dream 1.0.23 license metadata](https://pypi.org/pypi/dreamlabs/1.0.23/json) |
| Runtime | The externally installed `yam_arena` implements the embodiment, camera integration and robot loading in Isaac. Its pinned README reports no declared license for its integration code. The task package contains its own hooks and requires that runtime under the official embodiment name. NVIDIA supplies the simulator under its own terms. [Runtime’s pinned license statement](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1#license) |
