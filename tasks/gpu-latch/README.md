# Pull a single-slot GPU out of its PCIe x16 slot: hold the latch open under the card while the other hand draws it up

Benchmark task for a bimanual YAM station (Isaac Sim 6.1 / Isaac Lab-Arena `yam_arena`, PhysX).
Category: Impact on AI timelines (AI workstation / GPU server service).

Instruction: "Take the graphics card out of the motherboard and lay it on the anti-static mat."

## The task
A single-slot RTX A4000 sits in a representative x16 slot inside a side-open workstation chassis. The unpowered tower lies flat. Its side cover, PCIe door, auxiliary power lead and GPU end holder have been cleared. The bracket slides between the rear slot guides. The slot's retention latch, at its inner
end 141 mm from the bracket, hooks the notch at the end of the card's gold fingers. The latch is under the card: its
thumb pad is reachable only beside the card's back face (the cooler is on the other side).

- The latch must be pressed down (about 6-7 N on its thumb pad) and held while the card is drawn straight up. The
  contact rows grip the gold fingers with about 20 N, so the card comes free with a jerk once the pull exceeds about
  25 N (friction plus weight).
- If the latch is let go before the card's lip is past the hook (about 3 mm of rise) the hook springs back and catches
  it again. Pulling the card past the hook before the hook clears counts as forced extraction.
- The card is then carried to the anti-static mat on the table and laid down flat, and both hands move away.

Observable cues: the latch's thumb pad visibly tips down (about 20 deg) when pressed; the card's rise out of the slot
and the gap under the gold fingers are visible from the wrist cameras.

## Scoring (`src/scoring.py`, value at the end of the trial)
- 1.0 ends the trial when the mat supports the extracted card for three continuous seconds. The complete card footprint, including its metal bracket, must fit inside the mat. Its lowest point must lie within 5 mm of the mat, its face within 15° of horizontal, and every collision vertex must move slower than 5 mm/s. Every fingertip must be at least 40 mm away and the robot must have released all contact. The latch must have cleared before extraction, and the card must have avoided a drop.
- 0.5: the card is out of the slot, latch intact, never dropped, but not lying released on the mat (still held, or
  set down elsewhere).
- Score zero while the card remains seated. A forced extraction or an unsupported fall of 30 mm latches zero for the trial. A supported lowering motion remains eligible for placement credit.

From the side or wrist video, verify that the thumb pad pivots clear before the lip rises past the hook. Use the card or a scale in the same plane to judge a free fall. Inspect the real latch after extraction for cracking or a missing hook. The simulation records forced passage; it does not simulate fracture. The per-step contact load is telemetry only.

## Verified runs

All three runs used Inspect Robots CLI seed 0 (episode seed 1415158438), 20 Hz control and the same physical scene, task instruction and scorer.

| Policy or control | Task score | Steps | Observed outcome |
|---|---:|---:|---|
| Scripted reference | 1.0 | 774 | Latch reached 20.02° before withdrawal. At step 662 the mat supported the card with 2.90 N. The full placement and release hold then completed. |
| Dream-1 beta | 0.0 | 1,500 | The model issued 147 action requests and used the entire step budget. |
| Pull without pressing the latch | 0.0 | 279 | The retention lip passed the hook with no hand pressure on the rocker. Forced extraction latched the score at zero. |

## Dream-1 evaluation

Run the following from `isaac/` inside the configured YAM environment.

```bash
inspect-robots run --embodiment yam_arena --policy dream \
    -P model=dreamlabs/dream-1-beta -P rows_per_request=10 \
    -E task=task.py -E object_jitter=0 -E device=cpu \
    -E control_hz=20 -E cam_width=320 -E cam_height=176 \
    --instruction 'Take the graphics card out of the motherboard and lay it on the anti-static mat.' --scorer task_score --seed 0 --max-steps 1500 --store-frames --log-dir ./dream-output
```

## Run and score a trial
Inside the Isaac Sim 6.1 + Isaac Lab-Arena container, working directory `isaac/` (`isaac/run_frozen.sh` does this on a
frozen copy of `isaac/`, `src/` and `task.json`, writes `source-sha256.txt` and logs to `<out>/<run>/run.log`; see its
header for the environment variables it needs):

```
# official Inspect Robots run of the scripted reference demo (registered closed-loop policy gpu_latch_scripted)
python official_run.py -- run --embodiment yam_arena --policy gpu_latch_scripted -E task=task.py -E object_jitter=0 \
    -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 \
    --instruction "$(python -c 'import json;print(json.load(open("../task.json"))["instruction"])')" \
    --scorer task_score --max-steps 1500 --store-frames --log-dir <out>/official
inspect-robots video <out>/official/<log>.json --out <out>/official/video     # H.264 video of every camera

# plain do-nothing run (task.py demo() gives the yam_scripted policy no waypoints: both arms hold READY)
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py ... --max-steps 1500

# direct demo / negative stages (trace.jsonl, result.json, demo_<camera>.mp4, setup.png, final.png)
python demo.py task.py <out>/<run> --stage demo --seed 1415158438        # the default inspect-robots start
python demo.py task.py <out>/<run> --stage demo                           # named setup 0; GPU_SETUP=k for setup k
python demo.py task.py <out>/<run> --stage neg-nopress --seed 1415158438  # pull without pressing the latch: 0
python demo.py task.py <out>/<run> --stage neg-early --seed 1415158438    # press, let go, then pull: 0
python demo.py task.py <out>/<run> --stage neg-held --seed 1415158438     # drawn out, still held at the end: 0.5
GPU_REPLAY=<run>/trace.jsonl python demo.py task.py <out>/<run2> --stage replay --seed 14218   # open-loop replay
GPU_START="0,45,0;5,45,2" GPU_REPLAY=<run>/trace.jsonl python demo.py task.py <out>/<run4> --stage replay,replay   # boundary starts
python demo.py task.py <out>/<run5> --stage view --seed 1415158438 --cam-size 1280x704   # oblique setup picture
GPU_REPLAY_MIRROR=1 GPU_REPLAY=<run>/trace.jsonl python demo.py task.py <out>/<run3> --stage replay --seed 1
```
Run the unit checks with `python -m pytest tests/`.
Tiled evidence video (top + left wrist + right wrist, same timing):
`isaac/tile.sh <out_dir> <name> demo_top_cam.mp4 demo_left_cam.mp4 demo_right_cam.mp4`.

## Start states
A seeded reset moves the entire workstation by up to 25 mm in x and 60 mm in y, and rotates it by up to 5° around the vertical axis. The mat stays fixed. Inspect Robots CLI seed 0 produces episode seed 1415158438. Other episode seeds resample until the latch pad moves by at least 54 mm across the slot in the positive slot-normal direction, 24 mm across it in the opposite direction, or 60 mm along the slot. With this case orientation, a check of episode seeds 0 through 4,999 found positive across-slot offsets of 54–121 mm. These distances describe the reset geometry. They do not measure policy success across seeds.

Unseeded resets cycle through four marked chassis positions. `GPU_SETUP` selects a named position. `GPU_START="along,across,dyaw;..."` accepts offsets of the latch pad in millimetres along and across the official slot and degrees of yaw for controlled replay checks.

## Fidelity (full statement)
- Engine: Isaac Sim 6.1 / Isaac Lab-Arena `yam_arena`, PhysX on the CPU, physics 240 Hz, control 20 Hz, cameras
  320 x 176. All parts are full size.
- Card: NVIDIA RTX A4000 class single-slot card, 241.3 x 111.15 mm (datasheet), 0.50 kg (estimate; not on the
  datasheet), modelled as convex boxes: gold-finger tab (1.57 mm PCB), retention lip and notch, PCB, cooler shroud
  (14.5 mm), back cover, steel bracket with its top flange. The blower fan, trim and DisplayPort sockets are visual.
- Slot: PCIe x16 connector housing 89 mm long. The card stands in its channel with 0.5 mm side clearance; the channel
  walls, the end walls and the two slot-cover guides on the bracket bar only locate the card (frictionless).
- Contact friction (the connector's two contact rows): not individual spring contacts but a computed Coulomb friction
  on the card along the slot axis, limit 2 x 0.3 x 34 N = 20.4 N while the gold fingers are between the rows (full for
  the first 3 mm of rise, falling to 0 by 4.5 mm), with a stick phase (a 10 kN/m spring to an anchor that slips at the
  limit, so about 2 mm of give before the card breaks free) and applied at the card's centre of mass. The modeled contact load of 20.4 N falls within the 19.1–21.0 N withdrawal range measured by Molex on a 164-contact connector (mean 20.6 N). That fixture used a steel test blade. The connector in this Dell workstation has not been identified or pull-tested. [Molex TS-87715-207, §9.2](https://www.molex.com/content/dam/molex/molex-dot-com/products/automated/en-us/testsummarypdf/877/87715/TS-87715-207-001.pdf)
- Latch: a rigid L-shaped rocker on a hinge (hard limits 0-25 deg) with its pivot under the hook, so a pull on the
  card seats the hook. Its torsion spring is a computed torque (0.035 N m preload, 0.12 N m/rad; estimate): about
  6-7 N on the middle of its thumb pad opens it past the hook (17 deg); a 2 N touch does not move it. The thumb pad is
  20 mm wide, wider than many latches, so the YAM fingertip can press it beside the card's back (stated, not hidden).
  Latch inertia is raised to 4e-6 kg m^2 for a stable spring at 240 Hz.
- A closed rocker moving aside under a rising card is a forced extraction. The simulation tracks the retention lip itself and requires the rocker to clear the seated lip before withdrawal. Grip and support contact distinguish a drop from lowering a supported card.
- Friction: gripper pads on the card's shroud and back cover 0.9 (rubber on plastic, estimate, "max" combine); mat
  0.8; other parts 0.4-0.5 (PhysX averaging).
- Four chassis walls, a floor, rear slot cutout and bracket guides collide with the robot and card. The tower stands with its long axis pointing forward between the ready arms. The 420 × 369 × 173 mm outer envelope follows Dell's tower specification. The board position, cutout and 143 mm lip height relative to the board are estimates. Adjacent slots and components are simplified blocks.
- Anti-static mat: 457 x 180 mm (cut ESD bench mat), 2 mm, fixed on the table.
- Contact offset 1 mm, rest offset 0. The card and latch never sleep.
- Nothing welds, snaps or teleports the card; the scripted demo only sends the 14 joint / gripper commands and reads
  poses. On an environment reset (between trials only) the kit puts every body back at the next start state.

## Physical workstation and verification

Use a Dell Precision 3660 Tower with the single-slot NVIDIA RTX A4000. The 3660 lists this GPU as a supported option. NVIDIA specifies the 4.4 × 9.5 inch card and identifies model training and data science among its uses. [Dell specifications](https://www.dell.com/community/assets/community/687062f5-603c-4f5f-ab9d-31aa7cacb376/precision3660specsheet-2334ad54-a883-4af0-8d90-062a4973d389-1383239821.pdf) · [NVIDIA RTX A4000 specifications](https://www.nvidia.com/en-us/products/workstations/rtx-a4000/)

The BAKU robot-learning study trained its multi-task policies on a single RTX A4000. The authors report approximately six hours for the xArm training run and ten and a half hours for LIBERO. This is the GPU workload a replacement can restore. The authors do not identify their workstation chassis. [BAKU, NeurIPS 2024, appendix A](https://papers.neurips.cc/paper_files/paper/2024/file/ff887781480973bd3cb6026feb378d1e-Paper-Conference.pdf)

Puget Systems tracks professional RTX card failures in customer workstations. Its 2024 report records a field failure in the RTX Ada line, a later generation than the A4000. This documents a workstation maintenance use case. Service frequency for the Precision 3660 with an A4000 remains unknown. [Puget Systems workstation reliability report](https://www.pugetsystems.com/labs/articles/puget-systems-most-reliable-hardware-of-2024/)

Follow Dell's service preparation on a powered-down, disconnected machine on its side. Remove the cover, disconnect auxiliary power, clear the retaining door and remove the GPU end holder before reset. The scored portion starts at the exposed card and ends with the card on the adjacent ESD mat. Dell instructs the operator to hold the slot tab down during the lift. [Dell powered-GPU removal](https://www.dell.com/support/resources/en-sg/3dviewer/ca0700t3660002204a/how-to-replace-the-powered-gpu)

For a physical replication, photograph the latch beside the installed card and the rear opening at card height with a scale. Measure the exposed thumb pad, rim-to-board height and bracket opening. The simulated 20 mm pad width and internal dimensions are estimates. No physical reach or extraction test has been performed. Restore the assembly by seating the bracket in its opening, then lowering the card squarely until the tab engages.
 
## Third-party assets and licenses
| Asset | Source | License |
|---|---|---|
| YAM arm URDF and meshes | [I2RT Robotics model source](https://github.com/i2rt-robotics/i2rt/tree/120c3c81400171174604e503943f8d1ebc891058) | MIT. The installed runtime records the model commit in `yam_arena/assets/yam/SOURCE_COMMIT`. |
| Station, cameras and YAM embodiment | [YAM Arena runtime used on Brev](https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730#license) | Installed dependency. Its README reports that the integration code has no declared license. |
| Isaac Sim 6.1 and Isaac Lab-Arena | NVIDIA and [Isaac Lab-Arena](https://github.com/isaac-sim/IsaacLab-Arena/blob/c7edf3a9d5d8276565208f72422f8f8a0806d6cf/LICENSE.md) | NVIDIA Isaac Sim license and Apache-2.0 for the Arena package, respectively. |
| Benchtable, motherboard, slot, latch, card and mat geometry | generated in code by this task (`src/layout.py`, `isaac/gpukit.py`) | MIT (this repository, `LICENSE`) |
