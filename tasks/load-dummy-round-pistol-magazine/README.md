# Load a dummy round under a pistol magazine's feed lips with a sleeve loader

Category: Impact on jobs. Instruction, setup, scoring, fidelity and scores: `task.json`. This README gives the sources
(numbered [n], list at the end), the method, the measured results and the known limits.

An empty 9 mm double-stack magazine stands in a printed bench fixture. One inert 9x19 dummy round (aluminium snap cap)
and a printed sleeve loader lie on the table. No firearm and no live ammunition. The robot puts the loader on the
magazine and holds it down (its tongue holds the follower down through the lip gap), slides the round in from the front
under the feed lips, raises the loader so its tooth pushes the round back to the rear wall, lifts it clear and moves
both grippers away from the round.

Status: scene, scripted demo, resets, negatives and an open-loop replay through the official `inspect-robots run`
were run in a development simulation. Not done yet: the Dream-1 run, the final videos and setup screenshot, and any
real-hardware check (see Known limits).

## Who does this work, and how often

- Occupations. Police and sheriff's patrol officers: 670,520 US workers [1]; in 2020, 98.8 % of local police
  departments authorized handguns for on-duty sworn officers [2]. Security guards: 1,283,470 [1], of whom only the armed
  part handles firearms. No national count of armed guards was found; state licence figures give the share: California
  46,827 firearm permits against 351,170 security-guard registrations [3], Florida 28,915 statewide firearm licences
  against 169,012 security-officer licences [4] (firearm permits can also be held by other licensees, and
  registrations are not the same as employed guards). Correctional officers and jailers: 380,500 [1]; not all posts
  are armed, and we found no count of armed ones.
- O*NET. Correctional officers have the task "Use weapons, handcuffs, and physical force to maintain discipline and
  order among prisoners." [5]; security guards have the work activity "Use weapons or physical force to maintain
  security." [6]. These lines are about using weapons. A pistol can only be used with a loaded magazine, so loading the
  magazine is a precondition of that use; we claim no more than that.
- Regular, mandated practice. California armed guards must pass a range qualification of 50 rounds "On two (2)
  separate occasions, at least four months apart, within each twelve-month period" (16 CCR 633) [7]. Texas peace
  officers must meet the firearms proficiency requirements "at least once each calendar year for each type of firearm
  carried"; for handguns "a minimum of 50 rounds ... including at least one timed reload", plus "a proficiency
  demonstration in the care and cleaning of the weapon(s) used" (37 TAC 218.9) [8]. The California police academy
  workbook (Learning Domain 35) has the learning objective "Steps for loading/unloading" and a section "Loading a
  Magazine" [9]. The California armed-guard firearms manual lists "Loading devices" in its range outline [10].
- Loading for others. A police trade column for firearms instructors: "If you haven't sent the class to
  lunch/dinner/break/home while you break out another case of ammo and load magazines for those who need extra help,
  in my opinion, you are not an instructor ... yet." [11]
- The tool. GLOCK lists a "Magazine speed loader" among the items supplied with the pistol [12]; it is "a sleeve that
  telescopes the magazine. A wedge inside its top presses downward on the magazine follower" [13].
- Why a robot. The work is repetitive fine insertion against a spring. Loader makers advertise "Eliminates thumb pain &
  injury" and "the rounds drop in with no thumb pushing or pressing" [14]; an NRA adaptive-shooting article says hand
  loading "relies heavily on the strength and stability of the thumbs" [15].

## Nearest tasks, by ability

The closest accepted task is the spring-pusher lane restock (one arm pulls a sprung pusher back by a handle, the other
sets two cartons into an open lane behind the old stock). This task shares only the two-arm pattern (one arm holds
the spring, the other inserts). The ability tested is different, and it lies in the insertion: the opening is closed from above by two fixed lips 8.5 mm apart, narrower than the 9.96 mm case, so
the round cannot be set in from above (that is vetoed and physically blocked). It must enter from the front, tilted,
under the lip ramp, slide rearward along the lips to the rear wall of a tube raked 21 deg, and end held by the lips
alone, pressed up against them by the follower. The spring is not pulled back by a handle but pushed down through the
lip gap by the loader's tongue.
Other near tasks: reloading a long-arm stapler (hold the sprung drawer out, lay a strip onto an open rail from above);
loading blanks into a gravity magazine (drop square parts into a channel 2.5 mm wider than the part, no spring, no
retaining lips); loading a dental cartridge (lay it in an open barrel window, then set the harpoon by a controlled
push). None of them has an insertion under fixed retaining lips narrower than the part.

## Tool and method

The printed loader follows the factory sleeve-loader method [12, 13, 16]: a sleeve slides down over the magazine top,
an internal part presses the follower down, the round goes in from the front while the loader is held down, and the
loader is then raised. In the factory method the person pushes the round rearward with thumb and index finger while
raising the loader [16]. Our loader adds a tooth at its front that makes this rearward push when the loader is raised
and pulled back; this is our adaptation for a robot with parallel grippers, not part of any commercial loader, and real
hardware must confirm it. We do not claim that workers use this exact tool. The bench fixture stands in for the hand
that holds the magazine upright.

## Start state (fixed layout, object_jitter 0)

Station frame: x forward from the line between the arm bases, y left, origin midway between the bases on the table top.
Magazine frame: origin under the centre of the mouth, x toward the magazine's front, z up. The magazine mouth centre is
at (0.200, 0.000) m with the magazine front at yaw -60 deg; the fixture base (120 x 80 mm) is centred at
(0.188, 0.020) m with its long side along the same yaw. Tool (its point over the mouth when seated) at (0.26, 0.18) m,
yaw -60 deg, standing on its ring. Round centre at (0.32, -0.26) m, yaw -90 deg (bullet to the robot's right). Both
arms at READY, grippers open. `MAG_START_ROUND` / `MAG_START_TOOL` move the round / tool for the start-pose checks only
(unset in the official run).

## Files
- `isaac/task.py`: yam_arena task file (instruction, score, success on 1.0).
- `isaac/magkit.py`: scene (magazine with follower and spring, fixture, tool, round), per-substep monitors (lip-gap
  test, magazine shift, objects off the table, tool contacts, finger loads), score hook, reset restore. A diagnostic
  counts an older centre form of the lip-gap test next to the scored case-axis form (`summary()["through_gap_substeps"]`,
  not scored).
- `isaac/partkit.py`: part builder and yam_arena hooks (convex-hull colliders, prismatic follower, 40 N finger cap,
  finger-pad friction, 480 Hz physics).
- `isaac/geom.py`: fixture colliders and the solid lower tube of the magazine, in the magazine frame.
- `isaac/task_scoring.py`: the scorer, a pure function of measured signals; `isaac/test_task_scoring.py`: its tests.
- `isaac/collider/`: magazine body and follower convex hulls and mouth dimensions (`meta.json`).
- `isaac/tool/tool_hulls.json`: the loader collider (from `build/make_tool.py`).
- `isaac/demo/`: scripted demo and negatives. `run_demo.py` (entry), `run_tool3.py` < `run_tool2.py` < `run_tool.py`
  < `run_p0.py` (motion code), `plan.py`, `tplan.py`, `reach.py` (planning), `demokit.py` (read-only demo
  measurements), `drv.py` (development-only RTX driver-check override), `official_replay.py` (open-loop replay of a
  demo's actions through the unmodified `inspect-robots run`).
- `build/`: fixture and loader (generator scripts, STEP, STL, checks), see `build/README.md`.
- `LICENSE`: MIT.

## How to run (Isaac Lab-Arena + yam_arena + Inspect Robots environment, from `isaac/`)
```sh
python -m pytest -q test_task_scoring.py                               # scorer checks, no simulator
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 6000   # do-nothing baseline
python demo/run_demo.py OUT                          # scripted demo: OUT/0-demo/actions.json, summary.json, MP4s
python demo/run_demo.py OUT --stages neg_idle,neg_drop,neg_laylips,neg_magknock,neg_toolon,neg_hold   # negatives
python demo/official_replay.py --actions OUT/0-demo/actions.json --hold-s 1 -- run --embodiment yam_arena \
  --policy mag_replay -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "<task.json instruction>" --scorer task_score --max-steps 6000 --store-frames --log-dir LOG
```
On a host whose driver fails the RTX version check, prefix `python demo/drv.py` and append `--driver-check-override`
(development only).

## Demo and measured results

Bimanual YAM: tool arm = left, round arm = right. The demo reads simulator state to plan and servo and acts only
through the 14 joint and gripper targets. Its logic runs at 10 Hz; each command is held for two 20 Hz control steps and
kept inside the official guardrails (bounds clamp, at most 4.5 % of each range per step; the CLI limit is 5 %), so the
replay through the official CLI executes the same actions. At the end the left gripper still holds the loader, 133 mm
from its seat and not touching the magazine; the instruction allows this.

Nominal start: score 1.0, first reached at step 1522 of 1614; open-loop replay of its actions through the official
`inspect-robots run` (default guardrails, 6,000-step limit): status success, task_score 1.0, 1541 steps.
Do-nothing through the official CLI: task_score 0 (2 trials of 100 steps).

Start-pose checks (one episode each; finger figures are the largest fingertip contact force, peak / largest 0.5 s mean;
the YAM fingers manage about 40 N):

| Start | Score | Tool-arm finger N | Round-arm finger N | Note |
|---|---|---|---|---|
| nominal | 1.0 (step 1522) | 32.0 / 22.1 | 20.1 / 20.1 | |
| round -5 mm x, -5 mm y, -10 deg | 1.0 (1527) | 32.0 / 22.3 | 20.7 / 20.6 | |
| round +5 mm x, -5 mm y, -10 deg | 1.0 (1589) | 36.7 / 28.7 | 20.9 / 20.8 | |
| tool +5 mm x, +5 mm y | 1.0 (1507) | **42.5** / 22.8 | 20.1 / 20.1 | peak over 40 N at the seat impact |
| round +5 mm x, +5 mm y, +10 deg | 0.25 | 32.0 / 22.1 | 19.9 / 19.6 | loader pitched in the gripper during the rearward push; round yaw 5.05 deg (limit 5) |
| tool -5 mm x, -5 mm y | 0.25 | **89.3 / 78.0** | 20.1 / 20.1 | loader pitched and slid 41 mm in the gripper; round 5.35 mm short of the rear wall |

So 4 of 6 starts reach 1.0, and 3 of 6 reach it with every fingertip under 40 N. Only the nominal start is supported;
no randomisation range is claimed. An optional change to the rearward push (wait for the loader to follow the command)
was tried on four starts and was not better (nominal 1.0; tool -5 mm 1.0 but with a 555 N peak; two others 0.25, one
with 94.7 N); it stays off.

Negatives (scripted, through the same scene and live scorer):

| Run | What happened | Score |
|---|---|---|
| idle | nothing moves (400 steps) | 0 |
| drop | round dropped on the table, 190 mm from any fingertip | 0 |
| round laid on top of the lips | round released 1 mm above the lips, stays on top (6.3 mm above the lip line); veto "round entered or rested in the lip gap from above" | 0 |
| loader left on | round under the lips but 5.25 mm short of the rear wall (not loaded) | 0.25 |
| gripper still holding | round 4.52 mm short (not loaded), fingertips 16 mm from it | 0 |
| magazine knocked | stimulus failed: largest magazine shift 0.92 mm / 1.03 deg, below the 2 mm / 2 deg veto | 0 |

The magazine-moved and object-fell vetoes and the "loaded with the loader still on" case are checked only by unit tests;
a tool-free loading attempt was not run.

## Known limits
- Fixed start only (table above): two of six nearby starts fail, one loads with a 42.5 N fingertip peak, and the nominal
  run's 32 N peak / 22 N sustained on the tool arm leaves about 8 N to the 40 N finger figure on an assumed spring.
- One round, the first and easiest: the spring is weakest with the magazine empty.
- The follower spring force is an estimate, not a measurement. No force-by-round-count data for this magazine was
  found. How the estimate was made: (1) one published force-gauge reading for a factory magazine, 5.8 with no unit and
  no stated compression, was read as 5.8 lbf (about 26 N) with the magazine full; (2) the spring was taken as linear
  (170 N/m along the tube) and scaled back over the follower travel, which gives about 10 N under the first round and
  8.3 N with the follower on its stop. A second published reading, 8.3 lbf (about 37 N) at full compression [16],
  scaled the same way gives about 12 N on the stop and about 14 N under the first round, so the real spring may be
  about 40 % stiffer than the model. The demo was not rerun at that value.
- A real trial needs a real factory magazine: the feed lips, follower and spring of the real part decide the result,
  and a printed copy of the model would only repeat our assumptions. Before trials, measure on that magazine the
  follower force on its stop and at 6.5 mm depression (push-pull gauge), the lip gap and the lip length (calipers), and
  check that the printed fixture and loader fit; if the lip gap or the force differ from the model, the loader tongue
  and the scripted demo may need adjusting.
- Magazine mouth, lips and follower come from a hobby CAD model with no stated license (see the asset table); the lip
  gap is set to 8.5 mm (a published spacing is 0.34 in, 8.6 mm [16]). Nothing has been measured on a real magazine,
  and the fixture and loader have not been printed or test-fitted.
- The loader's tooth is our adaptation (Tool and method). The round-arm wrist is 7.4 deg from its joint limit at the
  entry, so a different layout may be unreachable.
- Round mass 4.5 g is assumed (published A-Zoom pack weights conflict); a live 9 mm round weighs about 12 g [17].
- 17-round magazines are restricted in some US states; a 10-round body must be checked for the same mouth.

## Scoring (task_scoring.py) and the real-trial checks

The code and `task.json` scoring say the same thing; constants in brackets.

| Condition | Simulator test | Real-trial check |
|---|---|---|
| under the lips | case centre no higher than the lip line + 0.3 mm (`UNDER_TOL`); base behind the constant-gap lips; centre within the tube width; tilt < 10 deg; bullet axis within 20 deg of the front | photo into the mouth: case under both lips, bullet forward; magazine turned mouth down and tapped: round stays |
| base at the rear wall | base < 2 mm from the raked rear wall (`GAP_MAX`) and case mouth behind the lip-ramp end; for a level round the mouth rule binds at about 0.6 mm | a 0.6 mm feeler cannot be slid down behind the base |
| on the lip line | abs(centre - lip line) < 1 mm (`DZ_MAX`) | round pressed 2 mm down springs back against the lips (coarser) |
| square | yaw < 5 deg (`YAW_MAX_DEG`) | photo from above, case square to the lip edges (coarser) |
| tool off | tool contact < 0.05 N and its mouth point >= 72 mm from its seated position (`TOOL_OFF`; lifted straight, the ring bottom is then about 8 mm above the lip tops) | nothing of the tool touches the magazine or round, ring above the lip tops |
| hands away | every finger-pad point (TCP and pad tip, 6 mm outside each pad, both arms) >= 50 mm from the round centre (`CLEAR_MIN`) | video with a ruler in view |
| held 2 s | lowest level over the last 2 s (`HOLD_S`) | video |
| veto: round in the lip gap from above | any point of the case axis over the constant-gap lips more than 1 mm above the lip line while the case's lowest point is below the lip tops (`THROUGH_TOL`), any physics substep | video |
| veto: magazine moved | > 2 mm or > 2 deg from its start pose (`MAG_SHIFT`, `MAG_TILT_DEG`), any substep | pencil line across magazine and tower offset > 2 mm |
| veto: object fell | more than 50 mm below the table top (`FALL`) | video |

Levels: 1.0 loaded and tool off; 0.25 held under the lips but not loaded, or loaded with the tool still on; 0 otherwise.
The lip-gap veto is named for what it detects: a round entering or resting in the gap from above. A round slid in from
the front under the lips never meets it (no substep in any loading run); a round laid on top of the lips does.

## Physics notes
- PhysX 480 Hz (decimation 24 at 20 Hz), maximum depenetration velocity 10 m/s, round and magazine as convex hulls.
- Follower: prismatic joint on the 21 deg tube axis with a linear spring drive, 170 N/m, 8.3 N on its stop (estimated,
  see Known limits: 10 N at round 1, 26 N at round 17).
- Lip gap 8.5 mm. Frictions assumed: round 0.4, magazine, fixture and tool 0.35, finger pads 0.8 / 0.7. Finger force
  capped at 40 N. Magazine mass 75 g (a published empty weight is 2.7 oz, 77 g [16]).
- Loader: tongue 6.5 mm, presses the follower 5.5 mm in the nominal run with the loader seated; 74.65 g at 100 % PLA.

## Third-party assets and licenses
The SDK is yam-isaac-lab-arena at bd62771e60320433893bd1d2df0fc79b5ddcd6d1, installed by the user's environment.
Nothing from it is redistributed here. Licenses are given as each source states them.

| Asset | Used for | Source | License as stated by the source | Bundled |
|---|---|---|---|---|
| YAM arm and gripper (URDF, meshes, collision hulls, USD) | both arms | SDK `yam_arena/assets/yam/`, from i2rt (SOURCE_COMMIT 120c3c81400171174604e503943f8d1ebc891058) | MIT, Copyright (c) I2RT Robotics (`LICENSE.i2rt` there) | no |
| Station floor, table, top-camera stand, lights, cameras | scene | SDK station code: shapes generated in code, no mesh or texture file | SDK README: "This repository has no license yet" (Placeholder) | no |
| Isaac Lab-Arena | simulation framework | Isaac Lab-Arena | Apache 2.0 | no |
| Inspect Robots | runner, scorer, video | inspect-robots | as stated by the package | no |
| Arena object library (cube, plate) | not used | - | - | no |
| Magazine body, lips and follower shape | `isaac/collider/` hulls are derived from it | community STL "Glock17-3D-print-model" (hobby model) | none stated; origin unknown | derived hulls only |
| Fixture, loader, dummy round shape | task fixture | this folder (`build/`, `isaac/`) | authors' own work, MIT (`LICENSE`) | yes |

## Sources (read 2026-11-19)
1. BLS OEWS, national employment May 2025: security guards https://data.bls.gov/timeseries/OEUN000000000000033903201 ; police and sheriff's patrol officers https://data.bls.gov/timeseries/OEUN000000000000033305101 ; correctional officers and jailers https://data.bls.gov/timeseries/OEUN000000000000033301201
2. BJS, Local Police Departments, Procedures, Policies, and Technology, 2020 (Table 3, all sizes, on-duty handgun 98.8 %): https://bjs.ojp.gov/document/lpdppt20st.pdf
3. California BSIS staff report, April 2026 (licensing population through Feb 25, 2026): https://www.bsis.ca.gov/about_us/agendas/20260416_staffr.pdf
4. Florida FDACS, Number of Licensees by Type (October 2026): https://fdacs.gov/content/download/82618/file/number-of-licensees-by-type.pdf
5. O*NET 33-3012.00 Correctional Officers and Jailers: https://www.onetonline.org/link/details/33-3012.00
6. O*NET 33-9032.00 Security Guards: https://www.onetonline.org/link/details/33-9032.00
7. California Code of Regulations, title 16, section 633 (BSIS regulations): https://www.bsis.ca.gov/about_us/laws/bsis_regulations.pdf
8. Texas Administrative Code, title 37, section 218.9: https://www.law.cornell.edu/regulations/texas/37-Tex-Admin-Code-SS-218-9
9. California POST Basic Course Workbook, Learning Domain 35, Firearms/Chemical Agents: https://post.ca.gov/portals/0/post_docs/basic_course_resources/workbooks/LD_35-V4.0.pdf
10. California BSIS Firearms Training Manual (July 2023): https://www.bsis.ca.gov/forms_pubs/firearms_manual.pdf
11. Police1, "7 investments worth every penny for firearms instructors" (2024): https://www.police1.com/firearms-training/articles/7-investments-worth-every-penny-for-firearms-instructors-U9xEXHHLbePeC3WW/
12. GLOCK Instructions for use, Gen6 (14.10.2025), items supplied: https://eu-assets.contentstack.com/v3/assets/bltf7171cc1cfc1a31f/blt20f0aed102bfffd5/69f4e966d8e9145f380aab42/51959_Instructions-for-use_Relaunch_Standard_Gen6_14102025-EN-low.pdf
13. American Rifleman, "The Glock 17 Pistol: American Rifleman's Original Review" (1986, republished 2023): https://www.americanrifleman.org/content/the-glock-17-pistol-american-rifleman-s-original-review/
14. maglula, UpLULA product page: https://www.maglula.com/product/uplula/
15. NRA Family, "Arthritis Tips: How to Load & Unload Semi-Autos" (2022): https://www.nrafamily.org/content/arthritis-tips-how-to-load-unload-semi-autos/
16. Factory-loader use, step by step (forum walkthrough, 2007): https://www.glocktalk.com/threads/glock-speed-loader.713927/ ; Glock 17 Gen 4 magazine measurements (polymer body, 2.7 oz, spring 8.3 lbs at full compression, feed-lip spacing 0.34 in), 9MMMagazines 2022: https://9mmmagazines.com/glock-17-magazine-test/
17. Sellier & Bellot 9 mm Luger 115 gr FMJ data sheet (cartridge 12.2 g): https://www.sellierbellot.us/products/pistol-and-revolver-ammunition/pistol-and-revolver-cartridges/pdf/290/
