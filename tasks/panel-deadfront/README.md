# Refit an outdoor load center deadfront

A pair of YAM arms refits a Leviton LRDFC-P20 inner deadfront on its matching LR222-LDD outdoor enclosure. The plate starts upright in a stand. The hands grasp two removable magnetic flat-faced handles, lift the plate, insert the upper edge above both side guides at 45° and bring the lower edge inward so six breaker shrouds pass through their openings. The right hand uses a magnetic ratcheting driver to install one lower-right screw while the left supports the plate. Both hands finish clear. The setup represents a disconnected training fixture. [Product identification, photos, dimensional estimates and Figure 12](hardware-fit.md).

## Geometry and scoring

The tub measures 762 × 381 × 111.76 mm (H × W × D). The plate measures approximately 668 × 340 mm. Six LB120-T single-pole breakers occupy three consecutive spaces per column. Fourteen unused twist-outs and the central main-lug blank remain closed. The two removable rubber flat-faced handles use magnetic bases on solid areas of the plate. The assumed 85 g per handle brings moving mass to 2.30 kg. Both arms carry the plate together. Before the right hand releases, the controller requires an upward tub reaction of at least 11.3 N. The free-body plate and driver move under PhysX contact. The screw begins in the magnetic holder, meets the pilot from outside, and advances on positive driver rotation. Screw pitch, holder contact, sheet thickness, folds and guide profiles are approximations.

| Score | Observable state |
|---|---|
| 0 | Drop, missing angled guide entry, lost guide registration, a corner more than 1 mm off the mounting plane, or a shroud outside its assigned window |
| 0.25 | Upper edge entered at 45° ± 6° and stayed registered while the plate closed, with all corners flat and six shrouds through |
| 0.5 | The lower-right screw is also down |
| 1.0 | In addition, the four fingertips clear the plate by 30 mm and the driver clears the screw by 15 mm, continuously for 1 s while still |

## Running the task

From the task root run `python -m pytest tests` for geometry, scorer and thread tests with the Python YAM package available. Run `python isaac/check_reach.py` to check the nominal two-arm carry and ratchet poses against the installed YAM URDF. In the YAM Isaac environment run `python isaac/demo.py isaac/task.py /tmp/panel-ref --stage demo`. The official policy is `panel_deadfront_scripted` in `isaac/official_run.py`. The control rate is 20 Hz and the limit is 3,600 steps. Negative stages are `neg-straight`, `neg-no-screw` and `neg-held` with expected scores 0, 0.25 and 0.5. `idle` expects 0.

## Verification and recordings

The official reference completed in 3,393 of 3,600 steps with a score of 1.0. The screw advanced 16.78 mm over 10.57 turns. The finish required the hands and tool to clear the secured plate for one second. The straight-insertion check presented the plate without engaging the upper guides and scored 0.0 in 472 steps. Dream-1 scored 0.0 at 3,600 steps on seed 0. Its recording uses the same scene, instruction, scorer and runtime as the reference. Later changes affected the scripted driver controller. The submitted media include both full three-camera runs and the initial overhead reference frame.

## Third-party assets and licenses
| Asset | Source | License |
|---|---|---|
| YAM arm model (URDF, meshes) | i2rt, via the Isaac Lab-Arena YAM package | MIT |
| Station scene, cameras, `yam_arena` embodiment | Isaac Lab-Arena YAM package | as distributed with the package (no license file at the pinned revision) |
| Isaac Sim 6.1 / Isaac Lab-Arena | NVIDIA | NVIDIA Isaac Sim license / Apache-2.0 |
| Load centre, breakers, cover, screw, stand, driver, holster geometry | generated in code by this task (`src/layout.py`) | MIT (this repository, `LICENSE`) |
| `src/fastik.py` (YAM URDF kinematics) | this task's authors | MIT |
