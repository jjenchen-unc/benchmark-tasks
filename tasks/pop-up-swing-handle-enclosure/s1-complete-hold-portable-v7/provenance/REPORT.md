S1 actual pop/contact → whole-goal candidate v5

Ready for ROOT review and one unsubmitted physical request. This is a genuine controller change, with CPU conditional/synthetic evidence only. No physical full-goal, model or submission-readiness claim. Brev remains off; no jobs, simulator, models or external messages were launched.

Actual failure diagnosis

Job43761's 1,130-control stream is retained read-only. Peak cap travel across its complete controller stream is0.423280mm, below the native2.4mm catch-release threshold; at the ends of descend/press it is0.161500/0.031642mm. Pop is a REVOLUTE coordinate in radians, not metres: its observed range is0.000942545..0.000944883rad. The source caught equilibrium estimate, including the native0.25Nm/rad pop spring,200Nm/rad catch and lever gravity, is0.000943446rad. This closely explains the observed small pop as a still-engaged catch; it does not establish an immovable world joint.

CPU geometry uses actual raw8, actual base transform and actual cap body pose, plus every authored convex collision piece of both fingertips. MinimizingZ subject to each convex piece and the finite6mm cap disk finds top-face contact gaps0.137/0.147µm at the descend/press endpoints. This is contact with the actual finite cap, not merely a TCP midpoint proxy. Its local contact is near[-0.80,5.19]mm inside that disk. Source requires about6.6N static cap force at release; the old commanded5.5mm overtravel yielded insufficient loaded travel. Arm compliance/contact loading is the supported controller hypothesis; exact dynamic force attribution is not physically measured here.

Authored pop connects swing_1→lever_1 at localZ28mm; its180°Z joint frame maps localX to native−X. Cap connects door_1→cap_1; its90°Y frame maps localX to native−Z. Neither is a world-fixed joint. The corrected world anchor fromv4 is preserved. No new adapter/API defect was established and none was invented.

The old subsequent grasp assumed32° pop while the measured lever stayed near0.054°. Its grasp therefore targeted an elevated imaginary lever. This explains why repeating the old full opening route has no diagnostic value.

Implementation

Only source/reference.py and its read-only measurement/log plumbing in source/run.py change from the physically executedv4 snapshot. Cap feedback increments target overtravel by at most0.1mm/control, caps it at16mm and targets measured2.65mm travel. It requires cap>2.4mm AND pop>26° for three observations before retreat. It refuses grasp/turn if pop<22°. The tool grasp follows measured door/swing/pop rather than assuming a popped pose. It requires both independently measured raw fingers to stop between9.1 and15mm opening, within3mm of one another, for ten controls; an empty/asymmetric grip cannot advance.

Swing targets stay at most4° ahead of measurement; lid targets stay at most3° ahead. Finite phase deadlines stop stalled cap/grasp/swing/lift. The remaining route turns past native cam release, lifts to63°, releases, retreats and holds hands off for21s, requiring measured door≥60°. Only14D YAM robot actions are issued. Target overtravel is a robot command, never a cap state write. No evaluator placements/loads enter the reference controller.

All native variants, USD assets, masses, geometry, friction, joint laws, motor caps, source thresholds, scoring, runtime credit gates and4,198-control budget are byte-preserved. Existing50N finger caps remain unchanged. The chosen jaw command0.18 has an estimated13N/pad drive preload on the22mm bar; this is a static estimate, not measured grasp force.

Verification

Ten changed-logic CPU checks PASS, including actual source mechanism thresholds, a stalled actual-like cap abort at control260, asymmetric empty-grasp rejection, and a synthetic complete route of1,275 controls. Maximum controller phase deadlines total1,680 controls. Synthetic mechanism tracking is deliberately supplied by the test, never by production; no synthetic score is reported. The CPU check walltime is6.00s.

Finite grasp geometry at hypothetical raw fingers−11mm places opposing collision surfaces at+11.031/−11.032mm across the native22mm bar. This establishes a plausible opposing-pad interface, not achieved grasp, retention or full-route clearance. Finite actual cap analysis took3.59s.

The final actual reference_runner.py was exercised with existing container Python-I-B and the process-only driver override. The wrapper preserves isolated sys.path and adds no target directory; launcher-owned bootstrap works. Actual installed TaskFile construction succeeds before the guard exits86 at isaaclab.app; simulation never starts. Embedded source manifest/fingerprint refreshed. Numeric strict JSON passes. Guard byte identity reuses the original eight verified execv/execve/Popen/raw-spawn/fork/ctypes/cleanup escape cases, plus unchanged native-variant construction evidence; no unchanged escape suite or all-SDK scan was repeated. Evidence and hashes are recorded in evidence-reuse.json and consumed-source-sha256.json.

One request for ROOT

whole-goal-request.sbatch is UNSUBMITTED: a6000 GPU1/CPU4/RAM32GB,30min, canonical chdir, existing containers/unc/run.sh --gpu, Python-I-B, and only the task-local --legacy-rtx-override process flag. Exact dispatch after ROOT review: sbatch artifacts/submission-readiness/s1-actual-pop-contact-whole-v5-20261110-0900/whole-goal-request.sbatch

Historical job43761 walltime615.58s for1,130 controls plus20 neutral controls. Estimated candidate walltime10–17min including startup; stalled cap should stop in roughly3min. These are estimates, not a fresh physical run. Outer timeout is1,650s plus15s termination grace, within30min.

The request resolves whether bounded measured cap feedback can release the native catch and then achieve a genuine whole goal. It must be reviewed using per-control raw fingers, cap/pop/swing/door and contacts, with no continuation after a missed prerequisite. Actual full-goal/retention and existing foundation channels remain unverified; successful CPU checks do not remove those physical or later official model/media requirements. No automatic promotion or submission.
