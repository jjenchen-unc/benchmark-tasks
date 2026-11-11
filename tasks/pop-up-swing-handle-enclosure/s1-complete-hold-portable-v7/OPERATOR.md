S1 prepared final validation inputs; ROOT review pending. Brev stays OFF until the user opens it. No service-management command is provided. Do not edit, stop or duplicate ROOT's queued43823. No additional unchanged SERVER study is required by this bundle.

Extract s1-v6.tar.gz into a new directory. PKG is the extracted s1-v6 directory; PY is the existing pinned SDK Python; OUT is a new writable output parent. Runtime code resolves local inputs relative to PKG. Provenance records historical server paths only. Set the process-local driver override only when required by the installed driver.

```bash
export PKG=/path/to/s1-v6
export PY=/opt/IsaacLab-Arena/.venv/bin/python
export OUT=/path/to/fresh-s1-output
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1
export S1_LEGACY_RTX=1
"$PY" -I -B "$PKG/verify_runtime.py"
"$PY" -I -B "$PKG/run_phase.py" replay --out "$OUT/replay"
"$PY" -I -B "$PKG/receipt_builder.py" --replay "$OUT/replay" --out "$OUT/review"
```

Replay issues only the exact v5 Teacher's public14D robot targets. There are no evaluator placements/loads or direct task state/score/timer writes in this route. The wrapper starts one fresh SH-T1 native case42 at20Hz/4198 controls, object_jitter0, right active/left parked. Reset RGB PNGs, all three synchronized H264/yuv420p videos, before/after raw8/all-link FK, fixture cap/pop/swing/door, contact status, per-action feedback and native1kHz substep measurements are recorded. Native six-channel gates stay empty/publicscore0 throughout replay. The reference begins immediately after reset; the old runner's unrelated20-neutral/reset diagnostics are omitted, without changing Teacher bytes, objective or budget.

receipt_builder independently runs the exact native History over every fresh substep, requires full native goal and continuous20s actual no-contact hold, a finite14D action stream within4198, measured50substeps/control and complete media. Failures, partial runs, elapsed jumps, missing media and source changes refuse a review form. Kit exit0 is insufficient. The generated ROOT-REVIEW-UNAPPROVED.json has every channel false. ROOT reviews genuine cap release/pop, opposing loaded fingers, physical swing/lid opening/release, final hands-off hold, actual fixture setup, force/contact/motor/reset fidelity and blind media. Add relative evidence filenames+SHA256 for each of clock, joint_state, motor_total_limits, passive_mechanisms, robot_all_box_contacts and wrist_force, retaining unresolved findings. Reuse source-equivalent original proofs where valid. Set each approved flag with rationale, reviewer ROOT and approved true, then save ROOT-APPROVED.json alongside the copied replay. This is a local reviewer trust boundary, not sensor attestation or cryptographic identity.

```bash
"$PY" -I -B "$PKG/receipt_builder.py" --validate "$OUT/review/ROOT-APPROVED.json"
"$PY" -I -B "$PKG/run_phase.py" setup --out "$OUT/setup"
"$PY" -I -B "$PKG/run_phase.py" model --review "$OUT/review/ROOT-APPROVED.json" --out "$OUT/model"
```

Setup records fresh reset RGB without actions. Model refuses missing/unapproved/changed-source or altered replay receipts before simulator/provider imports. After explicit ROOT approval, model_task supplies exact native objects/scorer and enables only the six original channel names. The actual Inspect Dream-1 entry uses one continuous session, one attempt per request, rows_per_request4, the exact task.draft instruction, absolute14D YAM targets,20Hz/4198 controls, cpu physics/RTX cameras, no jitter and CLIseed42. Inspect's derived integer reset seed is passed unchanged; it does not alter native fixed physicalcase42. verify_model checks actual summary/record, synchronized1kHz clocks, native score, finite action count, continuous session and stored frame sequence, then encodes the real frames. A completed score0 model trial remains permissible with a separate fullscore demo. No provider/model calls occurred in preparing this bundle.

Fixture translation[0.25,0,-0.0525]m and identity orientation are unchanged. No new mount/pedestal/shape/mass/friction/mechanism/cap/threshold/scorer/budget change. All original variants and exact inventory remain in candidate/fixture.json. The original manifests remain byte exact; manifest.json refreshes the outer bundle fingerprint. full-consumed-source-map.json retains original source/asset proofs and current preSDK consumed sources without rehashing the entire SDK. runtime-source-pins.json reuses B1's verified unchanged installed API pins; verify_runtime checks only that map.

Evidence limits: actual43761 peak cap0.423280mm vs native2.4mm release; pop0.000942545..0.000944883rad and zero opening. Newv5 cap feedback targets2.65mm with16mm maximum robot overtravel/0.1mm increments; measured pop and balanced independent raw fingers gate advancement. CPU conditional complete route1275 controls, max deadlines1680, nativebudget4198. Finite actual cap contact and opposing-pad grasp geometry remain conditional proofs, not achieved retention. No old physical score transfers. Native SH-T1 full credit requires lid strictly>60deg and continuous20s no robot contact, with sticky raw wrist>80N for0.1s failure. The controller's21s hands-off phase provides margin without changing that predicate.

Walltime: old43761 measured615.58s/1130controls plus20neutral controls. New replay estimate10–17min including startup; contact/render/log cost may increase it. Stalled cap expected~3min. Whole-run walltime is unmeasured. Setup estimate1–4min; model estimate0.5–3h from variable policy latency, unmeasured. CPU guard~1s; synthetic protocol~3s. GPU_REQUEST_UNSUBMITTED.sbatch is an optional ROOT-only request: GPU1/CPU4/RAM32GB/30min, canonical chdir, existing containers/unc/run.sh --gpu,1650s timeout+15s grace, process-only driver override. It is not submitted. Existing supervisor quota<=45 no-new/<=40 stop governs all dispatch. Intended remaining physical validation is tomorrow's user-opened Brev, followed by ROOT channel/media review and actual model validation. Publication/live ownership/upload linkage remain outside this private input bundle.
