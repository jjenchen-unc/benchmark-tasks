from pathlib import Path
import json,hashlib,tarfile,shutil,ast
O=Path(__file__).resolve().parent
D=O.parents[1]/'continuation/server-b6-strict-numeric-boundary-20261110-1538/B6_portable_strict_physics_numeric_boundary'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
load=lambda p:json.loads(p.read_text())
write=lambda p,v:p.write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
m=load(O/'manifest.json');native=load(O/'provenance/v14-candidate-sha256.json')
assert len(native)==38
for n,h in native.items():assert sha(O/'candidate'/n)==h
for n,h in m['files'].items():assert sha(O/n)==h
for p in O.rglob('*.py'):ast.parse(p.read_text())
checks={p.name:load(p) for p in (O/'checks').glob('*.json') if p.name!='synthetic-actions.json'}
assert all(x.get('passed') is True for x in checks.values())
consumed={}
for x in checks.values():consumed.update(x.get('consumed_source_sha256',{}))
write(O/'full-consumed-source-map.json',dict(candidate_fingerprint=m['fingerprint'],package_sources_sha256={str(O/n):h for n,h in m['files'].items()},clean_process_consumed_sources_sha256=consumed,inherited_v15_full_map=load(O.parent/'b6-fullportable-v15-20261110-1525/full-consumed-source-map.json'),actual43967_trace_source=load(O/'provenance/actual43967-excerpt-source.json'),scope='Preserved original proof maps plus changed strict parsing boundary and actual entry imports; CPU diagnostics are not physical proof.'))
report='''B6 v16 strict numeric boundary repair, complete private input bundle for ROOT review. CPU conditional/synthetic evidence only; no full-goal, submission-ready or old-score-transfer claim.

Concrete failure repaired: v15 protocol.fresh invoked native History.update before strict field ownership, allowing coercion of string/bool wrist values, bool stop torques and integer held flags. V16 validates decoded physics row structure and numeric ownership before any History update. wrist_n must be a list of exactly two finite JSON numbers, excluding bool/string; stop_nm/drop_velocity_z have exact native keys and finite number values; held/captured and robot_touch require actual booleans. Existing numeric_score.validate_measurement is reused unchanged. Additional native push/mount structures and trace control/time ownership are checked. V16 ROOT receipt schema is source-bound b6-v16-root-physical-review; no approval receipt is supplied.

All38 candidate files, including native/core/control/mechanics/scorer/assets and embedded manifest, are byte-exact v15/v14. Embedded source hashes were revalidated without rewriting the unchanged manifest. No objective/variant, geometry, mass, friction, physical law, cap, threshold, scorer, controller or3034-control budget change. Only portable protocol validation, its tests, source binding and operator/version metadata change. Native History coercion remains untouched; malformed-record diagnostics are not physical task failures.

CPU tests exercise actual protocol.fresh, not only an isolated helper. A frozen first25-row excerpt from actual43967 passes strict validation and native History/status comparison, then intentionally stops at absent actions.json; this proves row acceptance only. Nineteen malformed row/trace variants each raise before the first native update. Tests include the exact wrist[string,bool], bool torque and integer held reproducer plus malformed velocity/captured/touch/shape/measurement cases. Synthetic source-bound result headers permit this diagnostic only; no complete run or review is fabricated. checks/fresh-boundary.json records20 tests and measured walltime. Existing strict JSON tests also pass.

Actual chosen runtime reference and model entries were checked in clean SDK-container CPU processes under persistent pre_sdk.py, python-I, no guard path injection. Pure YAM task construction is allowed before blocking isaaclab.app; no simulator/provider imports execute. Reference constructs11 objects; synthetic model construction is explicitly diagnostic with in-memory receipt replacement; actual missing receipt fails closed before SDK. Unchanged escape, replay/setup API, source/geometry/model proofs are retained from v15; no repeated SDK sweep or unchanged escape suite. Full consumed-source map preserves those original maps and adds changed-boundary/entry imports.

Physical evidence retained with exact scope: ROOT actual43967 PASS297controls/7425samples,230.362144947052s wall,47correction controls plus10 retention controls,19.6656mm lift. FIRST-LIFT+RETENTION ONLY; no full flip/handoff/seat/zoom/native20s hold or channel approval. Actual43897 positive13/13 is evaluator placement/load component evidence only, not genuine full-goal success. Failed43955 remains preserved. No old full score transfers to this bundle.

Unsubmitted GPU request: existing containers/unc/run.sh --gpu, GPU1 CPU4 RAM32GB30min, canonical chdir, process-only RTX driver override, OMP/BLAS threads1,1680s timeout plus20s kill grace. ROOT alone reviews/dispatches/promotes. Whole reference/replay estimated1100–2300s EACH, not measured; upper bound exceeds request timeout. Native budget3034 controls151.7sim seconds; conditional bound2423 controls121.15sim seconds includes native20s hold. Setup estimated30–120s. Exact commands and receipt schema are in OPERATOR.md.

Remaining only existing affected validation: fresh complete genuine reference/full native20s hold, fresh exact-action replay, ROOT channel review, then user-opened final official-runtime/model/media validation. No extra hardware/calibration/allvariant/FEM gate added. Brev stays OFF. No Slurm/GPU dispatch, simulator initialization, model/API, publication/messages, delegation or global state/ledger write occurred. No final-Brev-only claim until ROOT resolves the listed physical requirements.
'''
(O/'REPORT.md').write_text(report)
archive=O/'b6-v16.tar.gz'
with tarfile.open(archive,'w:gz') as t:
 for n in sorted(set(m['files'])|{'manifest.json','full-consumed-source-map.json','REPORT.md'}):t.add(O/n,arcname='b6-v16/'+n)
with tarfile.open(archive) as t:
 entries={n.name:hashlib.sha256(t.extractfile(n).read()).hexdigest() for n in t.getmembers() if n.isfile()}
for n,h in m['files'].items():assert entries['b6-v16/'+n]==h
write(O/'archive.json',dict(path=archive.name,sha256=sha(archive),bytes=archive.stat().st_size,files=len(entries),candidate_fingerprint=m['fingerprint'],source_members_verified=True))
r=dict(unit='B6_portable_strict_physics_numeric_boundary',status='implementation_complete_pending_ROOT_and_fresh_physical_validation',implementation_complete=True,candidate_root=str(O),candidate_fingerprint=m['fingerprint'],candidate_file_count=38,candidate_byte_exact_v15_v14=True,cpu_checks_pass=True,fresh_boundary_tests=20,malformed_variants_rejected_before_native_update=19,positive_actual_rows_accepted=25,physical_success=False,full_goal_verified=False,submission_ready=False,final_brev_only_remaining=False,actual43967='ROOT_PASS_FIRST_LIFT_PLUS10_RETENTION_ONLY_297controls_7425samples',actual43897='EVALUATOR_COMPONENT_13PASS_ONLY',public_channels_promoted=[],archive_sha256=sha(archive),archive_bytes=archive.stat().st_size,request=str(O/'GPU_REQUEST_UNSUBMITTED.sbatch'),request_submitted=False,simulator_initializations=0,Brev_operations=0,model_calls=0,native_budget_controls=3034,native_hold_s=20,estimated_whole_wall_s=[1100,2300],request_timeout_s=1680,remaining=['fresh whole genuine reference and native20s hold','fresh exact-action replay','ROOT individual channel review','user-opened final official runtime/model/media validation'],proofs={str(p.relative_to(O)):sha(p) for p in [O/'REPORT.md',O/'manifest.json',O/'archive.json',O/'full-consumed-source-map.json',O/'checks/fresh-boundary.json',O/'checks/protocol.json',O/'checks/v16-reference.json',O/'checks/v16-model-synthetic.json',O/'checks/v16-model-missing.json']})
write(O/'result.json',r)
D.mkdir(parents=True,exist_ok=True)
for n in ['REPORT.md','result.json','GPU_REQUEST_UNSUBMITTED.sbatch','OPERATOR.md','archive.json']:shutil.copy2(O/n,D/n)
write(D/'delivery-manifest.json',dict(candidate_root=str(O),candidate_fingerprint=m['fingerprint'],archive=str(archive),archive_sha256=sha(archive),files={str(O/n):sha(O/n) for n in ['REPORT.md','result.json','manifest.json','full-consumed-source-map.json','OPERATOR.md','GPU_REQUEST_UNSUBMITTED.sbatch']}))
print(json.dumps(dict(delivery=str(D),fingerprint=m['fingerprint'],archive_bytes=archive.stat().st_size,candidate_files_unchanged=len(native))))
