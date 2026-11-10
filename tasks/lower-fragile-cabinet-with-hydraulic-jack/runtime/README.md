# Private Hydraulic v5 portable inputs

ROOT review is pending. Start with OPERATOR.md. This package preserves the exact v4 controller, fixture, native embedded manifest and every native candidate file. Only new portable runtime/review/model wrappers were added. No physical full-goal, submission-ready or final-Brev-ready claim is made.

The authoritative v4 path contains `hydraulic-supported-yam-whole-v4`, unlike the abbreviated assigned path. ROOT cases.json pins that exact candidate. Preserved historical documents and original candidate draft metadata describe earlier mounts in places; OPERATOR.md and provenance/source-checks.json declare the selected mount. Do not submit these private provenance files.

Run `python -B -I verify_bundle.py` using the existing SDK CPU interpreter. Runtime inputs are frozen by manifest.json; original embedded sourcehashes.json remains byte-exact. Work outputs belong in fresh work directories. REPORT/result/archive and validation proof sidecars are excluded from the runtime manifest and bound by the final source archive SHA. No runtime sealing or source modification occurs.
