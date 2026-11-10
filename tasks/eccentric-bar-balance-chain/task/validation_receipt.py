"""Fail closed: only source-bound reviewed actual channels enable runtime credit."""
from pathlib import Path
import hashlib,json
from functools import lru_cache
HERE=Path(__file__).resolve().parent
CHANNELS=frozenset({'contact_pairs','bar_pose_velocity','wrist_force','physics_clock','motor_limits','nonfinger_link_force','workspace_contact'})
def verify_receipt(root=HERE):
 try:
  r=json.loads((root/'actual-validation-receipt.json').read_text(),parse_constant=lambda x:(_ for _ in ()).throw(ValueError(x)))
  digest=r.pop('self_sha256')
  if hashlib.sha256(json.dumps(r,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()!=digest:return frozenset()
  if r['schema_version']!=1 or type(r['schema_version']) is not int:return frozenset()
  if set(r['proved_channels'])!=CHANNELS or r['new_public_score_claim'] is not False:return frozenset()
  for p,h in r['source_sha256'].items():
   path=root/p
   if Path(p).is_absolute() or '..' in Path(p).parts or hashlib.sha256(path.read_bytes()).hexdigest()!=h:return frozenset()
  from runtime_binding import verify_runtime
  verify_runtime()
  return CHANNELS
 except (OSError,ValueError,KeyError,TypeError,RuntimeError):return frozenset()
@lru_cache(maxsize=1)
def validated_channels():
 # Immutable copied launch source is verified before SDK bootstrap; cached for 500Hz samples.
 return verify_receipt()
