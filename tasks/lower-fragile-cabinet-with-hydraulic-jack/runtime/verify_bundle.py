from pathlib import Path
import sys
O=Path(__file__).resolve().parent;sys.path.insert(0,str(O))
from protocol import check,load,sha
fingerprint=check();aliases=load(O/'provenance/portable-aliases.json')
for p,row in aliases['aliases'].items():
 if sha(O/row['portable'])!=row['sha256']:raise ValueError('alias changed '+p)
print({'fingerprint':fingerprint,'portable_historical_local_inputs':len(aliases['aliases']),'historical_mutable_doc_versions_unavailable':len(aliases['unavailable_historical_local_inputs']),'physical_proof':False})
