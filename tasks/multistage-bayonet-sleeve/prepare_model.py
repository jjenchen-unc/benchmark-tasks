"""Offline model preparation only. This helper makes no API calls."""
from pathlib import Path
import json
HERE=Path(__file__).resolve().parent
import sys
sys.path.insert(0,str(HERE))
from validation_receipt import verify_receipt
if not verify_receipt():raise RuntimeError('receipt/source mismatch')
print(json.dumps(dict(task=str(HERE/'task.py'),instruction=json.loads((HERE/'task.draft.json').read_text())['instruction'],device='cpu',object_jitter=0,control_hz=20,model_run_performed=False)))
