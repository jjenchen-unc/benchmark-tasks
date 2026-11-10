"""Execute byte-identical historical W1 rubric before SDK imports."""
import hashlib,importlib.util,sys
from pathlib import Path
p=Path(__file__).resolve().parent/'provenance/w1_scoring.py'
EXPECTED='fa7c18e40d49ab1d105a8ac4c35a3c71cb8542f7a2710935d82bede39cceb5ba'
if hashlib.sha256(p.read_bytes()).hexdigest()!=EXPECTED:raise RuntimeError('immutable source rubric mismatch')
if any(n in sys.modules for n in ('isaaclab','isaacsim')):raise RuntimeError('source rubric must execute before cold physics SDK imports')
spec=importlib.util.spec_from_file_location('b1_original_w1_scoring',p)
SOURCE=importlib.util.module_from_spec(spec);spec.loader.exec_module(SOURCE)
EXECUTED=True
