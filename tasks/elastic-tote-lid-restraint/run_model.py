"""Run the official Dream/Inspect entry; --preflight-only makes no model call."""
import argparse,hashlib,importlib.metadata,importlib.util,json,subprocess,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument("--out",type=Path,required=True);p.add_argument("--preflight-only",action="store_true");a=p.parse_args()
assert not a.out.resolve().is_relative_to(HERE) and not a.out.exists(),"Use fresh output outside source"
manifest=json.loads((HERE/"portable-input-manifest.json").read_text())
assert all(hashlib.sha256((HERE/n).read_bytes()).hexdigest()==h for n,h in manifest.items())
fields=json.loads((HERE/"submission-fields.json").read_text())
if (HERE/"task.json").exists():assert json.loads((HERE/"task.json").read_text())["instruction"]==fields["instruction"]
command=[sys.executable,"-B",str(HERE/"dream_continuity.py"),"--continuity-log",str(a.out/"continuity.jsonl"),
 "run","--embodiment","yam_arena","--policy","dream",
 "-P","model=dreamlabs/dream-1-beta","-P","rows_per_request=4",
 "-E","task="+str(HERE/"task.py"),"-E","device=cpu","-E","object_jitter=0",
 "-E","control_hz=20","-E","cam_width=320","-E","cam_height=176",
 "--instruction",fields["instruction"],"--scorer","task_score","--max-steps","4000",
 "--seed","42","--store-frames","--log-dir",str(a.out/"inspect")]
bindings=json.loads((HERE/"runtime-bindings.json").read_text())
expected_versions={n:v for n,v in bindings["UNC_development_runtime"]["versions"].items()
                   if n in ("isaacsim","inspect-robots","inspect-robots-yam","inspect-robots-dream")}
actual_versions={}
for name in expected_versions:
 try:actual_versions[name]=importlib.metadata.version(name)
 except importlib.metadata.PackageNotFoundError:actual_versions[name]=None
actual_modules={}
for module in bindings["UNC_development_runtime"]["yam_module_sha256"]:
 try:spec=importlib.util.find_spec(module)
 except ModuleNotFoundError:spec=None
 if spec and spec.origin:actual_modules[module]=hashlib.sha256(Path(spec.origin).read_bytes()).hexdigest()
runtime_matches=(actual_versions==expected_versions and actual_modules==bindings["UNC_development_runtime"]["yam_module_sha256"])
if a.preflight_only:
 print(json.dumps(dict(kind="source/argv preflight only, no model call",manifest_sha256=hashlib.sha256((HERE/"portable-input-manifest.json").read_bytes()).hexdigest(),actions=4000,control_hz=20,rows_per_request=4,seed=42,instruction=fields["instruction"],command=command,model_calls=0,runtime_matches=runtime_matches,installed_versions=actual_versions)))
else:
 assert runtime_matches,"Runtime differs from source-bound reference; review affected evidence before model calls"
 a.out.mkdir(parents=True,exist_ok=False)
 subprocess.run(command,check=True)
