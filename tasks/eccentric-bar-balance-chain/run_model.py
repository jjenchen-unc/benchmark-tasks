"""Reproduce the actual bounded Dream1 evaluator route with continuity checks."""
from pathlib import Path
import argparse,hashlib,json,subprocess,sys
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
def main():
 ap=argparse.ArgumentParser();ap.add_argument("--out",required=True,type=Path);ap.add_argument("--preflight-only",action="store_true");a=ap.parse_args();a.out=a.out.resolve()
 expected=json.loads((HERE/"runtime-manifest.json").read_text());actual={str(p.relative_to(HERE/"task")):hashlib.sha256(p.read_bytes()).hexdigest() for p in (HERE/"task").rglob("*") if p.is_file() and "__pycache__" not in p.parts};assert actual==expected,"Runtime differs from verified input"
 from package_checks import preflight
 preflight(HERE)
 task=json.loads((HERE/"task.draft.json").read_text());assert task["instruction"]==json.loads((HERE/"task/task.draft.json").read_text())["instruction"]
 if a.preflight_only:
  print(json.dumps({"runtime_files":len(actual),"instruction_matches":True,"model_calls":0,"simulation_starts":0}));return
 if a.out==HERE or HERE in a.out.parents:raise ValueError("Use a new output directory outside the task")
 a.out.mkdir(parents=True,exist_ok=False)
 registration={"physical_seed":42,"setting":"S","goal":"stack_ABC","inspect_reset_seed":"derived by Inspect and passed unchanged to SDK", "policy":"dream","model":"dreamlabs/dream-1-beta","instruction":task["instruction"],"max_steps":1896,"control_hz":20,"seed":42,"rows_per_request":4,"object_jitter":0,"device":"cpu","camera_size":[320,176],"scorer":"task_score","runtime_sha256":expected}
 (a.out/"registration.json").write_text(json.dumps(registration,indent=2)+"\n")
 cmd=[sys.executable,"-B",str(HERE/"tools/dream_continuity.py"),"--continuity-log",str(a.out/"continuity.jsonl"),"run","--embodiment","yam_arena","--policy","dream","-P","model=dreamlabs/dream-1-beta","-P","rows_per_request=4","-E","task="+str(HERE/"task/task.py"),"-E","device=cpu","-E","object_jitter=0","-E","control_hz=20","-E","cam_width=320","-E","cam_height=176","--instruction",task["instruction"],"--scorer","task_score","--max-steps","1896","--seed","42","--store-frames","--log-dir",str(a.out/"model")]
 # Keep multiline policy text out of OS argv while preserving the exact
 # strings consumed by Inspect. This avoids Kit crashreporter cmdline parsing.
 forward_argv=cmd[5:]
 argv_json=a.out/'forward-argv.json'
 argv_json.write_text(json.dumps(forward_argv,ensure_ascii=False)+'\n')
 cmd=cmd[:5]+['--forward-argv-json',str(argv_json)]
 if any('\n' in value or '\r' in value for value in cmd):raise RuntimeError('OS argv must contain no newlines')
 subprocess.run(cmd,check=True)
 subprocess.run([sys.executable,"-B",str(HERE/"tools/verify_model.py"),str(a.out)],check=True)
if __name__=="__main__":main()
