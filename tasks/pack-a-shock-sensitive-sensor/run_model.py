"""Reproduce the actual bounded Dream1 evaluator route with continuity checks."""
from pathlib import Path
import argparse,hashlib,json,subprocess,sys
HERE=Path(__file__).resolve().parent
def main():
 ap=argparse.ArgumentParser();ap.add_argument("--out",required=True,type=Path);ap.add_argument("--preflight-only",action="store_true");a=ap.parse_args();a.out=a.out.resolve()
 expected=json.loads((HERE/"runtime-manifest.json").read_text());actual={str(p.relative_to(HERE/"runtime")):hashlib.sha256(p.read_bytes()).hexdigest() for p in (HERE/"runtime").rglob("*") if p.is_file() and "__pycache__" not in p.parts};assert actual==expected,"Runtime differs from verified input"
 task=json.loads((HERE/"task.json").read_text());assert task["instruction"]==json.loads((HERE/"runtime/task.draft.json").read_text())["instruction"]
 if a.preflight_only:
  print(json.dumps({"runtime_files":len(actual),"instruction_matches":True,"model_calls":0}));return
 if a.out==HERE or HERE in a.out.parents:raise ValueError("Use a new output directory outside the task")
 a.out.mkdir(parents=True,exist_ok=False)
 registration={"policy":"dream","model":"dreamlabs/dream-1-beta","instruction":task["instruction"],"max_steps":1604,"control_hz":20,"seed":42,"rows_per_request":4,"object_jitter":0,"runtime_sha256":expected}
 (a.out/"registration.json").write_text(json.dumps(registration,indent=2)+"\n")
 cmd=[sys.executable,"-B",str(HERE/"tools/dream_continuity.py"),"--continuity-log",str(a.out/"continuity.jsonl"),"run","--embodiment","yam_arena","--policy","dream","-P","model=dreamlabs/dream-1-beta","-P","rows_per_request=4","-E","task="+str(HERE/"runtime/task.py"),"-E","device=cuda:0","-E","object_jitter=0","-E","control_hz=20","-E","cam_width=320","-E","cam_height=176","--instruction",task["instruction"],"--scorer","task_score","--max-steps","1604","--seed","42","--store-frames","--log-dir",str(a.out/"model")]
 subprocess.run(cmd,check=True)
 subprocess.run([sys.executable,"-B",str(HERE/"tools/verify_model.py"),str(a.out)],check=True)
if __name__=="__main__":main()
