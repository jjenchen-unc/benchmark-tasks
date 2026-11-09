"""Run the bundled genuine14D robot policy through the registered D3 runtime."""
from pathlib import Path
import argparse,hashlib,json,subprocess,sys
HERE=Path(__file__).resolve().parent
def main():
 p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--driver-check-override',action='store_true');p.add_argument('--preflight-only',action='store_true');a=p.parse_args()
 expected=json.loads((HERE/'runtime-manifest.json').read_text());provenance=json.loads((HERE/'demo-provenance.json').read_text())
 if expected!=provenance['verified_unc_runtime_manifest']:raise ValueError('Bundled runtime differs from verified original action source')
 if hashlib.sha256((HERE/'demo-actions.jsonl').read_bytes()).hexdigest()!=provenance['journal_sha256']:raise ValueError('Action policy changed')
 cmd=[sys.executable,'-B',str(HERE/'tools/run_final_replay.py'),'--task-source',str(HERE/'runtime'),'--journal',str(HERE/'demo-actions.jsonl'),'--journal-source',str(HERE/'runtime'),'--expected-source-manifest',str(HERE/'runtime-manifest.json'),'--out',str(a.out),'--width','640','--height','360']
 if a.driver_check_override:cmd+=['--driver-check-override']
 if a.preflight_only:cmd+=['--preflight-only']
 subprocess.run(cmd,check=True)
if __name__=='__main__':main()
