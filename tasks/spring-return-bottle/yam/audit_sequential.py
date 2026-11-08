"""Matched release-then-press comparison on the return spring.

No model calls; no object pose/joint writes or scoring changes. Source is saved
with the result so this negative comparison can be reproduced independently.
"""
import argparse,hashlib,json,os,shutil,sys
from pathlib import Path


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out',type=Path,required=True)
    args=ap.parse_args();out=args.out.resolve();out.mkdir(parents=True,exist_ok=False)
    root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'yam'))
    import spring_motion,sequential_reference
    spring_motion.plan=sequential_reference.plan
    for sub in ('yam','spring_bottle'):
        dest=out/'source'/sub;dest.mkdir(parents=True)
        for p in (root/sub).glob('*.py'):shutil.copy2(p,dest/p.name)
    manifest={str(p.relative_to(out/'source')):hashlib.sha256(p.read_bytes()).hexdigest()
        for p in (out/'source').rglob('*.py')}
    (out/'registration.json').write_text(json.dumps(dict(
        kind='negative matched sequential-release strategy; no model calls',
        policy='yam_scripted with same crown approach, release, body brace and press',seed=0,max_steps=500,
        source_sha256=manifest,expected='loop returns after release; terminal lid remains closed'),indent=2)+'\n')
    os.environ['BOTTLE_TRACE']=str(out/'measurements.jsonl')
    os.environ['BOTTLE_MODEL_REPORT']=str(out/'model.json')
    sys.argv=['inspect-robots','run','--embodiment','yam_arena','--policy','yam_scripted',
        '-E','task='+str(root/'yam/task.py'),'-E','object_jitter=0','-E','control_hz=20',
        '-E','cam_width=320','-E','cam_height=176','--instruction',
        'Open the bottle and keep it upright on the table.','--scorer','task_score',
        '--max-steps','500','--seed','0','--store-frames','--log-dir',str(out)]
    from inspect_robots.cli import main as cli_main
    cli_main()


if __name__=='__main__':main()
