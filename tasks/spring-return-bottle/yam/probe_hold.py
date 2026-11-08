"""Short robot-only holding gate. No model calls; the pressing hand stays clear."""
import argparse,json,os,shutil,sys
from dataclasses import replace
from pathlib import Path


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--out',type=Path,required=True)
    a=ap.parse_args();out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    root=Path(__file__).resolve().parents[1]
    import spring_motion
    original=spring_motion.plan
    def hold_only(top=.7475):
        plans=original(top);plans['left']=[replace(plans['left'][0],duration=30.)]
        return plans
    spring_motion.plan=hold_only
    for name in ('yam','spring_bottle'):
        dest=out/'source'/name;dest.mkdir(parents=True)
        for p in (root/name).glob('*.py'):shutil.copy2(p,dest/p.name)
    (out/'registration.json').write_text(json.dumps(dict(kind='holding-only diagnostic',max_steps=270,
        expected='loop stays at least60deg down for the final20ticks, bottle remains valid; no rocker press'),indent=2)+'\n')
    os.environ['BOTTLE_TRACE']=str(out/'measurements.jsonl')
    os.environ['BOTTLE_DIAGNOSTIC_ROBOT_STATE']='1'
    sys.argv=['inspect-robots','run','--embodiment','yam_arena','--policy','yam_scripted',
        '-E','task='+str(root/'yam/task.py'),'-E','object_jitter=0','-E','control_hz=20',
        '-E','cam_width=320','-E','cam_height=176','--instruction','Open the bottle and keep it upright on the table.',
        '--scorer','task_score','--max-steps','270','--seed','0','--store-frames','--log-dir',str(out)]
    from inspect_robots.cli import main as cli_main
    cli_main()


if __name__=='__main__':main()
