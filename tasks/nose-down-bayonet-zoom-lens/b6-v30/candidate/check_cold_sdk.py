"""Pure SDK registration only: never calls reset, _ensure_sim, step or AppLauncher."""
from pathlib import Path
import argparse,json,runpy,sys
sys.dont_write_bytecode=True
HERE=Path(__file__).resolve().parent
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--out',type=Path);a=p.parse_args()
 assert not {'yam_arena.scene','yam_arena.actions','yam_arena.inspect_embodiment'}&sys.modules.keys()
 runpy.run_path(str(HERE/'task.py'),run_name='b6_pure_cold_bootstrap')
 import kit
 from yam_arena.inspect_embodiment import yam_arena_embodiment
 emb=yam_arena_embodiment(task=str(HERE/'task.py'),device='cpu',control_hz=20,object_jitter=0,cam_width=320,cam_height=176)
 checks=dict(copied_source_exec_before_sdk=True,patched_embodiment=bool(getattr(type(emb),'_b6_patched',False)),no_sim_created=emb._base is None and emb._env is None,control20Hz=emb.info.control_hz==20.,raised_supported_bases=all(v[2]==.9 for v in emb.rig.bases.values()),all11_scene_objects=len(emb.task.objects)==11,only3_free_lenses=set(emb.task.free_objects)=={'lens_orange','lens_blue','lens_purple'})
 result=dict(state='pass' if all(checks.values()) else 'failed',checks=checks,bases=emb.rig.bases,task_sha256=emb.task.sha256,task_names=list(emb.task.names),kit_path=str(kit.__file__),physics_started=False,backend_proof=False);emb.close();text=json.dumps(result,indent=2);print(text)
 if a.out:a.out.write_text(text+'\n')
 if result['state']!='pass':raise SystemExit(2)
