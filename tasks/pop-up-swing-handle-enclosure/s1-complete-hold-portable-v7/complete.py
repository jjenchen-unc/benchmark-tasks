from pathlib import Path
O=Path(__file__).resolve().parent;B=O.parent/'b1-fullportable-v8-20261110-1440'
s=(B/'model_child.py').read_text().replace('B1_ROOT_REVIEW','S1_ROOT_REVIEW').replace('import kit','import s1_kit as kit').replace("from observer import emit,observe","from observer import observe").replace("emit(a.out/'observer.jsonl',observe(kit.ENGINE,'b1-wrench','model_reset'))","observer.emb=self\n emit(a.out/'observer.jsonl',observe(obs))").replace("emit(a.out/'pre-actions.jsonl',observe(kit.ENGINE,'b1-wrench','model_before_action',action.data))","pass # Native per-substep trace plus post-action raw8 observer below").replace("emit(a.out/'observer.jsonl',observe(kit.ENGINE,'b1-wrench','model_after_action',action.data))","emit(a.out/'observer.jsonl',observe(result.observation))").replace("kit.ENGINE.history.last","kit.ENGINE.last").replace('3870','4198').replace('B1-S','SH-T1').replace('b1.model_task_constructed','s1.model_task_constructed')
s=s.replace("os.environ['S1_ROOT_REVIEW']=str(Path(a.review).resolve())","os.environ['S1_ROOT_REVIEW']=str(Path(a.review).resolve())\nos.environ['S1_TARGET']='SH-T1'\nos.environ['S1_TRACE']=str(a.out.resolve()/'physics.jsonl')")
s=s.replace('reset_count=0',"import observer\ndef emit(path,row):\n with path.open('a') as f:f.write(dumps(row)+'\\n')\nreset_count=0")
(O/'model_child.py').write_text(s)
s=(B/'run_phase.py').read_text().replace('B1B2_LEGACY_RTX','S1_LEGACY_RTX')
s=s.replace("if a.phase=='model':\n from verify_model import verify\n verify(out)","if a.phase=='model':\n from verify_model import verify\n verify(out)")
(O/'run_phase.py').write_text(s)
s=(B/'verify_model.py').read_text();print(s[:1800])
