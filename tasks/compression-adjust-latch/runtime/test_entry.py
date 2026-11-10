"""Pure tests of the actual integration wrapper with a fake advancing SDK."""
import ast,dataclasses,importlib.util,json,math,sys,tempfile,types,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from step_guard import EpisodeGuard,EpisodeStopped
HERE=Path(__file__).resolve().parent
@dataclasses.dataclass
class Result:
    terminated:bool=False
    termination_reason:object=None
    info:dict=dataclasses.field(default_factory=dict)
class Entry(unittest.TestCase):
    def setup_sdk(self):
        events=[];status={'failure':None};Latch=type('Latch',(),{})
        class SDK:
            def __init__(s):
                s.task=types.SimpleNamespace(objects=[Latch()]);s.info=types.SimpleNamespace(control_hz=20);s.device='cpu';s.object_jitter=0;s.next_result=Result();s.asset=types.SimpleNamespace(joint_names=['pawl','lid_slide'],data=types.SimpleNamespace(joint_pos=types.SimpleNamespace(torch=np.zeros((1,2)))),write_joint_state_to_sim=lambda *a:None,set_joint_effort_target=lambda *a:None,set_joint_position_target=lambda *a:None,write_data_to_sim=lambda:None)
                s._base=types.SimpleNamespace(scene=Scene(s.asset))
            def _ensure_sim(s,x):events.append('ensure')
            def reset(s,*a,**k):events.append('reset');return 'observation'
            def _task_step(s):events.append('score');return s.next_result
            def step(s,action):events.append('physics');return s._task_step()
        class Scene(dict):
            def __init__(s,asset):super().__init__(compression_latch=asset,left_robot=types.SimpleNamespace(),right_robot=types.SimpleNamespace())
            def update(s,*a):pass
        class Observer:
            def __init__(s,emb):s.history=None
        tree=ast.parse((HERE/'kit.py').read_text());function=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='patch_embodiment')
        ns=dict(Latch=Latch,EpisodeGuard=EpisodeGuard,CONTROL_LIMIT=3848,math=math,FIX={'spec':{'grip':.001,'k':33416.5}},cam=lambda *a:(0,0),History=lambda:object(),current_status=lambda:status,replace=dataclasses.replace)
        exec(compile(ast.Module([function],[]),'actual_kit_patch','exec'),ns);ns['patch_embodiment'](types.SimpleNamespace(YamArenaEmbodiment=SDK))
        self.fake_imports=patch.dict(sys.modules,{'torch':types.SimpleNamespace(zeros_like=np.zeros_like),'observer':types.SimpleNamespace(Observer=Observer)});self.fake_imports.start();self.addCleanup(self.fake_imports.stop)
        s=SDK();s.reset(seed=42);events.clear();return s,events,status,ns
    def test_harness_derived_seed_does_not_change_fixed_fixture(self):
        s,e,_,_=self.setup_sdk()
        s.reset(seed=3867854786)
        self.assertEqual(e,['ensure','reset'])
        e.clear();s.step(None);self.assertEqual(e,['physics','score'])
    def test_nonzero_object_jitter_rejected_before_simulator(self):
        s,e,_,_=self.setup_sdk();s.object_jitter=.01
        with self.assertRaises(ValueError):s.reset(seed=3867854786)
        self.assertEqual(e,[])
    def test_termination_prevents_next_physics(self):
        s,e,_,_=self.setup_sdk();s.next_result=Result(True,'source_guard');r=s.step(None);self.assertTrue(r.terminated);self.assertEqual(e,['physics','score']);e.clear()
        with self.assertRaises(EpisodeStopped):s.step(None)
        self.assertEqual(e,[])
    def test_last_action_allowed_next_rejected(self):
        s,e,_,ns=self.setup_sdk();s._s4_episode_guard.actions=3847;ns['_STEPS']=3847;r=s.step(None);self.assertEqual(r.termination_reason,'time_limit');self.assertEqual(e,['physics','score']);e.clear()
        with self.assertRaises(EpisodeStopped):s.step(None)
        self.assertEqual(e,[])
    def test_known_source_failure_checked_before_sdk(self):
        s,e,status,_=self.setup_sdk();status['failure']='wrist_force_80N_100ms'
        with self.assertRaises(EpisodeStopped):s.step(None)
        self.assertEqual(e,[])
    def test_reset_clears_stop_and_count(self):
        s,e,_,_=self.setup_sdk();s.next_result=Result(True,'source_guard');s.step(None);s.next_result=Result();s.reset(seed=42);e.clear();s.step(None);self.assertEqual(e,['physics','score']);self.assertEqual(s._s4_episode_guard.actions,1)
    def test_failed_sdk_step_is_not_retryable(self):
        g=EpisodeGuard(3848);events=[]
        def fail():events.append('physics');raise ValueError('partial advance')
        with self.assertRaises(ValueError):g.invoke(fail)
        with self.assertRaises(EpisodeStopped):g.invoke(fail)
        self.assertEqual(events,['physics'])
    def test_non_s4_passthrough(self):
        s,e,_,_=self.setup_sdk();s.task=None;s.step(None);self.assertEqual(e,['physics','score'])
    def test_reset_failure_stays_stopped(self):
        s,e,_,_=self.setup_sdk();s._ensure_sim=lambda _:(_ for _ in ()).throw(ValueError('reset failure'))
        with self.assertRaises(ValueError):s.reset(seed=42)
        e.clear()
        with self.assertRaises(EpisodeStopped):s.step(None)
        self.assertEqual(e,[])
    def test_frozen_import_ignores_live_path(self):
        spec=importlib.util.spec_from_file_location('entry_driver',HERE/'driver.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
        self.assertEqual(Path(m.FrozenImports().find_spec('kit').origin),HERE/'kit.py')
        self.assertIsNone(m.FrozenImports().find_spec('yam_arena'))
    def test_optional_override_preserves_launcher_arguments(self):
        spec=importlib.util.spec_from_file_location('entry_driver_override',HERE/'driver.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);calls=[]
        class Launcher:
            def __init__(s,*a,**kw):calls.append(kw)
        original=Launcher.__init__
        with patch.dict(sys.modules,{'isaaclab.app':types.SimpleNamespace(AppLauncher=Launcher)}):
            m.install_override(False);self.assertIs(Launcher.__init__,original)
            m.install_override(True);Launcher(kit_args='--existing-option',headless=True)
        self.assertEqual(calls,[{'kit_args':'--existing-option --/rtx/verifyDriverVersion/enabled=false','headless':True}])
if __name__=='__main__':unittest.main(verbosity=2)
