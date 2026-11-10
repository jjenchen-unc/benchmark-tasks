"""Pure mechanism/scoring/port counterexamples. Never runs physics."""
import ast,hashlib,json,math,sys
from pathlib import Path
import numpy as np
import source_rubric
from history import History,Mechanism,REQUIRED,VALIDATED_RUNTIME_CHANNELS
HERE=Path(__file__).resolve().parent

def run():
    checks={};f=json.loads((HERE/'fixture.json').read_text());S=source_rubric.SOURCE
    m=dict(nut_backoff_deg=65.,arm_open_deg=90.,latch_rounded=False,arm_free=True,contact_free=True)
    def history(measurement,n):
        h=History()
        for _ in range(n):h.update(.002,measurement,[0.,0.],{})
        return h
    checks['original_module_executes']=source_rubric.EXECUTED and hasattr(S,'score')
    checks['original_byte_identical']=hashlib.sha256((HERE/'provenance/w1_scoring.py').read_bytes()).hexdigest()==source_rubric.EXPECTED
    checks['no_native_or_physics_import']=not any(n in sys.modules for n in ['mujoco','isaaclab','isaacsim'])
    h=history(m,9999);checks['19p998_seconds_no_credit']=h.result()['score']==0.
    h.update(.002,m,[0.,0.],{});checks['20_seconds_full']=h.result()['score']==1.
    old_hold=sum([.002]*10000)
    checks['source_float_boundary_counterexample']=old_hold<20. and h.h['hold_full_s']==20.
    h=history(dict(m,arm_open_deg=0.),10001);checks['release_partial']=h.result()['score']==.5
    h.update(.002,dict(m,latch_rounded=True),[0.,0.],{});h.update(.002,m,[0.,0.],{})
    checks['rounding_is_sticky']=h.result()['score']==0. and h.h['rounded_any']
    h=history(m,10001);h.update(.002,dict(m,contact_free=False),[0.,0.],{})
    checks['one_contact_resets_both_holds']=h.h['hold_full_s']==h.h['hold_release_s']==0. and h.result()['score']==0.
    h=history(m,10001);h.update(.002,dict(m,nut_backoff_deg=59.9),[0.,0.],{})
    checks['retightening_revokes_credit']=h.result()['score']==0.
    h=history(m,10);h.update(.002,dict(m,nut_backoff_deg=float('nan')),[0.,0.],{})
    checks['nan_fails_closed']=h.h['invalid_measurement_seen'] and h.result()['score']==0.
    h=History()
    for _ in range(9):h.update(.002,m,[31.,0.],{})
    checks['30N_guard_not_early']=h.failure is None
    h.update(.002,m,[31.,0.],{});checks['30N_20ms_guard']=h.failure=='wrist_force_guard'
    h=History()
    for _ in range(50):h.update(.002,m,[81.,0.],{})
    checks['80N_100ms_abort_priority']=h.failure=='wrist_force_abort'
    h=History()
    for _ in range(10):h.update(.002,m,[0.,0.],{'link1':51.})
    checks['50N_20ms_link_guard']=h.failure=='nonfinger_link_guard'
    h=history(m,10001);h.update(.002,m,[0.,0.],{},True)
    checks['object_bounds_veto']=h.result()['score']==0. and h.failure=='object_out_of_bounds'
    h=History();h.update(.05,m,[0.,0.],{})
    checks['20Hz_samples_cannot_fake_500Hz_history']=h.h['invalid_measurement_seen']
    z=np.zeros(5);offset=np.zeros((5,2));mech=Mechanism(f['params'])
    mech.update(.002,math.radians(60),z,offset);checks['strict_source_release_boundary']=not mech.released
    mech.update(.002,math.radians(60.01),z,offset);checks['angle_releases_arm']=mech.released
    mech=Mechanism(f['params']);checks['breakaway_not_reduced']=math.isclose(mech.friction(0),.3524)
    checks['running_torque_preserved']=math.isclose(mech.friction(math.radians(2.01)),.03)
    mech=Mechanism(f['params']);z[1]=.8
    for _ in range(5):mech.update(.002,0,z,offset)
    checks['correct_wrench_does_not_round']=not mech.rounded
    mech=Mechanism(f['params']);z[:]=0;z[2]=.4;z[1]=-.4
    for _ in range(4):mech.update(.002,0,z,offset)
    checks['strip_not_before_five_ticks']=not mech.rounded
    mech.update(.002,0,z,offset)
    checks['opposite_tool_torque_cannot_mask_damage']=mech.rounded
    mech=Mechanism(f['params']);z[:]=0;z[0]=.06
    for _ in range(5):mech.update(.002,0,z,offset)
    checks['undersized_forcing_rounds']=mech.rounded
    mech=Mechanism(f['params']);offset[0]=[.02,0]
    for _ in range(6):mech.update(.002,0,z,offset)
    checks['distant_undersized_contact_not_forcing']=not mech.rounded
    R=np.asarray(f['rotation']);checks['proper_rigid_transform']=np.allclose(R.T@R,np.eye(3)) and np.isclose(np.linalg.det(R),1)
    checks['56_original_task_geometries']=sum(len(b['geoms']) for b in f['bodies'])==56
    checks['all_five_wrenches_free']=[b['name'] for b in f['bodies'] if b['free']]==['w12','w1_2','w13','w14','wadj']
    checks['static_does_not_activate_credit']=bool(REQUIRED) and not VALIDATED_RUNTIME_CHANNELS
    tree=ast.parse((HERE/'reference.py').read_text())
    calls=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)]
    checks['teacher_no_task_state_or_force_writes']=not any(n.func.attr.startswith(('write_','set_external','set_joint','add_forces')) for n in calls)
    checks['teacher_uses_genuine_step']=any(n.func.attr=='step' for n in calls)
    checks={k:bool(v) for k,v in checks.items()}
    return dict(kind='pure/static, no backend proof',checks=checks,passed=int(sum(checks.values())),total=len(checks),all_pass=bool(all(checks.values())))

if __name__=='__main__':
    result=run();print(json.dumps(result,indent=2));raise SystemExit(0 if result['all_pass'] else 1)
