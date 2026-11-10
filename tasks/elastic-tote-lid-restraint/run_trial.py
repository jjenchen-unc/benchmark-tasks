"""Disclosed state-aware full-goal robot trial through public14D actions; no model evidence."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
parser = argparse.ArgumentParser()
parser.add_argument("--out", type=Path, required=True)
args = parser.parse_args()
assert not args.out.resolve().is_relative_to(HERE), "Choose fresh output outside tasksource"
args.out.mkdir(parents=True, exist_ok=False)
src = args.out / "source"
core_manifest=json.loads((HERE/"core-source-manifest.json").read_text())
src.mkdir()
for relative,digest in core_manifest.items():
    original=HERE/relative
    assert original.resolve().is_relative_to(HERE) and hashlib.sha256(original.read_bytes()).hexdigest()==digest
    (src/relative).parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(original,src/relative)
(args.out / "source-sha256.json").write_text(json.dumps(
    {str(p.relative_to(src)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(src.rglob("*")) if p.is_file()},
    indent=2))

_pref=json.loads((Path(__file__).resolve().parent/"verified-prefix-preflight.json").read_text())
assert _pref["status"]=="checked"
_portable_blob=(HERE/"portable-input-manifest.json").read_bytes()
_portable_manifest=json.loads(_portable_blob)
assert all(hashlib.sha256((HERE/n).read_bytes()).hexdigest()==h for n,h in _portable_manifest.items())
(args.out/"portable-input-manifest.json").write_bytes(_portable_blob)
assert _pref["source24"]==json.loads((args.out/"source-sha256.json").read_text())
assert all(hashlib.sha256((Path(__file__).resolve().parent/n).read_bytes()).hexdigest()==h for n,h in _pref["helper_sha256"].items())

import os  # noqa: E402
if os.environ.get("D1R_DRIVER_CHECK_OVERRIDE") == "1":
    # Same process-only override as containers/unc/inspect_entry.py; host driver unchanged.
    import functools
    from isaaclab.app import AppLauncher
    _orig = AppLauncher.__init__

    @functools.wraps(_orig)
    def _init(self, *a, **k):
        k["kit_args"] = (k.get("kit_args", "") + " --/rtx/verifyDriverVersion/enabled=false").strip()
        return _orig(self, *a, **k)
    AppLauncher.__init__ = _init
    print("UNC DEVELOPMENT ONLY: RTX driver-version check disabled for this process.", flush=True)
os.environ["D1R_TRACE"] = str(args.out / "measurements.jsonl")
sys.path.insert(0, str(HERE / "d1r_arena"))

from inspect_robots import Scene  # noqa: E402
from inspect_robots.types import Action  # noqa: E402
from PIL import Image  # noqa: E402
from yam_arena.inspect_embodiment import yam_arena_embodiment, _urdf_arm_limits  # noqa: E402
from yam_arena.constants import READY, ARM_JOINTS, FINGER_JOINTS, FINGER_OPEN  # noqa: E402

INSTR = json.loads((HERE/"submission-fields.json").read_text())["instruction"]
emb = yam_arena_embodiment(task=str(HERE / "task.py"), object_jitter=0, control_hz=20, cam_width=320, cam_height=176)
lo, hi = _urdf_arm_limits()
t0 = time.monotonic()
report = dict(kind="robot-actuated full-goal development trial; no model/publication claim",
              model_calls=0, checks={})
frames = {}
rows = []


from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parent))
from robot_motion import ArmMotion
from joint_target_force import position_target_for_force
from yam_arena.runtime import link_poses,tcp_world
from yam_arena import YAM_URDF
from yam_arena.urdf import quat_xyzw_to_matrix, forward_kinematics
from yam_arena.constants import DOWN_XYZW
records=[]
images={}
recorder=None
steps=0
motion_debug={}
journal=open(args.out/"robot-actions.jsonl","w",buffering=1)
try:
    obs=emb.reset(Scene(id="d1r-riser-robot-pickup",instruction=INSTR),seed=42)
    import kit
    base=emb._base
    from media_recorder import Recorder
    recorder=Recorder(emb,args.out,kit)
    movers={side:ArmMotion(base.scene[side+"_robot"]) for side in ("left","right")}
    from pxr import Usd,UsdGeom,UsdPhysics
    import omni.usd
    stage=omni.usd.get_context().get_stage()
    all_prims=list(Usd.PrimRange.Stage(stage,Usd.TraverseInstanceProxies()))
    report["riser_prim_details"]=[dict(path=str(p.GetPath()),type=p.GetTypeName(),
        apis=list(p.GetAppliedSchemas())) for p in all_prims if "riser" in str(p.GetPath()).lower()]
    cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),["default","render","proxy"])
    hardware={}
    for name,expected in (("mount_riser_left",(.16,.16,.20)),("mount_riser_right",(.16,.16,.20)),("mount_riser_camera",(.11,.11,.20))):
        matches=[p for p in all_prims if p.GetName()==name]
        entries=[]
        for p in matches:
            descendants=list(Usd.PrimRange(p,Usd.TraverseInstanceProxies()))
            colliders=[c for c in descendants if c.HasAPI(UsdPhysics.CollisionAPI)
                and UsdPhysics.CollisionAPI(c).GetCollisionEnabledAttr().Get() is not False]
            entries.append(dict(path=str(p.GetPath()),colliders=[str(c.GetPath()) for c in colliders],
                size=list(cache.ComputeWorldBound(p).ComputeAlignedBox().GetSize())))
        hardware[name]=entries
    report["riser_hardware"]=hardware
    assert all(any(e["colliders"] and np.allclose(e["size"],expected,atol=.001)
        for e in hardware[name]) for name,expected in (
        ("mount_riser_left",(.16,.16,.20)),("mount_riser_right",(.16,.16,.20)),("mount_riser_camera",(.11,.11,.20)))), "Unverified riser collision geometry"
    report["gripper_effort_cfg"]={side:float(movers[side].robot.cfg.actuators["gripper"].joint_effort_limit) for side in movers}
    assert all(v==22. for v in report["gripper_effort_cfg"].values())
    targets=np.asarray(obs.state["joint_pos"],float).copy()
    planned=json.loads((Path(__file__).resolve().parent/"ik-pickup.json").read_text())
    planned={v["side"]:v for v in planned}
    self_audit=json.loads((Path(__file__).resolve().parent/"bar-axis-near16-self-contact-audit.json").read_text())
    assert all(x["self_separation_m"]>.001 for x in self_audit["planned"])
    assert self_audit["source_plan_sha256"]==hashlib.sha256((Path(__file__).resolve().parent/"bar-axis-mouth-path-shallow15.json").read_bytes()).hexdigest()
    report["planned_nonadjacent_hand_clearance_m"]=min(x["self_separation_m"] for x in self_audit["planned"])
    proposal=json.loads((Path(__file__).resolve().parent/"bar-axis-mouth-path-shallow15.json").read_text())
    held_audit=json.loads((Path(__file__).resolve().parent/"bar-axis-shallow15-held-audit.json").read_text())
    assert not held_audit["issues"],held_audit["issues"]
    assert not held_audit["cord_centreline_hull_hits"],held_audit["cord_centreline_hull_hits"]
    assert held_audit["source_plan_sha256"]==hashlib.sha256((Path(__file__).resolve().parent/"bar-axis-mouth-path-shallow15.json").read_bytes()).hexdigest()
    report["nominal_held_audit"]=held_audit
    viable=[x for x in proposal["candidates"] if not x["issues"]]
    if not viable:raise RuntimeError("No clear sampled CPU mouth path")
    chosen=min(viable,key=lambda x:abs(x["angle"]-45.))
    pickup=next(x for x in json.loads((Path(__file__).resolve().parent/"bar-axis-grasp-narrow-candidates.json").read_text())["candidates"] if x["pickup_inward_angle_deg"]==chosen["angle"])
    planned["left"]["stages"]=pickup["stages"]
    report["left_grasp_proposal"]=dict(inward_angle_deg=chosen["angle"],jaw_axis_world=[-1,0,0],pickup_jaw_gap_mm=51.645,bar_endpoint_offset_mm=17.,bar_end="near",contact_offset_mm=16.,tcp_height_above_hook_mm=20.,hook_pitch_deg=15.,sampled_path_min_clearance_m=chosen["minclearance_m"])
    downs={side:np.asarray(planned[side]["stages"][0]["R"]) for side in movers}
    def tick(phase):
        global obs,steps
        res=emb.step(Action(data=targets.copy()))
        obs=res.observation;steps+=1
        row=dict(kit._LAST_CONTROL_ROW)
        row.update(phase=phase,robot_action=targets.tolist(),score=res.info.get("d1r_score"),
                   actual_joint_pos=np.asarray(obs.state["joint_pos"],float).tolist(),motion=dict(motion_debug))
        row["robot_raw_joint_pos"]={}
        for side in movers:
            robot=movers[side].robot
            ids=robot.find_joints(ARM_JOINTS+FINGER_JOINTS,preserve_order=True)[0]
            raw=np.asarray(robot.data.joint_pos.torch[0,ids].detach().cpu().numpy(),dtype=np.float64)
            row["robot_raw_joint_pos"][side]=raw.tolist()
            public_mean=float(np.clip(raw[6:].mean()/FINGER_OPEN,0,1))
            assert abs(public_mean-row["actual_joint_pos"][6 if side=="left" else 13])<1e-5
        records.append(row)
        journal.write(json.dumps(row,default=float)+"\n")
        recorder.write(obs)
        if res.terminated:raise RuntimeError("Task terminated: "+str(res.termination_reason))
        if steps>=4000:raise RuntimeError("Declared robot action budget exhausted")
    def go(goals,phase,limit=300):
        label={"pre_above":"pre_above","approach_above":"above","descend_to_handles":"grasp","lift_pair":"lift"}[phase]
        seeds={side:np.array(next(x["q"] for x in planned[side]["stages"] if x["stage"]==label)) for side in goals}
        integral={side:np.zeros(3) for side in goals}
        references={side:tcp_world(movers[side].robot).copy() for side in goals}
        if phase not in ("approach_above","pre_above"):
            seeds={side:np.array(obs.state["joint_pos"][(0 if side=="left" else 7):][:6]) for side in goals}
        if phase=="approach_above" and "left" in goals:
            seeds["left"]=np.asarray(obs.state["joint_pos"],float)[:6].copy()
        for _ in range(limit):
            reached=True
            for side,goal in goals.items():
                robot=movers[side].robot
                p=tcp_world(robot);R=link_poses(robot)["gripper"][:3,:3]
                error=np.asarray(goal)-p
                rv=Rotation.from_matrix(downs[side]@R.T).as_rotvec()
                reached &= np.linalg.norm(error)<.003 and np.linalg.norm(rv)<.06
                if np.linalg.norm(error)<.02 and np.linalg.norm(rv)<.2:
                    integral[side]=np.clip(integral[side]+.10*error,-.02,.02)
                if phase=="pre_above" or (phase=="approach_above" and side=="right"):
                    reference=np.asarray(goal)
                else:
                    delta=np.asarray(goal)-references[side]
                    references[side]+=delta*min(1.,.002/max(np.linalg.norm(delta),1e-12))
                    reference=references[side]
                command=movers[side].solve(reference+integral[side],downs[side],seed=seeds[side],command_ref=(targets[:6].copy() if side=="left" and phase!="pre_above" else None))
                if phase not in ("approach_above","pre_above") or (phase=="approach_above" and side=="left"):
                    seeds[side]=movers[side].last_q.copy()
                offset=0 if side=="left" else 7
                targets[offset:offset+6]=command
                motion_debug[side]=dict(tcp=p.tolist(),goal=np.asarray(goal).tolist(),error_m=float(np.linalg.norm(error)),
                    rotation_error_rad=float(np.linalg.norm(rv)),integral=integral[side].tolist())
            tick(phase)
            if reached:return
        raise RuntimeError("Motion did not converge: "+phase+" "+str(motion_debug))
    initial={side:np.array(kit._hook_pose(base,"strap_4_"+end)[0])
             for side,end in (("left","a"),("right","b"))}
    # Every stage acts through 14-D joint targets. No diagnostic placements/loads.
    targets[6]=.55
    for _ in range(12):tick("narrow_left_in_air")
    go({"left":np.array(pickup["stages"][0]["tcp"])},"pre_above")
    go({side:p+[0,0,.12] for side,p in initial.items()},"approach_above")
    go({side:p+[0,0,(.020 if side=="left" else .010)] for side,p in initial.items()},"descend_to_handles")
    targets[6]=0.
    targets[13]=.18  # Position target retains a restoring jaw-centering gradient.
    for _ in range(25):tick("close_fingers")
    go({side:p+[0,0,.10] for side,p in initial.items()},"lift_pair")
    for _ in range(20):tick("hold_pair")
    def compensate(side,end,nominal,previous):
        import strap
        h={e:kit._hook_pose(base,"strap_4_"+e) for e in "ab"}
        attach={e:strap.hook_attach_world(*h[e]) for e in "ab"}
        gap=kit._LAST_CONTROL_ROW["totes"]["tote_tall"]["lid_gap_m"]
        points=kit._CORDS[3].path(attach["a"],attach["b"],gap)  # read-only geometry
        T=kit._LAST_CONTROL_ROW["straps"]["strap_4"]["tension_n"]
        direction=points[1]-points[0] if end=="a" else points[-2]-points[-1]
        force=T*direction/max(np.linalg.norm(direction),1e-12)
        hp,quat=h[end];Rhook=strap.quat_to_R(quat)
        com=hp+Rhook@strap.COM_LOCAL
        return movers[side].payload_compensated_target(nominal,force,attach[end],com,previous)

    carry_plan=json.loads((Path(__file__).resolve().parent/"carry-seat-plan.json").read_text())
    lost={"left":0,"right":0}
    carry_starts={side:kit._hook_pose(base,"strap_4_"+end)[0].copy() for side,end in (("left","a"),("right","b"))}
    near_goals={side:np.array(next(x["arms"][side]["hook_target"] for x in reversed(carry_plan["carry_records"]) if x["phase"]=="carry_near_high")) for side in movers}
    for waypoint in carry_plan["carry_records"]:
        if waypoint["phase"]=="carry_near_high":
            for side in movers:
                waypoint["arms"][side]["hook_target"]=(carry_starts[side]+waypoint["t"]*(near_goals[side]-carry_starts[side])).tolist()
        for attempt in range(8):
            for side,end in (("left","a"),("right","b")):
                robot=movers[side].robot;p=tcp_world(robot);R=link_poses(robot)["gripper"][:3,:3]
                hp=kit._hook_pose(base,"strap_4_"+end)[0]
                offset=R.T@(hp-p)  # measured held-object transform; no object writes
                goal=np.array(waypoint["arms"][side]["hook_target"])
                if side=="left" and waypoint["phase"]=="carry_above_tote":
                    goal=np.array([.28,.16,1.102797408448131])+waypoint["t"]*(np.array([.35,.26,1.102797408448131])-np.array([.28,.16,1.102797408448131]))
                if side=="left":
                    seed=np.asarray(obs.state["joint_pos"],float)[:6]
                    command=movers[side].solve(goal-downs[side]@offset,downs[side],seed=seed,command_ref=targets[:6].copy())
                else:
                    command=movers[side].solve_hook(goal,offset,np.array(waypoint["arms"][side]["q"]),command_ref=targets[7:13].copy())
                start=0 if side=="left" else 7
                targets[start:start+6]=compensate(side,end,movers[side].last_q,targets[start:start+6].copy())
                motion_debug[side]=dict(hook=hp.tolist(),hook_goal=goal.tolist(),
                    hook_error_m=float(np.linalg.norm(hp-goal)),waypoint_t=waypoint["t"],
                    payload=movers[side].payload_compensation)
            tick(waypoint["phase"])
            reached=True
            for side,end in (("left","a"),("right","b")):
                contact=kit._LAST_CONTROL_ROW["robot_contacts"]["strap_4_"+end]
                lost[side]=0 if contact else lost[side]+1
                if lost[side]>3:raise RuntimeError("Lost real hook contact during carry: "+side)
                goal=np.array(waypoint["arms"][side]["hook_target"])
                if side=="left" and waypoint["phase"]=="carry_above_tote":
                    goal=np.array([.28,.16,1.102797408448131])+waypoint["t"]*(np.array([.35,.26,1.102797408448131])-np.array([.28,.16,1.102797408448131]))
                hp=kit._hook_pose(base,"strap_4_"+end)[0]
                reached &= np.linalg.norm(hp-goal)<.008
            if reached:break
        else:
            raise RuntimeError("Payload waypoint tracking failed: "+str(motion_debug))
    for _ in range(20):tick("hold_over_tote")
    report["carry_finished"]=True
    report["carry_final_hooks"]={e:kit._hook_pose(base,"strap_4_"+e)[0].tolist() for e in "ab"}
    path={"records":[x for x in chosen["records"] if x["stage"] in ("pitch_hook","outside_lower_clearance","advance_above_edge","outside_lower","mouth_above","mouth_entry","mouth_insert")]}
    assert path["records"]
    b_goal=kit._hook_pose(base,"strap_4_b")[0].copy()
    b_seed=np.asarray(obs.state["joint_pos"],float)[7:13].copy()
    right_start_robot=movers["right"].robot
    b_grasp_reference_offset=link_poses(right_start_robot)["gripper"][:3,:3].T@(b_goal-tcp_world(right_start_robot))

    def hold_b():
        robot=movers["right"].robot;p=tcp_world(robot);R=link_poses(robot)["gripper"][:3,:3]
        hp=kit._hook_pose(base,"strap_4_b")[0]
        offset=R.T@(hp-p)
        b_grasp_drift=float(np.linalg.norm(offset-b_grasp_reference_offset))
        if b_grasp_drift>.025:raise RuntimeError("Right hook translated25mm within grip: "+str(b_grasp_drift))
        movers["right"].solve_hook(b_goal,offset,b_seed)
        targets[7:13]=compensate("right","b",movers["right"].last_q,targets[7:13].copy())
        motion_debug["right"]=dict(hook=hp.tolist(),hook_goal=b_goal.tolist(),
            hook_error_m=float(np.linalg.norm(hp-b_goal)),mode="hold_loaded_other_end",grasp_translation_drift_m=b_grasp_drift,
            payload=movers["right"].payload_compensation)
    left_feedback=np.zeros(3)
    for waypoint in path["records"]:
        reference=np.array(waypoint["q"])
        desired_hook=np.array(waypoint["hook_pos"])
        desired_R=np.array(waypoint["R"])
        for attempt in range(10):
            robot=movers["left"].robot;R=link_poses(robot)["gripper"][:3,:3]
            p_tcp=tcp_world(robot)
            hp=kit._hook_pose(base,"strap_4_a")[0]
            measured_offset=R.T@(hp-p_tcp)
            if attempt>0:
                left_feedback=np.clip(.90*left_feedback+.20*(desired_hook-hp),-.003,.003)
            tcp_target=desired_hook-desired_R@measured_offset+left_feedback
            movers["left"].solve(tcp_target,desired_R,seed=reference)
            targets[:6]=compensate("left","a",movers["left"].last_q,targets[:6].copy())
            hold_b()
            motion_debug["left"]=dict(hook=hp.tolist(),hook_goal=desired_hook.tolist(),
                hook_error_m=float(np.linalg.norm(hp-desired_hook)),stage=waypoint["stage"],feedback_m=left_feedback.tolist(),
                payload=movers["left"].payload_compensation,
                rotation_error_rad=float(np.linalg.norm(Rotation.from_matrix(desired_R@R.T).as_rotvec())))
            tick(waypoint["stage"])
            actual_R=link_poses(robot)["gripper"][:3,:3]
            actual_hp=kit._hook_pose(base,"strap_4_a")[0]
            if (np.linalg.norm(actual_hp-desired_hook)<.006 and
                np.linalg.norm(Rotation.from_matrix(desired_R@actual_R.T).as_rotvec())<.08):
                break
        else:raise RuntimeError("First-hook path tracking failed: "+str(motion_debug["left"]))
    # Settle the same commanded final pose through genuine robot actions.
    for _ in range(20):
        robot=movers["left"].robot;R=link_poses(robot)["gripper"][:3,:3]
        hp=kit._hook_pose(base,"strap_4_a")[0]
        offset=R.T@(hp-tcp_world(robot))
        movers["left"].solve(desired_hook-desired_R@offset+left_feedback,desired_R,seed=reference)
        targets[:6]=compensate("left","a",movers["left"].last_q,targets[:6].copy())
        hold_b();tick("mouth_seat_settle")
    # Real contact handoff: servo a measured hook throat point to the finite bar.
    # The slight nominal press is resolved by normal PhysX contacts, under unchanged caps.
    import strap
    frame=strap.ToteFrame();axis=frame.bar_axis_world;eye=frame.world(frame.eyes[0])
    throat_local=np.array([.048,0.,-.0065])
    throat_goal=eye-.016*axis
    robot=movers["left"].robot
    hand_R=link_poses(robot)["gripper"][:3,:3].copy()
    hp,hq=kit._hook_pose(base,"strap_4_a")
    start_point=hp+strap.quat_to_R(hq)@throat_local
    throat_integral=np.zeros(3);reference=np.asarray(obs.state["joint_pos"],float)[:6].copy();stable=0
    for k in range(1,121):
        robot=movers["left"].robot;R=link_poses(robot)["gripper"][:3,:3]
        tcp=tcp_world(robot);hp,hq=kit._hook_pose(base,"strap_4_a");Rh=strap.quat_to_R(hq)
        point=hp+Rh@throat_local;point_reference=start_point+min(1.,k/20)*(throat_goal-start_point)
        throat_integral=np.clip(throat_integral+.08*(point_reference-point),-.006,.006)
        grasp_offset=R.T@(hp-tcp)
        hook_goal=point_reference-Rh@throat_local
        movers["left"].solve(hook_goal-hand_R@grasp_offset+throat_integral,hand_R,seed=reference)
        reference=movers["left"].last_q.copy()
        targets[:6]=compensate("left","a",reference,targets[:6].copy())
        hold_b()
        along=float(np.clip((point-eye)@axis,-.013,.013));bar_in_hook=Rh.T@(eye+along*axis-hp)
        motion_debug["left"]=dict(stage="measured_throat_contact",throat=point.tolist(),throat_goal=throat_goal.tolist(),
            throat_error_m=float(np.linalg.norm(point-throat_goal)),throat_x_error_m=float(abs(point[0]-throat_goal[0])),
            integral_m=throat_integral.tolist(),bar_in_hook=bar_in_hook.tolist(),
            bar_alignment=float(abs(Rh[:,1]@axis)),payload=movers["left"].payload_compensation)
        tick("measured_throat_contact")
        hp,hq=kit._hook_pose(base,"strap_4_a");Rh=strap.quat_to_R(hq);point=hp+Rh@throat_local
        good=(k>=20 and np.linalg.norm(point-throat_goal)<.0015 and abs(point[0]-throat_goal[0])<.0007
              and abs(Rh[:,1]@axis)>.95 and kit._LAST_CONTROL_ROW["straps"]["strap_4"]["seated_a"]==0)
        stable=stable+1 if good else 0
        if stable>=5:break
    else:raise RuntimeError("Measured throat contact did not converge: "+str(motion_debug["left"]))
    report["throat_contact_controls"]=k
    report["throat_contact_point_error_m"]=float(np.linalg.norm(point-throat_goal))
    report["held_seat_state"]=dict(kit._LAST_CONTROL_ROW["straps"]["strap_4"])
    if report["held_seat_state"]["seated_a"]!=0:
        raise RuntimeError("HookA did not reach actual eye seat while held")
    # Only robot gripper commands release the hook. Never edit its pose or score.
    loaded_command=targets[:6].copy()
    unloaded_command=np.asarray(obs.state["joint_pos"],float)[:6].copy()
    for k in range(1,31):
        fraction=k/30
        desired=(1-fraction)*loaded_command+fraction*unloaded_command
        targets[:6]=np.clip(targets[:6]+np.clip(desired-targets[:6],-.04,.04),lo,hi)
        targets[6]=fraction
        hold_b();tick("open_left_gradual")
    robot=movers["left"].robot
    retreat0=tcp_world(robot).copy();retreat_R=link_poses(robot)["gripper"][:3,:3].copy()
    retreat_seed=np.asarray(obs.state["joint_pos"],float)[:6].copy()
    for k in range(1,41):
        target=retreat0+np.array([0.,.060,.040])*k/40
        targets[:6]=movers["left"].solve(target,retreat_R,seed=retreat_seed,command_ref=targets[:6].copy())
        retreat_seed=movers["left"].last_q.copy()
        hold_b();tick("left_retreat")
    retained=[]
    for _ in range(30):
        hold_b();tick("first_hook_hands_off")
        row=kit._LAST_CONTROL_ROW
        retained.append(row["straps"]["strap_4"]["seated_a"]==0 and
            not row["robot_contacts"]["strap_4_a"])
    report["first_hook_retention_fraction"]=float(np.mean(retained))
    report["first_hook_retained"]=all(retained[-20:])
    report["first_hook_final"]=dict(kit._LAST_CONTROL_ROW["straps"]["strap_4"])
    if not report["first_hook_retained"]:raise RuntimeError("HookA did not remain seated after real release")
    # Continue the exact successful first-hook prefix with genuine robot contact on the lid.
    closure=json.loads((Path(__file__).resolve().parent/"retained-closure-plan-43447.json").read_text())
    press=closure["press"]
    if press["issues"]:raise RuntimeError("Uncleared CPU press path: "+str(press["issues"][:3]))
    contact_from_tcp=np.array(press["contact_from_tcp"]);press_R=np.array(press["R"])
    contact_xy=np.array(press["contact_xy"])
    targets[6]=0.
    for _ in range(20):hold_b();tick("close_empty_left_for_press")
    for waypoint in press["records"]:
        if waypoint["stage"]=="press_closed":continue # Closing is produced by actual contact force, never geometry writes.
        desired_tcp=np.array(waypoint["tcp"]);desired_R=np.array(waypoint["R"]);seed=np.array(waypoint["q"])
        for attempt in range(15):
            robot=movers["left"].robot;p=tcp_world(robot);R=link_poses(robot)["gripper"][:3,:3]
            targets[:6]=movers["left"].solve(desired_tcp,desired_R,seed=seed,command_ref=targets[:6].copy())
            hold_b();motion_debug["left"]=dict(stage=waypoint["stage"],tcp=p.tolist(),tcp_goal=desired_tcp.tolist(),
                tcp_error_m=float(np.linalg.norm(p-desired_tcp)),rotation_error_rad=float(np.linalg.norm(Rotation.from_matrix(desired_R@R.T).as_rotvec())))
            tick(waypoint["stage"])
            Rnow=link_poses(robot)["gripper"][:3,:3]
            if np.linalg.norm(tcp_world(robot)-desired_tcp)<.004 and np.linalg.norm(Rotation.from_matrix(desired_R@Rnow.T).as_rotvec())<.06:break
        else:raise RuntimeError("Press approach did not track: "+str(motion_debug["left"]))
    press_seed=np.asarray(obs.state["joint_pos"],float)[:6].copy()
    def hold_left_press(requested_downward_N):
        global press_seed
        robot=movers["left"].robot
        gap=kit._LAST_CONTROL_ROW["totes"]["tote_tall"]["lid_gap_m"]
        world_top=frame.o[2]+frame.box(gap)[2]
        contact_goal=np.r_[contact_xy,world_top]
        movers["left"].solve(contact_goal-press_R@contact_from_tcp,press_R,seed=press_seed)
        press_seed=movers["left"].last_q.copy()
        actual_q=np.asarray(obs.state["joint_pos"],dtype=np.float64)[:6]
        target,detail=position_target_for_force(movers["left"].fk,actual_q,contact_from_tcp,
            link_poses(robot)["base"][:3,:3],press_seed,np.array([0.,0.,-requested_downward_N]),
            np.array([80.,80.,80.,10.,10.,10.]),movers["left"].lo,movers["left"].hi,targets[:6].copy())
        targets[:6]=target
        motion_debug["left"]=dict(stage="bounded_robot_lid_press",contact_goal=contact_goal.tolist(),lid_gap_m=gap,press=detail)
    pressed_stable=0;press_contacts=0
    for k in range(1,181):
        requested=25.*min(1.,k/50.)
        hold_left_press(requested);hold_b();tick("bounded_robot_lid_press")
        row=kit._LAST_CONTROL_ROW;gap=row["totes"]["tote_tall"]["lid_gap_m"]
        press_contacts+=int(not row["hands_off_lid"])
        pressed_stable=pressed_stable+1 if gap<.002 and not row["hands_off_lid"] else 0
        if pressed_stable>=20:break
    else:raise RuntimeError("Lid did not close under bounded robot joint targets: "+str(motion_debug["left"]))
    report["lid_press"]=dict(closed=True,controls=k,actual_gap_m=gap,robot_contact_controls=press_contacts,
        requested_max_downward_N=25.,method="Position-target Jacobian bias; no effort/object/diagnostic force API")
    # Keep the public observation/action contract unchanged; export private raw telemetry.
    snapshot=dict(row=records[-1],raw_joint_names=ARM_JOINTS+FINGER_JOINTS,
        bases={side:link_poses(movers[side].robot)["base"].tolist() for side in movers},
        links={side:{n:T.tolist() for n,T in link_poses(movers[side].robot).items()} for side in movers},
        task_source_sha256=json.loads((args.out/"source-sha256.json").read_text()))
    calibration={}
    for side in movers:
        joints=dict(zip(ARM_JOINTS+FINGER_JOINTS,snapshot["row"]["robot_raw_joint_pos"][side]))
        fk=forward_kinematics(YAM_URDF,joints);B=np.array(snapshot["bases"][side]);errors=[]
        for name,actual_T in snapshot["links"][side].items():
            if name not in fk:continue
            predicted=B@fk[name];actual_T=np.array(actual_T)
            errors.append(dict(link=name,position_m=float(np.linalg.norm(predicted[:3,3]-actual_T[:3,3])),
                rotation_rad=float(np.linalg.norm(Rotation.from_matrix(predicted[:3,:3]@actual_T[:3,:3].T).as_rotvec()))))
        calibration[side]=errors
    snapshot["fk_calibration"]=calibration
    blob=json.dumps(snapshot,indent=2,default=float)
    (args.out/"b-live-state.json").write_text(blob)
    state_hash=hashlib.sha256(blob.encode()).hexdigest();report["b_live_state_sha256"]=state_hash
    report["raw_finger_measurement_complete"]=True
    report["fk_calibration"]=calibration
    assert all(e["position_m"]<.001 and e["rotation_rad"]<.01 for es in calibration.values() for e in es),calibration
    # A bounded pause allows a state-aware planner to supply DATA, never task-state writes.
    plan_path=args.out/"b-plan.json";wait_start=time.monotonic();wait_steps=steps
    choice=json.loads((HERE/"B-route-choice.json").read_text())
    child_env={**os.environ,"OPENBLAS_NUM_THREADS":"1","OMP_NUM_THREADS":"1","PYTHONDONTWRITEBYTECODE":"1"}
    deadline=wait_start+300
    commands=[
        [sys.executable,"-B",str(HERE/"plan_B_route_candidate.py"),"--state",str(args.out/"b-live-state.json"),"--task-root",str(src),"--out",str(args.out),"--settled-inboard","--yaw",str(choice["yaw"]),"--tilt",str(choice["tilt"]),"--contact-x",str(choice["contact_x"]),"--contact-z",str(choice["contact_z"]),"--route",choice["route"],"--bar-offset",str(choice["bar_offset"]),"--right-retreat",*[str(v) for v in choice["right_retreat"]]],
        [sys.executable,"-B",str(HERE/"promote_B_route_plan.py"),"--trial",str(args.out)]]
    for command in commands:subprocess.run(command,check=True,timeout=max(.001,deadline-time.monotonic()),env=child_env)
    report["planning_pause_wall_s"]=time.monotonic()-wait_start
    assert steps==wait_steps
    if plan_path.exists():
        plan=json.loads(plan_path.read_text())
        assert plan["state_sha256"]==state_hash and plan["status"]=="ready"
        assert plan["waypoints"] and len(plan["waypoints"])<1000
        # Traverse the checked wrist transition by continuous nominal JOINT targets.
        joint_prefix=[w for w in plan["waypoints"] if w["stage"] in ("B_retract_and_orient","B_orient_outside")]
        assert joint_prefix
        for w in joint_prefix:
            nominal=np.asarray(w["q"],float);assert nominal.shape==(6,) and np.isfinite(nominal).all()
            for attempt in range(15):
                targets[7:13]=compensate("right","b",nominal,targets[7:13].copy())
                hold_left_press(25.);motion_debug["right"]=dict(stage="B_joint_reorient",nominal_q=nominal.tolist())
                tick("B_joint_reorient")
                actual=np.asarray(obs.state["joint_pos"],float)[7:13]
                if np.max(np.abs(actual-nominal))<.04:break
            else:raise RuntimeError("Checked B joint transition did not track: "+str(actual-nominal))
        report["B_joint_prefix_waypoints"]=len(joint_prefix)
        # A small joint error alone does not establish a stationary held-object transform.
        settle_samples=[]
        for settle_control in range(120):
            targets[7:13]=compensate("right","b",nominal,targets[7:13].copy())
            hold_left_press(25.)
            motion_debug["right"]=dict(stage="B_joint_settle",nominal_q=nominal.tolist())
            tick("B_joint_settle")
            row=records[-1]
            assert row["straps"]["strap_4"]["seated_a"]==0 and not row["robot_contacts"]["strap_4_a"]
            assert row["robot_contacts"]["strap_4_b"] and row["totes"]["tote_tall"]["lid_gap_m"]<.002
            actual=np.asarray(obs.state["joint_pos"],float)[7:13]
            handR=link_poses(movers["right"].robot)["gripper"][:3,:3]
            hp,hquat=kit._hook_pose(base,"strap_4_b")
            relative=handR.T@strap.quat_to_R(hquat)
            offset=handR.T@(hp-tcp_world(movers["right"].robot))
            settle_samples.append((actual.copy(),offset.copy(),relative.copy()))
            if len(settle_samples)>=20:
                window=settle_samples[-20:]
                qspan=float(np.max(np.ptp(np.array([v[0] for v in window]),axis=0)))
                offsetspan=max(float(np.linalg.norm(v[1]-window[0][1])) for v in window)
                rotationspan=max(float(np.linalg.norm(Rotation.from_matrix(v[2]@window[0][2].T).as_rotvec())) for v in window)
                report["B_settle"]=dict(controls=settle_control+1,window_controls=20,
                    joint_span_rad=qspan,relative_translation_span_m=offsetspan,
                    relative_rotation_span_rad=rotationspan,
                    joint_error_rad=float(np.max(np.abs(actual-nominal))))
                if qspan<.01 and offsetspan<.001 and rotationspan<.02 and np.max(np.abs(actual-nominal))<.04:
                    break
        else:raise RuntimeError("B held-object transform did not settle: "+str(report.get("B_settle")))
        # Keep the public observation/action contract unchanged; export private raw telemetry.
        snapshot=dict(row=records[-1],raw_joint_names=ARM_JOINTS+FINGER_JOINTS,
            bases={side:link_poses(movers[side].robot)["base"].tolist() for side in movers},
            links={side:{n:T.tolist() for n,T in link_poses(movers[side].robot).items()} for side in movers},
            task_source_sha256=json.loads((args.out/"source-sha256.json").read_text()))
        calibration={}
        for side in movers:
            joints=dict(zip(ARM_JOINTS+FINGER_JOINTS,snapshot["row"]["robot_raw_joint_pos"][side]))
            fk=forward_kinematics(YAM_URDF,joints);B=np.array(snapshot["bases"][side]);errors=[]
            for name,actual_T in snapshot["links"][side].items():
                if name not in fk:continue
                predicted=B@fk[name];actual_T=np.array(actual_T)
                errors.append(dict(link=name,position_m=float(np.linalg.norm(predicted[:3,3]-actual_T[:3,3])),
                    rotation_rad=float(np.linalg.norm(Rotation.from_matrix(predicted[:3,:3]@actual_T[:3,:3].T).as_rotvec()))))
            calibration[side]=errors
        snapshot["fk_calibration"]=calibration
        blob=json.dumps(snapshot,indent=2,default=float)
        (args.out/"b-orient-live-state.json").write_text(blob)
        oriented_hash=hashlib.sha256(blob.encode()).hexdigest();report["b_orient_state_sha256"]=oriented_hash
        report["raw_finger_measurement_complete"]=True
        report["fk_calibration"]=calibration
        assert all(e["position_m"]<.001 and e["rotation_rad"]<.01 for es in calibration.values() for e in es),calibration
        final_plan_path=args.out/"b-final-plan.json";final_wait=time.monotonic();final_wait_steps=steps
        deadline=final_wait+300
        commands=[
            [sys.executable,"-B",str(HERE/"plan_B_route_candidate.py"),"--state",str(args.out/"b-orient-live-state.json"),"--task-root",str(src),"--out",str(args.out/"b-final"),"--skip-orient","--settled-inboard","--yaw",str(choice["final_yaw"]),"--tilt",str(choice["tilt"]),"--contact-x",str(choice["contact_x"]),"--contact-z",str(choice["contact_z"]),"--route",choice["route"],"--bar-offset",str(choice["bar_offset"]),"--right-retreat",*[str(v) for v in choice["right_retreat"]]],
            [sys.executable,"-B",str(HERE/"promote_B_route_plan.py"),"--trial",str(args.out),"--state-name","b-orient-live-state.json","--audit-subdir","b-final","--output-name","b-final-plan.json"]]
        for command in commands:subprocess.run(command,check=True,timeout=max(.001,deadline-time.monotonic()),env=child_env)
        assert steps==final_wait_steps and final_plan_path.exists(),"No checked postorientation B plan arrived"
        report["B_postorientation_pause_wall_s"]=time.monotonic()-final_wait
        plan=json.loads(final_plan_path.read_text())
        assert plan["state_sha256"]==oriented_hash and plan["status"]=="ready" and plan["waypoints"]
        assert all(w["stage"] not in ("B_retract_and_orient","B_orient_outside") for w in plan["waypoints"])
        position_feedback=np.zeros(3)
        for waypoint_index,w in enumerate(plan["waypoints"]):
            final_waypoint=waypoint_index==len(plan["waypoints"])-1
            goal=np.array(w["hook_pos"],float);Rgoal=np.array(w["hook_R"],float)
            assert goal.shape==(3,) and Rgoal.shape==(3,3) and np.isfinite(goal).all() and np.isfinite(Rgoal).all()
            assert np.allclose(Rgoal.T@Rgoal,np.eye(3),atol=1e-6) and np.linalg.det(Rgoal)>.999999
            if not final_waypoint:
                nominal=np.asarray(w["q"],float)
                assert nominal.shape==(6,) and np.isfinite(nominal).all()
                for attempt in range(15):
                    targets[7:13]=compensate("right","b",nominal,targets[7:13].copy())
                    hold_left_press(25.)
                    hp,hq=kit._hook_pose(base,"strap_4_b")
                    Rh=strap.quat_to_R(hq)
                    motion_debug["right"]=dict(stage=w["stage"],control="checked_nominal_joint_transit",
                        nominal_q=nominal.tolist(),hook=hp.tolist(),hook_goal=goal.tolist(),
                        hook_error_m=float(np.linalg.norm(hp-goal)),
                        rotation_error_rad=float(np.linalg.norm(Rotation.from_matrix(Rgoal@Rh.T).as_rotvec())))
                    tick(w["stage"])
                    actual=np.asarray(obs.state["joint_pos"],float)[7:13]
                    if np.max(np.abs(actual-nominal))<.04:break
                else:raise RuntimeError("B nominal joint transit did not track: "+str(actual-nominal))
                continue
            for attempt in range(60 if final_waypoint else 15):
                robot=movers["right"].robot;p=tcp_world(robot);R=link_poses(robot)["gripper"][:3,:3]
                hp,hq=kit._hook_pose(base,"strap_4_b");Rh=strap.quat_to_R(hq)
                relative=R.T@Rh;desired_R=Rgoal@relative.T;offset=R.T@(hp-p)
                position_feedback=np.clip(.9*position_feedback+.15*(goal-hp),-.004,.004)
                movers["right"].solve(goal-desired_R@offset+position_feedback,desired_R,seed=np.asarray(w["q"],float))
                targets[7:13]=compensate("right","b",movers["right"].last_q,targets[7:13].copy())
                hold_left_press(25.)
                motion_debug["right"]=dict(stage=w["stage"],hook=hp.tolist(),hook_goal=goal.tolist(),hook_error_m=float(np.linalg.norm(hp-goal)),
                    rotation_error_rad=float(np.linalg.norm(Rotation.from_matrix(Rgoal@Rh.T).as_rotvec())),payload=movers["right"].payload_compensation)
                tick(w["stage"])
                hp,hq=kit._hook_pose(base,"strap_4_b");Rh=strap.quat_to_R(hq)
                if np.linalg.norm(hp-goal)<(.003 if final_waypoint else .006) and np.linalg.norm(Rotation.from_matrix(Rgoal@Rh.T).as_rotvec())<(.02 if final_waypoint else .10):break
            else:raise RuntimeError("B path did not track: "+str(motion_debug["right"]))
        throat_local=np.array(plan["contact_local"],float);bar_goal=np.array(plan["bar_goal"],float)
        assert throat_local.shape==(3,) and bar_goal.shape==(3,) and np.isfinite(throat_local).all() and np.isfinite(bar_goal).all()
        robot=movers["right"].robot;hand_R=link_poses(robot)["gripper"][:3,:3].copy();feedback=np.zeros(3);stable=0
        for k in range(120):
            R=link_poses(robot)["gripper"][:3,:3];p=tcp_world(robot);hp,hq=kit._hook_pose(base,"strap_4_b");Rh=strap.quat_to_R(hq)
            point=hp+Rh@throat_local;feedback=np.clip(feedback+.08*(bar_goal-point),-.006,.006);offset=R.T@(hp-p)
            movers["right"].solve(bar_goal-Rh@throat_local-hand_R@offset+feedback,hand_R,seed=np.asarray(obs.state["joint_pos"],float)[7:13])
            targets[7:13]=compensate("right","b",movers["right"].last_q,targets[7:13].copy());hold_left_press(25.)
            motion_debug["right"]=dict(stage="B_measured_contact",point=point.tolist(),goal=bar_goal.tolist(),error_m=float(np.linalg.norm(point-bar_goal)))
            tick("B_measured_contact")
            hp,hq=kit._hook_pose(base,"strap_4_b");point=hp+strap.quat_to_R(hq)@throat_local
            good=np.linalg.norm(point-bar_goal)<.0015 and kit._LAST_CONTROL_ROW["straps"]["strap_4"]["seated_b"]==1
            stable=stable+1 if good else 0
            if stable>=5:break
        else:raise RuntimeError("B contact did not converge: "+str(motion_debug["right"]))
        report["B_held_seat"]=dict(kit._LAST_CONTROL_ROW["straps"]["strap_4"])
        loaded=targets[7:13].copy();empty_hold=None
        raw=np.asarray(records[-1]["robot_raw_joint_pos"]["right"][6:],float)
        opening_start=float(np.clip(np.max(raw/FINGER_OPEN),0,1));report["B_opening_start"]=opening_start
        for k in range(1,31):
            t=opening_start+(1-opening_start)*k/30
            if empty_hold is None and not kit._LAST_CONTROL_ROW["robot_contacts"]["strap_4_b"]:
                empty_hold=np.asarray(obs.state["joint_pos"],float)[7:13].copy()
            if empty_hold is None:targets[7:13]=loaded.copy()
            else:targets[7:13]=np.clip(targets[7:13]+np.clip(empty_hold-targets[7:13],-.04,.04),lo,hi)
            targets[13]=t
            hold_left_press(25.);tick("B_open_gradual")
        robot=movers["right"].robot;p0=tcp_world(robot).copy();R0=link_poses(robot)["gripper"][:3,:3].copy()
        retreat_vector=np.asarray(plan["right_retreat_vector"],float)
        assert retreat_vector.shape==(3,) and np.isfinite(retreat_vector).all() and np.linalg.norm(retreat_vector)<.2
        for k in range(1,41):
            targets[7:13]=movers["right"].solve(p0+retreat_vector*k/40,R0,
                seed=np.asarray(obs.state["joint_pos"],float)[7:13],command_ref=targets[7:13].copy())
            hold_left_press(25.);tick("B_retreat")
        if kit._LAST_CONTROL_ROW["straps"]["strap_4"]["seated_b"]!=1:raise RuntimeError("B not retained after real release/retreat")
        for k in range(1,41):
            hold_left_press(25.*(1-k/40));tick("left_press_unload")
        robot=movers["left"].robot;p0=tcp_world(robot).copy();R0=link_poses(robot)["gripper"][:3,:3].copy()
        for k in range(1,41):
            targets[:6]=movers["left"].solve(p0+np.array([-.08,.06,.10])*k/40,R0,
                seed=np.asarray(obs.state["joint_pos"],float)[:6],command_ref=targets[:6].copy())
            tick("left_press_retreat")
        for _ in range(420):tick("full_goal_hands_off")
        report["full_goal_reached"]=bool(kit.current_score()==1.)
        if not report["full_goal_reached"]:raise RuntimeError("Full goal not reached after genuine hands-off hold")
    else:
        report["B_continuation_not_run"]="No plan arrived during the bounded planning window; raw measurement/lid-press diagnostic only"
    final={end:kit._hook_pose(base,"strap_4_"+end)[0].tolist() for end in "ab"}
    report.update(steps=steps,hook_final=final,initial= {k:v.tolist() for k,v in initial.items()},
        lifted_each_over_5cm=all(final[e][2]-initial[s][2]>.05 for s,e in (("left","a"),("right","b"))),
        final_score=kit.current_score(),scope=("full-goal robot trial; no model evidence" if report.get("full_goal_reached") else "raw-finger calibration / first-hook and lid-press milestone only; no model/fullgoal claim"))
except BaseException as exc:
    report.update(error_type=type(exc).__name__,error=str(exc),steps=steps)
    raise
finally:
    journal.close()
    try:
        if recorder is not None:
            report["recording"]=recorder.close()
    finally:
        (args.out/"result.json").write_text(json.dumps(report,indent=2,default=float))
        emb.close()
