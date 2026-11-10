"""Portable measured-state B placement and release planning. No physics writes."""
from pathlib import Path
import argparse,hashlib,json,sys,time,numpy as np
from scipy.spatial.transform import Rotation,Slerp
p=argparse.ArgumentParser();p.add_argument("--state",type=Path,required=True);p.add_argument("--task-root",type=Path,default=Path(__file__).resolve().parent);p.add_argument("--out",type=Path,required=True);p.add_argument("--yaw",type=float,default=44.);p.add_argument("--skip-orient",action="store_true");p.add_argument("--settled-inboard",action="store_true");p.add_argument("--tilt",type=float,default=15.);p.add_argument("--contact-x",type=float,default=.0495);p.add_argument("--contact-z",type=float,default=-.014);p.add_argument("--right-retreat",type=float,nargs=3,default=[-.06,-.04,.08]);p.add_argument("--bar-offset",type=float,default=.008);p.add_argument("--route",choices=["outside","inboard","mouth"],default="outside");a=p.parse_args()
right_retreat_vector=np.asarray(a.right_retreat,float);assert right_retreat_vector.shape==(3,) and np.isfinite(right_retreat_vector).all() and np.linalg.norm(right_retreat_vector)<.2
a.out.mkdir(exist_ok=True,parents=True);sys.path.insert(0,str(a.task_root/"d1r_arena"))
import geometry as geo,strap
from routeguard import RouteGuard
from cord_gripper_contact import classify
from measured_planning_scene import MeasuredScene
from yam_arena.constants import FINGER_OPEN
blob=a.state.read_bytes();snapshot=json.loads(blob);digest=hashlib.sha256(blob).hexdigest();scene=MeasuredScene(snapshot,geo,strap);row=scene.row
assert row["guard_failure"] is None and row["straps"]["strap_4"]["failure"] is None
assert row["straps"]["strap_4"]["seated_a"]==0 and not row["robot_contacts"]["strap_4_a"]
assert row["robot_contacts_measured"] and row["robot_contacts"]["strap_4_b"]
assert row["totes"]["tote_tall"]["lid_gap_m"]<.002
assert all(snapshot.get("links",{}).get(s) for s in ("left","right"))
fk=scene.verify_fk();assert all(e["position_m"]<.001 and e["rotation_rad"]<.01 for es in fk.values() for e in es)
g=scene.grasp("right","strap_4_b");eye=scene.frame.world(scene.frame.eyes[1]);axis=scene.frame.bar_axis_world
Ri=next(h["R"] for h in geo.hooks() if h["name"]=="strap_4_b")
endR=Rotation.from_rotvec(np.deg2rad(a.tilt)*axis).as_matrix()@Rotation.from_rotvec([0,0,np.deg2rad(a.yaw)]).as_matrix()@Rotation.from_rotvec([0,0,-np.pi]).as_matrix()@Ri
barpoint=eye+a.bar_offset*axis;contact=np.array([a.contact_x,0,a.contact_z]);endp=barpoint-endR@contact;endhand=endR@g["relative"].T;endtcp=endp-endhand@g["offset"]
source_goal_seated_bar=strap.seat_check(endp,Rotation.from_matrix(endR).as_quat(),0.,scene.frame)
assert source_goal_seated_bar==1,"B goal fails source seating predicate"
goal_geometry=scene.check_held_hook("strap_4_b",endp,endR,bar_eye=eye)
assert not goal_geometry["fixture_hits"],("B goal intersects physical fixture",goal_geometry)
assert min(v["gap_m"] for v in goal_geometry["all_bar_capsule_gaps"])>=-.0001,("B goal intersects eye bar",goal_geometry)
endq=scene.fit_pose("right",endtcp,endhand,scene.raw["right"][:6],True);assert endq is not None,"B endpoint IK failed"
candidate=dict(world_bar_axis_tilt_deg=a.tilt,yaw_deg=a.yaw,seat_x_m=a.contact_x,seat_z_m=a.contact_z,bar_axis_offset_m=a.bar_offset,hook_pos=endp.tolist(),hook_R=endR.tolist(),tcp=endtcp.tolist(),R=endhand.tolist(),q=endq.tolist())
startp=g["hook_pos"];startR=g["hook_R"];outside=startp+np.array([-.035,-.045,-.035] if a.route=="outside" else [0.,.03,.03]);partial=Slerp([0,1],Rotation.from_matrix(np.array([startR,endR])))(.30).as_matrix()
stages=[("B_retract_and_orient",outside,partial),("B_orient_outside",outside,endR),("B_above_bar_end",endp+np.array([.04,-.02,.06]),endR),("B_lower_outside_bar",endp+np.array([.035,0,0]),endR),("B_slide_onto_bar",endp,endR)]
if a.route=="mouth":
 stages=stages[:2]+[("B_above_bar_end",endp+np.array([.015,.03,.10]),endR),
          ("B_lower_mouth_clear",endp+np.array([0.,.020,.004]),endR),
          ("B_enter_mouth",endp,endR)]
elif a.settled_inboard:
 stages=stages[:2]+[("B_above_bar_end",endp+np.array([.015,.03,.10]),endR),
          ("B_lower_inboard",endp+np.array([.025,.02,0.]),endR),
          ("B_move_to_bar_axis",endp+np.array([.025,0.,0.]),endR),
          ("B_slide_onto_bar",endp,endR)]
if a.skip_orient:stages=stages[2:]
A=row["hooks"]["strap_4_a"];attachA=strap.hook_attach_world(A["pos"],A["quat_xyzw"]);gap=row["totes"]["tote_tall"]["lid_gap_m"];gaps={n:row["totes"][n]["lid_gap_m"] for n in ("tote_tall","tote_low")}
cord=strap.Cord(3,scene.frame);cord.route=snapshot.get("cord_state",{}).get("route",row["straps"]["strap_4"]["route"]);cord.prev_above=snapshot.get("cord_state",{}).get("prev_above",True);guard=RouteGuard()
assert guard.check(cord.path(attachA,startp+startR@strap.ATTACH_LOCAL,gap),gaps) is None
records=[];issues=[];maxT=0.;min_wrist5=10.;mins=dict(fixture=1.,self=1.,cross=1.,held=1.);seed=scene.raw["right"][:6].copy();previous_p=startp;previous_R=startR
for stage,goal,Rgoal in stages:
 count=max(1,int(np.ceil(np.linalg.norm(goal-previous_p)/.003)),int(np.ceil(np.linalg.norm(Rotation.from_matrix(Rgoal@previous_R.T).as_rotvec())/np.deg2rad(3))))
 interp=Slerp([0,1],Rotation.from_matrix(np.array([previous_R,Rgoal])))
 for fraction in np.linspace(0,1,count+1)[1:]:
  hp=(1-fraction)*previous_p+fraction*goal;Rh=interp(fraction).as_matrix();hand=Rh@g["relative"].T;tcp=hp-hand@g["offset"]
  q=scene.fit_pose("right",tcp,hand,seed,False)
  if q is None:issues.append(dict(stage=stage,kind="IK",fraction=float(fraction)));break
  if np.max(np.abs(q-seed))>.18:issues.append(dict(stage=stage,kind="joint_branch_jump"));break
  min_wrist5=min(min_wrist5,abs(float(q[4])));seed=q;scene.add_model_hook("strap_4_b",hp,Rh)
  attach=hp+Rh@strap.ATTACH_LOCAL;cord.update_route(attachA,attach,gap);points=cord.path(attachA,attach,gap);failure=guard.check(points,gaps)
  T=max(0.,geo.STRAP_EA*(sum(np.linalg.norm(v-u) for u,v in zip(points[:-1],points[1:]))/geo.RELAXED_LENGTHS[3]-1));maxT=max(maxT,T)
  if failure or cord.failure or T>=28:issues.append(dict(stage=stage,kind="cord_guard_or_load",failure=failure or cord.failure,tension_n=T,detail=guard.detail));break
  check=scene.check_robot("right",q,points,held_hook="strap_4_b")
  held=scene.check_held_hook("strap_4_b",hp,Rh,bar_eye=eye)
  for key,value in [("fixture",check["fixture_separation_m"]),("self",check["self_hand_separation_m"]),("cross",check["other_arm_separation_m"]),("held",held["minimum_fixture_m"])]:mins[key]=min(mins[key],value)
  if min(check["fixture_separation_m"],check["self_hand_separation_m"],check["other_arm_separation_m"])<.001:issues.append(dict(stage=stage,kind="robot",check=check));break
  if classify(check,segment=len(points)-2,pad_proxy=True)["issues"]:issues.append(dict(stage=stage,kind="cord_robot",detail=check["exact_cord_flags"]));break
  if held["fixture_hits"]:issues.append(dict(stage=stage,kind="held_fixture",detail=held));break
  records.append(dict(stage=stage,hook_pos=hp.tolist(),hook_R=Rh.tolist(),q=q.tolist(),tension_n=T,held_check=held,robot_check=check,owned_segment=len(points)-2))
 if issues:break
 previous_p=goal;previous_R=Rgoal
 print("PORTABLE_B_STAGE",stage,len(records),flush=True)
attempt=dict(route=a.route,minimum_absolute_wrist5_rad=min_wrist5,candidate=candidate,waypoint_count=len(records),issues=issues,max_tension_n=maxT,minimum_fixture_m=mins["fixture"],minimum_self_m=mins["self"],minimum_cross_m=mins["cross"],minimum_held_fixture_m=mins["held"])
audit=dict(kind="Read-only measured B nominal plan; no actual robot/fullgoal proof",state_sha256=digest,source24=snapshot["task_source_sha256"],status="checked" if not issues else "failed",attempts=[attempt],waypoints=records,contact_local=contact.tolist(),bar_goal=barpoint.tolist(),fk_errors=fk,initial_guard_passed=True,source_goal_seated_bar=source_goal_seated_bar,goal_geometry=goal_geometry)
(a.out/"b-path-audit.json").write_text(json.dumps(audit,indent=2));assert not issues,issues
# Opening and both retreat paths, with nominal released B represented in the scene.
q=np.array(records[-1]["q"]);scene.add_model_hook("strap_4_b",endp,endR);points=cord.path(attachA,endp+endR@strap.ATTACH_LOCAL,gap);opening=[]
import itertools
owned_segment=len(points)-2
for fl,fr in itertools.product(np.linspace(0,1,11),repeat=2):
 fraction=min(fl,fr);progress=np.array([fl,fr])
 fingers=(1-progress)*scene.raw["right"][6:]+progress*FINGER_OPEN
 check=scene.check_robot("right",q,points,held_hook="strap_4_b",fingers=fingers)
 assert min(check["fixture_separation_m"],check["self_hand_separation_m"],check["other_arm_separation_m"])>.001,("opening",fraction,check)
 classification=classify(check,segment=owned_segment,pad_proxy=True)
 assert not classification["issues"],("opening_cord",fraction,classification)
 opening.append(dict(fraction=float(fraction),finger_fractions=[float(fl),float(fr)],check=check,cord_contact=classification))
right=[];qright=q.copy()
for fraction in np.linspace(0,1,46):
 fit=scene.fit_pose("right",endtcp+right_retreat_vector*fraction,endhand,qright,False);assert fit is not None,("right_retreat_IK",fraction)
 qright=fit;check=scene.check_robot("right",fit,points,fingers=[FINGER_OPEN]*2)
 assert min(check["fixture_separation_m"],check["self_hand_separation_m"],check["other_arm_separation_m"])>.001,("right_retreat",fraction,check)
 classification=classify(check,segment=owned_segment,pad_proxy=False)
 assert not classification["issues"] and (fraction<1. or not classification["surface_contacts"]),("right_retreat_cord",fraction,classification)
 right.append(dict(fraction=float(fraction),q=fit.tolist(),check=check,cord_contact=classification))
scene.raw["right"][6:]=FINGER_OPEN
left=[];qleft=scene.raw["left"][:6].copy();lefttcp,leftR=scene.tcp("left",qleft)
for fraction in np.linspace(0,1,61):
 fit=scene.fit_pose("left",lefttcp+np.array([-.08,.06,.10])*fraction,leftR,qleft,False);assert fit is not None,("left_retreat_IK",fraction)
 qleft=fit;check=scene.check_robot("left",fit,points,other_q=qright,allow_lid=fraction<.1)
 assert min(check["fixture_separation_m"],check["self_hand_separation_m"],check["other_arm_separation_m"])>-.0001,("left_retreat",fraction,check)
 assert not any(v["distance_lower_m"]<.004-1e-7 for v in check["exact_cord_flags"]),("left_retreat_cord",fraction,check)
 left.append(dict(fraction=float(fraction),q=fit.tolist(),check=check))
release=dict(kind="Nominal B release/retreat; no robot proof",state_sha256=digest,owned_segment=owned_segment,candidate=candidate,opening=opening,right_retreat=right,left_retreat=left,right_retreat_vector=right_retreat_vector.tolist(),passed=True)
(a.out/"b-release-retreat-audit.json").write_text(json.dumps(release,indent=2))
print("PORTABLE_B_COMPLETE",len(records),len(opening),len(right),len(left),flush=True)
