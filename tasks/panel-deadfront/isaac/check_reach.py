"""Offline URDF check for the grasp, insertion path and fastening poses (no Isaac or dynamics)."""

import json
import math

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from yam_arena import station

import demo as D
import layout as L


def cover_path(setup, name):
    sign = 1 if name == "left" else -1
    gp, gr = L.grip_center(sign), L.grip_frame(sign)
    pc, rc = L.cover_stand_pose(setup)
    po = L.panel_origin(setup)
    first = L.ENTRY_WAYPOINTS[0]
    p1, r1 = L.guide_pose(L.INSERT_ANGLE, po, dx=first[0], dz=first[1])
    pick = pc + rc@gp
    arm = D.Arm(name, station.BASES[name])
    # Use the policy's actual multiseed pre-grasp solver.
    for _ in arm.goto(pick-.06*(rc@gr[:, 2]), rc@gr, 1., label="pregrasp"):
        pass
    tcp_path = [(pick - t*.06*(rc@gr[:, 2]), rc@gr) for t in np.linspace(1, 0, 10)]
    tcp_path += [(pick+rc[:, 2]*(t*D.LIFT), rc@gr) for t in np.linspace(0, 1, 10)]
    pc += rc[:, 2]*D.LIFT
    interpolate = Slerp([0, 1], Rotation.from_matrix([rc, r1]))
    poses = [(pc*(1-t)+p1*t, interpolate(t).as_matrix()) for t in np.linspace(0, 1, 25)]
    for start, end in zip(L.ENTRY_WAYPOINTS, L.ENTRY_WAYPOINTS[1:]):
        poses += [L.guide_pose(45, po, dx=dx, dz=dz) for dx, dz in np.linspace(start, end, 10)]
    poses += [L.guide_pose(t, po) for t in np.linspace(45, 0, 41)]
    # The bounded final seating adjustment can command any of these limiting poses.
    poses += [L.guide_pose(angle, po, dx=dx, dz=-.0006) for angle in (-.65, .65) for dx in (-.002, .002)]
    tcp_path += [(p+r@gp, r@gr) for p, r in poses]
    error = np.zeros(2)
    mass, com, _ = L.cover_mass_props()
    com[1] = sign*L.GRIP_Y
    max_torque = np.zeros(6)
    limit_violation = 0.
    from yam_arena.constants import SHOULDER, WRIST
    stiffness = np.array([SHOULDER["stiffness"]]*3 + [WRIST["stiffness"]]*3)
    for pt, rt in tcp_path:
        q, pe, ae = arm.solve(pt, rt)
        error = np.maximum(error, [pe*1000, math.degrees(ae)])
        arm.q = q
        arm.support(pt + rt @ gr.T @ (com-gp), mass*9.81*.5)
        command = q+arm.payload_offset
        max_torque = np.maximum(max_torque, abs(arm.payload_offset*stiffness))
        limit_violation = max(limit_violation, max(arm.lo-command), max(command-arm.hi))
    return dict(samples=len(tcp_path), max_mm=round(error[0], 3), max_deg=round(error[1], 3),
                load_nm=max_torque.round(2).tolist(), load_joint_limit_error_rad=round(limit_violation, 4))


def tool_transit_clearance(mouth, axis):
    # Sample each rigid segment, including the screw staged at the driver's nose.
    # Subtract its full radius and a further 1 mm for the maximum centerline sampling error.
    rim = L.DRIVER_HOLSTER + [0.,0.,L.HOLSTER_SIZE[2]/2]
    offsets = np.array(L.HOLSTER_SIZE)/2
    floors, holster = [], []
    for lo, hi, radius in ((-.154,-.049,L.DRV_HANDLE_R),(-.049,-.009,L.DRV_SHAFT_R),
                           (-.009,.0035,L.HEAD_R),(.0035,.0215,L.SHANK_R)):
        stations = np.linspace(lo, hi, int(np.ceil((hi-lo)/.002))+1)
        pts = mouth + stations[:, None]*axis
        floors.append(min(pts[:,2])-radius-L.TOP-.001)
        delta = np.maximum(abs(pts-rim)-offsets,0.)
        holster.append(min(np.linalg.norm(delta,axis=1))-radius-.001)
    return min(floors), min(holster)


def fastening(setup):
    from tool_clearance import fixture_boxes

    c = D.Ctx()
    c.tool_fixtures = fixture_boxes(L.stand_pos(setup), L.ry(L.LEAN))
    c.right.measured_grip = L.DRV_HANDLE_R / D.FINGER_OPEN
    # Ideal in-hand transform for the policy's actual holster grasp.
    pd, rd = L.driver_holster_pose()
    pt, rt = D.driver_pick_pose(pd, rd)
    c.GD = (rt.T@(pd-pt), rt.T@rd)
    for motion in (c.right.goto(pt-.05*rt[:, 2], rt, 3., label="holster approach"),
                   c.right.move(pt, rt, 1.5, label="driver pickup"),
                   c.right.move(pt+[0., 0., .065], rt, 2., label="driver lift"),
                   c.right.move(D.driver_clear_point(pt), rt, 1.5, label="clear stand")):
        for _ in motion:
            pass
    axis = L.screw_axis_point(0, setup)
    standoff = axis + [L.head_under_x(0.)-L.HEAD_T-D.DRIVER_FAR, 0., 0.]
    c.right.measured = c.right.q.copy()
    clearance = np.full(2, np.inf)
    prev_q = c.right.q.copy()
    max_speed = 0.0
    arm_clear = float("inf")
    for _ in D.goto_driver(c, standoff, 4.5, "screw approach", observe_grip=False):
        max_speed = max(max_speed, float(np.max(abs(c.right.q-prev_q)))*D.CONTROL_HZ)
        prev_q = c.right.q.copy()
        mouth, axis_dir, _ = D._driver_of_q(c, c.right.q)
        clearance = np.minimum(clearance, tool_transit_clearance(mouth, axis_dir))
        arm_clear = min(arm_clear, c.tool_arm_clearance(c.right.q)[0])

    errors, sweeps = [], []
    for adv in (0., L.START_GAP/2, L.START_GAP):
        mouth = axis + [L.head_under_x(adv)-L.HEAD_T+D.DRV_PRESS, 0., 0.]
        q, pe, ae, phi = D.driver_ik(c, mouth, j6_pref=D.J6_PARK)
        c.right.q = q
        errors.append((pe*1000, ae))
        sweep = D.stroke_room(c, mouth, phi)
        sweeps.append(sweep)
        for roll in np.linspace(0., sweep, round(sweep/3)+1):
            q, pe, ae, _ = D.driver_ik(c, mouth, roll=phi+math.radians(roll), q0=q)
            arm_clear = min(arm_clear, c.tool_arm_clearance(q)[0])
            errors.append((pe*1000, ae))
    return dict(max_mm=round(max(e[0] for e in errors), 3),
                max_deg=round(max(e[1] for e in errors), 3), min_stroke_deg=min(sweeps),
                arm_fixture_gap_mm=round(arm_clear*1000,2),
                transit_max_joint_deg_s=round(math.degrees(max_speed),3),
                pickup_max_mm=round(c.right.worst_mm, 3), transit_table_mm=round(clearance[0]*1000,2),
                transit_holster_mm=round(clearance[1]*1000,2))


if __name__ == "__main__":
    good = True
    for setup in (0, 1, 2, ("seed", 0), ("seed", 1415158438)):
        hands = {name: cover_path(setup, name) for name in ("left", "right")}
        screw = fastening(setup)
        print(json.dumps(dict(setup=setup, hands=hands, screw=screw)), flush=True)
        good &= all(h["max_mm"] <= 4 and h["max_deg"] <= 4 and h["load_joint_limit_error_rad"] == 0
                    and max(h["load_nm"][:3]) < 27 and max(h["load_nm"][3:]) < 7 for h in hands.values())
        good &= screw["max_mm"] <= 1.5 and screw["max_deg"] <= 3 and screw["min_stroke_deg"] >= 30
        good &= screw["arm_fixture_gap_mm"] > 4
        good &= screw["transit_max_joint_deg_s"] <= math.degrees(D.TOOL_CARRY_JOINT_RATE)+.001
        good &= min(screw["transit_table_mm"], screw["transit_holster_mm"]) > 10
    raise SystemExit(0 if good else 1)
