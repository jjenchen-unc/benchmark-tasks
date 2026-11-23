"""Inverse dynamics for the YAM wrist carrying the driver, from its URDF inertial parameters."""
from functools import lru_cache
import xml.etree.ElementTree as ET

import numpy as np
from yam_arena import YAM_URDF, analysis
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS
from yam_arena.urdf import rpy_matrix

import fastik
import layout as L


@lru_cache(maxsize=1)
def link_inertias():
    inertias = {}
    for link in ET.parse(YAM_URDF).getroot().iter("link"):
        name = link.get("name")
        node = link.find("inertial")
        if node is None:
            continue
        origin = node.find("origin")
        xyz = np.fromstring(origin.get("xyz"), sep=" ")
        rot = rpy_matrix(np.fromstring(origin.get("rpy"), sep=" "))
        i = node.find("inertia").attrib
        inertia = np.array([[i["ixx"], i["ixy"], i["ixz"]], [i["ixy"], i["iyy"], i["iyz"]],
                            [i["ixz"], i["iyz"], i["izz"]]], float)
        inertias[name] = (float(node.find("mass").get("value")), xyz, rot@inertia@rot.T)
    return inertias


def joint_torque(q, qd, qdd, grasp, half_gap):
    """Generalized force at the requested joint motion, including the fingers and the weight of the external tool."""
    base = np.zeros(3)
    positions = dict(zip(ARM_JOINTS, q))
    positions.update(dict.fromkeys(FINGER_JOINTS, -half_gap))
    poses = analysis.fk_link_poses_world(positions, base)
    tcp, tip_rot, axes = fastik.fk(q)
    torque = np.zeros(6)
    omega, alpha, acc = np.zeros(3), np.zeros(3), np.zeros(3)
    last_origin = np.zeros(3)
    for k, name in enumerate(("link1", "link2", "link3", "link4", "link5", "gripper")):
        axis, origin = axes[k]
        offset = origin-last_origin
        acc += np.cross(alpha, offset) + np.cross(omega, np.cross(omega, offset))
        alpha += axis*qdd[k] + np.cross(omega, axis)*qd[k]
        omega += axis*qd[k]
        last_origin = origin
        bodies = []
        for member in ((name, "tip_left", "tip_right") if k == 5 else (name,)):
            mass, local, inertia = link_inertias()[member]
            tf = poses[member]
            rot = tf[:3, :3]
            bodies.append((mass, tf[:3, 3]+rot@local, rot@inertia@rot.T, False))
        if k == 5:
            gp, gr = grasp
            rot = tip_rot@gr
            com = tcp + tip_rot@gp + rot[:, 2]*(L.DRV_HANDLE_Z0+.040)
            bodies.append((L.DRV_MASS, com, rot@np.diag([1.6e-4, 1.6e-4, 2e-5])@rot.T, True))
        for mass, com, inertia, external in bodies:
            r = com-origin
            force = mass*(acc+np.cross(alpha, r)+np.cross(omega, np.cross(omega, r)))
            if external:
                # The stock YAM action already supplies whole-arm gravity compensation.
                force[2] += mass*9.81
            moment = inertia@alpha + np.cross(omega, inertia@omega)
            for i in range(k+1):
                ax, o = axes[i]
                torque[i] += ax@(moment+np.cross(com-o, force))
    return torque
