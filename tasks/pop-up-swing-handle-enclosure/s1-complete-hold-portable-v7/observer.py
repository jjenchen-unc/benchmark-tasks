import numpy as np
import s1_kit as kit
from contacts import arr
emb=None
def robot_snapshot(observation):
    robots = {}
    for side in ['left', 'right']:
        r = emb._base.scene[side + '_robot']
        robots[side] = {'joint_names': list(r.joint_names), 'raw_joint_pos': arr(r.data.joint_pos)[0].copy(), 'raw_joint_vel': arr(r.data.joint_vel)[0].copy(), 'body_names': list(r.body_names), 'body_pos_w': arr(r.data.body_pos_w)[0].copy(), 'body_quat_w_xyzw': arr(r.data.body_quat_w)[0].copy(), 'root_pos_w': arr(r.data.root_pos_w)[0].copy(), 'root_quat_w_xyzw': arr(r.data.root_quat_w)[0].copy()}
    from yam_arena import YAM_URDF
    from yam_arena.urdf import forward_kinematics, quat_xyzw_to_matrix
    from yam_arena.constants import TCP_OFFSET
    for (side, row) in robots.items():
        fk = forward_kinematics(YAM_URDF, dict(zip(row['joint_names'], map(float, row['raw_joint_pos']))))
        R = quat_xyzw_to_matrix(np.asarray(row['root_quat_w_xyzw']))
        base = np.asarray(row['root_pos_w'])
        row['fk_link_origins_w'] = {name: base + R @ T[:3, 3] for (name, T) in fk.items()}
        row['fk_tcp_w'] = base + R @ (fk['gripper'][:3, 3] + fk['gripper'][:3, :3] @ np.asarray(TCP_OFFSET))
        row['fk_approach_w'] = R @ fk['gripper'][:3, 2]
    return {'public14': np.asarray(observation.state['joint_pos']).copy(), 'robots': robots}

def fixture_snapshot():
    E = kit.ENGINE
    return {'qv': E.state(), 'body_names': list(E.asset.body_names), 'body_pos_w': arr(E.asset.data.body_pos_w)[0].copy(), 'body_quat_w_xyzw': arr(E.asset.data.body_quat_w)[0].copy(), 'source_trigger_states': E.last, 'status': E.status()}

def geometry_proxies(robot, fixture):
    from yam_arena.urdf import quat_xyzw_to_matrix
    row = robot['robots']['right']
    names = fixture['body_names']
    j = names.index('cap_1')
    R = quat_xyzw_to_matrix(np.asarray(fixture['body_quat_w_xyzw'][j]))
    face = np.asarray(fixture['body_pos_w'][j]) + R @ np.array([0.0, 0.0, 0.014])
    tip = np.asarray(row['fk_tcp_w']) + 0.013 * np.asarray(row['fk_approach_w'])
    delta = tip - face
    lever = np.asarray(fixture['body_pos_w'][names.index('lever_1')])
    return {'cap_tip_midpoint_axial_m': float(delta @ R[:, 2]), 'cap_tip_midpoint_transverse_m': float(np.linalg.norm(delta - delta @ R[:, 2] * R[:, 2])), 'tcp_to_lever_origin_m': float(np.linalg.norm(np.asarray(row['fk_tcp_w']) - lever)), 'scope': 'FK midpoint proxies; individual mesh contacts remain measured sensor channels'}

def observe(obs):
 r=robot_snapshot(obs);f=fixture_snapshot();return dict(robot=r,fixture=f,geometry_proxies=geometry_proxies(r,f))
