"""Nut, clamp and arm mechanism, run every 2 ms physics step, and the measurements the scorer uses.

During actions this module changes only the nut's breakaway friction and opens the angle-triggered locks
(clamp release, rounding). It never places a wrench, moves a joint or adds a force. Only reset writes joint state.
"""
import math
import numpy as np
from scipy.spatial.transform import Rotation
from contacts import Contacts, arr
from scoring import Mechanism, Hold, racked

RUN_LIMIT = 3870
NAMES = ['w12', 'w1_2', 'w13', 'w14', 'wadj']


class Physics:
    def __init__(self, emb, fixture):
        self.emb = emb; self.base = emb._base; self.f = fixture
        self.asset = self.base.scene['wrench_fixture']; self.settling = True
        self.qids = {n: i for i, n in enumerate(self.asset.joint_names)}
        self.bids = {n: i for i, n in enumerate(self.asset.body_names)}
        self.R = np.asarray(fixture['rotation']); self.T = np.asarray(fixture['translation'])
        self.origin = arr(self.base.scene.env_origins)[0]
        self.mechanism = Mechanism(fixture['params']); self.hold = Hold()
        self.done = False; self.error = None; self.controls = 0; self.events = []; self.last = None
        # Rack position of each wrench: its published start pose.
        bodies = {b['name']: b for b in fixture['bodies']}
        self.rack = {}
        for n in NAMES:
            b = bodies[n]; q = b['quat']
            rot = self.R @ Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
            self.rack[n] = (self.R @ b['pos'] + self.T, rot @ [0, -1, 0])
        from isaaclab_physx.physics import PhysxManager
        sim = PhysxManager.get_physics_sim_view()
        links = list(self.asset.root_view.link_paths[0])
        self.tools = [str(self.base.scene[n].root_view.prim_paths[0]) for n in NAMES]
        robots = sum([list(self.base.scene[s + '_robot'].root_view.link_paths[0]) for s in ['left', 'right']], [])
        self.nut_path = links[self.bids['latch_head']]; self.arm_path = links[self.bids['swing_arm']]
        self.nut_contact = Contacts(sim, [self.nut_path], self.tools)
        self.hands_off = Contacts(sim, [self.nut_path, self.arm_path], robots + self.tools)
        self.wrench_touch = Contacts(sim, robots, self.tools)

    def limits(self, name, values):
        import torch
        self.asset.write_joint_position_limit_to_sim_index(limits=torch.tensor([[values]], dtype=torch.float32, device=self.base.device), env_ids=[0], full_data=False, joint_ids=[self.qids[name]], warn_limit_violation=False)

    def reset_mechanisms(self):
        import torch
        self.settling = True; self.mechanism = Mechanism(self.f['params']); self.hold = Hold()
        self.done = False; self.error = None; self.controls = 0; self.events = []; self.last = None
        self.limits('arm_hinge', [0., 0.]); self.limits('latch_slip', [0., 0.])
        z = torch.zeros_like(self.asset.data.joint_pos.torch)
        self.asset.write_joint_state_to_sim_index(position=z, velocity=z)
        self.asset.set_joint_effort_target(z)
        self.rounded_visual(False)

    def begin(self):
        self.hold = Hold(); self.settling = False; self.controls = 0; self.done = False

    def pre_step(self, dt):
        import torch
        if self.error: return
        try:
            q = float(arr(self.asset.data.joint_pos)[0, self.qids['latch_turn']])
            tau = torch.tensor([[float(self.mechanism.friction(q))]], dtype=torch.float32, device=self.base.device)
            self.asset.write_joint_friction_coefficient_to_sim_index(joint_friction_coeff=tau, joint_dynamic_friction_coeff=tau, joint_viscous_friction_coeff=torch.tensor([[.0005]], dtype=torch.float32, device=self.base.device), env_ids=[0], full_data=False, joint_ids=[self.qids['latch_turn']])
        except Exception as e: self.error = type(e).__name__ + ': ' + str(e)

    def post_step(self, dt):
        if self.settling or self.error: return
        try:
            q = arr(self.asset.data.joint_pos)[0]; states = arr(self.asset.data.body_link_state_w)[0]
            if not np.isfinite(np.r_[q, states.ravel()]).all(): raise ValueError('nonfinite fixture state')
            head = states[self.bids['latch_head']]; axis = Rotation.from_quat(head[3:7]).as_matrix()[:, 2]
            _, _, M = self.nut_contact.read(dt, head[:3])
            torques = M[0] @ axis
            tools = {n: arr(self.base.scene[n].data.root_link_state_w)[0] for n in NAMES}
            if not all(np.isfinite(s).all() for s in tools.values()): raise ValueError('nonfinite wrench state')
            offset = [(self.R.T @ (tools[n][:3] - head[:3]))[:2] for n in NAMES]
            turn = float(q[self.qids['latch_turn']]); arm = float(q[self.qids['arm_hinge']])
            for event in self.mechanism.update(dt, turn, torques, offset):
                if event == 'nut_rounded': self.limits('latch_slip', [-1e6, 1e6]); self.rounded_visual(True)
                if event == 'arm_released': self.limits('arm_hinge', [-math.pi / 2, 0.])
                self.events.append(dict(event=event, step=self.controls, nut_turn_deg=math.degrees(turn)))
            on_rack = []; detail = []
            for n in NAMES:
                pos0, axis0 = self.rack[n]; s = tools[n]
                rot = Rotation.from_quat(s[3:7]).as_matrix(); d = s[:3] - self.origin - pos0; long = rot @ [0, -1, 0]
                heading = math.degrees(math.atan2(axis0[0] * long[1] - axis0[1] * long[0], axis0[0] * long[0] + axis0[1] * long[1]))
                tilt = math.degrees(math.acos(min(1., abs(rot[2, 2]))))
                on_rack.append(racked(1000 * math.hypot(d[0], d[1]), heading, 1000 * d[2], tilt))
                detail.append([round(1000 * math.hypot(d[0], d[1]), 1), round(heading, 1), round(1000 * d[2], 1), round(tilt, 1)])
            m = dict(nut_turn_deg=math.degrees(turn), arm_open_deg=-math.degrees(arm), nut_rounded=self.mechanism.rounded,
                     nut_or_arm_touched=bool(self.hands_off.read(dt)[0].any()), wrench_racked=on_rack,
                     wrench_touched=bool(self.wrench_touch.read(dt)[0].any()), wrench_offset_mm_deg_mm_deg=detail)
            self.hold.update(m); self.last = m
        except Exception as e:
            self.error = type(e).__name__ + ': ' + str(e)

    def status(self):
        return dict(score=0. if self.error else self.hold.score(), measurement=self.last, hold_full_s=self.hold.full * .002,
                    hold_release_s=self.hold.release * .002, error=self.error, controls=self.controls, events=self.events)

    def reason(self):
        if self.error: return 'measurement_unavailable'
        if self.controls >= RUN_LIMIT: return 'time_limit'
        if self.status()['score'] >= 1.: return 'success'

    def rounded_visual(self, rounded):
        from isaaclab.sim.utils import get_current_stage
        from pxr import Gf
        stage = get_current_stage(); path = self.nut_path + '/latch_hex'
        color = (.85, .85, .88) if rounded else (.62, .60, .55)
        stage.GetPrimAtPath(path).GetAttribute('primvars:displayColor').Set([Gf.Vec3f(*color)])
        stage.GetPrimAtPath(path + '_material/Shader').GetAttribute('inputs:diffuseColor').Set(Gf.Vec3f(*color))
