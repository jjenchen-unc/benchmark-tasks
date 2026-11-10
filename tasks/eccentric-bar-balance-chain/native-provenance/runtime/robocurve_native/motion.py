"""Public Panda pose tools over source-native robot actuators.

Implements the delivered HARNESS_REQUIREMENTS R1-R14 control conventions.
This is a new controller, not evidence that source references replay identically.
IK follows locking_pliers_assets/robot.py:ik_stock (measured joint seed,
DLS, null-space posture); rate interpolation follows magic_arm.Driver.
No task parameters, object coordinates, model fields or evaluator state are
returned by this module's public observation/tool results.
"""
from __future__ import annotations

from collections import deque
import math
import re

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

SIDES = ('left', 'right')
DT = .05
VELOCITY = np.array([2.175] * 4 + [2.61] * 3)
TORQUE = np.array([80.] * 4 + [12.] * 3)
BUDGETS = dict(kitchen_opening=25000, locking_pliers=18824, bike_pump=13216,
              pipe_wrench=11664, B2_ratchet=9918, S2_draw_latch=7964,
              caulking_gun=5530, lab_stand=5214, pins=4572, magic_arm=4266,
              claw_hammer=4220, S1_swinghandle=4198, B1_wrench=3870,
              S4_compression_latch=3848, cabinets=3776, S5_remote_release=3646,
              drawers=3586, S3_quarter_turn=3308, hydraulic_jack=3210,
              B3_hex=3060, B6_camera=3034, D1_straps=2584, bottle=2568,
              C1_liners=2268, B7_couplings=6000, B4_bars=1896,
              D3_foam=1604, B5_monitor=1428)


def workspace_bounds(bench):
    """Normalize explicitly declared public tool boxes, never infer from state."""
    value = getattr(bench, 'WORKSPACE', None)
    if value is None:
        for tool in getattr(bench, 'TOOLS', []):
            for key, scale in (('workspace', 1.), ('tool_point_box_m', 1.), ('workspace_mm', .001)):
                if key in tool:
                    return np.array([tool[key][k] for k in ('x','y','z')], dtype=float)*scale
        if getattr(bench, 'BENCH_ID', '') == 'B4_bars':
            # B4_bars.ROBOT_TEXT explicitly declares this box in millimetres.
            return np.array([[-.30,.35],[-.40,.40],[.825,1.18]])
        raise ValueError('Task has no recognized declared public workspace')
    if isinstance(value, dict):
        if 'lo' in value and 'hi' in value:
            return np.array([value['lo'], value['hi']], dtype=float).T
        if 'x_cm' in value:
            return np.array([value[k] for k in ('x_cm','y_cm','z_cm')], dtype=float)*.01
        return np.array([value[k] for k in ('x','y','z')], dtype=float)
    return np.array(value, dtype=float).T


def episode_budget(bench, target):
    descriptor=next((t for t in bench.TARGETS if t['id']==target),{})
    return int(descriptor.get('agent_tick_budget', BUDGETS.get(getattr(bench,'BENCH_ID',''),2400)))


def robotiq_gap_m(model, data, pads):
    """Measured inner-pad separation, metres, using robot box FK only.

    Project both pad boxes onto their opposing normal bisector. Subtract the
    boxes' support radii from projected centre separation; contact/penetration
    gives zero physical opening. No object pose or joint-angle-to-width fit.
    """
    left,right=pads
    rotations=[data.geom_xmat[g].reshape(3,3) for g in pads]
    axis=rotations[0][:,1]-rotations[1][:,1]
    if np.linalg.norm(axis)<1e-10:
        axis=data.geom_xpos[right]-data.geom_xpos[left]
    norm=float(np.linalg.norm(axis))
    if norm<1e-10:
        return 0.
    axis=axis/norm
    radii=[float(np.dot(model.geom_size[g],np.abs(R.T@axis))) for g,R in zip(pads,rotations)]
    separation=abs(float(np.dot(data.geom_xpos[right]-data.geom_xpos[left],axis)))
    return max(0.,separation-sum(radii))


def _finite(value, shape, name):
    value = np.asarray(value, dtype=float)
    if value.shape != shape or not np.isfinite(value).all():
        raise ValueError(f'{name} must be finite with shape {shape}')
    return value


class MotionController:
    """Stateful, 20 Hz robot tools. Instantiate fresh for each independent target.

    Jaw commands specify TOTAL opening in metres, [-.030, .090]. Negative
    widths request graded Franka squeeze; physical finger joints stay limited.
    On drawers/S3, robot-only position-actuator command ranges are extended for
    the explicitly required squeeze request (R1/R3). Other benches reject it.
    Source mechanism geometry, state and scoring are untouched.
    """
    def __init__(self, task, *, max_ticks=None, finger_tau=.3):
        self.task, self.m, self.d = task, task.model, task.data
        m, d = self.m, self.d
        self.bench_id = getattr(task.bench, 'BENCH_ID', task.module_path.stem)
        self.graded_squeeze = self.bench_id in ('drawers', 'S3_quarter_turn')
        self.max_ticks = episode_budget(task.bench,task.target) if max_ticks is None else int(max_ticks)
        if self.max_ticks < 1 or finger_tau <= 0:
            raise ValueError('Positive episode tick budget and finger filter required')
        ratio = DT / float(m.opt.timestep)
        if not math.isclose(ratio, round(ratio), abs_tol=1e-8):
            raise ValueError('Physics timestep must divide the 50 ms agent tick')
        self.substeps = round(ratio)
        self.finger_tau = float(finger_tau)
        self.ticks, self.reason = 0, None
        self.scratch = mujoco.MjData(m)
        self.joints = [[m.joint(f'{s}_joint{i}').id for i in range(1, 8)] for s in SIDES]
        self.q = [m.jnt_qposadr[j].astype(int) for j in self.joints]
        self.v = [m.jnt_dofadr[j].astype(int) for j in self.joints]
        self.tips = [m.site(f'{s}_tip').id for s in SIDES]
        self.arm_act = []
        for joints in self.joints:
            aids = []
            for joint in joints:
                found = [a for a in range(m.nu) if int(m.actuator_trntype[a]) == int(mujoco.mjtTrn.mjTRN_JOINT)
                         and int(m.actuator_trnid[a, 0]) == joint]
                if len(found) != 1:
                    raise ValueError('Expected one direct motor per Panda arm joint')
                a = found[0]
                if not np.isclose(m.actuator_gear[a, 0], 1.):
                    raise ValueError('Non-unit robot motor gear needs an explicit torque adapter')
                aids.append(a)
            self.arm_act.append(aids)
        self.qt = [d.qpos[q].copy() for q in self.q]
        self.posture = [q.copy() for q in self.qt]
        self.positions = d.site_xpos[self.tips].copy()
        self.rotations = np.array([d.site_xmat[t].reshape(3, 3).copy() for t in self.tips])
        self.grippers = []
        self.jaw = np.zeros(2)
        for a, side in enumerate(SIDES):
            r85 = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f'{side}_r85_fingers_actuator')
            if r85 >= 0:
                pads=[m.geom(f'{side}_r85_{finger}_pad1').id for finger in ('left','right')]
                self.grippers.append(dict(kind='robotiq_2f85', actuators=[r85], q=[],pads=pads))
                # Native Robotiq command: 0 open, 255 closed. Retain build pose.
                self.jaw[a] = .085 * (1 - np.clip(d.ctrl[r85], 0, 255) / 255)
            else:
                aids = [m.actuator(f'{side}_finger_{f}').id for f in 'ab']
                qids = [int(m.jnt_qposadr[m.joint(f'{side}_finger_{f}').id]) for f in 'ab']
                if self.graded_squeeze:
                    for aid in aids:
                        m.actuator_ctrlrange[aid, 0] = min(m.actuator_ctrlrange[aid, 0], -.015)
                self.grippers.append(dict(kind='franka', actuators=aids, q=qids))
                self.jaw[a] = float(d.qpos[qids].sum())
        self.jaw_goal = self.jaw.copy()
        self.workspace = workspace_bounds(task.bench)
        if self.workspace.shape != (3, 2):
            raise ValueError('WORKSPACE must map x,y,z to low/high bounds')
        self.baseline = self.wrenches().copy()
        self.abort_time = np.zeros(2)
        self.link_samples = deque()
        self.metrics = dict(max_total_joint_torque=np.zeros((2, 7)), max_joint_velocity=np.zeros((2, 7)),
                            max_joint_target_rate=np.zeros((2, 7)), max_wrist_force=np.zeros(2),
                            max_link_window_force=0., guard_stops=0)

    def wrenches(self):
        fn = getattr(self.task.bench, 'wrist_wrench', None)
        if fn is not None:
            return _finite(np.asarray(fn(self.m, self.d)).reshape(2, 6), (2, 6), 'wrist wrench')
        result = []
        for side in SIDES:
            R = self.d.site(f'{side}_wrist').xmat.reshape(3, 3)
            result.append(np.r_[R @ self.d.sensor(f'{side}_force').data,
                                R @ self.d.sensor(f'{side}_torque').data])
        return np.array(result)

    def tare(self):
        """User-requested sensor tare; does not advance time or change dynamics."""
        self.baseline = self.wrenches().copy()
        return {'tared': True}

    def instruction(self):
        text = self.task.bench.public_text(self.task.target)
        # Source public text is static; never interpolate params or seed.
        if re.search(r'\bseed\s*[:=]?\s*\d+', text, flags=re.I):
            raise ValueError('Refusing source text that exposes an episode seed')
        return text

    def observe(self, *, images=False, width=320, height=240, camera=None, workspace_view=False):
        """Explicit allowlist of RGB and measured robot proprioception only."""
        measured = []
        for a in range(2):
            R = self.d.site_xmat[self.tips[a]].reshape(3, 3)
            measured.extend(self.d.site_xpos[self.tips[a]])
            measured.extend(Rotation.from_matrix(R).as_euler('xyz'))
            g = self.grippers[a]
            if g['kind'] == 'franka':
                measured.append(float(self.d.qpos[g['q']].sum()))
            else:
                measured.append(robotiq_gap_m(self.m,self.d,g['pads']))
        state = dict(eef_state=np.array(measured), joint_pos=np.concatenate([self.d.qpos[q] for q in self.q]),
                     joint_vel=np.concatenate([self.d.qvel[v] for v in self.v]),
                     wrist_wrench=self.wrenches(), tared_wrist_wrench=self.wrenches()-self.baseline)
        result = dict(state=state, instruction=self.instruction(), time=float(self.d.time),
                      termination_reason=self.reason)
        if images:
            result['images'] = {'overview': self.task.render(width=width, height=height, camera=camera)}
            if workspace_view:
                # Fixed, public workspace framing. No object pose, target,
                # random seed, task state or scene parameter enters this view.
                view = mujoco.MjvCamera()
                view.lookat[:] = self.workspace.mean(axis=1)
                view.distance = max(.7, float(np.ptp(self.workspace,axis=1).max())*1.25)
                view.azimuth, view.elevation = 135., -25.
                result['images']['workspace'] = self.task.render(width=width,height=height,camera=view)
        return result

    def _ik(self, arm, position, rotation, *, down=False):
        m, scratch = self.m, self.scratch
        # Only robot joints enter scratch FK; hidden object coordinates stay at defaults.
        for q in self.q:
            scratch.qpos[q] = self.d.qpos[q]
        q, v, tip = self.q[arm], self.v[arm], self.tips[arm]
        lo, hi = m.jnt_range[self.joints[arm]].T
        jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        rw = .5 if down else .3
        for _ in range(12 if down else 20):
            mujoco.mj_kinematics(m, scratch)
            mujoco.mj_comPos(m, scratch)
            R = scratch.site_xmat[tip].reshape(3, 3)
            err = np.r_[position-scratch.site_xpos[tip], rw*Rotation.from_matrix(rotation@R.T).as_rotvec()]
            if np.linalg.norm(err) < 1e-5:
                break
            mujoco.mj_jacSite(m, scratch, jp, jr, tip)
            J = np.vstack([jp[:, v], rw*jr[:, v]])
            inverse = J.T @ np.linalg.solve(J@J.T+1e-4*np.eye(6), np.eye(6))
            dq = inverse@err + (np.eye(7)-np.linalg.pinv(J)@J)@(.02*(self.posture[arm]-scratch.qpos[q]))
            scratch.qpos[q] = np.clip(scratch.qpos[q]+np.clip(dq, -.08, .08), lo+.01, hi-.01)
        return scratch.qpos[q].copy()

    def _link_forces(self):
        force = np.zeros(6)
        loads = {}
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            names = [self.m.body(int(self.m.geom_bodyid[g])).name or '' for g in (c.geom1, c.geom2)]
            for body, name in zip((int(self.m.geom_bodyid[c.geom1]), int(self.m.geom_bodyid[c.geom2])), names):
                if not name.startswith(('left_', 'right_')) or any(x in name for x in ('finger','pad','driver','coupler','spring','follower')):
                    continue
                # Fixed mounts and bases are not moving arm links.
                if not ('link' in name or 'hand' in name or 'palm' in name or 'r85_base' in name):
                    continue
                mujoco.mj_contactForce(self.m, self.d, i, force)
                loads[body] = loads.get(body, 0.) + float(np.linalg.norm(force[:3]))
        return loads

    def _guard(self, threshold, above):
        dt = float(self.m.opt.timestep)
        w = self.wrenches()
        raw = np.linalg.norm(w[:, :3], axis=1)
        guard = raw.copy()
        for a, g in enumerate(self.grippers):
            if g['kind'] == 'robotiq_2f85':
                guard[a] = np.linalg.norm(w[a, :3]-self.baseline[a, :3])
        above[:] = np.where(guard > threshold, above+dt, 0.)
        self.abort_time = np.where(raw > 80., self.abort_time+dt, 0.)
        self.metrics['max_wrist_force'] = np.maximum(self.metrics['max_wrist_force'], raw)
        now = float(self.d.time)
        self.link_samples.append((now, self._link_forces()))
        while self.link_samples and self.link_samples[0][0] < now-.020+dt/2:
            self.link_samples.popleft()
        totals = {}
        for _, sample in self.link_samples:
            for body, force in sample.items():
                totals[body] = totals.get(body, 0.) + force*dt/.020
        link = max(totals.values(), default=0.)
        self.metrics['max_link_window_force'] = max(self.metrics['max_link_window_force'], link)
        if np.any(self.abort_time >= .100-1e-10):
            self.reason = 'wrist_force_abort'
        elif link > 50.:
            return 'nonfinger_link_guard'
        elif np.any(above >= .020-1e-10):
            return 'wrist_force_guard'
        return self.reason

    def move(self, *, arm, position, rpy_deg=None, yaw_deg=None, jaw_gap_m=None, ticks=1, guard_n=30., _second=None, _hold=False):
        """Command an absolute pose for one arm; other arm retains its target.

        Per tick: <=8 mm and <=.08 rad SO(3) movement from previous COMMAND,
        measured-seed IK, joint-rate interpolation, filtered grippers, time guards.
        A guard ends this tool call and retains achieved commanded pose for retry.
        """
        if self.reason:
            raise RuntimeError(f'Episode ended: {self.reason}')
        if arm not in SIDES:
            raise ValueError('arm must be left or right')
        a = SIDES.index(arm)
        target = _finite(position, (3,), 'position')
        if not _hold and (np.any(target < self.workspace[:, 0]) or np.any(target > self.workspace[:, 1])):
            raise ValueError('Goal is outside the task WORKSPACE')
        if rpy_deg is not None and yaw_deg is not None:
            raise ValueError('Specify either RPY or top-down yaw')
        down = rpy_deg is None and yaw_deg is not None
        if rpy_deg is not None:
            angles = _finite(rpy_deg, (3,), 'rpy_deg')
            if np.any(np.abs(angles) > 180):
                raise ValueError('RPY degrees must be in [-180,180]')
            rotation = Rotation.from_euler('xyz', angles, degrees=True).as_matrix()
        elif yaw_deg is not None:
            if not np.isfinite(yaw_deg) or abs(yaw_deg) > 180:
                raise ValueError('yaw_deg must be in [-180,180]')
            rotation = Rotation.from_euler('z', yaw_deg, degrees=True).as_matrix()@np.diag([1., -1., -1.])
        else:
            rotation = self.rotations[a].copy()
        poses = {a: (target, rotation, down)}
        if _second is not None:
            b = 1-a
            xyz, rpy, gap = _second
            xyz = _finite(xyz, (3,), 'second position')
            rpy = _finite(rpy, (3,), 'second rpy_deg')
            if np.any(xyz < self.workspace[:, 0]) or np.any(xyz > self.workspace[:, 1]):
                raise ValueError('Second goal is outside the task WORKSPACE')
            if np.any(np.abs(rpy) > 180) or not np.isfinite(gap) or not -.030 <= gap <= .090:
                raise ValueError('Second RPY/jaw command exceeds public bounds')
            if gap < 0 and not self.graded_squeeze:
                raise ValueError('This task does not declare negative-jaw squeeze')
            poses[b] = (xyz, Rotation.from_euler('xyz', rpy, degrees=True).as_matrix(), False)
        if not np.isfinite(guard_n) or not 5 <= guard_n <= 60:
            raise ValueError('guard_n must be in [5,60]')
        if not isinstance(ticks, (int, np.integer)) or not 1 <= ticks <= self.max_ticks:
            raise ValueError('ticks must be a positive integer within the episode budget')
        if jaw_gap_m is not None:
            if not np.isfinite(jaw_gap_m) or not -.030 <= jaw_gap_m <= .090:
                raise ValueError('total jaw gap must be in [-.030,.090] m')
            if jaw_gap_m < 0 and not self.graded_squeeze:
                raise ValueError('This task does not declare negative-jaw squeeze')
            self.jaw_goal[a] = jaw_gap_m
        if _second is not None:
            self.jaw_goal[1-a] = _second[2]
        above = np.zeros(2)  # R7 resets at every tool call.
        self.link_samples.clear()
        stopped = None
        start_ticks = self.ticks
        for _ in range(ticks):
            if self.ticks >= self.max_ticks:
                self.reason = 'time_limit'
                break
            for b, (goal_pos, goal_rot, _) in poses.items():
                delta = goal_pos-self.positions[b]
                self.positions[b] += delta*min(1., .008/max(np.linalg.norm(delta), 1e-12))
                rv = Rotation.from_matrix(goal_rot@self.rotations[b].T).as_rotvec()
                self.rotations[b] = Rotation.from_rotvec(rv*min(1., .08/max(np.linalg.norm(rv), 1e-12))).as_matrix()@self.rotations[b]
            qstart = [q.copy() for q in self.qt]
            qend = [self._ik(b, self.positions[b], self.rotations[b], down=poses.get(b, (None, None, False))[2]) for b in range(2)]
            for b in range(2):
                qend[b] = qstart[b]+np.clip(qend[b]-qstart[b], -.85*VELOCITY*DT, .85*VELOCITY*DT)
            for sub in range(self.substeps):
                dt = float(self.m.opt.timestep)
                action = np.zeros(self.m.nu)
                for b in range(2):
                    qnext = qstart[b]+(qend[b]-qstart[b])*(sub+1)/self.substeps
                    self.metrics['max_joint_target_rate'][b] = np.maximum(self.metrics['max_joint_target_rate'][b], np.abs(qnext-self.qt[b])/dt)
                    self.qt[b] = qnext
                    bias = self.d.qfrc_bias[self.v[b]]
                    total = np.clip(550*(qnext-self.d.qpos[self.q[b]])-45*self.d.qvel[self.v[b]]+bias, -TORQUE, TORQUE)
                    action[self.arm_act[b]] = total-bias
                    self.metrics['max_total_joint_torque'][b] = np.maximum(self.metrics['max_total_joint_torque'][b], np.abs(total))
                filtered = (self.jaw_goal-self.jaw)*(-math.expm1(-dt/self.finger_tau))
                # R5: filtering alone can exceed realistic speed for the
                # full open-to-negative-squeeze span. Also bound total gap rate.
                speed = np.array([.05 if g['kind']=='robotiq_2f85' else .15 for g in self.grippers])
                self.jaw += np.clip(filtered,-speed*dt,speed*dt)
                for b, g in enumerate(self.grippers):
                    if g['kind'] == 'franka':
                        action[g['actuators']] = np.clip(self.jaw[b]/2, -.015, .045)
                    else:
                        # Nominal command conversion, not a measured encoder
                        # calibration. Proprioception uses pad geometry instead.
                        action[g['actuators']] = 255*(1-np.clip(self.jaw[b], 0, .085)/.085)
                limited = self.m.actuator_ctrllimited.astype(bool)
                action[limited] = np.clip(action[limited], self.m.actuator_ctrlrange[limited, 0], self.m.actuator_ctrlrange[limited, 1])
                self.task.step(action)
                for b in range(2):
                    self.metrics['max_joint_velocity'][b] = np.maximum(self.metrics['max_joint_velocity'][b], np.abs(self.d.qvel[self.v[b]]))
                stopped = self._guard(guard_n, above)
                if stopped:
                    break
            self.ticks += 1
            if stopped:
                self.metrics['guard_stops'] += 1
                break
        if self.ticks >= self.max_ticks and not self.reason:
            self.reason = 'time_limit'
        return dict(ticks=self.ticks-start_ticks, sim_seconds=float(self.d.time),
                    stop_reason=stopped or self.reason, remaining_ticks=max(0, self.max_ticks-self.ticks))

    def move_pair(self, targets, *, ticks=1, guard_n=30.):
        """Both arms share a single 50 ms control tick; targets=(2,7), XYZ/RPYdeg/gap."""
        values = _finite(targets, (2, 7), 'paired targets')
        return self.move(arm='left', position=values[0, :3], rpy_deg=values[0, 3:6],
                         jaw_gap_m=values[0, 6], ticks=ticks, guard_n=guard_n,
                         _second=(values[1, :3], values[1, 3:6], values[1, 6]))

    def wait(self, *, ticks=1, guard_n=30.):
        """Hold both commanded poses, including a declared above-workspace HOME."""
        return self.move(arm='left',position=self.positions[0].copy(),ticks=ticks,guard_n=guard_n,_hold=True)


def main():
    """Offline JSON-lines tools; no policy client, API token or paid calls."""
    import argparse
    import json
    import os
    import sys
    from .task import NativeTask, jsonable
    ap=argparse.ArgumentParser(description='Public native Panda robot tools (JSON lines on stdin)')
    ap.add_argument('--module',required=True)
    ap.add_argument('--target')
    ap.add_argument('--seed',type=int,default=42,help='Evaluator reset seed; never returned to the agent')
    ap.add_argument('--setting',choices=['A','S','T-new','T-combo'])
    ap.add_argument('--max-ticks',type=int)
    args=ap.parse_args()
    os.environ.setdefault('MUJOCO_GL','osmesa')
    task=NativeTask(args.module,target=args.target,seed=args.seed,setting=args.setting)
    controller=MotionController(task,max_ticks=args.max_ticks)
    tools={'move':controller.move,'move_pair':controller.move_pair,'observe':controller.observe,
           'wait':controller.wait,'tare':controller.tare}
    try:
        for line in sys.stdin:
            try:
                request=json.loads(line)
                name=request['tool']; kwargs=request.get('arguments',{})
                if name not in tools or any(k.startswith('_') for k in kwargs):
                    raise ValueError('Unknown public tool or private argument')
                if kwargs.get('images'):
                    raise ValueError('RGB is provided by the Inspect adapter; JSON tools return robot state only')
                result={'ok':True,'result':tools[name](**kwargs)}
            except (ValueError,KeyError,TypeError,RuntimeError) as error:
                result={'ok':False,'error':str(error)}
            print(json.dumps(result,default=jsonable,allow_nan=False),flush=True)
    finally:
        task.close()


if __name__=='__main__':
    main()
