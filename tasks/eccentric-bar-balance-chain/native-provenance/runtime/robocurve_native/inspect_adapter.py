"""Inspect Robots 1.x interface for native dual 7-DOF Panda tasks.

This generic embodiment is for a compatible Inspect policy, not Dream-1/YAM.
Source-native scoring remains evaluator-only; observations are RGB and an
explicit robot-proprioception allowlist. Each target starts a fresh episode.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from inspect_robots.embodiment import EmbodimentInfo
from inspect_robots.registry import embodiment, scorer, task as register_task
from inspect_robots.scene import Scene
from inspect_robots.scorer import Score
from inspect_robots.spaces import ActionSemantics, Box, CameraSpec, ObservationSpace, StateField, StateSpec
from inspect_robots.task import Task
from inspect_robots.types import Observation, StepResult

from .motion import MotionController, workspace_bounds, episode_budget
from .task import NativeTask, load


def action_space(workspace, *, squeeze=False):
    labels = tuple(f'{side}_{key}' for side in ('left','right') for key in ('x','y','z','roll','pitch','yaw','jaw_gap'))
    return Box(shape=(14,),
               low=np.tile(np.r_[workspace[:,0], [-math.pi]*3, -.030 if squeeze else 0.], 2),
               high=np.tile(np.r_[workspace[:,1], [math.pi]*3, .090], 2),
               semantics=ActionSemantics(control_mode='eef_abs_pose', rotation_repr='euler_xyz',
                                         gripper='continuous', frame='world', dim_labels=labels,
                                         max_step=tuple([.008]*3+[2*math.pi]*3+[.09])*2))


class NativePandaEmbodiment:
    def __init__(self, module, *, target=None, setting=None, resolution=320, max_ticks=None, camera=None, guard_n=30.):
        self.module = Path(module).resolve()
        self.source = load(self.module)
        self.target = target or getattr(self.source,'TASK_A',self.source.TARGETS[0]['id'])
        self.setting, self.camera = setting, camera
        self.guard_n = float(guard_n)
        if not np.isfinite(self.guard_n) or not 5 <= self.guard_n <= 60:
            raise ValueError('guard_n must be in [5,60]')
        self.resolution = int(resolution)
        if not 64 <= self.resolution <= 640:
            raise ValueError('resolution must be in 64..640')
        bench = getattr(self.source,'BENCH_ID',self.module.stem)
        self.max_ticks = episode_budget(self.source,self.target) if max_ticks is None else int(max_ticks)
        self.native = self.controller = None
        self.info = EmbodimentInfo(name='native_panda', morphology='dual-panda-7dof',
            action_space=action_space(workspace_bounds(self.source), squeeze=bench in ('drawers','S3_quarter_turn')),
            control_hz=20., is_simulated=True,
            observation_space=ObservationSpace(cameras=tuple(CameraSpec(name,self.resolution,self.resolution) for name in ('overview','workspace')),
                state=StateSpec(fields=(
                    StateField('eef_state',(14,), 'left/right XYZ metres, XYZ Euler radians, total measured inner-pad jaw gap metres for both Franka and Robotiq'),
                    StateField('joint_pos',(14,),'rad'), StateField('joint_vel',(14,),'rad/s'),
                    StateField('wrist_wrench',(2,6),'world N and N.m'),
                    StateField('tared_wrist_wrench',(2,6),'world N and N.m relative to reset/user tare'),
                ))),
            capabilities=frozenset({'resettable','auto_reset','privileged_success','renderable'}),
            environment_id=f'robocurve-native-panda-{bench}-v1',
            docs='Two 7-DOF Panda arms. Actions: left XYZ metres, fixed-axis XYZ Euler radians, total jaw gap metres, then right likewise. '
                 'The tool point is the source tip site. Local +Z points along the fingers; roll=pi,pitch=yaw=0 points down. '
                 'Both arms advance together at 20 Hz, from previous commanded targets, at most 8 mm/.08 rad per tick. '
                 f'Other robot tools provide wrist tare and per-move guards; Inspect actions use the configured {self.guard_n:g} N guard. '
                 'Negative jaw width is available only on tasks declaring graded squeeze; it requests finger force, not negative physical separation. '
                 'Robotiq measured width uses robot pad geometry; its width-to-actuator command is a nominal linear conversion. '
                 'Images and measured robot proprioception only. No object poses, physics model, random seed or hidden parameters are visible. '
                 'This Panda embodiment is incompatible with the 6-DOF YAM Dream-1 interface.')

    def bind_task(self, envelope):
        self.max_ticks = min(self.max_ticks, envelope.max_steps)

    def reset(self, scene, *, seed=None):
        self.close()
        # The evaluator owns this seed. It is never inserted into observation text.
        chosen_seed = seed if seed is not None else getattr(scene,'init_seed',42)
        if chosen_seed is None:
            chosen_seed = 42
        self.native = NativeTask(self.module, seed=chosen_seed, target=self.target, setting=self.setting)
        self.controller = MotionController(self.native,max_ticks=self.max_ticks)
        return self.observe()

    def observe(self):
        if self.controller is None:
            raise RuntimeError('Reset the embodiment first')
        value = self.controller.observe(images=True,width=self.resolution,height=self.resolution,camera=self.camera,workspace_view=True)
        return Observation(images=value['images'],state=value['state'],instruction=value['instruction'],
                           state_time=value['time'],image_times={name:value['time'] for name in value['images']})

    def step(self, action):
        values = np.array(action.data,dtype=float,copy=True)
        space = self.info.action_space
        if values.shape != (14,) or not np.isfinite(values).all() or np.any(values<space.low) or np.any(values>space.high):
            raise ValueError('Expected 14 finite absolute pose targets in the declared space')
        poses = values.reshape(2,7)
        poses[:,3:6] = np.degrees(poses[:,3:6])
        result = self.controller.move_pair(poses,guard_n=self.guard_n)
        reason = self.controller.reason
        # Evaluator-only record info. It is never copied into Observation.state.
        status = self.native.status()
        if status['disallowed_damage'] and not reason:
            reason = self.controller.reason = 'source_damage'
        return StepResult(observation=self.observe(),terminated=reason is not None and reason!='time_limit',
                          truncated=reason=='time_limit',termination_reason=reason,
                          info={'native_score':status['score'],'control_stop':result['stop_reason']})

    def close(self):
        if self.native is not None:
            self.native.close()
        self.native = self.controller = None


class NativePandaScorer:
    name = 'native_panda_score'

    def __call__(self, record, target):
        values = [float(s.result.info.get('native_score',0.)) for s in record.steps]
        # Any subsequent disallowed damage makes native_score zero; final source
        # state therefore governs the score rather than max-over-time recovery.
        return Score(value=values[-1] if values else 0., metadata={'evidence':'native source scorer plus package prefix-hold partial credit when present; not reviewer acceptance'})


@embodiment('native_panda')
def make_embodiment(module, target=None, setting=None, resolution=320, max_ticks=None, camera=None, guard_n=30.):
    return NativePandaEmbodiment(module,target=target,setting=setting,resolution=resolution,max_ticks=max_ticks,camera=camera,guard_n=guard_n)


native_embodiment = make_embodiment


@scorer('native_panda_score')
def make_scorer():
    return NativePandaScorer()


@register_task('native-panda')
def make_task(module, target=None, setting='S', max_ticks=None, seed=42):
    source = load(module)
    goal = target or getattr(source,'TASK_A',source.TARGETS[0]['id'])
    bench = getattr(source,'BENCH_ID',Path(module).stem)
    limit = episode_budget(source,goal) if max_ticks is None else int(max_ticks)
    return Task(name='native-panda',scenes=[Scene(id='fixed-layout',instruction=source.public_text(goal),init_seed=seed)],
                scorer=NativePandaScorer(),max_steps=limit)


if __name__ == '__main__':
    from inspect_robots.cli import main
    main()
