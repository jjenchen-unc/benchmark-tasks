"""Enclosure with an adjustable compression latch for yam_arena task files.

Spawns the fixture from latch.usda and runs its passive mechanics every 2 ms physics step: the gasket spring,
the cam that the handle turns, the threaded grip-length adjuster that the knob turns (it locks once the pawl
bears on the flange), and permanent cam stripping when the cam is overloaded. It also records, every physics
step, the quantities a judge can see or measure: handle angle, knob turns, lid-to-flange gap and whether any
robot part touches the enclosure. Robots only send the standard 14 joint targets; nothing here moves a part
except through these passive mechanics.
"""
import importlib.abc, importlib.util, math, sys
from dataclasses import dataclass
from pathlib import Path
import numpy as np
from yam_arena.task_file import Box, ObjectState

HERE = Path(__file__).resolve().parent
POS = (0.28, 0.0, 0.7475)                    # fixture origin on the table (m)
YAW = math.radians(-105)
BASE_RISE = 0.15                             # both arm bases stand on 150 mm supports
BASE_MESH_BOTTOM = -0.001101112
G0 = 0.0068                                  # uncompressed gasket thickness (m)
K = 33416.5                                  # gasket stiffness (N/m)
GRIP = 0.001942                              # hidden: grip length offset of this latch (m)
TH_OC = math.radians(172)                    # cam over-centre angle
LEAD = .0064 / (TH_OC - math.pi / 2)         # 6.4 mm cam pull from 90 to 172 degrees
REV = .00015 / (math.pi - TH_OC)             # 0.15 mm back-off from 172 to 180 degrees
PITCH = .001 / (2 * math.pi)                 # 1 mm per knob turn
DT = .002
STRIP_NM, STRIP_S = 3.0, 0.1                 # cam strips after 3 N m held for 0.1 s


def cam(th, kn):
    a0 = -LEAD * math.pi / 2 - GRIP + PITCH * kn
    slope = LEAD
    if th > TH_OC:
        a0 += (LEAD + REV) * TH_OC
        slope = -REV
    return a0 + slope * th, slope


class Mechanism:
    def __init__(self):
        self.lock = None; self.stripped = False; self.over = 0; self.force = 0.; self.torque = 0.

    def efforts(self, q, v, clamp, dt):
        # Knob screw locks while the pawl bears on the flange (above 2 N), frees below 0.5 N.
        if clamp > 2 and self.lock is None: self.lock = q['knob']
        if clamp < .5: self.lock = None
        kn = q['knob'] if self.lock is None else self.lock
        c, s = cam(q['handle'], kn); e = q['pawl'] - c
        J = np.array([1., -s, -PITCH if self.lock is None else 0.])
        vel = np.array([v['pawl'], v['handle'], v['knob']]); inv = np.array([1 / 250.03, 1 / .00502, 1 / .005004])
        kc, dc = 1e7, 1e4
        self.force = 0. if self.stripped else -(kc * e + (dc + kc * dt) * float(J @ vel)) / (1 + dt * (dc + kc * dt) * float((J * J) @ inv))
        self.torque = abs(self.force * s)
        self.over = self.over + 1 if self.torque > STRIP_NM else 0
        if self.over * dt >= STRIP_S - 1e-12: self.stripped = True; self.force = 0.; self.torque = 0.
        out = dict(zip(('pawl', 'handle', 'knob'), self.force * J))
        out['lid_slide'] = -8 * K * max(0., q['lid_slide'] - (G0 - .0005))   # gasket bottoms out over its last 0.5 mm
        if self.lock is not None:
            k, d = 1e5, 100.; iv = 1 / .005004
            out['knob'] += -(k * (q['knob'] - self.lock) + (d + k * dt) * v['knob']) / (1 + dt * (d + k * dt) * iv)
        return out


def arr(x):
    x = getattr(x, 'torch', x)
    return x.detach().cpu().numpy() if hasattr(x, 'detach') else np.asarray(x)


class Contacts:
    def __init__(self, view, sensors, filters):
        self.cap = 8192
        self.view = view.create_rigid_contact_view(sensors, filter_patterns=[filters] * len(sensors), max_contact_data_count=self.cap)

    def read(self, dt):
        """Contacts as (separation m, normal force N) pairs."""
        force, _, _, sep, count, start = self.view.get_contact_data(dt)
        count = arr(count).astype(int).ravel(); start = arr(start).astype(int).ravel()
        if count.sum() >= self.cap: raise ValueError('contact buffer full')
        f = arr(force).reshape(-1); d = arr(sep).reshape(-1)
        ids = [i for n, s in zip(count, start) for i in range(s, s + n)]
        return [(float(d[i]), float(f[i])) for i in ids]


# Episode record, replaced at every reset. samples: one (handle rad, knob rad, gap m, touch) per physics step,
# or None if the state could not be read. diag: internal values for the validation script only.
EPISODE = {'samples': [], 'diag': {}}
_LIVE = {'sim': None}


class Sim:
    def __init__(self, emb):
        from isaaclab_physx.physics import PhysxManager
        base = emb._base; self.asset = base.scene['compression_latch']; self.mech = Mechanism(); self.clamp = 0.; self.error = None
        paths = list(self.asset.root_view.link_paths[0])
        robots = [p for side in ('left', 'right') for p in base.scene[side + '_robot'].root_view.link_paths[0]]
        view = PhysxManager.get_physics_sim_view()
        self.touch = Contacts(view, paths, robots)
        self.pawl = Contacts(view, [p for p in paths if p.endswith('/pawl')], [p for p in paths if p.endswith('/enclosure')])

    def joints(self):
        n = self.asset.joint_names
        return dict(zip(n, arr(self.asset.data.joint_pos)[0])), dict(zip(n, arr(self.asset.data.joint_vel)[0]))

    def apply(self):
        if self.error: return
        import torch
        try:
            q, v = self.joints(); eff = self.mech.efforts(q, v, self.clamp, DT)
            t = torch.tensor([[eff[n] for n in self.asset.joint_names]], dtype=self.asset.data.joint_pos.torch.dtype, device=self.asset.data.joint_pos.torch.device)
            if not torch.isfinite(t).all(): raise ValueError('nonfinite effort')
            self.asset.set_joint_effort_target(t)
        except Exception as exc: self.error = repr(exc)

    def sample(self, dt):
        rec = None
        try:
            if self.error: raise RuntimeError(self.error)
            q, _ = self.joints()
            touch = any(d <= 1e-4 for d, _ in self.touch.read(dt))     # a robot body within 0.1 mm of a latch part
            self.clamp = sum(abs(f) for _, f in self.pawl.read(dt))
            rec = (float(q['handle']), float(q['knob']), G0 - float(q['lid_slide']), touch)
            if not all(map(math.isfinite, rec[:3])): rec = None
            g = EPISODE['diag']
            g['peak_torque'] = max(g.get('peak_torque', 0.), self.mech.torque); g['stripped'] = self.mech.stripped
            g['clamp'] = self.clamp; g['cam_force'] = self.mech.force
        except Exception as exc: self.error = repr(exc)
        EPISODE['samples'].append(rec)


@dataclass(frozen=True)
class Latch(Box): pass


def _spawner():
    import isaaclab.sim as sim
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    def spawn(prim_path, cfg, translation=None, orientation=None, **kw):
        from pxr import UsdPhysics, Sdf
        cfg = sim.UsdFileCfg(usd_path=str(HERE / 'latch.usda'), activate_contact_sensors=True)
        prim = cfg.func(prim_path, cfg, translation=translation, orientation=orientation, **kw)
        # The pawl sits under the flange and touches only the flange.
        env = prim_path.rsplit('/', 1)[0]
        rel = UsdPhysics.FilteredPairsAPI.Apply(prim.GetStage().GetPrimAtPath(prim_path + '/links/pawl/pawl_tip')).CreateFilteredPairsRel()
        for name in ('LeftArm', 'RightArm', 'floor', 'table', 'stand_foot', 'stand_mast', 'stand_arm', 'camera_housing', 'lamp', 'left_robot_pedestal', 'right_robot_pedestal'):
            rel.AddTarget(Sdf.Path(env + '/' + name))
        return prim

    @configclass
    class SpawnCfg(SpawnerCfg): func: object = clone(spawn)

    class Part(Object):
        def _generate_articulation_cfg(self):
            acts = {n: ImplicitActuatorCfg(joint_names_expr=[n], stiffness=k, damping=d) for n, k, d in [('lid_slide', K, 200.), ('handle', 0., .01), ('knob', 0., .02), ('pawl', 0., 5.)]}
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=acts, init_state=ArticulationCfg.InitialStateCfg(joint_pos={'.*': 0.}, joint_vel={'.*': 0.}))
            return self._add_initial_pose_to_cfg(cfg)
    return Part(name='compression_latch', object_type=ObjectType.ARTICULATION, spawner_cfg=SpawnCfg(), initial_pose=Pose(position_xyz=POS, rotation_xyzw=(0., 0., math.sin(YAW / 2), math.cos(YAW / 2))))


def _patch_scene(m):
    if getattr(m, '_latch_patched', False): return
    old_object, old_states, old_build = m.task_object, m.object_states, m.build_task_environment

    def task_object(spec):
        return _spawner() if isinstance(spec, Latch) else old_object(spec)

    def object_states(env, names):
        out = old_states(env, [n for n in names if n != 'compression_latch']) if any(n != 'compression_latch' for n in names) else {}
        if 'compression_latch' in names:
            c, s = math.cos(YAW), math.sin(YAW); R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])
            pts = np.array([[x, y, z] for x in (-.15, .162) for y in (-.2, .2) for z in (0., .21)]) @ R.T + POS
            out['compression_latch'] = ObjectState(pos=np.array(POS), quat_xyzw=np.array([0., 0., 0., 1.]), lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=pts.min(0), high=pts.max(0))
        return {n: out[n] for n in names}

    def build(*a, **kw):
        env = old_build(*a, **kw); task = kw.get('task')
        if task is not None and any(isinstance(o, Latch) for o in task.objects):
            prev = env.env_cfg_callback

            def configure(cfg):
                if prev is not None: cfg = prev(cfg)
                from isaaclab.sensors import ContactSensorCfg
                from yam_arena.station import BASES
                for side in ('left', 'right'):
                    robot = getattr(cfg.scene, side + '_robot'); robot.spawn.activate_contact_sensors = True
                    robot.init_state.pos = tuple(np.asarray(BASES[side]) + [0., 0., BASE_RISE])
                cfg.scene.latch_contacts = ContactSensorCfg(prim_path='{ENV_REGEX_NS}/compression_latch/links/lid')
                cfg.sim.dt = DT; cfg.decimation = 25; cfg.sim.render_interval = 25
                return cfg
            env.env_cfg_callback = configure
        return env
    m.task_object, m.object_states, m.build_task_environment = task_object, object_states, build; m._latch_patched = True


def _patch_embodiment(m):
    cls = m.YamArenaEmbodiment
    if getattr(cls, '_latch_patched', False): return
    old_reset = cls.reset

    def reset(self, *a, **kw):
        if self.task is None or not any(isinstance(o, Latch) for o in self.task.objects): return old_reset(self, *a, **kw)
        if self.object_jitter != 0: raise ValueError('this task needs object_jitter=0')
        if not math.isclose(float(self.info.control_hz), 20.): raise ValueError('this task needs control_hz=20')
        if str(self.device) != 'cpu': raise ValueError('this task needs device=cpu (CPU PhysX)')
        _LIVE['sim'] = None
        self._ensure_sim(None)
        import torch
        asset = self._base.scene['compression_latch']; q = torch.zeros_like(asset.data.joint_pos.torch)
        q[:, asset.joint_names.index('pawl')] = cam(0, 0)[0]
        q[:, asset.joint_names.index('lid_slide')] = 9.81 * 1.6 / K
        asset.write_joint_state_to_sim(q, torch.zeros_like(q)); asset.set_joint_effort_target(torch.zeros_like(q)); asset.set_joint_position_target(torch.zeros_like(q))
        _LIVE['sim'] = Sim(self)
        if not getattr(asset, '_latch_hook', False):
            write = asset.write_data_to_sim

            def hooked_write(*args, **kwargs):
                if _LIVE['sim'] is not None: _LIVE['sim'].apply()
                return write(*args, **kwargs)
            asset.write_data_to_sim = hooked_write; asset._latch_hook = True
        scene = self._base.scene
        if not getattr(scene, '_latch_hook', False):
            update = scene.update

            def hooked_update(dt, *args, **kwargs):
                r = update(dt, *args, **kwargs)
                if _LIVE['sim'] is not None: _LIVE['sim'].sample(float(dt))
                return r
            scene.update = hooked_update; scene._latch_hook = True
        obs = old_reset(self, *a, **kw)
        EPISODE['samples'] = []; EPISODE['diag'] = {}    # settling after reset earns nothing
        return obs
    cls.reset = reset; cls._latch_patched = True


class _Finder(importlib.abc.MetaPathFinder):
    targets = {'yam_arena.scene': _patch_scene, 'yam_arena.inspect_embodiment': _patch_embodiment}

    def find_spec(self, name, path, target=None):
        if name not in self.targets: return None
        sys.meta_path.remove(self)
        try: spec = importlib.util.find_spec(name)
        finally: sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None: return None
        run, patch = spec.loader.exec_module, self.targets[name]
        spec.loader.exec_module = lambda mod: (run(mod), patch(mod))
        return spec


def install():
    for name, patch in _Finder.targets.items():
        if name in sys.modules: patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path): sys.meta_path.insert(0, _Finder())
