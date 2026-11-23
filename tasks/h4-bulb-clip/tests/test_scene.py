"""Pre-simulator wiring check. Requires yam_arena on PYTHONPATH, with no Isaac app."""
from pathlib import Path
from types import SimpleNamespace
import sys
from unittest.mock import Mock, patch

import numpy as np
from yam_arena.task_file import load

task = load(Path(__file__).resolve().parents[1] / "isaac" / "task.py")
import h4kit  # installed by the task loader
from yam_arena import station

# The pinned task file and rig are real. Stand-ins record the Arena scene configuration.
modules = {
    name: Mock() for name in (
        "isaaclab", "isaaclab.sim", "isaaclab_arena.assets.object",
        "isaaclab_arena.assets.object_library", "isaaclab_arena.assets.object_type",
        "isaaclab_arena.utils.pose", "isaaclab_physx.sim.schemas",
        "isaaclab_arena.environments.isaaclab_arena_environment", "isaaclab_arena.scene.scene",
    )
}
modules["isaaclab"].sim = modules["isaaclab.sim"]
object_ctor = modules["isaaclab_arena.assets.object"].Object
object_ctor.side_effect = lambda **kw: SimpleNamespace(**kw)
modules["isaaclab_arena.utils.pose"].Pose.side_effect = lambda **kw: SimpleNamespace(**kw)
modules["isaaclab.sim"].CuboidCfg.side_effect = lambda **kw: SimpleNamespace(**kw)
modules["isaaclab_arena.scene.scene"].Scene.side_effect = lambda assets: assets
modules["isaaclab_arena.environments.isaaclab_arena_environment"].IsaacLabArenaEnvironment.side_effect = (
    lambda **kw: SimpleNamespace(**kw)
)
scene = SimpleNamespace(task_object=Mock(), object_states=Mock(), build_task_environment=Mock(),
                        env_cfg_callback=lambda _: lambda cfg: cfg)
default_builder = scene.build_task_environment
rig = station.rig()
with patch.dict(sys.modules, modules), patch.object(h4kit, "_arena_object", return_value=task.objects[0]):
    h4kit._patch_scene(scene)
    environment = scene.build_task_environment(SimpleNamespace(rig=rig), task, 20)
    assert not default_builder.called
    assert [asset.name for asset in environment.scene[1:]] == ["table", "floor", "camera_shell", "bench"]
    table, _, housing = environment.scene[1:4]
    assert np.isclose(table.initial_pose.position_xyz[2] + table.spawner_cfg.size[2] / 2, h4kit.L.TOP)
    assert housing.initial_pose.rotation_xyzw == rig.top_cam.rot_xyzw
    camera_offset = np.array(housing.initial_pose.position_xyz) - np.array(rig.top_cam.pos)
    assert np.isclose(camera_offset @ h4kit._rot(rig.top_cam.rot_xyzw)[:, 2], 0.032)
    stock = Mock()
    spawn = SimpleNamespace(func=stock)
    cfg = SimpleNamespace(scene=SimpleNamespace(left_robot=SimpleNamespace(spawn=spawn)),
                          sim=SimpleNamespace(physics=SimpleNamespace(enable_ccd=False)))
    cfg = environment.env_cfg_callback(cfg)
    assert spawn.func is not stock
    with patch.dict(sys.modules, {"tool_mount": Mock()}) as fake:
        assert spawn.func("left", spawn) is stock.return_value
        fake["tool_mount"].install.assert_called_once_with(stock.return_value)
    assert cfg.sim.dt == 1/480 and cfg.decimation == 24 and cfg.sim.physics.enable_ccd
    other_task = SimpleNamespace(objects=[])
    scene.build_task_environment(None, other_task, 20)
    default_builder.assert_called_once_with(None, other_task, 20)
print("PASS official task-file loader, H4 room dispatch, table height, calibrated camera shell, physics config and unrelated task delegation (Arena config stand-ins)")
