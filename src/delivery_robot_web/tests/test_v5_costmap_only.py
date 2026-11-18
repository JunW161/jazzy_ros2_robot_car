from pathlib import Path
from types import SimpleNamespace as NS

from delivery_robot_web.core import Store, MotionLease
from delivery_robot_web.processes import ProcessManager


def make_store(tmp_path):
    s = Store(tmp_path / 'state')
    s.set_admin('admin', 'robot-test-password-2026')
    return s


def healthy_ros(store):
    motion = MotionLease(store.config)
    state = {
        'nodes': ['/amcl'],
        'tf_odom': {'age': 0.02},
        'tf_map': {'age': 0.02, 'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        'scan_publishers': 1,
        'scan': {'receive_age': 0.01},
        'tf_odom_laser': True,
        'global_costmap_publishers': 0,
        'local_costmap_publishers': 0,
    }
    return NS(status=lambda: state, maintenance=False, motion=motion), state


def test_v52_costmap_start_uses_verified_launch_and_auto_active(tmp_path, monkeypatch):
    store = make_store(tmp_path)
    ros, _ = healthy_ros(store)
    pm = ProcessManager(store, ros, offline=False)
    ws = tmp_path / 'ros2_ws'
    launch_file = ws / 'install/delivery_robot_nav2/share/delivery_robot_nav2/launch/costmaps_only.launch.py'
    launch_file.parent.mkdir(parents=True)
    launch_file.write_text('# launch')
    (ws / 'install/setup.bash').write_text('# overlay')
    store.config['ros_workspace'] = str(ws)

    monkeypatch.setattr(pm, 'recoverable_costmap_processes', lambda: [])
    captured = {}

    class FakeProc:
        pid = 1234
        def poll(self): return None

    class FakeHandle:
        def close(self): pass

    def fake_spawn(name, command, cwd=None, context=None):
        captured.update(name=name, command=command, context=context)
        pm.children[name] = {
            'process': FakeProc(), 'log': tmp_path / 'x.log', 'handle': FakeHandle(),
            'command': command, 'context': dict(context or {})
        }
        return {'message': 'spawned'}

    monkeypatch.setattr(pm, '_spawn_shell', fake_spawn)
    monkeypatch.setattr(pm, '_ensure_costmaps_active', lambda: {
        '/global_costmap/global_costmap': 'active [3]',
        '/local_costmap/local_costmap': 'active [3]',
    })

    out = pm.start('costmap')
    assert captured['name'] == 'costmap'
    assert 'ros2 launch delivery_robot_nav2 costmaps_only.launch.py' in captured['command']
    assert 'ros2 run nav2_costmap_2d nav2_costmap_2d' not in captured['command']
    assert captured['context']['startup'] == 'costmaps_only.launch.py'
    assert 'ACTIVE' in out['message']


def test_v5_existing_exact_costmap_is_adopted_not_duplicated(tmp_path, monkeypatch):
    store = make_store(tmp_path)
    ros, _ = healthy_ros(store)
    pm = ProcessManager(store, ros, offline=False)
    monkeypatch.setattr(pm, 'recoverable_costmap_processes', lambda: [
        {'pid': 4567, 'cmdline': '/opt/ros/jazzy/lib/nav2_costmap_2d/nav2_costmap_2d --params-file /home/tuan/ros2_ws/install/delivery_robot_nav2/share/delivery_robot_nav2/config/costmaps_v1.yaml'}
    ])
    monkeypatch.setattr(pm, '_ensure_costmaps_active', lambda: {
        '/global_costmap/global_costmap': 'active [3]',
        '/local_costmap/local_costmap': 'active [3]',
    })
    monkeypatch.setattr(pm, '_spawn_shell', lambda *a, **k: (_ for _ in ()).throw(AssertionError('must not spawn duplicate')))
    out = pm.start('costmap')
    assert 'đã tồn tại' in out['message']
    assert 'ACTIVE' in out['message']


def test_v5_external_exact_costmap_visible_even_before_node_discovery(tmp_path, monkeypatch):
    store = make_store(tmp_path)
    ros, state = healthy_ros(store)
    state['nodes'] = ['/amcl']
    pm = ProcessManager(store, ros, offline=False)
    monkeypatch.setattr(pm.agent, 'snapshot', lambda *a, **k: {'running': False, 'installed': False, 'message': ''})
    monkeypatch.setattr(pm, 'udp_in_use', lambda _p: False)
    monkeypatch.setattr(pm, 'recoverable_costmap_processes', lambda: [
        {'pid': 7777, 'cmdline': '/opt/ros/jazzy/lib/nav2_costmap_2d/nav2_costmap_2d --params-file /home/tuan/ros2_ws/install/delivery_robot_nav2/share/delivery_robot_nav2/config/costmaps_v1.yaml'}
    ])
    st = pm.status()['costmap']
    assert st['running'] is True
    assert st['external'] is True
    assert st['recoverable'] is True
    assert st['pid'] == 7777


def test_v5_rviz_layout_is_original_v26_without_costmap_displays():
    rviz = Path(__file__).resolve().parents[1] / 'delivery_robot_web' / 'rviz' / 'localization.rviz'
    text = rviz.read_text()
    assert '/global_costmap/costmap' not in text
    assert '/local_costmap/costmap' not in text
    assert 'Value: /map' in text
    assert 'Value: /scan' in text


def test_v5_amcl_start_contract_remains_v26():
    src = (Path(__file__).resolve().parents[1] / 'delivery_robot_web' / 'processes.py').read_text()
    # These are the V2.6 AMCL ownership/safety contracts.  V5 Costmap changes
    # must not replace them with a new localization workflow.
    assert "exec ros2 launch delivery_robot_localization_v14 amcl_localization.launch.py" in src
    assert "AMCL/Map Server đang chạy ngoài web. Dừng Terminal AMCL cũ trước khi chuyển quyền cho web." in src
    assert "recoverable_localization_processes" in src


def test_v52_costmap_command_is_exact_verified_terminal_launch(tmp_path, monkeypatch):
    store = make_store(tmp_path)
    ros, _ = healthy_ros(store)
    pm = ProcessManager(store, ros, offline=False)
    ws = tmp_path / 'ros2_ws'
    launch_file = ws / 'install/delivery_robot_nav2/share/delivery_robot_nav2/launch/costmaps_only.launch.py'
    launch_file.parent.mkdir(parents=True)
    launch_file.write_text('# launch')
    (ws / 'install/setup.bash').write_text('# overlay')
    store.config['ros_workspace'] = str(ws)
    monkeypatch.setattr(pm, 'recoverable_costmap_processes', lambda: [])
    captured = {}
    class FakeProc:
        pid = 55
        def poll(self): return None
    class FakeHandle:
        def close(self): pass
    def spawn(name, command, cwd=None, context=None):
        captured['command'] = command
        pm.children[name] = {'process': FakeProc(), 'log': tmp_path/'x.log', 'handle': FakeHandle(), 'command': command, 'context': dict(context or {})}
        return {'message': 'spawned'}
    monkeypatch.setattr(pm, '_spawn_shell', spawn)
    monkeypatch.setattr(pm, '_ensure_costmaps_active', lambda: {
        '/global_costmap/global_costmap': 'active [3]',
        '/local_costmap/local_costmap': 'active [3]',
    })
    pm.start('costmap')
    assert captured['command'].splitlines()[-1] == 'exec ros2 launch delivery_robot_nav2 costmaps_only.launch.py'
