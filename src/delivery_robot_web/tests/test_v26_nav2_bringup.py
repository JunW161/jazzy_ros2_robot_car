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
    }
    return NS(status=lambda: state, maintenance=False, motion=motion), state


def test_v26_ui_has_costmap_controls():
    root = Path(__file__).resolve().parents[1] / 'delivery_robot_web' / 'static'
    html = (root / 'index.html').read_text()
    js = (root / 'app.js').read_text()
    for token in ('id="startCostmap"', 'id="stopCostmap"',
                  'id="globalCostmapWebState"', 'id="localCostmapWebState"'):
        assert token in html
    assert "/api/process/costmap/start" in js
    assert "/api/process/costmap/stop" in js


def test_orphan_project_amcl_is_recoverable_without_fixed_pid(monkeypatch):
    launch = {'pid': 111, 'cmdline': '/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch delivery_robot_localization_v14 amcl_localization.launch.py map:=/tmp/x.yaml'}
    orphan = {'pid': 98765, 'cmdline': '/opt/ros/jazzy/lib/nav2_amcl/amcl --ros-args -r __node:=amcl --params-file /home/tuan/ros2_ws/install/delivery_robot_localization_v14/share/delivery_robot_localization_v14/config/amcl.yaml --params-file /tmp/launch_params_x'}
    other = {'pid': 222, 'cmdline': '/opt/ros/jazzy/lib/nav2_amcl/amcl --ros-args -r __node:=amcl --params-file /tmp/other.yaml'}

    def fake_matches(cls, needle):
        if 'ros2 launch delivery_robot_localization_v14' in needle:
            return [launch]
        if needle == '/opt/ros/jazzy/lib/nav2_amcl/amcl':
            return [orphan, other]
        return []

    monkeypatch.setattr(ProcessManager, 'local_process_matches', classmethod(fake_matches))
    got = ProcessManager.recoverable_localization_processes()
    assert {x['pid'] for x in got} == {111, 98765}


def test_costmap_start_uses_verified_launch_and_web_verification(tmp_path, monkeypatch):
    store = make_store(tmp_path)
    ros, state = healthy_ros(store)
    state['global_costmap_publishers'] = 0
    state['local_costmap_publishers'] = 0
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
    assert 'ros2 launch delivery_robot_nav2 costmaps_only.launch.py' in captured['command']
    assert 'ros2 run nav2_costmap_2d nav2_costmap_2d' not in captured['command']
    assert captured['context']['startup'] == 'costmaps_only.launch.py'
    assert 'ACTIVE' in out['message']

def test_costmap_status_recovers_exact_project_children(tmp_path, monkeypatch):
    store = make_store(tmp_path)
    ros, state = healthy_ros(store)
    state['nodes'] += ['/global_costmap/global_costmap', '/local_costmap/local_costmap']
    state['global_costmap_publishers'] = 1
    state['local_costmap_publishers'] = 1
    pm = ProcessManager(store, ros, offline=False)
    monkeypatch.setattr(pm.agent, 'snapshot', lambda *a, **k: {'running': False, 'installed': False, 'message': ''})
    monkeypatch.setattr(pm, 'udp_in_use', lambda _p: False)
    monkeypatch.setattr(pm, 'recoverable_costmap_processes', lambda: [
        {'pid': 4567, 'cmdline': '/opt/ros/jazzy/lib/nav2_costmap_2d/nav2_costmap_2d --params-file /home/tuan/ros2_ws/install/delivery_robot_nav2/share/delivery_robot_nav2/config/costmaps_v1.yaml'}
    ])
    st = pm.status()['costmap']
    assert st['running'] is True
    assert st['external'] is True
    assert st['recoverable'] is True
    assert st['ready'] is True
