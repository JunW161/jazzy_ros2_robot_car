from pathlib import Path
from types import SimpleNamespace as NS
import yaml
import pytest

from delivery_robot_web.core import Store, MotionLease, UserError
from delivery_robot_web.processes import ProcessManager
from delivery_robot_web.ros_adapter import OfflineAdapter
from delivery_robot_web.maps import import_candidates, import_existing


def make_store(tmp_path):
    s = Store(tmp_path / 'state')
    s.set_admin('admin', 'robot-test-password-2026')
    return s


def healthy_ros(store):
    motion = MotionLease(store.config)
    state = {
        'nodes': [],
        'tf_odom': {'age': 0.05},
        'scan_publishers': 1,
        'scan': {'receive_age': 0.05},
        'tf_odom_laser': True,
    }
    return NS(status=lambda: state, maintenance=False, motion=motion), state


def test_v24_ui_has_localization_controls():
    root = Path(__file__).resolve().parents[1] / 'delivery_robot_web' / 'static'
    html = (root / 'index.html').read_text()
    js = (root / 'app.js').read_text()
    for token in ('data-tab="localization"', 'id="localizationCanvas"', 'id="localizationMap"',
                  'id="startLocalization"', 'id="sendInitialPose"', 'id="openRviz"'):
        assert token in html
    for token in ('/api/localization/start', '/api/localization/initial-pose', '/api/rviz/start', 'latest.scan_points'):
        assert token in js


def test_rviz_profile_targets_map_and_topics():
    path = Path(__file__).resolve().parents[1] / 'delivery_robot_web' / 'rviz' / 'localization.rviz'
    data = yaml.safe_load(path.read_text())
    manager = data['Visualization Manager']
    assert manager['Global Options']['Fixed Frame'] == 'map'
    text = path.read_text()
    assert 'Value: /map' in text
    assert 'Value: /scan' in text
    assert 'rviz_default_plugins/SetInitialPose' in text


def test_localization_command_uses_selected_saved_map(tmp_path, monkeypatch):
    store = make_store(tmp_path)
    ros, state = healthy_ros(store)
    pm = ProcessManager(store, ros, offline=False)
    monkeypatch.setattr(pm, 'local_process_contains', lambda _: False)
    ws = tmp_path / 'ros2_ws'
    (ws / 'install').mkdir(parents=True)
    (ws / 'install/setup.bash').write_text('# fake overlay')
    store.config['ros_workspace'] = str(ws)
    map_yaml = tmp_path / 'map.yaml'
    map_yaml.write_text('image: map.pgm\nresolution: 0.05\norigin: [0,0,0]\n')
    captured = {}
    def fake_spawn(name, command, cwd=None, context=None):
        captured.update(name=name, command=command, cwd=cwd, context=context)
        return {'message': 'spawned'}
    monkeypatch.setattr(pm, '_spawn_shell', fake_spawn)
    result = pm.start_localization(map_yaml, 'a'*32, 'map_phong_01')
    assert captured['name'] == 'localization'
    assert 'delivery_robot_localization_v14 amcl_localization.launch.py' in captured['command']
    assert f'map:={map_yaml}' in captured['command']
    assert captured['context']['map_name'] == 'map_phong_01'
    assert 'Initial Pose' in result['message']


def test_localization_refuses_slam_or_external_amcl(tmp_path, monkeypatch):
    store = make_store(tmp_path)
    ros, state = healthy_ros(store)
    pm = ProcessManager(store, ros, offline=False)
    monkeypatch.setattr(pm, 'local_process_contains', lambda _: False)
    ws = tmp_path / 'ros2_ws'; (ws/'install').mkdir(parents=True); (ws/'install/setup.bash').write_text('#')
    store.config['ros_workspace'] = str(ws)
    path = tmp_path/'map.yaml'; path.write_text('x')
    state['nodes'] = ['/slam_toolbox']
    with pytest.raises(UserError, match='SLAM'):
        pm.start_localization(path, 'a'*32, 'map')
    state['nodes'] = ['/amcl']
    with pytest.raises(UserError, match='AMCL'):
        pm.start_localization(path, 'a'*32, 'map')


def test_offline_localization_and_rviz_are_never_started(tmp_path):
    store = make_store(tmp_path)
    motion = MotionLease(store.config)
    ros = OfflineAdapter(store.config, motion)
    pm = ProcessManager(store, ros, offline=True)
    with pytest.raises(UserError, match='OFFLINE'):
        pm.start_localization(tmp_path/'map.yaml', 'a'*32, 'map')
    with pytest.raises(UserError, match='OFFLINE'):
        pm.start('rviz')


def test_exact_external_localization_is_recoverable_and_can_be_stopped(tmp_path, monkeypatch):
    store = make_store(tmp_path)
    ros, state = healthy_ros(store)
    state['nodes'] = ['/amcl']
    pm = ProcessManager(store, ros, offline=False)
    monkeypatch.setattr(pm.agent, 'snapshot', lambda *a, **k: {'running': False, 'installed': False, 'message': ''})
    monkeypatch.setattr(pm, 'udp_in_use', lambda _port: False)
    monkeypatch.setattr(pm, 'recoverable_localization_processes', lambda: [
        {'pid': 4321, 'cmdline': '/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch delivery_robot_localization_v14 amcl_localization.launch.py map:=/tmp/map.yaml'}
    ])
    st = pm.status()['localization']
    assert st['external'] is True
    assert st['recoverable'] is True
    assert st['pid'] == 4321

    monkeypatch.setattr('delivery_robot_web.processes.time.sleep', lambda *_: None)
    monkeypatch.setattr('delivery_robot_web.processes.os.kill', lambda *_: (_ for _ in ()).throw(ProcessLookupError()))
    out = pm.stop('localization')
    assert 'Đã dừng Map Server + AMCL' in out['message']
