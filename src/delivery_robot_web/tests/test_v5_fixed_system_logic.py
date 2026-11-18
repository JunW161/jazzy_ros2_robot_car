import inspect
from pathlib import Path
from types import SimpleNamespace as NS

from delivery_robot_web.core import Store, MotionLease
from delivery_robot_web.processes import ProcessManager
from delivery_robot_web import cli
from delivery_robot_web.app import create_app


def make_pm(tmp_path):
    store = Store(tmp_path / 'state')
    motion = MotionLease(store.config)
    ros = NS(
        motion=motion,
        maintenance=False,
        status=lambda: {
            'nodes': [], 'scan_publishers': 0, 'odom_publishers': 0,
            'odom': {'receive_age': None}, 'scan': {'receive_age': None},
            'tf_odom': None, 'tf_map': None, 'tf_odom_laser': False,
            'global_costmap_publishers': 0, 'local_costmap_publishers': 0,
        },
        begin_map_session=lambda: None,
        end_map_session=lambda: None,
    )
    return ProcessManager(store, ros, offline=False)


def test_web_serve_does_not_start_robot_components_automatically():
    src = inspect.getsource(cli.main)
    # Startup is allowed to STOP stale exact project processes, but never start them.
    assert 'cleanup_stale_project_stack()' in src
    assert 'manager.start(' not in src
    assert src.index('cleanup_stale_project_stack()') < src.index('create_app(')


def test_connect_preflights_whole_stack_and_rolls_back_partial_start():
    src = inspect.getsource(create_app)
    block = src[src.index("@app.post('/api/system/connect')"):src.index("@app.post('/api/system/disconnect')")]
    assert 'manager.preflight_base_stack()' in block
    assert "order = ('agent','odom_bridge','ekf_odom','robot_model','lidar')" in block
    assert 'for name in reversed(started)' in block
    assert 'rolled_back' in block


def test_normal_web_close_includes_costmap():
    src = inspect.getsource(ProcessManager.close)
    assert "('rviz', 'costmap', 'localization', 'slam', 'lidar', 'robot_model', 'ekf_odom', 'odom_bridge')" in src


def test_recoverable_base_stack_is_strict(tmp_path, monkeypatch):
    pm = make_pm(tmp_path)
    pm.store.config['lidar_script'] = str(tmp_path / 'Downloads/LD14P_WIFI_BRIDGE_V1/ubuntu/start_ld14p_wifi.sh')

    rows = {
        'odom_tf_joint_bridge.py': [
            {'pid': 11, 'cmdline': '/usr/bin/python3 host_ros2/odom_tf_joint_bridge.py --ros-args -p left_joint:=wheel_joint_l -p right_joint:=wheel_joint_r -p publish_tf:=false'},
            {'pid': 12, 'cmdline': '/usr/bin/python3 odom_tf_joint_bridge.py --ros-args -p publish_tf:=true'},
        ],
        'official_odometry.launch.py': [
            {'pid': 21, 'cmdline': 'ros2 launch delivery_robot_localization_v14 official_odometry.launch.py start_bridge:=false'},
        ],
        '/robot_localization/ekf_node': [
            {'pid': 22, 'cmdline': '/opt/ros/jazzy/lib/robot_localization/ekf_node --ros-args -r __node:=ekf_filter_node_v14'},
        ],
        '/robot_state_publisher/robot_state_publisher': [
            {'pid': 31, 'cmdline': '/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher --ros-args -p robot_description:=<robot name="delivery_car_URDF_MAIN">'},
            {'pid': 32, 'cmdline': '/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher --ros-args -p robot_description:=<robot name="other">'},
        ],
        'start_ld14p_wifi.sh': [
            {'pid': 41, 'cmdline': f"bash {tmp_path}/Downloads/LD14P_WIFI_BRIDGE_V1/ubuntu/start_ld14p_wifi.sh 192.168.1.153 7777"},
        ],
        'ld14p_wifi.launch.py': [],
        '/ldlidar_publisher_ld14': [],
        'socat': [],
    }
    monkeypatch.setattr(pm, 'local_process_matches', lambda needle: rows.get(needle, []))
    assert {x['pid'] for x in pm.recoverable_base_processes('odom_bridge')} == {11}
    assert {x['pid'] for x in pm.recoverable_base_processes('ekf_odom')} == {21, 22}
    assert {x['pid'] for x in pm.recoverable_base_processes('robot_model')} == {31}
    assert {x['pid'] for x in pm.recoverable_base_processes('lidar')} == {41}


def test_startup_cleanup_never_starts_anything(tmp_path, monkeypatch):
    pm = make_pm(tmp_path)
    stopped = []
    mapping = {
        'rviz': [], 'slam': [], 'lidar': [{'pid': 4, 'cmdline': 'lidar'}],
        'robot_model': [{'pid': 3, 'cmdline': 'model'}],
        'ekf_odom': [{'pid': 2, 'cmdline': 'ekf'}],
        'odom_bridge': [{'pid': 1, 'cmdline': 'odom'}],
    }
    monkeypatch.setattr(pm, 'recoverable_base_processes', lambda name: mapping.get(name, []))
    monkeypatch.setattr(pm, 'recoverable_costmap_processes', lambda: [])
    monkeypatch.setattr(pm, 'recoverable_localization_processes', lambda: [])
    monkeypatch.setattr(pm, '_stop_exact_processes', lambda matches, label: stopped.append((matches[0]['pid'], label)))
    monkeypatch.setattr(pm.agent, 'snapshot', lambda fresh=False: {'running': False})
    cleaned = pm.cleanup_stale_project_stack()
    assert cleaned == ['lidar', 'robot_model', 'ekf_odom', 'odom_bridge']
    assert [x[0] for x in stopped] == [4, 3, 2, 1]
    assert pm.children == {}


def test_recoverable_external_process_can_be_stopped_from_ui():
    js = (Path(__file__).resolve().parents[1] / 'delivery_robot_web' / 'static' / 'app.js').read_text()
    assert "stop.disabled=!own?.running||(external&&!own?.recoverable)" in js


def test_installer_refuses_mixed_old_backend_new_frontend():
    root = Path(__file__).resolve().parents[3]
    text = (root / 'INSTALL_UBUNTU.sh').read_text()
    assert 'Web delivery_robot_web đang chạy' in text
    assert 'Ctrl+C terminal Web cũ' in text


def test_connect_prepares_fresh_stack_before_preflight():
    src = inspect.getsource(create_app)
    block = src[src.index("@app.post('/api/system/connect')"):src.index("@app.post('/api/system/disconnect')")]
    assert 'manager.prepare_base_stack_for_connect()' in block
    assert block.index('manager.prepare_base_stack_for_connect()') < block.index('manager.preflight_base_stack()')


def test_ekf_recovery_accepts_orphan_child_by_node_name(tmp_path, monkeypatch):
    pm = make_pm(tmp_path)
    rows = {
        'official_odometry.launch.py': [],
        '/robot_localization/ekf_node': [],
        'ekf_node': [
            {'pid': 77, 'cmdline': '/opt/ros/jazzy/lib/robot_localization/ekf_node --ros-args -r __node:=ekf_filter_node_v14 --params-file /home/tuan/ros2_ws/install/delivery_robot_sensor_fusion_v14/share/delivery_robot_sensor_fusion_v14/config/ekf_compare.yaml'}
        ],
        'ekf_filter_node_v14': [],
    }
    monkeypatch.setattr(pm, 'local_process_matches', lambda needle: rows.get(needle, []))
    assert {x['pid'] for x in pm.recoverable_base_processes('ekf_odom')} == {77}
