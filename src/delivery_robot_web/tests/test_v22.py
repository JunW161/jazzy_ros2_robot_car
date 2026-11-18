"""V2.2 tests: exact verified startup commands, Docker helper, and fresh SLAM sessions.
No test touches a real robot, Docker daemon, serial port or ROS graph.
"""
import inspect
import math
import os
import subprocess
import threading
import time
from pathlib import Path
from types import SimpleNamespace as NS
import pytest

from delivery_robot_web.core import DEFAULT_CONFIG, Store, MotionLease, UserError
from delivery_robot_web.motion_profile import wheel_rpms, clamp_twist, limits_info
from delivery_robot_web.agent_service import AgentService
from delivery_robot_web.processes import ProcessManager
from delivery_robot_web.ros_adapter import RosAdapter
from delivery_robot_web.app import create_app


def test_40rpm_profile_is_preserved():
    c = DEFAULT_CONFIG.copy()
    assert c['max_linear'] == pytest.approx(.184306769)
    assert c['max_angular'] == pytest.approx(1.890325836)
    assert wheel_rpms(c['max_linear'], 0, c) == pytest.approx((40, 40))
    assert wheel_rpms(-c['max_linear'], 0, c) == pytest.approx((-40, -40))
    li = limits_info(c)
    w20 = 4 * math.pi * li['radius_m'] * li['turn_rpm'] / (60 * li['track_m'])
    assert wheel_rpms(0, w20, c) == pytest.approx((-20, 20))


@pytest.mark.parametrize('v,w', [(.1843, 1.8903), (-.1843, -1.8903), (9, 9), (-4, 2), (0, 0)])
def test_combined_velocity_respects_wheel_limit(v, w):
    c = DEFAULT_CONFIG.copy(); a, b = clamp_twist(v, w, c)
    assert max(map(abs, wheel_rpms(a, b, c))) <= 40 + 1e-10


class AgentRunner:
    def __init__(self):
        self.calls = []
        self.active = False
        self.fail_start = False

    def __call__(self, args, **kwargs):
        self.calls.append(args)
        action = args[-1]
        if action == 'status':
            return NS(returncode=0 if self.active else 3,
                      stdout='RUNNING\n' if self.active else 'STOPPED\n', stderr='')
        if action == 'start':
            if self.fail_start:
                return NS(returncode=1, stdout='', stderr='sudo helper denied')
            self.active = True
            return NS(returncode=0, stdout='RUNNING\n', stderr='')
        if action == 'stop':
            self.active = False
            return NS(returncode=0, stdout='STOPPED\n', stderr='')
        if action == 'log':
            return NS(returncode=0 if self.active else 3, stdout='agent log\n', stderr='')
        raise AssertionError(action)


def make_agent(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '0')
    monkeypatch.delenv('ROS_LOCALHOST_ONLY', raising=False)
    helper = tmp_path / 'delivery-robot-agent-control'
    helper.write_text('#!/bin/sh\n')
    helper.chmod(0o755)
    run = AgentRunner()
    return AgentService(DEFAULT_CONFIG.copy(), helper=helper, runner=run), run


def test_agent_uses_only_narrow_fixed_helper(tmp_path, monkeypatch):
    a, run = make_agent(tmp_path, monkeypatch)
    assert not a.snapshot(fresh=True)['running']
    a.start(); assert a.snapshot(fresh=True)['running']
    assert 'agent log' in a.log()
    a.stop(); assert not a.snapshot(fresh=True)['running']
    assert all(c[:2] == ['/usr/bin/sudo', '-n'] for c in run.calls)
    assert all(c[2] == str(a.helper) for c in run.calls)
    assert {c[-1] for c in run.calls} <= {'start', 'stop', 'status', 'log'}
    assert not any('/usr/bin/docker' in c for call in run.calls for c in call)


def test_agent_setup_missing_is_actionable(tmp_path):
    a = AgentService(DEFAULT_CONFIG.copy(), helper=tmp_path/'missing')
    with pytest.raises(UserError, match='SETUP_AGENT_WEB_HELPER'):
        a.start()


def test_agent_rejects_wrong_port_or_localhost(tmp_path, monkeypatch):
    a, _ = make_agent(tmp_path, monkeypatch)
    a.config['agent_port'] = 9999
    with pytest.raises(UserError, match='8888'):
        a.start()
    a.config['agent_port'] = 8888
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    with pytest.raises(UserError, match='ROS_LOCALHOST_ONLY'):
        a.start()


def minimal_ros(motion):
    return NS(
        motion=motion, maintenance=False,
        status=lambda: {'nodes': [], 'scan_publishers': 0, 'odom_publishers': 0,
                        'odom': {'receive_age': None}, 'tf_odom': None,
                        'mapping_issues': [], 'tf_map': None},
        begin_map_session=lambda: None, end_map_session=lambda: None,
    )


def manager(tmp_path, monkeypatch):
    store = Store(tmp_path/'state'); motion = MotionLease(store.config)
    pm = ProcessManager(store, minimal_ros(motion))
    pm.agent, _ = make_agent(tmp_path, monkeypatch)
    monkeypatch.setattr(pm, 'udp_in_use', lambda p: False)
    monkeypatch.setattr(pm, 'local_process_contains', lambda n: False)
    return pm


class Child:
    def __init__(self): self.pid = 900001; self.returncode = None
    def poll(self): return self.returncode
    def wait(self, timeout=None): return self.returncode


def test_host_commands_match_operator_verified_commands(tmp_path, monkeypatch):
    pm = manager(tmp_path, monkeypatch); c = pm.store.config
    motor = tmp_path/'micro_ros_ws/micro_ros_espidf_component/examples/robot_motor_control'
    (motor/'host_ros2').mkdir(parents=True)
    (motor/'host_ros2/odom_tf_joint_bridge.py').write_text('# bridge')
    c['motor_project'] = str(motor)
    rosws = tmp_path/'ros2_ws'; (rosws/'install').mkdir(parents=True)
    (rosws/'install/setup.bash').write_text('# setup')
    urdf = rosws/'src/delivery_robot_description/urdf/delivery_car_robot.urdf'
    urdf.parent.mkdir(parents=True); urdf.write_text('<robot name="original"/>')
    c['ros_workspace'] = str(rosws); c['description_urdf'] = str(urdf)
    lidar = tmp_path/'Downloads/LD14P_WIFI_BRIDGE_V1/ubuntu/start_ld14p_wifi.sh'
    lidar.parent.mkdir(parents=True); lidar.write_text('#!/bin/sh\n'); lidar.chmod(0o755)
    c['lidar_script'] = str(lidar)

    cmds = []
    def popen(argv, **kwargs):
        cmds.append((argv, kwargs)); return Child()
    monkeypatch.setattr('delivery_robot_web.processes.subprocess.Popen', popen)
    monkeypatch.setattr('delivery_robot_web.processes.time.sleep', lambda _: None)

    for name in ('odom_bridge', 'robot_model', 'lidar'):
        pm.start(name)

    odom = cmds[0][0][2]
    model = cmds[1][0][2]
    scan = cmds[2][0][2]
    assert 'source /opt/ros/jazzy/setup.bash' in odom
    assert 'exec /usr/bin/python3 host_ros2/odom_tf_joint_bridge.py --ros-args -p left_joint:=wheel_joint_l -p right_joint:=wheel_joint_r -p publish_tf:=false' in odom
    assert f'cd {motor}' in odom
    assert 'source /opt/ros/jazzy/setup.bash' in model
    assert f'source {rosws}/install/setup.bash' in model
    assert 'ros2 run robot_state_publisher robot_state_publisher --ros-args' in model
    assert 'robot_description:="$(cat ' in model and str(urdf) in model
    assert f'cd {lidar.parent.parent}' in scan
    assert './ubuntu/start_ld14p_wifi.sh 192.168.1.153 7777' in scan
    for argv, kwargs in cmds:
        assert argv[:2] == ['/bin/bash', '-lc']
        assert kwargs['start_new_session'] and kwargs['stdin'] == subprocess.DEVNULL
    for item in pm.children.values(): item['handle'].close()


def test_connect_endpoint_has_exact_start_order():
    from delivery_robot_web.system_control import BASE_ORDER
    assert BASE_ORDER == ('agent', 'odom_bridge', 'ekf_odom', 'robot_model', 'lidar', 'cmd_mux')


def test_root_helper_is_fixed_and_not_systemd():
    root = Path(__file__).resolve().parents[3]
    if not (root/'agent_control_root.sh').is_file() or not (root/'SETUP_AGENT_WEB_HELPER.sh').is_file():
        pytest.skip('Root Agent helper is absent from the uploaded source; this release preserves the existing installed helper.')
    helper = (root/'agent_control_root.sh').read_text()
    setup = (root/'SETUP_AGENT_WEB_HELPER.sh').read_text()
    assert 'microros/micro-ros-agent:jazzy' in helper
    assert '--net=host' in helper and 'udp4 --port 8888 -v6' in helper
    assert 'systemctl start' not in helper
    assert 'NOPASSWD: ALL' not in setup
    assert 'usermod' not in setup
    assert 'visudo -cf' in setup
    assert '/usr/local/sbin/delivery-robot-agent-control start' in setup


def test_live_map_session_ignores_old_map_and_clears_after_stop():
    r = RosAdapter.__new__(RosAdapter)
    r.lock = threading.RLock(); r.map_capture_enabled = False
    r.map_msg = None; r.map_at = 0.0; r.map_rev = 0
    r.cached = {'tf_map': {'x': 1}, 'scan_points': [[1, 2]]}
    old = NS()
    r.on_map(old)
    assert r.map_msg is None
    r.begin_map_session(); assert r.map_capture_enabled and r.map_msg is None
    fresh = NS(); r.on_map(fresh); assert r.map_msg is fresh
    r.end_map_session()
    assert not r.map_capture_enabled and r.map_msg is None and r.map_at == 0.0
    assert r.cached['tf_map'] is None and r.cached['scan_points'] == []
    r.on_map(old); assert r.map_msg is None


def test_slam_stop_ends_map_session(tmp_path, monkeypatch):
    store = Store(tmp_path/'state'); motion = MotionLease(store.config)
    calls = []
    ros = minimal_ros(motion); ros.end_map_session = lambda: calls.append('end')
    pm = ProcessManager(store, ros)
    log = store.root/'logs/slam.log'; log.write_text('ok')
    child = Child(); child.returncode = 0
    handle = open(log, 'ab')
    pm.children['slam'] = {'process': child, 'log': log, 'handle': handle, 'command': 'slam'}
    pm.stop('slam')
    assert calls == ['end']
