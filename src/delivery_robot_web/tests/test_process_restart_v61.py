"""Host ownership, explicit recovery and real-manager shutdown regressions.

The OS runner and ROS transport are fakes; ProcessManager/SystemCoordinator logic
is exercised unchanged. These tests do not send robot commands or OS signals.
"""
from pathlib import Path
from types import SimpleNamespace
import time

import pytest

from delivery_robot_web import processes
from delivery_robot_web.agent_service import AgentService
from delivery_robot_web.core import MotionLease, Store, UserError
from delivery_robot_web.delivery import DeliveryCoordinator
from delivery_robot_web.processes import COMPONENT_NODES, ProcessManager, STOP_ORDER
from delivery_robot_web.system_control import BASE_ORDER, SystemCoordinator


class Child:
    def __init__(self, pid):
        self.pid, self.returncode = pid, None

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        return self.returncode


@pytest.fixture
def rig(tmp_path, monkeypatch):
    store = Store(tmp_path / 'state')
    workspace = tmp_path / 'ros2_ws'
    motor = tmp_path / 'motor'
    urdf = tmp_path / 'delivery_robot.urdf'
    lidar = tmp_path / 'lidar/ubuntu/start_ld14p_wifi.sh'
    mux_launch = workspace / 'install/delivery_robot_cmd_mux/share/delivery_robot_cmd_mux/launch/cmd_mux.launch.py'
    for path in (workspace / 'install/setup.bash', motor / 'host_ros2/odom_tf_joint_bridge.py', urdf, lidar, mux_launch):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('<robot name="delivery_car_robot"/>\n' if path == urdf else '# fixture\n')
    store.config.update(ros_workspace=str(workspace), motor_project=str(motor),
                        description_urdf=str(urdf), lidar_script=str(lidar))
    motion = MotionLease(store.config)
    health = {'ros_ok': True, 'nodes': [], 'odom_publishers': 0, 'scan_publishers': 0,
              'odom': {'receive_age': 0.01}, 'scan': {'receive_age': 0.01},
              'tf_odom': {'age': 0.01}, 'tf_odom_laser': True,
              'global_costmap_publishers': None, 'local_costmap_publishers': None}
    events, retired = [], []
    gateway = SimpleNamespace(
        snapshot=lambda: {'ready': False, 'goal': {'active': False}, 'mission': {'active': False},
                          'mux': {'available': False, 'mode': 'STOP'}},
        stop_all=lambda: {'mux_confirmed': False},
        managed_shutdown_complete=lambda names: {},
    )
    ros = SimpleNamespace(status=lambda: dict(health), navigation=gateway,
                          motion=motion, maintenance=False,
                          end_map_session=lambda: None,
                          reset_observations=lambda names: retired.extend(names))
    manager = ProcessManager(store, ros)
    monkeypatch.setattr(manager, '_autonomy_processes', lambda name: [])
    monkeypatch.setattr(manager, 'recoverable_base_processes', lambda name: [])
    monkeypatch.setattr(manager, 'recoverable_localization_processes', lambda: [])
    monkeypatch.setattr(manager, 'recoverable_costmap_processes', lambda: [])
    monkeypatch.setattr(manager, 'local_process_contains', lambda needle: False)
    monkeypatch.setattr(manager, '_owned_group_running', lambda pgid: False)
    monkeypatch.setattr(manager, 'udp_in_use', lambda port: False)
    agent = {'installed': True, 'status_known': True, 'running': False, 'message': ''}
    monkeypatch.setattr(manager.agent, 'snapshot', lambda **kw: dict(agent))
    def start_agent():
        events.append(('start', 'agent'))
        agent['running'] = True
        health['odom_publishers'] = 1
        return 'started'
    def stop_agent():
        events.append(('stop', 'agent'))
        agent['running'] = False
        return 'stopped'
    monkeypatch.setattr(manager.agent, 'start', start_agent)
    monkeypatch.setattr(manager.agent, 'stop', stop_agent)
    pid = [400000]
    def popen(args, **kwargs):
        cmd = args[-1]
        name = next(name for marker, name in (
            ('odom_tf_joint_bridge.py', 'odom_bridge'),
            ('official_odometry.launch.py', 'ekf_odom'),
            ('robot_state_publisher', 'robot_model'),
            ('start_ld14p_wifi.sh', 'lidar'), ('cmd_mux.launch.py', 'cmd_mux')) if marker in cmd)
        assert kwargs['start_new_session'] is True
        assert kwargs['stdin'] == processes.subprocess.DEVNULL
        events.append(('start', name))
        health['nodes'] = list(set(health['nodes']) | set(COMPONENT_NODES[name]))
        if name == 'lidar':
            health['scan_publishers'] = 1
        pid[0] += 1
        return Child(pid[0])
    monkeypatch.setattr(processes.subprocess, 'Popen', popen)
    monkeypatch.setattr(processes, 'time', SimpleNamespace(
        time=time.time, time_ns=time.time_ns, monotonic=time.monotonic, sleep=lambda seconds: None))
    def stop_child(process):
        name = next(n for n, item in manager.children.items() if item['process'] is process)
        events.append(('stop', name))
        process.returncode = 0
    monkeypatch.setattr(manager, '_stop_owned_child', stop_child)
    value = SimpleNamespace(store=store, manager=manager, ros=ros, gateway=gateway, motion=motion,
                            health=health, events=events, retired=retired, agent=agent,
                            workspace=workspace, motor=motor)
    yield value
    for item in manager.children.values():
        item['handle'].close()
    store.db.close()


def install_child(rig, name):
    rig.manager.children[name] = {'process': Child(800000 + len(rig.manager.children)),
                                 'handle': SimpleNamespace(close=lambda: None),
                                 'context': {}}


def finish(system):
    for worker in list(system.workers):
        worker.join(3)
        assert not worker.is_alive()
    return system.snapshot()


def test_actual_manager_system_three_cycles_with_dds_ghosts(rig):
    r = rig
    delivery = DeliveryCoordinator(r.store, r.ros, r.motion, r.manager)
    system = SystemCoordinator(r.store, r.ros, r.motion, r.manager, delivery)
    delivery.system = system
    try:
        for _ in range(3):
            system.begin('connect')
            connected = finish(system)
            assert connected['state'] == 'succeeded', connected.get('message')
            assert all(r.manager.status()[name]['running'] for name in BASE_ORDER)
            system.begin('disconnect')
            result = finish(system)
            assert result['state'] == 'succeeded', result
            assert result['warnings']  # no mux is needed to close host processes
            assert r.health['nodes']  # ROS graph deliberately never clears
            assert not any(item.get('running') for item in r.manager.status().values())
        assert [name for action, name in r.events if action == 'start'] == list(BASE_ORDER) * 3
        assert [name for action, name in r.events if action == 'stop'] == [n for n in STOP_ORDER if n in BASE_ORDER] * 3
        assert len(r.retired) == len(BASE_ORDER) * 3
    finally:
        system.close()
        delivery.close()


def test_dead_ros_and_one_stop_error_still_attempt_all_owned(rig, monkeypatch):
    r = rig
    for name in ('navigation', 'ekf_odom', 'odom_bridge'):
        install_child(r, name)
    r.manager.agent_started = r.agent['running'] = True
    r.ros.status = lambda: (_ for _ in ()).throw(RuntimeError('executor died'))
    r.gateway.stop_all = lambda: (_ for _ in ()).throw(RuntimeError('no executor'))
    original = r.manager._stop_owned_child
    def stop_child(process):
        if process is r.manager.children['navigation']['process']:
            raise OSError('cannot signal navigation')
        original(process)
    monkeypatch.setattr(r.manager, '_stop_owned_child', stop_child)
    rows = r.manager._stop_plan(('navigation', 'ekf_odom', 'odom_bridge', 'agent'))
    assert [row['ok'] for row in rows] == [False, True, True, True]
    assert [name for action, name in r.events if action == 'stop'] == ['ekf_odom', 'odom_bridge', 'agent']
    assert r.retired == ['ekf_odom', 'odom_bridge', 'agent']
    assert 'navigation' not in r.manager.confirmed_stops


def test_individual_stop_cascades_then_reopens(rig):
    r = rig
    for name in ('mission', 'navigation', 'odom_bridge', 'ekf_odom'):
        install_child(r, name)
    result = r.manager.stop('odom_bridge', request_stop=False)
    assert result['ok']
    assert [name for action, name in r.events if action == 'stop'] == ['mission', 'navigation', 'odom_bridge']
    assert r.manager.children['ekf_odom']['process'].poll() is None
    r.manager.start('odom_bridge')
    assert r.manager.children['odom_bridge']['process'].poll() is None


def test_only_successful_replacement_calls_ros_restart_boundary(rig, monkeypatch):
    resumed = []
    rig.ros.managed_start_complete = lambda names: resumed.extend(names)
    rig.manager.start('odom_bridge')
    assert not resumed
    rig.manager.stop('odom_bridge', request_stop=False)
    rig.manager.start('odom_bridge')
    assert resumed == ['odom_bridge']
    rig.manager.stop('odom_bridge', request_stop=False)
    popen = processes.subprocess.Popen
    def failed(*args, **kwargs):
        child = popen(*args, **kwargs)
        child.returncode = 1
        return child
    monkeypatch.setattr(processes.subprocess, 'Popen', failed)
    with pytest.raises(UserError, match='thoát'):
        rig.manager.start('odom_bridge')
    assert resumed == ['odom_bridge']
    assert 'odom_bridge' in rig.manager.confirmed_stops


def test_agent_restart_boundary_requires_new_owned_container(rig):
    resumed = []
    rig.ros.managed_start_complete = lambda names: resumed.extend(names)
    rig.manager.start('agent')
    assert not resumed
    rig.manager.stop('agent', cascade=False, request_stop=False)
    rig.manager.start('agent')
    assert resumed == ['agent']


def test_rviz_stop_does_not_interrupt_motion_or_request_mux(rig, monkeypatch):
    install_child(rig, 'rviz')
    monkeypatch.setattr(rig.motion, 'stop', lambda *args, **kwargs: pytest.fail('RViz is display only'))
    rig.gateway.stop_all = lambda: pytest.fail('RViz does not control command authority')
    rig.manager.stop('rviz')
    assert rig.retired == ['rviz']


def test_strict_recovery_rejects_generic_node_and_other_workspace(rig):
    pm = rig.manager
    executable = '/opt/ros/jazzy/lib/robot_localization/ekf_node'
    generic = {'argv': [executable, '--ros-args', '-r', '__node:=ekf_filter_node_v14']}
    assert not pm._project_identity_matches('ekf_odom', generic)
    params = rig.workspace / 'install/delivery_robot_sensor_fusion_v14/share/delivery_robot_sensor_fusion_v14/config/ekf.yaml'
    assert pm._project_identity_matches('ekf_odom', {'argv': [executable, '--params-file', str(params)]})
    assert not pm._project_identity_matches('ekf_odom', {'argv': [executable, '--params-file', '/another_ws/delivery_robot_sensor_fusion_v14/ekf.yaml']})
    bridge = {'argv': ['python3', 'host_ros2/odom_tf_joint_bridge.py', '--ros-args',
                       '-p', 'publish_tf:=false', '-p', 'left_joint:=wheel_joint_l', '-p', 'right_joint:=wheel_joint_r'],
              'cwd': str(rig.motor)}
    assert pm._project_identity_matches('odom_bridge', bridge)
    bridge['cwd'] = '/another_project'
    assert not pm._project_identity_matches('odom_bridge', bridge)


def test_only_explicit_stop_recovers_verified_previous_session(rig, monkeypatch):
    pm = rig.manager
    item = {'pid': 880001, 'argv': ['python3', str(rig.motor / 'host_ros2/odom_tf_joint_bridge.py'),
                                  'publish_tf:=false', 'left_joint:=wheel_joint_l', 'right_joint:=wheel_joint_r']}
    item['cmdline'] = ' '.join(item['argv'])
    candidates = [item]
    monkeypatch.setattr(pm, 'recoverable_base_processes', lambda name: candidates if name == 'odom_bridge' else [])
    stopped = []
    def recover(matches, label):
        stopped.append((label, matches[0]['pid']))
        candidates.clear()
    monkeypatch.setattr(pm, '_stop_exact_processes', recover)
    with pytest.raises(UserError):
        pm.prepare_base_stack_for_connect(reuse_owned=True)
    assert not stopped
    result = pm.stop('odom_bridge', request_stop=False)
    assert result['results'][-1]['recovered']
    assert stopped == [('odom_bridge', 880001)]


def test_fixed_helper_agent_from_previous_web_is_explicitly_stopped(rig):
    rig.agent['running'] = True
    assert not rig.manager.agent_started
    result = rig.manager.stop('agent', cascade=False, request_stop=False)
    assert result['results'][0]['recovered']
    assert rig.events == [('stop', 'agent')]
    assert rig.retired == ['agent']


def test_unknown_agent_on_udp_port_is_reported_not_stopped(rig, monkeypatch):
    rig.agent['installed'] = False
    monkeypatch.setattr(rig.manager, 'udp_in_use', lambda port: True)
    rows = rig.manager._stop_plan(('agent',), request_stop=False)
    assert rows[0]['ok'] is False and rows[0]['state'] == 'external'
    assert not rig.events and not rig.retired


def test_pid_reuse_prevents_any_recovery_signal(rig, monkeypatch):
    item = {'pid': 880001, 'start_ticks': 100, 'argv': [], 'cmdline': ''}
    monkeypatch.setattr(rig.manager, '_process_identity', lambda pid: {**item, 'start_ticks': 101})
    monkeypatch.setattr(processes.os, 'kill', lambda *args: pytest.fail('PID changed before recovery'))
    with pytest.raises(UserError, match='đổi danh tính'):
        rig.manager._stop_exact_processes([item], 'odom_bridge')


def test_progress_callback_failure_does_not_skip_child_cleanup(rig):
    for name in ('ekf_odom', 'odom_bridge'):
        install_child(rig, name)
    def bad_progress(row):
        raise RuntimeError('client disconnected')
    rows = rig.manager._stop_plan(('ekf_odom', 'odom_bridge'), progress=bad_progress, request_stop=False)
    assert all(row['ok'] for row in rows)
    assert rig.retired == ['ekf_odom', 'odom_bridge']


@pytest.mark.parametrize('after', ['RUNNING\n', 'unknown'])
def test_agent_stop_requires_fresh_verified_container_exit(tmp_path, after):
    helper = tmp_path / 'helper'
    helper.write_text('# fixed helper')
    status_calls = [0]
    actions = []
    def run(args, **kwargs):
        action = args[-1]
        actions.append(action)
        if action == 'status':
            status_calls[0] += 1
            value = 'RUNNING\n' if status_calls[0] == 1 else after
            return SimpleNamespace(returncode=5 if value == 'unknown' else 0, stdout=value, stderr='')
        return SimpleNamespace(returncode=0, stdout='stop sent', stderr='')
    service = AgentService({'agent_port': 8888}, helper, run)
    with pytest.raises(UserError, match='chưa xác nhận'):
        service.stop()
    assert actions == ['status', 'stop', 'status']
