"""Regression cases grounded in the Jazzy dict-MessageInfo executor crash."""
from array import array
from collections import deque
from concurrent.futures import Future
import copy
import inspect
import queue
import sys
import threading
import time
from types import ModuleType, SimpleNamespace as S

import pytest

from delivery_robot_web.core import DEFAULT_CONFIG, MotionLease, UserError
from delivery_robot_web.navigation_ros import NavigationRos, publisher_gid
from delivery_robot_web.ros_adapter import RosAdapter
from test_navigation_protocol import Client, gateway, mission_message, status_message


def adapter():
    r = RosAdapter.__new__(RosAdapter)
    r.config = DEFAULT_CONFIG.copy(); r.motion = MotionLease(r.config)
    r.lock = threading.RLock(); r.executor_lock = threading.RLock()
    r.callback_errors = {}; r.executor_error = ''; r.executor_last_error = ''
    r.executor_heartbeat = time.monotonic(); r.closed = False
    r.observation_epoch = 0; r.minimum_tf_stamp = 0.0
    r.endpoint_observations = {}; r.retired_endpoints = {}; r.observed_publishers = {}
    r.thread = S(is_alive=lambda: True)
    r.logs = []
    r.node = S(get_clock=lambda: S(now=lambda: S(nanoseconds=1_800_000_000_000_000_000)),
               get_logger=lambda: S(error=r.logs.append))
    r.jobs = queue.Queue(); r.maintenance = False
    r.cached = {'nodes': ['/delivery_robot_motor', '/amcl', '/ekf_filter_node_v14'],
                'odom_publishers': 1, 'scan_publishers': 1, 'cmd_subscribers': 1,
                'global_costmap_publishers': 0, 'local_costmap_publishers': 0,
                'other_cmd_publishers': [], 'motor_cmd_receivers': ['/delivery_robot_motor'],
                'motor_cmd_publishers': ['/delivery_robot_cmd_mux'],
                'tf_odom': {'stamp': 1_799_999_999.99, 'x': 0.0, 'y': 0.0, 'yaw': 0.0},
                'tf_map': None, 'tf_odom_laser': True, 'scan_tf_ready': True,
                'scan_points': [], 'slam_state': 'offline'}
    r.last_graph = r.graph_at = time.monotonic(); r.last_service = 0.0
    r.odom = r.scan = r.map_msg = r.ros_map_msg = None
    r.odom_at = r.scan_at = r.map_at = r.ros_map_at = 0.0
    r.map_rev = r.ros_map_rev = 0; r.map_capture_enabled = False
    r.safety = r.wheels = r.heading = None
    r.safety_at = r.wheels_at = r.heading_at = 0.0
    r.arrivals = {'odom': deque(maxlen=50), 'scan': deque(maxlen=50)}
    r.scan_history = deque(maxlen=10); r.pending_scans = deque()
    r.gate_forwarded = r.gate_dropped = 0
    return r


def odometry(stamp=1_800_000_000):
    return S(header=S(frame_id='odom', stamp=S(sec=stamp, nanosec=0)),
             pose=S(pose=S(position=S(x=0., y=0., z=0.), orientation=S(x=0., y=0., z=0., w=1.))),
             twist=S(twist=S(linear=S(x=0., y=0., z=0.), angular=S(x=0., y=0., z=0.))))


@pytest.mark.parametrize('info', [
    {'publisher_gid': bytes([1, 2, 3])}, {'publisher_gid': [1, 2, 3]},
    {'publisher_gid': array('B', [1, 2, 3])}, S(publisher_gid=bytes([1, 2, 3])),
    {'publisher_gid': memoryview(bytes([1, 2, 3]))},
])
def test_jazzy_dictionary_and_attribute_metadata_use_same_gid(info):
    assert publisher_gid(info) == '010203'
    nav = gateway(); nav.action_providers = ('010203',)
    nav._on_mux(S(data='MANUAL'), info)
    nav._on_mission(mission_message(), info)
    nav._on_action_status(status_message(), info)
    assert nav.mux_provider == nav.mission_provider == '010203'
    assert nav.mux_mode == 'MANUAL' and nav.action_ids == {'01'}


@pytest.mark.parametrize('info', [None, {}, {'publisher_gid': None}, {'publisher_gid': 1000000000},
                                 {'publisher_gid': ['bad']}, {'publisher_gid': b''},
                                 {'publisher_gid': bytes(24)}, object()])
def test_absent_or_malformed_optional_metadata_never_kills_callback(info):
    assert publisher_gid(info) is None
    nav = gateway()
    nav._on_mux(S(data='STOP'), info)
    assert nav.mux_mode == 'STOP' and nav.mux_at > 0


def test_required_callback_arity_passes_real_dict_metadata():
    ros = adapter(); nav = gateway(); ros.navigation = nav
    callback = ros.guard_callback('mux', nav._on_mux, with_info=True)
    assert len(inspect.signature(callback).parameters) == 2
    callback(S(data='MANUAL'), {'publisher_gid': bytes([4, 5, 6]), 'source_timestamp': 42})
    assert nav.mux_provider == '040506' and not ros.callback_errors
    assert not ros.logs


def registered_gateway(monkeypatch):
    def install_module(name, **values):
        module = ModuleType(name)
        module.__dict__.update(values)
        monkeypatch.setitem(sys.modules, name, module)
    service = S(Request=lambda: S())
    install_module('rclpy.action', ActionClient=lambda *args: S(server_is_ready=lambda: True))
    install_module('rclpy.qos', QoSProfile=lambda **kw: S(**kw),
                   ReliabilityPolicy=S(RELIABLE=1), DurabilityPolicy=S(TRANSIENT_LOCAL=1))
    install_module('lifecycle_msgs.srv', GetState=service)
    install_module('rcl_interfaces.srv', GetParameters=service)
    install_module('std_srvs.srv', Trigger=service, SetBool=service)
    install_module('std_msgs.msg', String=object)
    install_module('nav_msgs.msg', Path=object)
    install_module('geometry_msgs.msg', PoseStamped=object)
    install_module('action_msgs.msg', GoalStatusArray=object)
    install_module('nav2_msgs.action', NavigateToPose=object)
    install_module('delivery_robot_interfaces.msg', MissionState=object)
    install_module('delivery_robot_interfaces.srv', StartDelivery=service)
    ros = adapter(); callbacks = {}
    ros.node.create_client = lambda *args: Client()
    def subscribe(typ, topic, callback, qos):
        callbacks[topic] = callback
        return callback
    ros.node.create_subscription = subscribe
    ros.node.destroy_subscription = lambda handle: True
    ros.node.create_timer = lambda *args: None
    ros.navigation = NavigationRos(ros)
    return ros, callbacks


def test_actual_navigation_constructor_registers_dict_messageinfo_callbacks(monkeypatch):
    """Exercise the registration path; no attribute-only MessageInfo substitute."""
    ros, callbacks = registered_gateway(monkeypatch)
    info = {'publisher_gid': bytes([1, 2, 3]), 'source_timestamp': 11,
            'received_timestamp': 12, 'publication_sequence_number': 1,
            'reception_sequence_number': 1}
    inputs = {
        '/delivery_robot/cmd_mux/state': S(data='STOP'),
        '/delivery_robot/mission/state': mission_message(),
        '/navigate_to_pose/_action/status': status_message(),
    }
    for topic, msg in inputs.items():
        callback = callbacks[topic]
        # rclpy uses signature binding to distinguish one- and two-arg callbacks.
        with pytest.raises(TypeError):
            inspect.signature(callback).bind(msg)
        inspect.signature(callback).bind(msg, info)
        callback(msg, info)
    assert ros.navigation.mux_provider == ros.navigation.mission_provider == '010203'
    assert ros.navigation.action_observations['010203'] == {'01'}
    assert not ros.callback_errors and not ros.logs


@pytest.mark.parametrize('component', ['cmd_mux', 'mission', 'navigation'])
def test_verified_restart_accepts_same_gid_but_rejects_old_callback_generation(monkeypatch, component):
    ros, callbacks = registered_gateway(monkeypatch)
    nav = ros.navigation; metadata = {'publisher_gid': b'same-gid'}
    mux_topic = '/delivery_robot/cmd_mux/state'
    mission_topic = '/delivery_robot/mission/state'
    action_topic = '/navigate_to_pose/_action/status'
    callbacks[mux_topic](S(data='MANUAL'), metadata)
    callbacks[mission_topic](mission_message(), metadata)
    nav._observe_action_providers((b'same-gid'.hex(),))
    callbacks[action_topic](status_message(), metadata)
    old = dict(callbacks)
    nav.managed_shutdown_complete([component])
    nav.managed_start_complete([component], valid=lambda: True)
    if component in ('navigation', 'cmd_mux'):
        old[mux_topic](S(data='MANUAL'), metadata)
        assert nav.mux_mode == 'UNKNOWN' and nav.mux_at == 0.0
        callbacks[mux_topic](S(data='STOP'), metadata)
        assert nav.mux_mode == 'STOP' and nav.mux_at > 0
        old[mux_topic](S(data='MANUAL'), metadata)
        assert nav.mux_mode == 'STOP'
    if component == 'mission':
        old[mission_topic](mission_message(), metadata)
        assert nav.mission_at == 0.0 and not nav.mission['active']
        callbacks[mission_topic](mission_message('IDLE', ''), metadata)
        assert nav.mission_at > 0 and not nav.mission['active']
        old[mission_topic](mission_message(), metadata)
        assert not nav.mission['active']
    if component == 'navigation':
        old[action_topic](status_message(), metadata)
        assert not nav.action_ids
        callbacks[action_topic](status_message(2), metadata)
        assert nav.action_ids == {'02'}
        old[action_topic](status_message(1), metadata)
        assert nav.action_ids == {'02'}
        plan = S(header=S(frame_id='map'), poses=[S(pose=S(position=S(x=1, y=2)))])
        old['/plan'](plan)
        assert not nav.plan['points']
        callbacks['/plan'](plan)
        assert nav.plan['points'] == [[1., 2.]]
    assert not ros.callback_errors


def test_canceled_start_does_not_release_retired_navigation_provider(monkeypatch):
    ros, callbacks = registered_gateway(monkeypatch)
    nav = ros.navigation; topic = '/delivery_robot/cmd_mux/state'
    metadata = {'publisher_gid': b'same-gid'}
    callbacks[topic](S(data='MANUAL'), metadata)
    nav.managed_shutdown_complete(['cmd_mux'])
    with pytest.raises(UserError, match='bị dừng'):
        nav.managed_start_complete(['cmd_mux'], valid=lambda: False)
    callbacks[topic](S(data='MANUAL'), metadata)
    assert b'same-gid'.hex() in nav.retired_mux_providers
    assert nav.mux_at == 0.0 and nav.mux_mode == 'UNKNOWN'


def test_callback_error_is_logged_disarms_and_next_valid_sample_recovers():
    ros = adapter(); ros.navigation = gateway()
    ros.motion.arm('operator', True)
    stops = []; ros.navigation._stop_async = lambda: stops.append(True)
    callback = ros.guard_callback('mux', ros.navigation._on_mux, with_info=True)
    callback(S(), {'publisher_gid': [1]})
    assert not ros.motion.status()['armed']
    assert not ros.status()['ros_ok'] and ros.callback_errors['mux']
    assert 'Traceback' in ros.logs[0] and 'AttributeError' in ros.logs[0]
    assert stops == [True]
    callback(S(data='STOP'), {'publisher_gid': [1]})
    assert ros.status()['ros_ok'] and not ros.callback_errors
    assert not ros.motion.status()['armed']  # Recovery never re-arms a lease.


def test_age_is_receipt_clock_not_message_epoch_and_old_stamp_is_not_corrected():
    ros = adapter(); ros.on_odom(odometry(1_799_996_400))
    s = ros.status()
    assert s['odom']['receive_age'] < 0.1
    assert s['odom']['stamp_age'] == pytest.approx(3600.0)
    assert not s['motion_ready']
    ros.on_odom(odometry(0)); s = ros.status()
    assert s['odom']['stamp_age'] is None and s['odom']['stamp_valid'] is False
    assert not s['motion_ready']


def test_stalled_executor_and_old_graph_cannot_claim_live_readiness():
    ros = adapter(); ros.on_odom(odometry())
    ros.executor_heartbeat -= 8; ros.graph_at -= 8
    s = ros.status()
    assert not s['ros_ok'] and s['ros_executor']['state'] == 'stalled'
    assert not s['graph_fresh'] and s['odom_publishers'] is None and s['nodes'] == []
    assert s['ros_error']


def test_dead_executor_refuses_rpc_and_stop_returns_without_service_timeouts():
    ros = adapter(); ros.thread = S(is_alive=lambda: False)
    nav = gateway(); nav.ros.executor_available = ros.executor_available
    before = time.monotonic()
    with pytest.raises(UserError, match='chưa được gửi'):
        nav._call(nav.clients['mode'], S(), timeout=5.0)
    result = nav.stop_all()
    assert result['ros_unavailable'] and not result['mux_confirmed']
    assert time.monotonic()-before < .2
    assert not nav.clients['mode'].requests and not nav.clients['stop'].requests
    assert not nav.control_pending


def test_dead_executor_cancel_and_manual_mode_do_not_claim_ready():
    nav = gateway()
    nav.ros.executor_available = lambda: False
    nav.ros.status = lambda: {'ros_ok': False, 'ros_error': 'executor failed'}
    nav.goal.update(state='ACTIVE', token=3)
    nav.handle = S(cancel_goal_async=lambda: pytest.fail('Dead executor must not enqueue cancel'))
    before = time.monotonic()
    result = nav.cancel_navigation()
    assert result['ros_unavailable'] and not result['confirmed']
    assert time.monotonic() - before < .1
    assert not nav.snapshot()['ready'] and not nav.snapshot()['mux']['manual_ready']


def test_confirmed_stop_retires_old_graph_and_queued_sensor_messages():
    ros = adapter(); ros.on_odom(odometry()); before = copy.deepcopy(ros.config)
    old = S(endpoint_gid=b'old', node_namespace='/', node_name='delivery_robot_motor')
    new = S(endpoint_gid=b'new', node_namespace='/', node_name='delivery_robot_motor')
    ros.filter_endpoints('/odom', [old]); ros.filter_endpoints('/cmd_vel', [old], 'subscription')
    callback = ros.subscription_callback('odom', '/odom', ros.on_odom)
    callback(odometry(), {'publisher_gid': b'old'})
    ros.reset_observations(['agent'])
    assert ros.odom is None and ros.odom_at == 0 and not ros.arrivals['odom']
    assert ros.status()['odom']['receive_age'] is None
    assert ros.filter_endpoints('/odom', [old]) == []
    assert ros.filter_endpoints('/cmd_vel', [old], 'subscription') == []
    callback(odometry(), {'publisher_gid': b'old'})
    assert ros.odom is None
    callback(odometry(), {'publisher_gid': b'new'})
    assert ros.odom is not None and ros.filter_endpoints('/odom', [old, new]) == [new]
    assert ros.config == before


@pytest.mark.parametrize('component', ['odom_bridge', 'ekf_odom', 'robot_model'])
def test_stopping_transform_producer_preserves_live_motor_telemetry(component):
    ros = adapter()
    old = S(endpoint_gid=b'motor-live', node_namespace='/', node_name='delivery_robot_motor')
    callback = ros.subscription_callback('odom', '/odom', ros.on_odom)
    ros.filter_endpoints('/odom', [old])
    callback(odometry(), {'publisher_gid': b'motor-live'})
    ros.on_safety(S(data=[0., -1., -1., 0., 0., 1., 0., 0., 0., 0., 0., 1.]))
    ros.reset_observations([component])
    assert ros.odom is not None and ros.safety is not None
    assert ros.filter_endpoints('/odom', [old]) == [old]
    replacement_sample = odometry(1_800_000_001)
    callback(replacement_sample, {'publisher_gid': b'motor-live'})
    assert ros.odom is replacement_sample
    assert ros.cached['tf_odom'] is None


def test_late_readiness_query_cannot_restore_old_state_after_confirmed_stop():
    nav = gateway(); nav.pending_queries = {}; nav.ros.observation_epoch = 0
    client = Client(); observations = []
    nav._query('lc:bt_navigator', client, S(), observations.append)
    nav.ros.observation_epoch += 1
    client.futures[0].set_result(S(current_state=S(label='active')))
    assert not observations
    nav._query('lc:bt_navigator', client, S(), observations.append)
    response = S(current_state=S(label='unconfigured'))
    client.futures[1].set_result(response)
    assert observations == [response]


def test_reset_only_clears_relevant_observations_and_rejects_previous_tf(monkeypatch):
    ros = adapter(); ros.on_odom(odometry())
    ros.scan = S(header=S(frame_id='laser', stamp=S(sec=1_800_000_000, nanosec=0)))
    ros.scan_at = time.monotonic(); ros.ros_map_msg = object(); ros.ros_map_at = time.monotonic()
    ros.reset_observations(['lidar'])
    assert ros.scan is None and ros.odom is not None and ros.ros_map_msg is not None
    tm = ModuleType('rclpy.time'); tm.Time = lambda: object()
    monkeypatch.setitem(sys.modules, 'rclpy.time', tm)
    transform = S(header=S(stamp=S(sec=1_799_999_999, nanosec=0)))
    ros.buffer = S(lookup_transform=lambda *args: transform)
    assert ros.lookup('odom', 'base_link') is None
    transform.header.stamp.sec = 1_800_000_001
    assert ros.lookup('odom', 'base_link') is transform
    ros.reset_observations(['localization'])
    assert ros.ros_map_msg is None and ros.ros_map_at == 0


def test_empty_stop_list_preserves_observations():
    ros = adapter(); sample = odometry(); ros.on_odom(sample)
    before = ros.observation_epoch
    assert ros.reset_observations([]) == {'cleared': []}
    assert ros.odom is sample and ros.observation_epoch == before


def test_executor_failure_report_and_explicit_recovery_need_new_heartbeat():
    ros = adapter(); ros.rclpy = S(ok=lambda: True)
    recovered, release = threading.Event(), threading.Event()
    attempts = []
    def spin():
        attempts.append(True)
        if len(attempts) == 1:
            raise RuntimeError('simulated unhandled executor failure')
        ros.executor_heartbeat = time.monotonic()
        recovered.set(); release.wait(2)
    ros.executor = S(spin=spin)
    ros._spin(); ros.thread = S(is_alive=lambda: False)
    assert not ros.status()['ros_ok'] and ros.status()['ros_executor']['state'] == 'failed'
    assert 'simulated unhandled' in ros.status()['ros_error']
    assert 'Traceback' in ros.logs[0]
    try:
        result = ros.recover_executor()
        assert result['restarted'] and recovered.wait(.2)
        assert ros.executor_error == '' and ros.status()['ros_ok']
        assert not ros.motion.status()['armed']
    finally:
        ros.closed = True; release.set(); ros.thread.join(timeout=1)


def test_recovery_never_spawns_second_thread_for_stalled_executor():
    ros = adapter(); ros.rclpy = S(ok=lambda: True); ros.executor_heartbeat -= 10
    with pytest.raises(UserError, match='thứ hai'):
        ros.recover_executor()

