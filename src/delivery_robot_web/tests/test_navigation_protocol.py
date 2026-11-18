"""Offline checks of Web command ownership, readiness and late ROS responses."""
from concurrent.futures import Future
from types import SimpleNamespace as S
import math
import threading
import time

import pytest

from delivery_robot_web.core import UserError
from delivery_robot_web.navigation_ros import (
    NavigationRos, OfflineNavigation, NAVIGATION_NODES, offline_snapshot, validate_pose,
)


class Client:
    def __init__(self, ready=True):
        self.ready = ready; self.futures = []; self.requests = []
    def service_is_ready(self): return self.ready
    def call_async(self, request):
        f = Future(); self.futures.append(f); self.requests.append(request); return f


class ImmediateQueue:
    def put(self, task): task()


def gateway():
    nav = NavigationRos.__new__(NavigationRos)
    nav.lock = threading.RLock(); nav.mux_condition = threading.Condition(nav.lock)
    nav.epoch = 0; nav.control_pending = set(); nav.control_requests = {}
    nav.goal_send_future = nav.goal_result_future = None
    nav.Trigger = S(Request=lambda: S())
    nav.clients = {name: Client() for name in ('mode', 'stop', 'start', 'cancel')}
    nav.ros = S(jobs=ImmediateQueue(), motion=S(stop=lambda *a, **k: None, status=lambda: {'armed': False}))
    nav.goal = offline_snapshot()['goal']; nav.goal_result_event = threading.Event()
    nav.handle = None; nav.mission_pending = None
    nav.mission = dict(offline_snapshot()['mission'], state_name='IDLE')
    nav.mission_at = time.monotonic(); nav.plan = {'frame_id': 'map', 'points': []}; nav.plan_at = 0.0
    nav.mux_mode = 'STOP'; nav.mux_at = time.monotonic()
    nav.params_at = time.monotonic()
    nav.params = {'output_topic': '/cmd_vel', 'manual_topic': '/cmd_vel_manual', 'nav_topic': '/cmd_vel_nav',
                  'max_linear': .32, 'max_angular': .35}
    nav.map_yaml = '/example/map.yaml'
    nav.lifecycle = {n: 'active' for n in (*NAVIGATION_NODES, 'amcl', 'map_server')}
    nav.lifecycle_at = {n: time.monotonic() for n in nav.lifecycle}
    nav.graph = {'motor_publishers': ['/delivery_robot_cmd_mux'], 'motor_receivers': ['/delivery_robot_motor'],
                 'nav_publishers': ['/collision_monitor'], 'nav_receivers': ['/delivery_robot_cmd_mux'],
                 'manual_receivers': ['/delivery_robot_cmd_mux'],
                 'nodes': ['/' + n for n in (*NAVIGATION_NODES, 'amcl', 'map_server',
                            'delivery_robot_cmd_mux', 'delivery_robot_mission_manager')]}
    nav.action_ids = set(); nav.action_providers = ('provider-a',); nav.action_observations = {}
    nav.last_action_providers = ('provider-a',); nav.retired_action_providers = set()
    nav.mission_provider = nav.mux_provider = None
    nav.retired_mission_providers = set(); nav.retired_mux_providers = set()
    nav.nav = S(server_is_ready=lambda: True)
    nav.ros.status = lambda: {'mapping_issues': [], 'motion_issues': [], 'tf_map': {'age': .01}, 'slam_state': 'offline'}
    return nav


def test_offline_never_claims_robot_ready():
    nav = OfflineNavigation()
    assert not nav.snapshot()['ready']
    assert not nav.stop_all()['mux_confirmed']
    for method, args in [(nav.navigate_to, ({},)), (nav.start_delivery, ({}, {})),
                         (nav.mission_action, ('confirm_pickup',)), (nav.set_manual, ())]:
        with pytest.raises(UserError): method(*args)


@pytest.mark.parametrize('value', [float('nan'), float('inf'), None, 'oops'])
def test_invalid_goal_coordinate_refused(value):
    with pytest.raises(UserError): validate_pose({'x': value, 'y': 0, 'yaw': 0})


def test_pose_normalizes_heading_without_changing_location():
    p = validate_pose({'name': 'Bàn A', 'x': 2, 'y': -1, 'yaw': 5 * math.pi})
    assert p['x'] == 2 and p['y'] == -1
    assert abs(abs(p['yaw']) - math.pi) < 1e-10


def test_ready_requires_real_mux_route_and_fresh_lifecycle():
    nav = gateway(); s = nav.snapshot()
    assert s['ready'] and s['mux']['manual_ready']
    nav.params['output_topic'] = '/cmd_vel_mux_test'
    assert not nav.snapshot()['ready']
    nav.params['output_topic'] = '/cmd_vel'
    nav.lifecycle_at['controller_server'] = time.monotonic() - 4
    assert not nav.snapshot()['ready']


def test_ready_refuses_duplicate_motor_or_nav_endpoint():
    nav = gateway()
    nav.graph['motor_receivers'].append('/delivery_robot_motor')
    assert not nav.snapshot()['ready']
    nav.graph['motor_receivers'].pop()
    nav.graph['nodes'].append('/bt_navigator')
    assert not nav.snapshot()['ready']


def test_stale_mission_never_available_even_if_service_exists():
    nav = gateway(); nav.mission_at = time.monotonic() - 10
    assert not nav.snapshot()['mission']['available']


def test_late_control_reply_keeps_reservation_until_stop_ack():
    nav = gateway(); client = Client(); late = []
    with pytest.raises(UserError):
        nav._call(client, S(), timeout=.01, epoch=0, late=lambda: late.append(True))
    assert nav.control_pending
    client.futures[0].set_result(S(success=True))
    assert late == [True]
    assert nav.control_pending
    nav.clients['stop'].futures[0].set_result(S(success=True))
    assert not nav.control_pending


def test_cancel_before_queued_request_never_sends_it():
    nav = gateway(); tasks = []
    nav.ros.jobs = S(put=lambda t: tasks.append(t))
    error = []
    def call():
        try: nav._call(nav.clients['mode'], S(), timeout=1, epoch=0)
        except UserError as exc: error.append(str(exc))
    thread = threading.Thread(target=call); thread.start()
    deadline = time.monotonic() + 1
    while not tasks and time.monotonic() < deadline:
        time.sleep(.001)
    nav.epoch += 1; tasks[0](); thread.join(timeout=1)
    assert not nav.clients['mode'].requests and error
    assert not nav.control_pending


def test_old_goal_result_does_not_stop_new_goal():
    nav = gateway(); stops = []; nav._stop_async = lambda: stops.append(True)
    nav.goal.update(token=9, state='ACTIVE')
    future = Future(); future.set_result(S(status=4, result=S(error_code=0, error_msg='')))
    nav._result(future, 8)
    assert nav.goal['state'] == 'ACTIVE' and not stops


def test_canceled_result_is_not_success_and_multiple_stop_epochs_settle():
    nav = gateway(); nav.epoch = 7
    nav.goal.update(token=2, state='CANCELING')
    nav._stop_async = lambda: None
    future = Future(); future.set_result(S(status=5, result=S(error_code=0, error_msg='')))
    nav._result(future, 2)
    assert nav.goal['state'] == 'CANCELED' and nav.goal_result_event.is_set()


def test_global_stop_invalidates_pending_dispatch_before_waiting():
    nav = gateway(); nav.epoch = 3; observed = []
    def call(*args, **kwargs):
        observed.append(nav.epoch)
        return S(success=True)
    nav._call = call
    nav.cancel_navigation = lambda: {'cancel_requested': False, 'confirmed': True}
    result = nav.stop_all()
    assert observed and all(epoch == 4 for epoch in observed)
    assert result['mux_confirmed'] and result['cancel_confirmed']


def test_plan_is_bounded_and_wrong_frame_ignored():
    nav = gateway()
    path = S(header=S(frame_id='map'), poses=[S(pose=S(position=S(x=float(i), y=0.))) for i in range(2000)])
    nav._on_plan(path)
    assert 1 <= len(nav.plan['points']) <= 600
    previous = nav.plan
    path.header.frame_id = 'odom'; nav._on_plan(path)
    assert nav.plan is previous


def test_manual_lease_authority_expires_or_changes():
    nav = gateway()
    assert not nav.manual_authority()
    nav._on_mux(S(data='MANUAL'))
    assert nav.manual_authority()
    nav._on_mux(S(data='NAVIGATION'))
    assert not nav.manual_authority()
    nav.mux_mode = 'MANUAL'; nav.mux_at = time.monotonic() - 2
    assert not nav.manual_authority()


def test_motor_lease_cannot_survive_external_mux_stop(monkeypatch):
    from collections import deque
    from types import ModuleType
    import queue
    import sys
    from delivery_robot_web.core import DEFAULT_CONFIG, MotionLease
    from delivery_robot_web.ros_adapter import RosAdapter
    tm = ModuleType('rclpy.time'); tm.Time = S(from_msg=lambda value: value)
    monkeypatch.setitem(sys.modules, 'rclpy.time', tm)
    ros = RosAdapter.__new__(RosAdapter)
    ros.config = DEFAULT_CONFIG.copy(); ros.motion = MotionLease(ros.config)
    ros.motion.arm('operator', True)
    ros.status = lambda: {}
    ros.motion_ready = lambda *_args: True
    ros.navigation = S(manual_authority=lambda: False)
    ros.lock = threading.RLock(); ros.jobs = queue.Queue()
    ros.last_stop = 0; ros.zeros_remaining = 0; ros.was_armed = True; ros.cmd_pub = None
    ros.last_graph = ros.last_service = time.monotonic() + 100
    ros.pending_scans = deque(); ros.gate_dropped = ros.gate_forwarded = 0
    commands = []
    ros.Twist = lambda: S(linear=S(x=0.0), angular=S(z=0.0))
    ros.cmd_qos = None
    ros.node = S(create_publisher=lambda *a: S(publish=commands.append), destroy_publisher=lambda p: None)
    ros.tick()
    assert not ros.motion.status()['armed']
    assert commands and all(m.linear.x == 0 and m.angular.z == 0 for m in commands)


def test_readonly_query_has_one_wait_and_cancels_stale_local_future():
    nav = gateway(); nav.pending_queries = {}; client = Client(); observed = []
    nav._query('state', client, S(), observed.append)
    first = client.futures[0]
    nav._query('state', client, S(), observed.append)
    assert len(client.futures) == 1
    nav.pending_queries['state'] = (first, time.monotonic() - 4)
    nav._query('state', client, S(), observed.append)
    assert first.cancelled() and len(client.futures) == 2
    client.futures[1].set_result('active')
    assert observed == ['active']


def status_message(ident=1, status=2):
    return S(status_list=[S(status=status, goal_info=S(goal_id=S(uuid=bytes([ident]))))])


def mission_message(state='GOING_TO_PICKUP', ident='delivery-1'):
    return S(mission_id=ident, state=1, state_name=state, pickup_name='A', dropoff_name='B',
             distance_remaining=1.2, nav_error_code=0, message='Moving')


def test_action_status_does_not_expire_during_discovery_gap():
    nav = gateway(); provider = b'old'
    nav.action_providers = nav.last_action_providers = (provider.hex(),)
    nav._on_action_status(status_message(), S(publisher_gid=provider))
    nav._observe_action_providers(())
    assert nav.action_ids == {'01'}
    nav._observe_action_providers((provider.hex(),))
    assert nav.action_ids == {'01'} and nav.snapshot()['external_goal_active']


def test_replacement_action_provider_clears_obsolete_external_goal():
    nav = gateway(); old, new = b'old', b'new'
    nav.action_providers = nav.last_action_providers = (old.hex(),)
    nav._on_action_status(status_message(), S(publisher_gid=old))
    nav._observe_action_providers((new.hex(),))
    assert not nav.action_ids
    nav._on_action_status(status_message(), S(publisher_gid=old))
    assert not nav.action_ids
    nav._on_action_status(status_message(2), S(publisher_gid=new))
    assert nav.action_ids == {'02'}


def test_owned_goal_is_interrupted_only_after_proven_server_replacement():
    nav = gateway(); nav.goal.update(state='ACTIVE', token=4, providers=('provider-a',))
    nav._observe_action_providers(())
    assert nav.goal['state'] == 'ACTIVE'
    nav._observe_action_providers(('provider-b',))
    assert nav.goal['state'] == 'FAILED' and nav.goal['retired']
    assert nav.goal_result_event.is_set()


def test_owned_shutdown_releases_uncertain_request_without_stopping_replacement():
    nav = gateway(); late = []
    with pytest.raises(UserError):
        nav._call(nav.clients['mode'], S(), timeout=.01, epoch=0, late=lambda: late.append(True))
    future = nav.clients['mode'].futures[0]
    # Model a transport callback already in flight when local cleanup occurs.
    future.cancel = lambda: False
    assert nav.control_pending
    nav.managed_shutdown_complete(('cmd_mux',))
    assert not nav.control_pending and not nav.control_requests
    future.set_result(S(success=True))
    assert not late and not nav.clients['stop'].requests


def test_owned_shutdown_marks_active_mission_interrupted_and_keeps_completed_result():
    nav = gateway()
    nav.mission.update(active=True, id='delivery-1', state_name='WAITING_PICKUP')
    result = nav.managed_shutdown_complete(('mission',))
    assert result['mission']['state_name'] == 'ERROR'
    assert result['mission']['id'] == 'delivery-1' and not result['mission']['active']
    nav.mission.update(active=False, id='delivery-2', state=5, state_name='COMPLETED')
    result = nav.managed_shutdown_complete(('mission',))
    assert result['mission']['state_name'] == 'COMPLETED' and result['mission']['id'] == 'delivery-2'


def test_delayed_old_provider_telemetry_cannot_revive_stopped_mission_or_mux():
    nav = gateway(); old, new = S(publisher_gid=b'old'), S(publisher_gid=b'new')
    nav._on_mission(mission_message(), old); nav._on_mux(S(data='NAVIGATION'), old)
    nav.managed_shutdown_complete(('mission', 'cmd_mux'))
    nav._on_mission(mission_message(), old); nav._on_mux(S(data='NAVIGATION'), old)
    assert nav.mission['state_name'] == 'ERROR' and nav.mux_mode == 'UNKNOWN'
    nav._on_mission(mission_message('IDLE', ''), new); nav._on_mux(S(data='STOP'), new)
    assert nav.mission['state_name'] == 'IDLE' and nav.mux_mode == 'STOP'


def test_confirm_arrived_dropoff_does_not_require_new_motion_readiness():
    nav = gateway(); nav.lifecycle_at['planner_server'] = 0
    nav.clients['confirm_dropoff'] = Client()
    nav.mission.update(state_name='WAITING_DROPOFF', active=True)
    nav._call = lambda *a, **k: S(success=True, message='Cargo received')
    assert nav.mission_action('confirm_dropoff')['success']
    nav.mission['state_name'] = 'WAITING_PICKUP'
    with pytest.raises(UserError, match='sẵn sàng'):
        nav.mission_action('confirm_pickup')


def test_queued_cancel_for_retired_goal_does_not_stop_new_goal():
    nav = gateway(); tasks = []
    nav.goal.update(state='ACTIVE', token=4, providers=('provider-a',))
    nav.ros.jobs = S(put=lambda task: tasks.append(task))
    thread = threading.Thread(target=nav.cancel_navigation); thread.start()
    deadline = time.monotonic() + 1
    while not tasks and time.monotonic() < deadline:
        time.sleep(.001)
    assert tasks
    nav.managed_shutdown_complete(('navigation',))
    nav.goal.update(state='ACTIVE', token=9, retired=False)
    tasks[0](); thread.join(timeout=1)
    assert not thread.is_alive() and not nav.clients['stop'].requests
