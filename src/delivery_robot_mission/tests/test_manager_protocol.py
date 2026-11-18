"""Exercise the mission protocol with manually completed ROS futures.

No ROS daemon, hardware, or network is used. The tests model response ordering
that is hard to reproduce reliably with a real delivery robot.
"""
import importlib.util
from pathlib import Path
from concurrent.futures import Future
from types import ModuleType, SimpleNamespace as S
import sys

import pytest


class Struct(S):
    def __init__(self, *args, **kwargs):
        super().__init__(**kwargs)


def pose(x=0.0, y=0.0, w=1.0):
    return S(header=S(frame_id='map', stamp=None),
             pose=S(position=S(x=x, y=y, z=0.0), orientation=S(x=0.0, y=0.0, z=0.0, w=w)))


class Client:
    def __init__(self):
        self.ready = True
        self.requests = []
        self.futures = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, req):
        f = Future(); self.requests.append(req); self.futures.append(f); return f

    def finish(self, index=-1, **values):
        self.futures[index].set_result(S(**values))


class Handle:
    accepted = True

    def __init__(self):
        self.result = Future()
        self.cancels = []

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        f = Future(); self.cancels.append(f); return f

    def finish(self, status=4, error_code=0, error_msg=''):
        self.result.set_result(S(status=status, result=S(error_code=error_code, error_msg=error_msg)))


class Nav:
    def __init__(self, *args, **kwargs):
        self.ready = True; self.sent = []

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal, feedback_callback=None):
        future = Future(); self.sent.append((goal, feedback_callback, future)); return future

    def accept(self, index=-1):
        handle = Handle(); self.sent[index][2].set_result(handle); return handle


class Node:
    def __init__(self, _name):
        self.parameters = {}; self.messages = []; self.errors = []

    def declare_parameter(self, name, value): self.parameters[name] = value
    def get_parameter(self, name): return S(value=self.parameters[name])
    def create_client(self, *args, **kwargs): return Client()
    def create_service(self, *args, **kwargs): pass
    def create_timer(self, *args, **kwargs): pass
    def create_publisher(self, *args, **kwargs): return S(publish=self.messages.append)
    def get_clock(self): return S(now=lambda: S(to_msg=lambda: S(sec=0, nanosec=0)))
    def get_logger(self): return S(info=lambda x: None, error=self.errors.append)


def message():
    return S(header=S(stamp=None, frame_id=''), active_goal=None)


@pytest.fixture
def manager(monkeypatch):
    modules = {
        'rclpy': {}, 'rclpy.action': {'ActionClient': Nav},
        'rclpy.callback_groups': {'MutuallyExclusiveCallbackGroup': Struct},
        'rclpy.executors': {'SingleThreadedExecutor': Struct},
        'rclpy.node': {'Node': Node},
        'rclpy.qos': {'QoSProfile': Struct, 'QoSDurabilityPolicy': S(TRANSIENT_LOCAL=1),
                      'QoSHistoryPolicy': S(KEEP_LAST=1), 'QoSReliabilityPolicy': S(RELIABLE=1)},
        'action_msgs': {}, 'action_msgs.msg': {'GoalStatus': S(STATUS_SUCCEEDED=4)},
        'std_srvs': {}, 'std_srvs.srv': {'SetBool': S(Request=Struct), 'Trigger': S(Request=Struct, Response=Struct)},
        'nav2_msgs': {}, 'nav2_msgs.action': {'NavigateToPose': S(Goal=Struct)},
        'delivery_robot_interfaces': {}, 'delivery_robot_interfaces.msg': {'MissionState': message},
        'delivery_robot_interfaces.srv': {'StartDelivery': S(Request=Struct, Response=Struct)},
    }
    for name, attrs in modules.items():
        mod = ModuleType(name); mod.__dict__.update(attrs); monkeypatch.setitem(sys.modules, name, mod)
    path = Path(__file__).parents[1] / 'delivery_robot_mission' / 'mission_manager.py'
    spec = importlib.util.spec_from_file_location('delivery_robot_mission._protocol_under_test', path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module.MissionManager()


def start(manager, **changes):
    req = S(pickup_pose=pose(), dropoff_pose=pose(1, 2), pickup_name='Kho', dropoff_name='Bàn A',
            require_pickup_confirmation=True, require_dropoff_confirmation=True)
    req.__dict__.update(changes)
    return manager.start_cb(req, S())


def enter_pickup(manager):
    assert start(manager).accepted
    manager.mux.finish(success=True, message='NAVIGATION')
    return manager.nav.accept()


def test_missing_mux_refuses_mission(manager):
    manager.mux.ready = False
    response = start(manager)
    assert not response.accepted
    assert not manager.nav.sent
    assert manager.sm.state == 0


def test_zero_quaternion_refused(manager):
    assert not start(manager, pickup_pose=pose(w=0)).accepted
    assert not manager.mux.requests


def test_nonfinite_pose_refused(manager):
    assert not start(manager, dropoff_pose=pose(float('nan'))).accepted


def test_full_delivery_requires_stop_and_cargo_confirmations(manager):
    pickup = enter_pickup(manager)
    pickup.finish()
    assert manager.sm.state == 2
    assert not manager.confirm_pickup_cb(S(), S()).success
    manager.mux_stop.finish(success=True, message='STOP')
    assert manager.confirm_pickup_cb(S(), S()).success
    assert len(manager.mux.requests) == 2
    assert len(manager.nav.sent) == 1
    manager.mux.finish(success=True, message='NAVIGATION')
    dropoff = manager.nav.accept()
    dropoff.finish()
    assert manager.sm.state == 4
    assert not manager.confirm_dropoff_cb(S(), S()).success
    manager.mux_stop.finish(success=True, message='STOP')
    assert manager.confirm_dropoff_cb(S(), S()).success
    assert manager.sm.state == 5
    assert not manager.sm.active


def test_late_mode_ack_after_cancel_never_sends_goal(manager):
    assert start(manager).accepted
    assert manager.cancel_cb(S(), S()).success
    manager.mux_stop.finish(success=True, message='STOP')
    assert manager.sm.state != 6  # The pending NAVIGATION call remains unresolved.
    manager.mux.finish(success=True, message='NAVIGATION')
    assert len(manager.mux_stop.requests) == 2
    assert not manager.nav.sent
    manager.mux_stop.finish(success=True, message='STOP')
    assert manager.sm.state == 6


def test_reassert_stop_after_out_of_order_mode_and_stop_responses(manager):
    assert start(manager).accepted
    manager.cancel_cb(S(), S())
    # A response for NAVIGATION arrives while the earlier STOP response is pending.
    manager.mux.finish(success=True, message='NAVIGATION')
    manager.mux_stop.finish(0, success=True, message='STOP')
    assert len(manager.mux_stop.requests) == 2
    assert manager.sm.state != 6
    manager.mux_stop.finish(1, success=True, message='STOP')
    assert manager.sm.state == 6
    assert not manager.nav.sent


def test_cancel_before_goal_acceptance_cancels_late_handle(manager):
    assert start(manager).accepted
    manager.mux.finish(success=True, message='NAVIGATION')
    manager.cancel_cb(S(), S())
    manager.mux_stop.finish(success=True, message='STOP')
    handle = manager.nav.accept()
    assert len(handle.cancels) == 1
    assert manager.sm.state != 6
    # Cancel acceptance alone is not the action's terminal result.
    handle.cancels[0].set_result(S(goals_canceling=[S()]))
    assert manager.sm.state != 6
    handle.finish(status=5)
    manager.mux_stop.finish(success=True, message='STOP')
    assert manager.sm.state == 6


def test_canceled_action_with_zero_error_is_not_success(manager):
    handle = enter_pickup(manager)
    handle.finish(status=5, error_code=0)
    assert manager.sm.state == 7
    assert not manager.nav.sent[1:]


def test_transport_timeout_blocks_new_mission_until_settled(manager):
    assert start(manager).accepted
    manager.operation_deadline = 0
    manager.tick()
    assert manager.sm.state == 7
    assert not start(manager).accepted
    manager.mux.finish(success=True, message='NAVIGATION')
    manager.mux_stop.finish(0, success=True, message='STOP')
    manager.mux_stop.finish(1, success=True, message='STOP')
    assert not manager.transport_unresolved
    assert not manager.nav.sent


def test_reset_refuses_pending_cancellation(manager):
    handle = enter_pickup(manager)
    manager.cancel_cb(S(), S())
    assert not manager.reset_cb(S(), S()).success
    assert len(handle.cancels) == 1


def test_late_feedback_from_old_mission_does_not_replace_new_distance(manager):
    manager.goal_token = 8
    manager.distance_remaining = 2.0
    manager.feedback_cb(S(feedback=S(distance_remaining=99.0)), 7)
    assert manager.distance_remaining == 2.0
