"""Delivery orchestration with explicit cargo confirmations and command ownership.

All transport operations are asynchronous. Cancel is a request until the action
has terminated and the mux has acknowledged STOP; late callbacks never start the
next leg or revive a canceled mission.
"""
from __future__ import annotations
import copy
import math
import threading
import time
import uuid

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from std_srvs.srv import SetBool, Trigger
from nav2_msgs.action import NavigateToPose

from delivery_robot_interfaces.msg import MissionState
from delivery_robot_interfaces.srv import StartDelivery
from .state_machine import (
    StateMachine, NAMES, IDLE, GOING_TO_PICKUP, WAITING_PICKUP,
    GOING_TO_DROPOFF, WAITING_DROPOFF, COMPLETED, CANCELED, ERROR,
)


class MissionManager(Node):
    def __init__(self):
        super().__init__('delivery_robot_mission_manager')
        self.group = MutuallyExclusiveCallbackGroup()
        self.declare_parameter('use_cmd_mux', True)
        self.declare_parameter('auto_confirm_pickup_seconds', -1.0)
        self.declare_parameter('auto_confirm_dropoff_seconds', -1.0)
        self.declare_parameter('navigation_timeout_seconds', 180.0)
        self.declare_parameter('command_timeout_seconds', 5.0)
        self.lock = threading.RLock()
        self.sm = StateMachine()
        self.mission_id = ''
        self.pickup_name = self.dropoff_name = ''
        self.pickup_pose = self.dropoff_pose = self.active_goal = None
        self.require_pickup = self.require_dropoff = True
        self.goal_handle = None
        self.goal_token = 0
        self.pending_goal = self.pending_mode = False
        self.pending_stop = False
        self.stop_again = False
        self.stop_confirmed = False
        self.terminal_intent = None
        self.transport_unresolved = False
        self.distance_remaining = 0.0
        self.nav_error_code = 0
        self.wait_deadline = None
        self.operation_deadline = None
        self.navigation_deadline = None
        self.stop_deadline = None
        self.after_stop = None
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose', callback_group=self.group)
        self.mux = self.create_client(SetBool, '/delivery_robot/cmd_mux/set_navigation', callback_group=self.group)
        self.mux_stop = self.create_client(Trigger, '/delivery_robot/cmd_mux/stop', callback_group=self.group)
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MissionState, '/delivery_robot/mission/state', qos)
        self.create_service(StartDelivery, '/delivery_robot/mission/start', self.start_cb, callback_group=self.group)
        self.create_service(Trigger, '/delivery_robot/mission/confirm_pickup', self.confirm_pickup_cb, callback_group=self.group)
        self.create_service(Trigger, '/delivery_robot/mission/confirm_dropoff', self.confirm_dropoff_cb, callback_group=self.group)
        self.create_service(Trigger, '/delivery_robot/mission/cancel', self.cancel_cb, callback_group=self.group)
        self.create_service(Trigger, '/delivery_robot/mission/reset', self.reset_cb, callback_group=self.group)
        self.create_timer(0.2, self.tick, callback_group=self.group)
        self.get_logger().info('Mission Manager ready; cargo confirmation required by Web')

    def _duration(self, name):
        value = float(self.get_parameter(name).value)
        return value if math.isfinite(value) and value > 0 else 5.0

    def _uses_mux(self):
        return bool(self.get_parameter('use_cmd_mux').value)

    @staticmethod
    def _valid_pose(pose):
        p, q = pose.pose.position, pose.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        return (pose.header.frame_id in ('', 'map') and
                all(math.isfinite(v) for v in values) and
                abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.0) < 0.01)

    def _in_flight(self):
        return bool(self.pending_goal or self.pending_mode or self.goal_handle is not None)

    def start_cb(self, req, resp):
        with self.lock:
            if self.sm.active or self._in_flight() or self.pending_stop or self.transport_unresolved:
                resp.accepted = False; resp.message = 'A mission or cancellation is still active'; return resp
            if not self._valid_pose(req.pickup_pose) or not self._valid_pose(req.dropoff_pose):
                resp.accepted = False; resp.message = 'Both poses need finite values, a unit quaternion, and map frame'; return resp
            if not self.nav.server_is_ready():
                resp.accepted = False; resp.message = '/navigate_to_pose unavailable'; return resp
            if self._uses_mux() and not (self.mux.service_is_ready() and self.mux_stop.service_is_ready()):
                resp.accepted = False; resp.message = 'Command mux unavailable; mission refused'; return resp
            self.goal_token += 1
            self.mission_id = uuid.uuid4().hex
            self.pickup_name = (req.pickup_name or 'Pickup')[:80]
            self.dropoff_name = (req.dropoff_name or 'Dropoff')[:80]
            self.pickup_pose = copy.deepcopy(req.pickup_pose)
            self.dropoff_pose = copy.deepcopy(req.dropoff_pose)
            self.pickup_pose.header.frame_id = self.dropoff_pose.header.frame_id = 'map'
            self.require_pickup = bool(req.require_pickup_confirmation)
            self.require_dropoff = bool(req.require_dropoff_confirmation)
            self.distance_remaining = 0.0; self.nav_error_code = 0
            self.wait_deadline = self.navigation_deadline = None
            self.terminal_intent = self.after_stop = None
            self.stop_confirmed = False
            self.sm.start()
            resp.accepted = True; resp.mission_id = self.mission_id; resp.message = 'Mission accepted'
            token = self.goal_token
        self._enable_navigation(lambda: self._send_goal(self.pickup_pose, 'pickup', token), token)
        self.publish_state()
        return resp

    def _current(self, token):
        return token == self.goal_token and self.terminal_intent is None and self.sm.active

    def _enable_navigation(self, continuation, token):
        if not self._current(token):
            return
        if not self._uses_mux():
            continuation(); return
        if not self.mux.service_is_ready():
            self._fail('Command mux unavailable; mission stopped'); return
        self.pending_mode = True
        self.stop_confirmed = False
        self.operation_deadline = time.monotonic() + self._duration('command_timeout_seconds')
        req = SetBool.Request(); req.data = True
        try:
            future = self.mux.call_async(req)
        except Exception as exc:
            self.pending_mode = False
            self._fail(f'Command mux request error: {exc}'); return
        def done(f):
            if token != self.goal_token:
                # An old request cannot enable command authority for a new mission.
                self._stop_mux(force=True); return
            self.pending_mode = False; self.operation_deadline = None
            if not self._current(token):
                self._stop_mux(force=True); return
            try:
                response = f.result()
                if response is None or not response.success:
                    self._fail('Command mux refused NAVIGATION mode'); return
            except Exception as exc:
                self._fail(f'Command mux error: {exc}'); return
            continuation()
        future.add_done_callback(done)

    def _send_goal(self, pose, stage, token):
        if not self._current(token):
            return
        if not self.nav.server_is_ready():
            self._fail('/navigate_to_pose unavailable'); return
        pose = copy.deepcopy(pose)
        pose.header.stamp = self.get_clock().now().to_msg()
        goal = NavigateToPose.Goal(); goal.pose = pose
        self.active_goal = pose
        self.pending_goal = True
        self.operation_deadline = time.monotonic() + self._duration('command_timeout_seconds')
        try:
            future = self.nav.send_goal_async(goal, feedback_callback=lambda f: self.feedback_cb(f, token))
            future.add_done_callback(lambda f: self.goal_response(f, stage, token))
        except Exception as exc:
            self.pending_goal = False
            self._fail(f'Navigation send error: {exc}')
        self.publish_state()

    def goal_response(self, future, stage, token):
        try:
            handle = future.result()
        except Exception as exc:
            if token == self.goal_token:
                self.pending_goal = False
                self._fail(f'Navigation send error: {exc}')
            return
        if token != self.goal_token:
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
                self._stop_mux()
            return
        self.pending_goal = False; self.operation_deadline = None
        if handle is None or not handle.accepted:
            if self.terminal_intent is not None:
                self._settle_terminal()
            else:
                self._fail(f'{stage} goal rejected')
            return
        self.goal_handle = handle
        self.navigation_deadline = time.monotonic() + self._duration('navigation_timeout_seconds')
        result = handle.get_result_async()
        result.add_done_callback(lambda f: self.goal_result(f, stage, token))
        if not self._current(token):
            self._cancel_handle(handle)
            self._stop_mux()

    def feedback_cb(self, feedback, token):
        if self._current(token):
            value = float(feedback.feedback.distance_remaining)
            if math.isfinite(value):
                self.distance_remaining = max(0.0, value)

    def goal_result(self, future, stage, token):
        if token != self.goal_token:
            return
        self.goal_handle = None; self.navigation_deadline = None
        self.distance_remaining = 0.0
        if self.terminal_intent is not None:
            self._stop_mux(); self._settle_terminal(); return
        try:
            wrapped = future.result(); result = wrapped.result
            success = wrapped.status == GoalStatus.STATUS_SUCCEEDED and result.error_code == 0
            self.nav_error_code = int(result.error_code)
        except Exception as exc:
            self._fail(f'Navigation result error: {exc}'); return
        if not success:
            self._fail(f'{stage} navigation failed: {result.error_code} {result.error_msg}'); return
        if stage == 'pickup':
            self.sm.pickup_arrived(self.require_pickup)
            if self.require_pickup:
                self._stop_mux(after=lambda: self._begin_wait('auto_confirm_pickup_seconds'))
            else:
                self._send_goal(self.dropoff_pose, 'dropoff', token)
        else:
            # Always stop at dropoff before publishing COMPLETED or accepting a
            # cargo confirmation. Arrival alone is not proof of delivered cargo.
            self.sm.dropoff_arrived(True)
            self._stop_mux(after=(lambda: self._begin_wait('auto_confirm_dropoff_seconds'))
                           if self.require_dropoff else self._complete)
        self.publish_state()

    def _begin_wait(self, parameter):
        if self.terminal_intent is not None:
            return
        value = float(self.get_parameter(parameter).value)
        self.wait_deadline = time.monotonic() + value if math.isfinite(value) and value >= 0 else None
        self.publish_state()

    def _complete(self):
        if self.terminal_intent is None and self.sm.state == WAITING_DROPOFF and self.stop_confirmed:
            self.sm.confirm_dropoff()
            self.wait_deadline = None
            self.publish_state()

    def confirm_pickup_cb(self, _req, resp):
        if self.sm.state != WAITING_PICKUP or self.terminal_intent is not None:
            resp.success = False; resp.message = 'Not waiting for pickup confirmation'; return resp
        if not self.stop_confirmed or self.pending_stop:
            resp.success = False; resp.message = 'Waiting for Command Mux STOP acknowledgement'; return resp
        self.sm.confirm_pickup(); self.wait_deadline = None
        token = self.goal_token
        self._enable_navigation(lambda: self._send_goal(self.dropoff_pose, 'dropoff', token), token)
        self.publish_state()
        resp.success = True; resp.message = 'Pickup confirmed; dropoff navigation requested'
        return resp

    def confirm_dropoff_cb(self, _req, resp):
        if self.sm.state != WAITING_DROPOFF or self.terminal_intent is not None:
            resp.success = False; resp.message = 'Not waiting for dropoff confirmation'; return resp
        if not self.stop_confirmed or self.pending_stop:
            resp.success = False; resp.message = 'Waiting for Command Mux STOP acknowledgement'; return resp
        self._complete()
        resp.success = True; resp.message = 'Cargo receipt confirmed; mission completed'
        return resp

    def _cancel_handle(self, handle):
        try:
            future = handle.cancel_goal_async()
            def done(f):
                try:
                    response = f.result()
                    if not response.goals_canceling and self.goal_handle is handle:
                        self.sm.message = 'Cancel requested; awaiting the action terminal result'
                except Exception as exc:
                    self.sm.message = f'Cancel acknowledgement unavailable: {exc}'
            future.add_done_callback(done)
        except Exception as exc:
            self.sm.message = f'Cancel request error: {exc}'

    def cancel_cb(self, _req, resp):
        if not (self.sm.active or self._in_flight() or self.pending_stop or self.transport_unresolved):
            resp.success = True; resp.message = 'No active mission; STOP requested'
            self._stop_mux(); return resp
        self.terminal_intent = CANCELED
        self.wait_deadline = self.after_stop = None
        self.sm.message = 'Cancellation requested; waiting for Nav2 and Command Mux'
        if self.goal_handle is not None:
            self._cancel_handle(self.goal_handle)
        self._stop_mux()
        self._settle_terminal()
        self.publish_state()
        resp.success = True
        resp.message = 'Cancellation requested; observe mission state for confirmation'
        return resp

    def reset_cb(self, _req, resp):
        if self.sm.active or self._in_flight() or self.pending_stop or self.transport_unresolved:
            resp.success = False; resp.message = 'Cannot reset an active or unconfirmed operation'; return resp
        self.sm.reset()
        self.goal_token += 1
        self.goal_handle = self.active_goal = None
        self.mission_id = self.pickup_name = self.dropoff_name = ''
        self.nav_error_code = 0; self.distance_remaining = 0.0
        self.terminal_intent = None
        self.publish_state()
        resp.success = True; resp.message = 'Mission reset'
        return resp

    def _fail(self, message):
        self.terminal_intent = ERROR
        self.after_stop = self.wait_deadline = None
        self.sm.fail(message)
        if self.goal_handle is not None:
            self._cancel_handle(self.goal_handle)
        self._stop_mux()
        self.publish_state()
        self.get_logger().error(message)

    def _settle_terminal(self):
        if self.terminal_intent is None:
            return
        if self._in_flight() or self.pending_stop or not self.stop_confirmed:
            return
        self.transport_unresolved = False
        if self.terminal_intent == CANCELED:
            self.sm.cancel()
        # Keep the original error message when the terminal intent is ERROR.
        self.after_stop = self.wait_deadline = None
        self.publish_state()

    def _stop_mux(self, after=None, force=False):
        if after is not None:
            self.after_stop = after
        if self.pending_stop:
            self.stop_again = self.stop_again or force
            return
        if not self._uses_mux():
            self.stop_confirmed = True
            self._after_stop()
            return
        self.stop_confirmed = False
        if not self.mux_stop.service_is_ready():
            self.transport_unresolved = True
            self.sm.fail('Command Mux STOP unavailable; robot stop is not confirmed')
            self.terminal_intent = ERROR
            self.after_stop = None
            return
        self.pending_stop = True
        self.stop_deadline = time.monotonic() + self._duration('command_timeout_seconds')
        token = self.goal_token
        try:
            future = self.mux_stop.call_async(Trigger.Request())
        except Exception as exc:
            self.pending_stop = False; self.transport_unresolved = True
            self.sm.fail(f'STOP request failed: {exc}'); self.terminal_intent = ERROR
            return
        def done(f):
            if token != self.goal_token:
                return
            self.pending_stop = False; self.stop_deadline = None
            try:
                response = f.result()
                self.stop_confirmed = bool(response is not None and response.success)
            except Exception:
                self.stop_confirmed = False
            if not self.stop_confirmed:
                self.transport_unresolved = True; self.terminal_intent = ERROR
                self.after_stop = None
                self.sm.fail('Command Mux did not acknowledge STOP')
            self._after_stop()
        future.add_done_callback(done)

    def _after_stop(self):
        if self.stop_again:
            self.stop_again = False
            self._stop_mux()
            return
        if self.terminal_intent is not None:
            self._settle_terminal()
        elif self.stop_confirmed and self.after_stop:
            callback = self.after_stop; self.after_stop = None
            callback()
        self.publish_state()

    def tick(self):
        now = time.monotonic()
        if self.operation_deadline is not None and now >= self.operation_deadline:
            self.operation_deadline = None
            self.transport_unresolved = True
            self._fail('ROS command acknowledgement timed out; waiting for late response and STOP')
        if self.stop_deadline is not None and now >= self.stop_deadline:
            self.stop_deadline = None
            self.transport_unresolved = True
            self.terminal_intent = ERROR; self.after_stop = None
            self.sm.fail('STOP acknowledgement timed out; physical stop is not confirmed')
        if self.navigation_deadline is not None and now >= self.navigation_deadline:
            self.navigation_deadline = None
            self._fail('Navigation leg timed out; cancel requested')
        if self.wait_deadline is not None and now >= self.wait_deadline and self.terminal_intent is None:
            self.wait_deadline = None
            if self.sm.state == WAITING_PICKUP:
                self.confirm_pickup_cb(Trigger.Request(), Trigger.Response())
            elif self.sm.state == WAITING_DROPOFF:
                self.confirm_dropoff_cb(Trigger.Request(), Trigger.Response())
        self.publish_state()

    def publish_state(self):
        with self.lock:
            msg = MissionState()
            msg.header.stamp = self.get_clock().now().to_msg(); msg.header.frame_id = 'map'
            msg.mission_id = self.mission_id; msg.state = self.sm.state; msg.state_name = NAMES[self.sm.state]
            msg.pickup_name = self.pickup_name; msg.dropoff_name = self.dropoff_name
            if self.active_goal is not None:
                msg.active_goal = self.active_goal
            msg.distance_remaining = float(self.distance_remaining)
            msg.nav_error_code = int(self.nav_error_code)
            msg.message = self.sm.message
            if self.pending_stop and self.terminal_intent is None:
                msg.message += '; waiting for Command Mux STOP acknowledgement'
        self.pub.publish(msg)


def main():
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = MissionManager()
    executor = SingleThreadedExecutor(); executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel_cb(Trigger.Request(), Trigger.Response())
        deadline = time.monotonic() + 2.0
        try:
            while rclpy.ok() and time.monotonic() < deadline and (node._in_flight() or node.pending_stop):
                executor.spin_once(timeout_sec=0.05)
        except KeyboardInterrupt:
            pass
        executor.shutdown(); node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
