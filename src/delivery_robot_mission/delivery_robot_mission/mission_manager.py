from __future__ import annotations
import math
import threading
import time
import uuid

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
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
        self.group = ReentrantCallbackGroup()
        self.declare_parameter('use_cmd_mux', True)
        self.declare_parameter('auto_confirm_pickup_seconds', -1.0)
        self.declare_parameter('auto_confirm_dropoff_seconds', -1.0)
        self.lock = threading.RLock()
        self.sm = StateMachine()
        self.mission_id = ''
        self.pickup_name = ''
        self.dropoff_name = ''
        self.pickup_pose = None
        self.dropoff_pose = None
        self.require_pickup = True
        self.require_dropoff = True
        self.active_goal = None
        self.goal_handle = None
        self.goal_token = 0
        self.distance_remaining = 0.0
        self.nav_error_code = 0
        self.wait_deadline = None
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
        self.create_timer(0.5, self.tick, callback_group=self.group)
        self.get_logger().info('Mission Manager ready')

    def _valid_pose(self, pose):
        vals = [pose.pose.position.x, pose.pose.position.y, pose.pose.orientation.x,
                pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
        return (pose.header.frame_id in ('', 'map') and all(math.isfinite(v) for v in vals))

    def start_cb(self, req, resp):
        with self.lock:
            if self.sm.active:
                resp.accepted = False; resp.message = 'A mission is already active'; return resp
            if not self._valid_pose(req.pickup_pose) or not self._valid_pose(req.dropoff_pose):
                resp.accepted = False; resp.message = 'Pickup/dropoff pose must be finite and in map frame'; return resp
            self.mission_id = uuid.uuid4().hex
            self.pickup_name = req.pickup_name or 'Pickup'
            self.dropoff_name = req.dropoff_name or 'Dropoff'
            self.pickup_pose = req.pickup_pose; self.dropoff_pose = req.dropoff_pose
            self.pickup_pose.header.frame_id = 'map'; self.dropoff_pose.header.frame_id = 'map'
            self.require_pickup = bool(req.require_pickup_confirmation)
            self.require_dropoff = bool(req.require_dropoff_confirmation)
            self.distance_remaining = 0.0; self.nav_error_code = 0; self.wait_deadline = None
            self.sm.start()
            resp.accepted = True; resp.mission_id = self.mission_id; resp.message = 'Mission accepted'
        self._enable_navigation(self._send_pickup)
        self.publish_state()
        return resp

    def _enable_navigation(self, continuation):
        if not bool(self.get_parameter('use_cmd_mux').value):
            continuation(); return
        if not self.mux.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn('Command mux service unavailable; continuing without mode switch')
            continuation(); return
        req = SetBool.Request(); req.data = True
        future = self.mux.call_async(req)
        def done(f):
            try:
                if not f.result().success:
                    self._fail('Command mux refused NAVIGATION mode'); return
            except Exception as exc:
                self._fail(f'Command mux error: {exc}'); return
            continuation()
        future.add_done_callback(done)

    def _send_pickup(self):
        self._send_goal(self.pickup_pose, 'pickup')

    def _send_dropoff(self):
        self._send_goal(self.dropoff_pose, 'dropoff')

    def _send_goal(self, pose, stage):
        if not self.nav.wait_for_server(timeout_sec=2.0):
            self._fail('/navigate_to_pose unavailable')
            return
        pose.header.stamp = self.get_clock().now().to_msg()
        goal = NavigateToPose.Goal()
        goal.pose = pose
        with self.lock:
            self.goal_token += 1
            token = self.goal_token
            self.active_goal = pose
        future = self.nav.send_goal_async(goal, feedback_callback=self.feedback_cb)
        future.add_done_callback(lambda f: self.goal_response(f, stage, token))
        self.publish_state()

    def goal_response(self, future, stage, token):
        with self.lock:
            if token != self.goal_token:
                return
        try:
            handle = future.result()
        except Exception as exc:
            self._fail(f'Navigation send error: {exc}'); return
        if not handle.accepted:
            self._fail(f'{stage} goal rejected'); return
        with self.lock:
            self.goal_handle = handle
        result = handle.get_result_async()
        result.add_done_callback(lambda f: self.goal_result(f, stage, token))

    def feedback_cb(self, feedback):
        with self.lock:
            self.distance_remaining = float(feedback.feedback.distance_remaining)
        self.publish_state()

    def goal_result(self, future, stage, token):
        with self.lock:
            if token != self.goal_token:
                return
        try:
            wrapped = future.result(); result = wrapped.result
            success = wrapped.status == GoalStatus.STATUS_SUCCEEDED and result.error_code == 0
        except Exception as exc:
            self._fail(f'Navigation result error: {exc}'); return
        with self.lock:
            # A late action result must not overwrite an operator cancel.
            if self.sm.state == CANCELED:
                self.goal_handle = None
                self.publish_state()
                return
            self.goal_handle = None
            self.nav_error_code = int(result.error_code)
            self.distance_remaining = 0.0
            if not success:
                self.sm.fail(f'{stage} navigation failed: {result.error_code} {result.error_msg}')
                stop = True; next_stage = None
            elif stage == 'pickup':
                self.sm.pickup_arrived(self.require_pickup)
                stop = False; next_stage = 'wait_pickup' if self.require_pickup else 'dropoff'
            else:
                self.sm.dropoff_arrived(self.require_dropoff)
                stop = self.sm.state == COMPLETED
                next_stage = 'wait_dropoff' if self.require_dropoff else None
        if next_stage == 'dropoff':
            self._send_dropoff()
        elif next_stage == 'wait_pickup':
            seconds = float(self.get_parameter('auto_confirm_pickup_seconds').value)
            if seconds >= 0: self.wait_deadline = time.monotonic() + seconds
        elif next_stage == 'wait_dropoff':
            seconds = float(self.get_parameter('auto_confirm_dropoff_seconds').value)
            if seconds >= 0: self.wait_deadline = time.monotonic() + seconds
        if stop: self._stop_mux()
        self.publish_state()

    def confirm_pickup_cb(self, _req, resp):
        with self.lock:
            try: self.sm.confirm_pickup()
            except ValueError as exc: resp.success = False; resp.message = str(exc); return resp
            self.wait_deadline = None
        self._send_dropoff(); resp.success = True; resp.message = 'Dropoff navigation started'; return resp

    def confirm_dropoff_cb(self, _req, resp):
        with self.lock:
            try: self.sm.confirm_dropoff()
            except ValueError as exc: resp.success = False; resp.message = str(exc); return resp
            self.wait_deadline = None
        self._stop_mux(); self.publish_state(); resp.success = True; resp.message = 'Mission completed'; return resp

    def cancel_cb(self, _req, resp):
        with self.lock:
            if not self.sm.active:
                resp.success = False
                resp.message = 'No active mission'
                return resp
            handle = self.goal_handle
            self.goal_token += 1
            self.goal_handle = None
            self.sm.cancel()
            self.wait_deadline = None
        if handle is not None:
            handle.cancel_goal_async()
        self._stop_mux()
        self.publish_state()
        resp.success = True
        resp.message = 'Mission canceled'
        return resp

    def reset_cb(self, _req, resp):
        with self.lock:
            try: self.sm.reset()
            except ValueError as exc: resp.success = False; resp.message = str(exc); return resp
            self.goal_token += 1
            self.goal_handle = None
            self.mission_id = ''; self.active_goal = None; self.nav_error_code = 0; self.distance_remaining = 0.0
        self.publish_state(); resp.success = True; resp.message = 'Mission reset'; return resp

    def _fail(self, message):
        with self.lock:
            self.goal_token += 1
            self.sm.fail(message); self.goal_handle = None
        self._stop_mux(); self.publish_state(); self.get_logger().error(message)

    def _stop_mux(self):
        if self.mux_stop.wait_for_service(timeout_sec=0.1): self.mux_stop.call_async(Trigger.Request())

    def tick(self):
        deadline = self.wait_deadline
        if deadline is not None and time.monotonic() >= deadline:
            with self.lock:
                state = self.sm.state; self.wait_deadline = None
            if state == WAITING_PICKUP:
                dummy = Trigger.Response(); self.confirm_pickup_cb(Trigger.Request(), dummy)
            elif state == WAITING_DROPOFF:
                dummy = Trigger.Response(); self.confirm_dropoff_cb(Trigger.Request(), dummy)
        self.publish_state()

    def publish_state(self):
        with self.lock:
            msg = MissionState(); msg.header.stamp = self.get_clock().now().to_msg(); msg.header.frame_id = 'map'
            msg.mission_id = self.mission_id; msg.state = self.sm.state; msg.state_name = NAMES[self.sm.state]
            msg.pickup_name = self.pickup_name; msg.dropoff_name = self.dropoff_name
            if self.active_goal is not None: msg.active_goal = self.active_goal
            msg.distance_remaining = float(self.distance_remaining); msg.nav_error_code = int(self.nav_error_code)
            msg.message = self.sm.message
        self.pub.publish(msg)


def main():
    rclpy.init(); node = MissionManager(); executor = MultiThreadedExecutor(num_threads=4); executor.add_node(node)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally: node._stop_mux(); executor.shutdown(); node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
