#!/usr/bin/env python3
from __future__ import annotations
import argparse
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import SetBool, Trigger


class RealTest(Node):
    def __init__(self):
        super().__init__('delivery_robot_navigation_real_short_test')
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.set_mode = self.create_client(SetBool, '/delivery_robot/cmd_mux/set_navigation')
        self.stop_mux = self.create_client(Trigger, '/delivery_robot/cmd_mux/stop')
        self.stop_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def emergency_stop(self):
        if self.stop_mux.wait_for_service(timeout_sec=0.2):
            self.stop_mux.call_async(Trigger.Request())
        zero = Twist()
        for _ in range(10):
            self.stop_pub.publish(zero)
            rclpy.spin_once(self, timeout_sec=0.03)


def spin_until(node, future, timeout):
    end = time.monotonic() + timeout
    while rclpy.ok() and not future.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    return future.done()


def main():
    p = argparse.ArgumentParser(description='REAL ROBOT short NavigateToPose test.')
    p.add_argument('--goal-x', type=float, required=True)
    p.add_argument('--goal-y', type=float, required=True)
    p.add_argument('--timeout', type=float, default=20.0)
    p.add_argument('--confirm', required=True, help='Must equal MOVE_ROBOT')
    args = p.parse_args()
    if args.confirm != 'MOVE_ROBOT':
        raise SystemExit('Refusing: use --confirm MOVE_ROBOT only after clearing the test area.')
    rclpy.init()
    node = RealTest()
    if node.set_mode.wait_for_service(timeout_sec=1.0):
        req = SetBool.Request(); req.data = True
        future = node.set_mode.call_async(req)
        spin_until(node, future, 2.0)
        if not future.done() or not future.result().success:
            node.emergency_stop(); raise SystemExit('FAIL: command mux did not enter NAVIGATION mode')
    if not node.nav.wait_for_server(timeout_sec=5.0):
        node.emergency_stop(); raise SystemExit('FAIL: /navigate_to_pose unavailable')
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = args.goal_x
    goal.pose.pose.position.y = args.goal_y
    goal.pose.pose.orientation.w = 1.0
    send = node.nav.send_goal_async(goal)
    if not spin_until(node, send, 5.0) or not send.result().accepted:
        node.emergency_stop(); raise SystemExit('FAIL: goal rejected')
    handle = send.result()
    result_future = handle.get_result_async()
    if not spin_until(node, result_future, args.timeout):
        handle.cancel_goal_async()
        node.emergency_stop()
        raise SystemExit('FAIL: timeout; goal canceled and stop sent')
    wrapped = result_future.result()
    node.emergency_stop()
    result = wrapped.result
    print(f'status={wrapped.status} error_code={result.error_code} error_msg={result.error_msg!r}')
    ok = result.error_code == 0
    print('REAL SHORT TEST:', 'PASS' if ok else 'FAIL')
    node.destroy_node(); rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
