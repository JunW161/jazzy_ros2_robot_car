#!/usr/bin/env python3
from __future__ import annotations
import argparse
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose


class SafeNavigationTest(Node):
    def __init__(self):
        super().__init__('delivery_robot_navigation_safe_test')
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.samples = []
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=30,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(Twist, '/cmd_vel_nav_test', self._cmd, qos)

    def _cmd(self, msg):
        self.samples.append((float(msg.linear.x), float(msg.angular.z)))


def spin_until(node, future, timeout):
    end = time.monotonic() + timeout
    while rclpy.ok() and not future.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    return future.done()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--goal-x', type=float, default=-0.613)
    p.add_argument('--goal-y', type=float, default=0.339)
    p.add_argument('--duration', type=float, default=3.0)
    args = p.parse_args()
    rclpy.init()
    node = SafeNavigationTest()
    if not node.client.wait_for_server(timeout_sec=5.0):
        raise SystemExit('FAIL: /navigate_to_pose unavailable')
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = args.goal_x
    goal.pose.pose.position.y = args.goal_y
    goal.pose.pose.orientation.w = 1.0
    send = node.client.send_goal_async(goal)
    if not spin_until(node, send, 5.0) or not send.result().accepted:
        raise SystemExit('FAIL: NavigateToPose rejected')
    handle = send.result()
    end = time.monotonic() + args.duration
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    cancel = handle.cancel_goal_async()
    spin_until(node, cancel, 2.0)
    nonzero = [(x, z) for x, z in node.samples if abs(x) > 1e-4 or abs(z) > 1e-4]
    max_x = max((abs(x) for x, _ in nonzero), default=0.0)
    max_z = max((abs(z) for _, z in nonzero), default=0.0)
    print(f'cmd samples={len(node.samples)} nonzero={len(nonzero)} max|vx|={max_x:.3f} max|wz|={max_z:.3f}')
    ok = bool(nonzero) and max_x <= 0.36 and max_z <= 1.25
    print('NAVIGATION SAFE TEST:', 'PASS' if ok else 'FAIL')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
