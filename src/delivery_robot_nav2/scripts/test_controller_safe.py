#!/usr/bin/env python3
from __future__ import annotations
import argparse
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from nav2_msgs.action import ComputePathToPose, FollowPath


class SafeControllerTest(Node):
    def __init__(self):
        super().__init__('delivery_robot_controller_safe_test')
        self.compute = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.follow = ActionClient(self, FollowPath, '/follow_path')
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=20,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        self.samples = []
        self.create_subscription(Twist, '/cmd_vel_nav_test', self._cmd, qos)

    def _cmd(self, msg: Twist):
        self.samples.append((time.monotonic(), float(msg.linear.x), float(msg.angular.z)))


def spin_until(node, future, timeout):
    end = time.monotonic() + timeout
    while rclpy.ok() and not future.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    return future.done()


def main():
    p = argparse.ArgumentParser(description='Compute a path, feed it to FollowPath, collect safe cmd_vel and cancel.')
    p.add_argument('--goal-x', type=float, default=-0.613)
    p.add_argument('--goal-y', type=float, default=0.339)
    p.add_argument('--duration', type=float, default=3.0)
    args = p.parse_args()

    rclpy.init()
    node = SafeControllerTest()
    if not node.compute.wait_for_server(timeout_sec=5.0):
        raise SystemExit('FAIL: /compute_path_to_pose unavailable')
    if not node.follow.wait_for_server(timeout_sec=5.0):
        raise SystemExit('FAIL: /follow_path unavailable')

    cgoal = ComputePathToPose.Goal()
    cgoal.goal.header.frame_id = 'map'
    cgoal.goal.header.stamp = node.get_clock().now().to_msg()
    cgoal.goal.pose.position.x = args.goal_x
    cgoal.goal.pose.position.y = args.goal_y
    cgoal.goal.pose.orientation.w = 1.0
    cgoal.planner_id = 'GridBased'
    cgoal.use_start = False
    send = node.compute.send_goal_async(cgoal)
    if not spin_until(node, send, 5.0) or not send.result().accepted:
        raise SystemExit('FAIL: planner rejected goal')
    result_f = send.result().get_result_async()
    if not spin_until(node, result_f, 10.0):
        raise SystemExit('FAIL: planner result timeout')
    result = result_f.result().result
    if result.error_code != 0 or not result.path.poses:
        raise SystemExit(f'FAIL: planner error={result.error_code} {result.error_msg}')
    print(f'Path poses: {len(result.path.poses)}')

    fgoal = FollowPath.Goal()
    fgoal.path = result.path
    fgoal.controller_id = 'FollowPath'
    fgoal.goal_checker_id = 'goal_checker'
    fgoal.progress_checker_id = 'progress_checker'
    if hasattr(fgoal, 'path_handler_id'):
        fgoal.path_handler_id = ''
    follow_send = node.follow.send_goal_async(fgoal)
    if not spin_until(node, follow_send, 5.0) or not follow_send.result().accepted:
        raise SystemExit('FAIL: controller rejected path')
    handle = follow_send.result()
    end = time.monotonic() + args.duration
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    cancel = handle.cancel_goal_async()
    spin_until(node, cancel, 2.0)

    nonzero = [(x, z) for _, x, z in node.samples if abs(x) > 1e-4 or abs(z) > 1e-4]
    max_x = max((abs(x) for x, _ in nonzero), default=0.0)
    max_z = max((abs(z) for _, z in nonzero), default=0.0)
    print(f'cmd samples={len(node.samples)} nonzero={len(nonzero)} max|vx|={max_x:.3f} max|wz|={max_z:.3f}')
    ok = bool(nonzero) and max_x <= 0.36 and max_z <= 1.25
    print('CONTROLLER SAFE TEST:', 'PASS' if ok else 'FAIL')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
