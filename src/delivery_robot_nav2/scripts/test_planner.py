#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import time

import rclpy
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class PlannerTest(Node):
    def __init__(self):
        super().__init__('delivery_robot_planner_test')
        self.client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')


def spin_until(node, future, timeout):
    end = time.monotonic() + timeout
    while rclpy.ok() and not future.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    return future.done()


def main() -> int:
    parser = argparse.ArgumentParser(description='Send a ComputePathToPose request without moving the robot.')
    parser.add_argument('--goal-x', type=float, default=-0.613)
    parser.add_argument('--goal-y', type=float, default=0.339)
    parser.add_argument('--goal-yaw', type=float, default=0.0, help='Degrees')
    parser.add_argument('--planner-id', default='GridBased')
    args = parser.parse_args()

    rclpy.init()
    node = PlannerTest()
    if not node.client.wait_for_server(timeout_sec=5.0):
        raise SystemExit('FAIL: /compute_path_to_pose unavailable')

    yaw = math.radians(args.goal_yaw)
    goal = ComputePathToPose.Goal()
    goal.goal.header.frame_id = 'map'
    goal.goal.header.stamp = node.get_clock().now().to_msg()
    goal.goal.pose.position.x = args.goal_x
    goal.goal.pose.position.y = args.goal_y
    goal.goal.pose.orientation.z = math.sin(yaw / 2.0)
    goal.goal.pose.orientation.w = math.cos(yaw / 2.0)
    goal.planner_id = args.planner_id
    goal.use_start = False

    send = node.client.send_goal_async(goal)
    if not spin_until(node, send, 5.0) or send.result() is None or not send.result().accepted:
        raise SystemExit('FAIL: planner rejected goal')
    result_future = send.result().get_result_async()
    if not spin_until(node, result_future, 10.0):
        raise SystemExit('FAIL: planner timeout')
    wrapped = result_future.result()
    result = wrapped.result
    path = result.path
    length = 0.0
    for i in range(1, len(path.poses)):
        a = path.poses[i - 1].pose.position
        b = path.poses[i].pose.position
        length += math.hypot(b.x - a.x, b.y - a.y)
    print(f'status={wrapped.status} error_code={result.error_code} error_msg={result.error_msg!r}')
    print(f'path_poses={len(path.poses)} path_length={length:.3f}m planning_time={result.planning_time.sec + result.planning_time.nanosec/1e9:.6f}s')
    ok = result.error_code == 0 and len(path.poses) > 1
    print('PLANNER TEST:', 'PASS' if ok else 'FAIL')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
