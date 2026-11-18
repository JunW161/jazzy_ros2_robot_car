#!/usr/bin/env python3
from __future__ import annotations
import argparse
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.time import Time
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class Checker(Node):
    def __init__(self):
        super().__init__('delivery_robot_nav2_checker')
        best_effort = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                                 reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.odom_seen = False
        self.scan_seen = False
        self.create_subscription(Odometry, '/odometry/filtered', self._odom, best_effort)
        self.create_subscription(LaserScan, '/scan', self._scan, best_effort)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.compute = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.follow = ActionClient(self, FollowPath, '/follow_path')
        self.navigate = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def _odom(self, _):
        self.odom_seen = True

    def _scan(self, _):
        self.scan_seen = True

    def lifecycle(self, name: str) -> str:
        client = self.create_client(GetState, f'/{name}/get_state')
        if not client.wait_for_service(timeout_sec=0.5):
            return 'MISSING'
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        if not future.done() or future.result() is None:
            return 'NO_RESPONSE'
        return future.result().current_state.label


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--stage', choices=['planner', 'controller', 'navigation'], default='navigation')
    args = parser.parse_args()
    rclpy.init()
    node = Checker()
    end = time.monotonic() + 3.0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    ok = True
    print('=== DELIVERY ROBOT NAV2 CHECK ===')
    print(f'/odometry/filtered data: {node.odom_seen}')
    print(f'/scan data:              {node.scan_seen}')
    ok &= node.odom_seen and node.scan_seen

    for target, source in [('odom', 'base_link'), ('map', 'base_link')]:
        try:
            node.tf_buffer.lookup_transform(target, source, Time())
            print(f'TF {target} -> {source}: OK')
        except Exception as exc:
            print(f'TF {target} -> {source}: FAIL ({exc})')
            ok = False

    required = ['planner_server']
    if args.stage in ('controller', 'navigation'):
        required += ['controller_server']
    if args.stage == 'navigation':
        required += ['behavior_server', 'velocity_smoother', 'collision_monitor', 'bt_navigator']
    for name in required:
        state = node.lifecycle(name)
        print(f'{name}: {state}')
        ok &= state == 'active'

    print(f'/compute_path_to_pose: {node.compute.wait_for_server(timeout_sec=0.2)}')
    if args.stage in ('controller', 'navigation'):
        follow = node.follow.wait_for_server(timeout_sec=0.2)
        print(f'/follow_path:           {follow}')
        ok &= follow
    if args.stage == 'navigation':
        nav = node.navigate.wait_for_server(timeout_sec=0.2)
        print(f'/navigate_to_pose:      {nav}')
        ok &= nav

    node.destroy_node()
    rclpy.shutdown()
    print('NAV2 CHECK:', 'PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
