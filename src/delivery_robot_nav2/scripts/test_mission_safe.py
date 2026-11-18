#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import time

import rclpy
from delivery_robot_interfaces.msg import MissionState
from delivery_robot_interfaces.srv import StartDelivery
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_srvs.srv import Trigger


class MissionSafeTest(Node):
    def __init__(self):
        super().__init__('delivery_robot_mission_safe_test')
        self.state = None
        self.cmd_samples = []
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(MissionState, '/delivery_robot/mission/state', self._state, qos)
        self.create_subscription(Twist, '/cmd_vel_nav_test', self._cmd, 20)
        self.start_client = self.create_client(StartDelivery, '/delivery_robot/mission/start')
        self.cancel_client = self.create_client(Trigger, '/delivery_robot/mission/cancel')

    def _state(self, msg):
        self.state = msg

    def _cmd(self, msg):
        self.cmd_samples.append((float(msg.linear.x), float(msg.angular.z)))


def fill_pose(pose, node, x, y, yaw_deg):
    yaw = math.radians(yaw_deg)
    pose.header.frame_id = 'map'
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)


def spin_until(node, future, timeout):
    end = time.monotonic() + timeout
    while rclpy.ok() and not future.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    return future.done()


def main() -> int:
    parser = argparse.ArgumentParser(description='Start then cancel a pickup/dropoff mission in SAFE Nav2 mode.')
    parser.add_argument('--pickup-x', type=float, default=-0.613)
    parser.add_argument('--pickup-y', type=float, default=0.339)
    parser.add_argument('--dropoff-x', type=float, default=-0.613)
    parser.add_argument('--dropoff-y', type=float, default=0.339)
    parser.add_argument('--duration', type=float, default=3.0)
    args = parser.parse_args()

    rclpy.init()
    node = MissionSafeTest()
    if not node.start_client.wait_for_service(timeout_sec=5.0):
        raise SystemExit('FAIL: mission start service unavailable')
    request = StartDelivery.Request()
    request.pickup_name = 'SAFE_TEST_PICKUP'
    request.dropoff_name = 'SAFE_TEST_DROPOFF'
    fill_pose(request.pickup_pose, node, args.pickup_x, args.pickup_y, 0.0)
    fill_pose(request.dropoff_pose, node, args.dropoff_x, args.dropoff_y, 0.0)
    request.require_pickup_confirmation = True
    request.require_dropoff_confirmation = True
    future = node.start_client.call_async(request)
    if not spin_until(node, future, 5.0) or future.result() is None or not future.result().accepted:
        raise SystemExit('FAIL: mission was not accepted')
    print(f'mission_id={future.result().mission_id}')
    end = time.monotonic() + max(1.0, args.duration)
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    if node.cancel_client.wait_for_service(timeout_sec=1.0):
        cancel = node.cancel_client.call_async(Trigger.Request())
        spin_until(node, cancel, 2.0)
    nonzero = [(x, z) for x, z in node.cmd_samples if abs(x) > 1e-4 or abs(z) > 1e-4]
    max_x = max((abs(x) for x, _ in nonzero), default=0.0)
    max_z = max((abs(z) for _, z in nonzero), default=0.0)
    print(f'state={getattr(node.state, "state_name", "NO_STATE")} cmd_samples={len(node.cmd_samples)} nonzero={len(nonzero)} max|vx|={max_x:.3f} max|wz|={max_z:.3f}')
    ok = node.state is not None and max_x <= 0.36 and max_z <= 1.25
    print('MISSION SAFE TEST:', 'PASS' if ok else 'FAIL')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
