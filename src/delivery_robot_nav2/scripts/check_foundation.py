#!/usr/bin/env python3
from __future__ import annotations

import argparse
import statistics
import time

import rclpy
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformListener


BEST_EFFORT = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class FoundationCheck(Node):
    def __init__(self):
        super().__init__('delivery_robot_foundation_check')
        self.samples: dict[str, list[float]] = {
            'raw_odom': [],
            'imu': [],
            'filtered': [],
            'scan': [],
        }
        self.arrivals: dict[str, list[float]] = {key: [] for key in self.samples}
        self.create_subscription(Odometry, '/odom', lambda m: self._sample('raw_odom', m.header.stamp), BEST_EFFORT)
        self.create_subscription(Imu, '/imu/data_raw', lambda m: self._sample('imu', m.header.stamp), BEST_EFFORT)
        self.create_subscription(Odometry, '/odometry/filtered', lambda m: self._sample('filtered', m.header.stamp), BEST_EFFORT)
        self.create_subscription(LaserScan, '/scan', lambda m: self._sample('scan', m.header.stamp), BEST_EFFORT)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _sample(self, name, stamp):
        now = self.get_clock().now().nanoseconds / 1e9
        msg_time = stamp.sec + stamp.nanosec / 1e9
        self.samples[name].append((now - msg_time) * 1000.0)
        self.arrivals[name].append(time.monotonic())

    def lifecycle(self, node_name: str) -> str:
        client = self.create_client(GetState, f'/{node_name}/get_state')
        if not client.wait_for_service(timeout_sec=0.5):
            return 'MISSING'
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        if not future.done() or future.result() is None:
            return 'NO_RESPONSE'
        return future.result().current_state.label


def main() -> int:
    parser = argparse.ArgumentParser(description='Check the stable robot foundation before any Nav2 stage.')
    parser.add_argument('--seconds', type=float, default=8.0)
    parser.add_argument('--require-localization', action='store_true')
    args = parser.parse_args()

    rclpy.init()
    node = FoundationCheck()
    end = time.monotonic() + max(2.0, args.seconds)
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    print('=== DELIVERY ROBOT FOUNDATION CHECK ===')
    ok = True
    minimum_hz = {'raw_odom': 7.0, 'imu': 25.0, 'filtered': 20.0, 'scan': 4.5}
    maximum_avg_age_ms = {'raw_odom': 150.0, 'imu': 150.0, 'filtered': 80.0, 'scan': 100.0}
    for name in ('raw_odom', 'imu', 'filtered', 'scan'):
        ages = node.samples[name]
        arrivals = node.arrivals[name]
        if not ages:
            print(f'{name:10s}: NO DATA')
            ok = False
            continue
        hz = 0.0
        if len(arrivals) > 1 and arrivals[-1] > arrivals[0]:
            hz = (len(arrivals) - 1) / (arrivals[-1] - arrivals[0])
        avg = statistics.mean(ages)
        print(
            f'{name:10s}: n={len(ages):4d} hz={hz:6.2f} '
            f'age avg={avg:7.1f}ms min={min(ages):7.1f} max={max(ages):7.1f}'
        )
        if hz < minimum_hz[name] or avg > maximum_avg_age_ms[name]:
            ok = False

    for target, source in (('odom', 'base_link'), ('base_link', 'base_laser')):
        try:
            node.tf_buffer.lookup_transform(target, source, Time())
            print(f'TF {target} -> {source}: OK')
        except Exception as exc:
            print(f'TF {target} -> {source}: FAIL ({exc})')
            ok = False

    if args.require_localization:
        amcl = node.lifecycle('amcl')
        map_server = node.lifecycle('map_server')
        print(f'amcl:       {amcl}')
        print(f'map_server: {map_server}')
        ok &= amcl == 'active' and map_server == 'active'
        try:
            node.tf_buffer.lookup_transform('map', 'base_link', Time())
            print('TF map -> base_link: OK')
        except Exception as exc:
            print(f'TF map -> base_link: FAIL ({exc})')
            ok = False

    node.destroy_node()
    rclpy.shutdown()
    print('FOUNDATION CHECK:', 'PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
