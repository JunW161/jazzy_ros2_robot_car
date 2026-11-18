#!/usr/bin/env python3
"""Non-invasive preflight for Robot_Delivery_V13 EKF inputs."""
import math
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class CheckInputs(Node):
    def __init__(self):
        super().__init__('delivery_fusion_v13_input_check')
        self.odom = []
        self.imu = []
        self.odom_frames = set()
        self.imu_frames = set()
        self.imu_orientation_unknown = []
        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(Odometry, '/odom', self.on_odom, odom_qos)
        self.create_subscription(Imu, '/imu/data_raw', self.on_imu, qos_profile_sensor_data)

    @staticmethod
    def age_ms(stamp):
        now_ns = time.time_ns()
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        return (now_ns - stamp_ns) / 1e6

    def on_odom(self, msg):
        self.odom.append((
            time.monotonic(),
            self.age_ms(msg.header.stamp),
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.covariance[0],
            msg.twist.covariance[7],
        ))
        self.odom_frames.add((msg.header.frame_id, msg.child_frame_id))

    def on_imu(self, msg):
        self.imu.append((time.monotonic(), self.age_ms(msg.header.stamp), msg.angular_velocity.z))
        self.imu_frames.add(msg.header.frame_id)
        self.imu_orientation_unknown.append(msg.orientation_covariance[0] < 0.0)


def rate(samples):
    if len(samples) < 2:
        return 0.0
    dt = samples[-1][0] - samples[0][0]
    return (len(samples) - 1) / dt if dt > 0 else 0.0


def stats(values):
    if not values:
        return 'NO DATA'
    return f'avg={statistics.mean(values):.1f} ms min={min(values):.1f} ms max={max(values):.1f} ms'


def main():
    rclpy.init()
    node = CheckInputs()
    duration = 8.0
    print(f'Checking /odom and /imu/data_raw for {duration:.0f} s. Keep robot stationary for this preflight...')
    end = time.monotonic() + duration
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    print('\n=== Robot_Delivery_V13 EKF input preflight ===')
    print(f'/odom: {len(node.odom)} samples, {rate(node.odom):.2f} Hz, frames={sorted(node.odom_frames)}')
    print(f'  stamp age: {stats([x[1] for x in node.odom])}')
    if node.odom:
        vy = [x[3] for x in node.odom if math.isfinite(x[3])]
        cvx = [x[4] for x in node.odom if math.isfinite(x[4])]
        cvy = [x[5] for x in node.odom if math.isfinite(x[5])]
        if vy:
            print(f'  nonholonomic vy: mean={statistics.mean(vy):+.8f} m/s max_abs={max(abs(v) for v in vy):.8f}')
        if cvx and cvy:
            print(f'  twist covariance: vx={statistics.median(cvx):.6f} vy={statistics.median(cvy):.6f}')

    print(f'/imu/data_raw: {len(node.imu)} samples, {rate(node.imu):.2f} Hz, frames={sorted(node.imu_frames)}')
    print(f'  stamp age: {stats([x[1] for x in node.imu])}')
    if node.imu:
        gz = [x[2] for x in node.imu if math.isfinite(x[2])]
        if gz:
            print(f'  stationary gyro_z: mean={statistics.mean(gz):+.6f} rad/s std={statistics.pstdev(gz):.6f}')

    problems = []
    if len(node.odom) < 20:
        problems.append('Too few /odom samples. Expect about 10 Hz from current firmware.')
    if len(node.imu) < 80:
        problems.append('Too few IMU samples. Current firmware normally provides tens of Hz when fresh.')
    if node.odom_frames and node.odom_frames != {('odom', 'base_link')}:
        problems.append(f'Unexpected /odom frames: {node.odom_frames}')
    if node.imu_frames and node.imu_frames != {'imu_link'}:
        problems.append(f'Unexpected IMU frame(s): {node.imu_frames}')
    if node.imu_orientation_unknown and not all(node.imu_orientation_unknown):
        problems.append('IMU orientation unexpectedly marked available; V13 expects no quaternion orientation.')
    if node.odom:
        vy = [x[3] for x in node.odom if math.isfinite(x[3])]
        cvy = [x[5] for x in node.odom if math.isfinite(x[5])]
        if not vy or max(abs(v) for v in vy) > 1e-4:
            problems.append('Raw /odom linear.y is not approximately zero; do not fuse vy=0 until reviewed.')
        if not cvy or statistics.median(cvy) <= 0.0 or statistics.median(cvy) >= 100.0:
            problems.append('Raw /odom vy covariance is not a usable finite constraint.')

    if problems:
        print('\nPREFLIGHT: REVIEW')
        for p in problems:
            print(f' - {p}')
    else:
        print('\nPREFLIGHT: V13 INPUT CONTRACT OK')
        print('V13 may fuse vx and the measured nonholonomic vy=0 constraint from /odom, plus gyro Z from IMU.')
        print('Timestamp age is reported, not corrected. Large age must still be fixed at the source before Nav2.')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
