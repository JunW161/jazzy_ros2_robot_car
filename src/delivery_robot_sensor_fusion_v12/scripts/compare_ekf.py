#!/usr/bin/env python3
"""Compare raw /odom with /odometry/filtered_v12 without changing TF ownership."""
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Compare(Node):
    def __init__(self):
        super().__init__('delivery_fusion_v12_compare')
        self.raw = None
        self.filt = None
        self.raw_count = 0
        self.filt_count = 0
        self.t0 = time.monotonic()
        self.raw_zero = None
        self.filt_zero = None
        self.create_subscription(Odometry, '/odom', self.raw_cb, 20)
        self.create_subscription(Odometry, '/odometry/filtered_v12', self.filt_cb, 20)
        self.create_timer(1.0, self.report)

    def raw_cb(self, m):
        self.raw = m
        self.raw_count += 1
        if self.raw_zero is None:
            self.raw_zero = (m.pose.pose.position.x, m.pose.pose.position.y, yaw(m.pose.pose.orientation))

    def filt_cb(self, m):
        self.filt = m
        self.filt_count += 1
        if self.filt_zero is None:
            self.filt_zero = (m.pose.pose.position.x, m.pose.pose.position.y, yaw(m.pose.pose.orientation))

    def report(self):
        elapsed = max(1e-6, time.monotonic() - self.t0)
        if self.raw is None or self.filt is None:
            print(f'waiting: raw={self.raw_count} filtered={self.filt_count}')
            return
        r = self.raw
        f = self.filt
        rz = self.raw_zero
        fz = self.filt_zero
        rdx = r.pose.pose.position.x - rz[0]
        rdy = r.pose.pose.position.y - rz[1]
        rdyaw = math.degrees(math.atan2(math.sin(yaw(r.pose.pose.orientation)-rz[2]), math.cos(yaw(r.pose.pose.orientation)-rz[2])))
        fdx = f.pose.pose.position.x - fz[0]
        fdy = f.pose.pose.position.y - fz[1]
        fdyaw = math.degrees(math.atan2(math.sin(yaw(f.pose.pose.orientation)-fz[2]), math.cos(yaw(f.pose.pose.orientation)-fz[2])))
        print(
            f'rate raw={self.raw_count/elapsed:5.1f}Hz filt={self.filt_count/elapsed:5.1f}Hz | '
            f'raw d=({rdx:+.3f},{rdy:+.3f},{rdyaw:+.1f}deg) vx={r.twist.twist.linear.x:+.3f} wz={r.twist.twist.angular.z:+.3f} | '
            f'ekf d=({fdx:+.3f},{fdy:+.3f},{fdyaw:+.1f}deg) vx={f.twist.twist.linear.x:+.3f} wz={f.twist.twist.angular.z:+.3f}'
        )


def main():
    rclpy.init()
    node = Compare()
    print('Compare mode only. EKF does NOT own odom->base_link TF.')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
