#!/usr/bin/env python3
"""Raised-wheels ONLY: bench motor test with reliable discovery/arming checks.
Never use with wheels touching the floor. Keeps firmware TTL / bench interlocks.
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray


class Bench(Node):
    def __init__(self):
        super().__init__('robot_v8_bench_wheel_test')
        self.arm = self.create_publisher(Bool, '/robot/bench_arm', 10)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.safety = self.wheels = None
        self.last_safety = self.last_wheels = 0.0
        self.create_subscription(Float32MultiArray, '/robot/safety_status',
                                 self.on_safety, qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, '/robot/wheel_debug',
                                 self.on_wheel, qos_profile_sensor_data)

    def on_safety(self, m):
        self.safety = list(m.data)
        self.last_safety = time.monotonic()

    def on_wheel(self, m):
        self.wheels = list(m.data)
        self.last_wheels = time.monotonic()

    def tick(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=max(0.0, min(0.03, end - time.monotonic())))

    def send_cmd(self, v=0.0, w=0.0):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd.publish(msg)

    def send_arm(self, enabled):
        msg = Bool()
        msg.data = enabled
        self.arm.publish(msg)

    def zeros(self, seconds=1.0):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.send_cmd()
            self.tick(0.05)

    def live(self):
        now = time.monotonic()
        return (self.safety is not None and len(self.safety) >= 12 and
                self.wheels is not None and len(self.wheels) >= 10 and
                now - self.last_safety < 0.8 and now - self.last_wheels < 0.8)

    def competing(self):
        names = [x.node_name for x in self.get_publishers_info_by_topic('/cmd_vel')
                 if x.node_name != self.get_name()]
        if names:
            raise RuntimeError('Competing /cmd_vel publisher: ' + str(names))

    def wait_ready(self, seconds=8.0):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.competing()
            self.send_cmd()
            self.tick(0.1)
            if (self.live() and self.cmd.get_subscription_count() >= 1 and
                    self.arm.get_subscription_count() >= 1):
                if int(self.safety[5]) != 1:
                    raise RuntimeError('Robot not IDLE: ' + str(self.safety))
                return
        raise RuntimeError('Missing live V8 safety/wheel telemetry or DDS subscriptions '
                           'after 8s; motors were NOT commanded')

    def wait_arm(self, seconds=4.0):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.competing()
            self.send_cmd()
            self.send_arm(True)
            self.tick(0.15)
            if self.live() and int(self.safety[9]) == 1 and self.safety[10] > 1000:
                return
        raise RuntimeError('No explicit ESP32 bench-arm acknowledgment; '
                           'motors were NOT commanded')

    def report(self):
        if not self.live():
            print('Telemetry not fresh', flush=True)
            return
        w, s = self.wheels, self.safety
        print(f'state={int(s[5])} interlock={int(s[8])} arm={int(s[9])} '
              f'fresh={int(s[11])} target_RPM={w[0]:+.1f}/{w[1]:+.1f} '
              f'measured_RPM={w[2]:+.1f}/{w[3]:+.1f} '
              f'PWM={w[4]:+.0f}/{w[5]:+.0f}', flush=True)


def main():
    p = argparse.ArgumentParser(description='BENCH ONLY: both wheels raised')
    p.add_argument('--wheels-raised-confirmed', action='store_true')
    p.add_argument('--stop-switch-confirmed', action='store_true')
    p.add_argument('--v', type=float, default=0.10)
    p.add_argument('--w', type=float, default=0.0)
    p.add_argument('--seconds', type=float, default=1.0)
    a = p.parse_args()
    if not (a.wheels_raised_confirmed and a.stop_switch_confirmed):
        p.error('Both raised-wheels and independent stop-switch confirmations required')
    if not all(math.isfinite(x) for x in (a.v, a.w, a.seconds)):
        p.error('Non-finite motion')
    if not (0 < a.seconds <= 2.0 and abs(a.v) <= 0.10 and abs(a.w) <= 0.30):
        p.error('Require seconds in (0,2], |v|<=0.10, |w|<=0.30')
    if a.v == 0 and a.w == 0:
        p.error('Set nonzero v or w')
    rclpy.init()
    b = Bench()
    try:
        print('BENCH: waiting for actual DDS discovery and live 12-field status...', flush=True)
        b.wait_ready()
        print('Bench telemetry verified. Requesting temporary arm...', flush=True)
        b.wait_arm()
        print('BENCH ARMED on ESP32; both wheels MUST remain raised.', flush=True)
        start = time.monotonic()
        next_print = start
        while time.monotonic() - start < a.seconds:
            b.competing()
            if not b.live():
                raise RuntimeError('Lost fresh status; stopping bench test')
            if int(b.safety[9]) != 1 or int(b.safety[5]) == 4:
                raise RuntimeError('Bench arm revoked or FAULT; stopping')
            b.send_cmd(a.v, a.w)
            b.tick(0.05)
            if time.monotonic() >= next_print:
                b.report()
                next_print = time.monotonic() + 0.25
    except KeyboardInterrupt:
        print('Stopped by operator', flush=True)
    finally:
        print('STOP: send zeros, disarm, send zeros', flush=True)
        try:
            b.zeros(1.0)
            for _ in range(5):
                b.send_arm(False)
                b.tick(0.05)
            b.zeros(1.0)
            b.report()
        finally:
            b.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
