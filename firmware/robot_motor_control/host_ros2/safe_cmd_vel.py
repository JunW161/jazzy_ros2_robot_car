#!/usr/bin/env python3
"""Supervised short motion test with ROS discovery, live state and guaranteed stop.

No obstacle avoidance exists yet: run only in a clear, controlled space with
an independent motor-power cut-off. This is not a safety-certified E-stop.
"""
import argparse
import math
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


def stopped_message():
    return Twist()


class MotionTest(Node):
    def __init__(self):
        super().__init__('robot_safe_cmd_test')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.safety, self.wheels = None, None
        self.safety_time = 0.0
        self.wheel_time = 0.0
        self.create_subscription(Float32MultiArray, '/robot/safety_status',
                                 self.on_safety, qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, '/robot/wheel_debug',
                                 self.on_wheel, qos_profile_sensor_data)

    def on_safety(self, msg):
        self.safety = list(msg.data)
        self.safety_time = time.monotonic()

    def on_wheel(self, msg):
        self.wheels = list(msg.data)
        self.wheel_time = time.monotonic()

    def spin(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=max(0.0, min(0.03, end - time.monotonic())))

    def send(self, v=0.0, w=0.0):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.publisher.publish(msg)

    def zero_for(self, seconds=1.6):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.send()
            self.spin(0.05)

    def check_competing_publishers(self):
        others = [p.node_name for p in self.get_publishers_info_by_topic('/cmd_vel')
                  if p.node_name != self.get_name()]
        if others:
            raise RuntimeError('Another /cmd_vel publisher is present: ' + str(others))

    def wait_ready(self, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.check_competing_publishers()
            self.send()  # keep watchdog zero and populate cmdEver
            self.spin(0.10)
            ok = (self.publisher.get_subscription_count() >= 1 and
                  self.safety is not None and len(self.safety) >= 12 and
                  time.monotonic() - self.safety_time < 0.8 and
                  self.wheels is not None and len(self.wheels) >= 10 and
                  time.monotonic() - self.wheel_time < 0.8)
            if ok:
                if int(self.safety[5]) != 1:
                    raise RuntimeError(f'Motor not IDLE: state={self.safety[5]}, '
                                       f'interlock={self.safety[8]}; do not drive')
                return
        raise RuntimeError('No complete live motor telemetry or /cmd_vel subscriber after '
                           f'{timeout:g}s. Check Agent, ROS node and QoS, no motor cmd sent.')

    def report(self):
        if self.wheels is None or self.safety is None:
            print('Waiting for wheel/safety reports', flush=True)
            return
        w, s = self.wheels, self.safety
        print(f'state={int(s[5])} interlock={int(s[8])} fresh={int(s[11])} '
              f'target_RPM={w[0]:+.1f}/{w[1]:+.1f} '
              f'measured_RPM={w[2]:+.1f}/{w[3]:+.1f} '
              f'PWM={w[4]:+.0f}/{w[5]:+.0f}', flush=True)


def interrupted_by_sigterm(_signum, _frame):
    raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--v', type=float, default=0.10)
    parser.add_argument('--w', type=float, default=0.0)
    parser.add_argument('--seconds', type=float, default=2.0,
                        help='Duration; 0 runs until Ctrl+C (supervised only)')
    parser.add_argument('--hz', type=float, default=20.0)
    parser.add_argument('--discovery-timeout', type=float, default=8.0)
    a = parser.parse_args()
    if not all(math.isfinite(x) for x in (a.v, a.w, a.seconds, a.hz, a.discovery_timeout)):
        parser.error('All values must be finite')
    if abs(a.v) > 0.48 or abs(a.w) > 2.0 or a.seconds < 0 or not 5 <= a.hz <= 50:
        parser.error('Invalid motion/hz limit')
    if a.discovery_timeout < 2 or a.discovery_timeout > 30:
        parser.error('discovery-timeout must be 2..30 seconds')
    if abs(a.v) < 1e-6 and abs(a.w) < 1e-6:
        parser.error('Use a nonzero command for a motion test')
    signal.signal(signal.SIGTERM, interrupted_by_sigterm)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = MotionTest()
    try:
        print('Checking ROS discovery, wheel/safety topics and other /cmd_vel publishers...', flush=True)
        node.wait_ready(a.discovery_timeout)
        print('READY: IDLE and telemetry live. Starting supervised motion.', flush=True)
        start = time.monotonic()
        next_report = start
        period = 1.0 / a.hz
        while a.seconds == 0 or time.monotonic() - start < a.seconds:
            node.check_competing_publishers()
            if node.safety is None or time.monotonic() - node.safety_time > 1.2:
                raise RuntimeError('Lost live safety-status telemetry; stopping command')
            if int(node.safety[5]) == 4 or int(node.safety[8]) in (3, 7, 8, 9, 11, 12, 13):
                raise RuntimeError(f'Firmware interlock={int(node.safety[8])} '
                                   f'state={int(node.safety[5])}; stopping')
            if time.monotonic() - start > 1.0 and int(node.safety[11]) != 1:
                raise RuntimeError('ESP32 has not reported a fresh /cmd_vel; stopping')
            if (time.monotonic() - start > 1.0 and int(node.safety[5]) == 1 and
                    node.wheels is not None and len(node.wheels) >= 10 and
                    abs(node.wheels[0]) < 0.1 and abs(node.wheels[1]) < 0.1):
                raise RuntimeError('Motor remains IDLE with zero target RPM after 1s; '
                                   'inspect /cmd_vel and interlock')
            node.send(a.v, a.w)
            node.spin(period)
            if time.monotonic() >= next_report:
                node.report()
                next_report = time.monotonic() + 0.4
    except KeyboardInterrupt:
        print('Interrupted by operator.', flush=True)
    finally:
        print('STOP: repeated zero Twist (1.6s). Confirm wheels stop physically.', flush=True)
        try:
            node.zero_for(1.6)
            node.report()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
