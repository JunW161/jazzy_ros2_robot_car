#!/usr/bin/env python3
"""Send /robot/heading_enable with feedback acknowledgement (Robot_Delivery_V12).

Run only when robot is IDLE and motor wheels fully stopped for guard reset.
Requires /robot/heading_status length >=42 and a connected micro-ROS Agent.
"""
import argparse
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float32MultiArray


class HeadingToggle(Node):
    def __init__(self):
        super().__init__('robot_delivery_v12_heading_toggle')
        self.pub = self.create_publisher(Bool, '/robot/heading_enable', 10)
        self.status = None
        self.stamp = 0.0
        self.create_subscription(Float32MultiArray, '/robot/heading_status',
                                 self.on_status, qos_profile_sensor_data)

    def on_status(self, msg):
        if len(msg.data) >= 42:
            self.status = list(msg.data)
            self.stamp = time.monotonic()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('on', 'off'))
    parser.add_argument('--timeout', type=float, default=9.0)
    args = parser.parse_args()
    if not 2 <= args.timeout <= 60:
        parser.error('--timeout must be 2..60 seconds')
    value = args.action == 'on'
    rclpy.init()
    node = HeadingToggle()
    try:
        start = time.monotonic()
        deadline = start + args.timeout
        baseline = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.status and time.monotonic() - node.stamp < 1:
                baseline = int(node.status[38])
                break
        if baseline is None:
            print('FAIL: no V12 /robot/heading_status (need length >= 42).')
            return 2
        # Discovery can lag behind the status subscriber. Do not fail on the
        # first graph sample; allow the publisher to discover ESP32.
        while node.pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.10)
        if node.pub.get_subscription_count() == 0:
            print('FAIL: no matching subscriber for /robot/heading_enable.')
            return 2
        print(f'V12 heading {args.action.upper()} request, baseline RX={baseline}')
        next_publish = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish:
                msg = Bool();msg.data = value
                node.pub.publish(msg)
                next_publish = now + 0.8
            rclpy.spin_once(node, timeout_sec=0.10)
            h = node.status
            if h is None or time.monotonic() - node.stamp > 1.0:
                continue
            rx, applied, resets, result = (int(h[n]) for n in (38,39,40,41))
            enabled, active, guard = (int(h[n]) for n in (10,11,29))
            if rx > baseline and applied >= rx:
                if ((result == 1 and value and guard == 0 and enabled == 1) or
                    (result == 2 and not value and enabled == 0)):
                    print(f'CONFIRMED: enabled={enabled} active={active} guard={guard} '
                          f'rx={rx} applied={applied} guard_resets={resets}')
                    return 0
                if result == -2:
                    print('OFF applied, waiting for safe IDLE to reset guard. Stop robot completely, then check status and issue OFF again if needed.')
                    return 3
                if result == -4:
                    print('BLOCKED: an earlier OFF has not been acknowledged at safe IDLE. Send OFF at IDLE, wait for CONFIRMED, then ON.')
                    return 3
                if result == -3 or guard != 0:
                    print(f'BLOCKED: guard={guard}; inspect sensors, stop robot, '
                          'send OFF at IDLE, then ON.')
                    return 3
        print('FAIL: no matching V12 acknowledgement; verify ROS Agent/QoS and state.')
        return 2
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
