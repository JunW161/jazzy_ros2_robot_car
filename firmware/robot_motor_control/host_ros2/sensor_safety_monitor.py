#!/usr/bin/env python3
"""ROS2 passive observer. Never publishes /cmd_vel or actuates a motor."""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Float32MultiArray


class Monitor(Node):
    def __init__(self):
        super().__init__('sensor_safety_monitor')
        self.imu = None
        self.distance = None
        self.status = None
        self.last = {}
        self.create_subscription(Imu, '/imu/data_raw', lambda m:self.on('imu', m), qos_profile_sensor_data)
        self.create_subscription(Range, '/front_range', lambda m:self.on('range', m), qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, '/robot/safety_status', lambda m:self.on('safety', m), qos_profile_sensor_data)
        self.create_timer(1.0, self.report)

    def on(self, name, msg):
        self.last[name] = time.monotonic()
        if name == 'imu': self.imu = msg.angular_velocity.z
        if name == 'range': self.distance = msg.range
        if name == 'safety': self.status = list(msg.data)

    def report(self):
        now=time.monotonic()
        ages={k: round(now-self.last[k], 2) for k in self.last}
        self.get_logger().info('gyro_z=%s dist=%s safety=%s age=%s' %
                               (self.imu,self.distance,self.status,ages))


def main():
    rclpy.init()
    node=Monitor()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()

if __name__ == '__main__':main()
