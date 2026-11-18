#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger


def main():
    rclpy.init()
    node = Node('delivery_robot_stop_robot')
    client = node.create_client(Trigger, '/delivery_robot/cmd_mux/stop')
    if client.wait_for_service(timeout_sec=0.3):
        client.call_async(Trigger.Request())
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    zero = Twist()
    end = time.monotonic() + 0.6
    while time.monotonic() < end:
        pub.publish(zero)
        rclpy.spin_once(node, timeout_sec=0.03)
    print('Zero velocity sent; command mux STOP requested when available.')
    node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
