#!/usr/bin/env python3
import argparse, math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped


def main():
    p = argparse.ArgumentParser(description='Publish AMCL initial pose in map frame.')
    p.add_argument('--x', type=float, required=True)
    p.add_argument('--y', type=float, required=True)
    p.add_argument('--yaw', type=float, required=True, help='Yaw in degrees by default.')
    p.add_argument('--radians', action='store_true')
    a = p.parse_args()
    yaw = a.yaw if a.radians else math.radians(a.yaw)
    rclpy.init()
    n = Node('delivery_set_initial_pose')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
    pub = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', qos)
    m = PoseWithCovarianceStamped()
    m.header.frame_id = 'map'
    m.header.stamp = n.get_clock().now().to_msg()
    m.pose.pose.position.x = a.x; m.pose.pose.position.y = a.y
    m.pose.pose.orientation.z = math.sin(yaw/2.0); m.pose.pose.orientation.w = math.cos(yaw/2.0)
    # Conservative initial uncertainty: 0.25 m^2 for x/y, ~15 deg sigma for yaw.
    m.pose.covariance[0] = 0.25
    m.pose.covariance[7] = 0.25
    m.pose.covariance[35] = math.radians(15.0) ** 2
    end = time.monotonic() + 1.0
    while time.monotonic() < end:
        pub.publish(m)
        rclpy.spin_once(n, timeout_sec=0.05)
        time.sleep(0.1)
    print(f'Published initial pose: x={a.x:.3f} y={a.y:.3f} yaw={math.degrees(yaw):.1f} deg')
    n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
