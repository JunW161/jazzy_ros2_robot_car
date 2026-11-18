#!/usr/bin/env python3
"""Bridge encoder odom to TF and wheel debug to JointState for RViz.
Disable publish_tf if an EKF or simulator already owns odom -> base_link.
"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class OdomTfJointBridge(Node):
    def __init__(self):
        super().__init__('delivery_odom_tf_joint_bridge')
        self.declare_parameter('publish_tf',True)
        self.declare_parameter('publish_joint_states',True)
        self.declare_parameter('left_joint','left_wheel_joint')
        self.declare_parameter('right_joint','right_wheel_joint')
        self.declare_parameter('wheel_radius_m',0.044)
        self.odom_seen=False
        self.tf_pub=TransformBroadcaster(self)
        self.joint_pub=self.create_publisher(JointState,'/joint_states',10)
        self.create_subscription(Odometry,'/odom',self.on_odom,10)
        self.create_subscription(Float32MultiArray,'/robot/wheel_debug',self.on_debug,
            QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=10,
                       reliability=ReliabilityPolicy.BEST_EFFORT))
        self.get_logger().warning('Run this bridge ONLY if no other node publishes odom->base_link TF. '
                                  'Match left/right joint names to your URDF.')

    def on_odom(self,m):
        self.odom_seen=True
        if not self.get_parameter('publish_tf').value:
            return
        if m.header.frame_id != 'odom' or m.child_frame_id != 'base_link':
            return
        t=TransformStamped()
        t.header=m.header
        t.child_frame_id='base_link'
        t.transform.translation.x=m.pose.pose.position.x
        t.transform.translation.y=m.pose.pose.position.y
        t.transform.translation.z=m.pose.pose.position.z
        t.transform.rotation=m.pose.pose.orientation
        self.tf_pub.sendTransform(t)

    def on_debug(self,m):
        if not self.get_parameter('publish_joint_states').value or len(m.data)!=10:
            return
        radius=self.get_parameter('wheel_radius_m').value
        if radius<=0:
            return
        msg=JointState()
        msg.header.stamp=self.get_clock().now().to_msg()
        msg.name=[self.get_parameter('left_joint').value,
                  self.get_parameter('right_joint').value]
        msg.position=[float(m.data[6])/radius,float(m.data[7])/radius]
        msg.velocity=[float(m.data[2])*2*math.pi/60,
                      float(m.data[3])*2*math.pi/60]
        self.joint_pub.publish(msg)


def main():
    rclpy.init()
    node=OdomTfJointBridge()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
