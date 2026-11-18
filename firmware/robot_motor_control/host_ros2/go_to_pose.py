#!/usr/bin/env python3
"""ROS 2 Jazzy waypoint TEST controller: /goal_pose in odom frame -> /cmd_vel.

NO obstacle avoidance. Only use with supervised, short-distance tests on
an empty floor. This is NOT Nav2 and should not run concurrently with teleop.
"""
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, String
from pose_math import clamp, wrap, yaw_from_quat, goal_errors, forward_arc_command


class GoToPose(Node):
    def __init__(self):
        super().__init__('delivery_go_to_pose')
        self.declare_parameter('enable_motion', False)
        self.declare_parameter('allow_spin', False)
        self.declare_parameter('max_goal_distance_m', 1.5)
        self.declare_parameter('goal_timeout_s', 25.0)
        self.declare_parameter('odom_timeout_s', 0.7)
        self.declare_parameter('goal_position_tolerance_m', 0.055)
        self.declare_parameter('goal_yaw_tolerance_rad', 0.12)
        self.declare_parameter('wheel_track_m', 0.195)
        self.declare_parameter('max_v_mps', 0.12)
        self.declare_parameter('ignore_final_yaw_without_spin', True)
        self.pose = None
        self.odom_at = 0.0
        self.goal = None
        self.goal_at = 0.0
        self.phase = 'IDLE'
        self.rejected_reason = ''
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/robot/goal_status', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.create_subscription(Empty, '/robot/cancel_goal', self.on_cancel, 10)
        self.create_timer(0.05, self.tick)
        self.get_logger().warning('TEST CONTROLLER: NO obstacle avoidance. '
                                  'Disabled until enable_motion:=true. '
                                  'Do not use simultaneously with manual /cmd_vel or Nav2.')

    def param(self, name):
        return self.get_parameter(name).value

    def on_odom(self, msg):
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
            self.get_logger().warning('Ignoring odom with frames != odom/base_link', throttle_duration_sec=5.0)
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        if not (math.isfinite(p.x) and math.isfinite(p.y)):
            return
        try:
            yaw = yaw_from_quat(q.x,q.y,q.z,q.w)
        except ValueError:
            return
        self.pose = (p.x,p.y,yaw)
        self.odom_at = time.monotonic()

    def publish_cmd(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def on_goal(self, msg):
        self.goal = None  # Immediately revoke any previous goal
        self.phase = 'IDLE'
        if not self.param('enable_motion'):
            self.phase = 'REJECTED_DISABLED'
            self.publish_cmd(0.0,0.0)
            return
        if not self.pose or time.monotonic()-self.odom_at > self.param('odom_timeout_s'):
            self.phase = 'REJECTED_ODOM_STALE'
            self.publish_cmd(0.0,0.0)
            return
        if msg.header.frame_id != 'odom':
            self.phase = 'REJECTED_FRAME_USE_ODOM'
            self.publish_cmd(0.0,0.0)
            return
        p,q=msg.pose.position,msg.pose.orientation
        if not (math.isfinite(p.x) and math.isfinite(p.y) and abs(p.z)<0.01):
            self.phase = 'REJECTED_POSITION'
            self.publish_cmd(0.0,0.0)
            return
        try:
            gyaw=yaw_from_quat(q.x,q.y,q.z,q.w)
        except ValueError:
            self.phase = 'REJECTED_QUATERNION'
            self.publish_cmd(0.0,0.0)
            return
        dist, heading, yaw_err=goal_errors(*self.pose,p.x,p.y,gyaw)
        if dist > self.param('max_goal_distance_m'):
            self.phase='REJECTED_TOO_FAR'
            self.publish_cmd(0.0,0.0)
            return
        # Forward-only mode supports goals in front and gentle arcs; it cannot
        # turn in place or guarantee arbitrary final yaw.
        if not self.param('allow_spin') and abs(heading)>0.25 and dist>self.param('goal_position_tolerance_m'):
            self.phase='REJECTED_NEEDS_ROTATION'
            self.publish_cmd(0.0,0.0)
            return
        if not self.param('allow_spin') and not self.param('ignore_final_yaw_without_spin') and abs(yaw_err)>self.param('goal_yaw_tolerance_rad'):
            self.phase='REJECTED_FINAL_YAW'
            self.publish_cmd(0.0,0.0)
            return
        self.goal=(p.x,p.y,gyaw)
        self.goal_at=time.monotonic()
        self.phase='ALIGN' if self.param('allow_spin') else 'DRIVE'
        self.get_logger().info(f'GOAL accepted odom: x={p.x:.3f} y={p.y:.3f} yaw={gyaw:.3f}; phase={self.phase}')

    def on_cancel(self, _):
        self.goal=None
        self.phase='CANCELLED'
        self.publish_cmd(0.0,0.0)

    def tick(self):
        s=String()
        s.data=self.phase
        self.status_pub.publish(s)
        if self.goal is None:
            # Stop bursts only; sustained zero publisher could interfere with teleop.
            return
        if not self.param('enable_motion') or self.pose is None or time.monotonic()-self.odom_at>self.param('odom_timeout_s'):
            self.phase='ABORTED_ODOM_OR_DISABLED';self.goal=None
            self.publish_cmd(0.0,0.0)
            return
        if time.monotonic()-self.goal_at>self.param('goal_timeout_s'):
            self.phase='ABORTED_TIMEOUT';self.goal=None
            self.publish_cmd(0.0,0.0)
            return
        distance,ehead,eyaw=goal_errors(*self.pose,*self.goal)
        pos_tol=self.param('goal_position_tolerance_m')
        yaw_tol=self.param('goal_yaw_tolerance_rad')
        if distance<=pos_tol:
            if not self.param('allow_spin') or abs(eyaw)<=yaw_tol:
                self.phase='REACHED';self.goal=None
                self.publish_cmd(0.0,0.0)
                return
            self.phase='FINAL_TURN'
        elif self.param('allow_spin') and abs(ehead)>0.30:
            self.phase='ALIGN'
        else:
            self.phase='DRIVE'
        if self.phase=='DRIVE':
            v,w=forward_arc_command(distance,ehead,
                   track=self.param('wheel_track_m'),max_v=self.param('max_v_mps'))
            self.publish_cmd(v,w)
        else:
            # Turn-in-place REQUIRES both signed motor directions to be enabled,
            # tested and calibrated on the ESP32 before this flag is set true.
            angle = eyaw if self.phase=='FINAL_TURN' else ehead
            w=math.copysign(clamp(2.0*abs(angle),0.65,1.15),angle)
            self.publish_cmd(0.0,w)


def main():
    rclpy.init()
    node=GoToPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Automatic zero Twist, then watchdog on ESP32 for added safety.
        for _ in range(3):
            node.publish_cmd(0.0,0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
