#!/usr/bin/env python3
"""Safe ROS 2 Jazzy position-test controller for the ESP32 delivery robot V2.

Subscribes to /odom and /goal_pose, publishes /cmd_vel and /robot/goal_status.
Accepts goals in odom (absolute) OR base_link (relative to the CURRENT pose).
Forward-only by default; requires supervised, obstacle-free short tests.
This does NOT implement Nav2, avoidance, global localization, or emergency stop.
"""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, String


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    components = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in components):
        raise ValueError('Non-finite quaternion')
    norm = math.sqrt(sum(v * v for v in components))
    if norm < 1e-6:
        raise ValueError('Zero quaternion')
    x, y, z, w = (v / norm for v in components)
    return math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))


def goal_errors(pose, goal):
    x, y, yaw = pose
    gx, gy, gyaw = goal
    dx, dy = gx - x, gy - y
    return math.hypot(dx, dy), wrap(math.atan2(dy, dx) - yaw), wrap(gyaw - yaw)


def convert_goal_to_odom(pose, goal_msg):
    """Transform a stamped goal at reception time, without a TF listener.

    Only odom and base_link are accepted. base_link is a *snapshot* relative
    goal, not a continuously moving target frame.
    """
    p, q = goal_msg.pose.position, goal_msg.pose.orientation
    if not (math.isfinite(p.x) and math.isfinite(p.y) and math.isfinite(p.z)
            and abs(p.z) < 0.01):
        raise ValueError('Position must be finite and planar')
    gyaw = yaw_from_quaternion(q)
    frame = goal_msg.header.frame_id
    if frame == 'odom':
        return (p.x, p.y, gyaw)
    if frame == 'base_link':
        x, y, yaw = pose
        return (x + math.cos(yaw)*p.x - math.sin(yaw)*p.y,
                y + math.sin(yaw)*p.x + math.cos(yaw)*p.y,
                wrap(yaw + gyaw))
    raise ValueError('frame_id must be odom or base_link')


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
        self.start_until = 0.0
        self.stop_until = 0.0
        self.phase = 'WAITING_FOR_ODOM'
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/robot/goal_status', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.create_subscription(Empty, '/robot/cancel_goal', self.on_cancel, 10)
        self.create_timer(0.05, self.tick)
        self.get_logger().warning(
            'TEST ONLY: no obstacle avoidance. enable_motion defaults false. '
            'Use ONE /cmd_vel publisher. Goals: odom=absolute, base_link=relative. '
            'Keep allow_spin=false until reverse/turn-in-place is verified.')

    def param(self, name):
        return self.get_parameter(name).value

    def on_odom(self, msg):
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
            self.get_logger().warning('Ignoring /odom: expected odom -> base_link')
            return
        try:
            yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        except ValueError:
            return
        p = msg.pose.pose.position
        if not (math.isfinite(p.x) and math.isfinite(p.y)):
            return
        self.pose = (p.x, p.y, yaw)
        self.odom_at = time.monotonic()
        if self.phase == 'WAITING_FOR_ODOM':
            self.phase = 'IDLE'

    def publish_cmd(self, v=0.0, w=0.0):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def stop(self, status, reason=''):
        self.goal = None
        self.phase = status
        # Keep sending zero long enough to execute ESP32's DECEL state.
        # Firmware's /cmd_vel watchdog is 800 ms; a single zero is unreliable.
        self.stop_until = time.monotonic() + 1.6
        self.publish_cmd(0.0, 0.0)
        if reason:
            self.get_logger().warning(f'{status}: {reason}')
        else:
            self.get_logger().info(status)

    def on_goal(self, msg):
        now = time.monotonic()
        if now < self.stop_until:
            self.get_logger().warning('Goal rejected: previous stop is still in progress')
            return
        if self.goal is not None:
            self.stop('REJECTED_BUSY', 'Cancel or finish current goal first')
            return
        if not self.param('enable_motion'):
            self.stop('REJECTED_DISABLED', 'Set -p enable_motion:=true')
            return
        if self.pose is None or now - self.odom_at > self.param('odom_timeout_s'):
            self.stop('REJECTED_ODOM_STALE', 'Check /odom frame and rate')
            return
        try:
            goal = convert_goal_to_odom(self.pose, msg)
        except ValueError as exc:
            self.stop('REJECTED_GOAL_FRAME_OR_POSE', str(exc))
            return
        distance, heading, yaw_error = goal_errors(self.pose, goal)
        if distance > self.param('max_goal_distance_m'):
            self.stop('REJECTED_TOO_FAR', f'distance={distance:.2f} m')
            return
        if not self.param('allow_spin') and distance > self.param('goal_position_tolerance_m') and abs(heading) > 0.25:
            self.stop('REJECTED_NEEDS_ROTATION',
                      f'heading_error={math.degrees(heading):.1f} deg; try a goal ahead in base_link')
            return
        if (not self.param('allow_spin') and not self.param('ignore_final_yaw_without_spin')
                and abs(yaw_error) > self.param('goal_yaw_tolerance_rad')):
            self.stop('REJECTED_FINAL_YAW', 'allow_spin is false')
            return
        self.goal = goal
        self.goal_at = now
        # Important: existing robot_drive.cpp LAUNCH only engages for equal
        # forward wheel setpoints >=20 RPM. First 0.5 s straight, then arc.
        # Avoids starting the motor through low-PWM curved ACTIVE mode.
        self.start_until = now + 0.50
        self.phase = 'ALIGN' if self.param('allow_spin') else 'DRIVE_START'
        self.get_logger().info(
            f'ACCEPTED goal odom=({goal[0]:.3f}, {goal[1]:.3f}, {goal[2]:.3f}) '
            f'distance={distance:.3f}m heading={math.degrees(heading):.1f}deg')

    def on_cancel(self, _):
        self.stop('CANCELLED', 'Goal cancelled')

    def tick(self):
        now = time.monotonic()
        status = String()
        status.data = self.phase
        self.status_pub.publish(status)
        if now < self.stop_until:
            self.publish_cmd(0.0, 0.0)
            return
        if self.goal is None:
            return
        if not self.param('enable_motion') or self.pose is None or now - self.odom_at > self.param('odom_timeout_s'):
            self.stop('ABORTED_ODOM_OR_DISABLED', 'Odom stale or enable_motion disabled')
            return
        if now - self.goal_at > self.param('goal_timeout_s'):
            self.stop('ABORTED_TIMEOUT', 'Goal timeout')
            return
        distance, ehead, eyaw = goal_errors(self.pose, self.goal)
        pos_tol = self.param('goal_position_tolerance_m')
        yaw_tol = self.param('goal_yaw_tolerance_rad')
        if distance <= pos_tol:
            if not self.param('allow_spin') or abs(eyaw) <= yaw_tol:
                self.stop('REACHED', f'Remaining position error {distance:.3f}m')
                return
            self.phase = 'FINAL_TURN'
        elif self.param('allow_spin') and abs(ehead) > 0.30:
            self.phase = 'ALIGN'
        else:
            if not self.param('allow_spin') and now < self.start_until:
                # Equal wheel RPM makes LAUNCH startup available in V2 firmware.
                self.phase = 'DRIVE_START'
            else:
                self.phase = 'DRIVE'
        if self.phase == 'DRIVE_START':
            self.publish_cmd(min(0.10, self.param('max_v_mps')), 0.0)
        elif self.phase == 'DRIVE':
            # Forward-only. Keep both wheel targets non-negative.
            v = clamp(0.9 * distance, 0.10, self.param('max_v_mps'))
            w = clamp(1.7 * ehead, -0.7, 0.7)
            w = clamp(w, -1.8 * v / self.param('wheel_track_m'),
                            1.8 * v / self.param('wheel_track_m'))
            self.publish_cmd(v, w)
        else:
            # ONLY if clockwise and counterclockwise wheel directions have
            # been physically validated. This can command negative wheel RPM.
            angle = eyaw if self.phase == 'FINAL_TURN' else ehead
            w = math.copysign(clamp(2.0 * abs(angle), 0.65, 1.15), angle)
            self.publish_cmd(0.0, w)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = GoToPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Keep sending zeros while ROS is still active, to trigger firmware DECEL.
        # This is a normal stop, not an emergency stop or guaranteed braking.
        end = time.monotonic() + 1.6
        while time.monotonic() < end:
            node.publish_cmd(0.0, 0.0)
            time.sleep(0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
