from __future__ import annotations
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from .logic import Velocity, clamp_velocity, select_velocity


class CommandMux(Node):
    def __init__(self):
        super().__init__('delivery_robot_cmd_mux')
        self.declare_parameter('manual_topic', '/cmd_vel_manual')
        self.declare_parameter('nav_topic', '/cmd_vel_nav')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('manual_timeout', 0.35)
        self.declare_parameter('nav_timeout', 0.35)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('max_linear', 0.35)
        self.declare_parameter('max_angular', 1.20)
        self.declare_parameter('initial_mode', 'STOP')

        self.manual_timeout = float(self.get_parameter('manual_timeout').value)
        self.nav_timeout = float(self.get_parameter('nav_timeout').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.mode = str(self.get_parameter('initial_mode').value).upper()
        if self.mode not in ('STOP', 'MANUAL', 'NAVIGATION'):
            self.mode = 'STOP'

        self.lock = threading.RLock()
        self.manual = Velocity(); self.nav = Velocity()
        self.manual_at = 0.0; self.nav_at = 0.0
        manual_topic = str(self.get_parameter('manual_topic').value)
        nav_topic = str(self.get_parameter('nav_topic').value)
        out_topic = str(self.get_parameter('output_topic').value)
        self.out = self.create_publisher(Twist, out_topic, 10)
        self.state_pub = self.create_publisher(String, '/delivery_robot/cmd_mux/state', 10)
        self.create_subscription(Twist, manual_topic, self._manual, 10)
        self.create_subscription(Twist, nav_topic, self._nav, 10)
        self.create_service(SetBool, '/delivery_robot/cmd_mux/set_navigation', self._set_navigation)
        self.create_service(Trigger, '/delivery_robot/cmd_mux/stop', self._stop)
        self.create_service(Trigger, '/delivery_robot/cmd_mux/set_manual', self._manual_mode)
        rate = max(5.0, float(self.get_parameter('publish_rate').value))
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f'Command mux started in {self.mode} mode; output={out_topic}')

    def _velocity(self, msg: Twist) -> Velocity:
        return clamp_velocity(Velocity(msg.linear.x, msg.angular.z), self.max_linear, self.max_angular)

    def _manual(self, msg: Twist):
        with self.lock:
            self.manual = self._velocity(msg); self.manual_at = time.monotonic()

    def _nav(self, msg: Twist):
        with self.lock:
            self.nav = self._velocity(msg); self.nav_at = time.monotonic()

    def _change(self, mode: str):
        with self.lock:
            self.mode = mode
            self.manual_at = 0.0; self.nav_at = 0.0
        self._publish_zero()
        self.get_logger().warn(f'Command authority -> {mode}')

    def _set_navigation(self, req, resp):
        self._change('NAVIGATION' if req.data else 'MANUAL')
        resp.success = True; resp.message = f'Mode is {self.mode}'
        return resp

    def _stop(self, _req, resp):
        self._change('STOP'); resp.success = True; resp.message = 'STOP active'
        return resp

    def _manual_mode(self, _req, resp):
        self._change('MANUAL'); resp.success = True; resp.message = 'MANUAL active'
        return resp

    def _publish_zero(self):
        self.out.publish(Twist())

    def _tick(self):
        now = time.monotonic()
        with self.lock:
            selected = select_velocity(
                self.mode, self.manual, self.nav,
                now - self.manual_at <= self.manual_timeout,
                now - self.nav_at <= self.nav_timeout,
            )
            mode = self.mode
        msg = Twist(); msg.linear.x = selected.linear_x; msg.angular.z = selected.angular_z
        self.out.publish(msg)
        state = String(); state.data = mode; self.state_pub.publish(state)

    def destroy_node(self):
        for _ in range(5):
            self._publish_zero()
        return super().destroy_node()


def main():
    rclpy.init()
    node = CommandMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
