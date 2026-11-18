from __future__ import annotations
import json
import math
import threading

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener

from delivery_robot_interfaces.msg import MissionState


class WebBridge(Node):
    def __init__(self):
        super().__init__('delivery_robot_web_bridge')
        self.lock = threading.RLock(); self.plan = None; self.mission = None; self.mux_mode = 'UNKNOWN'; self.goal_handle = None
        self.tf_buffer = Buffer(); self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.mux = self.create_client(SetBool, '/delivery_robot/cmd_mux/set_navigation')
        self.pub = self.create_publisher(String, '/delivery_robot/web/nav2_status', 10)
        self.create_subscription(Path, '/plan', self._plan, 10)
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(MissionState, '/delivery_robot/mission/state', self._mission, qos)
        self.create_subscription(String, '/delivery_robot/cmd_mux/state', self._mux_state, 10)
        self.create_subscription(PoseStamped, '/delivery_robot/web/goal', self._goal, 10)
        self.create_service(Trigger, '/delivery_robot/web/cancel_navigation', self._cancel)
        self.create_timer(0.5, self._publish)

    def _plan(self, msg):
        with self.lock: self.plan = msg
    def _mission(self, msg):
        with self.lock: self.mission = msg
    def _mux_state(self, msg):
        with self.lock: self.mux_mode = msg.data

    def _goal(self, pose):
        if pose.header.frame_id in ('', 'map'): pose.header.frame_id = 'map'
        else:
            self.get_logger().error('Web goal must be in map frame'); return
        if not self.nav.wait_for_server(timeout_sec=0.2):
            self.get_logger().error('/navigate_to_pose unavailable'); return
        def send():
            goal = NavigateToPose.Goal()
            goal.pose = pose
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            future = self.nav.send_goal_async(goal)
            future.add_done_callback(self._goal_response)

        if self.mux.wait_for_service(timeout_sec=0.1):
            req = SetBool.Request()
            req.data = True
            future = self.mux.call_async(req)

            def mode_done(done_future):
                try:
                    response = done_future.result()
                except Exception as exc:
                    self.get_logger().error(f'Command mux service failed: {exc}')
                    return
                if response is None or not response.success:
                    self.get_logger().error('Command mux refused NAVIGATION mode')
                    return
                send()

            future.add_done_callback(mode_done)
        else:
            self.get_logger().error(
                'Command mux unavailable; Web navigation goal refused'
            )
            return

    def _goal_response(self, future):
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'Web navigation goal send failed: {exc}')
            return
        if handle is None or not handle.accepted:
            self.get_logger().error('Web navigation goal rejected')
            return
        with self.lock:
            self.goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._goal_result)

    def _goal_result(self, _future):
        with self.lock:
            self.goal_handle = None

    def _cancel(self, _req, resp):
        with self.lock: handle = self.goal_handle
        if handle is None:
            resp.success = False; resp.message = 'No active Web navigation goal'; return resp
        handle.cancel_goal_async(); resp.success = True; resp.message = 'Cancel requested'; return resp

    def _publish(self):
        pose = None
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            q = tf.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            pose = {
                'x': tf.transform.translation.x,
                'y': tf.transform.translation.y,
                'yaw': yaw,
                'z': q.z,
                'w': q.w,
            }
        except Exception: pass
        with self.lock:
            plan = self.plan; mission = self.mission; mux = self.mux_mode
        data = {
            'nodes': sorted(name for name, _ns in self.get_node_names_and_namespaces()),
            'navigate_to_pose_available': self.nav.server_is_ready(),
            'robot_pose': pose,
            'plan_points': len(plan.poses) if plan is not None else 0,
            'mux_mode': mux,
            'mission': None if mission is None else {
                'id': mission.mission_id, 'state': mission.state_name,
                'message': mission.message, 'pickup': mission.pickup_name,
                'dropoff': mission.dropoff_name, 'distance_remaining': mission.distance_remaining,
                'nav_error_code': mission.nav_error_code,
            },
        }
        msg = String(); msg.data = json.dumps(data, separators=(',', ':')); self.pub.publish(msg)


def main():
    rclpy.init(); node = WebBridge()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
