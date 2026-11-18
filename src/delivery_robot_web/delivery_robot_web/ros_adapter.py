"""ROS 2 runs in a dedicated executor thread; HTTP handlers never block its loop."""
from __future__ import annotations
import math
import queue
import threading
import time
from collections import deque
from .core import UserError
from .maps import yaw_of


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


class RosAdapter:
    def __init__(self, config, motion):
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
        from nav_msgs.msg import Odometry, OccupancyGrid
        from sensor_msgs.msg import LaserScan
        from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
        from std_msgs.msg import Float32MultiArray
        from tf2_ros import Buffer, TransformListener
        from rclpy.duration import Duration
        self.rclpy, self.config, self.motion, self.Twist = rclpy, config, motion, Twist
        self.PoseWithCovarianceStamped = PoseWithCovarianceStamped
        self.lock = threading.RLock()
        self.jobs = queue.Queue()
        self.odom = self.scan = self.map_msg = None
        self.odom_at = self.scan_at = self.map_at = 0.0
        self.map_rev = 0
        # Latest /map seen on ROS regardless of mapping/localization mode. This
        # powers the RViz-like technical Web view. The separate map_msg cache
        # below is still gated to Web-owned SLAM sessions so old maps never
        # reappear in the Mapping tab after Stop/Save.
        self.ros_map_msg = None
        self.ros_map_at = 0.0
        self.ros_map_rev = 0
        self.map_capture_enabled = False
        self.safety = self.wheels = self.heading = None
        self.safety_at = self.wheels_at = self.heading_at = 0.0
        self.maintenance = False
        self.scan_history = deque(maxlen=10)
        self.arrivals = {'odom': deque(maxlen=50), 'scan': deque(maxlen=50)}
        self.cached = {'nodes': [], 'odom_publishers': 0, 'scan_publishers': 0,
                       'global_costmap_publishers': 0, 'local_costmap_publishers': 0,
                       'cmd_subscribers': 0, 'other_cmd_publishers': [], 'tf_odom': None,
                       'tf_map': None, 'scan_tf_ready': False, 'scan_points': [],
                       'slam_state': 'unknown', 'tf_odom_laser': False}
        self.last_graph = 0.0
        self.last_service = 0.0
        self.last_stop = motion.stop_generation
        self.zeros_remaining = 0
        self.was_armed = False
        self.closed = False
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        self.node = Node('delivery_robot_web')
        self.buffer = Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = TransformListener(self.buffer, self.node, spin_thread=False)
        sensor_qos = qos_profile_sensor_data
        self.node.create_subscription(Odometry, config['odom_topic'], self.on_odom, sensor_qos)
        self.node.create_subscription(LaserScan, config['scan_topic'], self.on_scan, sensor_qos)
        self.node.create_subscription(Float32MultiArray, '/robot/safety_status', self.on_safety, sensor_qos)
        self.node.create_subscription(Float32MultiArray, '/robot/wheel_debug', self.on_wheels, sensor_qos)
        self.node.create_subscription(Float32MultiArray, '/robot/heading_status', self.on_heading, sensor_qos)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
        cmd_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.VOLATILE)
        self.cmd_qos = cmd_qos
        self.cmd_pub = None  # No command publisher while the web only observes.
        self.gate_pub = self.node.create_publisher(LaserScan, '/web/scan_slam', sensor_qos)
        self.initial_pose_pub = self.node.create_publisher(PoseWithCovarianceStamped, '/initialpose', cmd_qos)
        self.pending_scans = deque()
        self.gate_forwarded = self.gate_dropped = 0
        from lifecycle_msgs.srv import GetState
        self.lifecycle_client = self.node.create_client(GetState, '/slam_toolbox/get_state')
        self.lifecycle_future = None
        self.node.create_timer(0.05, self.tick)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, name='ros-executor', daemon=True)
        self.thread.start()

    def on_safety(self, msg):
        with self.lock:
            self.safety, self.safety_at = list(msg.data), time.monotonic()

    def on_wheels(self, msg):
        with self.lock:
            self.wheels, self.wheels_at = list(msg.data), time.monotonic()

    def on_heading(self, msg):
        with self.lock:
            self.heading, self.heading_at = list(msg.data), time.monotonic()

    def on_odom(self, msg):
        with self.lock:
            self.odom, self.odom_at = msg, time.monotonic()
            self.arrivals['odom'].append(self.odom_at)

    def on_scan(self, msg):
        with self.lock:
            self.scan, self.scan_at = msg, time.monotonic()
            self.arrivals['scan'].append(self.scan_at)
            self.scan_history.append((self.scan_at, msg))
            self.pending_scans.append((self.scan_at, msg))
            if len(self.pending_scans) > 24:
                self.pending_scans.popleft()
                self.gate_dropped += 1

    def on_map(self, msg):
        with self.lock:
            # Always retain the latest ROS /map for the technical localization
            # display. This is observation-only and never changes saved files.
            self.ros_map_msg, self.ros_map_at = msg, time.monotonic()
            self.ros_map_rev = getattr(self, 'ros_map_rev', 0) + 1
            # Mapping's LIVE cache remains session-gated. This prevents a
            # transient/static old map from reappearing after Stop/Save.
            if not self.map_capture_enabled:
                return
            self.map_msg, self.map_at = msg, self.ros_map_at
            self.map_rev += 1

    def lookup(self, target, source, at=None):
        from rclpy.time import Time
        try:
            return self.buffer.lookup_transform(target, source, at if at is not None else Time())
        except Exception:
            return None

    def pose(self, transform):
        if transform is None:
            return None
        t = transform.transform
        if not all(math.isfinite(v) for v in (t.translation.x, t.translation.y, t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)):
            return None
        return {'x': t.translation.x, 'y': t.translation.y,
                'yaw': yaw_of(t.rotation), 'stamp': stamp_seconds(transform.header.stamp)}

    def tick(self):
        # Movement always checked first. No long-running filesystem/network work here.
        status = self.status()
        ready = self.motion_ready(status)
        v, w, armed = self.motion.output(ready)
        if self.motion.stop_generation != self.last_stop or (self.was_armed and not armed):
            self.zeros_remaining = 3
            self.last_stop = self.motion.stop_generation
        if armed or self.zeros_remaining:
            msg = self.Twist()
            msg.linear.x, msg.angular.z = float(v), float(w)
            if self.cmd_pub is None:
                self.cmd_pub = self.node.create_publisher(self.Twist, self.config['cmd_topic'], self.cmd_qos)
            self.cmd_pub.publish(msg)
            if not armed:
                self.zeros_remaining = max(0, self.zeros_remaining - 1)
        if not armed and self.zeros_remaining == 0 and self.cmd_pub is not None:
            self.node.destroy_publisher(self.cmd_pub)
            self.cmd_pub = None
        self.was_armed = armed
        # ROS services are created/called on this executor, results delivered to HTTP threads.
        for _ in range(3):
            try:
                task = self.jobs.get_nowait()
            except queue.Empty:
                break
            try:
                task()
            except Exception as exc:
                self.node.get_logger().error(str(exc))
        now = time.monotonic()
        # Bounded TF-aware relay: preserves bytes, values and header.stamp.
        # A delayed packet is not a clock correction and is never used for collision avoidance.
        from rclpy.time import Time
        for _ in range(3):
            with self.lock:
                if not self.pending_scans: break
                received, pending = self.pending_scans[0]
            if now-received > 1.0:
                with self.lock:
                    self.pending_scans.popleft(); self.gate_dropped += 1
                continue
            if self.lookup(self.config['odom_frame'], pending.header.frame_id, Time.from_msg(pending.header.stamp)) is None:
                break
            with self.lock:
                self.pending_scans.popleft(); self.gate_forwarded += 1
            self.gate_pub.publish(pending)
        if now - self.last_graph >= 0.25:
            self.last_graph = now
            c = self.config
            tf_odom = self.lookup(c['odom_frame'], c['base_frame'])
            tf_map = self.lookup(c['map_frame'], c['base_frame'])
            tf_laser = self.lookup(c['odom_frame'], c['laser_frame'])
            from rclpy.time import Time
            with self.lock:
                history = list(self.scan_history)
            ready_at_stamp = False
            points = []
            for received, scan in reversed(history):
                if now - received > 1.0:
                    continue
                at = Time.from_msg(scan.header.stamp)
                if self.lookup(c['odom_frame'], scan.header.frame_id, at):
                    ready_at_stamp = True
                tf_scan = self.lookup(c['map_frame'], scan.header.frame_id, at)
                if tf_scan and not points:
                    t = tf_scan.transform
                    a = yaw_of(t.rotation)
                    ca, sa = math.cos(a), math.sin(a)
                    step = max(1, len(scan.ranges) // 600)
                    for i in range(0, len(scan.ranges), step):
                        d = scan.ranges[i]
                        if not math.isfinite(d) or not scan.range_min <= d <= scan.range_max:
                            continue
                        angle = scan.angle_min + i * scan.angle_increment
                        px, py = d * math.cos(angle), d * math.sin(angle)
                        points.append([t.translation.x + ca*px-sa*py, t.translation.y+sa*px+ca*py])
                if ready_at_stamp and points:
                    break
            with self.lock:
                self.cached.update(tf_odom=self.pose(tf_odom), tf_map=self.pose(tf_map),
                    tf_odom_laser=tf_laser is not None, scan_tf_ready=ready_at_stamp, scan_points=points)
            # Graph queries at 4 Hz: sufficient for status, not a hard realtime safety guarantee.
            try:
                names = self.node.get_node_names_and_namespaces()
                nodes = [(ns.rstrip('/') + '/' + n) for n, ns in names]
                pubs = self.node.get_publishers_info_by_topic(c['cmd_topic'])
                others = [p.node_name for p in pubs if p.node_name != self.node.get_name()]
                if sum(p.node_name == self.node.get_name() for p in pubs) > 1:
                    others.append('delivery_robot_web (trùng publisher)')
                subs = self.node.get_subscriptions_info_by_topic(c['cmd_topic'])
                good_subs = sum(p.topic_type == 'geometry_msgs/msg/Twist' for p in subs)
                motor_pubs = self.node.get_publishers_info_by_topic('/cmd_vel')
                motor_subs = self.node.get_subscriptions_info_by_topic('/cmd_vel')
                motor_owners = sorted(
                    p.node_namespace.rstrip('/') + '/' + p.node_name
                    for p in motor_pubs
                )
                motor_receivers = sorted(
                    p.node_namespace.rstrip('/') + '/' + p.node_name
                    for p in motor_subs
                    if p.topic_type == 'geometry_msgs/msg/Twist'
                )
                with self.lock:
                    self.cached.update(nodes=nodes,
                        odom_publishers=self.node.count_publishers(c['odom_topic']),
                        scan_publishers=self.node.count_publishers(c['scan_topic']),
                        global_costmap_publishers=self.node.count_publishers('/global_costmap/costmap'),
                        local_costmap_publishers=self.node.count_publishers('/local_costmap/costmap'),
                        cmd_subscribers=good_subs, other_cmd_publishers=others,
                        motor_cmd_publishers=motor_owners,
                        motor_cmd_receivers=motor_receivers)
            except Exception:
                pass
        if now - self.last_service > 2.0:
            self.last_service = now
            if self.lifecycle_client.service_is_ready():
                if not self.lifecycle_future or self.lifecycle_future.done():
                    from lifecycle_msgs.srv import GetState
                    self.lifecycle_future = self.lifecycle_client.call_async(GetState.Request())
                    def finished(f):
                        try:
                            label = f.result().current_state.label
                            with self.lock:
                                self.cached['slam_state'] = label
                        except Exception:
                            pass
                    self.lifecycle_future.add_done_callback(finished)
            else:
                with self.lock:
                    self.cached['slam_state'] = 'offline'

    @staticmethod
    def rate(times):
        return (len(times)-1)/(times[-1]-times[0]) if len(times)>1 and times[-1]>times[0] else 0.0

    def status(self):
        with self.lock:
            now = time.monotonic()
            wall = self.node.get_clock().now().nanoseconds / 1e9
            result = dict(self.cached)
            for key in ('odom', 'scan'):
                msg = getattr(self, key)
                arrived = getattr(self, key + '_at')
                result[key] = {'receive_age': now-arrived if msg else None,
                    'stamp_age': wall-stamp_seconds(msg.header.stamp) if msg else None,
                    'hz': self.rate(self.arrivals[key]) if msg and now-arrived < 2 else 0,
                    'frame': msg.header.frame_id if msg else None}
            if self.odom:
                p = self.odom.pose.pose
                valid = all(math.isfinite(v) for v in (p.position.x, p.position.y, p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w, self.odom.twist.twist.linear.x, self.odom.twist.twist.angular.z))
                result['odom']['valid'] = valid
                if valid:
                    result['odom_pose'] = {'x': p.position.x, 'y': p.position.y, 'yaw': yaw_of(p.orientation)}
            result['motor'] = self.motor_status(now)
            result['scan_gate'] = {'forwarded':self.gate_forwarded,'dropped':self.gate_dropped,'pending':len(self.pending_scans)}
            result['maintenance'] = self.maintenance
            result['map_revision'] = self.map_rev
            result['map_receive_age'] = now-self.map_at if self.map_msg else None
            if result.get('tf_odom'):
                result['tf_odom'] = dict(result['tf_odom'], age=wall-result['tf_odom']['stamp'])
            if result.get('tf_map'):
                result['tf_map'] = dict(result['tf_map'], age=wall-result['tf_map']['stamp'])
            result['mode'] = 'ROS2'
            result['ros_ok'] = self.thread.is_alive() if hasattr(self, 'thread') else True
            result['motion_issues'] = self.motion_issues(result)
            result['motion_ready'] = not result['motion_issues']
            result['mapping_issues'] = self.mapping_issues(result)
            return result

    def motor_status(self, now):
        s, w, h = self.safety, self.wheels, self.heading
        def finite_list(x, n):
            return x is not None and len(x) >= n and all(math.isfinite(v) for v in x[:n])
        result = {'safety_age': now-self.safety_at if s else None,
                  'wheel_age': now-self.wheels_at if w else None}
        if finite_list(s, 12):
            result.update(state=int(s[5]), interlock=int(s[8]), bench=bool(s[9]), cmd_fresh=bool(s[11]))
            result['state_name'] = {0:'DISARMED',1:'IDLE',2:'LAUNCH',3:'ACTIVE',4:'FAULT',5:'DECEL'}.get(int(s[5]), 'UNKNOWN')
        if finite_list(w,10):
            result.update(target_l=w[0], target_r=w[1], rpm_l=w[2], rpm_r=w[3], pwm_l=w[4], pwm_r=w[5])
        if finite_list(h,12):
            result.update(yaw_deg=h[0], imu_fresh=bool(h[8]), heading_enabled=bool(h[10]), heading_age=now-self.heading_at)
        return result

    def motion_issues(self, s):
        issues = []
        d = s.get('odom', {})
        age, recv = d.get('stamp_age'), d.get('receive_age')
        if not s.get('ros_ok', True): issues.append('ROS executor chưa hoạt động.')
        if s.get('maintenance', False): issues.append('Đang cấu hình/build/flash USB; điều khiển đã khóa.')
        if s.get('odom_publishers') != 1: issues.append('/odom cần đúng một publisher.')
        if not d.get('valid', False): issues.append('Odometry không hợp lệ.')
        if age is None or not -0.05 <= age <= self.config['max_odom_age']: issues.append('Timestamp odom quá cũ/lệch đồng hồ; kiểm tra đồng bộ micro-ROS.')
        if recv is None or recv > self.config['max_receive_age']: issues.append('Mất dữ liệu odom mới.')
        if s.get('cmd_subscribers',0) < 1:
            issues.append('Command Mux chưa subscribe /cmd_vel_manual.')
        if s.get('other_cmd_publishers'):
            issues.append('Có nguồn /cmd_vel_manual khác: '+', '.join(s['other_cmd_publishers']))
        if '/delivery_robot_motor' not in s.get('motor_cmd_receivers', []):
            issues.append('ESP32 chưa subscribe /cmd_vel.')
        if s.get('motor_cmd_publishers') != ['/delivery_robot_cmd_mux']:
            issues.append('Nguồn /cmd_vel không phải duy nhất Command Mux.')
        if self.config.get('require_motor_telemetry', True):
            m = s.get('motor', {})
            if m.get('safety_age') is None or m['safety_age'] > 0.8 or 'state' not in m: issues.append('Thiếu safety_status mới, đúng bố cục 12 trường.')
            if m.get('wheel_age') is None or m['wheel_age'] > 0.8 or 'rpm_l' not in m: issues.append('Thiếu wheel_debug mới, đúng bố cục 10 trường.')
            if m.get('state') not in (1,2,3,5): issues.append('Firmware chưa IDLE/đang chạy hợp lệ hoặc đang FAULT.')
            if m.get('interlock') in (3,7,8,9,11,12,13): issues.append('Firmware đang khóa chuyển động; không bỏ qua interlock.')
            if m.get('bench'): issues.append('Đang BENCH; tắt bench bằng công cụ hiện có trước khi lái trên web.')
        return issues

    def motion_ready(self, s=None):
        return not self.motion_issues(s or self.status())

    def mapping_issues(self, s):
        issues = []
        for key in ('odom', 'scan'):
            d = s.get(key, {})
            if s.get(key+'_publishers') != 1:
                issues.append(f'{key}: cần đúng một publisher.')
            if d.get('receive_age') is None or d['receive_age'] > self.config['max_receive_age']:
                issues.append(f'{key}: chưa nhận dữ liệu mới.')
            if d.get('stamp_age') is None or not -0.05 <= d['stamp_age'] <= self.config['max_odom_age']:
                issues.append(f'{key}: timestamp cũ/lệch đồng hồ; không tự bù thời gian.')
        if not s.get('odom', {}).get('valid', False):
            issues.append('Dữ liệu odometry chưa hợp lệ.')
        if not s.get('tf_odom_laser'):
            issues.append('Thiếu TF odom → base_link → base_laser.')
        if not s.get('scan_tf_ready'):
            issues.append('Chưa có TF tại timestamp của các scan gần đây.')
        return issues

    def begin_map_session(self):
        """Start a fresh live-map session. Never reuse the previous web cache."""
        with self.lock:
            self.map_capture_enabled = True
            self.map_msg = None
            self.map_at = 0.0
            self.map_rev += 1
            self.cached['tf_map'] = None
            self.cached['scan_points'] = []

    def end_map_session(self):
        """Forget the live map when SLAM is saved/stopped. Saved map files stay intact."""
        with self.lock:
            self.map_capture_enabled = False
            self.map_msg = None
            self.map_at = 0.0
            self.map_rev += 1
            self.cached['tf_map'] = None
            self.cached['scan_points'] = []

    def current_map(self):
        with self.lock:
            return self.map_rev, self.map_msg

    def current_ros_map(self):
        with self.lock:
            msg = getattr(self, 'ros_map_msg', None)
            at = getattr(self, 'ros_map_at', 0.0)
            age = None if msg is None else max(0.0, time.monotonic() - at)
            return getattr(self, 'ros_map_rev', 0), msg, age

    def clear_ros_map_observation(self):
        with self.lock:
            self.ros_map_msg = None
            self.ros_map_at = 0.0
            self.ros_map_rev = getattr(self, 'ros_map_rev', 0) + 1

    def publish_initial_pose(self, x: float, y: float, yaw: float):
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise UserError('Initial Pose phải là số hữu hạn.')
        event = threading.Event()
        error = []
        def send():
            try:
                msg = self.PoseWithCovarianceStamped()
                msg.header.frame_id = self.config['map_frame']
                msg.header.stamp = self.node.get_clock().now().to_msg()
                msg.pose.pose.position.x = float(x)
                msg.pose.pose.position.y = float(y)
                msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
                msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
                msg.pose.covariance[0] = 0.25
                msg.pose.covariance[7] = 0.25
                msg.pose.covariance[35] = math.radians(15.0) ** 2
                self.initial_pose_pub.publish(msg)
            except Exception as exc:
                error.append(str(exc))
            finally:
                event.set()
        self.jobs.put(send)
        if not event.wait(1.0):
            raise UserError('ROS executor chưa gửi được Initial Pose.')
        if error:
            raise UserError('Không gửi được Initial Pose: ' + error[0])
        return {'x': float(x), 'y': float(y), 'yaw': float(yaw)}

    def request(self, service_type, name, request, timeout=15):
        # Client discovery is asynchronous. Wait via a short ROS timer, not by
        # blocking this single-thread executor (which also drives the deadman).
        event = threading.Event()
        result = {}
        deadline = time.monotonic() + timeout

        def start():
            client = self.node.create_client(service_type, name)
            state = {'done': False, 'future': None, 'timer': None}

            def complete(value=None, error=None):
                if state['done']:
                    return
                state['done'] = True
                if state['timer'] is not None:
                    state['timer'].cancel()
                    self.node.destroy_timer(state['timer'])
                if error:
                    result['error'] = error
                    if state['future'] is not None and not state['future'].done():
                        state['future'].cancel()  # Local wait only; no remote rollback.
                else:
                    result['value'] = value
                self.node.destroy_client(client)
                event.set()

            def finish(future):
                if state['done']:
                    return
                try:
                    complete(value=future.result())
                except Exception as exc:
                    complete(error=str(exc))

            def poll():
                if state['done']:
                    return
                if time.monotonic() >= deadline:
                    complete(error=f'Hết thời gian chờ {name}; chưa xác nhận kết quả. Không tự thử lại.')
                    return
                if state['future'] is None and client.service_is_ready():
                    try:
                        state['future'] = client.call_async(request)
                        state['future'].add_done_callback(finish)
                    except Exception as exc:
                        complete(error=str(exc))

            state['timer'] = self.node.create_timer(0.05, poll)
            poll()

        self.jobs.put(start)
        if not event.wait(timeout + 0.2):
            # A timed out operation may still finish remotely: never report
            # success or retry automatically. The ROS timer reaps the client.
            raise UserError(f'Hết thời gian chờ {name}. Chưa xác nhận kết quả; xem log trước khi thử lại.')
        if 'error' in result:
            raise UserError(result['error'])
        return result['value']

    def save_map(self, path):
        from slam_toolbox.srv import SaveMap
        req = SaveMap.Request()
        req.name.data = str(path)
        response = self.request(SaveMap, '/slam_toolbox/save_map', req, 25)
        if response.result != 0:
            raise UserError(f'SLAM lưu ảnh map thất bại, result={response.result}.')

    def serialize(self, path):
        from slam_toolbox.srv import SerializePoseGraph
        req = SerializePoseGraph.Request()
        req.filename = str(path)
        res = self.request(SerializePoseGraph, '/slam_toolbox/serialize_map', req, 30)
        if res.result != 0:
            raise UserError(f'Lưu pose graph thất bại, result={res.result}.')

    def activate(self):
        from lifecycle_msgs.srv import ChangeState
        req = ChangeState.Request()
        req.transition.id = 3  # TRANSITION_ACTIVATE; only call for inactive node
        response = self.request(ChangeState, '/slam_toolbox/change_state', req)
        if not response.success:
            raise UserError('Không kích hoạt được SLAM. Xem log SLAM.')

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.motion.stop('Ứng dụng đang đóng.')
        time.sleep(0.2)
        self.executor.shutdown(timeout_sec=2)
        self.thread.join(timeout=3)
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


class OfflineAdapter:
    """Explicit UI-only mode. Never masquerades as a robot, never publishes commands."""
    def __init__(self, config, motion):
        self.config, self.motion = config, motion

    def status(self):
        empty = {'hz': 0, 'stamp_age': None, 'receive_age': None, 'frame': None}
        return {'mode': 'OFFLINE', 'ros_ok': False, 'odom': dict(empty), 'scan': dict(empty),
            'odom_publishers': 0, 'scan_publishers': 0, 'cmd_subscribers': 0,
            'other_cmd_publishers': [], 'nodes': [], 'map_revision': 0,
            'tf_odom': None, 'tf_map': None, 'scan_points': [], 'slam_state': 'offline',
            'mapping_issues': ['Chế độ kiểm tra giao diện, không kết nối ROS 2.'],
            'motion_ready': False, 'motion_issues': ['OFFLINE: không phát lệnh robot.'], 'motor': {}, 'maintenance': False}

    def begin_map_session(self): pass
    def end_map_session(self): pass
    def current_map(self): return 0, None
    def current_ros_map(self): return 0, None, None
    def clear_ros_map_observation(self): pass
    def motion_ready(self, *args): return False
    def close(self): self.motion.stop()
    def activate(self): raise UserError('Đang ở chế độ OFFLINE.')
    def publish_initial_pose(self, *args): raise UserError('Đang ở chế độ OFFLINE.')
    def save_map(self, path): raise UserError('Đang ở chế độ OFFLINE.')
    def serialize(self, path): raise UserError('Đang ở chế độ OFFLINE.')
