"""Observed Nav2 state and bounded, acknowledged operator commands.

The ROS executor owns clients/actions. HTTP workers only enqueue work and wait
on threading events. An HTTP timeout never means that a robot action succeeded;
late accepted actions are canceled and the mux is returned to STOP.
"""
from __future__ import annotations

import copy
import math
import threading
import time
from collections.abc import Mapping

from .core import UserError


NAVIGATION_NODES = (
    'planner_server', 'controller_server', 'behavior_server', 'bt_navigator',
    'waypoint_follower', 'velocity_smoother', 'collision_monitor',
)
ACTIVE_MISSION_STATES = {'GOING_TO_PICKUP', 'WAITING_PICKUP',
                         'GOING_TO_DROPOFF', 'WAITING_DROPOFF'}
ACTIVE_GOAL_STATES = {'SENDING', 'ACTIVE', 'CANCELING', 'UNKNOWN'}


def finite(value, default=None):
    try:
        value = float(value)
        return value if math.isfinite(value) else default
    except (ValueError, TypeError):
        return default


def publisher_gid(info):
    """Normalize Jazzy's MessageInfo dict and attribute-based implementations.

    Metadata is optional observation data, never a reason to kill the executor.
    Reject integers (bytes(n) allocates n zeroes), malformed and empty GIDs.
    """
    if info is None:
        return None
    try:
        value = info.get('publisher_gid') if isinstance(info, Mapping) else getattr(info, 'publisher_gid', None)
        if value is None or isinstance(value, (str, int, bool)):
            return None
        if not 0 < len(value) <= 64:
            return None
        encoded = bytes(value)
        return encoded.hex() if any(encoded) else None
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None


def validate_pose(pose):
    if not isinstance(pose, dict):
        raise UserError('Điểm đến không hợp lệ.')
    values = {k: finite(pose.get(k)) for k in ('x', 'y', 'yaw')}
    if any(v is None for v in values.values()):
        raise UserError('Vị trí X, Y và hướng phải là số hữu hạn.')
    values['yaw'] = math.atan2(math.sin(values['yaw']), math.cos(values['yaw']))
    values['name'] = str(pose.get('name') or 'Điểm đến')[:80]
    return values


def offline_snapshot():
    return {
        'offline': True, 'ready': False, 'issues': ['OFFLINE: chưa kết nối robot.'],
        'lifecycle': {name: 'unavailable' for name in NAVIGATION_NODES},
        'localization': {'amcl': 'unavailable', 'map_server': 'unavailable'},
        'map_yaml': None, 'action_available': False, 'external_goal_active': False,
        'mux': {'available': False, 'mode': 'UNKNOWN', 'age': None,
                'output_topic': None, 'manual_ready': False},
        'mission': {'available': False, 'id': '', 'state_name': 'UNAVAILABLE',
                    'active': False, 'pending': False, 'receive_age': None,
                    'pickup_name': '', 'dropoff_name': '', 'message': '',
                    'distance_remaining': None, 'nav_error_code': 0},
        'goal': {'state': 'IDLE', 'active': False, 'id': '', 'message': '',
                 'distance_remaining': None, 'pose': None},
        'plan': {'frame_id': 'map', 'points': [], 'receive_age': None},
    }


class OfflineNavigation:
    def snapshot(self):
        return offline_snapshot()

    def command_epoch(self):
        return 0

    def _refuse(self, *args, **kwargs):
        raise UserError('OFFLINE: chức năng này cần kết nối ROS 2 và robot thật.')

    start_delivery = mission_action = navigate_to = cancel_navigation = set_manual = _refuse

    def stop_all(self):
        return {'success': False, 'mux_confirmed': False, 'mission_cancel_requested': False,
                'goal_cancel_requested': False, 'message': 'OFFLINE: không có robot để xác nhận STOP.'}


class NavigationRos:
    """Construct before RosAdapter starts its executor; snapshot is read-only."""

    def __init__(self, ros):
        from rclpy.action import ActionClient
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from lifecycle_msgs.srv import GetState
        from rcl_interfaces.srv import GetParameters
        from std_srvs.srv import Trigger, SetBool
        from std_msgs.msg import String
        from nav_msgs.msg import Path
        from geometry_msgs.msg import PoseStamped
        from action_msgs.msg import GoalStatusArray
        from nav2_msgs.action import NavigateToPose
        from delivery_robot_interfaces.msg import MissionState
        from delivery_robot_interfaces.srv import StartDelivery

        self.ros, self.node = ros, ros.node
        self.Trigger, self.SetBool = Trigger, SetBool
        self.GetState, self.GetParameters = GetState, GetParameters
        self.PoseStamped, self.NavigateToPose = PoseStamped, NavigateToPose
        self.StartDelivery = StartDelivery
        self.lock = threading.RLock()
        self.mux_condition = threading.Condition(self.lock)
        self.epoch = 0
        self.control_pending = set()
        self.control_requests = {}
        self.handle = None
        self.goal_send_future = self.goal_result_future = None
        self.goal_result_event = threading.Event()
        self.goal = offline_snapshot()['goal']
        self.mission = offline_snapshot()['mission']
        self.mission_at = 0.0
        self.mission_pending = None
        self.mux_mode, self.mux_at = 'UNKNOWN', 0.0
        self.params, self.params_at = {}, 0.0
        self.map_yaml = None
        self.plan, self.plan_at = {'frame_id': 'map', 'points': []}, 0.0
        self.lifecycle, self.lifecycle_at = {}, {}
        self.pending_queries = {}
        self.last_watchdog_stop = 0.0
        self.action_ids = set()
        self.action_providers = None
        self.action_observations = {}
        self.last_action_providers = ()
        self.retired_action_providers = set()
        self.mission_provider = self.mux_provider = None
        self.retired_mission_providers = set()
        self.retired_mux_providers = set()
        self.graph = {'motor_publishers': [], 'motor_receivers': [], 'nav_publishers': [],
                      'nav_receivers': [], 'manual_receivers': [], 'nodes': []}
        self.nav = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')
        self.clients = {
            'mode': self.node.create_client(SetBool, '/delivery_robot/cmd_mux/set_navigation'),
            'stop': self.node.create_client(Trigger, '/delivery_robot/cmd_mux/stop'),
            'start': self.node.create_client(StartDelivery, '/delivery_robot/mission/start'),
        }
        for name in ('confirm_pickup', 'confirm_dropoff', 'cancel', 'reset'):
            self.clients[name] = self.node.create_client(Trigger, '/delivery_robot/mission/' + name)
        self.lifecycle_clients = {
            name: self.node.create_client(GetState, '/' + name + '/get_state')
            for name in (*NAVIGATION_NODES, 'amcl', 'map_server')
        }
        self.param_client = self.node.create_client(GetParameters, '/delivery_robot_cmd_mux/get_parameters')
        self.map_param_client = self.node.create_client(GetParameters, '/map_server/get_parameters')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # Required two-argument callbacks ensure rclpy supplies MessageInfo;
        # optional second arguments can be classified as message-only callbacks.
        self._subscription_specs = {
            '/delivery_robot/mission/state': ('mission', MissionState, self._on_mission, qos, True),
            '/navigate_to_pose/_action/status': ('action_status', GoalStatusArray, self._on_action_status, qos, True),
            '/delivery_robot/cmd_mux/state': ('mux', String, self._on_mux, 10, True),
            '/plan': ('plan', Path, self._on_plan, 10, False),
            '/received_global_plan': ('plan', Path, self._on_plan, 10, False),
        }
        self._subscription_generations = {topic: 0 for topic in self._subscription_specs}
        self._subscriptions = {}
        for topic in self._subscription_specs:
            self._create_observation_subscription(topic)
        self.node.create_timer(0.75, self.ros.guard_callback('navigation_poll', self._poll, timer=True))

    def _create_observation_subscription(self, topic):
        name, message_type, callback, qos, with_info = self._subscription_specs[topic]
        generation = self._subscription_generations[topic]
        guarded = self.ros.guard_callback(name, callback, with_info=with_info)
        if with_info:
            def receive(message, info):
                if generation != self._subscription_generations[topic]:
                    return
                return guarded(message, info)
        else:
            def receive(message):
                if generation != self._subscription_generations[topic]:
                    return
                return guarded(message)
        self._subscriptions[topic] = self.node.create_subscription(message_type, topic, receive, qos)

    def managed_start_complete(self, components, valid=None):
        """Executor-only: renew observations after a verified stop/new owned start.

        Old queued subscription callbacks keep their previous generation. This
        permits a restarted provider to reuse a GID without accepting old data.
        """
        components = set(components)
        topics = set()
        if components & {'navigation', 'cmd_mux'}:
            topics.add('/delivery_robot/cmd_mux/state')
        if 'mission' in components:
            topics.add('/delivery_robot/mission/state')
        if 'navigation' in components:
            topics.update(('/navigate_to_pose/_action/status', '/plan', '/received_global_plan'))
        if not topics:
            return
        with self.lock:
            epoch = self.epoch
            for topic in topics:
                self._subscription_generations[topic] += 1
        for topic in topics:
            old = self._subscriptions.pop(topic, None)
            if old is not None:
                self.node.destroy_subscription(old)
            self._create_observation_subscription(topic)
        with self.lock:
            if self.epoch != epoch or (valid is not None and not valid()):
                for topic in topics:
                    self._subscription_generations[topic] += 1
                raise UserError('Khởi động đã bị dừng; không nhận dữ liệu của phiên cũ.')
            if components & {'navigation', 'cmd_mux'}:
                self.retired_mux_providers.clear()
                self.mux_provider = None
                self.mux_mode = 'UNKNOWN'; self.mux_at = self.params_at = 0.0
            if 'mission' in components:
                self.retired_mission_providers.clear()
                self.mission_provider = None
                self.mission = offline_snapshot()['mission']
                self.mission_at = 0.0; self.mission_pending = None
            if 'navigation' in components:
                self.retired_action_providers.clear()
                self.action_providers = None; self.last_action_providers = ()
                self.action_observations.clear(); self.action_ids.clear()
                self.plan = {'frame_id': 'map', 'points': []}; self.plan_at = 0.0
                for name in NAVIGATION_NODES:
                    self.lifecycle_at[name] = 0.0

    def _on_mux(self, msg, info=None):
        with self.lock:
            gid = publisher_gid(info)
            if gid in self.retired_mux_providers:
                return
            if gid is not None:
                self.mux_provider = gid
            self.mux_mode = msg.data if msg.data in ('STOP', 'MANUAL', 'NAVIGATION') else 'UNKNOWN'
            self.mux_at = time.monotonic()
            self.mux_condition.notify_all()

    def _on_mission(self, msg, info=None):
        with self.lock:
            gid = publisher_gid(info)
            if gid in self.retired_mission_providers:
                return
            if gid is not None:
                self.mission_provider = gid
            self.mission_at = time.monotonic()
            self.mission = {
                'id': str(msg.mission_id), 'state': int(msg.state), 'state_name': str(msg.state_name),
                'pickup_name': str(msg.pickup_name), 'dropoff_name': str(msg.dropoff_name),
                'distance_remaining': finite(msg.distance_remaining),
                'nav_error_code': int(msg.nav_error_code), 'message': str(msg.message),
                'active': msg.state_name in ACTIVE_MISSION_STATES, 'pending': False,
            }
            if self.mission_pending and msg.mission_id and (
                msg.mission_id == self.mission_pending.get('accepted_id') or
                msg.mission_id != self.mission_pending.get('previous_id')
            ):
                self.mission_pending = None

    @staticmethod
    def _uuid(value):
        return bytes(value.uuid).hex()

    def _on_action_status(self, msg, info=None):
        with self.lock:
            ids = {self._uuid(s.goal_info.goal_id) for s in msg.status_list
                   if s.status in (1, 2, 3)}
            gid = publisher_gid(info)
            if gid in self.retired_action_providers:
                return
            if gid is not None:
                self.action_observations[gid] = ids
                if self.action_providers:
                    self.action_ids = set().union(*(self.action_observations.get(p, set())
                                                    for p in self.action_providers))
                else:
                    self.action_ids = ids
            else:
                self.action_ids = ids

    def _on_plan(self, msg):
        if msg.header.frame_id != 'map':
            return
        poses = msg.poses
        step = max(1, math.ceil(len(poses) / 600))
        points = []
        for pose in poses[::step]:
            x, y = finite(pose.pose.position.x), finite(pose.pose.position.y)
            if x is not None and y is not None:
                points.append([x, y])
        with self.lock:
            self.plan = {'frame_id': 'map', 'points': points}
            self.plan_at = time.monotonic()

    def _observe_action_providers(self, providers):
        with self.lock:
            providers = tuple(sorted(p for p in providers if p not in self.retired_action_providers))
            old_providers = self.action_providers
            self.action_providers = providers
            if old_providers != providers:
                # Discovery may temporarily disappear without the action ending.
                # Status is event-driven, so retain it until a replacement server
                # or a terminal status supplies evidence that the goal ended.
                if not providers:
                    return
                self.action_ids = set().union(*(self.action_observations.get(p, set()) for p in providers))
                old_goal_provider = self.goal.get('providers')
                if (providers and old_goal_provider and set(providers).isdisjoint(old_goal_provider)
                        and self.goal['state'] in ACTIVE_GOAL_STATES):
                    self._retire_goal('Nav2 đã khởi động lại; mục tiêu trước bị gián đoạn.')
                if providers:
                    self.retired_action_providers.update(set(self.last_action_providers) - set(providers))
                    self.last_action_providers = providers
                self.action_observations = {p: self.action_observations[p] for p in providers
                                            if p in self.action_observations}

    def _query(self, key, client, request, apply):
        now = time.monotonic()
        observation_epoch = getattr(self.ros, 'observation_epoch', 0)
        old = self.pending_queries.get(key)
        if old and not old[0].done() and now - old[1] < 3.0:
            return
        if old and not old[0].done():
            old[0].cancel()  # Read-only query; release the pending local wait.
        if not client.service_is_ready():
            return
        try:
            future = client.call_async(request)
            self.pending_queries[key] = (future, now)
            def done(f):
                # Ignore an older delayed answer after a new query was sent.
                with self.lock:
                    if (self.pending_queries.get(key, (None,))[0] is not f or
                            observation_epoch != getattr(self.ros, 'observation_epoch', 0)):
                        return
                    try:
                        apply(f.result())
                    except Exception:
                        pass
            future.add_done_callback(done)
        except Exception:
            pass

    def _poll(self):
        for name, client in self.lifecycle_clients.items():
            def apply(response, name=name):
                with self.lock:
                    self.lifecycle[name] = str(response.current_state.label)
                    self.lifecycle_at[name] = time.monotonic()
            self._query('lc:' + name, client, self.GetState.Request(), apply)
        req = self.GetParameters.Request()
        req.names = ['output_topic', 'manual_topic', 'nav_topic', 'max_linear', 'max_angular']
        def parameters(response):
            values = response.values
            if len(values) != len(req.names):
                return
            with self.lock:
                self.params = {name: value.string_value if name.endswith('topic')
                               else finite(value.double_value) for name, value in zip(req.names, values)}
                self.params_at = time.monotonic()
        self._query('mux:params', self.param_client, req, parameters)
        map_req = self.GetParameters.Request(); map_req.names = ['yaml_filename']
        def map_parameters(response):
            if response.values and response.values[0].string_value:
                with self.lock:
                    self.map_yaml = response.values[0].string_value
        self._query('map:params', self.map_param_client, map_req, map_parameters)
        def names(entries):
            return [e.node_namespace.rstrip('/') + '/' + e.node_name for e in entries
                    if e.topic_type == 'geometry_msgs/msg/Twist']
        def endpoints(topic, kind='publisher'):
            getter = (self.node.get_publishers_info_by_topic if kind == 'publisher'
                      else self.node.get_subscriptions_info_by_topic)
            entries = getter(topic)
            filter_entries = getattr(self.ros, 'filter_endpoints', None)
            return filter_entries(topic, entries, kind) if filter_entries is not None else entries
        try:
            graph = {
                'motor_publishers': names(endpoints('/cmd_vel')),
                'motor_receivers': names(endpoints('/cmd_vel', 'subscription')),
                'nav_publishers': names(endpoints('/cmd_vel_nav')),
                'nav_receivers': names(endpoints('/cmd_vel_nav', 'subscription')),
                'manual_receivers': names(endpoints('/cmd_vel_manual', 'subscription')),
                'nodes': [ns.rstrip('/') + '/' + n for n, ns in self.node.get_node_names_and_namespaces()],
            }
            providers = tuple(sorted(gid for p in endpoints('/navigate_to_pose/_action/status')
                                     if (gid := publisher_gid({'publisher_gid': getattr(p, 'endpoint_gid', None)}))))
            with self.lock:
                self.graph = graph
            self._observe_action_providers(providers)
        except Exception:
            pass
        now = time.monotonic()
        with self.lock:
            mission_stale = bool(self.mission.get('active') and self.mission_at and now-self.mission_at > 3.0)
        if mission_stale and now-self.last_watchdog_stop > 1.0:
            self.last_watchdog_stop = now
            self._stop_async()

    def snapshot(self):
        now = time.monotonic()
        base = self.ros.status()
        with self.lock:
            lifecycle = {name: self.lifecycle.get(name, 'unavailable')
                         if now - self.lifecycle_at.get(name, 0) < 3.0 else 'unavailable'
                         for name in (*NAVIGATION_NODES, 'amcl', 'map_server')}
            mux_age = now - self.mux_at if self.mux_at else None
            mux_available = (self.clients['mode'].service_is_ready() and
                             self.clients['stop'].service_is_ready() and
                             mux_age is not None and mux_age < 1.5 and now-self.params_at < 3.0)
            graph = copy.deepcopy(self.graph)
            manual_ready = (base.get('ros_ok') is not False and
                            base.get('graph_fresh') is not False and
                            mux_available and self.params.get('output_topic') == '/cmd_vel' and
                            graph['nodes'].count('/delivery_robot_cmd_mux') == 1 and
                            self.params.get('manual_topic') == '/cmd_vel_manual' and
                            graph['motor_publishers'] == ['/delivery_robot_cmd_mux'] and
                            graph['motor_receivers'].count('/delivery_robot_motor') == 1 and
                            graph['manual_receivers'].count('/delivery_robot_cmd_mux') == 1)
            mux = dict(self.params, available=mux_available, mode=self.mux_mode,
                       age=mux_age, receive_age=mux_age, manual_ready=manual_ready,
                       output_topic=self.params.get('output_topic'))
            mission = copy.deepcopy(self.mission)
            mission_age = now-self.mission_at if self.mission_at else None
            mission['receive_age'] = mission_age
            mission['available'] = bool(self.clients['start'].service_is_ready() and
                                        graph['nodes'].count('/delivery_robot_mission_manager') == 1 and
                                        mission_age is not None and mission_age < 3.0)
            if self.mission_pending:
                mission['pending'] = True
                mission['active'] = True
                mission['message'] = 'Đang xác nhận yêu cầu nhiệm vụ; không gửi lại.'
            if mission.get('active') and not mission['available']:
                mission['message'] = 'Mất phản hồi Mission Manager; trạng thái nhiệm vụ chưa được xác nhận.'
            goal = copy.deepcopy(self.goal)
            goal['active'] = goal['state'] in ACTIVE_GOAL_STATES
            external = bool(self.action_ids - ({goal['id']} if goal.get('id') else set()))
            action_ready = self.nav.server_is_ready()
            if goal['active'] and not action_ready:
                goal.update(state='UNKNOWN', message='Mất kết nối Nav2; kết quả mục tiêu chưa xác nhận.')
            plan = copy.deepcopy(self.plan)
            plan['receive_age'] = now-self.plan_at if self.plan_at else None
            if not action_ready or (plan['receive_age'] is not None and plan['receive_age'] > 15.0):
                plan['points'] = []
            map_yaml = self.map_yaml
        issues = []
        if base.get('ros_ok') is False:
            issues.append(base.get('ros_error') or 'Luồng xử lý ROS không sẵn sàng.')
        for name in NAVIGATION_NODES:
            if lifecycle[name] != 'active':
                issues.append(f'{name}: {lifecycle[name]}')
            if graph['nodes'].count('/' + name) > 1:
                issues.append(f'Trùng node /{name}.')
        if not action_ready:
            issues.append('Chưa có action /navigate_to_pose.')
        if not manual_ready:
            issues.append('Command Mux /cmd_vel chưa sẵn sàng hoặc có nguồn điều khiển trùng.')
        if self.params.get('nav_topic') != '/cmd_vel_nav' or graph['nav_publishers'] != ['/collision_monitor']:
            issues.append('Chưa xác nhận Collision Monitor xuất /cmd_vel_nav tới mux.')
        if graph['nav_receivers'].count('/delivery_robot_cmd_mux') != 1:
            issues.append('Mux chưa nhận đúng một luồng /cmd_vel_nav.')
        for name in ('amcl', 'map_server'):
            if lifecycle[name] != 'active':
                issues.append(f'{name} chưa active.')
        issues.extend(base.get('mapping_issues', []))
        issues.extend(base.get('motion_issues', []))
        tf = base.get('tf_map') or {}
        tf_age = finite(tf.get('age'))
        if tf_age is None or not -1.0 <= tf_age <= 0.8:
            issues.append('Chưa có TF map → base_link mới.')
        if base.get('slam_state') == 'active':
            issues.append('Dừng tạo bản đồ trước khi giao hàng trên bản đồ đã lưu.')
        return {'offline': False, 'ready': not issues, 'issues': list(dict.fromkeys(issues)),
                'lifecycle': {n: lifecycle[n] for n in NAVIGATION_NODES},
                'localization': {n: lifecycle[n] for n in ('amcl', 'map_server')},
                'map_yaml': map_yaml, 'action_available': action_ready,
                'external_goal_active': external, 'command_pending': bool(self.control_pending), 'mux': mux,
                'mission': mission, 'goal': goal, 'plan': plan}

    def _assert_idle(self, navigation=True):
        s = self.snapshot()
        if self.control_pending:
            raise UserError('Một lệnh chuyển quyền điều khiển chưa được xác nhận; chưa gửi lệnh mới.')
        if self.ros.motion.status().get('armed'):
            raise UserError('Khóa điều khiển thủ công trước khi bắt đầu điều hướng.')
        if s['mission']['active'] or s['goal']['active'] or s['external_goal_active']:
            raise UserError('Đã có nhiệm vụ/mục tiêu đang chạy hoặc đang chờ xác nhận hủy.')
        if navigation and not s['ready']:
            raise UserError('Navigation chưa sẵn sàng: ' + '; '.join(s['issues'][:5]))
        if not s['mux']['manual_ready']:
            raise UserError('Chưa xác nhận Command Mux thật xuất /cmd_vel.')
        return s

    def _call(self, client, request, timeout=3.0, epoch=None, late=None):
        """Keep the future alive after timeout so a late mutation can be stopped."""
        available = getattr(self.ros, 'executor_available', None)
        if available is not None and not available():
            raise UserError('ROS executor không phản hồi; yêu cầu chưa được gửi.')
        event, expired = threading.Event(), threading.Event()
        box = {}
        deadline = time.monotonic() + timeout
        marker = object() if epoch is not None else None
        record = {'client': client, 'future': None, 'retired': False}
        if marker is not None:
            with self.lock:
                self.control_pending.add(marker)
                self.control_requests[marker] = record
        def release():
            if marker is not None:
                with self.lock:
                    self.control_pending.discard(marker)
                    self.control_requests.pop(marker, None)
        def start():
            if record['retired']:
                box['error'] = 'Tiến trình xử lý lệnh đã đóng.'; box['retired'] = True
                release(); event.set(); return
            if time.monotonic() > deadline or (epoch is not None and epoch != self.epoch):
                box['error'] = 'Yêu cầu đã hết hạn hoặc đã bị hủy trước khi gửi.'; release(); event.set(); return
            if not client.service_is_ready():
                box['error'] = 'Dịch vụ ROS chưa sẵn sàng.'; release(); event.set(); return
            try:
                future = client.call_async(request)
                record['future'] = future
                box['sent'] = True
                def done(f):
                    if record['retired']:
                        box['error'] = 'Tiến trình xử lý lệnh đã đóng.'; box['retired'] = True
                        release(); event.set(); return
                    try:
                        box['value'] = f.result()
                    except Exception as exc:
                        box['error'] = str(exc)
                    try:
                        stale = expired.is_set() or time.monotonic() > deadline or (
                            epoch is not None and epoch != self.epoch)
                        if (stale or ('error' in box and marker is not None)) and late:
                            late()
                            self._stop_async(after=release)
                        else:
                            release()
                    finally:
                        event.set()
                future.add_done_callback(done)
            except Exception as exc:
                box['error'] = str(exc); release(); event.set()
        self.ros.jobs.put(start)
        if not event.wait(timeout):
            expired.set()
            error = UserError('Hết thời gian xác nhận ROS. Không tự gửi lại; dùng Dừng để kết thúc yêu cầu.')
            error.uncertain = True
            raise error
        if 'error' in box:
            error = UserError(box['error'])
            error.uncertain = box.get('sent', False)
            error.retired = box.get('retired', False)
            raise error
        if epoch is not None and epoch != self.epoch:
            raise UserError('Yêu cầu đã bị hủy.')
        return box.get('value')

    def _stop_async(self, after=None):
        """Executor-only. Also used to neutralize a late mode response."""
        try:
            if self.clients['stop'].service_is_ready():
                future = self.clients['stop'].call_async(self.Trigger.Request())
                if after is not None:
                    def done(f):
                        try:
                            if f.result().success:
                                after()
                        except Exception:
                            pass
                    future.add_done_callback(done)
        except Exception:
            pass

    def _cancel_mission_async(self):
        try:
            if self.clients['cancel'].service_is_ready():
                future = self.clients['cancel'].call_async(self.Trigger.Request())
                future.add_done_callback(lambda _f: self._stop_async())
        except Exception:
            pass
        finally:
            self._stop_async()

    def _mode(self, navigation, epoch):
        req = self.SetBool.Request(); req.data = navigation
        response = self._call(self.clients['mode'], req, epoch=epoch, late=self._stop_async)
        if response is None or not response.success:
            raise UserError('Command Mux từ chối chuyển chế độ.')
        return {'success': True, 'message': str(response.message)}

    def command_epoch(self):
        """Capture an intent before waiting on HTTP locks or validating a map."""
        with self.lock:
            return self.epoch

    def _check_dispatch(self, expected_epoch, dispatch_check):
        # Called with self.lock held, immediately before reserving a mutation.
        # STOP uses the same lock to invalidate already reserved requests.
        if expected_epoch is not None and expected_epoch != self.epoch:
            raise UserError('Yêu cầu đã bị hủy bởi lệnh dừng hoặc thao tác mới hơn.')
        if dispatch_check is not None:
            dispatch_check()

    def set_manual(self, *, expected_epoch=None, dispatch_check=None):
        with self.lock:
            self._assert_idle(navigation=False)
            self._check_dispatch(expected_epoch, dispatch_check)
            self.epoch += 1
            epoch = self.epoch
        result = self._mode(False, epoch)
        with self.mux_condition:
            observed = self.mux_condition.wait_for(
                lambda: epoch != self.epoch or self.manual_authority(), timeout=1.0)
        if not observed or epoch != self.epoch or not self.manual_authority():
            self.ros.jobs.put(self._stop_async)
            raise UserError('Chưa quan sát được quyền MANUAL; không mở điều khiển bánh.')
        return result

    def manual_authority(self):
        with self.lock:
            return self.mux_mode == 'MANUAL' and time.monotonic() - self.mux_at < 1.0

    def _pose(self, data):
        msg = self.PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x = data['x']; msg.pose.position.y = data['y']
        msg.pose.orientation.z = math.sin(data['yaw'] / 2)
        msg.pose.orientation.w = math.cos(data['yaw'] / 2)
        return msg

    def start_delivery(self, pickup, dropoff, *, expected_epoch=None, dispatch_check=None):
        pickup, dropoff = validate_pose(pickup), validate_pose(dropoff)
        with self.lock:
            s = self._assert_idle()
            if not s['mission']['available']:
                raise UserError('Mission Manager chưa sẵn sàng.')
            self._check_dispatch(expected_epoch, dispatch_check)
            self.epoch += 1; epoch = self.epoch
            self.mission_pending = {'previous_id': self.mission.get('id', ''), 'started': time.monotonic()}
        req = self.StartDelivery.Request()
        req.pickup_name = pickup['name']; req.dropoff_name = dropoff['name']
        req.pickup_pose = self._pose(pickup); req.dropoff_pose = self._pose(dropoff)
        req.require_pickup_confirmation = True; req.require_dropoff_confirmation = True
        try:
            response = self._call(self.clients['start'], req, epoch=epoch,
                                  late=self._cancel_mission_async)
        except UserError as exc:
            # Keep an uncertain start reserved until observation or STOP resolves it.
            if not getattr(exc, 'uncertain', True):
                with self.lock:
                    self.mission_pending = None
            if not getattr(exc, 'retired', False):
                self.ros.jobs.put(self._cancel_mission_async)
            raise
        if response is None or not response.accepted:
            with self.lock:
                self.mission_pending = None
            raise UserError(str(getattr(response, 'message', 'Mission Manager từ chối nhiệm vụ.')))
        with self.lock:
            if self.mission_pending:
                self.mission_pending['accepted_id'] = response.mission_id
                if self.mission.get('id') == response.mission_id:
                    self.mission_pending = None
        return {'accepted': True, 'mission_id': response.mission_id, 'message': response.message}

    def mission_action(self, action, *, expected_epoch=None, dispatch_check=None):
        if action not in ('confirm_pickup', 'confirm_dropoff', 'cancel', 'reset'):
            raise UserError('Thao tác nhiệm vụ không hợp lệ.')
        if action == 'cancel':
            result = self.stop_all()
            if not result['mux_confirmed']:
                raise UserError(result['message'])
            return {'success': True, 'message': 'Mux đã xác nhận STOP; theo dõi trạng thái nhiệm vụ để xác nhận hủy.',
                    'cancel_confirmed': result['cancel_confirmed']}
        with self.lock:
            if action in ('confirm_pickup', 'confirm_dropoff'):
                s = self.snapshot()
                if not s['mission']['available']:
                    raise UserError('Mất phản hồi Mission Manager; chưa thể xác nhận.')
                expected = 'WAITING_PICKUP' if action == 'confirm_pickup' else 'WAITING_DROPOFF'
                if s['mission']['state_name'] != expected:
                    raise UserError('Nhiệm vụ chưa ở bước chờ xác nhận tương ứng.')
                if action == 'confirm_pickup' and not s['ready']:
                    raise UserError('Hệ thống chưa sẵn sàng: ' + '; '.join(s['issues'][:4]))
            self._check_dispatch(expected_epoch, dispatch_check)
            epoch = self.epoch
        response = self._call(self.clients[action], self.Trigger.Request(),
                              epoch=epoch,
                              late=self._cancel_mission_async if action == 'confirm_pickup' else None)
        if response is None or not response.success:
            raise UserError(str(getattr(response, 'message', 'Chưa xác nhận thao tác nhiệm vụ.')))
        return {'success': True, 'message': response.message}

    def navigate_to(self, pose, *, expected_epoch=None, dispatch_check=None):
        pose = validate_pose(pose)
        with self.lock:
            self._assert_idle()
            self._check_dispatch(expected_epoch, dispatch_check)
            self.epoch += 1; epoch = self.epoch
            self.goal = {'state': 'SENDING', 'active': True, 'id': '', 'pose': pose,
                         'token': epoch, 'request_sent': False,
                         'providers': tuple(self.action_providers or ()), 'retired': False,
                         'message': 'Đang gửi mục tiêu.', 'distance_remaining': None}
            self.goal_result_event.clear()
        try:
            self._mode(True, epoch)
        except UserError as exc:
            with self.lock:
                if self.goal.get('token') == epoch and not self.goal.get('retired'):
                    state = 'FAILED' if epoch == self.epoch else 'CANCELED'
                    self.goal.update(state=state, message='Yêu cầu điều hướng đã kết thúc trước khi gửi goal.')
                    self.goal_result_event.set()
            if not getattr(exc, 'retired', False):
                self.ros.jobs.put(self._stop_async)
            raise
        event, expired = threading.Event(), threading.Event()
        result = {}
        def send():
            if epoch != self.epoch or expired.is_set():
                with self.lock:
                    if self.goal.get('token') == epoch and not self.goal.get('retired'):
                        self.goal.update(state='CANCELED', message='Đã hủy trước khi gửi mục tiêu.')
                        self.goal_result_event.set()
                result['error'] = 'Mục tiêu đã bị hủy trước khi gửi.'; event.set(); return
            goal = self.NavigateToPose.Goal(); goal.pose = self._pose(pose)
            try:
                with self.lock:
                    self.goal['request_sent'] = True
                future = self.nav.send_goal_async(goal, feedback_callback=lambda f: self._feedback(f, epoch))
                self.goal_send_future = future
                def accepted(f):
                    try:
                        handle = f.result()
                        if handle is None or not handle.accepted:
                            raise UserError('Nav2 từ chối mục tiêu.')
                        ident = self._uuid(handle.goal_id)
                        retired = self.goal.get('token') != epoch or self.goal.get('retired', False)
                        stale = epoch != self.epoch or expired.is_set()
                        with self.lock:
                            if not stale and not retired:
                                self.handle = handle
                                self.goal.update(state='ACTIVE', id=ident, message='Robot đang đi đến điểm đã chọn.')
                        final = handle.get_result_async()
                        if not retired:
                            self.goal_result_future = final
                        final.add_done_callback(lambda ff: self._result(ff, epoch))
                        if stale or retired:
                            handle.cancel_goal_async()
                            if not retired:
                                self._stop_async()
                            result['error'] = 'Mục tiêu nhận muộn; đã yêu cầu hủy và STOP.'
                        else:
                            result.update(accepted=True, goal_id=ident, message='Nav2 đã nhận mục tiêu.')
                    except Exception as exc:
                        result['error'] = str(exc)
                        with self.lock:
                            if self.goal.get('token') == epoch and not self.goal.get('retired'):
                                self.goal.update(state='FAILED' if epoch == self.epoch else 'CANCELED',
                                                 message=str(exc))
                                self.goal_result_event.set()
                        if self.goal.get('token') == epoch and not self.goal.get('retired'):
                            self._stop_async()
                    event.set()
                future.add_done_callback(accepted)
            except Exception as exc:
                with self.lock:
                    if self.goal.get('token') == epoch and not self.goal.get('retired'):
                        self.goal.update(state='FAILED', message=str(exc))
                        self.goal_result_event.set()
                result['error'] = str(exc); event.set()
                if self.goal.get('token') == epoch and not self.goal.get('retired'):
                    self._stop_async()
        self.ros.jobs.put(send)
        if not event.wait(4.0):
            expired.set()
            with self.lock:
                relevant = self.goal.get('token') == epoch and not self.goal.get('retired')
                if relevant:
                    self.goal.update(state='UNKNOWN', message='Chưa xác nhận mục tiêu; đang yêu cầu STOP.')
            if relevant:
                self.ros.jobs.put(self._stop_async)
            raise UserError('Nav2 chưa trả lời mục tiêu. Không gửi lại; dùng Dừng để kết thúc.')
        if 'error' in result:
            raise UserError(result['error'])
        return result

    def _feedback(self, feedback, epoch):
        with self.lock:
            if epoch == self.epoch and self.goal['state'] == 'ACTIVE':
                self.goal['distance_remaining'] = finite(feedback.feedback.distance_remaining)

    def _retire_goal(self, message):
        """Caller holds self.lock and has evidence the old action server ended."""
        self.epoch += 1
        self.goal.update(state='FAILED', active=False, retired=True, message=message,
                         distance_remaining=None)
        self.handle = None
        self.goal_result_event.set()
        for future in (self.goal_send_future, self.goal_result_future):
            if future is not None and not future.done():
                future.cancel()
        self.goal_send_future = self.goal_result_future = None

    def managed_shutdown_complete(self, components):
        """Retire observations ONLY after owned process shutdown is verified.

        Internal process-manager/coordinator hook, not a browser API. A mere
        service timeout must never call this method. Remote request futures are
        canceled locally after their server processes have stopped; any late
        callback is marked retired and cannot stop or mutate a new server.
        """
        components = set(components)
        if not components <= {'navigation', 'mission', 'cmd_mux'}:
            raise ValueError('Unsupported managed component')
        retired_clients = set()
        with self.lock:
            self.epoch += 1
            if 'navigation' in components:
                if self.goal['state'] in ACTIVE_GOAL_STATES:
                    self._retire_goal('Tiến trình Nav2 đã đóng; mục tiêu bị gián đoạn.')
                self.action_ids.clear()
                self.retired_action_providers.update(self.last_action_providers)
                self.retired_action_providers.update(self.goal.get('providers', ()))
                self.action_observations.clear()
                self.action_providers = ()
                self.plan = {'frame_id': 'map', 'points': []}; self.plan_at = 0.0
                for name in NAVIGATION_NODES:
                    self.lifecycle_at[name] = 0.0
            if 'mission' in components:
                if self.mission_provider:
                    self.retired_mission_providers.add(self.mission_provider)
                if self.mission.get('active') or self.mission_pending:
                    ident = self.mission.get('id', '')
                    if self.mission_pending:
                        ident = self.mission_pending.get('accepted_id') or ident
                    self.mission.update(id=ident, state=7, state_name='ERROR', active=False,
                                        pending=False, message='Mission Manager đã đóng; nhiệm vụ bị gián đoạn.')
                self.mission_pending = None; self.mission_at = 0.0
                retired_clients.update(self.clients.get(name) for name in
                                       ('start', 'confirm_pickup', 'confirm_dropoff', 'cancel', 'reset'))
            if components & {'navigation', 'cmd_mux'}:
                if self.mux_provider:
                    self.retired_mux_providers.add(self.mux_provider)
                self.mux_at = self.params_at = 0.0
                self.mux_mode = 'UNKNOWN'
                retired_clients.update((self.clients['mode'], self.clients['stop']))
            for marker, record in list(self.control_requests.items()):
                if record['client'] in retired_clients:
                    record['retired'] = True
                    future = record['future']
                    self.control_pending.discard(marker)
                    self.control_requests.pop(marker, None)
                    if future is not None and not future.done():
                        future.cancel()
            return {'mission': copy.deepcopy(self.mission), 'goal': copy.deepcopy(self.goal)}

    def _result(self, future, epoch):
        try:
            wrapped = future.result()
            code = int(wrapped.result.error_code)
            if wrapped.status == 4 and code == 0:
                state, message = 'SUCCEEDED', 'Đã đến điểm đích.'
            elif wrapped.status == 5:
                state, message = 'CANCELED', 'Nav2 đã xác nhận hủy mục tiêu.'
            else:
                state, message = 'FAILED', str(wrapped.result.error_msg or 'Điều hướng thất bại.')
        except Exception as exc:
            state, message, code = 'FAILED', str(exc), 0
        with self.lock:
            # Cancel increments the epoch; a final result still settles the same
            # canceled goal, but never overwrites a newer goal.
            relevant = self.goal.get('token') == epoch and not self.goal.get('retired')
            if relevant:
                self.handle = None
                self.goal.update(state=state, message=message, error_code=code,
                                 distance_remaining=0.0 if state == 'SUCCEEDED' else None)
                self.goal_result_event.set()
        if relevant:
            self._stop_async()

    def cancel_navigation(self):
        with self.lock:
            active = self.goal['state'] in ACTIVE_GOAL_STATES
            if not active:
                return {'success': True, 'cancel_requested': False,
                        'confirmed': True, 'message': 'Không có mục tiêu riêng của Web đang chạy.'}
            if self.goal['state'] != 'CANCELING':
                self.epoch += 1
            self.goal.update(state='CANCELING', message='Đang chờ Nav2 xác nhận hủy.')
            handle = self.handle
            token = self.goal.get('token')
        available = getattr(self.ros, 'executor_available', None)
        if available is not None and not available():
            return {'success': False, 'cancel_requested': False, 'confirmed': False,
                    'ros_unavailable': True,
                    'message': 'ROS executor không phản hồi; chưa gửi được yêu cầu hủy Nav2.'}
        event = threading.Event(); result = {}
        def cancel():
            with self.lock:
                if self.goal.get('token') != token or self.goal.get('retired'):
                    event.set(); return
            self._stop_async()
            if handle is None:
                # Pending acceptance is canceled in its callback.
                event.set(); return
            try:
                future = handle.cancel_goal_async()
                def done(f):
                    try:
                        response = f.result()
                        result['accepted'] = bool(response.goals_canceling)
                    except Exception as exc:
                        result['error'] = str(exc)
                    event.set()
                future.add_done_callback(done)
            except Exception as exc:
                result['error'] = str(exc); event.set()
        self.ros.jobs.put(cancel)
        event.wait(2.0)
        confirmed = self.goal_result_event.wait(0.5)
        return {'success': confirmed, 'cancel_requested': True, 'confirmed': confirmed,
                'message': ('Nav2 đã kết thúc mục tiêu.' if confirmed else
                            'Đã yêu cầu hủy; đang chờ kết quả Nav2. Mux được yêu cầu STOP.')}

    def stop_all(self):
        self.ros.motion.stop('Người vận hành yêu cầu STOP.', force=True)
        with self.lock:
            # Invalidate a pending service/mode continuation immediately, before
            # any network wait. Its late callback will reassert STOP.
            self.epoch += 1
            mission_active = bool(self.mission.get('active') or self.mission_pending)
        available = getattr(self.ros, 'executor_available', None)
        if available is not None and not available():
            # Process shutdown must remain possible even when ROS callbacks died.
            # Do not spend several service timeouts on a queue nobody can drain.
            return {'success': False, 'mux_confirmed': False, 'ros_unavailable': True,
                    'mission_cancel_requested': False, 'goal_cancel_requested': False,
                    'goal_cancel_confirmed': False, 'cancel_confirmed': False,
                    'message': 'ROS executor không phản hồi; chưa có xác nhận STOP từ mux.',
                    'errors': ['Không gửi được yêu cầu ROS; tiếp tục đóng tiến trình do Web quản lý.']}
        errors = []
        mux_confirmed = False
        # Send STOP before waiting on any action or mission service.
        try:
            response = self._call(self.clients['stop'], self.Trigger.Request(), timeout=2.0)
            mux_confirmed = bool(response is not None and response.success)
        except UserError as exc:
            errors.append(str(exc))
        direct = self.cancel_navigation()
        mission_requested = False
        if mission_active:
            try:
                response = self._call(self.clients['cancel'], self.Trigger.Request(), timeout=2.0)
                mission_requested = bool(response is not None and response.success)
                if not mission_requested:
                    errors.append(str(getattr(response, 'message', 'Chưa xác nhận hủy nhiệm vụ.')))
            except UserError as exc:
                errors.append(str(exc))
        # Reassert STOP after potentially concurrent service continuations.
        try:
            response = self._call(self.clients['stop'], self.Trigger.Request(), timeout=1.5)
            mux_confirmed = bool(response is not None and response.success)
        except UserError as exc:
            mux_confirmed = False; errors.append(str(exc))
        return {'success': mux_confirmed, 'mux_confirmed': mux_confirmed,
                'mission_cancel_requested': mission_requested,
                'goal_cancel_requested': direct['cancel_requested'],
                'goal_cancel_confirmed': direct['confirmed'],
                'cancel_confirmed': bool(direct['confirmed'] and not self.snapshot()['mission']['active']),
                'message': ('Command Mux đã xác nhận STOP.' if mux_confirmed else
                            'Chưa xác nhận STOP từ Command Mux. Kiểm tra kết nối robot.'),
                'errors': errors}
