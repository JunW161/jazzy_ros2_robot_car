"""Start only the operator-verified robot commands, in the verified order.

Web V2.5 Map V4 keeps the verified base stack unchanged:
Agent -> Odom bridge(no TF) -> EKF official TF -> Robot model -> LiDAR.
SLAM and saved-map Localization (Map Server + AMCL) are mutually exclusive actions.
"""
from __future__ import annotations
import os
import math
import shlex
import signal
import socket
import subprocess
import threading
import time
from pathlib import Path
import yaml
from .core import UserError
from .agent_service import AgentService


NAVIGATION_NODES = (
    '/planner_server', '/controller_server', '/behavior_server', '/bt_navigator',
    '/waypoint_follower', '/velocity_smoother', '/collision_monitor',
)
FOUNDATION_COMPONENTS = ('agent', 'odom_bridge', 'ekf_odom', 'robot_model', 'lidar',
                         'localization', 'slam', 'costmap')
STOP_ORDER = ('mission', 'navigation', 'cmd_mux', 'rviz', 'costmap', 'localization', 'slam',
              'lidar', 'robot_model', 'ekf_odom', 'odom_bridge', 'agent')
COMPONENT_NODES = {
    'navigation': NAVIGATION_NODES + ('/delivery_robot_cmd_mux',
        '/global_costmap/global_costmap', '/local_costmap/local_costmap'),
    'mission': ('/delivery_robot_mission_manager',),
    'cmd_mux': ('/delivery_robot_cmd_mux',),
    'costmap': ('/global_costmap/global_costmap', '/local_costmap/local_costmap'),
    'localization': ('/amcl', '/map_server', '/lifecycle_manager_localization'),
    'slam': ('/slam_toolbox',), 'lidar': ('/ldlidar_publisher_ld14',),
    'robot_model': ('/robot_state_publisher',), 'ekf_odom': ('/ekf_filter_node_v14',),
    'odom_bridge': ('/delivery_odom_tf_joint_bridge',), 'rviz': ('/rviz2',),
}
DEPENDENTS = {
    'navigation': ('mission',), 'cmd_mux': ('mission', 'navigation'),
    'costmap': ('mission', 'navigation'),
    'localization': ('mission', 'navigation', 'costmap'),
    'slam': (),
    'lidar': ('mission', 'navigation', 'rviz', 'costmap', 'localization', 'slam'),
    'robot_model': ('mission', 'navigation', 'rviz', 'costmap', 'localization', 'slam'),
    'ekf_odom': ('mission', 'navigation', 'rviz', 'costmap', 'localization', 'slam'),
    'odom_bridge': ('mission', 'navigation', 'rviz', 'costmap', 'localization', 'slam'),
    'agent': STOP_ORDER[:-1],
}


class ProcessManager:
    def __init__(self, store, ros, offline=False):
        self.store, self.ros, self.offline = store, ros, offline
        self.lock = threading.RLock()
        self.children = {}
        self.agent = AgentService(store.config)
        self.agent_started = False
        self.operations = {}
        self.failures = {}
        self.closing = False
        self._discovery_cache = {}
        self.stop_lock = threading.RLock()
        self.confirmed_stops = set()

    def _owned_live(self, name):
        if name == 'agent':
            return self.agent_started
        with self.lock:
            item = self.children.get(name)
        return bool(item and (item['process'].poll() is None or
                             self._owned_group_running(item['process'].pid)))

    def _reset_after_stop(self, name):
        with self.lock:
            self.confirmed_stops.add(name)
            if name == 'navigation':
                self.confirmed_stops.update(('cmd_mux', 'costmap'))
            self.failures.pop(name, None)
            self._discovery_cache.clear()
        reset = getattr(self.ros, 'reset_observations', None)
        if reset is not None:
            reset([name])

    def _new_owner_started(self, name, restart_required):
        """A successful replacement may legitimately reuse its old DDS identity."""
        if restart_required:
            resume = getattr(self.ros, 'managed_start_complete', None)
            if resume is not None:
                resume([name])
        with self.lock:
            self.confirmed_stops.discard(name)
            if name == 'navigation':
                self.confirmed_stops.difference_update(('cmd_mux', 'costmap'))

    def _state_for_process_checks(self):
        """Do not turn a verified dead local process back into a DDS owner.

        A genuine matching local process still blocks duplicate startup. ROS
        readiness separately requires new data, so this only removes stale
        process-existence gates, never upgrades a sensor/TF to healthy.
        """
        state = dict(self.ros.status())
        nodes = list(state.get('nodes', []))
        with self.lock:
            stopped = tuple(self.confirmed_stops)
        for name in stopped:
            if self._owned_live(name):
                continue
            if name in ('navigation', 'mission', 'cmd_mux'):
                matches = self._autonomy_processes(name)
            elif name == 'localization':
                matches = self.recoverable_localization_processes()
            elif name == 'costmap':
                matches = self.recoverable_costmap_processes()
            else:
                matches = self.recoverable_base_processes(name) if name != 'agent' else []
            if matches:
                continue
            retired = COMPONENT_NODES.get(name, ())
            nodes = [node for node in nodes if node not in retired]
            if name == 'lidar':
                state['scan_publishers'] = 0
            elif name == 'agent':
                state['odom_publishers'] = 0
            elif name in ('costmap', 'navigation'):
                state['global_costmap_publishers'] = state['local_costmap_publishers'] = 0
        state['nodes'] = nodes
        return state

    def _emit_progress(self, progress, row):
        if progress is not None:
            try:
                progress(dict(row))
            except Exception:
                # UI observer failure must not interrupt process teardown.
                pass

    def _navigation_snapshot(self):
        navigation = getattr(self.ros, 'navigation', None)
        if navigation is None:
            return {}
        try:
            return navigation.snapshot()
        except Exception:
            # A disconnected observation must never be promoted to READY.
            return {}

    def _autonomy_processes(self, name):
        """Read-only discovery; matching a command never establishes ownership."""
        now = time.monotonic()
        with self.lock:
            cached = self._discovery_cache.get(name)
            if cached and now - cached[0] < 0.5:
                return list(cached[1])
        if name == 'cmd_mux':
            needles = ('/delivery_robot_cmd_mux/cmd_mux',
                       'ros2 launch delivery_robot_cmd_mux cmd_mux.launch.py')
        else:
            needles = (
            ('navigation_integrated_mux.launch.py',
             '/nav2_controller/controller_server', '/nav2_planner/planner_server',
             '/nav2_bt_navigator/bt_navigator',
             '/nav2_behaviors/behavior_server',
             '/nav2_waypoint_follower/waypoint_follower',
             '/nav2_velocity_smoother/velocity_smoother',
             '/nav2_collision_monitor/collision_monitor') if name == 'navigation' else
            ('/delivery_robot_mission/mission_manager',
             'ros2 launch delivery_robot_mission mission_manager.launch.py')
            )
        found = {}
        for needle in needles:
            for item in self.local_process_matches(needle):
                found[item['pid']] = item
        values = list(found.values())
        with self.lock:
            self._discovery_cache[name] = (now, values)
        return list(values)

    def _autonomy_status(self, state=None):
        state = self._state_for_process_checks() if state is None else state
        nodes = state.get('nodes', [])
        snapshot = self._navigation_snapshot()
        result = {}
        with self.lock:
            children = dict(self.children)
            operations = dict(self.operations)
            failures = dict(self.failures)
        for name in ('navigation', 'mission', 'cmd_mux'):
            item = children.get(name)
            code = item['process'].poll() if item else None
            orphaned = bool(item and code is not None and self._owned_group_running(item['process'].pid))
            owned = bool(item and (code is None or orphaned))
            matches = [] if self.offline else self._autonomy_processes(name)
            expected = (NAVIGATION_NODES if name == 'navigation' else
                        ('/delivery_robot_mission_manager',) if name == 'mission' else
                        ('/delivery_robot_cmd_mux',))
            observed = [node for node in expected if node in nodes]
            mux_present = '/delivery_robot_cmd_mux' in nodes
            present = owned or bool(matches or observed)
            external = present and not owned
            if name == 'navigation':
                ready = bool(snapshot.get('ready')) and present
                complete = all(nodes.count(node) == 1 for node in NAVIGATION_NODES) and nodes.count('/delivery_robot_cmd_mux') == 1
            elif name == 'mission':
                mission = snapshot.get('mission') or {}
                age = mission.get('receive_age')
                ready = bool(mission.get('available')) and (age is None or age < 2.5) and present
                complete = bool(observed)
            else:
                mux = snapshot.get('mux') or {}
                age = mux.get('receive_age', mux.get('age'))
                ready = bool(mux.get('available')) and (age is None or age < 2.5) and present
                complete = nodes.count('/delivery_robot_cmd_mux') == 1
            # Surviving children remain ours after their launch parent exits.
            # Even healthy ROS observations cannot make a broken launch ready.
            ready = ready and not orphaned and name not in failures
            if name in operations:
                status = operations[name]
            elif orphaned or failures.get(name):
                status = 'failed'
            elif present:
                status = 'external' if external else ('ready' if ready else 'starting')
                if owned and not ready and time.monotonic() - item.get('started_at', time.monotonic()) > 30:
                    status = 'degraded'
            elif failures.get(name) or (item and code not in (None, 0)):
                status = 'failed'
            else:
                status = 'stopped'
            if orphaned:
                message = ('Tiến trình launch đã thoát nhưng còn tiến trình con do Web sở hữu. '
                           'Dừng thành phần này trên Web trước khi khởi động lại.')
            elif failures.get(name) and owned:
                message = failures[name]
            elif external:
                message = ('Đang chạy ngoài phiên Web này; Web quan sát và gửi yêu cầu ROS, '
                           'không sở hữu tiến trình. Dừng launch tại nơi đã mở.')
                if not complete:
                    message += ' Chỉ thấy một phần stack; không mở thêm bản trùng.'
            elif ready:
                message = 'Đã xác nhận dữ liệu/trạng thái ROS sẵn sàng.'
            elif owned:
                message = 'Tiến trình đã mở; đang chờ các node và dịch vụ sẵn sàng.'
            else:
                message = failures.get(name) or 'Chưa khởi động.'
            recoverable = [m for m in matches if self._project_identity_matches(name, m)]
            if external and recoverable:
                message = 'Tiến trình dự án đã được xác minh; có thể dừng từ Web.'
            result[name] = {
                'running': present, 'managed': owned, 'external': external,
                'recoverable': bool(recoverable) and not owned, 'ready': ready, 'status': status,
                'complete': complete, 'message': message,
                'pid': item['process'].pid if owned else (matches[0]['pid'] if matches else None),
                'returncode': code, 'orphaned': orphaned, 'context': dict(item.get('context', {})) if item else {},
            }
        if result['cmd_mux']['running'] and result['navigation']['running'] and not result['cmd_mux']['managed']:
            result['cmd_mux']['context']['provided_by'] = 'navigation'
            result['cmd_mux']['message'] = 'Command mux nằm trong stack Navigation; dừng bằng thao tác Navigation.'
        return result

    def navigation_present(self):
        return self._autonomy_status()['navigation']['running']

    def _require_navigation_stopped(self, operation):
        if self.navigation_present():
            raise UserError('Dừng Navigation trước khi ' + operation + '. '
                            'Navigation chạy ngoài Web phải được dừng tại Terminal đã mở.')

    @staticmethod
    def _fresh_age(value, minimum=-0.75, maximum=1.0):
        return isinstance(value, (int, float)) and math.isfinite(value) and minimum <= value < maximum

    def _navigation_preflight(self, state):
        issues = []
        nodes = state.get('nodes', [])
        if '/slam_toolbox' in nodes:
            issues.append('Dừng SLAM và bật định vị trên bản đồ đã lưu.')
        if nodes.count('/amcl') != 1:
            issues.append('Cần đúng một AMCL đang chạy.')
        for field, label in (('tf_odom', 'odom→base_link'), ('tf_map', 'map→base_link')):
            if not self._fresh_age((state.get(field) or {}).get('age')):
                issues.append('Chưa có TF ' + label + ' mới; kiểm tra định vị/Initial Pose.')
        if state.get('scan_publishers') != 1 or not self._fresh_age(
                (state.get('scan') or {}).get('receive_age'), minimum=0.0, maximum=0.7):
            issues.append('Cần đúng một nguồn LiDAR và dữ liệu /scan mới.')
        if state.get('odom_publishers') != 1 or not self._fresh_age(
                (state.get('odom') or {}).get('receive_age'), minimum=0.0, maximum=0.5):
            issues.append('Cần đúng một nguồn odometry và dữ liệu encoder mới.')
        if not state.get('tf_odom_laser'):
            issues.append('Thiếu TF giữa robot và LiDAR.')
        if issues:
            raise UserError('Chưa khởi động Navigation: ' + ' '.join(issues))

    def _start_autonomy(self, name, cancel_event=None):
        """Reserve the component briefly; process startup never holds the status lock."""
        with self.lock:
            if self.closing or (cancel_event is not None and cancel_event.is_set()):
                raise UserError('Web đang đóng; không khởi động thêm tiến trình.')
            existing = self.children.get(name)
            if existing and existing['process'].poll() is None:
                return {'message': 'Đã chạy bởi Web; không mở bản trùng.', 'managed': True}
            if existing and self._owned_group_running(existing['process'].pid):
                raise UserError('Launch ' + name + ' đã thoát nhưng còn tiến trình con do Web sở hữu. '
                                'Dừng thành phần này trên Web trước khi khởi động lại.')
            if self.operations.get(name):
                raise UserError('Đang xử lý ' + name + '; chờ thao tác hiện tại hoàn tất.')
            self.operations[name] = 'starting'
            self.failures.pop(name, None)
            self._discovery_cache.clear()
        try:
            state = self._state_for_process_checks()
            observed = self._autonomy_status(state)[name]
            if observed['running']:
                if observed['managed']:
                    raise UserError('Còn tiến trình con do Web sở hữu; dừng thành phần trước khi khởi động lại.')
                if observed['complete']:
                    return {'message': observed['message'], 'external': True, 'managed': False}
                raise UserError('Đã có một phần Navigation/mux hoặc Mission bên ngoài Web. '
                                'Dừng stack đó ở Terminal trước khi khởi động stack tích hợp.')
            if name == 'navigation':
                if self.recoverable_costmap_processes() or any(
                        n in state.get('nodes', []) for n in ('/global_costmap/global_costmap', '/local_costmap/local_costmap')):
                    raise UserError('Costmap riêng đang chạy. Dừng Costmap riêng trước; Navigation tự quản lý cả hai costmap.')
                self._navigation_preflight(state)
                package, launch = 'delivery_robot_nav2', 'navigation_integrated_mux.launch.py'
            elif name == 'mission':
                if not self._autonomy_status(state)['navigation']['running']:
                    raise UserError('Khởi động Navigation trước khi mở Mission Manager.')
                package, launch = 'delivery_robot_mission', 'mission_manager.launch.py'
            else:
                if self._autonomy_status(state)['navigation']['running']:
                    raise UserError('Navigation đã quản lý command mux; chờ stack sẵn sàng, không mở mux thứ hai.')
                package, launch = 'delivery_robot_cmd_mux', 'cmd_mux.launch.py'
            ws = Path(self.store.config['ros_workspace']).expanduser().resolve()
            setup = ws / 'install/setup.bash'
            launch_file = ws / 'install' / package / 'share' / package / 'launch' / launch
            if not setup.is_file() or not launch_file.is_file() or launch_file.stat().st_size == 0:
                raise UserError('Chưa có launch đã build: ' + str(launch_file) + '. Build gói cập nhật rồi mở lại Web.')
            if cancel_event is not None and cancel_event.is_set():
                raise UserError('Thao tác khởi động đã bị hủy; không mở tiến trình.')
            if name == 'navigation':
                self._handover_standalone_mux(cancel_event)
            arguments = ' initial_mode:=STOP' if name == 'cmd_mux' else ''
            command = ('set -e\nsource /opt/ros/jazzy/setup.bash\n'
                       f'source {shlex.quote(str(setup))}\n'
                       f'exec ros2 launch {package} {launch}{arguments}')
            kwargs = {'cancel_event': cancel_event} if cancel_event is not None else {}
            return self._spawn_shell(name, command, context={'package': package, 'startup': launch}, **kwargs)
        except Exception as exc:
            with self.lock:
                self.failures[name] = str(exc)
            raise
        finally:
            with self.lock:
                self.operations.pop(name, None)

    def _handover_standalone_mux(self, cancel_event=None):
        if cancel_event is not None and cancel_event.is_set():
            raise UserError('Thao tác khởi động đã bị hủy; giữ nguyên quyền sở hữu tiến trình.')
        observed = self._autonomy_status()['cmd_mux']
        if not observed['running']:
            return
        if not observed['managed']:
            raise UserError('Command mux đang chạy ngoài Web. Dừng mux tại Terminal gốc trước khi bật Navigation tích hợp.')
        navigation = getattr(self.ros, 'navigation', None)
        if navigation is None:
            raise UserError('Chưa có kết nối ROS để xác nhận STOP trước khi chuyển command mux.')
        stopped = navigation.stop_all()
        if not isinstance(stopped, dict) or not stopped.get('mux_confirmed') is True:
            raise UserError('Chưa nhận xác nhận STOP từ command mux; chưa chuyển sang Navigation.')
        if cancel_event is not None and cancel_event.is_set():
            raise UserError('Thao tác khởi động đã bị hủy sau khi yêu cầu STOP.')
        self.stop('cmd_mux', cascade=False, request_stop=False)
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            if cancel_event is not None and cancel_event.is_set():
                raise UserError('Thao tác khởi động đã bị hủy; không mở Navigation.')
            state = self._state_for_process_checks()
            if '/delivery_robot_cmd_mux' not in state.get('nodes', []) and not self._autonomy_processes('cmd_mux'):
                return
            time.sleep(0.15)
        raise UserError('Mux riêng đã được dừng nhưng ROS vẫn còn endpoint cũ. Chờ discovery cập nhật rồi bật Navigation lại.')

    @staticmethod
    def udp_in_use(port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind(('0.0.0.0', port))
            return False
        except OSError:
            return True
        finally:
            sock.close()

    @staticmethod
    def _process_identity(pid):
        """Read only the identity and ROS paths needed to recover this project."""
        directory = Path('/proc') / str(int(pid))
        try:
            if directory.stat().st_uid != os.getuid() or int(pid) == os.getpid():
                return None
            fields = (directory / 'stat').read_text().rsplit(')', 1)[1].split()
            if fields[0] in ('Z', 'X'):
                return None
            argv = [arg.decode(errors='replace') for arg in
                    (directory / 'cmdline').read_bytes().split(b'\0') if arg]
            prefixes = []
            try:
                # Do not retain or expose arbitrary environment values.
                for entry in (directory / 'environ').read_bytes().split(b'\0'):
                    if entry.startswith(b'AMENT_PREFIX_PATH='):
                        prefixes = entry.split(b'=', 1)[1].decode(errors='replace').split(':')
                        break
            except OSError:
                pass
            return {'pid': int(pid), 'argv': argv, 'cmdline': ' '.join(argv),
                    'ppid': int(fields[1]), 'pgid': int(fields[2]), 'session': int(fields[3]),
                    'start_ticks': int(fields[19]), 'cwd': str(directory.joinpath('cwd').resolve()),
                    'ament_prefixes': prefixes}
        except (OSError, ValueError, IndexError):
            return None

    @staticmethod
    def local_process_matches(needle):
        matches = []
        for p in Path('/proc').glob('[0-9]*'):
            try:
                pid = int(p.name)
                if p.stat().st_uid != os.getuid() or pid == os.getpid():
                    continue
                args = (p / 'cmdline').read_bytes().replace(b'\x00', b' ').decode(errors='replace').strip()
                if needle in args:
                    identity = ProcessManager._process_identity(pid)
                    if identity is not None:
                        matches.append(identity)
            except (OSError, ValueError):
                pass
        return matches

    @classmethod
    def local_process_contains(cls, needle):
        return bool(cls.local_process_matches(needle))

    @classmethod
    def recoverable_localization_processes(cls):
        """Return only localization processes that are unmistakably ours.

        V2.5.1 could recover the ros2 launch parent, but a launch parent may exit
        while nav2_amcl survives and is re-parented to systemd --user.  PIDs are
        intentionally not persisted or hard-coded.  Recovery therefore matches
        either the exact project launch command or the exact AMCL executable with
        this project's installed amcl.yaml.
        """
        found = {}
        needles = (
            'ros2 launch delivery_robot_localization_v14 amcl_localization.launch.py',
            '/opt/ros/jazzy/lib/nav2_amcl/amcl',
        )
        for needle in needles:
            for item in cls.local_process_matches(needle):
                cmd = item['cmdline']
                if needle.endswith('/amcl') and not (
                    'delivery_robot_localization_v14' in cmd and '/config/amcl.yaml' in cmd
                ):
                    continue
                found[item['pid']] = item
        return list(found.values())

    def _project_identity_matches(self, name, item):
        """An explicit stop may recover this workspace, never a generic ROS name."""
        c = self.store.config
        workspace = Path(c['ros_workspace']).expanduser().resolve()
        motor = Path(c['motor_project']).expanduser().resolve()
        lidar = Path(c['lidar_script']).expanduser().resolve()
        try:
            args = item.get('argv') or shlex.split(item.get('cmdline', ''))
        except ValueError:
            return False
        command = item.get('cmdline', '')
        def in_path(value, root):
            try:
                return Path(value).is_absolute() and Path(value).resolve().is_relative_to(root)
            except (OSError, ValueError):
                return False
        from_workspace = any(in_path(p, workspace) for p in item.get('ament_prefixes', []))
        from_workspace |= any(in_path(arg, workspace) for arg in args)
        def launch(package, filenames):
            return from_workspace and any(
                args[i:i+2] == ['launch', package] and i+2 < len(args) and args[i+2] in filenames
                for i in range(len(args)))
        def project_param(package):
            return any(i+1 < len(args) and arg == '--params-file' and
                       in_path(args[i+1], workspace) and package in Path(args[i+1]).parts
                       for i, arg in enumerate(args))
        if name == 'odom_bridge':
            script = motor / 'host_ros2/odom_tf_joint_bridge.py'
            exact = str(script) in args or (
                item.get('cwd') == str(motor) and 'host_ros2/odom_tf_joint_bridge.py' in args)
            return exact and all(value in args for value in (
                'publish_tf:=false', 'left_joint:=wheel_joint_l', 'right_joint:=wheel_joint_r'))
        if name == 'ekf_odom':
            return launch('delivery_robot_localization_v14', {'official_odometry.launch.py'}) or (
                any(arg.endswith('/robot_localization/ekf_node') for arg in args) and
                project_param('delivery_robot_sensor_fusion_v14'))
        if name == 'robot_model':
            try:
                urdf = Path(c['description_urdf']).expanduser().resolve().read_text().rstrip('\n')
            except OSError:
                return False
            return bool(urdf) and any(arg.endswith('/robot_state_publisher/robot_state_publisher') for arg in args) and (
                'robot_description:=' + urdf in args)
        if name == 'lidar':
            launch_path = lidar.parent / 'ld14p_wifi.launch.py'
            return str(lidar) in args or str(launch_path) in args or (
                item.get('cwd') == str(lidar.parent.parent) and
                any(arg in ('./ubuntu/' + lidar.name, 'ubuntu/' + lidar.name) for arg in args))
        if name == 'slam':
            params = str((self.store.root / 'runtime/slam_web.yaml').resolve())
            return params in args or ('slam_params_file:=' + params) in args
        if name == 'rviz':
            config = str((Path(__file__).parent / 'rviz/localization.rviz').resolve())
            return config in args and any(Path(arg).name == 'rviz2' for arg in args)
        if name == 'localization':
            return launch('delivery_robot_localization_v14', {'amcl_localization.launch.py'}) or (
                any(arg.endswith('/nav2_amcl/amcl') for arg in args) and
                project_param('delivery_robot_localization_v14'))
        if name == 'costmap':
            return launch('delivery_robot_nav2', {'costmaps_only.launch.py'}) or (
                any(arg.endswith('/nav2_costmap_2d/nav2_costmap_2d') for arg in args) and
                project_param('delivery_robot_nav2'))
        if name == 'navigation':
            launches = {'navigation_integrated_mux.launch.py', 'navigation_mux.launch.py',
                'navigation_safe.launch.py', 'navigation_real_direct.launch.py',
                'controller_safe.launch.py', 'controller_real_direct.launch.py', 'planner_only.launch.py'}
            return launch('delivery_robot_nav2', launches) or (
                any('/nav2_' in arg and '/lib/' in arg for arg in args) and
                project_param('delivery_robot_nav2'))
        if name in ('mission', 'cmd_mux'):
            package = 'delivery_robot_mission' if name == 'mission' else 'delivery_robot_cmd_mux'
            executable = 'mission_manager' if name == 'mission' else 'cmd_mux'
            filename = executable + '.launch.py'
            return launch(package, {filename}) or any(
                in_path(arg, workspace) and Path(arg).name == executable and package in Path(arg).parts
                for arg in args)
        return False

    def recoverable_project_processes(self, name):
        if name in ('navigation', 'mission', 'cmd_mux'):
            candidates = self._autonomy_processes(name)
        elif name == 'localization':
            candidates = self.recoverable_localization_processes()
        elif name == 'costmap':
            candidates = self.recoverable_costmap_processes()
        else:
            candidates = self.recoverable_base_processes(name)
        return [item for item in candidates if self._project_identity_matches(name, item)]

    @classmethod
    def recoverable_costmap_processes(cls):
        """Return only the delivery_robot_nav2 costmap launch/children."""
        found = {}
        for needle in (
            'ros2 launch delivery_robot_nav2 costmaps_only.launch.py',
            '/opt/ros/jazzy/lib/nav2_costmap_2d/nav2_costmap_2d',
        ):
            for item in cls.local_process_matches(needle):
                cmd = item['cmdline']
                if 'nav2_costmap_2d/nav2_costmap_2d' in cmd and not (
                    'delivery_robot_nav2' in cmd and 'costmaps_v1.yaml' in cmd
                ):
                    continue
                found[item['pid']] = item
        return list(found.values())

    def recoverable_base_processes(self, name):
        """Return only unmistakable processes from this delivery-robot stack.

        These matchers are intentionally strict. They are used only to recover
        stale processes left by a previous Web session so the next Connect can
        start a fresh, fully managed stack with fresh logs. Unknown/manual ROS
        processes are never killed automatically.
        """
        c = self.store.config
        found = {}

        def add_matches(needle, accept):
            for item in self.local_process_matches(needle):
                if accept(item['cmdline']):
                    found[item['pid']] = item

        if name == 'odom_bridge':
            add_matches('odom_tf_joint_bridge.py', lambda cmd:
                'publish_tf:=false' in cmd and
                'left_joint:=wheel_joint_l' in cmd and
                'right_joint:=wheel_joint_r' in cmd)

        elif name == 'ekf_odom':
            add_matches('official_odometry.launch.py', lambda cmd:
                'delivery_robot_localization_v14' in cmd and 'start_bridge:=false' in cmd)
            # Catch the child EKF even if ros2 launch has already exited and
            # systemd --user adopted the orphan.  Match by the exact node name
            # or this project's V14 params, not by a generic EKF executable.
            add_matches('/robot_localization/ekf_node', lambda cmd:
                ('ekf_filter_node_v14' in cmd or
                 'delivery_robot_sensor_fusion_v14' in cmd or
                 'ekf_compare.yaml' in cmd))
            add_matches('ekf_node', lambda cmd:
                ('ekf_filter_node_v14' in cmd or
                 'delivery_robot_sensor_fusion_v14' in cmd or
                 'ekf_compare.yaml' in cmd))
            add_matches('ekf_filter_node_v14', lambda cmd:
                ('ekf_node' in cmd or 'robot_localization' in cmd))

        elif name == 'robot_model':
            add_matches('/robot_state_publisher/robot_state_publisher', lambda cmd:
                ('delivery_car_robot' in cmd or 'delivery_car_URDF_MAIN' in cmd or
                 ('wheel_joint_l' in cmd and 'wheel_joint_r' in cmd and 'base_link' in cmd)))

        elif name == 'lidar':
            script = Path(c['lidar_script']).expanduser().resolve()
            project = script.parent.parent
            project_s = str(project)
            add_matches(script.name, lambda cmd: project_s in cmd or 'LD14P_WIFI_BRIDGE_V1' in cmd)
            add_matches('ld14p_wifi.launch.py', lambda cmd: project_s in cmd or 'LD14P_WIFI_BRIDGE_V1' in cmd)
            add_matches('/ldlidar_publisher_ld14', lambda cmd:
                ('/tmp/ld14p_wifi' in cmd or 'LDLiDAR_LD14P' in cmd))
            add_matches('ldlidar', lambda cmd:
                ('/tmp/ld14p_wifi' in cmd and ('LD14P' in cmd or 'ldlidar' in cmd.lower())))
            add_matches('socat', lambda cmd:
                '/tmp/ld14p_wifi' in cmd and str(c.get('lidar_port', 7777)) in cmd)

        elif name == 'slam':
            add_matches('online_async_launch.py', lambda cmd:
                'slam_toolbox' in cmd and 'slam_web.yaml' in cmd)
            add_matches('async_slam_toolbox_node', lambda cmd:
                'slam_web.yaml' in cmd or 'slam_toolbox' in cmd)

        elif name == 'rviz':
            add_matches('rviz2', lambda cmd:
                'delivery_robot_web' in cmd and 'localization.rviz' in cmd)

        return list(found.values())

    def cleanup_stale_project_stack(self):
        """Compatibility hook: Web startup only observes external processes.

        A package/command match is not proof that this Web instance owns a PID.
        No process is stopped on startup. Explicit Stop can recover same-user
        configured-project processes after revalidating PID and start time.
        """
        return []

    def _wait_base_graph_clear(self, timeout=5.0):
        """Wait for DDS graph to forget exact base-stack nodes after cleanup.

        Killing an orphaned launch child is immediate at the OS level, while
        ROS 2 discovery may still report the node for a short period.  Connect
        should not mislabel that stale discovery entry as a new external EKF.
        """
        watched = {
            '/delivery_odom_tf_joint_bridge',
            '/ekf_filter_node_v14',
            '/robot_state_publisher',
            '/ldlidar_publisher_ld14',
        }
        deadline = time.monotonic() + timeout
        last = []
        while time.monotonic() < deadline:
            state = self.ros.status()
            last = [n for n in state.get('nodes', []) if n in watched]
            if not last and not state.get('scan_publishers', 0):
                return True
            time.sleep(0.25)
        return False

    def prepare_base_stack_for_connect(self, reuse_owned=False):
        """Validate ownership without killing a running robot from another session."""
        if self.offline:
            raise UserError('OFFLINE: không khởi động phần cứng/ROS từ giao diện thử.')
        if not (reuse_owned and self._owned_live('navigation')):
            self._require_navigation_stopped('kết nối lại hệ thống nền')
        with self.lock:
            owned = [name for name in ('cmd_mux', 'odom_bridge', 'ekf_odom', 'robot_model', 'lidar')
                     if name in self.children and self.children[name]['process'].poll() is None]
        if owned and not reuse_owned:
            raise UserError('Hệ thống đã được phiên Web này khởi động: ' + ', '.join(owned) +
                            '. Dùng Dừng hệ thống trước khi Kết nối lại.')
        external = [name for name in ('odom_bridge', 'ekf_odom', 'robot_model', 'lidar')
                    if not (reuse_owned and self._owned_live(name)) and self.recoverable_base_processes(name)]
        if external:
            raise UserError('Thành phần đang chạy ngoài phiên Web: ' + ', '.join(external) +
                            '. Bấm Dừng hệ thống để đóng đúng tiến trình dự án trước khi kết nối lại.')
        return []

    def preflight_base_stack(self, reuse_owned=False):
        """Validate the whole base stack before starting the first component.

        This prevents the old partial-start failure mode where Agent/Odom/EKF were
        already started and the sequence only failed later at Robot Model or LiDAR.
        """
        if self.offline:
            raise UserError('OFFLINE: không khởi động phần cứng/ROS từ giao diện thử.')
        s, c = self._state_for_process_checks(), self.store.config
        nodes = s.get('nodes', [])
        if reuse_owned:
            for name, component_nodes in COMPONENT_NODES.items():
                if self._owned_live(name):
                    nodes = [node for node in nodes if node not in component_nodes]
            if self._owned_live('lidar'):
                s['scan_publishers'] = 0

        # Observe external processes; never assume a matching command is owned.
        stale = {}
        for name in ('odom_bridge', 'ekf_odom', 'robot_model', 'lidar'):
            if reuse_owned and self._owned_live(name):
                continue
            got = self.recoverable_base_processes(name)
            if got:
                stale[name] = [x['pid'] for x in got]
        if stale:
            raise UserError('Còn tiến trình cũ của dự án: ' + str(stale) + '. Dừng tại Terminal/phiên đã mở chúng trước khi Kết nối lại.')

        agent = self.agent.snapshot(fresh=True)
        if self.udp_in_use(c['agent_port']) and not agent.get('running'):
            raise UserError('UDP 8888 đang do Agent/lệnh ngoài Web sử dụng. Dừng Terminal Agent cũ trước.')
        if '/delivery_odom_tf_joint_bridge' in nodes:
            raise UserError('Đã có Odom Bridge ngoài Web. Dừng tiến trình ngoài trước khi Kết nối hệ thống.')
        if '/ekf_filter_node_v14' in nodes:
            raise UserError('Đã có EKF V14 ngoài Web. Dừng tiến trình ngoài trước khi Kết nối hệ thống.')
        if '/robot_state_publisher' in nodes:
            raise UserError('Đã có Robot State Publisher ngoài Web. Dừng tiến trình ngoài trước khi Kết nối hệ thống.')
        if s.get('scan_publishers', 0) or '/ldlidar_publisher_ld14' in nodes:
            raise UserError('Đã có LiDAR /scan ngoài Web. Dừng tiến trình ngoài trước khi Kết nối hệ thống.')
        if '/amcl' in nodes or '/map_server' in nodes or '/slam_toolbox' in nodes:
            raise UserError('Localization/SLAM đang chạy trước khi Kết nối hệ thống. Hãy dừng phiên cũ trước.')
        if '/global_costmap/global_costmap' in nodes or '/local_costmap/local_costmap' in nodes:
            raise UserError('Costmap đang chạy trước khi Kết nối hệ thống. Hãy dừng phiên cũ trước.')

        motor = Path(c['motor_project']).expanduser().resolve()
        if not (motor / 'host_ros2' / 'odom_tf_joint_bridge.py').is_file():
            raise UserError('Thiếu Odom Bridge: ' + str(motor / 'host_ros2' / 'odom_tf_joint_bridge.py'))
        ros_ws = Path(c['ros_workspace']).expanduser().resolve()
        if not (ros_ws / 'install' / 'setup.bash').is_file():
            raise UserError('Thiếu ROS overlay: ' + str(ros_ws / 'install' / 'setup.bash'))
        urdf = Path(c['description_urdf']).expanduser().resolve()
        if not urdf.is_file():
            raise UserError('Thiếu URDF: ' + str(urdf))
        lidar = Path(c['lidar_script']).expanduser().resolve()
        if not lidar.is_file():
            raise UserError('Thiếu script LiDAR: ' + str(lidar))
        return True

    @staticmethod
    def _pids_alive(pids):
        alive = []
        for pid in pids:
            try:
                os.kill(pid, 0)
                alive.append(pid)
            except ProcessLookupError:
                pass
            except PermissionError:
                alive.append(pid)
        return alive

    def _stop_exact_processes(self, matches, label):
        """Explicit project recovery, pinned to uid/PID/start time.

        Never signal an arbitrary terminal process group. Isolated ROS launch
        sessions can include verified companion nodes (map server/lifecycle).
        Each PID is revalidated before every signal, including escalation.
        """
        records = {}
        groups = set()
        for match in matches:
            current = self._process_identity(match['pid'])
            if current is None:
                continue
            if ('start_ticks' in match and match['start_ticks'] != current['start_ticks']) or not self._project_identity_matches(label, current):
                raise UserError('Tiến trình đã đổi danh tính; chưa gửi tín hiệu: ' + str(match['pid']))
            records[current['pid']] = current
            if current['pgid'] == current['session'] and current['pgid'] != os.getpgrp():
                groups.add(current['pgid'])
        if not records:
            return
        allowed = {
            'navigation': {'controller_server', 'planner_server', 'behavior_server', 'bt_navigator',
                'waypoint_follower', 'velocity_smoother', 'collision_monitor', 'lifecycle_manager', 'cmd_mux'},
            'mission': {'mission_manager'}, 'cmd_mux': {'cmd_mux'},
            'localization': {'amcl', 'map_server', 'lifecycle_manager'},
            'costmap': {'nav2_costmap_2d', 'lifecycle_manager'},
            'ekf_odom': {'ekf_node', 'static_transform_publisher'},
            'robot_model': {'robot_state_publisher'}, 'odom_bridge': {'odom_tf_joint_bridge.py'},
            'slam': {'async_slam_toolbox_node', 'lifecycle_manager'},
            'lidar': {'start_ld14p_wifi.sh', 'ld14p_wifi.launch.py', 'ldlidar_publisher_ld14', 'socat'},
            'rviz': {'rviz2'},
        }.get(label, set())
        # Capture isolated session siblings only if their executable is a
        # component of this exact ROS launch. Do not kill an unrelated process
        # merely because it shares a login/session.
        group_members = {group: [] for group in groups}
        for directory in Path('/proc').glob('[0-9]*'):
            record = self._process_identity(directory.name)
            if record is None or record['pgid'] not in groups or record['session'] != record['pgid']:
                continue
            group_members[record['pgid']].append(record)
        for group, members in group_members.items():
            permitted = all(record['pid'] in records or self._project_identity_matches(label, record) or
                            (record['argv'] and Path(record['argv'][0]).name in allowed)
                            for record in members)
            if permitted:
                records.update((record['pid'], record) for record in members)
            else:
                groups.discard(group)

        # For normal terminal launches, include only observed descendants of an
        # exact matched parent; never the terminal shell or its other jobs.
        descendants = {}
        for directory in Path('/proc').glob('[0-9]*'):
            record = self._process_identity(directory.name)
            if record is not None:
                descendants[record['pid']] = record
        changed = True
        while changed:
            changed = False
            for pid, record in descendants.items():
                if pid not in records and record['ppid'] in records:
                    records[pid] = record
                    changed = True

        def remaining():
            alive = []
            for record in records.values():
                now = self._process_identity(record['pid'])
                if now is not None and now['start_ticks'] == record['start_ticks']:
                    alive.append(record)
            return alive

        if not records:
            raise UserError(f'Không tìm thấy tiến trình {label} đúng của dự án để dừng.')
        for sig, seconds in ((signal.SIGINT, 4.0), (signal.SIGTERM, 2.0), (signal.SIGKILL, 1.0)):
            for record in remaining():
                try:
                    os.kill(record['pid'], sig)
                except ProcessLookupError:
                    pass
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                if not remaining():
                    return
                time.sleep(0.1)
        alive = [record['pid'] for record in remaining()]
        if alive:
            raise UserError(f'{label} vẫn chưa dừng: PID {alive}')

    def _ros_cli(self, command, timeout=6):
        c = self.store.config
        setup = Path(c['ros_workspace']).expanduser().resolve() / 'install/setup.bash'
        shell = (
            'source /opt/ros/jazzy/setup.bash\n'
            f'source {shlex.quote(str(setup))}\n'
            f'{command}'
        )
        return subprocess.run(
            ['/bin/bash', '-lc', shell],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, timeout=timeout, check=False,
        )

    def _ensure_costmaps_active(self, cancel_event=None):
        nodes = ('/global_costmap/global_costmap', '/local_costmap/local_costmap')
        deadline = time.monotonic() + 18.0
        last = {}

        # costmaps_only.launch.py already starts lifecycle_manager_costmaps.
        # Wait briefly for its normal autostart path before issuing any manual
        # transition, avoiding configure/activate races.
        grace = time.monotonic() + 7.0
        while time.monotonic() < grace:
            if self.closing or (cancel_event is not None and cancel_event.is_set()):
                raise UserError('Đã hủy khởi động Costmap.')
            active = []
            for node in nodes:
                get = self._ros_cli(f'ros2 lifecycle get {shlex.quote(node)}', timeout=3)
                text = (get.stdout or '').strip()
                last[node] = text
                active.append('active [3]' in text)
            if all(active):
                return last
            time.sleep(0.4)

        while time.monotonic() < deadline:
            if self.closing or (cancel_event is not None and cancel_event.is_set()):
                raise UserError('Đã hủy khởi động Costmap.')
            all_active = True
            for node in nodes:
                get = self._ros_cli(f'ros2 lifecycle get {shlex.quote(node)}', timeout=3)
                text = (get.stdout or '').strip()
                last[node] = text
                if 'active [3]' in text:
                    continue
                all_active = False
                if 'unconfigured [1]' in text:
                    self._ros_cli(f'ros2 lifecycle set {shlex.quote(node)} configure', timeout=4)
                # Re-query after configure / lifecycle-manager race.
                get2 = self._ros_cli(f'ros2 lifecycle get {shlex.quote(node)}', timeout=3)
                text2 = (get2.stdout or '').strip()
                last[node] = text2
                if 'inactive [2]' in text2:
                    self._ros_cli(f'ros2 lifecycle set {shlex.quote(node)} activate', timeout=4)
            if all_active:
                return last
            # One final verification may now show active after our transitions.
            verify = []
            for node in nodes:
                out = self._ros_cli(f'ros2 lifecycle get {shlex.quote(node)}', timeout=3).stdout or ''
                last[node] = out.strip()
                verify.append('active [3]' in out)
            if all(verify):
                return last
            time.sleep(0.5)
        raise UserError('Costmap đã chạy nhưng lifecycle chưa ACTIVE. Trạng thái: ' + str(last))

    def status(self):
        with self.lock:
            result = {}
            for name, item in self.children.items():
                code = item['process'].poll()
                orphaned = code is not None and self._owned_group_running(item['process'].pid)
                running = code is None or orphaned
                status = self.operations.get(name) or ('failed' if orphaned or name in self.failures
                          else 'starting' if running else 'stopped')
                result[name] = {
                    'running': running, 'pid': item['process'].pid, 'returncode': code,
                    'log': str(item.get('log', '')), 'managed': running, 'external': False,
                    'orphaned': orphaned, 'status': status, 'ready': False,
                    'message': ('Launch đã thoát; còn tiến trình con cần dừng.' if orphaned else
                                self.failures.get(name) or ('Đang chạy.' if running else 'Đã dừng.')),
                    'context': item.get('context', {}),
                }
        if self.offline:
            result.update(self._autonomy_status({'nodes': []}))
            return result

        state = self._state_for_process_checks()
        service = self.agent.snapshot()
        busy = self.udp_in_use(self.store.config['agent_port'])
        external_agent = bool((busy or service['running']) and not self.agent_started)
        result['agent'] = {
            **service,
            'running': bool(service.get('running') or busy),
            'managed': self.agent_started and service.get('running', False),
            'external': external_agent,
            'recoverable': bool(service.get('installed') and service.get('running')),
            'status': self.operations.get('agent') or ('unknown' if service.get('installed') and not service.get('status_known', True)
                      else 'running' if service.get('running') else 'external' if busy else 'stopped'),
            'returncode': None,
            'ready': state.get('odom_publishers') == 1 and
                     state.get('odom', {}).get('receive_age') is not None and
                     state['odom']['receive_age'] < 1.0,
            'message': (
                'UDP 8888 đang do Agent ngoài web sử dụng. Dừng lệnh Docker thủ công trước khi chuyển quyền cho web.'
                if external_agent else service.get('message') or
                ('Agent Docker đã chạy; chờ /odom.' if service['running'] else 'Agent Docker đã dừng.')
            ),
        }

        nodes = state.get('nodes', [])
        for name, node in [
            ('odom_bridge', '/delivery_odom_tf_joint_bridge'),
            ('ekf_odom', '/ekf_filter_node_v14'),
            ('robot_model', '/robot_state_publisher'),
            ('lidar', '/ldlidar_publisher_ld14'),
            ('slam', '/slam_toolbox'),
            ('localization', '/amcl'),
            ('rviz', '/rviz2'),
        ]:
            item = result.get(name)
            if not item or not item['running']:
                if name == 'localization':
                    recoverable = self.recoverable_localization_processes()
                elif name in ('odom_bridge', 'ekf_odom', 'robot_model', 'lidar', 'slam', 'rviz'):
                    recoverable = self.recoverable_base_processes(name)
                else:
                    recoverable = []
                if node not in nodes and not recoverable:
                    continue
                can_recover = bool(self.recoverable_project_processes(name))
                result[name] = {
                    'running': True, 'external': True, 'managed': False,
                    'recoverable': can_recover,
                    'status': 'external',
                    'pid': recoverable[0]['pid'] if recoverable else None,
                    'returncode': None,
                    'message': (
                        'Tiến trình dự án đã được xác minh; có thể dừng từ Web.'
                        if can_recover else
                        'Đang chạy ngoài Web và không nhận diện là tiến trình cũ của dự án. Dừng ở Terminal gốc trước.'
                    ),
                }

        # Costmaps are a pair of lifecycle nodes managed as one Web component.
        # Web V5 recovery is process-first: if the exact delivery_robot_nav2
        # costmap processes survived a previous Web/Terminal session, the Stop
        # button remains available even before DDS node discovery catches up.
        global_costmap = '/global_costmap/global_costmap' in nodes
        local_costmap = '/local_costmap/local_costmap' in nodes
        global_pub = (state.get('global_costmap_publishers') or 0) > 0
        local_pub = (state.get('local_costmap_publishers') or 0) > 0
        cost_item = result.get('costmap')
        exact_costmap = self.recoverable_costmap_processes()
        if (not cost_item or not cost_item.get('running')) and (exact_costmap or global_costmap or local_costmap):
            result['costmap'] = {
                'running': True, 'external': True, 'managed': False,
                'recoverable': bool(self.recoverable_project_processes('costmap')),
                'status': 'external',
                'pid': exact_costmap[0]['pid'] if exact_costmap else None,
                'returncode': None,
                'ready': global_pub and local_pub,
                'message': (
                    'Costmap đang chạy ngoài phiên Web hiện tại; dừng tại Terminal/phiên đã mở.'
                    if exact_costmap else
                    'Có Costmap ngoài Web không nhận diện được. Dừng stack ngoài trước khi chuyển quyền cho Web.'
                ),
                'context': {'global_active': global_pub, 'local_active': local_pub},
            }
        elif cost_item and cost_item.get('running') and not cost_item.get('external'):
            cost_item['ready'] = global_pub and local_pub
            cost_item.setdefault('context', {})
            cost_item['context'].update(global_active=global_pub, local_active=local_pub)
            cost_item['message'] = (
                'Global + Local Costmap ACTIVE và đang publish.' if cost_item['ready']
                else 'Costmap process đang chạy; Web đang chờ lifecycle ACTIVE/topic costmap.'
            )

        for name in ('odom_bridge', 'ekf_odom', 'robot_model', 'lidar'):
            item = result.get(name)
            if not item or not item.get('running') or item.get('external'):
                continue
            if name == 'lidar':
                age = state.get('scan', {}).get('receive_age')
                ready = state.get('scan_publishers') == 1 and age is not None and age < 1.0
                wait = 'LiDAR bridge chạy; chưa nhận /scan mới. Xem IP và log.'
            elif name == 'odom_bridge':
                ready = '/delivery_odom_tf_joint_bridge' in nodes and state.get('odom_publishers') == 1
                wait = 'Odom Bridge no-TF chạy; đang chờ raw /odom từ ESP32 #1.'
            elif name == 'ekf_odom':
                tf = state.get('tf_odom') or {}
                ready = self._fresh_age(tf.get('age'), minimum=-0.10, maximum=0.8)
                wait = 'EKF V14 chạy; đang chờ TF odom→base_link filtered.'
            else:
                ready = '/robot_state_publisher' in nodes
                wait = 'Đang chờ node robot_state_publisher; xem log nếu chưa xuất hiện.'
            item['ready'] = ready and not item.get('orphaned') and name not in self.failures
            item['status'] = self.operations.get(name) or ('failed' if item.get('orphaned') or name in self.failures else 'ready' if item['ready'] else 'starting')
            item['message'] = self.failures.get(name) or ('Web quản lý · đã thấy dữ liệu/node tương ứng.' if ready else wait)
        autonomy = self._autonomy_status(state)
        result.update(autonomy)
        if autonomy['navigation']['running'] and result.get('costmap', {}).get('running'):
            result['costmap'].update(managed=False, external=True, recoverable=False,
                message='Costmap do Navigation quản lý; dùng thao tác Navigation để khởi động/dừng.')
            result['costmap'].setdefault('context', {})['provided_by'] = 'navigation'
        return result

    def _request_motion_stop(self):
        self.ros.motion.stop('Đang dừng hệ thống.', force=True)
        gateway = getattr(self.ros, 'navigation', None)
        if gateway is None:
            return {'mux_confirmed': False, 'message': 'Chưa có kết nối command mux.'}
        try:
            result = gateway.stop_all()
            return result if isinstance(result, dict) else {'mux_confirmed': False, 'message': 'Phản hồi STOP không hợp lệ.'}
        except Exception as exc:
            return {'mux_confirmed': False, 'message': str(exc)}

    def _stop_plan(self, names, progress=None, *, recover=True, request_stop=True, strict_dependencies=False):
        results = []
        with self.stop_lock:
            acknowledgement = self._request_motion_stop() if request_stop and names != ('rviz',) else None
            for name in names:
                self._emit_progress(progress, {'component': name, 'state': 'running', 'message': 'Đang dừng.'})
                try:
                    owned = self._owned_live(name)
                    matches = self.recoverable_project_processes(name) if recover and not owned and name != 'agent' else []
                    service = self.agent.snapshot(fresh=True) if name == 'agent' else {}
                    helper_owned = (name == 'agent' and recover and service.get('installed') and
                                    (service.get('running') or not service.get('status_known', True)))
                    known_dead = name in self.children and not owned
                    if owned or matches or helper_owned or known_dead:
                        result = self._stop_one(name, matches=matches, recovered=not owned and bool(matches or helper_owned))
                        row = {'component': name, 'ok': True, 'state': 'succeeded', **result}
                    else:
                        info = self.status().get(name, {})
                        if info.get('running') or info.get('external'):
                            row = {'component': name, 'ok': False, 'state': 'external', 'external': True,
                                   'message': 'Còn tiến trình ngoài phạm vi dự án được xác minh; chưa dừng thành phần này.'}
                        else:
                            row = {'component': name, 'ok': True, 'state': 'succeeded', 'already_stopped': True,
                                   'message': 'Đã dừng.'}
                    if acknowledgement is not None and acknowledgement.get('mux_confirmed') is not True:
                        row['warning'] = 'Chưa xác nhận STOP từ mux; việc đóng tiến trình vẫn được thực hiện.'
                        row['stop_result'] = acknowledgement
                except Exception as exc:
                    row = {'component': name, 'ok': False, 'state': 'failed', 'message': str(exc)}
                    try:
                        self.store.audit('process_stop_failed', name + ': ' + str(exc))
                    except Exception:
                        pass
                results.append(row)
                self._emit_progress(progress, row)
                if strict_dependencies and not row['ok']:
                    break
        return results

    def stop_all(self, progress=None, *, recover=True, request_stop=True):
        """Explicit operator stop: attempt every project component in reverse order.

        Actual child identity drives shutdown, not ROS discovery. A dead executor,
        a missing mux ACK or one failed child never skips the remaining cleanup.
        Exact configured-project recovery is allowed only on this explicit action.
        """
        if self.offline:
            raise UserError('OFFLINE: không dừng tiến trình thật.')
        return self._stop_plan(STOP_ORDER, progress, recover=recover, request_stop=request_stop)

    def _spawn_shell(self, name: str, command: str, cwd: str | None = None,
                     context: dict | None = None, cancel_event=None):
        # Popen + registration is atomic with close(): no delayed background
        # startup can create a robot process after shutdown has enumerated owners.
        with self.lock:
            if self.closing:
                raise UserError('Web đang đóng; không khởi động thêm tiến trình.')
            if cancel_event is not None and cancel_event.is_set():
                raise UserError('Thao tác khởi động đã bị hủy; không mở tiến trình.')
            if getattr(self.ros, 'maintenance', False):
                raise UserError('Đang cấu hình USB; không mở tiến trình robot.')
            restart_required = name in self.confirmed_stops
            if name in self.children:
                if self.children[name]['process'].poll() is None:
                    raise UserError('Tiến trình đã chạy; không mở bản trùng.')
                try:
                    self.children[name]['handle'].close()
                except Exception:
                    pass
            log_path = self.store.root / 'logs' / f'{name}_{time.time_ns()}.log'
            log = open(log_path, 'ab', buffering=0)
            try:
                process = subprocess.Popen(
                    ['/bin/bash', '-lc', command],
                    cwd=cwd or str(Path.home()),
                    stdout=log, stderr=subprocess.STDOUT,
                    stdin=subprocess.DEVNULL,
                    start_new_session=True, env=dict(os.environ),
                )
            except Exception:
                log.close()
                raise
            self.children[name] = {'process': process, 'log': log_path, 'handle': log, 'command': command,
                                   'context': dict(context or {}), 'started_at': time.monotonic()}
            self.failures.pop(name, None)
            self._discovery_cache.clear()
        time.sleep(0.8)
        if process.poll() is not None:
            log.close()
            error = log_path.read_text(errors='replace')[-2800:]
            raise UserError(f'{name} thoát với mã {process.returncode}. Log: ' + error)
        self._new_owner_started(name, restart_required)
        self.store.audit('process_start', name)
        return {'message': f'Đã mở tiến trình {name}; đang chờ xác nhận trạng thái ROS.'}

    def start(self, name, cancel_event=None):
        if self.closing:
            raise UserError('Web đang đóng; không khởi động thêm tiến trình.')
        if cancel_event is not None and cancel_event.is_set():
            raise UserError('Thao tác khởi động đã bị hủy; không mở tiến trình.')
        if self.offline:
            raise UserError('OFFLINE: không khởi động phần cứng/ROS từ giao diện thử.')
        if name not in ('agent', 'odom_bridge', 'ekf_odom', 'robot_model', 'lidar', 'slam', 'rviz', 'costmap', 'navigation', 'mission', 'cmd_mux'):
            raise UserError('Thành phần không nằm trong danh sách được phép.')
        if getattr(self.ros, 'maintenance', False):
            raise UserError('Đang cấu hình USB; không thay đổi tiến trình robot.')
        if name in ('navigation', 'mission', 'cmd_mux'):
            return self._start_autonomy(name, cancel_event)
        if name in ('slam', 'costmap'):
            self._require_navigation_stopped('mở ' + name)
        spawn_kwargs = {'cancel_event': cancel_event} if cancel_event is not None else {}

        with self.lock:
            if name in self.children and self.children[name]['process'].poll() is None:
                return {'message': 'Đã chạy bởi ứng dụng; không mở bản trùng.'}
            if name in self.children and self._owned_group_running(self.children[name]['process'].pid):
                raise UserError('Còn tiến trình con của ' + name + '; dừng thành phần trước khi mở lại.')
            s, c = self._state_for_process_checks(), self.store.config

            if name == 'agent':
                before = self.agent.snapshot(fresh=True)
                if not before['running'] and self.udp_in_use(c['agent_port']):
                    raise UserError('UDP 8888 đang bị Agent/lệnh Docker ngoài web chiếm. Dừng Terminal Agent cũ trước.')
                message = self.agent.start()
                if not before['running']:
                    self.agent_started = True
                    self._new_owner_started('agent', 'agent' in self.confirmed_stops)
                self.store.audit('process_start', 'agent docker exact-command helper')
                return {'message': message}

            if name == 'odom_bridge':
                if '/delivery_odom_tf_joint_bridge' in s['nodes'] or self.local_process_contains('odom_tf_joint_bridge.py'):
                    raise UserError('Odometry Bridge đang chạy. Không khởi động bản thứ hai.')
                project = Path(c['motor_project']).expanduser().resolve()
                script = project / 'host_ros2' / 'odom_tf_joint_bridge.py'
                if not script.is_file():
                    raise UserError(f'Không tìm thấy {script}')
                command = (
                    f"cd {shlex.quote(str(project))}\n"
                    "source /opt/ros/jazzy/setup.bash\n"
                    "exec /usr/bin/python3 host_ros2/odom_tf_joint_bridge.py "
                    "--ros-args -p left_joint:=wheel_joint_l -p right_joint:=wheel_joint_r -p publish_tf:=false"
                )
                return self._spawn_shell(name, command, str(project), **spawn_kwargs)

            if name == 'ekf_odom':
                if '/ekf_filter_node_v14' in s['nodes'] or self.local_process_contains('official_odometry.launch.py'):
                    raise UserError('EKF V14 official đang chạy. Không khởi động bản thứ hai.')
                ros_ws = Path(c['ros_workspace']).expanduser().resolve()
                setup = ros_ws / 'install' / 'setup.bash'
                if not setup.is_file():
                    raise UserError(f'Không có overlay ROS 2: {setup}')
                command = (
                    "source /opt/ros/jazzy/setup.bash\n"
                    f"source {shlex.quote(str(setup))}\n"
                    "exec ros2 launch delivery_robot_localization_v14 official_odometry.launch.py start_bridge:=false"
                )
                return self._spawn_shell(name, command, **spawn_kwargs)

            if name == 'robot_model':
                if '/robot_state_publisher' in s['nodes'] or self.local_process_contains('robot_state_publisher'):
                    raise UserError('Robot State Publisher đã chạy. Không phát thêm mô hình/TF.')
                if any('joint_state_publisher' in n for n in s['nodes']):
                    raise UserError('Dừng joint_state_publisher_gui trước để không phát khớp cạnh tranh.')
                urdf = Path(c['description_urdf']).expanduser().resolve()
                ros_ws = Path(c['ros_workspace']).expanduser().resolve()
                setup = ros_ws / 'install' / 'setup.bash'
                if not urdf.is_file():
                    raise UserError(f'Không có URDF: {urdf}')
                if not setup.is_file():
                    raise UserError(f'Không có overlay ROS 2: {setup}')
                command = (
                    "source /opt/ros/jazzy/setup.bash\n"
                    f"source {shlex.quote(str(setup))}\n"
                    "exec ros2 run robot_state_publisher robot_state_publisher --ros-args "
                    f"-p robot_description:=\"$(cat {shlex.quote(str(urdf))})\""
                )
                return self._spawn_shell(name, command, **spawn_kwargs)

            if name == 'lidar':
                if s.get('scan_publishers') or self.local_process_contains('start_ld14p_wifi.sh'):
                    raise UserError('LiDAR/Bridge đã chạy bên ngoài ứng dụng. Không mở thêm kết nối TCP.')
                script = Path(c['lidar_script']).expanduser().resolve()
                if not script.is_file():
                    raise UserError(f'Không tìm thấy script LiDAR: {script}')
                project = script.parent.parent
                command = (
                    f"cd {shlex.quote(str(project))}\n"
                    f"exec ./ubuntu/{shlex.quote(script.name)} {shlex.quote(str(c['lidar_ip']))} {int(c['lidar_port'])}"
                )
                return self._spawn_shell(name, command, str(project), **spawn_kwargs)

            if name == 'costmap':
                # WEB V5.2: use the exact Terminal workflow already verified on
                # this robot.  The launch file remains the owner of its normal
                # Nav2 lifecycle manager; Web only verifies the result and, when
                # needed, applies the same manual activate commands the operator
                # previously used in Terminal.
                exact = self.recoverable_costmap_processes()
                any_nodes = ('/global_costmap/global_costmap' in s['nodes'] or
                             '/local_costmap/local_costmap' in s['nodes'])
                if exact:
                    return {'message': 'Costmap đã tồn tại ngoài Web; chỉ quan sát, không mở bản trùng hoặc đổi lifecycle.',
                            'external': True, 'managed': False}
                if any_nodes:
                    raise UserError('Có Costmap ngoài Web không nhận diện được. Dừng stack đó trước để tránh trùng node.')
                if '/amcl' not in s['nodes']:
                    raise UserError('Bật AMCL trước khi bật Costmap Nav2.')
                tf_map = s.get('tf_map') or {}
                if not self._fresh_age(tf_map.get('age'), minimum=-0.15, maximum=1.0):
                    raise UserError('Chưa có TF map→odom/base_link mới. Hãy đặt Initial Pose trước.')
                if s.get('scan_publishers') != 1:
                    raise UserError('Cần đúng một /scan publisher trước khi bật Costmap.')
                ros_ws = Path(c['ros_workspace']).expanduser().resolve()
                setup = ros_ws / 'install' / 'setup.bash'
                launch_file = ros_ws / 'install' / 'delivery_robot_nav2' / 'share' / 'delivery_robot_nav2' / 'launch' / 'costmaps_only.launch.py'
                if not setup.is_file():
                    raise UserError(f'Không có overlay ROS 2: {setup}')
                if not launch_file.is_file():
                    raise UserError(f'Không có launch Costmap đã build: {launch_file}')

                command = (
                    'source /opt/ros/jazzy/setup.bash\n'
                    f'source {shlex.quote(str(setup))}\n'
                    'exec ros2 launch delivery_robot_nav2 costmaps_only.launch.py'
                )
                result = self._spawn_shell(
                    'costmap', command,
                    context={'stack': 'delivery_robot_nav2', 'startup': 'costmaps_only.launch.py'},
                    **spawn_kwargs
                )
                try:
                    # Give the launch-file lifecycle manager a chance to autostart
                    # first. If it leaves a node inactive, _ensure_costmaps_active
                    # applies configure/activate exactly as the proven Terminal fix.
                    states = self._ensure_costmaps_active(**({'cancel_event': cancel_event} if cancel_event is not None else {}))
                    with self.lock:
                        if 'costmap' in self.children:
                            self.children['costmap']['context']['lifecycle'] = states
                except Exception:
                    try:
                        self.stop('costmap')
                    except Exception:
                        pass
                    raise
                result['message'] = 'Đã chạy costmaps_only.launch.py và xác nhận Global + Local Costmap ACTIVE.'
                return result

            if name == 'rviz':
                if '/rviz2' in s['nodes'] or self.local_process_contains('rviz2'):
                    raise UserError('RViz2 đang chạy. Dùng cửa sổ hiện có thay vì mở bản trùng.')
                if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
                    raise UserError('Web server không có DISPLAY/WAYLAND_DISPLAY để mở cửa sổ RViz2.')
                config = Path(__file__).parent / 'rviz' / 'localization.rviz'
                command = (
                    "source /opt/ros/jazzy/setup.bash\n"
                    f"source {shlex.quote(str(Path(c['ros_workspace']).expanduser().resolve() / 'install/setup.bash'))}\n"
                    f"exec rviz2 -d {shlex.quote(str(config))}"
                )
                return self._spawn_shell(name, command, **spawn_kwargs)

            # SLAM is intentionally separate from the base system connect.
            if '/slam_toolbox' in s['nodes'] or self.local_process_contains('async_slam_toolbox_node'):
                raise UserError('SLAM đang chạy bên ngoài ứng dụng. Dừng phiên hiện có trước.')
            if '/amcl' in s['nodes'] or self._fresh_age((s.get('tf_map') or {}).get('age'), minimum=-0.1, maximum=1.0):
                raise UserError('Đã có AMCL hoặc nguồn TF map. Dừng định vị trước khi tạo map mới.')
            if s['mapping_issues']:
                raise UserError('Chưa bắt đầu SLAM: ' + ' '.join(s['mapping_issues']))

            from ament_index_python.packages import get_package_share_directory
            template = Path(get_package_share_directory('slam_toolbox')) / 'config/mapper_params_online_async.yaml'
            data = yaml.safe_load(template.read_text())
            params = data['slam_toolbox']['ros__parameters']
            params.update(
                odom_frame=c['odom_frame'], map_frame=c['map_frame'],
                base_frame=c['base_frame'], scan_topic='/web/scan_slam', mode='mapping',
                use_sim_time=False, use_map_saver=True, transform_publish_period=0.02,
                restamp_tf=False, minimum_travel_distance=0.15, minimum_travel_heading=0.15,
            )
            # A new web SLAM session must never resume a previous serialized map.
            if 'map_file_name' in params:
                params['map_file_name'] = ''
            target = self.store.root / 'runtime' / 'slam_web.yaml'
            target.write_text(yaml.safe_dump(data, sort_keys=False))
            self.ros.begin_map_session()
            ros_ws = Path(c['ros_workspace']).expanduser().resolve()
            command = (
                "source /opt/ros/jazzy/setup.bash\n"
                f"source {shlex.quote(str(ros_ws / 'install/setup.bash'))}\n"
                "exec ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false "
                "autostart:=true use_lifecycle_manager:=false "
                f"slam_params_file:={shlex.quote(str(target))}"
            )
            try:
                return self._spawn_shell(name, command, **spawn_kwargs)
            except Exception:
                self.ros.end_map_session()
                raise

    def start_localization(self, map_yaml: Path, map_id: str, map_name: str, cancel_event=None):
        if self.closing or (cancel_event is not None and cancel_event.is_set()):
            raise UserError('Đã hủy khởi động định vị.')
        self._require_navigation_stopped('thay đổi bản đồ định vị')
        if self.offline:
            raise UserError('OFFLINE: không khởi động AMCL thật.')
        if getattr(self.ros, 'maintenance', False):
            raise UserError('Đang cấu hình USB; không thay đổi tiến trình robot.')
        path = Path(map_yaml).expanduser().resolve()
        if path.suffix.lower() not in ('.yaml', '.yml') or not path.is_file():
            raise UserError('Không tìm thấy YAML bản đồ đã chọn.')
        with self.lock:
            if 'localization' in self.children and self.children['localization']['process'].poll() is None:
                current = self.children['localization'].get('context', {})
                raise UserError('Localization đang chạy với map ' + str(current.get('map_name', 'đã chọn')) + '.')
            s, c = self._state_for_process_checks(), self.store.config
            if '/slam_toolbox' in s.get('nodes', []) or self.local_process_contains('async_slam_toolbox_node'):
                raise UserError('SLAM đang chạy. Dừng SLAM trước khi bật AMCL với map đã lưu.')
            if '/amcl' in s.get('nodes', []) or '/map_server' in s.get('nodes', []):
                raise UserError('AMCL/Map Server đang chạy ngoài web. Dừng Terminal AMCL cũ trước khi chuyển quyền cho web.')
            tf = s.get('tf_odom') or {}
            if not self._fresh_age(tf.get('age'), minimum=-0.15, maximum=1.0):
                raise UserError('Chưa có TF odom→base_link mới từ EKF V14.')
            if s.get('scan_publishers') != 1 or s.get('scan', {}).get('receive_age') is None or s['scan']['receive_age'] > 1.0:
                raise UserError('Cần đúng một /scan publisher và dữ liệu LiDAR mới trước khi bật AMCL.')
            if not s.get('tf_odom_laser'):
                raise UserError('Thiếu TF odom→base_link→base_laser.')
            ros_ws = Path(c['ros_workspace']).expanduser().resolve()
            setup = ros_ws / 'install' / 'setup.bash'
            if not setup.is_file():
                raise UserError(f'Không có overlay ROS 2: {setup}')
            command = (
                "source /opt/ros/jazzy/setup.bash\n"
                f"source {shlex.quote(str(setup))}\n"
                "exec ros2 launch delivery_robot_localization_v14 amcl_localization.launch.py "
                f"map:={shlex.quote(str(path))}"
            )
            result = self._spawn_shell('localization', command,
                context={'map_id': map_id, 'map_name': map_name, 'map_yaml': str(path)},
                **({'cancel_event': cancel_event} if cancel_event is not None else {}))
            result['message'] = f'Đã mở Map Server + AMCL với map “{map_name}”. Hãy đặt Initial Pose nếu AMCL chưa có map→odom.'
            return result

    @staticmethod
    def _owned_group_running(pgid):
        """Detect surviving children of our session, excluding reaped/zombie tasks."""
        for directory in Path('/proc').glob('[0-9]*'):
            try:
                if directory.stat().st_uid != os.getuid():
                    continue
                fields = (directory / 'stat').read_text().rsplit(')', 1)[1].split()
                if int(fields[2]) == pgid and fields[0] not in ('Z', 'X'):
                    return True
            except (OSError, ValueError, IndexError):
                pass
        return False

    def _stop_owned_child(self, process):
        # All children are spawned with start_new_session=True. Retain their
        # original process-group id even if the launch parent exits first.
        pgid = process.pid
        for sig, seconds in ((signal.SIGINT, 5.0), (signal.SIGTERM, 2.0), (signal.SIGKILL, 1.0)):
            if process.poll() is not None and not self._owned_group_running(pgid):
                return
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                pass
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                try:
                    process.wait(timeout=min(0.1, max(0.001, deadline - time.monotonic())))
                except subprocess.TimeoutExpired:
                    pass
                if process.poll() is not None and not self._owned_group_running(pgid):
                    return
                time.sleep(0.05)
        raise UserError('Nhóm tiến trình do Web quản lý vẫn chưa dừng; xem log và trạng thái hệ thống.')

    def _stop_one(self, name, *, matches=None, recovered=False):
        """Stop one already selected owner; no graph-dependent shutdown gates."""
        with self.lock:
            if self.operations.get(name) == 'stopping':
                raise UserError('Đang dừng ' + name + '.')
            self.operations[name] = 'stopping'
            item = self.children.get(name)
        warning = None
        try:
            if name != 'rviz':
                self.ros.motion.stop('Đang dừng ' + name + '.', force=True)
            if name == 'agent':
                message = self.agent.stop()
                verified = self.agent.snapshot(fresh=True)
                if verified.get('running') or not verified.get('status_known', True):
                    raise UserError('Helper Agent chưa xác nhận container đã dừng.')
                self.agent_started = False
            elif matches:
                self._stop_exact_processes(matches, name)
                message = 'Đã dừng tiến trình dự án được xác minh.'
            elif item is not None:
                self._stop_owned_child(item['process'])
                try:
                    item['handle'].close()
                except Exception:
                    pass
                message = 'Đã dừng.'
            else:
                raise UserError('Không tìm thấy tiến trình thuộc dự án để dừng.')
            if name in ('navigation', 'mission', 'cmd_mux'):
                gateway = getattr(self.ros, 'navigation', None)
                retire = getattr(gateway, 'managed_shutdown_complete', None)
                if retire is not None:
                    try:
                        retired = retire([name])
                        record = getattr(self.store, 'interrupt_delivery', None)
                        if record is not None and isinstance(retired, dict):
                            record(retired.get('mission') or {})
                    except Exception as exc:
                        warning = 'Tiến trình đã dừng; chưa cập nhật được trạng thái: ' + str(exc)
            if name == 'slam':
                end = getattr(self.ros, 'end_map_session', None)
                if end is not None:
                    end()
            try:
                self._reset_after_stop(name)
            except Exception as exc:
                warning = 'Tiến trình đã dừng; chưa làm mới được dữ liệu: ' + str(exc)
            self.store.audit('process_stop', name + (' recovered project' if recovered else ''))
            result = {'message': message, 'managed': not recovered, 'recovered': recovered,
                      'process_stopped': True}
            if warning:
                result['warning'] = warning
            return result
        except Exception as exc:
            with self.lock:
                self.failures[name] = str(exc)
            raise
        finally:
            with self.lock:
                self.operations.pop(name, None)

    def stop(self, name, progress=None, *, cascade=True, request_stop=True):
        if self.offline:
            raise UserError('OFFLINE: không dừng dịch vụ thật.')
        if name not in STOP_ORDER:
            raise UserError('Thành phần không nằm trong danh sách được phép.')
        dependencies = DEPENDENTS.get(name, ()) if cascade else ()
        targets = tuple(component for component in STOP_ORDER if component in dependencies or component == name)
        results = self._stop_plan(targets, progress, request_stop=request_stop,
                                  strict_dependencies=True)
        failed = next((row for row in results if not row.get('ok')), None)
        if failed is not None:
            label = ('Dừng Navigation trước. ' if failed['component'] == 'navigation' else '')
            raise UserError(label + failed['component'] + ': ' + failed['message'])
        return {'message': 'Đã dừng ' + name + '.', 'ok': True,
                'managed': any(row.get('managed') for row in results), 'results': results,
                'process_stopped': True}

    def log(self, name):
        if name == 'agent':
            return 'OFFLINE: không đọc Agent thật.' if self.offline else self.agent.log()
        with self.lock:
            item = self.children.get(name)
            if not item:
                st = self.status().get(name, {})
                if st.get('running') and st.get('external') and st.get('recoverable'):
                    return ('Tiến trình đúng của dự án còn từ phiên Web trước nên phiên hiện tại không có file log. '
                            'Bấm Dừng hệ thống hoặc Kết nối hệ thống sau khi dọn sạch để Web khởi động lại và tạo log mới.')
                if st.get('running') and st.get('external'):
                    return 'Thành phần đang chạy ngoài Web; log thuộc Terminal/tiến trình ngoài đó.'
                return 'Thành phần chưa được Web khởi động trong phiên này.'
            with open(item['log'], 'rb') as f:
                f.seek(0, 2)
                size = f.tell()
                f.seek(max(0, size - 18000))
                return f.read().decode(errors='replace')

    def close(self):
        """Shutdown only this session's owners; explicit UI Stop can recover older sessions."""
        with self.lock:
            self.closing = True
        try:
            return self.stop_all(recover=False, request_stop=False)
        except Exception as exc:
            self.store.audit('shutdown_stop_failed', str(exc))
            return [{'component': 'system', 'ok': False, 'state': 'failed', 'message': str(exc)}]
