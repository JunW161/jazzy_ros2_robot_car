"""Start only the operator-verified robot commands, in the verified order.

Web V2.5 Map V4 keeps the verified base stack unchanged:
Agent -> Odom bridge(no TF) -> EKF official TF -> Robot model -> LiDAR.
SLAM and saved-map Localization (Map Server + AMCL) are mutually exclusive actions.
"""
from __future__ import annotations
import os
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


class ProcessManager:
    def __init__(self, store, ros, offline=False):
        self.store, self.ros, self.offline = store, ros, offline
        self.lock = threading.RLock()
        self.children = {}
        self.agent = AgentService(store.config)
        self.agent_started = False

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
    def local_process_matches(needle):
        matches = []
        for p in Path('/proc').glob('[0-9]*'):
            try:
                pid = int(p.name)
                if p.stat().st_uid != os.getuid() or pid == os.getpid():
                    continue
                args = (p / 'cmdline').read_bytes().replace(b'\x00', b' ').decode(errors='replace').strip()
                if needle in args:
                    matches.append({'pid': pid, 'cmdline': args})
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
        """Make Web startup observationally OFF without starting anything.

        A previous Web process may have been killed before its finally block ran,
        leaving ROS children re-parented to systemd --user.  Only exact project
        processes are cleaned here.  Unknown/manual processes are left alone and
        will be reported as external conflicts by the preflight step.
        """
        cleaned = []
        # Stop dependants first, then the verified base stack in reverse order.
        for name, label, getter in (
            ('rviz', 'RViz2 của Web cũ', lambda: self.recoverable_base_processes('rviz')),
            ('costmap', 'Nav2 Costmap cũ', self.recoverable_costmap_processes),
            ('localization', 'AMCL/Map Server cũ', self.recoverable_localization_processes),
            ('slam', 'SLAM cũ', lambda: self.recoverable_base_processes('slam')),
            ('lidar', 'LiDAR cũ', lambda: self.recoverable_base_processes('lidar')),
            ('robot_model', 'Robot Model cũ', lambda: self.recoverable_base_processes('robot_model')),
            ('ekf_odom', 'EKF V14 cũ', lambda: self.recoverable_base_processes('ekf_odom')),
            ('odom_bridge', 'Odom Bridge cũ', lambda: self.recoverable_base_processes('odom_bridge')),
        ):
            matches = getter()
            if not matches:
                continue
            try:
                self._stop_exact_processes(matches, label)
                cleaned.append(name)
                self.store.audit('startup_cleanup', name)
            except Exception as exc:
                self.store.audit('startup_cleanup_failed', f'{name}: {exc}')

        # The Docker Agent has its own narrow helper. If it was left running by a
        # previous Web session, stop it so opening the Web never means "system ON".
        try:
            info = self.agent.snapshot(fresh=True)
            if info.get('running'):
                self.agent.stop()
                cleaned.append('agent')
                self.store.audit('startup_cleanup', 'agent')
        except Exception as exc:
            self.store.audit('startup_cleanup_failed', f'agent: {exc}')
        self.agent_started = False
        if any(name in cleaned for name in ('lidar', 'robot_model', 'ekf_odom', 'odom_bridge')):
            self._wait_base_graph_clear(timeout=5.0)
        return cleaned

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

    def prepare_base_stack_for_connect(self):
        """Stop only stale, exact project processes before a fresh Connect.

        Nothing is started here.  The button press remains the only action that
        starts Agent -> Odom -> EKF -> Robot Model -> LiDAR.  This also handles
        an EKF child re-parented to systemd --user after an older Web session.
        """
        if self.offline:
            raise UserError('OFFLINE: không khởi động phần cứng/ROS từ giao diện thử.')

        # If this Web instance already owns a base component, do not tear it
        # down behind the operator's back.  Treat repeated Connect as a no-op
        # conflict instead of mixing two sessions.
        with self.lock:
            owned = [name for name in ('odom_bridge','ekf_odom','robot_model','lidar')
                     if name in self.children and self.children[name]['process'].poll() is None]
        if owned:
            raise UserError('Hệ thống đã được phiên Web này khởi động: ' + ', '.join(owned) + '. Dùng Dừng hệ thống trước khi Kết nối lại.')

        cleaned = []
        for name, label in (
            ('lidar', 'LiDAR cũ'),
            ('robot_model', 'Robot Model cũ'),
            ('ekf_odom', 'EKF V14 cũ'),
            ('odom_bridge', 'Odom Bridge cũ'),
        ):
            matches = self.recoverable_base_processes(name)
            if matches:
                self._stop_exact_processes(matches, label)
                cleaned.append(name)
                self.store.audit('connect_cleanup', name)

        # Stop the exact Web Agent helper if a previous Web session left it on.
        try:
            info = self.agent.snapshot(fresh=True)
            if info.get('running'):
                self.agent.stop()
                self.agent_started = False
                cleaned.append('agent')
                self.store.audit('connect_cleanup', 'agent')
        except Exception:
            pass

        if cleaned:
            self._wait_base_graph_clear(timeout=5.0)
        return cleaned

    def preflight_base_stack(self):
        """Validate the whole base stack before starting the first component.

        This prevents the old partial-start failure mode where Agent/Odom/EKF were
        already started and the sequence only failed later at Robot Model or LiDAR.
        """
        if self.offline:
            raise UserError('OFFLINE: không khởi động phần cứng/ROS từ giao diện thử.')
        s, c = self.ros.status(), self.store.config
        nodes = s.get('nodes', [])

        # Exact stale project processes should have been cleaned on Web startup.
        stale = {}
        for name in ('odom_bridge', 'ekf_odom', 'robot_model', 'lidar'):
            got = self.recoverable_base_processes(name)
            if got:
                stale[name] = [x['pid'] for x in got]
        if stale:
            raise UserError('Còn tiến trình cũ của dự án: ' + str(stale) + '. Dùng Dừng hệ thống hoặc restart Web để dọn sạch.')

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
        """Stop exact project processes without relying on a fixed PID.

        Prefer their process groups so orphaned siblings (map_server, lifecycle
        manager, costmap peer) stop together.  Escalate to SIGKILL only after
        graceful SIGINT/SIGTERM have failed, and only for groups discovered from
        the exact project matches above.
        """
        pids = sorted({int(m['pid']) for m in matches})
        if not pids:
            raise UserError(f'Không tìm thấy tiến trình {label} đúng của dự án để dừng.')
        own_pgid = os.getpgrp()
        pgids = set()
        for pid in pids:
            try:
                pgid = os.getpgid(pid)
                if pgid > 1 and pgid != own_pgid:
                    pgids.add(pgid)
            except ProcessLookupError:
                pass

        for sig, seconds in ((signal.SIGINT, 4.0), (signal.SIGTERM, 2.0), (signal.SIGKILL, 1.0)):
            for pgid in list(pgids):
                try:
                    os.killpg(pgid, sig)
                except ProcessLookupError:
                    pass
                except PermissionError:
                    pass
            # Also signal exact PIDs in case an orphan changed process group.
            for pid in self._pids_alive(pids):
                try:
                    os.kill(pid, sig)
                except ProcessLookupError:
                    pass
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                if not self._pids_alive(pids):
                    return
                time.sleep(0.1)
        remaining = self._pids_alive(pids)
        if remaining:
            raise UserError(f'{label} vẫn chưa dừng sau SIGKILL: PID {remaining}')

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

    def _ensure_costmaps_active(self):
        nodes = ('/global_costmap/global_costmap', '/local_costmap/local_costmap')
        deadline = time.monotonic() + 18.0
        last = {}

        # costmaps_only.launch.py already starts lifecycle_manager_costmaps.
        # Wait briefly for its normal autostart path before issuing any manual
        # transition, avoiding configure/activate races.
        grace = time.monotonic() + 7.0
        while time.monotonic() < grace:
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
            result = {
                name: {
                    'running': item['process'].poll() is None,
                    'pid': item['process'].pid,
                    'returncode': item['process'].poll(),
                    'log': str(item['log']), 'managed': True, 'external': False,
                    'message': ('Tiến trình đang chạy; kiểm tra dữ liệu ROS.'
                                if item['process'].poll() is None
                                else 'Tiến trình đã thoát. Bấm Xem log để đọc lỗi.'),
                    'context': item.get('context', {}),
                }
                for name, item in self.children.items()
            }
        if self.offline:
            return result

        state = self.ros.status()
        service = self.agent.snapshot()
        busy = self.udp_in_use(self.store.config['agent_port'])
        external_agent = busy and not service['running']
        result['agent'] = {
            **service,
            'managed': service['installed'],
            'external': external_agent,
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
            if (not item or not item['running']) and node in nodes:
                if name == 'localization':
                    recoverable = self.recoverable_localization_processes()
                elif name in ('odom_bridge', 'ekf_odom', 'robot_model', 'lidar', 'slam', 'rviz'):
                    recoverable = self.recoverable_base_processes(name)
                else:
                    recoverable = []
                can_recover = bool(recoverable)
                result[name] = {
                    'running': True, 'external': True, 'managed': False,
                    'recoverable': can_recover,
                    'pid': recoverable[0]['pid'] if recoverable else None,
                    'returncode': None,
                    'message': (
                        'Tiến trình đúng của dự án còn từ phiên Web trước; Web có thể dừng an toàn rồi khởi động lại để có log mới.'
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
        global_pub = state.get('global_costmap_publishers', 0) > 0
        local_pub = state.get('local_costmap_publishers', 0) > 0
        cost_item = result.get('costmap')
        exact_costmap = self.recoverable_costmap_processes()
        if (not cost_item or not cost_item.get('running')) and (exact_costmap or global_costmap or local_costmap):
            result['costmap'] = {
                'running': True, 'external': True, 'managed': False,
                'recoverable': bool(exact_costmap),
                'pid': exact_costmap[0]['pid'] if exact_costmap else None,
                'returncode': None,
                'ready': global_pub and local_pub,
                'message': (
                    'Costmap đúng stack delivery_robot_nav2 đang chạy; Web có thể dừng an toàn.'
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
                ready = bool(tf) and -0.10 <= tf.get('age', 99) < 0.8
                wait = 'EKF V14 chạy; đang chờ TF odom→base_link filtered.'
            else:
                ready = '/robot_state_publisher' in nodes
                wait = 'Đang chờ node robot_state_publisher; xem log nếu chưa xuất hiện.'
            item['ready'] = ready
            item['message'] = 'Web quản lý · đã thấy dữ liệu/node tương ứng.' if ready else wait
        return result

    def stop_all(self):
        """Operator-requested system stop: reverse the verified start order."""
        results = []
        status = self.status()
        for name in ('rviz', 'costmap', 'localization', 'slam', 'lidar', 'robot_model', 'ekf_odom', 'odom_bridge', 'agent'):
            item = status.get(name, {})
            stoppable = (
                item.get('running') and
                ((item.get('managed') and not item.get('external')) or item.get('recoverable'))
            )
            if stoppable:
                try:
                    results.append({'component': name, 'ok': True, **self.stop(name)})
                except UserError as exc:
                    results.append({'component': name, 'ok': False, 'message': str(exc)})
        return results

    def _spawn_shell(self, name: str, command: str, cwd: str | None = None, context: dict | None = None):
        if name in self.children:
            try:
                self.children[name]['handle'].close()
            except Exception:
                pass
        log_path = self.store.root / 'logs' / f'{name}_{time.time_ns()}.log'
        log = open(log_path, 'ab', buffering=0)
        env = dict(os.environ)
        try:
            process = subprocess.Popen(
                ['/bin/bash', '-lc', command],
                cwd=cwd or str(Path.home()),
                stdout=log, stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,
                start_new_session=True, env=env,
            )
        except Exception:
            log.close()
            raise
        self.children[name] = {'process': process, 'log': log_path, 'handle': log, 'command': command, 'context': dict(context or {})}
        time.sleep(0.8)
        if process.poll() is not None:
            log.close()
            error = log_path.read_text(errors='replace')[-2800:]
            raise UserError(f'{name} thoát với mã {process.returncode}. Log: ' + error)
        self.store.audit('process_start', name)
        return {'message': f'Đã khởi động {name} theo lệnh đã kiểm chứng; theo dõi dữ liệu thật ở Tổng quan.'}

    def start(self, name):
        if self.offline:
            raise UserError('OFFLINE: không khởi động phần cứng/ROS từ giao diện thử.')
        if name not in ('agent', 'odom_bridge', 'ekf_odom', 'robot_model', 'lidar', 'slam', 'rviz', 'costmap'):
            raise UserError('Thành phần không nằm trong danh sách được phép.')
        if getattr(self.ros, 'maintenance', False):
            raise UserError('Đang cấu hình USB; không thay đổi tiến trình robot.')

        with self.lock:
            if name in self.children and self.children[name]['process'].poll() is None:
                return {'message': 'Đã chạy bởi ứng dụng; không mở bản trùng.'}
            s, c = self.ros.status(), self.store.config

            if name == 'agent':
                before = self.agent.snapshot(fresh=True)
                if not before['running'] and self.udp_in_use(c['agent_port']):
                    raise UserError('UDP 8888 đang bị Agent/lệnh Docker ngoài web chiếm. Dừng Terminal Agent cũ trước.')
                message = self.agent.start()
                if not before['running']:
                    self.agent_started = True
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
                return self._spawn_shell(name, command, str(project))

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
                return self._spawn_shell(name, command)

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
                return self._spawn_shell(name, command)

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
                return self._spawn_shell(name, command, str(project))

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
                    states = self._ensure_costmaps_active()
                    return {
                        'message': 'Costmap đúng của dự án đã tồn tại; Web đã đảm bảo Global + Local ACTIVE.',
                        'states': states,
                    }
                if any_nodes:
                    raise UserError('Có Costmap ngoài Web không nhận diện được. Dừng stack đó trước để tránh trùng node.')
                if '/amcl' not in s['nodes']:
                    raise UserError('Bật AMCL trước khi bật Costmap Nav2.')
                tf_map = s.get('tf_map') or {}
                if not tf_map or not -0.15 <= tf_map.get('age', 99) < 1.0:
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
                    context={'stack': 'delivery_robot_nav2', 'startup': 'costmaps_only.launch.py'}
                )
                try:
                    # Give the launch-file lifecycle manager a chance to autostart
                    # first. If it leaves a node inactive, _ensure_costmaps_active
                    # applies configure/activate exactly as the proven Terminal fix.
                    states = self._ensure_costmaps_active()
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
                return self._spawn_shell(name, command)

            # SLAM is intentionally separate from the base system connect.
            if '/slam_toolbox' in s['nodes'] or self.local_process_contains('async_slam_toolbox_node'):
                raise UserError('SLAM đang chạy bên ngoài ứng dụng. Dừng phiên hiện có trước.')
            if '/amcl' in s['nodes'] or (s.get('tf_map') and -0.1 <= s['tf_map'].get('age', 99) < 1.0):
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
                return self._spawn_shell(name, command)
            except Exception:
                self.ros.end_map_session()
                raise

    def start_localization(self, map_yaml: Path, map_id: str, map_name: str):
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
            s, c = self.ros.status(), self.store.config
            if '/slam_toolbox' in s.get('nodes', []) or self.local_process_contains('async_slam_toolbox_node'):
                raise UserError('SLAM đang chạy. Dừng SLAM trước khi bật AMCL với map đã lưu.')
            if '/amcl' in s.get('nodes', []) or '/map_server' in s.get('nodes', []):
                raise UserError('AMCL/Map Server đang chạy ngoài web. Dừng Terminal AMCL cũ trước khi chuyển quyền cho web.')
            tf = s.get('tf_odom') or {}
            if not tf or not -0.15 <= tf.get('age', 99) < 1.0:
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
            result = self._spawn_shell('localization', command, context={'map_id': map_id, 'map_name': map_name, 'map_yaml': str(path)})
            result['message'] = f'Đã mở Map Server + AMCL với map “{map_name}”. Hãy đặt Initial Pose nếu AMCL chưa có map→odom.'
            return result

    def stop(self, name):
        if self.offline:
            raise UserError('OFFLINE: không dừng dịch vụ thật.')
        if name == 'agent':
            if not self.agent.snapshot(fresh=True)['running']:
                raise UserError('Agent helper không chạy; web không tắt Agent ngoài web.')
            self.ros.motion.stop('Dừng trước khi ngắt Agent.', force=True)
            time.sleep(0.2)
            message = self.agent.stop()
            self.agent_started = False
            self.store.audit('process_stop', 'agent docker exact-command helper')
            return {'message': message}

        # Localization depends on a stable map TF; stop managed/recoverable
        # costmaps first so no Nav2 layer is left dangling after AMCL stops.
        if name == 'localization':
            cst = self.status().get('costmap', {})
            if cst.get('running'):
                if not cst.get('external') or cst.get('recoverable'):
                    self.stop('costmap')
                else:
                    raise UserError('Có Costmap ngoài Web không nhận diện được. Dừng Costmap đó trước khi dừng AMCL.')

        with self.lock:
            item = self.children.get(name)
            if name == 'localization' and (item is None or item['process'].poll() is not None):
                matches = self.recoverable_localization_processes()
                if matches:
                    self.ros.motion.stop('Dừng điều khiển trước khi thay đổi tiến trình.', force=True)
                    time.sleep(0.2)
                    self._stop_exact_processes(matches, 'Localization/AMCL')
                    self.store.audit('process_stop', 'localization recovered exact project processes')
                    return {'message': 'Đã dừng Map Server + AMCL đúng của dự án, kể cả process mồ côi; không cần PID thủ công.'}
            if name == 'costmap' and (item is None or item['process'].poll() is not None):
                matches = self.recoverable_costmap_processes()
                if matches:
                    self.ros.motion.stop('Dừng điều khiển trước khi thay đổi tiến trình.', force=True)
                    time.sleep(0.2)
                    self._stop_exact_processes(matches, 'Nav2 Costmap')
                    self.store.audit('process_stop', 'costmap recovered exact project processes')
                    return {'message': 'Đã dừng Global + Local Costmap đúng của delivery_robot_nav2.'}
            if name in ('odom_bridge', 'ekf_odom', 'robot_model', 'lidar', 'slam', 'rviz') and (item is None or item['process'].poll() is not None):
                matches = self.recoverable_base_processes(name)
                if matches:
                    self.ros.motion.stop('Dừng điều khiển trước khi thay đổi tiến trình.', force=True)
                    time.sleep(0.2)
                    self._stop_exact_processes(matches, name)
                    self.store.audit('process_stop', f'{name} recovered exact project processes')
                    return {'message': f'Đã dừng tiến trình {name} cũ đúng của dự án.'}
            if item is None:
                raise UserError('Web không sở hữu và không nhận diện được đúng tiến trình của dự án. Dùng Terminal gốc.')
            self.ros.motion.stop('Dừng điều khiển trước khi thay đổi tiến trình.', force=True)
            time.sleep(0.2)
            process = item['process']
            if process.poll() is None:
                for sig, seconds in ((signal.SIGINT, 5), (signal.SIGTERM, 2), (signal.SIGKILL, 1)):
                    try:
                        os.killpg(process.pid, sig)
                    except ProcessLookupError:
                        break
                    try:
                        process.wait(timeout=seconds)
                        break
                    except subprocess.TimeoutExpired:
                        continue
                if process.poll() is None:
                    raise UserError('Tiến trình do Web quản lý vẫn chưa dừng sau SIGKILL.')
            # ros2 launch may exit before a child; clean only exact project orphans.
            if name == 'localization':
                leftovers = self.recoverable_localization_processes()
                if leftovers:
                    self._stop_exact_processes(leftovers, 'Localization/AMCL')
            elif name == 'costmap':
                leftovers = self.recoverable_costmap_processes()
                if leftovers:
                    self._stop_exact_processes(leftovers, 'Nav2 Costmap')
            try:
                item['handle'].close()
            except Exception:
                pass
            if name == 'slam':
                # Stop means end this mapping session and forget the live map.
                # Saved map files in the library are untouched.
                self.ros.end_map_session()
            self.store.audit('process_stop', name)
            return {'message': (
                'Đã dừng SLAM và xóa bản đồ LIVE của phiên vừa kết thúc.'
                if name == 'slam' else f'Đã dừng {name}. Các thành phần khác được giữ nguyên.'
            )}

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
        # Only processes started by this web instance are stopped on server shutdown.
        for name in ('rviz', 'costmap', 'localization', 'slam', 'lidar', 'robot_model', 'ekf_odom', 'odom_bridge'):
            if name in self.children:
                try:
                    self.stop(name)
                except Exception:
                    pass
        if self.agent_started:
            try:
                self.stop('agent')
            except Exception:
                pass
