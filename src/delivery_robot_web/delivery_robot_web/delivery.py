"""Application orchestration for Nav2 and delivery; no shell input from the browser.

Long lifecycle bring-up is a background operation. A STOP invalidates a pending
start before waiting for process cleanup, so a slow bring-up cannot restart it.
"""
from __future__ import annotations

from array import array
from contextlib import nullcontext
from copy import deepcopy
import math
import threading
import time
import uuid

from .core import UserError
from .maps import load_yaml_map, _map_occupancy_values, yaw_of, validate_point_cell


ACTIVE_GOALS = {'SENDING', 'ACTIVE', 'CANCELING', 'CANCELLING', 'ACCEPTED'}
ACTIVE_MISSIONS = {'GOING_TO_PICKUP', 'WAITING_PICKUP', 'GOING_TO_DROPOFF', 'WAITING_DROPOFF'}


class DeliveryCoordinator:
    def __init__(self, store, ros, motion, manager):
        self.store, self.ros, self.motion, self.manager = store, ros, motion, manager
        self.lock = threading.RLock()
        self.dispatch_lock = threading.RLock()
        self.process_lock = threading.RLock()
        self.operation = {'id': '', 'kind': '', 'state': 'idle', 'message': ''}
        self.cancel_event = threading.Event()
        self.thread = None
        self._map_cache = None
        self.system = None

    def system_busy(self):
        return self.system is not None and self.system.busy()

    @property
    def gateway(self):
        gateway = getattr(self.ros, 'navigation', None)
        if gateway is None:
            raise UserError('Chưa có bộ kết nối Navigation. Khởi động lại Web sau khi cập nhật.')
        return gateway

    def snapshot(self):
        if getattr(self.ros, 'navigation', None) is None:
            return {'offline': self.manager.offline, 'ready': False,
                    'issues': ['Bộ kết nối Navigation chưa sẵn sàng.'],
                    'lifecycle': {}, 'mission': {'available': False},
                    'goal': {'state': 'IDLE', 'active': False}, 'mux': {'available': False},
                    'plan': {'frame_id': 'map', 'points': []}}
        result = self.gateway.snapshot()
        self.store.observe_delivery(result.get('mission', {}))
        return result

    def verified_map_id(self):
        key = self._map_cache
        if key is None:
            return None
        revision, message, _age = self.ros.current_ros_map()
        return key[0] if message is not None and key[1:3] == (revision, id(message)) else None

    def operation_snapshot(self):
        with self.lock:
            return deepcopy(self.operation)

    def is_active(self):
        s = self.snapshot()
        mission, goal = s.get('mission', {}), s.get('goal', {})
        return (bool(mission.get('active')) or mission.get('state_name') in ACTIVE_MISSIONS or
                bool(goal.get('active')) or goal.get('state') in ACTIVE_GOALS or
                bool(s.get('external_goal_active')) or bool(s.get('command_pending')))

    def ensure_idle(self):
        if self.is_active():
            raise UserError('Đang có nhiệm vụ/điều hướng. Hủy hoặc Dừng trước khi thay đổi hệ thống.')

    def _ensure_dispatch_ready(self):
        if self.manager.offline:
            raise UserError('OFFLINE: chỉ xem giao diện, không điều khiển robot.')
        if self.motion.status()['armed']:
            raise UserError('Đang giữ quyền lái tay. Dừng điều khiển tay trước khi giao hàng.')
        if self.operation_snapshot()['state'] == 'running':
            raise UserError('Hệ thống đang khởi động/dừng Navigation; chờ trạng thái sẵn sàng.')
        if self.system_busy():
            raise UserError('Hệ thống đang xử lý một thao tác.')
        snapshot = self.snapshot()
        if not snapshot.get('ready'):
            issues = snapshot.get('issues') or ['Navigation chưa sẵn sàng.']
            raise UserError('Chưa nhận nhiệm vụ: ' + '; '.join(str(s) for s in issues[:5]))
        if self.ros.status().get('maintenance'):
            raise UserError('Đang cấu hình thiết bị; chưa nhận nhiệm vụ.')
        self.ensure_idle()

    def validate_map(self, ident):
        """Compare the actual ROS map to the chosen immutable saved map.

        This also works with an externally launched AMCL. Matching a map name
        alone is insufficient: two maps can have the same name and geometry.
        """
        record = self.store.map(ident)
        process = self.manager.status().get('localization', {})
        active_id = process.get('context', {}).get('map_id')
        if active_id and active_id != ident:
            raise UserError('Điểm thuộc bản đồ khác với bản đồ AMCL đang dùng.')
        if self.ros.status().get('slam_state') == 'active':
            raise UserError('Hãy kết thúc SLAM và bật định vị trên bản đồ đã lưu trước khi giao hàng.')
        revision, message, _age = self.ros.current_ros_map()
        if message is None:
            raise UserError('Chưa nhận bản đồ /map từ ROS để đối chiếu điểm đến.')
        directory = self.store.root / 'maps' / ident
        yaml_path = directory / 'map.yaml'
        data, image_path, meta = load_yaml_map(yaml_path, directory)
        key = (ident, revision, id(message), yaml_path.stat().st_mtime_ns,
               image_path.stat().st_mtime_ns)
        if self._map_cache == key:
            return record
        info = message.info
        actual_origin = (info.origin.position.x, info.origin.position.y, yaw_of(info.origin.orientation))
        angle_error = math.atan2(math.sin(actual_origin[2]-meta['origin'][2]),
                                 math.cos(actual_origin[2]-meta['origin'][2]))
        geometry_matches = (
            message.header.frame_id.lstrip('/') == 'map' and
            info.width == meta['width'] and info.height == meta['height'] and
            math.isclose(info.resolution, meta['resolution'], rel_tol=1e-5, abs_tol=1e-8) and
            all(math.isclose(actual_origin[i], meta['origin'][i], abs_tol=1e-5) for i in (0, 1)) and
            abs(angle_error) < 1e-5)
        if not geometry_matches:
            raise UserError('Bản đồ ROS đang chạy không khớp bản đồ đã chọn. Bật AMCL với đúng bản đồ.')
        width, height, expected = _map_occupancy_values(data, image_path)
        actual = array('b', message.data)
        if len(actual) != width * height:
            raise UserError('Dữ liệu /map chưa đủ ô, chưa gửi mục tiêu.')
        # Image files use top-left row order; OccupancyGrid starts bottom-left.
        flipped = array('b')
        for row in range(height-1, -1, -1):
            flipped.extend(expected[row*width:(row+1)*width])
        equal = actual == flipped
        if not equal and meta['mode'] == 'scale':
            # ROS/image integer rounding can differ by one occupancy unit.
            equal = all((a == b if a < 0 or b < 0 else abs(a-b) <= 1)
                        for a, b in zip(actual, flipped))
        if not equal:
            raise UserError('Nội dung bản đồ ROS khác bản đồ của các điểm đã lưu; chưa gửi mục tiêu.')
        self._map_cache = key
        return record

    def _point(self, map_id, point_id):
        point = self.store.point(map_id, str(point_id))
        validate_point_cell(self.store, map_id, point['x'], point['y'])
        return point

    def capture_intent(self):
        """Remember STOP state before a request waits or reads map files."""
        system_epoch = self.system.epoch() if self.system is not None else None
        with self.motion.lock:
            generation = self.motion.stop_generation
        return {'system_epoch': system_epoch, 'motion_generation': generation,
                'gateway_epoch': self.gateway.command_epoch()}

    def check_intent(self, intent):
        if self.system is not None and (
                intent['system_epoch'] != self.system.epoch() or self.system.busy()):
            raise UserError('Yêu cầu đã bị hủy bởi lệnh dừng hoặc thao tác hệ thống.')
        with self.motion.lock:
            if intent['motion_generation'] != self.motion.stop_generation:
                raise UserError('Yêu cầu đã bị hủy bởi lệnh dừng.')

    def dispatch_arguments(self, intent):
        return {'expected_epoch': intent['gateway_epoch'],
                'dispatch_check': lambda: self.check_intent(intent)}

    def finish_manual_arm(self, owner, intent):
        # Only the short local lease change is atomic with MotionLease.stop().
        # No RPC, ROS wait or dispatch_lock is held by emergency STOP.
        ready = self.ros.motion_ready() and self.gateway.manual_authority()
        with self.motion.lock:
            self.check_intent(intent)
            return self.motion.arm(owner, ready)

    def start_delivery(self, data):
        intent = self.capture_intent()
        with self.dispatch_lock:
            self.check_intent(intent)
            self._ensure_dispatch_ready()
            map_id = str(data.get('map_id', ''))
            record = self.validate_map(map_id)
            pickup = self._point(map_id, data.get('pickup_id', ''))
            dropoff = self._point(map_id, data.get('dropoff_id', ''))
            if pickup['kind'] not in ('pickup', 'both') or dropoff['kind'] not in ('delivery', 'both'):
                raise UserError('Chọn đúng loại điểm lấy hàng và giao hàng.')
            if pickup['id'] == dropoff['id']:
                raise UserError('Chọn hai điểm lấy hàng và giao hàng khác nhau.')
            result = self.gateway.start_delivery(pickup, dropoff, **self.dispatch_arguments(intent))
            if not result.get('accepted') or not result.get('mission_id'):
                raise UserError(result.get('message', 'Mission Manager chưa nhận nhiệm vụ.'))
            self.store.record_delivery(result['mission_id'], record, pickup, dropoff)
            self.store.audit('delivery_start', f"{result['mission_id']}: {pickup['name']} → {dropoff['name']}")
            return result

    def navigate_to(self, data):
        intent = self.capture_intent()
        with self.dispatch_lock:
            self.check_intent(intent)
            self._ensure_dispatch_ready()
            map_id = str(data.get('map_id', ''))
            self.validate_map(map_id)
            point = self._point(map_id, data.get('point_id', ''))
            result = self.gateway.navigate_to(point, **self.dispatch_arguments(intent))
            self.store.audit('navigate_to', f"{map_id}:{point['id']}:{point['name']}")
            return result

    def mission_action(self, action):
        if action not in ('confirm_pickup', 'confirm_dropoff', 'cancel', 'reset'):
            raise UserError('Thao tác nhiệm vụ không hợp lệ.')
        # Cancellation must bypass dispatch_lock: never queue STOP behind a slow send.
        if action == 'cancel':
            result = self.gateway.mission_action(action)
        else:
            intent = self.capture_intent()
            with self.dispatch_lock:
                self.check_intent(intent)
                if self.operation_snapshot()['state'] == 'running' or self.system_busy():
                    raise UserError('Đang thay đổi trạng thái Navigation; chưa nhận xác nhận nhiệm vụ.')
                result = self.gateway.mission_action(action, **self.dispatch_arguments(intent))
        self.store.audit('delivery_'+action, str(result.get('message', '')))
        self.snapshot()
        return result

    def emergency_stop(self):
        self.motion.stop('Người vận hành nhấn Dừng.', force=True)
        if self.system is not None:
            self.system.cancel_startup()
        with self.lock:
            self.cancel_event.set()
        if self.manager.offline:
            return {'ok': True, 'mux_confirmed': False,
                    'message': 'OFFLINE: đã khóa điều khiển giao diện; không có robot kết nối.'}
        try:
            result = self.gateway.stop_all()
        except Exception as exc:
            result = {'mux_confirmed': False, 'cancel_confirmed': False, 'message': str(exc)}
        self.store.audit('software_stop', str(result.get('message', '')))
        return {'ok': bool(result.get('mux_confirmed')), **result}

    def begin(self, kind, *, expected_epoch=None):
        # A start reservation and a manual/mission/USB request share one lock.
        # STOP deliberately bypasses it so it can invalidate an in-flight send.
        if kind == 'start':
            if self.system is not None and expected_epoch is None:
                expected_epoch = self.system.epoch()
            if self.system_busy():
                raise UserError('Hệ thống đang xử lý một thao tác.')
            if self.ros.status().get('ros_ok') is False:
                raise UserError('Kết nối ROS bị lỗi. Dừng hệ thống rồi kết nối lại.')
            with self.dispatch_lock:
                return self._begin(kind, expected_epoch=expected_epoch)
        return self._begin(kind)

    def _begin(self, kind, *, expected_epoch=None):
        if kind not in ('start', 'stop'):
            raise UserError('Thao tác Navigation không hợp lệ.')
        if self.manager.offline:
            raise UserError('OFFLINE: không khởi động hoặc dừng tiến trình robot.')
        if kind == 'start':
            if self.system_busy():
                raise UserError('Hệ thống đang xử lý một thao tác.')
            if self.ros.status().get('maintenance'):
                raise UserError('Đang cấu hình thiết bị, chưa bật Navigation.')
            if self.motion.status()['armed']:
                raise UserError('Dừng điều khiển tay trước khi bật Navigation.')
            self.ensure_idle()
        else:
            self.motion.stop('Đang tắt Navigation.', force=True)
            if self.system is not None:
                self.system.cancel_startup()
        reservation = self.system.lock if kind == 'start' and self.system is not None else nullcontext()
        with reservation, self.lock:
            if kind == 'start' and self.system is not None and (
                    expected_epoch != self.system.epoch() or self.system.busy()):
                raise UserError('Khởi động Navigation đã bị hủy bởi lệnh dừng.')
            if self.operation['state'] == 'running' and kind != 'stop':
                raise UserError('Đã có thao tác Navigation đang chạy.')
            if self.operation['state'] == 'running' and self.operation['kind'] == kind:
                return {'operation': deepcopy(self.operation)}
            self.cancel_event.set()
            cancel = self.cancel_event = threading.Event()
            ident = uuid.uuid4().hex
            self.operation = {'id': ident, 'kind': kind, 'state': 'running',
                              'message': 'Đang khởi động Navigation…' if kind == 'start' else 'Đang dừng Navigation…',
                              'created_at': time.time(), 'updated_at': time.time()}
            self.thread = threading.Thread(target=self._run, args=(ident, kind, cancel),
                                           name='navigation-operation', daemon=True)
            self.thread.start()
            return {'operation': deepcopy(self.operation)}

    def _update(self, ident, state, message):
        with self.lock:
            if self.operation['id'] == ident:
                self.operation.update(state=state, message=message, updated_at=time.time())

    def _run(self, ident, kind, cancel):
        started = {}
        completed = False
        try:
            if kind == 'start':
                with self.process_lock:
                    if cancel.is_set():
                        return
                    processes = self.manager.status()
                    costmap = processes.get('costmap', {})
                    if costmap.get('running') and not costmap.get('external'):
                        self.manager.stop('costmap')
                    for name in ('navigation', 'mission'):
                        if cancel.is_set():
                            return
                        before = self.manager.status().get(name, {})
                        try:
                            self.manager.start(name, cancel_event=cancel)
                        finally:
                            # Retain the acquired owner identity even if startup
                            # fails after spawning its process group.
                            acquired = self.manager.status().get(name, {})
                            if (not before.get('running') and acquired.get('managed') and
                                    not acquired.get('external') and acquired.get('pid') is not None):
                                started[name] = acquired['pid']
                deadline = time.monotonic() + 45.0
                while not cancel.wait(0.2):
                    s = self.snapshot()
                    mission = s.get('mission', {})
                    age = mission.get('receive_age')
                    if s.get('ready') and mission.get('available') and age is not None and age <= 3:
                        self._update(ident, 'succeeded', 'Navigation và quản lý giao hàng đã sẵn sàng.')
                        self.store.audit('navigation_started')
                        completed = True
                        return
                    if time.monotonic() >= deadline:
                        issues = s.get('issues') or []
                        raise UserError('Chưa sẵn sàng sau 45 giây. ' + '; '.join(issues[:4]) +
                                        ' Xem log Navigation / Mission để biết nguyên nhân.')
                    procs = self.manager.status()
                    if any(not procs.get(name, {}).get('running') for name in started):
                        raise UserError('Tiến trình Navigation/Mission đã thoát. Xem log tương ứng.')
                self._update(ident, 'failed', 'Đã ngắt quá trình chờ khởi động. Robot ở chế độ dừng.')
            else:
                self.motion.stop('Đang dừng Navigation.', force=True)
                try:
                    acknowledgement = self.gateway.stop_all()
                except Exception as exc:
                    acknowledgement = {'mux_confirmed': False, 'message': str(exc)}
                with self.process_lock:
                    procs = self.manager.status()
                    failures, results, warnings = [], [], []
                    for name in ('mission', 'navigation'):
                        process = procs.get(name, {})
                        if process.get('running'):
                            try:
                                out = self.manager.stop(name, cascade=False, request_stop=False)
                                results.append({'component': name, 'ok': True, **out})
                                if out.get('warning'):
                                    warnings.append(str(out['warning']))
                                warnings.extend(str(row['warning']) for row in out.get('results', [])
                                                if row.get('warning'))
                            except Exception as exc:
                                failures.append(name)
                                results.append({'component': name, 'ok': False, 'message': str(exc)})
                    # Restore a stopped manual mux after the integrated launch exits.
                    # Never replace an external owner or resurrect a canceled operation.
                    if (not failures and not cancel.is_set() and not self.system_busy() and
                            self.ros.status().get('ros_ok', True)):
                        base = self.manager.status()
                        if base.get('agent', {}).get('running'):
                            try:
                                self.manager.start('cmd_mux')
                            except Exception as exc:
                                warnings.append('Mux chưa sẵn sàng: ' + str(exc))
                if not acknowledgement.get('mux_confirmed'):
                    warnings.append('Không nhận được xác nhận STOP từ mux.')
                warnings = list(dict.fromkeys(warnings))
                note = 'Đã tắt Navigation.' if not failures else 'Chưa dừng được: ' + ', '.join(failures) + '.'
                with self.lock:
                    if self.operation['id'] == ident:
                        self.operation.update(results=results, warnings=warnings, stop=acknowledgement)
                self._update(ident, 'failed' if failures else 'succeeded', note)
                self.store.audit('navigation_stopped', note)
        except Exception as exc:
            self._update(ident, 'failed', str(exc))
            self.store.audit('navigation_'+kind+'_failed', str(exc))
        finally:
            if kind == 'start' and not completed and started:
                # Only rollback processes acquired by this request, never a
                # previously running/external stack, or a newer process that
                # reused the component name after a stop/start operation.
                with self.process_lock:
                    owned = []
                    for name, pid in reversed(list(started.items())):
                        p = self.manager.status().get(name, {})
                        if (p.get('running') and p.get('managed') and not p.get('external') and
                                p.get('pid') == pid):
                            owned.append(name)
                    if owned:
                        try:
                            self.gateway.stop_all()
                        except Exception:
                            pass
                    for name in owned:
                        try:
                            self.manager.stop(name, cascade=False, request_stop=False)
                        except Exception as exc:
                            self.store.audit('navigation_rollback_failed', f'{name}: {exc}')
            if cancel.is_set():
                self._update(ident, 'failed', 'Thao tác đã bị dừng; không tự gửi nhiệm vụ hoặc mở lại Navigation.')

    def disconnect(self):
        self.motion.stop('Dừng hệ thống do Web quản lý.', force=True)
        with self.lock:
            self.cancel_event.set()
        try:
            acknowledgement = self.emergency_stop()
        except Exception as exc:
            acknowledgement = {'ok': False, 'mux_confirmed': False, 'message': str(exc)}
        # Wait for the in-progress spawn/handover critical section. The canceled
        # operation cannot start its next component after this barrier.
        with self.process_lock:
            results = self.manager.stop_all()
        return {'results': results, 'stop': acknowledgement}

    def close(self):
        with self.lock:
            self.cancel_event.set()
        thread = self.thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=3.0)
