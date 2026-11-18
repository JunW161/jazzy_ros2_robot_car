"""Serialized, cancellable start/stop operations for the local robot stack.

HTTP requests reserve an operation; process work runs in a worker. A failed ROS
stop acknowledgement never prevents cleanup of verified project processes.
"""
from __future__ import annotations

from copy import deepcopy
import threading
import time
import uuid

from .core import UserError
from .processes import STOP_ORDER


BASE_ORDER = ('agent', 'odom_bridge', 'ekf_odom', 'robot_model', 'lidar', 'cmd_mux')
COMPONENTS = frozenset(STOP_ORDER)
LABELS = {
    'agent': 'micro-ROS Agent', 'odom_bridge': 'Odometry', 'ekf_odom': 'EKF',
    'robot_model': 'Mô hình robot', 'lidar': 'LiDAR', 'cmd_mux': 'Điều phối vận tốc',
    'navigation': 'Navigation', 'mission': 'Nhiệm vụ', 'localization': 'Định vị',
    'slam': 'SLAM', 'costmap': 'Costmap', 'rviz': 'RViz',
}


class OperationCanceled(UserError):
    pass


class SystemCoordinator:
    def __init__(self, store, ros, motion, manager, delivery):
        self.store, self.ros, self.motion = store, ros, motion
        self.manager, self.delivery = manager, delivery
        self.lock = threading.RLock()
        self.operation = {'id': '', 'kind': '', 'state': 'idle', 'message': '',
                          'steps': [], 'results': [], 'warnings': []}
        self.cancel_event = threading.Event()
        self.workers = []
        self.closing = False
        self.stop_epoch = 0

    def snapshot(self):
        with self.lock:
            return deepcopy(self.operation)

    def busy(self):
        with self.lock:
            return self.operation['state'] == 'running'

    def cancel_startup(self):
        with self.lock:
            self.stop_epoch += 1
            if (self.operation['state'] == 'running' and
                    self.operation['kind'] in ('connect', 'component_start')):
                self.cancel_event.set()

    def epoch(self):
        with self.lock:
            return self.stop_epoch

    def _cancel_navigation_startup(self):
        with self.delivery.lock:
            if self.delivery.operation.get('kind') == 'start':
                self.delivery.cancel_event.set()

    def begin(self, kind, component=None, *, expected_epoch=None):
        if expected_epoch is None:
            expected_epoch = self.epoch()
        if kind not in ('connect', 'disconnect', 'component_start', 'component_stop'):
            raise UserError('Thao tác hệ thống không hợp lệ.')
        if kind.startswith('component_') and component not in COMPONENTS:
            raise UserError('Thành phần không hợp lệ.')
        if self.manager.offline:
            raise UserError('OFFLINE: không có hệ thống robot kết nối.')
        stopping = kind in ('disconnect', 'component_stop')
        if stopping:
            # Never wait for a pending send before invalidating its motion lease.
            if component != 'rviz':
                self.motion.stop('Đang dừng hệ thống.', force=True)
                self._cancel_navigation_startup()
            return self._reserve(kind, component)
        with self.delivery.dispatch_lock:
            self.delivery.ensure_idle()
            if self.motion.status()['armed']:
                raise UserError('Dừng điều khiển tay trước khi bật thành phần.')
            if self.ros.status().get('maintenance'):
                raise UserError('Đang cấu hình thiết bị.')
            if self.delivery.operation_snapshot()['state'] == 'running':
                raise UserError('Navigation đang xử lý một thao tác.')
            return self._reserve(kind, component, expected_epoch=expected_epoch)

    def _reserve(self, kind, component, *, expected_epoch=None):
        with self.lock:
            if self.closing:
                raise UserError('Web đang đóng.')
            if kind in ('connect', 'component_start') and expected_epoch != self.stop_epoch:
                raise UserError('Khởi động hệ thống đã bị hủy bởi lệnh dừng.')
            old = self.operation
            if old['state'] == 'running':
                if old['kind'] == kind and old.get('component') == component:
                    return {'operation': deepcopy(old)}
                if old['kind'] == 'disconnect':
                    return {'operation': deepcopy(old)}
                if kind != 'disconnect' and not (kind == 'component_stop' and
                        old['kind'] in ('connect', 'component_start')):
                    raise UserError('Hệ thống đang xử lý một thao tác.')
            self.cancel_event.set()
            if kind in ('disconnect', 'component_stop') and component != 'rviz':
                self.stop_epoch += 1
            event = self.cancel_event = threading.Event()
            ident = uuid.uuid4().hex
            names = BASE_ORDER if kind == 'connect' else STOP_ORDER if kind == 'disconnect' else (component,)
            self.operation = {'id': ident, 'kind': kind, 'component': component,
                'state': 'running', 'message': 'Đang bật hệ thống…' if kind == 'connect' else
                'Đang dừng hệ thống…' if kind == 'disconnect' else
                ('Đang bật ' if kind == 'component_start' else 'Đang dừng ') + LABELS[component] + '…',
                'steps': [{'component': n, 'state': 'pending', 'message': ''} for n in names],
                'results': [], 'warnings': [], 'created_at': time.time(), 'updated_at': time.time()}
            thread = threading.Thread(target=self._run, args=(ident, kind, component, event),
                                      name='system-operation', daemon=True)
            self.workers = [w for w in self.workers if w.is_alive()]
            self.workers.append(thread)
            thread.start()
            return {'operation': deepcopy(self.operation)}

    def _update(self, ident, **values):
        with self.lock:
            if self.operation['id'] == ident:
                self.operation.update(values, updated_at=time.time())

    def _progress(self, ident, row):
        row = dict(row)
        row.setdefault('state', 'succeeded' if row.get('ok') else 'failed')
        with self.lock:
            if self.operation['id'] != ident:
                return
            steps = self.operation['steps']
            index = next((i for i, step in enumerate(steps)
                          if step['component'] == row.get('component')), None)
            if index is None:
                steps.append(row)
            else:
                steps[index].update(row)
            if row['state'] == 'running':
                self.operation['message'] = row.get('message') or 'Đang xử lý…'
            self.operation['updated_at'] = time.time()

    @staticmethod
    def _check_cancel(event):
        if event.is_set():
            raise OperationCanceled('Đã hủy thao tác bật hệ thống.')

    def _check_ros(self):
        status = self.ros.status()
        if status.get('ros_ok', True):
            return
        # Explicit reconnect can repair a stopped executor, after its previous
        # project processes are gone. It never silently resumes an old mission.
        live = [name for name, value in self.manager.status().items() if value.get('running')]
        recover = getattr(self.ros, 'recover_executor', None)
        if live or recover is None:
            raise UserError('Kết nối ROS bị lỗi. Dừng hệ thống trước khi kết nối lại.')
        recover()
        if not self.ros.status().get('ros_ok', False):
            raise UserError('Chưa khôi phục được kết nối ROS. Khởi động lại Web.')

    def _request_stop(self):
        try:
            return self.delivery.emergency_stop()
        except Exception as exc:
            return {'ok': False, 'mux_confirmed': False, 'message': str(exc)}

    def _rollback(self, acquired):
        # Caller holds process_lock. Identity checks also protect a newer owner
        # if a component was replaced outside this operation.
        if not acquired:
            return []
        self._request_stop()
        results = []
        for name, pid in reversed(list(acquired.items())):
            current = self.manager.status().get(name, {})
            if not current.get('managed') or current.get('external'):
                continue
            if name != 'agent' and current.get('pid') != pid:
                continue
            try:
                out = self.manager.stop(name, cascade=False, request_stop=False)
                results.append({'component': name, 'ok': True, **out})
            except Exception as exc:
                results.append({'component': name, 'ok': False, 'message': str(exc)})
        return results

    def _run(self, ident, kind, component, event):
        acquired = {}
        completed = False
        try:
            if kind in ('connect', 'component_start'):
                # Keep acquisition+rollback in the same critical section. Stop
                # can cancel this worker immediately, then wait for that barrier.
                with self.delivery.process_lock:
                    try:
                        self._check_cancel(event)
                        self._check_ros()
                        if kind == 'connect':
                            self.manager.prepare_base_stack_for_connect(reuse_owned=True)
                            self.manager.preflight_base_stack(reuse_owned=True)
                        names = BASE_ORDER if kind == 'connect' else (component,)
                        results = []
                        for name in names:
                            self._check_cancel(event)
                            self._progress(ident, {'component': name, 'state': 'running',
                                'message': 'Đang bật ' + LABELS[name] + '…'})
                            before = self.manager.status().get(name, {})
                            if before.get('running') and before.get('managed') and not before.get('orphaned'):
                                out = {'message': 'Đang hoạt động.'}
                            else:
                                try:
                                    out = self.manager.start(name, cancel_event=event)
                                finally:
                                    after = self.manager.status().get(name, {})
                                    if (not before.get('running') and after.get('managed') and
                                            not after.get('external')):
                                        acquired[name] = after.get('pid')
                            self._check_cancel(event)
                            row = {'component': name, 'ok': True, 'state': 'succeeded', **out}
                            results.append(row)
                            self._progress(ident, row)
                        state = self.manager.status()
                        ready = all(state.get(n, {}).get('ready', False) for n in names)
                        message = ('Hệ thống đã bật.' if kind == 'connect' else
                                   'Đã bật ' + LABELS[component] + '.')
                        if not ready:
                            message += ' Đang chờ dữ liệu.'
                        self._update(ident, state='succeeded', message=message,
                                     results=results, ready=ready)
                        completed = True
                    finally:
                        if not completed:
                            rollback = self._rollback(acquired)
                            if rollback:
                                self._update(ident, rollback=rollback)
            else:
                # STOP and cleanup are separate outcomes: no mux response must
                # not trap the operator in a half-stopped stack.
                acknowledgement = {} if component == 'rviz' else self._request_stop()
                self._update(ident, stop=acknowledgement)
                with self.delivery.process_lock:
                    progress = lambda row: self._progress(ident, row)
                    if kind == 'disconnect':
                        results = self.manager.stop_all(progress=progress, request_stop=False)
                    else:
                        out = self.manager.stop(component, progress=progress, request_stop=False)
                        results = out.get('results') or [{'component': component, 'ok': True, **out}]
                failed = [r for r in results if r.get('ok') is False]
                message = ('Đã dừng hệ thống.' if kind == 'disconnect' else
                           'Đã dừng ' + LABELS[component] + '.')
                if failed:
                    message = 'Chưa dừng được: ' + ', '.join(LABELS.get(r['component'], r['component']) for r in failed) + '.'
                warnings = list(dict.fromkeys(str(r['warning']) for r in results if r.get('warning')))
                if acknowledgement and not acknowledgement.get('mux_confirmed'):
                    warnings.append('Không nhận được xác nhận STOP từ mux; trạng thái tiến trình được kiểm tra riêng.')
                self._update(ident, state='failed' if failed else 'succeeded', message=message,
                             results=results, warnings=warnings)
                self.store.audit('system_'+kind, message)
        except Exception as exc:
            self._update(ident, state='failed', message=str(exc))
            self.store.audit('system_'+kind+'_failed', str(exc))

    def close(self):
        with self.lock:
            self.closing = True
            self.cancel_event.set()
            workers = list(self.workers)
        deadline = time.monotonic() + 3.0
        for worker in workers:
            if worker is not threading.current_thread():
                worker.join(max(0.0, deadline - time.monotonic()))
