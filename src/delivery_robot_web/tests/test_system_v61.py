"""Behavioral operation tests: repeated cycles, cancellation, degraded ROS, HTTP."""
from copy import deepcopy
from types import SimpleNamespace
import threading
import time

import pytest

from delivery_robot_web.core import MotionLease, Store, UserError
from delivery_robot_web.delivery import DeliveryCoordinator
from delivery_robot_web.processes import STOP_ORDER
from delivery_robot_web.system_control import SystemCoordinator, BASE_ORDER


class Gateway:
    def __init__(self):
        self.stops = 0
        self.ack = True
        self.error = False

    def snapshot(self):
        return {'ready': True, 'goal': {'active': False, 'state': 'IDLE'},
                'mission': {'active': False, 'state_name': 'IDLE'}}

    def stop_all(self):
        self.stops += 1
        if self.error:
            raise RuntimeError('ROS executor stopped')
        return {'mux_confirmed': self.ack, 'cancel_confirmed': self.ack}


class Manager:
    offline = False

    def __init__(self):
        self.processes = {}
        self.calls = []
        self.pid = 100
        self.fail_start = None
        self.fail_stop = None
        self.on_start = None
        self.checked = False

    def status(self):
        return deepcopy(self.processes)

    def prepare_base_stack_for_connect(self, reuse_owned=False):
        assert reuse_owned

    def preflight_base_stack(self, reuse_owned=False):
        self.checked = True

    def start(self, name, cancel_event=None):
        self.calls.append(('start', name))
        if name == self.fail_start:
            raise UserError('Start failed: ' + name)
        self.pid += 1
        self.processes[name] = {'running': True, 'ready': True, 'pid': self.pid,
                                'managed': True, 'external': False}
        if self.on_start:
            self.on_start(name, cancel_event)
        return {'message': 'Started ' + name}

    def stop(self, name, progress=None, *, cascade=True, request_stop=True):
        assert request_stop is False
        self.calls.append(('stop', name))
        if progress:
            progress({'component': name, 'state': 'running', 'message': 'Stopping ' + name})
        if name == self.fail_stop:
            raise OSError('Stop failed: ' + name)
        self.processes[name] = {'running': False, 'ready': False, 'managed': False}
        row = {'component': name, 'ok': True, 'state': 'succeeded', 'message': 'Stopped'}
        if progress:
            progress(row)
        return row

    def stop_all(self, progress=None, *, recover=True, request_stop=True):
        rows = []
        for name in STOP_ORDER:
            if not self.processes.get(name, {}).get('running'):
                row = {'component': name, 'ok': True, 'state': 'skipped', 'message': 'Already stopped'}
            else:
                try:
                    row = self.stop(name, progress, cascade=False, request_stop=request_stop)
                except Exception as exc:
                    row = {'component': name, 'ok': False, 'state': 'failed', 'message': str(exc)}
            rows.append(row)
            if progress:
                progress(row)
        return rows


@pytest.fixture
def stack(tmp_path):
    store = Store(tmp_path / 'state')
    motion = MotionLease(store.config)
    gateway = Gateway()
    health = {'ros_ok': True, 'maintenance': False}
    recovered = []
    def recover():
        recovered.append(True)
        health['ros_ok'] = True
    ros = SimpleNamespace(status=lambda: dict(health), navigation=gateway, recover_executor=recover)
    manager = Manager()
    delivery = DeliveryCoordinator(store, ros, motion, manager)
    system = SystemCoordinator(store, ros, motion, manager, delivery)
    delivery.system = system
    yield SimpleNamespace(store=store, motion=motion, gateway=gateway, health=health,
                          recovered=recovered, manager=manager, delivery=delivery, system=system)
    system.close()
    delivery.close()
    store.db.close()


def finish(stack, timeout=3.0):
    deadline = time.monotonic() + timeout
    for worker in list(stack.system.workers):
        worker.join(max(0, deadline - time.monotonic()))
        assert not worker.is_alive()
    return stack.system.snapshot()


def test_three_connect_stop_cycles_reopen_every_component(stack):
    s = stack
    for _ in range(3):
        s.system.begin('connect')
        assert finish(s)['state'] == 'succeeded'
        assert all(s.manager.processes[name]['running'] for name in BASE_ORDER)
        assert s.manager.checked
        s.system.begin('disconnect')
        assert finish(s)['state'] == 'succeeded'
        assert not any(p['running'] for p in s.manager.processes.values())
    assert [name for action, name in s.manager.calls if action == 'start'] == list(BASE_ORDER) * 3
    expected = [name for name in STOP_ORDER if name in BASE_ORDER]
    assert [name for action, name in s.manager.calls if action == 'stop'] == expected * 3


@pytest.mark.parametrize('fault', ['missing_mux', 'dead_executor'])
def test_missing_stop_ack_does_not_prevent_host_teardown(stack, fault):
    s = stack
    s.system.begin('connect'); finish(s)
    if fault == 'missing_mux':
        s.gateway.ack = False
    else:
        s.gateway.error = True
        s.health['ros_ok'] = False
    s.system.begin('disconnect')
    result = finish(s)
    assert result['state'] == 'succeeded'
    assert result['stop']['mux_confirmed'] is False
    assert result['warnings']
    assert not any(p['running'] for p in s.manager.processes.values())
    assert s.gateway.stops == 1  # one bounded request, not once per component


def test_cleanup_reports_failure_and_still_reaches_agent(stack):
    s = stack
    s.system.begin('connect'); finish(s)
    s.manager.fail_stop = 'lidar'
    s.system.begin('disconnect')
    result = finish(s)
    assert result['state'] == 'failed'
    assert s.manager.processes['lidar']['running']
    assert not s.manager.processes['agent']['running']
    assert any(r['component'] == 'lidar' and not r['ok'] for r in result['results'])


def test_cleanup_surfaces_observation_reset_warning(stack, monkeypatch):
    s = stack
    s.system.begin('connect'); finish(s)
    original = s.manager.stop
    def stopped_with_warning(name, *args, **kwargs):
        row = original(name, *args, **kwargs)
        if name == 'lidar':
            row['warning'] = 'Tiến trình đã dừng; chưa làm mới dữ liệu LiDAR.'
        return row
    monkeypatch.setattr(s.manager, 'stop', stopped_with_warning)
    s.system.begin('disconnect')
    result = finish(s)
    assert result['state'] == 'succeeded'
    assert 'Tiến trình đã dừng; chưa làm mới dữ liệu LiDAR.' in result['warnings']


def test_disconnect_preempts_slow_connect_and_never_resurrects_it(stack):
    s = stack
    spawned, release = threading.Event(), threading.Event()
    def slow(name, event):
        if name == 'agent':
            spawned.set()
            assert release.wait(2)
    s.manager.on_start = slow
    first = s.system.begin('connect')['operation']['id']
    assert spawned.wait(1)
    stop = s.system.begin('disconnect')['operation']['id']
    assert stop != first
    release.set()
    assert finish(s)['id'] == stop
    assert not any(p['running'] for p in s.manager.processes.values())
    assert [c for c in s.manager.calls if c[0] == 'start'] == [('start', 'agent')]


def test_failed_connect_rolls_back_new_owners_but_keeps_existing_component(stack):
    s = stack
    s.manager.start('robot_model')
    s.manager.fail_start = 'lidar'
    s.system.begin('connect')
    result = finish(s)
    assert result['state'] == 'failed'
    assert s.manager.processes['robot_model']['running']
    assert not s.manager.processes['agent']['running']
    assert not s.manager.processes['odom_bridge']['running']


def test_component_start_stop_start_is_repeatable(stack):
    s = stack
    for kind in ('component_start', 'component_stop', 'component_start'):
        s.system.begin(kind, 'lidar')
        assert finish(s)['state'] == 'succeeded'
    assert s.manager.processes['lidar']['running']


def test_connect_reuses_owned_stack_without_duplicate_processes(stack):
    s = stack
    s.system.begin('connect'); finish(s)
    s.system.begin('connect'); finish(s)
    assert [c for c in s.manager.calls if c[0] == 'start'] == [('start', n) for n in BASE_ORDER]


def test_explicit_connect_can_recover_executor_only_after_stack_stopped(stack):
    s = stack
    s.health['ros_ok'] = False
    s.manager.start('agent')
    s.system.begin('connect')
    assert finish(s)['state'] == 'failed'
    assert not s.recovered
    s.system.begin('disconnect'); finish(s)
    s.system.begin('connect')
    assert finish(s)['state'] == 'succeeded'
    assert s.recovered == [True]


def test_closing_rviz_does_not_send_motion_stop(stack):
    s = stack
    s.manager.start('rviz')
    s.system.begin('component_stop', 'rviz')
    assert finish(s)['state'] == 'succeeded'
    assert s.gateway.stops == 0


def test_repeated_click_returns_same_pending_operation(stack):
    s = stack
    entered, release = threading.Event(), threading.Event()
    def slow(name, event):
        if name == 'agent':
            entered.set()
            assert release.wait(2)
    s.manager.on_start = slow
    a = s.system.begin('connect')['operation']['id']
    assert entered.wait(1)
    b = s.system.begin('connect')['operation']['id']
    assert a == b
    release.set()
    assert finish(s)['state'] == 'succeeded'


def test_system_operation_blocks_mission_dispatch(stack):
    s = stack
    s.system.operation.update(state='running', kind='connect')
    with pytest.raises(UserError, match='Hệ thống đang xử lý'):
        s.delivery._ensure_dispatch_ready()
