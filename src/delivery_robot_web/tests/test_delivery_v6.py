"""HTTP and persistence regression tests; no ROS installation or motors required."""
import math
import threading
import time
from copy import deepcopy
from types import SimpleNamespace as NS

import pytest
from fastapi.testclient import TestClient
from PIL import Image

from delivery_robot_web.app import create_app
from delivery_robot_web.core import MotionLease, Store, UserError
from delivery_robot_web.delivery import DeliveryCoordinator
from delivery_robot_web.maps import import_candidates, import_existing


class FakeNavigation:
    def __init__(self):
        self.calls = []
        self.epoch = 0
        self.s = {'offline': False, 'ready': True, 'issues': [], 'lifecycle': {},
                  'mission': {'available': True, 'id': '', 'active': False,
                              'state_name': 'IDLE', 'receive_age': 0.1},
                  'goal': {'state': 'IDLE', 'active': False},
                  'mux': {'available': True, 'mode': 'STOP', 'output_topic': '/cmd_vel'},
                  'plan': {'frame_id': 'map', 'points': []}}

    def snapshot(self):
        return deepcopy(self.s)

    def command_epoch(self):
        return self.epoch

    def _check_dispatch(self, expected_epoch=None, dispatch_check=None):
        if expected_epoch is not None and expected_epoch != self.epoch:
            raise UserError('Yêu cầu đã bị hủy bởi lệnh dừng.')
        if dispatch_check is not None:
            dispatch_check()

    def start_delivery(self, pickup, dropoff, **intent):
        self._check_dispatch(**intent)
        self.epoch += 1
        self.calls.append(('start_delivery', pickup, dropoff))
        self.s['mission'].update(id='mission-test-1', active=True,
                                 state_name='GOING_TO_PICKUP', message='Moving to pickup')
        return {'accepted': True, 'mission_id': 'mission-test-1', 'message': 'Accepted'}

    def navigate_to(self, point, **intent):
        self._check_dispatch(**intent)
        self.epoch += 1
        self.calls.append(('navigate_to', point))
        self.s['goal'].update(state='ACTIVE', active=True)
        return {'accepted': True, 'goal_id': 'goal-1'}

    def mission_action(self, action, **intent):
        self._check_dispatch(**intent)
        self.calls.append((action,))
        self.s['mission'].update(state_name='CANCELED', active=False, message='Canceled')
        return {'success': True, 'message': action}

    def cancel_navigation(self):
        self.calls.append(('cancel_navigation',))
        self.s['goal'].update(state='CANCELED', active=False)
        return {'cancel_confirmed': True, 'mux_confirmed': True}

    def set_manual(self, **intent):
        self._check_dispatch(**intent)
        self.epoch += 1
        self.calls.append(('set_manual',))
        self.s['mux']['mode'] = 'MANUAL'
        return {'success': True}

    def manual_authority(self):
        return self.s['mux']['mode'] == 'MANUAL'

    def stop_all(self):
        self.epoch += 1
        self.calls.append(('stop_all',))
        self.s['goal'].update(state='CANCELED', active=False)
        self.s['mission'].update(state_name='CANCELED', active=False)
        self.s['mux']['mode'] = 'STOP'
        return {'mux_confirmed': True, 'cancel_confirmed': True, 'message': 'STOP acknowledged'}


class FakeRos:
    def __init__(self, map_id):
        self.navigation = FakeNavigation()
        self.maintenance = False
        self.revision = 1
        self.message = NS(header=NS(frame_id='map'), info=NS(width=8, height=6, resolution=0.1,
            origin=NS(position=NS(x=0.0, y=0.0), orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0))),
            data=[0] * 48)

    def status(self):
        return {'maintenance': self.maintenance, 'mode': 'TEST', 'motor': {'state': 1},
                'slam_state': 'unknown', 'motion_ready': True, 'motion_issues': []}

    def motion_ready(self): return True
    def current_ros_map(self): return self.revision, self.message, 0.1


class FakeManager:
    offline = False

    def __init__(self, map_id):
        self.processes = {'localization': {'running': True, 'external': False,
                                          'context': {'map_id': map_id}},
                          'navigation': {'running': True, 'external': False},
                          'mission': {'running': True, 'external': False}}
        self.calls = []

    def status(self): return deepcopy(self.processes)
    def start(self, name, cancel_event=None):
        self.calls.append(('start', name))
        self.processes[name] = {'running': True, 'managed': True, 'external': False,
                                'pid': 10000 + len(self.calls)}
        return {'message': name}
    def stop(self, name, progress=None, *, cascade=True, request_stop=True):
        self.calls.append(('stop', name))
        self.processes[name] = {'running': False, 'external': False}
        if progress:
            progress({'component': name, 'state': 'succeeded', 'ok': True})
        return {'message': name}
    def log(self, name): return 'test'
    def stop_all(self, progress=None, *, recover=True, request_stop=True): return []


@pytest.fixture
def system(tmp_path):
    store = Store(tmp_path / 'state')
    store.set_admin('admin', 'valid-test-password-v6')
    source = tmp_path / 'map_source'
    source.mkdir()
    Image.new('L', (8, 6), 254).save(source / 'map.pgm')
    (source / 'map.yaml').write_text(
        'image: map.pgm\nresolution: 0.1\norigin: [0.0, 0.0, 0.0]\n'
        'occupied_thresh: 0.65\nfree_thresh: 0.25\nnegate: 0\n')
    store.config['import_root'] = str(source)
    record = import_existing(store, import_candidates(source)[0]['key'], 'Test map')
    a = store.save_point(record['id'], {'name': 'Kho', 'kind': 'pickup', 'x': 0.2, 'y': 0.2, 'yaw': 0})
    b = store.save_point(record['id'], {'name': 'Bàn A', 'kind': 'delivery', 'x': 0.6, 'y': 0.3, 'yaw': math.pi/2})
    motion = MotionLease(store.config)
    ros = FakeRos(record['id'])
    manager = FakeManager(record['id'])
    app = create_app(store, ros, motion, manager)
    with TestClient(app, base_url='http://127.0.0.1:8080') as client:
        login = client.post('/api/login', headers={'Origin': 'http://127.0.0.1:8080'},
                            json={'username': 'admin', 'password': 'valid-test-password-v6'})
        headers = {'Origin': 'http://127.0.0.1:8080', 'X-CSRF-Token': login.json()['csrf']}
        yield NS(store=store, ros=ros, motion=motion, manager=manager, app=app,
                 client=client, headers=headers, record=record, pickup=a, dropoff=b)
    store.db.close()


def payload(s):
    return {'map_id': s.record['id'], 'pickup_id': s.pickup, 'dropoff_id': s.dropoff}


def test_delivery_persists_real_transitions_and_orientation(system):
    s = system
    result = s.client.post('/api/missions/start', json=payload(s), headers=s.headers)
    assert result.status_code == 200 and result.json()['accepted']
    assert s.ros.navigation.calls[-1][2]['yaw'] == pytest.approx(math.pi/2)
    s.ros.navigation.s['mission'].update(state_name='WAITING_PICKUP', message='Waiting for load')
    history = s.client.get('/api/missions/history').json()['items']
    assert history[0]['state_name'] == 'WAITING_PICKUP'
    s.ros.navigation.s['mission'].update(state_name='CANCELED', active=False, message='Operator canceled')
    assert s.client.get('/api/missions/history').json()['items'][0]['state_name'] == 'CANCELED'
    assert all(r['state_name'] != 'COMPLETED' for r in s.store.delivery_history())


def test_map_identity_checks_content_not_just_dimensions(system):
    s = system
    assert s.client.get('/api/navigation/map-match/'+s.record['id']).json()['matches']
    s.ros.message = deepcopy(s.ros.message)
    s.ros.message.data[20] = 100
    s.ros.revision += 1
    result = s.client.post('/api/missions/start', json=payload(s), headers=s.headers)
    assert result.status_code == 409 and 'Nội dung' in result.json()['detail']
    assert s.ros.navigation.calls == []


@pytest.mark.parametrize('case', ['wrong_map', 'unknown_point', 'wrong_kind', 'same_point', 'not_ready', 'manual', 'maintenance'])
def test_invalid_delivery_never_dispatches(system, case):
    s = system
    data = payload(s)
    if case == 'wrong_map': s.manager.processes['localization']['context']['map_id'] = 'other-map'
    if case == 'unknown_point': data['pickup_id'] = 'unknown'
    if case == 'wrong_kind': data['pickup_id'], data['dropoff_id'] = data['dropoff_id'], data['pickup_id']
    if case == 'same_point': data['dropoff_id'] = data['pickup_id']
    if case == 'not_ready': s.ros.navigation.s.update(ready=False, issues=['Missing TF'])
    if case == 'manual': s.motion.arm('test', True)
    if case == 'maintenance': s.ros.maintenance = True
    result = s.client.post('/api/missions/start', json=data, headers=s.headers)
    assert result.status_code == 409
    assert s.ros.navigation.calls == []


def test_cross_origin_cannot_create_task(system):
    s = system
    headers = dict(s.headers, Origin='http://untrusted.invalid')
    assert s.client.post('/api/missions/start', json=payload(s), headers=headers).status_code == 403
    assert s.ros.navigation.calls == []


def test_second_goal_does_not_preempt_mission(system):
    s = system
    assert s.client.post('/api/missions/start', json=payload(s), headers=s.headers).status_code == 200
    response = s.client.post('/api/navigation/goal',
        json={'map_id': s.record['id'], 'point_id': s.dropoff}, headers=s.headers)
    assert response.status_code == 409 and len(s.ros.navigation.calls) == 1


def test_global_stop_reaches_mux_and_cancels_tasks(system):
    s = system
    s.motion.arm('test', True)
    response = s.client.post('/api/control/stop', json={}, headers=s.headers)
    assert response.status_code == 200 and response.json()['mux_confirmed']
    assert not s.motion.status()['armed']
    assert ('stop_all',) in s.ros.navigation.calls


def test_unacknowledged_stop_is_not_reported_as_success(system):
    s = system
    s.ros.navigation.stop_all = lambda: {'mux_confirmed': False, 'message': 'No acknowledgement'}
    result = s.client.post('/api/control/stop', json={}, headers=s.headers)
    assert result.status_code == 200 and result.json()['ok'] is False


def test_manual_arm_switches_mode_before_releasing_lease(system):
    s = system
    response = s.client.post('/api/control/arm', json={'watchdog_tested': True, 'clear_area': True}, headers=s.headers)
    assert response.status_code == 200
    assert s.ros.navigation.calls == [('set_manual',)]
    assert s.motion.status()['armed']


def test_process_start_navigation_uses_background_coordinator(system):
    s = system
    response = s.client.post('/api/process/navigation/start', json={}, headers=s.headers)
    assert response.status_code == 200
    assert response.json()['operation']['state'] == 'running'
    s.app.state.delivery.thread.join(timeout=2)
    assert s.app.state.delivery.operation_snapshot()['state'] == 'succeeded'


def test_emergency_stop_invalidates_start_wait_without_dispatch(system):
    s = system
    s.ros.navigation.s['ready'] = False
    s.app.state.delivery.begin('start')
    s.app.state.delivery.emergency_stop()
    s.app.state.delivery.thread.join(timeout=2)
    assert not s.app.state.delivery.thread.is_alive()
    assert s.app.state.delivery.operation_snapshot()['state'] == 'failed'
    assert not any(call[0] in ('start_delivery', 'navigate_to') for call in s.ros.navigation.calls)


@pytest.mark.parametrize('barrier', ['external_goal_active', 'command_pending'])
def test_external_or_unresolved_command_blocks_another_delivery(system, barrier):
    s = system
    s.ros.navigation.s[barrier] = True
    response = s.client.post('/api/missions/start', json=payload(s), headers=s.headers)
    assert response.status_code == 409
    assert not s.ros.navigation.calls


def test_stop_during_spawn_rolls_back_only_new_owned_navigation(system, monkeypatch):
    s = system
    s.manager.processes['navigation'] = {'running': False}
    s.manager.processes['mission'] = {'running': False}
    spawned, release = threading.Event(), threading.Event()
    original_start = s.manager.start

    def slow_start(name, cancel_event=None):
        result = original_start(name, cancel_event)
        spawned.set()
        assert release.wait(2.0)
        return result

    monkeypatch.setattr(s.manager, 'start', slow_start)
    coordinator = s.app.state.delivery
    coordinator.begin('start')
    assert spawned.wait(2.0)
    try:
        coordinator.emergency_stop()
    finally:
        release.set()
    coordinator.thread.join(3.0)
    assert not coordinator.thread.is_alive()
    assert s.manager.calls == [('start', 'navigation'), ('stop', 'navigation')]
    assert s.manager.processes['localization']['running']
    assert coordinator.operation_snapshot()['state'] == 'failed'


def test_disconnect_cleans_owned_processes_even_if_ros_stop_fails(system, monkeypatch):
    s = system
    closed = []

    def unavailable():
        raise RuntimeError('ROS disconnected')

    monkeypatch.setattr(s.ros.navigation, 'stop_all', unavailable)
    monkeypatch.setattr(s.manager, 'stop_all', lambda: closed.append(True) or [])
    result = s.app.state.delivery.disconnect()
    assert closed == [True]
    assert result['stop']['mux_confirmed'] is False
    assert not s.motion.status()['armed']


def test_old_start_rollback_cannot_stop_new_process_with_same_name(system, monkeypatch):
    s = system
    coordinator = s.app.state.delivery
    s.manager.processes['navigation'] = {'running': False}
    s.manager.processes['mission'] = {'running': False}
    s.ros.navigation.s['ready'] = False
    reached_wait, release = threading.Event(), threading.Event()
    original_snapshot = s.ros.navigation.snapshot

    def blocked_snapshot():
        if threading.current_thread().name == 'navigation-operation':
            reached_wait.set()
            assert release.wait(2.0)
        return original_snapshot()

    monkeypatch.setattr(s.ros.navigation, 'snapshot', blocked_snapshot)
    coordinator.begin('start')
    assert reached_wait.wait(2.0)
    try:
        with coordinator.process_lock:
            # Model a newer owned stack after a superseding stop/start operation.
            for name in ('navigation', 'mission'):
                s.manager.processes[name]['pid'] += 100
            coordinator.cancel_event.set()
    finally:
        release.set()
    coordinator.thread.join(3.0)
    assert not coordinator.thread.is_alive()
    assert s.manager.calls == [('start', 'navigation'), ('start', 'mission')]
    assert not any(call[0] == 'stop_all' for call in s.ros.navigation.calls)


def test_start_failure_after_spawn_cleans_the_acquired_owner(system, monkeypatch):
    s = system
    s.manager.processes['navigation'] = {'running': False}
    s.manager.processes['mission'] = {'running': False}
    original_start = s.manager.start

    def failure_after_spawn(name, cancel_event=None):
        original_start(name, cancel_event)
        raise RuntimeError('launch failed after creating its process group')

    monkeypatch.setattr(s.manager, 'start', failure_after_spawn)
    s.app.state.delivery.begin('start')
    s.app.state.delivery.thread.join(3.0)
    assert s.manager.calls == [('start', 'navigation'), ('stop', 'navigation')]
    assert s.app.state.delivery.operation_snapshot()['state'] == 'failed'


def test_pending_navigation_operation_keeps_component_stop_available(system):
    s = system
    s.app.state.delivery.operation.update(state='running', kind='start')
    response = s.client.post('/api/process/lidar/stop', json={}, headers=s.headers)
    assert response.status_code == 200
    assert response.json()['operation']['kind'] == 'component_stop'
    for worker in s.app.state.system.workers:
        worker.join(2.0)
    assert ('stop', 'lidar') in s.manager.calls


def test_verified_process_shutdown_records_interrupted_task_without_success(system):
    s = system
    s.client.post('/api/missions/start', json=payload(s), headers=s.headers)
    interrupted = {'id': 'mission-test-1', 'state_name': 'ERROR',
                   'message': 'Mission Manager đã đóng; nhiệm vụ bị gián đoạn.'}
    s.store.interrupt_delivery(interrupted)
    assert s.store.delivery_history()[0]['state_name'] == 'ERROR'
    # A late teardown must never replace a known completed delivery.
    s.store.observe_delivery({'id': 'mission-test-1', 'available': True, 'receive_age': 0.0,
                             'state_name': 'COMPLETED', 'message': 'Confirmed receipt'})
    s.store.interrupt_delivery(interrupted)
    assert s.store.delivery_history()[0]['state_name'] == 'COMPLETED'


def test_delivery_history_survives_store_reopen(system):
    s = system
    s.client.post('/api/missions/start', json=payload(s), headers=s.headers)
    other = Store(s.store.root)
    try:
        assert other.delivery_history()[0]['pickup_name'] == 'Kho'
        assert other.has_admin()
        assert len(other.points(s.record['id'])) == 2
    finally:
        other.db.close()


def test_legacy_direct_topic_migrates_without_losing_user_settings(tmp_path):
    import json
    s = Store(tmp_path / 'state')
    before = dict(s.config, cmd_topic='/cmd_vel', max_linear=0.123, lidar_ip='192.168.1.42')
    s.config_path.write_text(json.dumps(before))
    s.db.close()
    reopened = Store(tmp_path / 'state')
    try:
        assert reopened.config['cmd_topic'] == '/cmd_vel_manual'
        assert reopened.config['max_linear'] == 0.123
        assert reopened.config['lidar_ip'] == '192.168.1.42'
    finally:
        reopened.db.close()
