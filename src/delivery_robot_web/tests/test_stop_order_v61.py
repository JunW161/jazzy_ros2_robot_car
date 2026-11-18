"""STOP dominates requests received earlier, including before ROS dispatch."""
import threading
import time

import pytest

from delivery_robot_web.core import UserError
from test_delivery_v6 import system, payload
from test_navigation_protocol import gateway


@pytest.mark.parametrize('operation', ['mission', 'goal'])
def test_stop_during_map_validation_prevents_late_dispatch(system, monkeypatch, operation):
    s = system; coordinator = s.app.state.delivery
    original = coordinator.validate_map
    def stop_during_validation(ident):
        record = original(ident)
        coordinator.emergency_stop()
        return record
    monkeypatch.setattr(coordinator, 'validate_map', stop_during_validation)
    if operation == 'mission':
        response = s.client.post('/api/missions/start', json=payload(s), headers=s.headers)
    else:
        response = s.client.post('/api/navigation/goal',
            json={'map_id': s.record['id'], 'point_id': s.dropoff}, headers=s.headers)
    assert response.status_code == 409
    assert 'hủy' in response.json()['detail']
    assert [c[0] for c in s.ros.navigation.calls] == ['stop_all']
    assert not s.store.delivery_history()


def test_stop_after_manual_mode_reply_never_rearms_lease(system, monkeypatch):
    s = system; coordinator = s.app.state.delivery
    original = s.ros.navigation.set_manual
    def stop_before_arm(**intent):
        result = original(**intent)
        coordinator.emergency_stop()
        return result
    monkeypatch.setattr(s.ros.navigation, 'set_manual', stop_before_arm)
    response = s.client.post('/api/control/arm', headers=s.headers,
        json={'watchdog_tested': True, 'clear_area': True})
    assert response.status_code == 409
    assert not s.motion.status()['armed']
    assert [c[0] for c in s.ros.navigation.calls] == ['set_manual', 'stop_all']


def test_stop_bypasses_blocked_manual_rpc_and_late_reply_does_not_arm(system, monkeypatch):
    s = system
    entered, release = threading.Event(), threading.Event()
    replies = []
    original = s.ros.navigation.set_manual
    def slow_manual(**intent):
        result = original(**intent)
        entered.set()
        assert release.wait(2)
        return result
    monkeypatch.setattr(s.ros.navigation, 'set_manual', slow_manual)
    thread = threading.Thread(target=lambda: replies.append(s.client.post('/api/control/arm',
        headers=s.headers, json={'watchdog_tested': True, 'clear_area': True})))
    thread.start()
    assert entered.wait(1)
    try:
        before = time.monotonic()
        stopped = s.client.post('/api/control/stop', headers=s.headers, json={})
        assert stopped.status_code == 200 and stopped.json()['mux_confirmed']
        assert time.monotonic() - before < .5
    finally:
        release.set(); thread.join(2)
    assert not thread.is_alive()
    assert replies[0].status_code == 409 and not s.motion.status()['armed']


@pytest.mark.parametrize('action', ['confirm_pickup', 'confirm_dropoff', 'reset'])
def test_queued_mission_confirmation_is_canceled_by_stop(system, monkeypatch, action):
    s = system; coordinator = s.app.state.delivery
    captured = threading.Event(); replies = []
    original = coordinator.capture_intent
    def capture():
        value = original(); captured.set(); return value
    monkeypatch.setattr(coordinator, 'capture_intent', capture)
    with coordinator.dispatch_lock:
        thread = threading.Thread(target=lambda: replies.append(s.client.post(
            '/api/missions/' + action, headers=s.headers, json={})))
        thread.start()
        assert captured.wait(1)
        coordinator.emergency_stop()
    thread.join(2)
    assert not thread.is_alive() and replies[0].status_code == 409
    assert [c[0] for c in s.ros.navigation.calls] == ['stop_all']


@pytest.mark.parametrize('route', ['/api/navigation/start', '/api/system/connect',
                                   '/api/process/lidar/start', '/api/rviz/start'])
def test_queued_start_cannot_run_after_stop_completed(system, monkeypatch, route):
    s = system; coordinator = s.app.state.delivery
    captured = threading.Event(); replies = []
    original = s.app.state.system.epoch
    def epoch():
        value = original(); captured.set(); return value
    monkeypatch.setattr(s.app.state.system, 'epoch', epoch)
    with coordinator.dispatch_lock:
        thread = threading.Thread(target=lambda: replies.append(s.client.post(
            route, headers=s.headers, json={})))
        thread.start()
        assert captured.wait(1)
        coordinator.emergency_stop()
    thread.join(2)
    assert not thread.is_alive() and replies[0].status_code == 409
    assert not s.manager.calls


@pytest.mark.parametrize('command', ['manual', 'goal', 'mission', 'confirm_pickup',
                                     'confirm_dropoff', 'reset'])
def test_gateway_rejects_old_expected_epoch_before_reserving_mutation(command):
    nav = gateway(); expected = nav.command_epoch()
    nav.epoch += 1  # STOP already acquired the gateway lock.
    pose = {'name': 'A', 'x': .2, 'y': .3, 'yaw': 0}
    if command == 'manual':
        call = lambda: nav.set_manual(expected_epoch=expected)
    elif command == 'goal':
        call = lambda: nav.navigate_to(pose, expected_epoch=expected)
    elif command == 'mission':
        call = lambda: nav.start_delivery(pose, pose, expected_epoch=expected)
    else:
        nav.mission['state_name'] = 'WAITING_PICKUP' if command == 'confirm_pickup' else 'WAITING_DROPOFF'
        call = lambda: nav.mission_action(command, expected_epoch=expected)
    with pytest.raises(UserError, match='hủy'):
        call()
    assert nav.epoch == expected + 1
    assert not nav.control_pending and nav.goal['state'] == 'IDLE'
    assert not any(client.requests for client in nav.clients.values())


def test_navigation_stop_surfaces_nested_observation_reset_warning(system, monkeypatch):
    s = system
    original = s.manager.stop
    def stopped_with_warning(name, *args, **kwargs):
        result = original(name, *args, **kwargs)
        result['results'] = [{'component': name, 'ok': True,
            'warning': 'Chưa làm mới dữ liệu sau khi dừng ' + name}]
        return result
    monkeypatch.setattr(s.manager, 'stop', stopped_with_warning)
    coordinator = s.app.state.delivery
    coordinator.begin('stop')
    coordinator.thread.join(2)
    result = coordinator.operation_snapshot()
    assert result['state'] == 'succeeded'
    assert 'Chưa làm mới dữ liệu sau khi dừng navigation' in result['warnings']
