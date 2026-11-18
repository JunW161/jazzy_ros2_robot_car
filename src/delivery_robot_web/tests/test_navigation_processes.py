"""Process ownership and launch lifecycle tests, without ROS or hardware."""
from pathlib import Path
from types import SimpleNamespace
import threading
import time

import pytest

from delivery_robot_web.core import Store, MotionLease, UserError
from delivery_robot_web.processes import ProcessManager, NAVIGATION_NODES, STOP_ORDER
from delivery_robot_web import cli


class Child:
    def __init__(self, pid=990001):
        self.pid = pid
        self.returncode = None
    def poll(self):
        return self.returncode
    def wait(self, timeout=None):
        return self.returncode


def prepared(tmp_path, monkeypatch):
    store = Store(tmp_path / 'state')
    ws = tmp_path / 'ros ws'
    for package, launch in (
        ('delivery_robot_nav2', 'navigation_integrated_mux.launch.py'),
        ('delivery_robot_mission', 'mission_manager.launch.py'),
        ('delivery_robot_cmd_mux', 'cmd_mux.launch.py'),
    ):
        file = ws / 'install' / package / 'share' / package / 'launch' / launch
        file.parent.mkdir(parents=True, exist_ok=True)
        file.write_text('# installed launch\n')
    (ws / 'install/setup.bash').write_text('# overlay\n')
    store.config['ros_workspace'] = str(ws)
    state = {
        'nodes': ['/amcl'], 'tf_odom': {'age': 0.02}, 'tf_map': {'age': -0.45},
        'odom_publishers': 1, 'odom': {'receive_age': 0.02},
        'scan_publishers': 1, 'scan': {'receive_age': 0.02}, 'tf_odom_laser': True,
    }
    snapshot = {'ready': False, 'mission': {'available': False},
                'mux': {'available': False, 'mode': 'UNKNOWN', 'receive_age': None}}
    gateway = SimpleNamespace(snapshot=lambda: snapshot,
                              stop_all=lambda: {'mux_confirmed': True})
    ros = SimpleNamespace(status=lambda: state, navigation=gateway, maintenance=False,
                          motion=MotionLease(store.config), end_map_session=lambda: None)
    pm = ProcessManager(store, ros)
    monkeypatch.setattr(pm, '_autonomy_processes', lambda name: [])
    monkeypatch.setattr(pm, 'recoverable_costmap_processes', lambda: [])
    monkeypatch.setattr(pm, 'recoverable_base_processes', lambda name: [])
    monkeypatch.setattr(pm, 'recoverable_localization_processes', lambda: [])
    monkeypatch.setattr(pm, 'udp_in_use', lambda port: False)
    monkeypatch.setattr(pm.agent, 'snapshot', lambda **kw: {
        'running': False, 'installed': False, 'message': ''})
    calls = []
    def spawn(name, command, cwd=None, context=None):
        calls.append((name, command))
        pm.children[name] = {'process': Child(), 'handle': SimpleNamespace(close=lambda: None),
                             'log': tmp_path / (name + '.log'), 'context': dict(context or {}),
                             'started_at': time.monotonic()}
        return {'message': 'started'}
    monkeypatch.setattr(pm, '_spawn_shell', spawn)
    return pm, state, snapshot, calls


def test_navigation_uses_exact_integrated_launch_and_does_not_claim_ready(tmp_path, monkeypatch):
    pm, _, _, calls = prepared(tmp_path, monkeypatch)
    pm.start('navigation')
    assert calls[0][1].splitlines()[-1] == 'exec ros2 launch delivery_robot_nav2 navigation_integrated_mux.launch.py'
    assert "'" in calls[0][1]  # overlay with a space is shell quoted
    item = pm.status()['navigation']
    assert item['running'] and item['managed']
    assert not item['ready'] and item['status'] == 'starting'


def test_navigation_duplicate_owned_start_is_idempotent(tmp_path, monkeypatch):
    pm, _, _, calls = prepared(tmp_path, monkeypatch)
    pm.start('navigation')
    pm.start('navigation')
    assert len(calls) == 1


def test_external_complete_navigation_is_observed_and_never_adopted(tmp_path, monkeypatch):
    pm, state, snap, calls = prepared(tmp_path, monkeypatch)
    state['nodes'] += list(NAVIGATION_NODES) + ['/delivery_robot_cmd_mux']
    snap['ready'] = True
    response = pm.start('navigation')
    assert response['external'] and not response['managed']
    assert not calls and not pm.children
    st = pm.status()['navigation']
    assert st['running'] and st['ready'] and st['external']
    with pytest.raises(UserError, match='ngoài phạm vi'):
        pm.stop('navigation')


def test_partial_or_duplicate_navigation_refuses_new_stack(tmp_path, monkeypatch):
    pm, state, _, calls = prepared(tmp_path, monkeypatch)
    state['nodes'] += ['/controller_server']
    with pytest.raises(UserError, match='một phần'):
        pm.start('navigation')
    state['nodes'] += list(NAVIGATION_NODES) + ['/delivery_robot_cmd_mux']
    with pytest.raises(UserError, match='một phần'):
        pm.start('navigation')
    assert not calls


def test_standalone_costmap_and_stale_foundation_prevent_navigation(tmp_path, monkeypatch):
    pm, state, _, calls = prepared(tmp_path, monkeypatch)
    state['nodes'].append('/local_costmap/local_costmap')
    with pytest.raises(UserError, match='Costmap riêng'):
        pm.start('navigation')
    state['nodes'].remove('/local_costmap/local_costmap')
    state['tf_map']['age'] = 25.0
    with pytest.raises(UserError, match='TF'):
        pm.start('navigation')
    assert not calls


def test_mission_and_standalone_mux_launch_fixed_commands(tmp_path, monkeypatch):
    pm, state, _, calls = prepared(tmp_path, monkeypatch)
    pm.start('cmd_mux')
    assert calls[-1][1].splitlines()[-1] == 'exec ros2 launch delivery_robot_cmd_mux cmd_mux.launch.py initial_mode:=STOP'
    assert not pm.navigation_present()
    with pytest.raises(UserError, match='Navigation trước'):
        pm.start('mission')
    state['nodes'] += list(NAVIGATION_NODES) + ['/delivery_robot_cmd_mux']
    pm.start('mission')
    assert calls[-1][1].splitlines()[-1] == 'exec ros2 launch delivery_robot_mission mission_manager.launch.py'


def test_external_standalone_mux_is_not_killed_for_navigation(tmp_path, monkeypatch):
    pm, state, _, calls = prepared(tmp_path, monkeypatch)
    state['nodes'].append('/delivery_robot_cmd_mux')
    with pytest.raises(UserError, match='mux đang chạy ngoài Web'):
        pm.start('navigation')
    assert not calls


def test_owned_mux_handover_requires_stop_ack(tmp_path, monkeypatch):
    pm, state, _, calls = prepared(tmp_path, monkeypatch)
    pm.start('cmd_mux')
    state['nodes'].append('/delivery_robot_cmd_mux')
    pm.ros.navigation.stop_all = lambda: {'mux_confirmed': False}
    with pytest.raises(UserError, match='xác nhận STOP'):
        pm.start('navigation')
    assert pm.children['cmd_mux']['process'].poll() is None
    assert [name for name, _ in calls] == ['cmd_mux']


def test_owned_mux_handover_waits_until_old_endpoint_disappears(tmp_path, monkeypatch):
    pm, state, _, calls = prepared(tmp_path, monkeypatch)
    pm.start('cmd_mux')
    state['nodes'].append('/delivery_robot_cmd_mux')
    def stop_owned(name, **kwargs):
        assert name == 'cmd_mux'
        assert kwargs == {'cascade': False, 'request_stop': False}
        pm.children[name]['process'].returncode = 0
        state['nodes'].remove('/delivery_robot_cmd_mux')
        return {'message': 'stopped'}
    monkeypatch.setattr(pm, 'stop', stop_owned)
    pm.start('navigation')
    assert [name for name, _ in calls] == ['cmd_mux', 'navigation']


def test_navigation_protects_foundation_and_map_mutations(tmp_path, monkeypatch):
    pm, state, _, _ = prepared(tmp_path, monkeypatch)
    state['nodes'].append('/controller_server')
    for component in ('agent', 'lidar', 'ekf_odom', 'localization', 'costmap'):
        with pytest.raises(UserError, match='Dừng Navigation'):
            pm.stop(component)
    with pytest.raises(UserError, match='Dừng Navigation'):
        pm.start('slam')
    with pytest.raises(UserError, match='Dừng Navigation'):
        pm.start_localization(tmp_path / 'map.yaml', 'id', 'Map')


def test_system_stop_order_only_owned_children(tmp_path, monkeypatch):
    pm, _, _, _ = prepared(tmp_path, monkeypatch)
    calls = []
    monkeypatch.setattr(pm, 'status', lambda: {
        name: {'running': True, 'managed': name != 'localization', 'external': name == 'localization'}
        for name in STOP_ORDER
    })
    for name in STOP_ORDER:
        if name not in ('agent', 'localization'):
            pm.children[name] = {'process': Child(), 'handle': SimpleNamespace(close=lambda: None)}
    pm.agent_started = True
    monkeypatch.setattr(pm, '_stop_one', lambda name, **kw: calls.append(name) or {'message': 'stopped'})
    result = pm.stop_all()
    assert calls[:3] == ['mission', 'navigation', 'cmd_mux']
    assert 'localization' not in calls
    assert calls[-1] == 'agent'
    assert next(row for row in result if row['component'] == 'localization')['state'] == 'external'


def test_confirmed_owned_shutdown_retires_gateway_and_records_interruption(tmp_path, monkeypatch):
    pm, state, _, _ = prepared(tmp_path, monkeypatch)
    state['nodes'] += list(NAVIGATION_NODES) + ['/delivery_robot_cmd_mux']
    pm.start('mission')
    events = []
    mission = {'id': 'delivery-7', 'state_name': 'ERROR', 'active': False}
    def stop_child(process):
        events.append('process_stopped')
        process.returncode = 0
    def retire(components):
        assert pm.children['mission']['process'].poll() == 0
        events.append(tuple(components))
        return {'mission': mission}
    monkeypatch.setattr(pm, '_stop_owned_child', stop_child)
    pm.ros.navigation.managed_shutdown_complete = retire
    monkeypatch.setattr(pm.store, 'interrupt_delivery', lambda record: events.append(record))
    result = pm.stop('mission')
    assert result['managed']
    assert events == ['process_stopped', ('mission',), mission]


def test_unconfirmed_process_shutdown_does_not_retire_gateway(tmp_path, monkeypatch):
    pm, _, _, _ = prepared(tmp_path, monkeypatch)
    pm.start('cmd_mux')
    events = []
    pm.ros.navigation.managed_shutdown_complete = lambda components: events.append(components)
    def fail(_process):
        raise UserError('group still running')
    monkeypatch.setattr(pm, '_stop_owned_child', fail)
    with pytest.raises(UserError, match='still running'):
        pm.stop('cmd_mux')
    assert not events


def test_shutdown_and_connect_do_not_kill_external_processes(tmp_path, monkeypatch):
    pm, _, _, _ = prepared(tmp_path, monkeypatch)
    monkeypatch.setattr(pm, 'recoverable_base_processes', lambda name: [{'pid': 8765, 'cmdline': name}])
    stopped = []
    monkeypatch.setattr(pm, '_stop_exact_processes', lambda *args: stopped.append(args))
    assert pm.cleanup_stale_project_stack() == []
    with pytest.raises(UserError, match='ngoài phiên Web'):
        pm.prepare_base_stack_for_connect()
    pm.close()
    assert stopped == []


def test_serve_startup_is_observational():
    import inspect
    body = inspect.getsource(cli.main)
    startup = body[:body.index('uvicorn.run(')]
    assert 'cleanup_stale_project_stack()' not in startup
    assert 'manager.start(' not in startup
    assert 'gateway.stop_all()' in body


def test_start_reservation_does_not_block_status_or_allow_duplicate(tmp_path, monkeypatch):
    pm, _, _, _ = prepared(tmp_path, monkeypatch)
    entered, release = threading.Event(), threading.Event()
    def slow_spawn(*args, **kwargs):
        entered.set()
        assert release.wait(2)
        return {'message': 'started'}
    monkeypatch.setattr(pm, '_spawn_shell', slow_spawn)
    thread = threading.Thread(target=lambda: pm.start('navigation'))
    thread.start()
    assert entered.wait(1)
    before = time.monotonic()
    assert pm.status()['navigation']['status'] == 'starting'
    assert time.monotonic() - before < 0.25
    with pytest.raises(UserError, match='Đang xử lý'):
        pm.start('navigation')
    release.set()
    thread.join(timeout=2)
    assert not thread.is_alive()


def test_close_prevents_a_late_background_process_spawn(tmp_path, monkeypatch):
    pm, _, _, _ = prepared(tmp_path, monkeypatch)
    pm.close()
    with pytest.raises(UserError, match='Web đang đóng'):
        pm.start('navigation')
    monkeypatch.setattr('delivery_robot_web.processes.subprocess.Popen',
                        lambda *a, **k: (_ for _ in ()).throw(AssertionError('No new child during shutdown')))
    with pytest.raises(UserError, match='Web đang đóng'):
        ProcessManager._spawn_shell(pm, 'navigation', 'unused')


def test_stop_owned_group_handles_launch_parent_that_already_exited(tmp_path, monkeypatch):
    pm, _, _, _ = prepared(tmp_path, monkeypatch)
    child = Child()
    child.returncode = 0
    alive = {'value': True}
    signals = []
    monkeypatch.setattr(pm, '_owned_group_running', lambda pgid: alive['value'])
    def kill_group(pgid, sig):
        signals.append((pgid, sig))
        alive['value'] = False
    monkeypatch.setattr('delivery_robot_web.processes.os.killpg', kill_group)
    pm._stop_owned_child(child)
    assert len(signals) == 1 and signals[0][0] == child.pid


@pytest.mark.parametrize('name', ['navigation', 'mission', 'cmd_mux'])
def test_owned_orphan_group_stays_managed_and_blocks_duplicate_start(name, tmp_path, monkeypatch):
    pm, state, snapshot, calls = prepared(tmp_path, monkeypatch)
    process = Child()
    process.returncode = 1
    pm.children[name] = {'process': process, 'handle': SimpleNamespace(close=lambda: None),
                         'log': tmp_path / (name + '.log')}
    expected = (list(NAVIGATION_NODES) + ['/delivery_robot_cmd_mux'] if name == 'navigation' else
                ['/delivery_robot_mission_manager'] if name == 'mission' else ['/delivery_robot_cmd_mux'])
    state['nodes'] += expected
    snapshot.update(ready=True, mission={'available': True, 'receive_age': 0.1},
                    mux={'available': True, 'receive_age': 0.1})
    monkeypatch.setattr(pm, '_owned_group_running', lambda pgid: pgid == process.pid)
    observed = pm.status()[name]
    assert observed['running'] and observed['managed'] and not observed['external']
    assert not observed['ready'] and observed['status'] == 'failed'
    assert observed['pid'] == process.pid and observed['returncode'] == 1
    with pytest.raises(UserError, match='tiến trình con do Web sở hữu'):
        pm.start(name)
    assert not calls
    stopped = []
    monkeypatch.setattr(pm, '_stop_one', lambda component, **kw: stopped.append(component) or {'message': 'stopped'})
    pm.stop_all()
    assert stopped == [name]


def test_readiness_is_not_inferred_from_elapsed_startup_time(tmp_path, monkeypatch):
    pm, _, snap, _ = prepared(tmp_path, monkeypatch)
    pm.start('navigation')
    pm.children['navigation']['started_at'] = time.monotonic() - 45
    st = pm.status()['navigation']
    assert st['status'] == 'degraded' and not st['ready']
    snap['ready'] = True
    st = pm.status()['navigation']
    assert st['status'] == 'ready' and st['ready']


def test_canceled_start_does_not_touch_existing_mux(tmp_path, monkeypatch):
    pm, _, _, calls = prepared(tmp_path, monkeypatch)
    event = threading.Event()
    event.set()
    with pytest.raises(UserError, match='bị hủy'):
        pm.start('navigation', cancel_event=event)
    assert calls == [] and not pm.children


def test_cancel_during_mux_ack_keeps_owned_mux_and_does_not_spawn_nav(tmp_path, monkeypatch):
    pm, state, _, calls = prepared(tmp_path, monkeypatch)
    pm.start('cmd_mux')
    state['nodes'].append('/delivery_robot_cmd_mux')
    event = threading.Event()
    def acknowledge():
        event.set()
        return {'mux_confirmed': True}
    pm.ros.navigation.stop_all = acknowledge
    with pytest.raises(UserError, match='bị hủy'):
        pm.start('navigation', cancel_event=event)
    assert pm.children['cmd_mux']['process'].poll() is None
    assert [name for name, _ in calls] == ['cmd_mux']


def test_spawn_checks_cancel_again_after_handover(tmp_path, monkeypatch):
    pm, _, _, _ = prepared(tmp_path, monkeypatch)
    event = threading.Event()
    monkeypatch.setattr(pm, '_handover_standalone_mux', lambda cancel: event.set())
    monkeypatch.setattr(pm, '_spawn_shell', ProcessManager._spawn_shell.__get__(pm, ProcessManager))
    monkeypatch.setattr('delivery_robot_web.processes.subprocess.Popen',
                        lambda *a, **k: (_ for _ in ()).throw(AssertionError('Late canceled spawn')))
    with pytest.raises(UserError, match='bị hủy'):
        pm.start('navigation', cancel_event=event)
    assert not pm.children


def test_owned_costmap_stop_retires_ghost_discovery_before_handoff(tmp_path, monkeypatch):
    pm, state, _, _ = prepared(tmp_path, monkeypatch)
    state['nodes'] += ['/global_costmap/global_costmap', '/local_costmap/local_costmap']
    process = Child()
    pm.children['costmap'] = {'process': process, 'handle': SimpleNamespace(close=lambda: None)}
    monkeypatch.setattr(pm, '_stop_owned_child', lambda child: setattr(child, 'returncode', 0))
    waits = []
    def discover_departure(seconds):
        waits.append(seconds)
        state['nodes'] = ['/amcl']
    monkeypatch.setattr('delivery_robot_web.processes.time.sleep', discover_departure)
    pm.stop('costmap')
    assert not waits  # Process death, not cached DDS names, confirms teardown.
    assert pm.status()['costmap']['running'] is False
    assert pm._state_for_process_checks()['nodes'] == ['/amcl']
