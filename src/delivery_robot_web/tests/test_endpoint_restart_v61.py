"""A verified restart can reuse GIDs; pre-restart callback generations stay dead."""
import threading
from types import SimpleNamespace as S

import pytest

from delivery_robot_web.core import UserError
from test_ros_reliability_v61 import adapter, odometry


def prepared():
    ros = adapter()
    ros.stream_specs = {
        'odom': (object, '/odom', ros.on_odom, 10),
        'scan': (object, '/scan', ros.on_scan, 10),
        'safety': (object, '/robot/safety_status', ros.on_safety, 10),
        'wheels': (object, '/robot/wheel_debug', ros.on_wheels, 10),
        'heading': (object, '/robot/heading_status', ros.on_heading, 10),
        'map': (object, '/map', ros.on_map, 10),
    }
    ros.created, ros.destroyed = [], []
    def create(typ, topic, callback, qos):
        handle = S(topic=topic, callback=callback)
        ros.created.append(handle)
        return handle
    ros.node.create_subscription = create
    ros.node.destroy_subscription = lambda handle: ros.destroyed.append(handle) or True
    ros.stream_subscriptions = {
        name: create(typ, topic, ros.subscription_callback(name, topic, callback), qos)
        for name, (typ, topic, callback, qos) in ros.stream_specs.items()
    }
    ros.created.clear()
    return ros


def queued_restart(ros, components):
    outcome = {}
    def call():
        try:
            outcome.update(ros.managed_start_complete(components))
        except Exception as exc:
            outcome['error'] = exc
    caller = threading.Thread(target=call)
    caller.start()
    task = ros.jobs.get(timeout=1)
    return caller, task, outcome


def test_same_gid_agent_restart_recreates_streams_and_drops_old_queued_callbacks():
    ros = prepared()
    old_callback = ros.stream_subscriptions['odom'].callback
    old_safety = ros.stream_subscriptions['safety'].callback
    endpoint = S(endpoint_gid=b'reused', node_namespace='/', node_name='delivery_robot_motor')
    ros.filter_endpoints('/odom', [endpoint])
    ros.filter_endpoints('/cmd_vel', [endpoint], 'subscription')
    old_callback(odometry(), {'publisher_gid': b'reused'})
    ros.reset_observations(['agent'])
    assert ros.filter_endpoints('/odom', [endpoint]) == []
    caller, task, result = queued_restart(ros, ['agent'])
    task(); caller.join(1)
    assert result == {'resumed': ['agent']}
    assert {entry.topic for entry in ros.created} == {'/odom', '/robot/safety_status', '/robot/wheel_debug', '/robot/heading_status'}
    assert len(ros.destroyed) == 4
    assert ros.odom is None and ros.status()['odom']['receive_age'] is None
    assert ros.filter_endpoints('/odom', [endpoint]) == [endpoint]
    assert ros.filter_endpoints('/cmd_vel', [endpoint], 'subscription') == [endpoint]
    # A callback captured in the old rclpy queue is rejected, including samples
    # lacking optional metadata. Receiving a new identity is not required.
    old_callback(odometry(), {'publisher_gid': b'reused'})
    old_callback(odometry(), {})
    old_safety(S(data=[1.] * 12), {})
    assert ros.odom is None and ros.safety is None
    fresh = odometry()
    ros.stream_subscriptions['odom'].callback(fresh, {'publisher_gid': b'reused'})
    assert ros.odom is fresh and ros.odom_at > 0
    assert ros.status()['odom']['stamp_age'] == 0.0


def test_restart_is_component_scoped_and_does_not_expire_other_retirements():
    ros = prepared()
    motor = S(endpoint_gid=b'motor', node_namespace='/', node_name='delivery_robot_motor')
    lidar = S(endpoint_gid=b'lidar', node_namespace='/', node_name='ldlidar_publisher_ld14')
    ros.filter_endpoints('/odom', [motor]); ros.filter_endpoints('/scan', [lidar])
    ros.reset_observations(['agent', 'lidar'])
    old_scan = ros.stream_subscriptions['scan']
    caller, task, result = queued_restart(ros, ['agent'])
    task(); caller.join(1)
    assert 'error' not in result
    assert ros.stream_subscriptions['scan'] is old_scan
    assert ros.filter_endpoints('/odom', [motor]) == [motor]
    assert ros.filter_endpoints('/scan', [lidar]) == []


def test_no_verified_prior_stop_is_noop_even_when_executor_unavailable():
    ros = prepared(); ros.executor_error = 'dead'
    assert ros.managed_start_complete(['agent']) == {'resumed': []}
    assert not ros.created and not ros.destroyed and ros.jobs.empty()


def test_expired_queued_restart_never_later_unretires_old_stream():
    ros = prepared()
    endpoint = S(endpoint_gid=b'old', node_namespace='/', node_name='delivery_robot_motor')
    ros.filter_endpoints('/odom', [endpoint]); ros.reset_observations(['agent'])
    with pytest.raises(UserError, match='Hết thời gian'):
        ros.managed_start_complete(['agent'], timeout=.05)
    ros.jobs.get_nowait()()
    assert not ros.created and not ros.destroyed
    assert ros.filter_endpoints('/odom', [endpoint]) == []


def test_new_stop_invalidates_queued_restart_generation():
    ros = prepared(); ros.reset_observations(['agent'])
    caller, task, result = queued_restart(ros, ['agent'])
    ros.reset_observations(['agent'])
    task(); caller.join(1)
    assert isinstance(result.get('error'), UserError)
    assert not ros.created and 'agent' in ros.retired_components


def test_failed_subscription_recreation_keeps_endpoint_retired():
    ros = prepared()
    endpoint = S(endpoint_gid=b'old', node_namespace='/', node_name='delivery_robot_motor')
    ros.filter_endpoints('/odom', [endpoint]); ros.reset_observations(['agent'])
    ros.node.create_subscription = lambda *args: (_ for _ in ()).throw(RuntimeError('create failed'))
    caller, task, result = queued_restart(ros, ['agent'])
    task(); caller.join(1)
    assert isinstance(result.get('error'), UserError)
    assert ros.filter_endpoints('/odom', [endpoint]) == []
