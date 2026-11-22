"""Exercise deployment configuration, namespaces, waypoint tracking and RC control."""

from pathlib import Path
from unittest.mock import Mock

from px4_msgs.msg import VehicleOdometry, VehicleStatus
import pytest
import rclpy

from acsi_music.adrc_deploy import ADRCDeployment, load_config


@pytest.fixture
def deployment(tmp_path, monkeypatch):
    """Construct deployment nodes from YAML using a controlled monotonic clock."""
    clock = [10.0]
    monkeypatch.setattr('acsi_music.adrc_offboard.time.monotonic', lambda: clock[0])
    nodes = []
    rclpy.init()

    def create(yaml_text='', automatic=False):
        path = tmp_path / 'deploy.yaml'
        path.write_text(yaml_text)
        node = ADRCDeployment(load_config(path), automatic=automatic)
        nodes.append(node)
        node.mode_pub = Mock()
        node.setpoint_pub = Mock()
        node.command_pub = Mock()
        return node

    try:
        yield create, clock
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


def odometry(node, sample=1000000, position=(0.0, 0.0, 0.0)):
    """Deliver a NED odometry sample without vehicle status."""
    msg = VehicleOdometry()
    msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
    msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
    msg.timestamp_sample = sample
    msg.position = list(position)
    node.odometry_callback(msg)


def test_user_streams_without_status_and_recovers(deployment):
    """RC mode selection never gates streaming or latches the user node stopped."""
    create, clock = deployment
    node = create()
    assert node.target == [0.0, 0.0, -1.0]
    assert not any(s.topic_name.endswith('/vehicle_status') for s in node.subscriptions)
    odometry(node)
    node.timer_callback()
    clock[0] += 0.01
    odometry(node, sample=1010000)
    node.timer_callback()
    assert node.axes[2].z[0] < 0.0
    assert node.setpoint_pub.publish.call_count == 2
    node.status_callback(VehicleStatus())
    node.timer_callback()
    assert node.setpoint_pub.publish.call_count == 3
    clock[0] += 0.6
    node.timer_callback()
    assert node.setpoint_pub.publish.call_count == 3
    assert not node.stopped
    odometry(node, sample=1610000)
    node.timer_callback()
    assert node.setpoint_pub.publish.call_count == 4
    node.command_pub.publish.assert_not_called()


@pytest.mark.parametrize('namespace, prefix', [
    ('', ''), ('drone3', '/drone3'), ('/drone3/', '/drone3'),
])
def test_topic_namespace(deployment, namespace, prefix):
    """Apply namespace consistently to every PX4 input and output."""
    create, _ = deployment
    node = create(f'namespace: "{namespace}"', automatic=True)
    topics = {p.topic_name for p in node.publishers}
    topics.update(s.topic_name for s in node.subscriptions)
    expected = {
        f'{prefix}/fmu/in/{name}'
        for name in ('offboard_control_mode', 'trajectory_setpoint', 'vehicle_command')
    } | {f'{prefix}/fmu/out/{name}' for name in ('vehicle_odometry', 'vehicle_status')}
    assert expected <= topics


def test_auto_warmup_and_system_id(deployment):
    """Only auto mode requests offboard/arming using the configured PX4 ID."""
    create, clock = deployment
    node = create('namespace: drone3\ncontroller:\n  target_system: 3', automatic=True)
    odometry(node)
    node.timer_callback()
    node.setpoint_pub.publish.assert_not_called()
    node.status_callback(VehicleStatus())
    node.timer_callback()
    clock[0] += 1.1
    odometry(node, sample=2100000)
    node.status_callback(VehicleStatus())
    node.timer_callback()
    assert node.command_pub.publish.call_count == 2
    assert all(c.args[0].target_system == 3 for c in node.command_pub.publish.call_args_list)


def test_waypoint_dwell_and_final_hold(deployment):
    """Execute YAML waypoints in order, require low-speed dwell, and hold the last."""
    create, clock = deployment
    node = create('waypoints: [[0, 0, -1], [1, 0, -1], [1, 1, -1]]\n'
                  'waypoint_hold_time: 0.2')
    odometry(node, position=(0.0, 0.0, -1.0))
    node.odometry.velocity[0] = 0.5
    node.timer_callback()
    assert node.waypoint_since is None
    for index in range(4):
        clock[0] += 0.1
        odometry(node, sample=1100000 + index * 100000, position=(0.0, 0.0, -1.0))
        node.timer_callback()
    assert node.waypoint_index == 1
    assert node.target == [1.0, 0.0, -1.0]
    for index in range(4):
        clock[0] += 0.1
        odometry(node, sample=1500000 + index * 100000, position=(1.0, 0.0, -1.0))
        node.timer_callback()
    assert node.waypoint_index == 2
    assert node.target == [1.0, 1.0, -1.0]
    for index in range(4):
        clock[0] += 0.1
        odometry(node, sample=1900000 + index * 100000, position=(1.0, 1.0, -1.0))
        node.timer_callback()
    assert node.waypoint_index == 2


def test_auto_does_not_advance_before_engagement(deployment):
    """Warmup cannot consume the mission's first waypoint."""
    create, clock = deployment
    node = create('waypoints: [[0, 0, 0], [1, 0, -1]]\nwaypoint_hold_time: 0',
                  automatic=True)
    odometry(node)
    node.status_callback(VehicleStatus())
    node.timer_callback()
    assert node.waypoint_index == 0
    status = VehicleStatus()
    status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
    status.arming_state = VehicleStatus.ARMING_STATE_ARMED
    node.status_callback(status)
    clock[0] += 0.01
    odometry(node, sample=1010000)
    node.timer_callback()
    assert node.waypoint_index == 1


@pytest.mark.parametrize('content', [
    'waypoints: [[0, 0]]', 'waypoints: [[0, 0, .nan]]',
    'waypoints: [[true, 0, -1]]', 'namespace: 3', 'waypoint_tolerance: -1',
    'waypoint_hold_time: .inf', 'controller: {auto_arm: true}',
    'controller: {target_system: 1.5}', 'unknown_setting: 3',
])
def test_invalid_yaml(tmp_path, content):
    """Reject malformed mission settings before constructing a ROS node."""
    path = tmp_path / 'invalid.yaml'
    path.write_text(content)
    with pytest.raises(ValueError):
        load_config(path)


def test_shipped_config():
    """Keep the installed example runnable with the specified default waypoint."""
    path = Path(__file__).resolve().parents[1] / 'config' / 'adrc_deploy.yaml'
    config = load_config(path)
    assert config['namespace'] == ''
    assert config['waypoints'] == [[0.0, 0.0, -1.0]]
