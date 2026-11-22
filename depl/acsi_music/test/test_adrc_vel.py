"""Check the velocity ESO, PX4 message contract, and separate deployment modes."""

import math
from unittest.mock import Mock

import numpy as np
from px4_msgs.msg import VehicleCommand, VehicleOdometry, VehicleStatus
import pytest
import rclpy

from acsi_music.adrc_vel_config import load_config
from acsi_music.adrc_vel_core import Reference, VelAccADRC
from acsi_music.adrc_vel_deploy import ADRCVelocityDeployment


@pytest.fixture
def deployment(tmp_path, monkeypatch):
    """Create independent velocity nodes with captured publishers and test time."""
    clock = [10.0]
    monkeypatch.setattr('acsi_music.adrc_vel_deploy.time.monotonic', lambda: clock[0])
    nodes = []
    rclpy.init()

    def create(content='', automatic=False):
        path = tmp_path / 'velocity.yaml'
        path.write_text(content)
        node = ADRCVelocityDeployment(load_config(path), automatic=automatic)
        nodes.append(node)
        node.mode_pub = Mock()
        node.setpoint_pub = Mock()
        if automatic:
            node.command_pub = Mock()
        return node

    try:
        yield create, clock
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


def feedback(node, sample=1000000, position=(0.0, 0.0, 0.0), active=False):
    """Supply valid NED odometry, plus status only when the node needs it."""
    msg = VehicleOdometry()
    msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
    msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
    msg.timestamp_sample = sample
    msg.position = list(position)
    node.odometry_callback(msg)
    if node.automatic:
        status = VehicleStatus()
        if active:
            status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
            status.arming_state = VehicleStatus.ARMING_STATE_ARMED
        node.status_callback(status)


def test_eso_and_feedforward():
    """Verify observer integration and disturbance cancellation against hand values."""
    ctrl = VelAccADRC()
    ctrl.reset([0.2, 0.0, 0.0])
    ctrl.u_model[:] = [0.5, 0.0, 0.0]
    ctrl.observe([0.1, 0.0, 0.0], 0.01, True)
    assert ctrl.z1 == pytest.approx([0.189, 0.0, 0.0])
    assert ctrl.z2 == pytest.approx([-0.064, 0.0, 0.0])
    velocity, acceleration = ctrl.control(
        [0.0, 0.0, 0.0], [10.0, 0.0, 0.0], np.zeros(3), [0.2, 0.0, 0.0], 0.5)
    assert velocity == pytest.approx([1.0, 0.0, 0.0])
    assert acceleration == pytest.approx([0.132, 0.0, 0.0])
    assert ctrl.u_model == pytest.approx([1.5918, 0.0, 0.0])
    ctrl.reset([0.0, 0.0, 0.0])
    ctrl.u_model[:] = 5.0
    ctrl.observe([0.0, 0.0, 0.0], 0.01, False)
    assert ctrl.z1 == pytest.approx(np.zeros(3))


def test_smooth_reference():
    """Verify waypoint interpolation and endpoint derivatives."""
    reference = Reference()
    reference.hold([0.0, 0.0, -1.0], 0.0)
    reference.move_to([1.0, 0.0, -1.0], 0.0, 3.0)
    position, velocity, acceleration = reference.sample(1.5)
    assert position == pytest.approx([0.5, 0.0, -1.0])
    assert velocity == pytest.approx([0.625, 0.0, 0.0])
    assert acceleration == pytest.approx(np.zeros(3))
    position, velocity, acceleration = reference.sample(4.0)
    assert position == pytest.approx([1.0, 0.0, -1.0])
    assert velocity == pytest.approx(np.zeros(3))
    assert acceleration == pytest.approx(np.zeros(3))


def test_velocity_mode_with_acceleration_feedforward(deployment):
    """Publish both setpoint vectors with only the velocity mode flag enabled."""
    create, clock = deployment
    node = create('controller: {ff_ramp_time: 0.0}')
    feedback(node)
    node.timer_callback()
    clock[0] += 0.01
    feedback(node, sample=1010000)
    node.timer_callback()
    mode = node.mode_pub.publish.call_args.args[0]
    msg = node.setpoint_pub.publish.call_args.args[0]
    assert mode.velocity
    assert not any((mode.position, mode.acceleration, mode.attitude, mode.body_rate))
    assert all(math.isnan(v) for v in msg.position)
    assert msg.velocity[2] < 0.0
    assert msg.acceleration[2] < 0.0
    assert all(math.isfinite(v) for v in (*msg.velocity, *msg.acceleration))
    assert msg.timestamp == mode.timestamp
    state = node.ctrl.z1.copy()
    node.timer_callback()
    assert node.ctrl.z1 == pytest.approx(state)


@pytest.mark.parametrize('namespace, prefix', [('', ''), ('drone3', '/drone3')])
@pytest.mark.parametrize('automatic', [False, True])
def test_namespace_and_user_has_no_commands(deployment, namespace, prefix, automatic):
    """Resolve all PX4 topics and isolate RC operation from status and commands."""
    create, _ = deployment
    node = create(f'namespace: "{namespace}"', automatic=automatic)
    publishers = {p.topic_name for p in node.publishers}
    subscriptions = {s.topic_name for s in node.subscriptions}
    assert f'{prefix}/fmu/in/trajectory_setpoint' in publishers
    assert f'{prefix}/fmu/in/offboard_control_mode' in publishers
    assert f'{prefix}/fmu/out/vehicle_odometry' in subscriptions
    assert (f'{prefix}/fmu/out/vehicle_status' in subscriptions) == automatic
    assert (f'{prefix}/fmu/in/vehicle_command' in publishers) == automatic


def test_auto_warmup_mode_exit_and_target_id(deployment):
    """Request automatic flight after warmup and stop after the operator exits."""
    create, clock = deployment
    node = create('controller: {target_system: 3}', automatic=True)
    feedback(node)
    node.timer_callback()
    node.command_pub.publish.assert_not_called()
    assert node.control_started is None
    clock[0] += 1.1
    feedback(node, sample=2100000)
    node.timer_callback()
    commands = [c.args[0] for c in node.command_pub.publish.call_args_list]
    assert [msg.command for msg in commands] == [
        VehicleCommand.VEHICLE_CMD_DO_SET_MODE, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
    ]
    assert all(msg.target_system == 3 for msg in commands)
    feedback(node, sample=2110000, active=True)
    node.timer_callback()
    assert node.control_started == clock[0]
    node.setpoint_pub.reset_mock()
    feedback(node, sample=2120000, active=False)
    node.timer_callback()
    assert node.stopped
    node.setpoint_pub.publish.assert_not_called()


def test_user_resumes_after_odometry_loss_without_status(deployment):
    """Pause stale streams, then restart the observer and current waypoint move."""
    create, clock = deployment
    node = create()
    feedback(node)
    node.timer_callback()
    clock[0] += 0.6
    node.timer_callback()
    assert node.setpoint_pub.publish.call_count == 1
    assert not node.stopped
    feedback(node, sample=1600000, position=(0.0, 0.0, -0.3))
    node.timer_callback()
    assert node.setpoint_pub.publish.call_count == 2
    assert node.ctrl.z2 == pytest.approx(np.zeros(3))
    assert node.reference.start == pytest.approx([0.0, 0.0, -0.3])


def test_waypoints_finish_then_hold(deployment):
    """Wait for trajectory completion plus dwell before moving on; hold the final goal."""
    create, clock = deployment
    node = create('waypoints: [[0, 0, -1], [1, 0, -1]]\n'
                  'waypoint_hold_time: 0\ncontroller: {move_time: 0.1}')
    feedback(node, position=(0.0, 0.0, -1.0))
    node.timer_callback()
    assert node.waypoint_index == 0
    clock[0] += 0.2
    feedback(node, sample=1200000, position=(0.0, 0.0, -1.0))
    node.timer_callback()
    assert node.waypoint_index == 1
    assert node.reference.goal == pytest.approx([1.0, 0.0, -1.0])
    clock[0] += 0.2
    feedback(node, sample=1400000, position=(1.0, 0.0, -1.0))
    node.timer_callback()
    assert node.waypoint_index == 1
    assert node.velocity == pytest.approx(np.zeros(3))


@pytest.mark.parametrize('field, value', [
    ('pose_frame', VehicleOdometry.POSE_FRAME_FRD),
    ('velocity_frame', VehicleOdometry.VELOCITY_FRAME_BODY_FRD),
    ('position', [math.nan, 0.0, 0.0]),
    ('velocity', [0.0, math.nan, 0.0]),
])
def test_invalid_feedback(deployment, field, value):
    """Reject incompatible frames and nonfinite measurements before streaming."""
    create, _ = deployment
    node = create()
    feedback(node)
    setattr(node.odometry, field, value)
    node.timer_callback()
    node.setpoint_pub.publish.assert_not_called()


@pytest.mark.parametrize('content', [
    'controller: {wcv_xy: 1.5}', 'controller: {use_feedforward: 1}',
    'controller: {control_rate: 0}', 'controller: {k_vel_z: .nan}',
    'controller: {ff_ramp_time: -1}', 'waypoints: [[0, 0]]',
])
def test_invalid_velocity_settings(deployment, content):
    """Reject incompatible acceleration-controller keys and invalid velocity settings."""
    create, _ = deployment
    with pytest.raises(ValueError):
        create(content)


@pytest.mark.parametrize('automatic', [False, True])
def test_print_only_mode_never_publishes_to_px4(deployment, automatic, monkeypatch):
    """Run the full controller without status or PX4 publishers in either variant."""
    create, clock = deployment
    node = create('enable_test: true\ncontroller: {ff_ramp_time: 0.0}', automatic=automatic)
    assert not any(p.topic_name.endswith((
        '/offboard_control_mode', '/trajectory_setpoint', '/vehicle_command',
    )) for p in node.publishers)
    assert not any(s.topic_name.endswith('/vehicle_status') for s in node.subscriptions)
    logger = Mock()
    monkeypatch.setattr(node, 'get_logger', lambda: logger)
    msg = VehicleOdometry()
    msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
    msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
    msg.timestamp_sample = 1000000
    node.odometry_callback(msg)
    node.timer_callback()
    for index in range(1, 121):
        clock[0] += 0.01
        msg.timestamp_sample = 1000000 + index * 10000
        node.odometry_callback(msg)
        node.timer_callback()
    assert node.status is None
    assert node.velocity[2] < 0.0
    assert node.acceleration[2] != 0.0
    assert logger.info.call_count == 121
    output = logger.info.call_args.args[0]
    assert 'velocity NED [m/s]' in output
    assert 'acceleration feedforward NED [m/s^2]' in output
    node.mode_pub.publish.assert_not_called()
    node.setpoint_pub.publish.assert_not_called()
    if automatic:
        node.publish_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        node.command_pub.publish.assert_not_called()


@pytest.mark.parametrize('value', ['"false"', '1', 'null'])
def test_enable_test_requires_boolean(deployment, value):
    """Reject ambiguous test-mode values rather than accidentally enabling outputs."""
    create, _ = deployment
    with pytest.raises(ValueError, match='enable_test'):
        create(f'enable_test: {value}')
