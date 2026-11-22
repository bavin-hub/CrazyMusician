"""Check ADRC dynamics and the acceleration-only PX4 interface without a vehicle."""

import math
from unittest.mock import Mock

from px4_msgs.msg import VehicleCommand, VehicleOdometry, VehicleStatus
import pytest
import rclpy

from acsi_music.adrc_offboard import ADRCOffboard, G, saturate_accel, VelocityADRC


@pytest.fixture
def controller(monkeypatch):
    """Use real ROS messages with captured publishers and a controlled clock."""
    clock = [10.0]
    monkeypatch.setattr('acsi_music.adrc_offboard.time.monotonic', lambda: clock[0])
    rclpy.init()
    node = ADRCOffboard()
    node.mode_pub = Mock()
    node.setpoint_pub = Mock()
    node.command_pub = Mock()
    try:
        yield node, clock
    finally:
        node.destroy_node()
        rclpy.shutdown()


def feedback(node, sample=1000000, active=False):
    """Supply valid local NED state and a matching vehicle status."""
    odometry = VehicleOdometry()
    odometry.timestamp_sample = sample
    odometry.pose_frame = VehicleOdometry.POSE_FRAME_NED
    odometry.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
    status = VehicleStatus()
    if active:
        status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        status.arming_state = VehicleStatus.ARMING_STATE_ARMED
    node.odometry_callback(odometry)
    node.status_callback(status)


def test_velocity_observer_and_control():
    """Check simultaneous Euler integration, disturbance cancellation and clipping."""
    axis = VelocityADRC(1.0, 1.5, 8.0, 1.5)
    axis.reset(0.2)
    axis.observe(v_meas=0.1, u_applied=0.5, h=0.01)
    assert axis.z == pytest.approx([0.189, -0.064])
    assert axis.control(0.0, 10.0) == pytest.approx(2.0305)
    assert axis.v_ref == 1.5
    axis.reset(0.0)
    assert axis.control(0.0, 0.0, r_dot=0.3, r_ddot=0.4) == pytest.approx(0.85)


def test_acceleration_limits():
    """Check vertical signs and coupled horizontal magnitude while preserving direction."""
    limit = (math.radians(30.0), 0.8 * G, 0.6 * G)
    acceleration = saturate_accel([30.0, 40.0, 20.0], *limit)
    assert acceleration[2] == pytest.approx(0.6 * G)
    assert math.hypot(*acceleration[:2]) == pytest.approx(0.4 * G * math.tan(limit[0]))
    assert acceleration[0] / acceleration[1] == pytest.approx(0.75)
    assert saturate_accel([0.0, 0.0, -20.0], *limit)[2] == pytest.approx(-0.8 * G)


def test_acceleration_only_contract(controller):
    """Ensure PX4 cannot interpret unused position/velocity defaults as setpoints."""
    node, _ = controller
    feedback(node)
    node.timer_callback()
    mode = node.mode_pub.publish.call_args.args[0]
    setpoint = node.setpoint_pub.publish.call_args.args[0]
    assert mode.acceleration
    assert not any((mode.position, mode.velocity, mode.attitude, mode.body_rate, mode.actuator))
    assert all(math.isnan(v) for v in (*setpoint.position, *setpoint.velocity))
    assert list(setpoint.acceleration) == pytest.approx([0.0, 0.0, -1.2])
    assert mode.timestamp == setpoint.timestamp
    node.command_pub.publish.assert_not_called()


def test_warmup_and_manual_mode(controller):
    """Require a full stream warmup before mode/arm requests; allow manual operation."""
    node, clock = controller
    feedback(node)
    node.timer_callback()
    clock[0] += 1.1
    feedback(node, sample=2100000)
    node.timer_callback()
    commands = [call.args[0] for call in node.command_pub.publish.call_args_list]
    assert [msg.command for msg in commands] == [
        VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
    ]
    node.command_pub.reset_mock()
    node.auto_arm = False
    clock[0] += 1.1
    feedback(node, sample=3200000)
    node.timer_callback()
    node.command_pub.publish.assert_not_called()


def test_observer_sample_timing_and_mode_exit(controller):
    """Avoid duplicate ESO integration and never reengage after leaving offboard."""
    node, clock = controller
    feedback(node, active=True)
    node.timer_callback()
    clock[0] += 0.01
    feedback(node, sample=1010000, active=True)
    node.timer_callback()
    assert node.axes[2].z[0] == pytest.approx(-0.012)
    state = list(node.axes[2].z)
    node.timer_callback()
    assert node.axes[2].z == state
    node.setpoint_pub.reset_mock()
    feedback(node, sample=1020000, active=False)
    node.timer_callback()
    feedback(node, sample=1030000, active=True)
    node.timer_callback()
    assert node.stopped
    node.setpoint_pub.publish.assert_not_called()


def test_missing_invalid_and_stale_feedback(controller):
    """Block startup on invalid data and latch stopped if feedback fails in flight."""
    node, clock = controller
    node.timer_callback()
    node.mode_pub.publish.assert_not_called()
    feedback(node)
    node.odometry.velocity[2] = math.nan
    node.timer_callback()
    node.mode_pub.publish.assert_not_called()
    feedback(node, active=True)
    node.timer_callback()
    node.mode_pub.reset_mock()
    clock[0] += node.timeout + 0.01
    node.timer_callback()
    assert node.stopped
    node.mode_pub.publish.assert_not_called()


def test_slow_status_and_estimator_reset(controller):
    """Allow normal 2 Hz status updates and reset the ESO after estimator resets."""
    node, clock = controller
    feedback(node, active=True)
    node.timer_callback()
    clock[0] += 0.6
    node.odometry_received = clock[0]
    assert node.feedback_valid(clock[0])
    node.axes[2].z[:] = [0.5, 2.0]
    node.odometry.timestamp_sample += 10000
    node.odometry.reset_counter += 1
    node.timer_callback()
    assert node.axes[2].z == [0.0, 0.0]


@pytest.mark.parametrize('field, value', [
    ('pose_frame', VehicleOdometry.POSE_FRAME_UNKNOWN),
    ('pose_frame', VehicleOdometry.POSE_FRAME_FRD),
    ('velocity_frame', VehicleOdometry.VELOCITY_FRAME_UNKNOWN),
    ('velocity_frame', VehicleOdometry.VELOCITY_FRAME_FRD),
    ('velocity_frame', VehicleOdometry.VELOCITY_FRAME_BODY_FRD),
])
def test_reject_non_ned_odometry(controller, field, value):
    """Never interpret body or arbitrarily aligned world velocities as NED."""
    node, _ = controller
    feedback(node)
    setattr(node.odometry, field, value)
    node.timer_callback()
    node.mode_pub.publish.assert_not_called()
    node.setpoint_pub.publish.assert_not_called()


def test_odometry_vectors_drive_control(controller):
    """Use all position and velocity vector components in their NED axis order."""
    node, _ = controller
    feedback(node)
    node.odometry.position = [0.2, -0.4, -1.0]
    node.odometry.velocity = [0.1, -0.2, 0.3]
    node.timer_callback()
    setpoint = node.setpoint_pub.publish.call_args.args[0]
    assert list(setpoint.acceleration) == pytest.approx([-0.45, 0.9, -1.65])
