"""Cascaded position/velocity ADRC with PX4 ROS 2 acceleration setpoints."""

import math
import time

from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand,
    VehicleOdometry, VehicleStatus,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)


G = 9.81


class VelocityADRC:
    """Original velocity ADRC and proportional position loop for one NED axis."""

    def __init__(self, k_pos, wcv, wo, v_max, b0=1.0):
        self.k_pos, self.wcv, self.v_max, self.b0 = k_pos, wcv, v_max, b0
        self.b1, self.b2 = 2.0 * wo, wo ** 2
        self.z = [0.0, 0.0]
        self.v_ref = 0.0

    def reset(self, vel):
        """Initialize the observer from measured velocity."""
        self.z[:] = (vel, 0.0)

    def observe(self, v_meas, u_applied, h):
        """Advance the ESO using the previous saturated acceleration command."""
        z1, z2 = self.z
        error = z1 - v_meas
        self.z[0] = z1 + h * (z2 - self.b1 * error + self.b0 * u_applied)
        self.z[1] = z2 - h * self.b2 * error

    def control(self, p_meas, reference, r_dot=0.0, r_ddot=0.0):
        """Return acceleration from position error and optional feedforward."""
        v_ref = self.k_pos * (reference - p_meas) + r_dot
        self.v_ref = min(max(v_ref, -self.v_max), self.v_max)
        z1, z2 = self.z
        return (self.wcv * (self.v_ref - z1) - z2 + r_ddot) / self.b0


def saturate_accel(acceleration, tilt_max, a_up_max, a_down_max):
    """Preserve the source controller's NED vertical and horizontal limits."""
    result = list(acceleration)
    result[2] = min(max(result[2], -a_up_max), a_down_max)
    xy_max = (G - result[2]) * math.tan(tilt_max)
    xy = math.hypot(result[0], result[1])
    if xy > xy_max:
        result[0] *= xy_max / xy
        result[1] *= xy_max / xy
    return result


class ADRCOffboard(Node):
    """Stream acceleration commands and the PX4 offboard heartbeat."""

    def __init__(self, *, node_name='adrc_offboard', namespace='', user_control=False,
                 parameter_overrides=None):
        super().__init__(node_name, namespace=namespace,
                         parameter_overrides=parameter_overrides)
        self.user_control = user_control
        defaults = {
            'control_rate': 100.0,
            'target_position': [0.0, 0.0, -2.0],
            'yaw': 0.0,
            'kpos_xy': 1.0, 'wcv_xy': 1.5, 'wo_xy': 8.0, 'vmax_xy': 1.5,
            'kpos_z': 1.0, 'wcv_z': 1.5, 'wo_z': 8.0, 'vmax_z': 0.8,
            'tilt_max_deg': 30.0,
            'a_up_max': 0.8 * G, 'a_down_max': 0.6 * G,
            'feedback_timeout': 0.5,
            'status_timeout': 2.0,
            'auto_arm': True,
            'target_system': 1,
            'target_component': 1,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = {name: self.get_parameter(name).value for name in defaults}
        for name in (
            'control_rate', 'kpos_xy', 'wcv_xy', 'wo_xy', 'vmax_xy',
            'kpos_z', 'wcv_z', 'wo_z', 'vmax_z', 'a_up_max',
            'a_down_max', 'feedback_timeout', 'status_timeout',
        ):
            if not math.isfinite(p[name]) or p[name] <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if p['control_rate'] < 20.0:
            raise ValueError('control_rate must be at least 20 Hz')
        if not 0.0 < p['tilt_max_deg'] < 90.0 or p['a_down_max'] >= G:
            raise ValueError('Require 0 < tilt_max_deg < 90 and a_down_max < 9.81')
        if len(p['target_position']) != 3 or not all(
            math.isfinite(v) for v in [*p['target_position'], p['yaw']]
        ):
            raise ValueError('target_position must contain three finite NED coordinates')
        self.target = p['target_position']
        self.yaw = p['yaw']
        self.auto_arm = p['auto_arm']
        self.target_system = p['target_system']
        self.target_component = p['target_component']
        for name in ('target_system', 'target_component'):
            if not 1 <= p[name] <= 255:
                raise ValueError(f'{name} must be between 1 and 255')
        self.timeout = p['feedback_timeout']
        self.status_timeout = p['status_timeout']
        self.limits = (math.radians(p['tilt_max_deg']), p['a_up_max'], p['a_down_max'])
        self.axes = [
            VelocityADRC(p[f'kpos_{axis}'], p[f'wcv_{axis}'],
                         p[f'wo_{axis}'], p[f'vmax_{axis}'])
            for axis in ('xy', 'xy', 'z')
        ]
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )
        self.mode_pub = self.create_publisher(
            OffboardControlMode, 'fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, 'fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, 'fmu/in/vehicle_command', qos)
        self.create_subscription(
            VehicleOdometry, 'fmu/out/vehicle_odometry',
            self.odometry_callback, qos)
        if not self.user_control:
            self.create_subscription(
                VehicleStatus, 'fmu/out/vehicle_status', self.status_callback, qos)
        self.odometry = None
        self.status = None
        self.odometry_received = self.status_received = None
        self.previous_sample = None
        self.last_reset_counter = None
        self.acceleration = [0.0] * 3
        self.stream_started = None
        self.last_request = -math.inf
        self.was_active = False
        self.stopped = False
        self.feedback_lost = False
        self.create_timer(1.0 / p['control_rate'], self.timer_callback)
        self.get_logger().info(
            f'ADRC target NED: {self.target}; auto_arm={self.auto_arm}')

    def odometry_callback(self, msg):
        """Store PX4 odometry; process each sample only once."""
        self.odometry = msg
        self.odometry_received = time.monotonic()

    def status_callback(self, msg):
        """Store PX4 mode and arming feedback."""
        self.status = msg
        self.status_received = time.monotonic()

    def feedback_valid(self, now):
        """Require fresh finite NED odometry and, in auto mode, vehicle status."""
        p = self.odometry
        return (
            p is not None
            and now - self.odometry_received <= self.timeout
            and (self.user_control or (
                self.status is not None
                and now - self.status_received <= self.status_timeout))
            and p.pose_frame == VehicleOdometry.POSE_FRAME_NED
            and p.velocity_frame == VehicleOdometry.VELOCITY_FRAME_NED
            and all(math.isfinite(v) for v in (*p.position, *p.velocity))
        )

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """Use the command interface from the px4_ros_com offboard example."""
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.command = command
        msg.param1, msg.param2 = param1, param2
        msg.target_system = self.target_system
        msg.target_component = self.target_component
        msg.source_system = msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def publish_acceleration(self):
        """Enable only acceleration control; disable unused trajectory fields."""
        timestamp = self.get_clock().now().nanoseconds // 1000
        mode = OffboardControlMode()
        mode.timestamp = timestamp
        mode.acceleration = True
        self.mode_pub.publish(mode)
        msg = TrajectorySetpoint()
        msg.timestamp = timestamp
        msg.position = [math.nan] * 3
        msg.velocity = [math.nan] * 3
        msg.acceleration = self.acceleration
        msg.jerk = [math.nan] * 3
        msg.yaw = self.yaw
        msg.yawspeed = math.nan
        self.setpoint_pub.publish(msg)

    def timer_callback(self):
        """Update ADRC on new feedback and continuously stream its setpoint."""
        if self.stopped:
            return
        now = time.monotonic()
        if not self.feedback_valid(now):
            if self.stream_started is not None and not self.feedback_lost:
                self.get_logger().error('Feedback invalid/stale; stopping offboard stream')
                self.feedback_lost = True
            # Once flying, leave recovery to PX4 rather than rearming on reconnection.
            if self.was_active and not self.user_control:
                self.stopped = True
            self.stream_started = self.previous_sample = None
            return
        self.feedback_lost = False
        active = self.user_control or (
            self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )
        if self.was_active and not active:
            self.stopped = True
            self.get_logger().info('Vehicle left armed offboard mode; stopping controller')
            return
        just_engaged = active and not self.was_active
        self.was_active = active
        if just_engaged:
            self.get_logger().info(
                'ADRC acceleration stream started' if self.user_control
                else 'ADRC acceleration control engaged')
        p = self.odometry
        self.update_target(now, active)
        sample = p.timestamp_sample or p.timestamp
        reset_counter = p.reset_counter
        if sample != self.previous_sample or just_engaged:
            velocity = [float(value) for value in p.velocity]
            dt = (sample - self.previous_sample) * 1e-6 if self.previous_sample else 0.0
            reset = (not active or just_engaged or not 0.0 < dt <= 0.05
                     or reset_counter != self.last_reset_counter)
            for i, axis in enumerate(self.axes):
                if reset:
                    axis.reset(velocity[i])
                else:
                    axis.observe(velocity[i], self.acceleration[i], dt)
            self.previous_sample = sample
            self.last_reset_counter = reset_counter
            command = [
                axis.control(float(position), reference)
                for axis, position, reference in zip(
                    self.axes, p.position, self.target)
            ]
            if not all(math.isfinite(v) for v in command):
                self.stopped = True
                self.get_logger().error('Nonfinite ADRC command; stopping offboard stream')
                return
            self.acceleration = saturate_accel(command, *self.limits)
        self.publish_acceleration()
        if self.stream_started is None:
            self.stream_started = now
        if (not self.user_control and self.auto_arm and not active
                and now - self.stream_started >= 1.0
                and now - self.last_request >= 1.0):
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            if self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.last_request = now

    def update_target(self, now, active):
        """Keep the fixed target; deployment nodes override waypoint progression."""


def main(args=None):
    """Run the acceleration offboard controller."""
    rclpy.init(args=args)
    node = None
    try:
        node = ADRCOffboard()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
