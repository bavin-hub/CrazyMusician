"""Independent ROS 2 velocity ADRC deployment with YAML NED waypoint missions."""

import argparse
import math
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
import numpy as np
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry, VehicleStatus,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args

from acsi_music.adrc_vel_config import load_config
from acsi_music.adrc_vel_core import Reference, VelAccADRC


class ADRCVelocityDeployment(Node):
    """Publish velocity plus acceleration feedforward, with auto or RC operation."""

    def __init__(self, config, *, automatic):
        settings = {
            'control_rate': 100.0, 'yaw': 0.0,
            'kpos_xy': 1.0, 'kpos_z': 1.0,
            'wo_xy': 8.0, 'wo_z': 8.0,
            'k_vel_xy': 1.8, 'k_vel_z': 4.0,
            'vmax_xy': 1.0, 'vmax_z': 0.8, 'tilt_max_deg': 20.0,
            'move_time': 3.0, 'ff_ramp_time': 1.0,
            'use_feedforward': True, 'use_r_ddot': True,
            'feedback_timeout': 0.5, 'status_timeout': 2.0,
            'target_system': 1, 'target_component': 1,
        }
        settings.update(config['controller'])
        for name, value in settings.items():
            if name.startswith('use_'):
                continue
            if not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
            if name not in ('yaw', 'ff_ramp_time') and value <= 0:
                raise ValueError(f'{name} must be positive')
        if settings['control_rate'] < 20.0 or settings['ff_ramp_time'] < 0:
            raise ValueError('control_rate must be >= 20 Hz and ff_ramp_time >= 0')
        if not 0.0 < settings['tilt_max_deg'] < 90.0:
            raise ValueError('tilt_max_deg must be between 0 and 90')
        for name in ('target_system', 'target_component'):
            if not 1 <= settings[name] <= 255:
                raise ValueError(f'{name} must be between 1 and 255')
        super().__init__(
            'adrc_vel_deploy_auto' if automatic else 'adrc_vel_deploy_user',
            namespace=config['namespace'],
        )
        self.automatic = automatic
        self.enable_test = config['enable_test']
        self.manage_flight = automatic and not self.enable_test
        self.settings = settings
        self.waypoints = config['waypoints']
        self.waypoint_index = 0
        self.waypoint_since = None
        self.tolerance = config['waypoint_tolerance']
        self.speed_tolerance = config['waypoint_speed_tolerance']
        self.hold_time = config['waypoint_hold_time']
        self.ctrl = VelAccADRC(
            k_pos=(settings['kpos_xy'],) * 2 + (settings['kpos_z'],),
            wo=(settings['wo_xy'],) * 2 + (settings['wo_z'],),
            k_vel=(settings['k_vel_xy'],) * 2 + (settings['k_vel_z'],),
            v_max=(settings['vmax_xy'],) * 2 + (settings['vmax_z'],),
            tilt_max_deg=settings['tilt_max_deg'],
            use_feedforward=settings['use_feedforward'], use_r_ddot=settings['use_r_ddot'],
        )
        self.reference = Reference()
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )
        self.mode_pub = self.setpoint_pub = None
        if not self.enable_test:
            self.mode_pub = self.create_publisher(
                OffboardControlMode, 'fmu/in/offboard_control_mode', qos)
            self.setpoint_pub = self.create_publisher(
                TrajectorySetpoint, 'fmu/in/trajectory_setpoint', qos)
        self.command_pub = None
        if self.manage_flight:
            self.command_pub = self.create_publisher(
                VehicleCommand, 'fmu/in/vehicle_command', qos)
            self.create_subscription(
                VehicleStatus, 'fmu/out/vehicle_status', self.status_callback, qos)
        self.create_subscription(
            VehicleOdometry, 'fmu/out/vehicle_odometry', self.odometry_callback, qos)
        self.odometry = self.status = None
        self.odometry_received = self.status_received = None
        self.previous_sample = self.last_reset_counter = None
        self.stream_started = self.control_started = None
        self.last_request = -math.inf
        self.was_active = self.stopped = self.feedback_lost = False
        self.velocity = self.acceleration = [0.0] * 3
        self.create_timer(1.0 / settings['control_rate'], self.timer_callback)
        self.get_logger().info(
            f'Velocity ADRC: {len(self.waypoints)} waypoint(s); '
            f'automatic={automatic}; enable_test={self.enable_test}')

    def odometry_callback(self, msg):
        """Cache the PX4 NED odometry sample and reception time."""
        self.odometry = msg
        self.odometry_received = time.monotonic()

    def status_callback(self, msg):
        """Cache flight state for automatic operation only."""
        self.status = msg
        self.status_received = time.monotonic()

    def feedback_valid(self, now):
        """Reject stale or non-NED feedback and nonfinite position/velocity."""
        msg = self.odometry
        return (
            msg is not None
            and now - self.odometry_received <= self.settings['feedback_timeout']
            and msg.pose_frame == VehicleOdometry.POSE_FRAME_NED
            and msg.velocity_frame == VehicleOdometry.VELOCITY_FRAME_NED
            and all(math.isfinite(v) for v in (*msg.position, *msg.velocity))
            and (not self.manage_flight or (
                self.status is not None
                and now - self.status_received <= self.settings['status_timeout']))
        )

    def start_move(self, now):
        """Move the reference smoothly to the current mission waypoint."""
        self.reference.move_to(
            self.waypoints[self.waypoint_index], now, self.settings['move_time'])
        self.waypoint_since = None

    def update_waypoint(self, now):
        """Advance after the reference finishes and the aircraft settles at the goal."""
        if self.waypoint_index == len(self.waypoints) - 1:
            return
        arrived = (
            now - self.reference.t0 >= self.reference.duration
            and math.dist(self.odometry.position, self.waypoints[self.waypoint_index])
            <= self.tolerance
            and np.linalg.norm(self.odometry.velocity) <= self.speed_tolerance
        )
        if not arrived:
            self.waypoint_since = None
        else:
            if self.waypoint_since is None:
                self.waypoint_since = now
            if now - self.waypoint_since >= self.hold_time:
                self.waypoint_index += 1
                self.start_move(now)
                self.get_logger().info(
                    f'Waypoint {self.waypoint_index + 1}: '
                    f'{self.waypoints[self.waypoint_index]}')

    def publish_setpoints(self):
        """Use velocity mode and acceleration feedforward in one trajectory message."""
        if self.enable_test:
            velocity = [round(float(v), 4) for v in self.velocity]
            acceleration = [round(float(v), 4) for v in self.acceleration]
            self.get_logger().info(
                f'TEST velocity NED [m/s]: {velocity}; '
                f'acceleration feedforward NED [m/s^2]: {acceleration}')
            return
        timestamp = self.get_clock().now().nanoseconds // 1000
        mode = OffboardControlMode()
        mode.timestamp = timestamp
        mode.velocity = True
        mode.acceleration = False
        self.mode_pub.publish(mode)
        msg = TrajectorySetpoint()
        msg.timestamp = timestamp
        msg.position = [math.nan] * 3
        msg.velocity = [float(v) for v in self.velocity]
        msg.acceleration = [float(v) for v in self.acceleration]
        msg.jerk = [math.nan] * 3
        msg.yaw = float(self.settings['yaw'])
        msg.yawspeed = math.nan
        self.setpoint_pub.publish(msg)

    def publish_command(self, command, param1, param2=0.0):
        """Send a command to the configured PX4 system in automatic mode."""
        if self.enable_test:
            return
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.command = command
        msg.param1, msg.param2 = param1, param2
        msg.target_system = self.settings['target_system']
        msg.target_component = self.settings['target_component']
        msg.source_system = msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def timer_callback(self):
        """Run on new samples and republish the latest command at the configured rate."""
        if self.stopped:
            return
        now = time.monotonic()
        if not self.feedback_valid(now):
            if self.stream_started is not None and not self.feedback_lost:
                self.get_logger().error('Invalid/stale feedback; stopping setpoint stream')
            self.feedback_lost = True
            if self.manage_flight and self.was_active:
                self.stopped = True
            self.stream_started = self.previous_sample = self.control_started = None
            self.waypoint_since = None
            return
        self.feedback_lost = False
        active = not self.manage_flight or (
            self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        if self.manage_flight and self.was_active and not active:
            self.stopped = True
            self.get_logger().info('Left armed offboard mode; stopping velocity controller')
            return
        just_engaged = active and not self.was_active
        self.was_active = active
        msg = self.odometry
        sample = msg.timestamp_sample or msg.timestamp
        if sample != self.previous_sample or just_engaged:
            dt = (sample - self.previous_sample) * 1e-6 if self.previous_sample else 0.0
            reset = (not active or just_engaged or not 0 < dt <= 0.05
                     or msg.reset_counter != self.last_reset_counter)
            if active and self.control_started is None:
                self.reference.hold(msg.position, now)
                self.start_move(now)
                self.control_started = now
            if reset:
                self.ctrl.reset(msg.velocity)
                self.ff_started = now
            else:
                self.ctrl.observe(msg.velocity, dt, active)
            if active:
                self.update_waypoint(now)
                r, rd, rdd = self.reference.sample(now)
                ramp = self.settings['ff_ramp_time']
                scale = min(1.0, (now - self.ff_started) / ramp) if ramp > 0 else 1.0
            else:
                # Stream the mission's first velocity target before mode/arming requests.
                r, rd, rdd = self.waypoints[0], np.zeros(3), np.zeros(3)
                scale = 0.0
            self.velocity, self.acceleration = self.ctrl.control(msg.position, r, rd, rdd, scale)
            self.previous_sample = sample
            self.last_reset_counter = msg.reset_counter
            if not np.all(np.isfinite([self.velocity, self.acceleration])):
                self.stopped = True
                self.get_logger().error('Nonfinite controller output; stopping stream')
                return
        self.publish_setpoints()
        if self.stream_started is None:
            self.stream_started = now
        if (self.manage_flight and not active and now - self.stream_started >= 1.0
                and now - self.last_request >= 1.0):
            self.publish_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            if self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                self.publish_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.last_request = now


def run_deployment(*, automatic, args=None):
    """Load the separate velocity-controller YAML and run one deployment node."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, help='Velocity ADRC deployment YAML')
    options = parser.parse_args(remove_ros_args(args=args)[1:])
    path = options.config or (
        Path(get_package_share_directory('acsi_music')) / 'config' / 'adrc_vel_deploy.yaml')
    config = load_config(path)
    rclpy.init(args=args)
    node = None
    try:
        node = ADRCVelocityDeployment(config, automatic=automatic)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
