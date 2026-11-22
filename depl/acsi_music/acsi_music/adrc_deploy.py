"""Shared YAML configuration and waypoint execution for deployment nodes."""

import argparse
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.parameter import Parameter
from rclpy.utilities import remove_ros_args
import yaml

from acsi_music.adrc_offboard import ADRCOffboard


def load_config(path):
    """Read a deployment YAML file and validate its waypoint settings."""
    with open(path, encoding='utf-8') as stream:
        config = yaml.safe_load(stream) or {}
    if not isinstance(config, dict):
        raise ValueError('Deployment YAML must contain a mapping')
    allowed = {'namespace', 'waypoints', 'waypoint_tolerance',
               'waypoint_speed_tolerance', 'waypoint_hold_time', 'controller'}
    unknown = set(config) - allowed
    if unknown:
        raise ValueError(f'Unknown deployment settings: {sorted(unknown)}')
    namespace = config.setdefault('namespace', '')
    if not isinstance(namespace, str):
        raise ValueError('namespace must be a string, e.g. "" or "drone3"')
    namespace = namespace.strip('/')
    config['namespace'] = f'/{namespace}' if namespace else ''
    waypoints = config.get('waypoints')
    if waypoints is None or waypoints == []:
        waypoints = [[0.0, 0.0, -1.0]]
    if not isinstance(waypoints, list) or not all(
        isinstance(point, list) and len(point) == 3
        and all(isinstance(v, (int, float)) and not isinstance(v, bool)
                and math.isfinite(v) for v in point)
        for point in waypoints
    ):
        raise ValueError('waypoints must be a list of finite [north, east, down] triples')
    config['waypoints'] = [[float(v) for v in point] for point in waypoints]
    for name, default in (
        ('waypoint_tolerance', 0.15), ('waypoint_speed_tolerance', 0.2),
        ('waypoint_hold_time', 1.0),
    ):
        value = config.setdefault(name, default)
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or value < 0
                or (name != 'waypoint_hold_time' and value == 0)):
            raise ValueError(f'{name} must be finite and positive (hold time may be zero)')
    controller = config.setdefault('controller', {})
    allowed_controller = {
        'control_rate', 'yaw', 'kpos_xy', 'wcv_xy', 'wo_xy', 'vmax_xy',
        'kpos_z', 'wcv_z', 'wo_z', 'vmax_z', 'tilt_max_deg',
        'a_up_max', 'a_down_max', 'feedback_timeout', 'status_timeout',
        'target_system', 'target_component',
    }
    if not isinstance(controller, dict) or set(controller) - allowed_controller:
        raise ValueError('controller contains unsupported settings')
    for name, value in controller.items():
        if name in ('target_system', 'target_component'):
            if type(value) is not int:
                raise ValueError(f'{name} must be an integer')
        elif isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f'{name} must be a number')
        else:
            controller[name] = float(value)
    return config


class ADRCDeployment(ADRCOffboard):
    """Execute NED waypoints using the shared acceleration ADRC controller."""

    def __init__(self, config, *, automatic):
        self.waypoints = config['waypoints']
        self.waypoint_index = 0
        self.waypoint_since = None
        self.waypoint_last_update = None
        self.waypoint_tolerance = config['waypoint_tolerance']
        self.waypoint_speed_tolerance = config['waypoint_speed_tolerance']
        self.waypoint_hold_time = config['waypoint_hold_time']
        parameters = dict(config['controller'])
        parameters['target_position'] = self.waypoints[0]
        parameters['auto_arm'] = automatic
        super().__init__(
            node_name='adrc_deploy_auto' if automatic else 'adrc_deploy_user',
            namespace=config['namespace'], user_control=not automatic,
            parameter_overrides=[Parameter(name, value=value)
                                 for name, value in parameters.items()],
        )

    def update_target(self, now, active):
        """Advance after dwelling near a waypoint at low speed; hold the final one."""
        if (not active or self.waypoint_last_update is None
                or now - self.waypoint_last_update > self.timeout):
            self.waypoint_since = None
        self.waypoint_last_update = now
        if not active or self.waypoint_index == len(self.waypoints) - 1:
            return
        distance = math.dist(self.odometry.position, self.target)
        speed = math.sqrt(sum(float(v) ** 2 for v in self.odometry.velocity))
        if distance > self.waypoint_tolerance or speed > self.waypoint_speed_tolerance:
            self.waypoint_since = None
            return
        if self.waypoint_since is None:
            self.waypoint_since = now
        if now - self.waypoint_since >= self.waypoint_hold_time:
            self.waypoint_index += 1
            self.target = self.waypoints[self.waypoint_index]
            self.waypoint_since = None
            self.get_logger().info(
                f'Waypoint {self.waypoint_index + 1}/{len(self.waypoints)}: {self.target}')


def run_deployment(*, automatic, args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--config', type=Path,
        default=Path(get_package_share_directory('acsi_music')) / 'config' / 'adrc_deploy.yaml',
        help='Deployment YAML file (namespace, NED waypoints and controller gains)',
    )
    options = parser.parse_args(remove_ros_args(args=args)[1:])
    config = load_config(options.config)
    rclpy.init(args=args)
    node = None
    try:
        node = ADRCDeployment(config, automatic=automatic)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
