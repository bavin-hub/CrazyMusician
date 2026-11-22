"""Separate deployment configuration for velocity-setpoint ADRC nodes."""

import math

import yaml


def load_config(path):
    """Read a deployment YAML file and validate its waypoint settings."""
    with open(path, encoding='utf-8') as stream:
        config = yaml.safe_load(stream) or {}
    if not isinstance(config, dict):
        raise ValueError('Deployment YAML must contain a mapping')
    allowed = {'namespace', 'enable_test', 'waypoints', 'waypoint_tolerance',
               'waypoint_speed_tolerance', 'waypoint_hold_time', 'controller'}
    unknown = set(config) - allowed
    if unknown:
        raise ValueError(f'Unknown deployment settings: {sorted(unknown)}')
    if type(config.setdefault('enable_test', False)) is not bool:
        raise ValueError('enable_test must be true or false')
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
        'control_rate', 'yaw', 'kpos_xy', 'k_vel_xy', 'wo_xy', 'vmax_xy',
        'kpos_z', 'k_vel_z', 'wo_z', 'vmax_z', 'tilt_max_deg',
        'move_time', 'ff_ramp_time', 'feedback_timeout', 'status_timeout',
        'use_feedforward', 'use_r_ddot',
        'target_system', 'target_component',
    }
    if not isinstance(controller, dict) or set(controller) - allowed_controller:
        raise ValueError('controller contains unsupported settings')
    for name, value in controller.items():
        if name in ('use_feedforward', 'use_r_ddot'):
            if type(value) is not bool:
                raise ValueError(f'{name} must be true or false')
        elif name in ('target_system', 'target_component'):
            if type(value) is not int:
                raise ValueError(f'{name} must be an integer')
        elif isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f'{name} must be a number')
        else:
            controller[name] = float(value)
    return config
