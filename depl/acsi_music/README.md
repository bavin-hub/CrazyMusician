# ADRC acceleration offboard node

## Separate velocity ADRC controllers

The velocity controller from `crazymusician` branch `adrc-vel` is ported into
`adrc_vel_core.py` and `adrc_vel_deploy.py`, with two independent executables:

```bash
ros2 run acsi_music adrc_vel_deploy_auto.py --config /path/to/adrc_vel_deploy.yaml
ros2 run acsi_music adrc_vel_deploy_user.py --config /path/to/adrc_vel_deploy.yaml
```

Executable names without `.py` also work. Omitting `--config` uses the installed
[config/adrc_vel_deploy.yaml](config/adrc_vel_deploy.yaml). Rebuild and source the
workspace after adding the new nodes. The existing acceleration controller files
and their YAML are separate and retain their behavior. Run one controller per vehicle.

Set top-level `enable_test: true` in the velocity YAML to compute and print NED
velocity (m/s) and acceleration feedforward (m/s²) without creating any PX4 input
publishers. This applies to both variants: no heartbeat, setpoints, mode commands,
or arming commands are sent. Test mode starts the waypoint reference with valid
odometry and does not require vehicle status, arming, or offboard engagement.
It prints at `control_rate` and still requires fresh NED odometry. The default
`enable_test: false` preserves normal PX4 publishing. Restart after changing it.

Both new nodes send `TrajectorySetpoint.velocity` plus acceleration feedforward
in `TrajectorySetpoint.acceleration`, with position set to NaN. The heartbeat sets
**velocity true and acceleration false**. PX4 closes the velocity loop and adds
the acceleration feedforward; the ESO models PX4's velocity response.
Set `k_vel_xy` and `k_vel_z` to match the vehicle's `MPC_XY_VEL_P_ACC` and
`MPC_Z_VEL_P_ACC` respectively. These are ESO model gains, not commands that
change PX4's tuning. Defaults match upstream: 1.8 and 4.0.

The YAML follows the same namespace and NED waypoint structure as the existing
deployment configuration, with a default single waypoint `[0, 0, -1]` and empty
namespace (`/fmu/...`). `namespace: "drone3"` gives `/drone3/fmu/...` for all PX4
topics. `target_system` is the separate PX4 MAV_SYS_ID used for automatic commands.

The auto variant streams for one second, requests offboard and arming, and starts
the smooth waypoint reference when armed offboard is confirmed. It stops after
leaving that mode or losing feedback in flight and requires a restart.

The user variant has no status subscription, mode checks, or command publisher.
It starts the reference and feedforward ramp on its first valid odometry sample,
and runs continuously while the operator controls offboard through RC. Mode
changes do not restart the mission. Odometry loss pauses the stream; recovery
resets the observer and replans from the current position to the current waypoint.
This preserves our deployment semantics rather than upstream's mode-gated target
capture; in user mode the observer cannot know whether PX4 is applying its commands.

Waypoint transitions use the upstream minimum-jerk reference over `move_time`
(default 3 seconds). After reference completion, the vehicle must stay within
the position and speed tolerances for `waypoint_hold_time` before advancing.
The final waypoint is held indefinitely; there is no automatic landing.

Velocity-controller settings in its separate YAML include:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `control_rate` | `100.0` | Publication rate, Hz; ESO/control update on new odometry |
| `kpos_xy`, `kpos_z` | `1.0` | Position gains, 1/s |
| `wo_xy`, `wo_z` | `8.0` | ESO bandwidths, rad/s |
| `k_vel_xy`, `k_vel_z` | `1.8`, `4.0` | Modeled PX4 velocity P gains |
| `vmax_xy`, `vmax_z` | `1.0`, `0.8` | Per-axis velocity reference limits, m/s |
| `tilt_max_deg` | `20.0` | Acceleration feedforward/model tilt limit |
| `use_feedforward` | `true` | Enable disturbance compensation (`-z2`) |
| `use_r_ddot` | `true` | Enable trajectory acceleration feedforward |
| `move_time` | `3.0` | Smooth move duration, seconds |
| `ff_ramp_time` | `1.0` | Feedforward phase-in after observer reset, seconds |

The upstream vertical feedforward/model acceleration limit remains ±0.5g.
Observers reset on estimator reset-counter changes, engagement, and sample gaps
over 50 ms. Both pose and velocity must be finite and in NED. Other freshness,
yaw, waypoint, and namespace settings are documented in the new YAML. Settings
are read at startup. Upstream's interactive target/debug topics and geofence
interface are not included; missions use this package's YAML waypoint interface.

## Deployment scripts

`adrc_deploy_auto.py` automatically requests arming and offboard after one second
of valid feedback/setpoint streaming. It starts executing waypoints once PX4
reports armed offboard mode. As with the original node, it stops if that mode
is left or feedback is lost after engagement; restart to resume.

`adrc_deploy_user.py` continuously runs ADRC and publishes acceleration setpoints
and the offboard heartbeat whenever valid odometry is available. It does not
subscribe to vehicle status, check modes, or send arming/mode commands. Arm and
select offboard from the RC. Switching offboard off and on does not stop the
node or restart the waypoint list. On stale/invalid odometry it pauses output,
then resets the observer and resumes when valid odometry returns.

Both use [config/adrc_deploy.yaml](config/adrc_deploy.yaml), installed into the
package share directory. Edit that file before building, or pass a custom copy:

```bash
ros2 run acsi_music adrc_deploy_auto.py --config /absolute/path/adrc_deploy.yaml
ros2 run acsi_music adrc_deploy_user.py --config /absolute/path/adrc_deploy.yaml
```

The executable names without `.py` also work. With no `--config`, both load the
installed default YAML. Run only one controller for a vehicle at a time.

```yaml
namespace: "drone3"  # "" gives /fmu/...; drone3 gives /drone3/fmu/...
waypoints:
  - [0.0, 0.0, -1.0]
  - [1.0, 0.0, -1.0]
  - [1.0, 1.0, -1.0]
waypoint_tolerance: 0.15
waypoint_speed_tolerance: 0.2
waypoint_hold_time: 1.0
controller:
  target_system: 3  # Set to PX4 MAV_SYS_ID; namespace does not determine this ID.
```

Coordinates are absolute local NED metres. Omitted or empty waypoints default
to `[0, 0, -1]`. The node advances in order after remaining within the position
and speed tolerances for the hold time, and holds the final waypoint indefinitely.
There is no automatic landing. In user mode the waypoint tracking runs from
startup, independently of RC mode; a waypoint already reached can advance before
offboard is selected. RC mode changes do not rewind the mission.
Optional `controller` settings use the gains and limits listed below.

## Original single-target node

Port of `~/cmu/sem3/acsi/crazymusician/controllers/adrc/adrc_offboard.py`,
using the ROS 2 interfaces from `px4_ros_com/src/examples/offboard_py/offboard_control.py`.
The proportional position loop, velocity ESO, disturbance cancellation, gains,
and acceleration limits are preserved. No MAVLink, NumPy, attitude quaternion,
hover thrust, or acceleration-to-attitude/thrust conversion is used in this node.
PX4 handles acceleration tracking downstream.

Build and run from the workspace (with PX4 and its DDS agent already running):

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select acsi_music
source install/setup.bash
ros2 run acsi_music adrc_offboard
```

By default the node streams at 100 Hz, requests offboard mode and arming after
one second of valid feedback/setpoints, and flies to local NED `[0, 0, -2]` m.
This is an absolute estimator-frame target: x north, y east, z down. It holds
that target until the node stops or PX4 leaves armed offboard mode. The node
does not run the original script's takeoff/step/wind/landing experiment.

```bash
ros2 run acsi_music adrc_offboard --ros-args \
  -p target_position:="[1.0, 0.0, -2.0]" \
  -p auto_arm:=false
```

`auto_arm:=false` streams setpoints but leaves arming and mode selection to the
operator. Parameters are read at startup; restart to change the target or gains.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `target_position` | `[0.0, 0.0, -2.0]` | Local NED position, m |
| `yaw` | `0.0` | Fixed heading, rad |
| `kpos_xy`, `kpos_z` | `1.0` | Position proportional gain, 1/s |
| `wcv_xy`, `wcv_z` | `1.5` | Velocity loop bandwidth, rad/s |
| `wo_xy`, `wo_z` | `8.0` | ESO bandwidth, rad/s |
| `vmax_xy`, `vmax_z` | `1.5`, `0.8` | Per-axis velocity reference limits, m/s |
| `tilt_max_deg` | `30.0` | Horizontal acceleration limit derived from tilt |
| `a_up_max`, `a_down_max` | `7.848`, `5.886` | Vertical acceleration limits, m/s² |
| `control_rate` | `100.0` | Setpoint publication rate, Hz (minimum 20) |
| `feedback_timeout` | `0.5` | Odometry freshness limit, seconds |
| `status_timeout` | `2.0` | Status freshness limit (PX4 publishes more slowly), seconds |
| `auto_arm` | `true` | Automatically request offboard and arm |

The node subscribes to `/fmu/out/vehicle_odometry` and
`/fmu/out/vehicle_status`. It publishes `/fmu/in/offboard_control_mode`,
`/fmu/in/trajectory_setpoint`, and `/fmu/in/vehicle_command`. Use standard ROS
topic remappings for a different PX4 namespace. Odometry position and velocity
must both declare the NED frame; unknown, FRD-world, and body-frame data are
rejected. Position and velocity come from the odometry vectors, and its
`reset_counter` resets the ESO when the estimator resets.

Only `OffboardControlMode.acceleration` is enabled. Trajectory position and
velocity are NaN; acceleration is in NED m/s², with zero meaning zero net
acceleration (do not add gravity). Yaw holds the configured heading.
The ESO advances once per new odometry sample using the previous limited command;
it resets on engagement, estimator resets, or sample gaps over 50 ms.
PX4 may further limit the requested acceleration, so this command is not a
measurement of the acceleration actually achieved by the aircraft.

Invalid or stale feedback stops the stream. After armed offboard operation has
begun, feedback loss or a mode/disarm change latches the node stopped; restart
to resume. Stopping the node also stops the heartbeat, leaving PX4's configured
offboard-loss behavior in charge. No landing command is issued automatically.
