# wmx_ros2_robot_option

ROS 2 robot arm **option** for the WMX3 motion control SDK.

This package reflects the `wmx_arm_server` of
[`wmx-server-development`](../../../) as a single ROS 2 node,
`wmx_robot_option_node`, built in the style of `wmx_ros2_package`. Where the
reference server exposes its control surface over JSON-RPC/HTTP and publishes
status over iceoryx shared memory, this node exposes the same surface over ROS 2
**services** and a periodic **status topic**, and talks to the engine directly
through the WMX3 `RobotMotion` option.

## Node: `wmx_robot_option_node`

On startup it waits for `wmx/engine/ready` (published by `wmx_engine_node`),
attaches to the WMX3 device, loads the robot kinematics model
(`robot_xml_path`) and the system/axis parameters (`wmx_param_file_path`), then
begins publishing status and serving requests.

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `robot_xml_path` | string | `""` | Robot kinematics XML model (RobotConfig::ImportParamXML) |
| `wmx_param_file_path` | string | `""` | WMX3 system/axis parameter XML |
| `robot_export_xml_path` | string | `""` | Export target; empty → `<robot_xml_path>_export.xml` |
| `status_frame` | string | `base_link` | `frame_id` stamped on the status topic |
| `collision_sensitivity` | double | `6.0` | Default collision threshold |
| `status_rate` | int | `50` | Status publication rate (Hz) |

### Published topics

| Topic | Type | Description |
|---|---|---|
| `wmx/robot/ready` | `std_msgs/Bool` | Latched, true once the option is initialized |
| `wmx/robot/status` | `wmx_ros2_message/RobotArmStatus` | Aggregated robot state (`get_status`) |

### Services (reflecting the `wmx_arm_server` RPC surface)

| Service | Type | Reference RPC |
|---|---|---|
| `wmx/robot/set_servo` | `std_srvs/SetBool` | `set_servo` |
| `wmx/robot/set_speed` | `wmx_ros2_message/SetRobotScalar` | `set_speed` |
| `wmx/robot/clear_errors` | `std_srvs/Trigger` | `clear_errors` |
| `wmx/robot/stop_motion` | `std_srvs/Trigger` | `stop_motion` |
| `wmx/robot/export_params` | `std_srvs/Trigger` | `export_params` |
| `wmx/robot/jog_pose` | `wmx_ros2_message/RobotJogPose` | `jog_arm_pose` |
| `wmx/robot/jog_pose_absolute` | `wmx_ros2_message/RobotJogPose` | `jog_arm_pose_absolute` |
| `wmx/robot/set_pose_ptp` | `wmx_ros2_message/RobotMovePose` | `set_arm_pose_ptp` |
| `wmx/robot/jog_angle` | `wmx_ros2_message/RobotMoveAngle` | `jog_arm_angle` |
| `wmx/robot/jog_angle_absolute` | `wmx_ros2_message/RobotMoveAngle` | `jog_arm_angle_absolute` |
| `wmx/robot/set_angle_ptp` | `wmx_ros2_message/RobotMoveAngle` | `set_arm_angle_ptp` |
| `wmx/robot/check_pose` | `wmx_ros2_message/RobotCheckPose` | `check_target_pose` |
| `wmx/robot/check_angle` | `wmx_ros2_message/RobotCheckAngle` | `check_target_angle` |
| `wmx/robot/set_collision_enable` | `std_srvs/SetBool` | `set_collision_enable` |
| `wmx/robot/set_fitting_param` | `std_srvs/SetBool` | `set_fitting_param` |
| `wmx/robot/set_collision_sensitivity` | `wmx_ros2_message/SetRobotScalar` | `set_collision_sensitivity` |

Cartesian poses use `[x, y, z, qx, qy, qz, qw]` (metres, quaternion); jog offsets
use `[x, y, z, rx, ry, rz]` (metres, radians); joint values are radians for
revolute joints and metres for prismatic joints.

## Build

```bash
colcon build --packages-select wmx_ros2_message wmx_ros2_robot_option
```

The WMX3 SDK at `WMX3_SDK_PATH` (default `/opt/wmx3`) must provide the
`RobotMotion` option (`librobotmotionapi`). Set a different SDK path with
`-DWMX3_SDK_PATH=/path/to/wmx3`.

## Run

```bash
# Bring up the engine first (from wmx_ros2_package), then:
ros2 launch wmx_ros2_robot_option wmx_ros2_robot_option.launch.py \
  robot_xml_path:=/abs/path/to/robot.xml
```

`config/cr3a_robot_option_example.xml` is a sample robot kinematics model; replace
it or pass `robot_xml_path` for your robot. The `wmx_param_file_path` defaults to
`cr3a_wmx_parameters.xml` from `wmx_ros2_package`.
