# WMX ROS2 Application

ROS2 interface for WMX3/LMX motor control hardware to control CR3A manipulator robot.

## Architecture

**Low-level Control (wmx_ros2_general.launch.py):**

wmx_ros2_general_example.cpp -> services/topics -> wmx_ros2_general_node -> WMX3 API (CreateDevice, StartCommunication, CoreMotion) -> LMX

**Trajectory Control (wmx_ros2_manipulator.launch.py):**

/follow_joint_trajectory (action) -> follow_joint_trajectory_server -> WMX3 API -> LMX -> Robot

Robot -> LMX -> WMX3 API -> manipulator_state -> /joint_states

## Packages

**wmx_ros2_message** - Custom messages and services for axis control

**wmx_ros2_package** - Main nodes for robot control

## Nodes

**manipulator_state** - Publishes joint feedback from WMX3 encoder to `/joint_states`

**follow_joint_trajectory_server** - Receives trajectory action and executes via WMX3 C-Spline

**wmx_ros2_general_node** - Low-level axis control via services and topics

## Launch Files

**wmx_ros2_manipulator.launch.py** - For trajectory control (starts `manipulator_state` + `follow_joint_trajectory_server`)

**wmx_ros2_general.launch.py** - For low-level axis control (starts `wmx_ros2_general_node`)

## MoveIt2 Integration

To connect with `movensys_isaac_manipulator`, change action name in `follow_joint_trajectory_server.cpp:80`:

```cpp
"/movensys_manipulator_arm_controller/follow_joint_trajectory"
```

## Documentation

For setup and execution, see [doc/1_setup.md](doc/1_setup.md)
