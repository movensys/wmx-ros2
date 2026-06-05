# Dobot CR3A manipulator (ros2_control)

This is the **ros2_control** variant of [`launch_dobot_cr3a_manipulator.md`](../../doc/launch_dobot_cr3a_manipulator.md).
Instead of the standalone `joint_trajectory_controller` + `joint_state_broadcaster`,
it runs a `controller_manager` with the WMX3 hardware interface
(`wmx_ros2_control/WmxSystemHardware`) and the standard
`joint_trajectory_controller` + `joint_state_broadcaster`.

## What it launches

- `wmx_ros2_general_nodes.launch.py` — engine / core_motion / io / ethercat.
  **Unchanged.**
- `robot_state_publisher` with `cr3a.wmx.urdf.xacro` (geometry from
  `movensys_manipulator_description` + the WMX `<ros2_control>` block).
- `controller_manager` (`ros2_control_node`) loading
  [`config/cr3a_controllers.yaml`](../config/cr3a_controllers.yaml).
- spawners: `joint_state_broadcaster`, `movensys_manipulator_arm_controller`
  (`joint_trajectory_controller/JointTrajectoryController`).
- `gripper_controller` — the **standalone IO-driven node**, unchanged.

The controller is named `movensys_manipulator_arm_controller` so MoveIt's
`moveit_controllers.yaml` action
(`/movensys_manipulator_arm_controller/follow_joint_trajectory`) resolves without
changes. `joint_trajectory_controller` interpolates the trajectory and writes a
per-cycle **position** to each joint; `WmxSystemHardware` streams those into the
WMX3 `CyclicBuffer` as `AbsolutePos` segments.

## CyclicBuffer streaming and `interval_cycles`

Each `write()` pushes a **single multi-axis** `CyclicBuffer` command covering all
six arm axes (one `AbsolutePos` segment per axis, indexed by selection position —
the same axis-selection convention as `wmx_ros2_package`'s
`joint_trajectory_controller`). Each segment spans `interval_cycles` WMX RT
cycles, so:

```
interval_cycles = WMX_RT_cycle_rate_Hz / controller_manager_update_rate_Hz
```

The defaults assume a 1 kHz WMX engine and `update_rate: 100`
(`config/cr3a_controllers.yaml`) → `interval_cycles: 10`. If you change either
rate, update `interval_cycles` (URDF xacro arg) to match, or the buffer will
under/over-run.

## Gripper

The gripper (`picker_1_joint` / `picker_2_joint`) is **not** part of the
ros2_control system — it is toggled over digital IO by the `gripper_controller`
node via `/wmx/set_gripper`, exactly as in the non-ros2_control launch. The
gripper links therefore appear at their fixed URDF pose in TF.

## Launch

```
sudo --preserve-env=PATH \
     --preserve-env=AMENT_PREFIX_PATH \
     --preserve-env=COLCON_PREFIX_PATH \
     --preserve-env=PYTHONPATH \
     --preserve-env=LD_LIBRARY_PATH \
     --preserve-env=ROS_DISTRO \
     --preserve-env=ROS_VERSION \
     --preserve-env=ROS_PYTHON_VERSION \
     --preserve-env=ROS_DOMAIN_ID \
     --preserve-env=RMW_IMPLEMENTATION \
     bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && source $HOME/workspaces/movensys_ws/install/setup.bash && \
     ros2 launch wmx_ros2_control wmx_ros2_control_cr3a_manipulator.launch.py use_sim_time:=false"
```

To connect MoveIt, launch the `movensys-manipulator` MoveIt stack as usual — the
`FollowJointTrajectory` action server is provided by
`movensys_manipulator_arm_controller`.

## Notes

- The WMX axis parameter file defaults to
  `wmx_ros2_package/config/cr3a_wmx_parameters.xml`; override with
  `wmx_param_file:=/path/to/params.xml`.
- The WMX param file must be configured so user units are **radians** (the
  trajectory positions are passed through 1:1, matching the existing
  `joint_trajectory_controller`).
- The WMX engine must reach **Communicating** (servo on) before activation.
