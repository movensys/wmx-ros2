# Differential robot (ros2_control)

This is the **ros2_control** variant of [`launch_diffbot_navigation.md`](../../doc/launch_diffbot_navigation.md).
Instead of the standalone `differential_drive_controller`, it runs a
`controller_manager` with the WMX3 hardware interface
(`wmx_ros2_control/WmxSystemHardware`) and the standard
`diff_drive_controller` + `joint_state_broadcaster`.

## What it launches

- `wmx_ros2_general_nodes.launch.py` — engine / core_motion / io / ethercat
  (owns the WMX engine, publishes `wmx/engine/ready`). **Unchanged.**
- `robot_state_publisher` with `diffbot.wmx.urdf.xacro` (geometry from
  `movensys_navigation_description` + the WMX `<ros2_control>` block).
- `controller_manager` (`ros2_control_node`) loading
  [`config/diffbot_controllers.yaml`](../config/diffbot_controllers.yaml).
- spawners: `joint_state_broadcaster`, `diffbot_base_controller`
  (`diff_drive_controller/DiffDriveController`).

`diff_drive_controller` consumes `/cmd_vel` (remapped from
`/diffbot_base_controller/cmd_vel`), commands the two wheel **velocity**
interfaces, and publishes `/odom` + odom TF. `WmxSystemHardware` forwards the
wheel velocities to WMX3 `CoreMotion::StartVel` and reports encoder
position/velocity back as state interfaces.

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
     ros2 launch wmx_ros2_control wmx_ros2_control_diffbot_navigation.launch.py use_sim_time:=false"
```

## Notes

- **`interval_cycles`** is irrelevant for the diffbot (the wheels are velocity
  controlled, not cyclic-buffer streamed). It only matters for position joints.
- The WMX axis parameter file is taken from
  `wmx_ros2_package/config/diffbot_wmx_parameters.xml`; override with
  `wmx_param_file:=/path/to/params.xml`.
- `wheel_separation` (0.55) and `wheel_radius` (0.095) live in
  `config/diffbot_controllers.yaml` — keep them in sync with the WMX param file
  and the URDF.
- The WMX engine must reach **Communicating** (servo on, EtherCAT up) before the
  controllers activate; otherwise `on_activate` fails with an explicit error.
