# Movensys 4-axis cartesian robot

WMX R2 owns the motion; MoveIt only plans. There is no `ros2_control_node`
anywhere in this path — `joint_trajectory_controller` below *is* the
`FollowJointTrajectory` action server that MoveIt executes against.

Root is required because the EtherCAT master runs in a real-time thread.

# For trajectory control
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
     ros2 launch wmx_r2_package wmx_r2_cartesian.launch.py use_sim_time:=false"
```

Set `use_sim_time:=true` for the HiL run (Isaac Sim publishes the clock).

# Axis map

| URDF joint | WMX axis | travel (m)        |
|------------|----------|-------------------|
| `axis_x`   | 0        | [-0.115,  0.100]  |
| `axis_y`   | 1        | [-0.100,  0.100]  |
| `axis_z`   | 2        | [ 0.012,  0.090]  |
| `axis_r`   | 3        | continuous (rad)  |

`joint_trajectory_controller` maps trajectory positions onto axes **by index**,
not by joint name. MoveIt emits the SRDF chain order `axis_y, axis_x, axis_z`,
which is why `joint_axes` in `config/cartesian_config.yaml` is `[1, 0, 2]`.

# Before running on real hardware

`config/cartesian_wmx_parameters.xml` is still a copy of
`default_wmx_parameters.xml`. Export the real one from WMX Studio; see the TODO
block at the top of that file for what has to be right, above all `axisUnit`
(1 WMX unit must equal 1 metre, because the ROS nodes do no unit conversion).
