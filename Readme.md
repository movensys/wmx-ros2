# WMX ROS2 Application

### Dependencies 
```
Advantech MIC-713-OX4A1
Ubuntu 20.04
ROS2 Foxy 
LMX Installation
```

```
sudo apt install -y ros-foxy-robot-localization \
                    ros-foxy-slam-toolbox \
                    ros-foxy-navigation2 \
                    ros-foxy-nav2* \
                    ros-foxy-graph-msgs \
                    ros-foxy-moveit* \
                    ros-foxy-ros2-control \
                    ros-foxy-ros2-controllers
```

### Configuration
```
source /opt/ros/foxy/setup.bash
source ~/wmx_ros2_ws/install/setup.bas
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=88
```

### Build
```
cd wmx_ros2_ws
colcon build
```

## WMX ROS2 General Package
### Running Command 
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
     bash -c "source /opt/ros/foxy/setup.bash && source /home/jetstream/wmx_ros2_ws/install/setup.bash && ros2 launch wmx_ros2_general_package wmx_ros2_general.launch.py"
```

```
ros2 run wmx_ros2_general_package wmx_ros2_general_example
```

### Service Configuration
```
sudo cp wmx_ros2_general_package.service /lib/systemd/system
sudo systemctl daemon-reload
sudo systemctl enable wmx_ros2_general_package.service
sudo systemctl start wmx_ros2_general_package.service
sudo systemctl restart wmx_ros2_general_package.service
```

```
journalctl -u wmx_ros2_general_package.service -f
```

```
sudo systemctl stop wmx_ros2_general_package.service
sudo systemctl disable wmx_ros2_general_package.service
sudo rm /lib/systemd/system/wmx_ros2_general_package.service
sudo systemctl daemon-reload
```

## WMX ROS2 Navigation2 Package
### Mapping HIL 
Desktop
```
ros2 launch baymax_description baymax_gazebo.launch.py
```

IPC
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
     bash -c "source /opt/ros/foxy/setup.bash && source /home/jetstream/wmx_ros2_ws/install/setup.bash && ros2 launch wmx_ros2_navigation2_package hil-wmx_ros2_navigation2.launch.py"
```

IPC
```
ros2 launch wmx_ros2_navigation2_package hil-mapping.launch.py
```

Desktop
```
rviz2
```

Desktop
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

IPC
```
ros2 run nav2_map_server map_saver_cli -f ./src/wmx_ros2_application/wmx_ros2_navigation2_package/maps/hil-map --ros-args -p save_map_timeout:=10000
```

### Navigation HIL 
Desktop
```
ros2 launch baymax_description baymax_gazebo.launch.py
```

IPC
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
     bash -c "source /opt/ros/foxy/setup.bash && source /home/jetstream/wmx_ros2_ws/install/setup.bash && ros2 launch wmx_ros2_navigation2_package hil-wmx_ros2_navigation2.launch.py"
```

IPC
```
ros2 launch wmx_ros2_navigation2_package hil-navigation.launch.py
```

Desktop
```
rviz2
```
Set initial pose

Desktop
```
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: { frame_id: 'map' },
    pose: {position: { x: 8.0, y: 8.0, z: 0.0 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 0.0 }}}}"
```

Check server
```
for node in $(ros2 lifecycle nodes -a); do echo "$node: $(ros2 lifecycle get $node)"; done
```

## WMX ROS2 MoveIt2 Package
### Dobot CR3A 
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
     bash -c "source /opt/ros/foxy/setup.bash && source /home/jetstream/wmx_ros2_ws/install/setup.bash && ros2 launch wmx_ros2_moveit2_package wmx_moveit2.launch.py"
```

```
ros2 launch cr3a_moveit_config cr3a_moveit.launch.py
```

Set initial joint states
```
ros2 topic pub /joint_states sensor_msgs/msg/JointState "{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: ''
  },
  name: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
  position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  velocity: [],
  effort: []
}"
```
