# WMX ROS2 Application

### Running On
```
Advantech MIC-713-OX4A1
Ubuntu 20.04
ROS2 Foxy 
[WMX3Engine] Build: Jun  6 2025:18:10:19 (v3.5.0.0)
```

### Dependencies 
```
sudo apt install -y ros-foxy-graph-msgs \
                    ros-foxy-moveit* \
                    ros-foxy-ros2-control \
                    ros-foxy-ros2-controllers
```

### Configuration
```
source /opt/ros/foxy/setup.bash
source ~/wmx_ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=70
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
     bash -c "source /opt/ros/foxy/setup.bash && source /home/jetstream/wmx_ros2_ws/install/setup.bash && ros2 launch wmx_ros2_package wmx_ros2_moveit2.launch.py"
```

## Debugging Command
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