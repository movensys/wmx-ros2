# WMX ROS2 Application

### Running On
```
Advantech MIC-713-OX4A1
Ubuntu 22.04
ROS2 Humble 
[WMX3Engine] Build: Jun  6 2025:18:10:19 (v3.5.0.0)
```

### Dependencies 
```
sudo apt install -y ros-humble-graph-msgs \
                    ros-humble-moveit* \
                    ros-humble-ros2-control \
                    ros-humble-ros2-controllers
```

### Configuration
```
source /opt/ros/humble/setup.bash
source ~/wmx_ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
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
