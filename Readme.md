# WMX ROS2 Application

### Dependencies 
```
Advantech MIC-713-OX4A1
Ubuntu 20.04
ROS2 Foxy 
LMX Installation
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
     bash -c "source /opt/ros/foxy/setup.bash && source /home/jetstream/wmx_ros2_ws/install/setup.bash && ros2 launch wmx_ros2_navigation2_package wmx_ros2_navigation.launch.py"
```
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```