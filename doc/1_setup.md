# WMX ROS2 Application

## Running On
```
Ubuntu 22.04
ROS2 Humble 
[WMX3Engine] Build: Jun  6 2025:18:10:19 (v3.5.0.0)
```

## Dependencies 
```
sudo apt install -y ros-humble-graph-msgs \
                    ros-humble-moveit* \
                    ros-humble-ros2-control \
                    ros-humble-ros2-controllers \
                    ros-humble-rmw-cyclonedds-cpp
```

## Setup
```
mkdir -p ~/wmx_ros2_ws/src
cd ~/wmx_ros2_ws/src && \
   git clone git@bitbucket.org:mvs_app/wmx_ros2_application.git
```

## Build
```
cd ~/wmx_ros2_ws
colcon build --packages-select wmx_ros2_message
source install/setup.bash
colcon build
source ~/.bashrc
```

## Bashrc
```
export ROS_DOMAIN_ID=70

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

source /opt/ros/humble/setup.bash
source ~/wmx_ros2_ws/install/setup.bash
```
```
source ~/.bashrc
```