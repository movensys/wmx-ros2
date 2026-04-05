# WMX ROS2 Application

**Note that this ROS2 application requires pre-installed WMX Linux.**


### Dependencies for humble ( ubuntu 22.04)
```
sudo apt install -y ros-humble-graph-msgs \
                    ros-humble-moveit* \
                    ros-humble-ros2-control \
                    ros-humble-ros2-controllers \
                    ros-humble-rmw-cyclonedds-cpp
```

### Dependencies for jazzy ( ubuntu 24.04)
```
sudo apt install -y ros-jazzy-graph-msgs \
                    ros-jazzy-moveit* \
                    ros-jazzy-ros2-control \
                    ros-jazzy-ros2-controllers \
                    ros-jazzy-rmw-cyclonedds-cpp
```

## Setup
```
mkdir -p ~/wmx_ros2_ws/src
cd ~/wmx_ros2_ws/src && \
   git clone https://github.com/movensys/wmx-ros2.git
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