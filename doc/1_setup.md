# WMX ROS2 Application

**Note that this ROS2 application requires pre-installed WMX Linux.**


## 1. Bashrc Configuration
- Add the following environment variables to your `~/.bashrc`:
```
export ROS_DOMAIN_ID=70                         #use any number
export ROS_DISTRO=jazzy                         #support {jazzy, humble}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

source /opt/ros/$ROS_DISTRO/setup.bash
source ~/workspaces/wmx_ros2_ws/install/setup.bash
```
```
source ~/.bashrc
```

### Dependencies 
```
sudo apt install -y ros-${ROS_DISTRO}-graph-msgs \
                    ros-${ROS_DISTRO}-moveit-ros \
                    ros-${ROS_DISTRO}-moveit-planners \
                    ros-${ROS_DISTRO}-moveit-plugins \
                    ros-${ROS_DISTRO}-moveit-setup-assistant \
                    ros-${ROS_DISTRO}-moveit-configs-utils \
                    ros-${ROS_DISTRO}-moveit-task-constructor-core \
                    ros-${ROS_DISTRO}-ros2-control \
                    ros-${ROS_DISTRO}-ros2-controllers \
                    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
```

## Setup
```
mkdir -p ~/workspaces/wmx_ros2_ws/src
cd ~/workspaces/wmx_ros2_ws/src && \
   git clone https://github.com/movensys/wmx-ros2.git
```

## Build
```
cd ~/workspaces/wmx_ros2_ws
colcon build --packages-select wmx_ros2_message
source install/setup.bash
colcon build
source ~/.bashrc
```