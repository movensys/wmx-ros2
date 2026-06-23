# WMX ROS2 Packages

**Note that this ROS2 Packages requires pre-installed WMX Linux.**

## 1. Bashrc Configuration [~/.bashrc]
```
export ROS_DOMAIN_ID=70                         #use any number
export ROS_DISTRO=jazzy                         #support {jazzy, humble}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

export ROBOT_OPTION=1                           #support {1: enable, 0:disable}

source /opt/ros/$ROS_DISTRO/setup.bash
source ~/workspaces/movensys_ws/install/setup.bash
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
                    ros-${ROS_DISTRO}-controller-manager \
                    ros-${ROS_DISTRO}-diff-drive-controller \
                    ros-${ROS_DISTRO}-joint-trajectory-controller \
                    ros-${ROS_DISTRO}-joint-state-broadcaster \
                    ros-${ROS_DISTRO}-xacro \
                    ros-${ROS_DISTRO}-topic-tools \
                    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
```

## Setup
```
mkdir -p ~/workspaces/movensys_ws/src
cd ~/workspaces/movensys_ws/src && \
   git clone https://github.com/movensys/wmx-ros2.git
```

## Build
```
source ~/.bashrc
cd ~/workspaces/movensys_ws
colcon build --packages-select wmx_ros2_message
source install/setup.bash
if [ "${ROBOT_OPTION:-0}" = "1" ]; then
   colcon build 
  else
    colcon build --packages-skip wmx_ros2_robot_option
  fi
source ~/.bashrc
```