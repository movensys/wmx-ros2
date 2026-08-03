# WMX R2 Packages

**Note that this ROS2 Packages requires pre-installed WMX Linux.**

## 1. Bashrc Configuration [~/.bashrc]
```
# ROS
export ROS_DOMAIN_ID=70                         # use any number
export ROS_DISTRO=jazzy                         # support {jazzy, humble}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# WMX
export WMX_CORE=3                               # CPU core for the WMX real-time engine (-1 = SDK default)
export WMX_AFFINITY_MASK=12                     # CPU affinity bitmask for the WMX engine threads (one bit per core e.g. 12 for cores 2 and 3)

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
   git clone https://github.com/movensys/wmx-r2.git
```

## Build
```
cd ~/workspaces/movensys_ws
colcon build --packages-select wmx_r2_message
source install/setup.bash
colcon build
source ~/.bashrc
```