FROM arm64v8/ros:foxy-ros-base

WORKDIR /amr_ipc_ws

RUN apt-get update && \
    apt-get install -y ros-foxy-teleop-twist-keyboard

COPY . /amr_ipc_ws/src/amr_ros2_interface/

ENV ROS_DOMAIN_ID=69
ENV ROS_DISTRO=foxy

RUN dpkg -i /amr_ipc_ws/src/amr_ros2_interface/20241016_mic-713_ox4_5.1.2_linux_5.10.120_rt70-installer.deb || true

CMD bash -c "source /opt/ros/foxy/setup.bash && \
             cd /amr_ipc_ws && \
             colcon build && \
             source install/setup.bash && \
             ros2 launch amr_ros2_interface diff_drive.launch.py"
