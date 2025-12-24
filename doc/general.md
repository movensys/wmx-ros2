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
     bash -c "source /opt/ros/humble/setup.bash && source /home/mic-713/wmx_ros2_ws/install/setup.bash && ros2 launch wmx_ros2_package wmx_ros2_general.launch.py"
```

```
ros2 run wmx_ros2_package wmx_ros2_general_example
```