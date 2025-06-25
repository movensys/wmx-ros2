# WMX_ROS2_Application

## Dependencies 
```
Advantech MIC-713-OX4A1
Ubuntu 20.04
ROS2 Foxy 
```

## Installation

## Running
```
sudo -E bash -c "source /opt/ros/foxy/setup.bash && source install/setup.bash && ros2 launch wmx_ros2_general_package wmx_ros2_general.launch.py"
```

```
sudo -E bash -c "source /opt/ros/foxy/setup.bash && source install/setup.bash && ros2 run wmx_ros2_general_package wmx_ros2_general_example"
```

## Service Configuration
```
sudo cp amr_ros2_interface.service /lib/systemd/system
sudo systemctl daemon-reload
sudo systemctl enable amr_ros2_interface.service
sudo systemctl start amr_ros2_interface.service
sudo systemctl restart amr_ros2_interface.service
```

```
journalctl -u amr_ros2_interface.service -f
```

```
sudo systemctl stop amr_ros2_interface.service
sudo systemctl disable amr_ros2_interface.service
sudo rm /lib/systemd/system/amr_ros2_interface.service
sudo systemctl daemon-reload
```