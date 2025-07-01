# Simple navigation HIL simulation
```
cd /opt/lmx/bin/
sudo ./lmx-start-engine
sudo ./lmx-start-comm
sudo ./lmx-servo-on 0 1
```

```
sudo -E bash -c "source /opt/ros/foxy/setup.bash && source install/setup.bash && ros2 launch amr_ros2_interface diff_drive.launch.py"
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/cmd_vel_safe
```

# Jetstream HIL simulation

# Jetstream Real world 


# Service 
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

# Dockerfile (on progress)
```
docker build -t amr_ros2_image .
docker run -it --name amr_ros2_container amr_ros2_image
```

```
docker stop amr_ros2_container
docker rm amr_ros2_container
docker rmi amr_ros2_image
```
