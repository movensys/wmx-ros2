# Simple navigation HIL simulation

hahakr
```
sudo ip addr add 192.168.0.166/24 dev enp4s0
sudo ip link set enp4s0 up
```

test-kr-1
```
sudo ip addr add 192.168.0.198/24 dev lan2
sudo ip link set lan2 up
```

```
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_cpp listener
```

```
ros2 launch foxy_navigation 01-controller.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```

```
ros2 launch foxy_navigation 02-mapping.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
ros2 run nav2_map_server map_saver_cli -f ./src/foxy_navigation/maps/map --ros-args -p save_map_timeout:=10000
```

```
ros2 launch foxy_navigation 03-autonomous.launch.py
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: { frame_id: 'map' },
    pose: {position: { x: 8.0, y: 8.0, z: 0.0 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 0.0 }}}}"
```