# WMX ROS2 General Package

## Subscriber

**/cmd_vel [geometry_msgs::msg::Twist]**
```
WMX API: CoreMotion.velocity->StartVel(&wmx3Api::Velocity::VelCommand());
```

## Publisher
**/velocity_controller/commands [std_msgs::msg::Float64MultiArray]**
```
float64[] axis_velocity

WMX API:  CoreMotion.GetStatus(&CoreMotionStatus)
          CoreMotionStatus.axesStatus
```
<br>
<br>

**/odom_enc [nav_msgs::msg::Odometry]**
