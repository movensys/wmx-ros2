# WMX ROS2 General Package

## Subscriber

**/wmx/axis/position [wmx_ros2_message/msg/AxisPose]**
```
int32[] index #axis index (e.g [0, 1])
float64[] target # pose target (rad)
string profile #profile type (not working yet)
float64[] velocity #velocity (rad/s)
float64[] acc #acceleration (rad/s^2)
float64[] dec #decceleration (rad/s^2)

WMX API: CoreMotion.motion->StartPos(&WMX3Api::Motion::PosCommand)
```
<br>
<br>
**/wmx/axis/position/relative [wmx_ros2_message/msg/AxisPose]**
```
int32[] index #axis index (e.g [0, 1])
float64[] target # pose target (rad)
string profile #profile type (not working yet)
float64[] velocity #velocity (rad/s)
float64[] acc #acceleration (rad/s^2)
float64[] dec #decceleration (rad/s^2)

WMX API: CoreMotion.motion->StartMov(&WMX3Api::Motion::PosCommand)
```
<br>
<br>
**/wmx/axis/velocity [wmx_ros2_message/msg/AxisVelocity]**
```
int32[] index #axis index (e.g [0, 1])
string profile #profile type (not working yet)
float64[] velocity #velocity (rad/s)
float64[] acc #acceleration (rad/s^2)
float64[] dec #decceleration (rad/s^2)

WMX API: CoreMotion.velocity->StartVel(&wmx3Api::Velocity::VelCommand)
```
<br>
<br>
## Publisher
**/wmx/axis/state [wmx_ros2_message/msg/AxisState]**
```
int32[] amp_alarm
int32[] servo_on
int32[] home_done
int32[] in_pos
int32[] negative_ls
int32[] positive_ls
int32[] home_switch
float64[] pos_cmd
float64[] velocity_cmd
float64[] actual_pos
float64[] actual_velocity
float64[] actual_torque

WMX API:  CoreMotion.GetStatus(&CoreMotionStatus)
          CoreMotionStatus.axesStatus
```

## Service
`/wmx/engine/set_device [wmx_ros2_message::srv::SetEngine]`
```
wmx3Lib_.CreateDevice(request->path.c_str(), DeviceType::DeviceTypeNormal, INFINITE);
wmx3Lib_.SetDeviceName(request->name.c_str());
```

`/wmx/engine/set_comm [std_srvs::srv::SetBool]`
```
wmx3Lib_.StartCommunication(INFINITE);
```

`/wmx/engine/get_status [std_srvs::srv::Trigger]`
```
wmx3Api::EngineStatus status; 
wmx3Lib_.GetEngineStatus(&status);
```

`/wmx/axis/set_on [wmx_ros2_message::srv::SetAxis]`
```
wmx3LibCm_.axisControl->SetServoOn(axis_index, on_off);
```

`/wmx/axis/clear_alarm [wmx_ros2_message::srv::SetAxis]`
```
wmx3LibCm_.axisControl->ClearAmpAlarm(request->index[i])
```

`/wmx/axis/set_mode [wmx_ros2_message::srv::SetAxis]`
```
wmx3LibCm_.axisControl->SetAxisCommandMode(axis_index, AxisCommandMode::Position)
wmx3LibCm_.axisControl->SetAxisCommandMode(axis_index, AxisCommandMode::Velocity);
```

`/wmx/axis/set_polarity [wmx_ros2_message::srv::SetAxis]`
```
wmx3LibCm_.config->SetAxisPolarity(request->index[i], request->data[i]);
```

`/wmx/axis/set_gear_ratio [wmx_ros2_message::srv::SetAxisGearRatio]`
```
wmx3LibCm_.config->SetGearRatio(request->index[i], request->numerator[i], request->denumerator[i]);
```

`wmx/axis/homing [wmx_ros2_message::srv::SetAxis]`
```
wmx3LibCm_.config->GetHomeParam(request->index[i], &homeParam_);
homeParam_.homeType = Config::HomeType::CurrentPos;
wmx3LibCm_.config->SetHomeParam(request->index[i], &homeParam_);
wmx3LibCm_.home->StartHome(request->index[i]);
wmx3LibCm_.motion->Wait(request->index[i]);
```