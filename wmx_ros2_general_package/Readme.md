# WMX ROS2 General Package

## Subscriber
`/wmx/axis/position [wmx_ros2_message::msg::AxisPose]`
```
wmx3LibCm_.motion->StartPos(&position_);
```

`/wmx/axis/position/relative [wmx_ros2_message::msg::AxisPose]`
```
wmx3LibCm_.motion->StartMov(&position_);
```

`/wmx/axis/velocity [wmx_ros2_message::msg::AxisVelocity]`
```
wmx3LibCm_.velocity->StartVel(&velocity_);
```

## Publisher
`/wmx/axis/state [wmx_ros2_message::msg::AxisState]`
```
wmx3LibCm_.GetStatus(&cmStatus_);
cmStatus_.axesStatus
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