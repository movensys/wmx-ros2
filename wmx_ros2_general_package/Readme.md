# WMX ROS2 General Package

## Subscriber

**/wmx/axis/position [wmx_ros2_message/msg/AxisPose]**
```
int32[] index       # axis index (e.g [0, 1])
float64[] target    # pose target (rad)
string profile      # profile type (not working yet)
float64[] velocity  # velocity (rad/s)
float64[] acc       # acceleration (rad/s^2)
float64[] dec       # decceleration (rad/s^2)

WMX API: CoreMotion.motion->StartPos(&WMX3Api::Motion::PosCommand)
```
<br>
<br>

**/wmx/axis/position/relative [wmx_ros2_message/msg/AxisPose]**
```
int32[] index       # axis index (e.g [0, 1])
float64[] target    # pose target (rad)
string profile      # profile type (not working yet)
float64[] velocity  # velocity (rad/s)
float64[] acc       # acceleration (rad/s^2)
float64[] dec       # decceleration (rad/s^2)

WMX API:  CoreMotion.motion->StartMov(&WMX3Api::Motion::PosCommand)
```
<br>
<br>

**/wmx/axis/velocity [wmx_ros2_message/msg/AxisVelocity]**
```
int32[] index       # axis index (e.g [0, 1])
string profile      # profile type (not working yet)
float64[] velocity  # velocity (rad/s)
float64[] acc       # acceleration (rad/s^2)
float64[] dec       # decceleration (rad/s^2)

WMX API:  CoreMotion.velocity->StartVel(&WMX3Api::Velocity::VelCommand)
```

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
**/wmx/engine/set_device [wmx_ros2_message/srv/SetEngine]**
```
bool data           # engine start or stop
string path         # WMX path
string name         # device name
---
bool success        # success or fail
string message      # response messages

WMX API:  WMX3Api.CreateDevice(request->path.c_str(), DeviceType::DeviceTypeNormal, INFINITE);
          WMX3Api.SetDeviceName(request->name.c_str());
```
<br>
<br>

**/wmx/engine/set_comm [std_srvs/srv/SetBool]**
```
bool data           # communication start or stop
---
bool success        # success or fail
string message      # response messages

WMX API:  WMX3Api.StartCommunication(INFINITE);
```
<br>
<br>

**/wmx/engine/get_status [std_srvs/srv/Trigger]**
```
---
bool success        # success or fail
string message      # response messages

WMX API:  WMX3Api.GetEngineStatus(&WMX3Api::EngineStatus);
```
<br>
<br>

**/wmx/axis/set_on [wmx_ros2_message/srv/SetAxis]**
```
int32[] index       # axis index
int32[] data        # on or off
---
bool success        # success or fail
string message      # response messages

WMX API:  CoreMotion.axisControl->SetServoOn(request->index, request->data);
```
<br>
<br>

**/wmx/axis/clear_alarm [wmx_ros2_message/srv/SetAxis]**
```
int32[] index       # axis index
int32[] data        # (not used)
---
bool success        # success or fail
string message      # response messages

WMX API:  CoreMotion.axisControl->ClearAmpAlarm(request->index)
```
<br>
<br>

**/wmx/axis/set_mode [wmx_ros2_message/srv/SetAxis]**
```
int32[] index       # axis index
int32[] data        # 0: position, 1: velocity
---
bool success        # success or fail
string message      # response messages

WMX API:  CoreMotion.axisControl->SetAxisCommandMode(request->index, AxisCommandMode::Position)
          CoreMotion.axisControl->SetAxisCommandMode(request->index, AxisCommandMode::Velocity);
```
<br>
<br>

**/wmx/axis/set_polarity [wmx_ros2_message/srv/SetAxis]**
```
int32[] index       # axis index
int32[] data        # 1: normal or -1: reverse
---
bool success        # success or fail
string message      # response messages

WMX API:  CoreMotion.config->SetAxisPolarity(request->index, request->data);
```
<br>
<br>

**/wmx/axis/set_gear_ratio [wmx_ros2_message/srv/SetAxisGearRatio]**
```
int32[] index            # axis index
float64[] numerator      # numerator
float64[] denumerator    # denumerator
---
bool success             # success or fail
string message           # response messages

WMX API:  CoreMotion.config->SetGearRatio(request->index, request->numerator, request->denumerator);
```

**wmx/axis/homing [wmx_ros2_message/srv/SetAxis]**
```
int32[] index       # axis index
int32[] data        # (not used)
---
bool success        # success or fail
string message      # response messages

WMX API:  CoreMotion.config->GetHomeParam(request->index, &Config::HomeParam);
          Config::HomeParam.homeType = Config::HomeType::CurrentPos;
          CoreMotion.config->SetHomeParam(request->index, &Config::HomeParam);
          CoreMotion.home->StartHome(request->index);
          CoreMotion.motion->Wait(request->index);
```