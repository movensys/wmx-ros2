# WMX ROS2 General Package

## Subscriber

## Publisher

## Service
`/wmx/engine/set_device [wmx_ros2_message::srv::SetEngine]`

```
wmx3Lib_.CreateDevice(request->path.c_str(), DeviceType::DeviceTypeNormal, INFINITE);
wmx3Lib_.SetDeviceName(request->name.c_str());
```

