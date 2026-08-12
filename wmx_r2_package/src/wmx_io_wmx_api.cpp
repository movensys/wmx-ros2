// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_io_node.hpp"

using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;
using wmx3Api::IO;

bool WmxIoNode::attachDevice()
{
  if (isDeviceAttached_) {
    return true;
  }

  unsigned int timeout = 10000;
  err_ = wmx3Lib_.CreateDevice(WMX3_SDK_PATH, DeviceType::DeviceTypeNormal, timeout);

  if (err_ != ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    if (err_ == ErrorCode::StartProcessLockError) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to attach to device (lock busy). Is the engine communicating?");
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to attach to device. Error=%d (%s)", err_, errString_);
    }
    return false;
  }

  wmx3Lib_.SetDeviceName("wmx_io_node");
  isDeviceAttached_ = true;
  RCLCPP_INFO(this->get_logger(), "Attached to WMX3 device");
  return true;
}

void WmxIoNode::releaseDevice()
{
  if (!isDeviceAttached_) {
    return;
  }

  wmxIo_.reset();
  err_ = wmx3Lib_.CloseDevice();
  if (err_ != ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    RCLCPP_ERROR(this->get_logger(), "Failed to close device");
  } else {
    RCLCPP_INFO(this->get_logger(), "Device closed");
  }
  isDeviceAttached_ = false;
}
