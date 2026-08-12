// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_core_motion_node.hpp"

using wmx3Api::AxisCommandMode;
using wmx3Api::Config;
using wmx3Api::CoreMotion;
using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;
using wmx3Api::ProfileType;

bool WmxCoreMotionNode::attachDevice()
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

  wmx3Lib_.SetDeviceName("wmx_core_motion_node");
  isDeviceAttached_ = true;
  RCLCPP_INFO(this->get_logger(), "Attached to WMX3 device");
  return true;
}

void WmxCoreMotionNode::releaseDevice()
{
  if (!isDeviceAttached_) {
    return;
  }

  wmx3LibCm_.reset();
  err_ = wmx3Lib_.CloseDevice();
  if (err_ != ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    RCLCPP_ERROR(this->get_logger(), "Failed to close device");
  } else {
    RCLCPP_INFO(this->get_logger(), "Device closed");
  }
  isDeviceAttached_ = false;
}

void WmxCoreMotionNode::stopAllJogs()
{
  if (!wmx3LibCm_) {
    return;
  }

  std::vector<int> jogging;
  {
    std::lock_guard<std::mutex> lock(jogMutex_);
    for (const auto & entry : jogState_) {
      jogging.push_back(entry.first);
    }
    jogState_.clear();
  }

  for (const int axis : jogging) {
    stopAxis(axis);
  }
}

// Decelerate an axis to a stop. Returns the WMX3 error code.
// Stopping an already idle axis is expected here (jog_run_time_ms may have
// elapsed before the operator released), so callers decide how loud to be.
int WmxCoreMotionNode::stopAxis(int axis)
{
  const int err = wmx3LibCm_->motion->Stop(axis);
  if (err != ErrorCode::None) {
    char errString[256];
    wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
    RCLCPP_DEBUG(
      this->get_logger(),
      "Stop on axis %d returned %d (%s)", axis, err, errString);
  }
  return err;
}
