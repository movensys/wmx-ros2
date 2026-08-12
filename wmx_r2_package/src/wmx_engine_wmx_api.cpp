// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_engine_node.hpp"

#include <cinttypes>

int WmxEngineNode::wmxStartEngine()
{
  RCLCPP_INFO(
    this->get_logger(), "Starting engine... (core=%d, affinityMask=0x%" PRIx64 ")",
    engineCore_, engineAffinityMask_);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  for (int attempt = 0; attempt < maxRetries_; attempt++) {
    if (attempt > 0) {
      RCLCPP_INFO(
        this->get_logger(), "Retrying device creation (attempt %d/%d)...",
        attempt + 1, maxRetries_);
      std::this_thread::sleep_for(std::chrono::milliseconds(retryDelay_));
    }

    err_ = wmx3Lib_.CreateDevice(
      devicePath_.c_str(), wmx3Api::DeviceType::DeviceTypeNormal, timeout_,
      engineCore_, engineAffinityMask_);

    if (err_ == wmx3Api::ErrorCode::None) {
      wmx3Lib_.SetDeviceName(deviceName_.c_str());
      wmx3LibCm_ = std::make_unique<wmx3Api::CoreMotion>(&wmx3Lib_);
      RCLCPP_INFO(this->get_logger(), "Device created (attempt %d)", attempt + 1);

      if (!wmxParamFilePath_.empty()) {
        wmxLoadParam(wmxParamFilePath_);
      }

      err_ = wmxStartCommunication();

      isEngineStarted_ = true;
      return err_;
    } else {
      wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
      if (err_ == createDeviceLockError_) {
        RCLCPP_WARN(
          this->get_logger(),
          "Device lock error (attempt %d/%d). Waiting...",
          attempt + 1, maxRetries_);
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Failed to create device (attempt %d/%d). Error=%d (%s)",
          attempt + 1, maxRetries_, err_, errString_);
      }
    }
  }

  wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
  RCLCPP_ERROR(
    this->get_logger(),
    "Failed to create device after %d attempts. Error=%d (%s)",
    maxRetries_, err_, errString_);
  isEngineStarted_ = true;
  return err_;
}

int WmxEngineNode::wmxStartCommunication()
{
  err_ = wmx3Lib_.StartCommunication(timeout_);
  if (err_ != wmx3Api::ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to start communication. Error=%d (%s)", err_, errString_);
  } else {
    isCommStarted_ = true;
    RCLCPP_INFO(this->get_logger(), "Communication started");
  }

  return err_;
}

int WmxEngineNode::wmxStopCommunication()
{
  err_ = wmx3Lib_.StopCommunication(timeout_);
  if (err_ != wmx3Api::ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to stop communication. Error=%d (%s)", err_, errString_);
  } else {
    isCommStarted_ = false;
    RCLCPP_INFO(this->get_logger(), "Communication stopped");
  }

  return err_;
}

int WmxEngineNode::wmxStopEngine()
{
  err_ = wmx3Lib_.StopEngine(timeout_);
  if (err_ != wmx3Api::ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    RCLCPP_ERROR(this->get_logger(), "Failed to stop engine");
  } else {
    RCLCPP_INFO(this->get_logger(), "Engine stopped");
  }

  wmx3LibCm_.reset();
  err_ = wmx3Lib_.CloseDevice();
  if (err_ != wmx3Api::ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    RCLCPP_ERROR(this->get_logger(), "Failed to close device");
  } else {
    isCommStarted_ = false;
    isEngineStarted_ = false;
    RCLCPP_INFO(this->get_logger(), "Device closed");
  }

  return err_;
}

bool WmxEngineNode::wmxLoadParam(const std::string & path)
{
  if (!wmx3LibCm_) {
    snprintf(buffer_, sizeof(buffer_), "No device: create it with wmx/engine/set_engine first.");
    return false;
  }

  wmx3Api::Config::SystemParam sysParamErr;
  wmx3Api::Config::AxisParam axisParamErr;

  err_ = wmx3LibCm_->config->ImportAndSetAll(
    const_cast<char *>(path.c_str()), &sysParamErr, &axisParamErr);

  if (err_ != wmx3Api::ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    snprintf(
      buffer_, sizeof(buffer_),
      "Failed to load params from %s. Error=%d (%s)", path.c_str(), err_, errString_);
    RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
    return false;
  }

  snprintf(buffer_, sizeof(buffer_), "Loaded params from: %s", path.c_str());
  RCLCPP_INFO(this->get_logger(), "%s", buffer_);
  return true;
}

void WmxEngineNode::wmxGetParam(
  const std::vector<int32_t> & axes, std::vector<std::string> & dump)
{
  wmx3Api::Config::AxisParam axisParam;
  wmx3LibCm_->config->GetAxisParam(&axisParam);

  for (int32_t i : axes) {
    dump.push_back("=== Axis " + std::to_string(i) + " ===");

    dump.push_back("[AxisParam]");
    dump.push_back(
      "  GearRatio          = " + std::to_string(axisParam.gearRatioNumerator[i]) +
      " / " + std::to_string(axisParam.gearRatioDenominator[i]));
    dump.push_back(
      "  AxisPolarity       = " +
      std::to_string(static_cast<int>(axisParam.axisPolarity[i])));
    dump.push_back(
      "  CommandMode        = " +
      std::to_string(static_cast<int>(axisParam.axisCommandMode[i])));
    dump.push_back("");
  }
}
