// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_core_motion_node.hpp"

using wmx3Api::AxisCommandMode;
using wmx3Api::Config;
using wmx3Api::CoreMotion;
using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;
using wmx3Api::ProfileType;

WmxCoreMotionNode::CallbackReturn WmxCoreMotionNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring wmx_core_motion_node...");

  if (!attachDevice()) {
    return CallbackReturn::FAILURE;
  }

  // get engineStatus to read # of axis (axisCount_)
  wmx3Lib_.GetEngineStatus(&engineStatus_);
  axisCount_ = 0;

  // numOfInterrupts means # of cyclic handlers (max 2)
  // numOfAxes means # of axes on that handler (not # of slaves)
  for (int i = 0; i < engineStatus_.numOfInterrupts; ++i) {
    axisCount_ += engineStatus_.interrupts[i].numOfAxes;
  }
  if (axisCount_ <= 0) {
    RCLCPP_WARN(this->get_logger(), "Engine reported 0 axes; axis state will be empty.");
  }

  wmx3LibCm_ = std::make_unique<CoreMotion>(&wmx3Lib_);

  setAxisOnService_ = this->create_service<wmx_r2_message::srv::SetAxis>(
    "wmx/axis/set_on",
    std::bind(&WmxCoreMotionNode::setAxisOn, this, _1, _2));

  clearAlarmService_ = this->create_service<wmx_r2_message::srv::SetAxis>(
    "wmx/axis/clear_alarm",
    std::bind(&WmxCoreMotionNode::clearAlarm, this, _1, _2));

  setAxisModeService_ = this->create_service<wmx_r2_message::srv::SetAxis>(
    "wmx/axis/set_mode",
    std::bind(&WmxCoreMotionNode::setAxisMode, this, _1, _2));

  setAxisPolarityService_ = this->create_service<wmx_r2_message::srv::SetAxis>(
    "wmx/axis/set_polarity",
    std::bind(&WmxCoreMotionNode::setAxisPolarity, this, _1, _2));

  setAxisGearRatioService_ = this->create_service<wmx_r2_message::srv::SetAxisGearRatio>(
    "wmx/axis/set_gear_ratio",
    std::bind(&WmxCoreMotionNode::setAxisGearRatio, this, _1, _2));

  setHomingService_ = this->create_service<wmx_r2_message::srv::SetAxis>(
    "wmx/axis/homing",
    std::bind(&WmxCoreMotionNode::setHoming, this, _1, _2),
    rclcpp::ServicesQoS(), homing_cb_group_);

  stopAxisService_ = this->create_service<wmx_r2_message::srv::SetAxis>(
    "wmx/axis/stop",
    std::bind(&WmxCoreMotionNode::stopAxes, this, _1, _2));

  axisStatePub_ = this->create_publisher<wmx_r2_message::msg::AxisState>(
    "wmx/axis/state", 1);

  axisVelSub_ = this->create_subscription<wmx_r2_message::msg::AxisVelocity>(
    "wmx/axis/velocity", 1,
    std::bind(&WmxCoreMotionNode::axisVelCallback, this, _1));

  axisPoseSub_ = this->create_subscription<wmx_r2_message::msg::AxisPose>(
    "wmx/axis/position", 1,
    std::bind(&WmxCoreMotionNode::axisPoseCallback, this, _1));

  axisPoseRelativeSub_ = this->create_subscription<wmx_r2_message::msg::AxisPose>(
    "wmx/axis/position/relative", 1,
    std::bind(&WmxCoreMotionNode::axisPoseRelativeCallback, this, _1));

  axisJogSub_ = this->create_subscription<wmx_r2_message::msg::AxisVelocity>(
    "wmx/axis/jog", 1,
    std::bind(&WmxCoreMotionNode::axisJogCallback, this, _1));

  axisStateTimer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / rate_),
    std::bind(&WmxCoreMotionNode::axisStateStep, this));
  axisStateTimer_->cancel();

  jogWatchdogTimer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&WmxCoreMotionNode::jogWatchdogStep, this));
  jogWatchdogTimer_->cancel();

  RCLCPP_INFO(
    this->get_logger(), "wmx_core_motion_node is configured (%d axes, %d Hz)",
    axisCount_, rate_);
  return CallbackReturn::SUCCESS;
}

WmxCoreMotionNode::CallbackReturn WmxCoreMotionNode::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  LifecycleNode::on_activate(previous_state);
  isActive_ = true;

  axisStateTimer_->reset();
  jogWatchdogTimer_->reset();

  RCLCPP_INFO(this->get_logger(), "wmx_core_motion_node is active");
  return CallbackReturn::SUCCESS;
}

WmxCoreMotionNode::CallbackReturn WmxCoreMotionNode::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  isActive_ = false;

  axisStateTimer_->cancel();
  jogWatchdogTimer_->cancel();
  stopAllJogs();

  LifecycleNode::on_deactivate(previous_state);

  RCLCPP_INFO(this->get_logger(), "wmx_core_motion_node is inactive");
  return CallbackReturn::SUCCESS;
}

WmxCoreMotionNode::CallbackReturn WmxCoreMotionNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  isActive_ = false;

  axisStateTimer_.reset();
  jogWatchdogTimer_.reset();
  stopAllJogs();

  axisVelSub_.reset();
  axisPoseSub_.reset();
  axisPoseRelativeSub_.reset();
  axisJogSub_.reset();

  setAxisOnService_.reset();
  clearAlarmService_.reset();
  setAxisModeService_.reset();
  setAxisPolarityService_.reset();
  setAxisGearRatioService_.reset();
  setHomingService_.reset();
  stopAxisService_.reset();

  axisStatePub_.reset();

  releaseDevice();

  RCLCPP_INFO(this->get_logger(), "wmx_core_motion_node is cleaned up");
  return CallbackReturn::SUCCESS;
}

WmxCoreMotionNode::CallbackReturn WmxCoreMotionNode::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}
