// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_io_node.hpp"

using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;
using wmx3Api::IO;

WmxIoNode::CallbackReturn WmxIoNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring wmx_io_node...");

  if (!attachDevice()) {
    return CallbackReturn::FAILURE;
  }

  wmxIo_ = std::make_unique<IO>(&wmx3Lib_);

  getInputBitService_ = this->create_service<wmx_r2_message::srv::GetIoBit>(
    "wmx/io/get_input_bit",
    std::bind(&WmxIoNode::getInputBit, this, _1, _2));

  getOutputBitService_ = this->create_service<wmx_r2_message::srv::GetIoBit>(
    "wmx/io/get_output_bit",
    std::bind(&WmxIoNode::getOutputBit, this, _1, _2));

  getInputBytesService_ = this->create_service<wmx_r2_message::srv::GetIoBytes>(
    "wmx/io/get_input_bytes",
    std::bind(&WmxIoNode::getInputBytes, this, _1, _2));

  getOutputBytesService_ = this->create_service<wmx_r2_message::srv::GetIoBytes>(
    "wmx/io/get_output_bytes",
    std::bind(&WmxIoNode::getOutputBytes, this, _1, _2));

  setOutputBitService_ = this->create_service<wmx_r2_message::srv::SetIoBit>(
    "wmx/io/set_output_bit",
    std::bind(&WmxIoNode::setOutputBit, this, _1, _2));

  setOutputBytesService_ = this->create_service<wmx_r2_message::srv::SetIoBytes>(
    "wmx/io/set_output_bytes",
    std::bind(&WmxIoNode::setOutputBytes, this, _1, _2));

  RCLCPP_INFO(this->get_logger(), "wmx_io_node is configured");
  return CallbackReturn::SUCCESS;
}

WmxIoNode::CallbackReturn WmxIoNode::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  LifecycleNode::on_activate(previous_state);
  isActive_ = true;
  RCLCPP_INFO(this->get_logger(), "wmx_io_node is active");
  return CallbackReturn::SUCCESS;
}

WmxIoNode::CallbackReturn WmxIoNode::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  isActive_ = false;
  LifecycleNode::on_deactivate(previous_state);
  RCLCPP_INFO(this->get_logger(), "wmx_io_node is inactive");
  return CallbackReturn::SUCCESS;
}

WmxIoNode::CallbackReturn WmxIoNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  isActive_ = false;

  getInputBitService_.reset();
  getOutputBitService_.reset();
  getInputBytesService_.reset();
  getOutputBytesService_.reset();
  setOutputBitService_.reset();
  setOutputBytesService_.reset();

  releaseDevice();

  RCLCPP_INFO(this->get_logger(), "wmx_io_node is cleaned up");
  return CallbackReturn::SUCCESS;
}

WmxIoNode::CallbackReturn WmxIoNode::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}
