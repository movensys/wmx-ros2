// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_ethercat_node.hpp"

using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;

WmxEtherCatNode::CallbackReturn WmxEtherCatNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring wmx_ethercat_node...");

  if (!attachDevice()) {
    return CallbackReturn::FAILURE;
  }

  getNetworkStateService_ = this->create_service<wmx_r2_message::srv::EcatGetNetworkState>(
    "wmx/ecat/get_network_state",
    std::bind(&WmxEtherCatNode::getNetworkState, this, _1, _2));

  registerReadService_ = this->create_service<wmx_r2_message::srv::EcatRegisterRead>(
    "wmx/ecat/register_read",
    std::bind(&WmxEtherCatNode::registerRead, this, _1, _2));

  resetStatisticsService_ = this->create_service<wmx_r2_message::srv::EcatResetStatistics>(
    "wmx/ecat/reset_statistics",
    std::bind(&WmxEtherCatNode::resetStatistics, this, _1, _2));

  scanNetworkService_ = this->create_service<wmx_r2_message::srv::EcatScanNetwork>(
    "wmx/ecat/scan_network",
    std::bind(&WmxEtherCatNode::scanNetwork, this, _1, _2));

  startHotconnectService_ = this->create_service<wmx_r2_message::srv::EcatStartHotconnect>(
    "wmx/ecat/start_hotconnect",
    std::bind(&WmxEtherCatNode::startHotconnect, this, _1, _2));

  RCLCPP_INFO(this->get_logger(), "wmx_ethercat_node is configured");
  return CallbackReturn::SUCCESS;
}

WmxEtherCatNode::CallbackReturn WmxEtherCatNode::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  LifecycleNode::on_activate(previous_state);
  isActive_ = true;
  RCLCPP_INFO(this->get_logger(), "wmx_ethercat_node is active");
  return CallbackReturn::SUCCESS;
}

WmxEtherCatNode::CallbackReturn WmxEtherCatNode::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  isActive_ = false;
  LifecycleNode::on_deactivate(previous_state);
  RCLCPP_INFO(this->get_logger(), "wmx_ethercat_node is inactive");
  return CallbackReturn::SUCCESS;
}

WmxEtherCatNode::CallbackReturn WmxEtherCatNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  isActive_ = false;

  getNetworkStateService_.reset();
  registerReadService_.reset();
  resetStatisticsService_.reset();
  scanNetworkService_.reset();
  startHotconnectService_.reset();

  releaseDevice();

  RCLCPP_INFO(this->get_logger(), "wmx_ethercat_node is cleaned up");
  return CallbackReturn::SUCCESS;
}

WmxEtherCatNode::CallbackReturn WmxEtherCatNode::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}
