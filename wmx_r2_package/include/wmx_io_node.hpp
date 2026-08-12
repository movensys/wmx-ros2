// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef WMX_IO_NODE_HPP_
#define WMX_IO_NODE_HPP_

#include <atomic>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "wmx_r2_message/srv/get_io_bit.hpp"
#include "wmx_r2_message/srv/get_io_bytes.hpp"
#include "wmx_r2_message/srv/set_io_bit.hpp"
#include "wmx_r2_message/srv/set_io_bytes.hpp"

#include "WMX3Api.h"
#include "IOApi.h"

using std::placeholders::_1;
using std::placeholders::_2;

// Managed node. wmx_engine_node drives the transitions:
//   configure  attach to the WMX3 device and advertise the IO services
//   activate   start serving IO requests
//   deactivate reject IO requests, keep the device attached
//   cleanup    drop the services and detach from the device
class WmxIoNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  WmxIoNode();
  ~WmxIoNode() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

private:
  std::atomic<bool> active_{false};
  bool deviceAttached_ = false;
  int err_;
  char errString_[256];
  char buffer_[512];

  wmx3Api::WMX3Api wmx3Lib_;
  std::unique_ptr<wmx3Api::IO> wmxIo_;

  rclcpp::Service<wmx_r2_message::srv::GetIoBit>::SharedPtr getInputBitService_;
  rclcpp::Service<wmx_r2_message::srv::GetIoBit>::SharedPtr getOutputBitService_;
  rclcpp::Service<wmx_r2_message::srv::GetIoBytes>::SharedPtr getInputBytesService_;
  rclcpp::Service<wmx_r2_message::srv::GetIoBytes>::SharedPtr getOutputBytesService_;
  rclcpp::Service<wmx_r2_message::srv::SetIoBit>::SharedPtr setOutputBitService_;
  rclcpp::Service<wmx_r2_message::srv::SetIoBytes>::SharedPtr setOutputBytesService_;

  bool attachDevice();
  void releaseDevice();
  std::string notActiveMessage() const;

  void getInputBit(
    const std::shared_ptr<wmx_r2_message::srv::GetIoBit::Request> request,
    std::shared_ptr<wmx_r2_message::srv::GetIoBit::Response> response);
  void getOutputBit(
    const std::shared_ptr<wmx_r2_message::srv::GetIoBit::Request> request,
    std::shared_ptr<wmx_r2_message::srv::GetIoBit::Response> response);
  void getInputBytes(
    const std::shared_ptr<wmx_r2_message::srv::GetIoBytes::Request> request,
    std::shared_ptr<wmx_r2_message::srv::GetIoBytes::Response> response);
  void getOutputBytes(
    const std::shared_ptr<wmx_r2_message::srv::GetIoBytes::Request> request,
    std::shared_ptr<wmx_r2_message::srv::GetIoBytes::Response> response);
  void setOutputBit(
    const std::shared_ptr<wmx_r2_message::srv::SetIoBit::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetIoBit::Response> response);
  void setOutputBytes(
    const std::shared_ptr<wmx_r2_message::srv::SetIoBytes::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetIoBytes::Response> response);
};

#endif  // WMX_IO_NODE_HPP_
