// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef WMX_ETHERCAT_NODE_HPP_
#define WMX_ETHERCAT_NODE_HPP_

#include <atomic>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "wmx_r2_message/srv/ecat_get_network_state.hpp"
#include "wmx_r2_message/srv/ecat_register_read.hpp"
#include "wmx_r2_message/srv/ecat_reset_statistics.hpp"
#include "wmx_r2_message/srv/ecat_scan_network.hpp"
#include "wmx_r2_message/srv/ecat_start_hotconnect.hpp"

#include "WMX3Api.h"
#include "EcApi.h"

using std::placeholders::_1;
using std::placeholders::_2;

// Managed node. wmx_engine_node drives the transitions:
//   configure  attach to the WMX3 device and advertise the EtherCAT services
//   activate   start serving EtherCAT requests
//   deactivate reject EtherCAT requests, keep the device attached
//   cleanup    drop the services and detach from the device
class WmxEtherCatNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  WmxEtherCatNode();
  ~WmxEtherCatNode() override;

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
  wmx3Api::ecApi::Ecat wmxEcat_;

  rclcpp::Service<wmx_r2_message::srv::EcatGetNetworkState>::SharedPtr getNetworkStateService_;
  rclcpp::Service<wmx_r2_message::srv::EcatRegisterRead>::SharedPtr registerReadService_;
  rclcpp::Service<wmx_r2_message::srv::EcatResetStatistics>::SharedPtr resetStatisticsService_;
  rclcpp::Service<wmx_r2_message::srv::EcatScanNetwork>::SharedPtr scanNetworkService_;
  rclcpp::Service<wmx_r2_message::srv::EcatStartHotconnect>::SharedPtr startHotconnectService_;

  bool attachDevice();
  void releaseDevice();
  std::string notActiveMessage() const;

  void getNetworkState(
    const std::shared_ptr<wmx_r2_message::srv::EcatGetNetworkState::Request> request,
    std::shared_ptr<wmx_r2_message::srv::EcatGetNetworkState::Response> response);

  void registerRead(
    const std::shared_ptr<wmx_r2_message::srv::EcatRegisterRead::Request> request,
    std::shared_ptr<wmx_r2_message::srv::EcatRegisterRead::Response> response);

  void resetStatistics(
    const std::shared_ptr<wmx_r2_message::srv::EcatResetStatistics::Request> request,
    std::shared_ptr<wmx_r2_message::srv::EcatResetStatistics::Response> response);

  void scanNetwork(
    const std::shared_ptr<wmx_r2_message::srv::EcatScanNetwork::Request> request,
    std::shared_ptr<wmx_r2_message::srv::EcatScanNetwork::Response> response);

  void startHotconnect(
    const std::shared_ptr<wmx_r2_message::srv::EcatStartHotconnect::Request> request,
    std::shared_ptr<wmx_r2_message::srv::EcatStartHotconnect::Response> response);
};

#endif  // WMX_ETHERCAT_NODE_HPP_
