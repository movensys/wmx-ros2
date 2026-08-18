// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef WMX_ENGINE_NODE_HPP_
#define WMX_ENGINE_NODE_HPP_

#include <iostream>
#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "wmx_r2_message/srv/set_engine.hpp"

#include "WMX3Api.h"

class WmxEngineNode : public rclcpp::Node
{
public:
  WmxEngineNode();
  ~WmxEngineNode();

private:
  wmx3Api::WMX3Api wmx3Lib_;
  std::atomic<bool> commStarted_{false};
  std::atomic<bool> startComplete_{false};
  std::thread startThread_;

  rclcpp::TimerBase::SharedPtr readyTimer_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr engineReadyPub_;

  rclcpp::Service<wmx_r2_message::srv::SetEngine>::SharedPtr setEngineService_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr setCommService_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr getEngineStatusService_;

  void startEngine();
  void stopEngine();
  void stopCommunication();
  void publishReady();

  void setEngineCallback(
    const std::shared_ptr<wmx_r2_message::srv::SetEngine::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetEngine::Response> response);
  void setCommCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void getEngineStatusCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
};

#endif  // WMX_ENGINE_NODE_HPP_
