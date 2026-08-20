// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef WMX_ENGINE_NODE_HPP_
#define WMX_ENGINE_NODE_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "wmx_r2_message/srv/set_engine.hpp"

#include "WMX3Api.h"

class WmxEngineNodeApi
{
public:
  struct Config
  {
    int core = -1;
    int64_t affinityMask = 0;
  };

  WmxEngineNodeApi(const rclcpp::Logger & logger, const Config & config);
  ~WmxEngineNodeApi();

  void setConfig(const Config & config);

  int startEngine(std::string & message);
  int stopEngine(std::string & message);

  int createDevice(const std::string & path, const std::string & name, std::string & message);
  int closeDevice(std::string & message);

  int startCommunication(std::string & message);
  int stopCommunication(std::string & message);

  std::string getEngineStatus();

  bool isCommStarted() const {return isCommStarted_.load();}

private:
  std::string errorText(int err);

  rclcpp::Logger logger_;
  Config config_;

  const char * deviceName_ = "wmx_engine_node";
  unsigned int timeout_ = 10000;
  int maxRetries_ = 5;
  int retryDelay_ = 2000;

  std::recursive_mutex deviceMutex_;

  wmx3Api::WMX3Api wmx3Lib_;

  std::atomic<bool> isCommStarted_{false};
};

class WmxEngineNode : public rclcpp::Node
{
public:
  WmxEngineNode();
  ~WmxEngineNode();

private:
  std::unique_ptr<WmxEngineNodeApi> api_;

  std::atomic<bool> startComplete_{false};
  std::thread startThread_;

  rclcpp::TimerBase::SharedPtr readyTimer_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr engineReadyPub_;

  rclcpp::Service<wmx_r2_message::srv::SetEngine>::SharedPtr setEngineService_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr setCommService_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr getEngineStatusService_;

  WmxEngineNodeApi::Config readConfig();
  void startEngine();
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
