// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef WMX_ENGINE_NODE_HPP_
#define WMX_ENGINE_NODE_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "wmx_r2_message/srv/get_wmx_params.hpp"
#include "wmx_r2_message/srv/load_wmx_params.hpp"

#include "WMX3Api.h"
#include "CoreMotionApi.h"

// Everything that talks to the WMX3 SDK. No ROS entities, only a logger.
class WmxEngineApi
{
public:
  struct Config
  {
    std::string devicePath = "/opt/wmx3/";
    std::string deviceName = "wmx_r2";
    std::string paramFilePath;
    int core = -1;
    int64_t affinityMask = 0;
  };

  WmxEngineApi(const rclcpp::Logger & logger, const Config & config);
  ~WmxEngineApi();

  // Create the device, import the parameter file and start communication.
  int startEngine();
  int stopEngine();
  int startCommunication();
  int stopCommunication();

  bool loadParam(const std::string & path);
  void getParam(const std::vector<int32_t> & axes, std::vector<std::string> & dump);

  std::string engineState();

  bool isDeviceOpen() const {return cm_ != nullptr;}
  bool isEngineStarted() const {return isEngineStarted_;}
  bool isCommStarted() const {return isCommStarted_;}

  // Result of the last call, ready to put in a service response.
  const char * message() const {return buffer_;}

private:
  rclcpp::Logger logger_;
  Config config_;

  unsigned int timeout_ = 10000;
  int maxRetries_ = 5;
  int retryDelay_ = 2000;
  const int createDeviceLockError_ = 297;
  int err_ = 0;
  char errString_[256] = {};
  char buffer_[512] = {};

  wmx3Api::WMX3Api wmx3Lib_;
  std::unique_ptr<wmx3Api::CoreMotion> cm_;
  wmx3Api::EngineStatus engineStatus_;

  std::atomic<bool> isEngineStarted_{false};
  std::atomic<bool> isCommStarted_{false};
};

class WmxEngineNode : public rclcpp::Node
{
public:
  WmxEngineNode();
  ~WmxEngineNode();

private:
  std::unique_ptr<WmxEngineApi> api_;
  std::thread startThread_;

  rclcpp::CallbackGroup::SharedPtr managerCbGroup_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr setEngineService_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr setCommService_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr getEngineStatusService_;
  rclcpp::Service<wmx_r2_message::srv::LoadWmxParams>::SharedPtr loadParamsService_;
  rclcpp::Service<wmx_r2_message::srv::GetWmxParams>::SharedPtr getParamsService_;

  void setEngineCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void setCommCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void getEngineStatusCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void loadWmxParamsCallback(
    const std::shared_ptr<wmx_r2_message::srv::LoadWmxParams::Request> request,
    std::shared_ptr<wmx_r2_message::srv::LoadWmxParams::Response> response);
  void getWmxParamsCallback(
    const std::shared_ptr<wmx_r2_message::srv::GetWmxParams::Request> request,
    std::shared_ptr<wmx_r2_message::srv::GetWmxParams::Response> response);
};

#endif  // WMX_ENGINE_NODE_HPP_
