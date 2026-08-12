// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef WMX_ENGINE_NODE_HPP_
#define WMX_ENGINE_NODE_HPP_

#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "wmx_r2_message/srv/get_wmx_params.hpp"
#include "wmx_r2_message/srv/load_wmx_params.hpp"
#include "wmx_r2_message/srv/set_node_state.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "WMX3Api.h"
#include "CoreMotionApi.h"

class WmxEngineNode : public rclcpp::Node
{
public:
  WmxEngineNode();
  ~WmxEngineNode();

  unsigned int timeout_ = 10000;
  int maxRetries_ = 5;
  int retryDelay_ = 2000;
  const int createDeviceLockError_ = 297;
  int err_ = 0;
  char errString_[256];
  char buffer_[512];

  int engineCore_ = -1;
  int64_t engineAffinityMask_ = 0;
  std::string devicePath_;
  std::string deviceName_;

  std::atomic<bool> isEngineStarted_{false};
  std::atomic<bool> isCommStarted_{false};

private:
  wmx3Api::WMX3Api wmx3Lib_;
  std::unique_ptr<wmx3Api::CoreMotion> wmx3LibCm_;
  wmx3Api::EngineStatus engineStatus_;
  std::string statusStr_;
  std::string wmxParamFilePath_;
  // Lifecycle nodes brought up automatically, in this order. Empty means every
  // lifecycle node found on the graph.
  std::vector<std::string> managedNodes_;
  std::thread startThread_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr setEngineService_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr setCommService_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr getEngineStatusService_;
  rclcpp::Service<wmx_r2_message::srv::LoadWmxParams>::SharedPtr loadParamsService_;
  rclcpp::Service<wmx_r2_message::srv::GetWmxParams>::SharedPtr getParamsService_;
  rclcpp::Service<wmx_r2_message::srv::SetNodeState>::SharedPtr setNodeStateService_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr getNodeStatesService_;

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
  void setNodeStateCallback(
    const std::shared_ptr<wmx_r2_message::srv::SetNodeState::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetNodeState::Response> response);
  void getNodeStatesCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  int wmxStartEngine();
  int wmxStopEngine();
  int wmxStartCommunication();
  int wmxStopCommunication();
  bool wmxLoadParam(const std::string & path);
  void wmxGetParam(const std::vector<int32_t> & axes, std::vector<std::string> & dump);

  struct LifecycleClients
  {
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr changeState;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr getState;
  };

  rclcpp::CallbackGroup::SharedPtr lifecycleClientCbGroup_;
  rclcpp::CallbackGroup::SharedPtr managerCbGroup_;

  std::unordered_map<std::string, LifecycleClients> lifecycleClients_;
  std::set<std::string> broughtUpNodes_;

  rclcpp::TimerBase::SharedPtr discoveryTimer_;

  std::string resolveNodeName(const std::string & name) const;
  const LifecycleClients & clientsFor(const std::string & node);
  std::vector<std::string> discoverLifecycleNodes();
  std::string nodeStateLabel(const std::string & node);
  bool changeNodeState(const std::string & node, uint8_t transition, std::string & message);
  bool bringUpNode(const std::string & node, std::string & message);
  bool bringDownNode(const std::string & node, std::string & message);
  bool shutdownNode(const std::string & node, std::string & message);
  void bringDownDiscoveredNodes(bool cleanup);
  void discoveryStep();
};

#endif  // WMX_ENGINE_NODE_HPP_
