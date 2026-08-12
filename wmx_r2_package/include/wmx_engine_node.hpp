// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef WMX_ENGINE_NODE_HPP_
#define WMX_ENGINE_NODE_HPP_

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "wmx_r2_message/srv/set_engine.hpp"
#include "wmx_r2_message/srv/set_node_state.hpp"

#include "WMX3Api.h"

// Owns the WMX3 engine and manages the lifecycle nodes that attach to it
// (wmx_core_motion_node, wmx_io_node, wmx_ethercat_node). They can only create
// their own device handle once the engine is communicating, so the engine is
// the one that drives their configure/activate transitions.
class WmxEngineNode : public rclcpp::Node
{
public:
  WmxEngineNode();
  ~WmxEngineNode();

private:
  // A lifecycle node this engine brings up and takes down.
  struct ManagedNode
  {
    std::string name;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr changeState;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr getState;
  };

  wmx3Api::WMX3Api wmx3Lib_;
  std::atomic<bool> commStarted_{false};
  std::atomic<bool> startComplete_{false};
  std::thread startThread_;

  bool autoManageNodes_ = true;
  std::vector<ManagedNode> managedNodes_;

  rclcpp::TimerBase::SharedPtr bringUpTimer_;

  // Manager services block while transitions run, so they get their own
  // mutually exclusive group; the lifecycle clients need a reentrant group so
  // their responses can be handled while a manager callback is waiting.
  rclcpp::CallbackGroup::SharedPtr managerCbGroup_;
  rclcpp::CallbackGroup::SharedPtr lifecycleClientCbGroup_;

  rclcpp::Service<wmx_r2_message::srv::SetEngine>::SharedPtr setEngineService_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr setCommService_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr getEngineStatusService_;
  rclcpp::Service<wmx_r2_message::srv::SetNodeState>::SharedPtr setNodeStateService_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr getNodeStatesService_;

  void startEngine();
  void stopEngine();
  void stopCommunication();

  // Managed node control.
  ManagedNode * findManagedNode(const std::string & name);
  std::string nodeStateLabel(const ManagedNode & node);
  bool changeNodeState(const ManagedNode & node, uint8_t transition, std::string & message);
  bool bringUpNode(const ManagedNode & node, std::string & message);
  bool bringDownNode(const ManagedNode & node, std::string & message);
  bool shutdownNode(const ManagedNode & node, std::string & message);
  void bringUpManagedNodes();
  void bringDownManagedNodes(bool cleanup);
  void bringUpStep();

  void setEngine(
    const std::shared_ptr<wmx_r2_message::srv::SetEngine::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetEngine::Response> response);
  void setComm(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void getEngineStatus(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void setNodeState(
    const std::shared_ptr<wmx_r2_message::srv::SetNodeState::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetNodeState::Response> response);
  void getNodeStates(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
};

#endif  // WMX_ENGINE_NODE_HPP_
