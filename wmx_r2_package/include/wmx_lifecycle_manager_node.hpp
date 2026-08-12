// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef WMX_LIFECYCLE_MANAGER_NODE_HPP_
#define WMX_LIFECYCLE_MANAGER_NODE_HPP_

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "wmx_r2_message/srv/set_node_state.hpp"

// Drives the lifecycle nodes on the graph on behalf of an owning node. Knows
// nothing about WMX.
class LifecycleManager
{
public:
  // managedNodes: nodes to bring up automatically, in that order. Empty means
  // every lifecycle node found on the graph.
  LifecycleManager(rclcpp::Node * node, const std::vector<std::string> & managedNodes);

  // Lifecycle nodes currently on the graph, in bring-up order.
  std::vector<std::string> discover();

  std::string state(const std::string & node);
  bool changeState(const std::string & node, uint8_t transition, std::string & message);
  bool bringUp(const std::string & node, std::string & message);
  bool bringDown(const std::string & node, std::string & message);
  bool shutdown(const std::string & node, std::string & message);

  // Bring up every discovered node not handled yet.
  void bringUpDiscovered();
  // Take the discovered nodes down, in reverse order.
  void bringDownDiscovered(bool cleanup);

  // A plain node name is resolved against the owning node's namespace.
  std::string resolveNodeName(const std::string & name) const;

  // Marks a node as handled, so bringUpDiscovered() leaves it alone.
  void markHandled(const std::string & node) {handledNodes_.insert(node);}

private:
  struct Clients
  {
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr changeState;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr getState;
  };

  const Clients & clientsFor(const std::string & node);

  rclcpp::Node * node_;
  rclcpp::Logger logger_;
  rclcpp::CallbackGroup::SharedPtr clientCbGroup_;

  std::vector<std::string> managedNodes_;
  std::unordered_map<std::string, Clients> clients_;
  // Nodes already brought up once. An entry is dropped when the node leaves the
  // graph, so a respawned node is brought up again.
  std::set<std::string> handledNodes_;
};

// Brings the lifecycle nodes up once wmx_engine_node reports Communicating, and
// takes them down when it stops. Runs as its own node so the engine only deals
// with the WMX3 device.
class WmxLifecycleManagerNode : public rclcpp::Node
{
public:
  WmxLifecycleManagerNode();

private:
  std::unique_ptr<LifecycleManager> lifecycle_;

  std::string engineStatusService_;
  bool requireEngine_ = true;
  bool nodesAreUp_ = false;

  rclcpp::CallbackGroup::SharedPtr managerCbGroup_;
  rclcpp::CallbackGroup::SharedPtr clientCbGroup_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr engineStatusClient_;
  rclcpp::TimerBase::SharedPtr discoveryTimer_;

  rclcpp::Service<wmx_r2_message::srv::SetNodeState>::SharedPtr setNodeStateService_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr getNodeStatesService_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr bringUpAllService_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr bringDownAllService_;

  bool isEngineCommunicating();
  void discoveryStep();

  void setNodeStateCallback(
    const std::shared_ptr<wmx_r2_message::srv::SetNodeState::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetNodeState::Response> response);
  void getNodeStatesCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void bringUpAllCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void bringDownAllCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
};

#endif  // WMX_LIFECYCLE_MANAGER_NODE_HPP_
