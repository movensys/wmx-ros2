// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_lifecycle_manager_node.hpp"

#include <algorithm>

using std::placeholders::_1;
using std::placeholders::_2;
using lifecycle_msgs::msg::Transition;

namespace
{
constexpr std::chrono::seconds kServiceWaitTimeout{5};
constexpr std::chrono::seconds kTransitionTimeout{30};
constexpr std::chrono::seconds kQueryServiceWaitTimeout{1};
constexpr std::chrono::seconds kQueryTimeout{5};

bool isDeviceLevelNode(const std::string & fullName)
{
  const std::size_t start = fullName.find_last_of('/');
  const std::string name =
    (start == std::string::npos) ? fullName : fullName.substr(start + 1);
  return name.rfind("wmx_", 0) == 0;
}

const char * transitionLabel(uint8_t transition)
{
  switch (transition) {
    case Transition::TRANSITION_CONFIGURE:             return "configure";
    case Transition::TRANSITION_CLEANUP:               return "cleanup";
    case Transition::TRANSITION_ACTIVATE:              return "activate";
    case Transition::TRANSITION_DEACTIVATE:            return "deactivate";
    case Transition::TRANSITION_UNCONFIGURED_SHUTDOWN: return "shutdown";
    case Transition::TRANSITION_INACTIVE_SHUTDOWN:     return "shutdown";
    case Transition::TRANSITION_ACTIVE_SHUTDOWN:       return "shutdown";
    default:                                           return "unknown transition";
  }
}
}  // namespace

LifecycleManager::LifecycleManager(
  rclcpp::Node * node, const std::vector<std::string> & managedNodes)
: node_(node), logger_(node->get_logger()), managedNodes_(managedNodes)
{
  clientCbGroup_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
}

std::string LifecycleManager::resolveNodeName(const std::string & name) const
{
  if (name.empty() || name.front() == '/') {
    return name;
  }

  const std::string ns = node_->get_effective_namespace();
  return (ns == "/") ? "/" + name : ns + "/" + name;
}

const LifecycleManager::Clients & LifecycleManager::clientsFor(const std::string & node)
{
  const auto cached = clients_.find(node);
  if (cached != clients_.end()) {
    return cached->second;
  }

  Clients clients;
  clients.changeState = node_->create_client<lifecycle_msgs::srv::ChangeState>(
    node + "/change_state", rclcpp::ServicesQoS(), clientCbGroup_);
  clients.getState = node_->create_client<lifecycle_msgs::srv::GetState>(
    node + "/get_state", rclcpp::ServicesQoS(), clientCbGroup_);

  return clients_.emplace(node, std::move(clients)).first->second;
}

std::vector<std::string> LifecycleManager::discover()
{
  std::vector<std::string> found;

  const auto nodes = node_->get_node_graph_interface()->get_node_names_and_namespaces();

  for (const auto & entry : nodes) {
    const std::string & name = entry.first;
    const std::string & ns = entry.second;
    const std::string fullName = (ns == "/") ? "/" + name : ns + "/" + name;

    if (fullName == node_->get_fully_qualified_name()) {
      continue;
    }

    std::map<std::string, std::vector<std::string>> services;
    try {
      services = node_->get_service_names_and_types_by_node(name, ns);
    } catch (const std::runtime_error &) {
      // The node left the graph between listing and querying it.
      continue;
    }

    const auto service = services.find(fullName + "/change_state");
    if (service == services.end()) {
      continue;
    }
    if (std::find(
        service->second.begin(), service->second.end(),
        "lifecycle_msgs/srv/ChangeState") == service->second.end())
    {
      continue;
    }

    found.push_back(fullName);
  }

  // managed_nodes given: keep only those, in the order they are listed.
  if (!managedNodes_.empty()) {
    std::vector<std::string> managed;
    for (const std::string & name : managedNodes_) {
      const std::string fullName = resolveNodeName(name);
      if (std::find(found.begin(), found.end(), fullName) != found.end()) {
        managed.push_back(fullName);
      }
    }
    return managed;
  }

  std::sort(
    found.begin(), found.end(),
    [](const std::string & a, const std::string & b) {
      const bool aIsDeviceLevel = isDeviceLevelNode(a);
      const bool bIsDeviceLevel = isDeviceLevelNode(b);
      if (aIsDeviceLevel != bIsDeviceLevel) {
        return aIsDeviceLevel;
      }
      return a < b;
    });

  return found;
}

// Current lifecycle state of a node, or "unavailable"/"unknown" when it cannot
// be reached.

std::string LifecycleManager::state(const std::string & node)
{
  const Clients & clients = clientsFor(node);

  if (!clients.getState->wait_for_service(kQueryServiceWaitTimeout)) {
    return "unavailable";
  }

  auto future = clients.getState->async_send_request(
    std::make_shared<lifecycle_msgs::srv::GetState::Request>());

  if (future.wait_for(kQueryTimeout) != std::future_status::ready) {
    clients.getState->remove_pending_request(future);
    return "unknown";
  }

  return future.get()->current_state.label;
}

bool LifecycleManager::changeState(
  const std::string & node, uint8_t transition, std::string & message)
{
  const char * label = transitionLabel(transition);
  const Clients & clients = clientsFor(node);

  if (!clients.changeState->wait_for_service(kServiceWaitTimeout)) {
    message = node + ": " + clients.changeState->get_service_name() + " is not available";
    return false;
  }

  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;

  auto future = clients.changeState->async_send_request(request);
  if (future.wait_for(kTransitionTimeout) != std::future_status::ready) {
    clients.changeState->remove_pending_request(future);
    message = node + ": " + label + " timed out";
    return false;
  }

  if (!future.get()->success) {
    message = node + ": " + label + " was rejected (state is now '" +
      state(node) + "')";
    return false;
  }

  message = node + ": " + label + " done";
  return true;
}

// configure (when unconfigured) then activate.

bool LifecycleManager::bringUp(const std::string & node, std::string & message)
{
  std::string label = state(node);

  if (label == "unconfigured") {
    if (!changeState(node, Transition::TRANSITION_CONFIGURE, message)) {
      return false;
    }
    label = state(node);
  }

  if (label == "inactive") {
    return changeState(node, Transition::TRANSITION_ACTIVATE, message);
  }

  if (label == "active") {
    message = node + ": already active";
    return true;
  }

  message = node + ": cannot be brought up from state '" + label + "'";
  return false;
}

// deactivate, when the node is active.

bool LifecycleManager::bringDown(const std::string & node, std::string & message)
{
  const std::string label = state(node);

  if (label == "active") {
    return changeState(node, Transition::TRANSITION_DEACTIVATE, message);
  }

  if (label == "inactive" || label == "unconfigured" || label == "finalized") {
    message = node + ": already down (state '" + label + "')";
    return true;
  }

  message = node + ": cannot be brought down from state '" + label + "'";
  return false;
}

// The shutdown transition id depends on the state the node is shut down from.

bool LifecycleManager::shutdown(const std::string & node, std::string & message)
{
  const std::string label = state(node);

  if (label == "finalized") {
    message = node + ": already finalized";
    return true;
  }

  uint8_t transition = Transition::TRANSITION_UNCONFIGURED_SHUTDOWN;
  if (label == "active") {
    transition = Transition::TRANSITION_ACTIVE_SHUTDOWN;
  } else if (label == "inactive") {
    transition = Transition::TRANSITION_INACTIVE_SHUTDOWN;
  } else if (label != "unconfigured") {
    message = node + ": cannot be shut down from state '" + label + "'";
    return false;
  }

  return changeState(node, transition, message);
}

// Take the lifecycle nodes off the device before the engine pulls it away from
// under them. With cleanup they also release their device handle.
// Reverse of the bring-up order: the controllers stop before the device-level
// nodes whose axes they command.

void LifecycleManager::bringDownDiscovered(bool cleanup)
{
  const std::vector<std::string> nodes = discover();

  for (auto it = nodes.rbegin(); it != nodes.rend(); ++it) {
    const std::string & node = *it;
    std::string message;

    if (bringDown(node, message)) {
      RCLCPP_INFO(logger_, "%s", message.c_str());
    } else {
      RCLCPP_ERROR(logger_, "%s", message.c_str());
      continue;
    }

    if (cleanup && state(node) == "inactive") {
      if (changeState(node, Transition::TRANSITION_CLEANUP, message)) {
        RCLCPP_INFO(logger_, "%s", message.c_str());
      } else {
        RCLCPP_ERROR(logger_, "%s", message.c_str());
      }
    }
  }
}

// Bring up every lifecycle node the engine has not handled yet. Runs while the
// engine is communicating, so a node that joins late (or respawns) is picked up
// without anyone having to list it anywhere.

void LifecycleManager::bringUpDiscovered()
{
  const std::vector<std::string> discovered = discover();

  // Forget nodes that left the graph, so a respawned one is brought up again.
  for (auto it = handledNodes_.begin(); it != handledNodes_.end(); ) {
    if (std::find(discovered.begin(), discovered.end(), *it) == discovered.end()) {
      RCLCPP_INFO(logger_, "%s left the graph", it->c_str());
      it = handledNodes_.erase(it);
    } else {
      ++it;
    }
  }

  for (const std::string & node : discovered) {
    if (handledNodes_.count(node) > 0) {
      continue;
    }

    std::string message;
    if (bringUp(node, message)) {
      RCLCPP_INFO(logger_, "%s", message.c_str());
      handledNodes_.insert(node);
    } else {
      // Left out of handledNodes_ so a transient failure (device lock, node
      // still starting) is retried on the next sweep.
      RCLCPP_ERROR_THROTTLE(
        logger_, *node_->get_clock(), 10000, "%s", message.c_str());
    }
  }
}

WmxLifecycleManagerNode::WmxLifecycleManagerNode()
: Node("wmx_lifecycle_manager_node")
{
  const auto managedNodes = this->declare_parameter<std::vector<std::string>>(
    "managed_nodes", std::vector<std::string>{});
  engineStatusService_ = this->declare_parameter<std::string>(
    "engine_status_service", "wmx/engine/get_status");
  requireEngine_ = this->declare_parameter<bool>("require_engine", true);
  const double period = this->declare_parameter<double>("discovery_period", 2.0);

  lifecycle_ = std::make_unique<LifecycleManager>(this, managedNodes);

  managerCbGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  clientCbGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  engineStatusClient_ = this->create_client<std_srvs::srv::Trigger>(
    engineStatusService_, rclcpp::ServicesQoS(), clientCbGroup_);

  setNodeStateService_ = this->create_service<wmx_r2_message::srv::SetNodeState>(
    "wmx/lifecycle/set_node_state",
    std::bind(&WmxLifecycleManagerNode::setNodeStateCallback, this, _1, _2),
    rclcpp::ServicesQoS(), managerCbGroup_);

  getNodeStatesService_ = this->create_service<std_srvs::srv::Trigger>(
    "wmx/lifecycle/get_node_states",
    std::bind(&WmxLifecycleManagerNode::getNodeStatesCallback, this, _1, _2),
    rclcpp::ServicesQoS(), managerCbGroup_);

  bringUpAllService_ = this->create_service<std_srvs::srv::Trigger>(
    "wmx/lifecycle/bring_up_all",
    std::bind(&WmxLifecycleManagerNode::bringUpAllCallback, this, _1, _2),
    rclcpp::ServicesQoS(), managerCbGroup_);

  bringDownAllService_ = this->create_service<std_srvs::srv::SetBool>(
    "wmx/lifecycle/bring_down_all",
    std::bind(&WmxLifecycleManagerNode::bringDownAllCallback, this, _1, _2),
    rclcpp::ServicesQoS(), managerCbGroup_);

  discoveryTimer_ = this->create_wall_timer(
    std::chrono::duration<double>(period),
    std::bind(&WmxLifecycleManagerNode::discoveryStep, this),
    managerCbGroup_);

  RCLCPP_INFO(this->get_logger(), "wmx_lifecycle_manager_node is ready");
}

bool WmxLifecycleManagerNode::isEngineCommunicating()
{
  if (!engineStatusClient_->wait_for_service(std::chrono::seconds(1))) {
    return false;
  }

  auto future = engineStatusClient_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());

  if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    engineStatusClient_->remove_pending_request(future);
    return false;
  }

  const auto result = future.get();
  return result->success && result->message == "Communicating";
}

// The lifecycle nodes attach to the device the engine owns, so they follow the
// engine: up while it communicates, down as soon as it stops.
void WmxLifecycleManagerNode::discoveryStep()
{
  if (!requireEngine_) {
    lifecycle_->bringUpDiscovered();
    return;
  }

  if (isEngineCommunicating()) {
    nodesAreUp_ = true;
    lifecycle_->bringUpDiscovered();
    return;
  }

  if (nodesAreUp_) {
    RCLCPP_WARN(
      this->get_logger(), "Engine stopped communicating; taking the lifecycle nodes down");
    lifecycle_->bringDownDiscovered(false);
    nodesAreUp_ = false;
  }
}

// ros2 service call /wmx/lifecycle/set_node_state wmx_r2_message/srv/SetNodeState
//   "{node_name: 'wmx_io_node', transition: 'deactivate'}"
void WmxLifecycleManagerNode::setNodeStateCallback(
  const std::shared_ptr<wmx_r2_message::srv::SetNodeState::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetNodeState::Response> response)
{
  if (request->node_name.empty()) {
    response->success = false;
    response->message =
      "node_name is required. Call wmx/lifecycle/get_node_states to list the nodes.";
    return;
  }

  const std::string node = lifecycle_->resolveNodeName(request->node_name);
  const std::string & transition = request->transition;
  std::string message;
  bool success;

  if (transition == "configure") {
    success = lifecycle_->changeState(
      node, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, message);
  } else if (transition == "activate") {
    success = lifecycle_->changeState(
      node, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, message);
  } else if (transition == "deactivate") {
    success = lifecycle_->changeState(
      node, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, message);
  } else if (transition == "cleanup") {
    success = lifecycle_->changeState(
      node, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP, message);
  } else if (transition == "shutdown") {
    success = lifecycle_->shutdown(node, message);
  } else if (transition == "bringup") {
    success = lifecycle_->bringUp(node, message);
  } else if (transition == "bringdown") {
    success = lifecycle_->bringDown(node, message);
  } else {
    response->success = false;
    response->message = "Unknown transition '" + transition +
      "'. Use configure, activate, deactivate, cleanup, shutdown, bringup or bringdown.";
    return;
  }

  if (success) {
    RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
    // Whatever the operator set stands: the discovery sweep leaves it alone.
    lifecycle_->markHandled(node);
  } else {
    RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
  }

  response->success = success;
  response->message = message;
  response->node_names.push_back(node);
  response->states.push_back(lifecycle_->state(node));
}

void WmxLifecycleManagerNode::getNodeStatesCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::string message;
  for (const std::string & node : lifecycle_->discover()) {
    message += node + ": " + lifecycle_->state(node) + "; ";
  }

  response->success = true;
  response->message = message.empty() ? "No lifecycle nodes found." : message;
}

void WmxLifecycleManagerNode::bringUpAllCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  lifecycle_->bringUpDiscovered();

  std::string message;
  for (const std::string & node : lifecycle_->discover()) {
    message += node + ": " + lifecycle_->state(node) + "; ";
  }

  response->success = true;
  response->message = message.empty() ? "No lifecycle nodes found." : message;
}

// data: true also cleans the nodes up, releasing their WMX device handle.
void WmxLifecycleManagerNode::bringDownAllCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  lifecycle_->bringDownDiscovered(request->data);

  std::string message;
  for (const std::string & node : lifecycle_->discover()) {
    message += node + ": " + lifecycle_->state(node) + "; ";
  }

  response->success = true;
  response->message = message.empty() ? "No lifecycle nodes found." : message;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WmxLifecycleManagerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
