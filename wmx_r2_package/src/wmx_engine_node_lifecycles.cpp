// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_engine_node.hpp"

#include <algorithm>

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

void WmxEngineNode::setNodeStateCallback(
  const std::shared_ptr<wmx_r2_message::srv::SetNodeState::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetNodeState::Response> response)
{
  if (request->node_name.empty()) {
    response->success = false;
    response->message =
      "node_name is required. Call wmx/engine/get_node_states to list the nodes.";
    return;
  }

  std::string node = request->node_name;
  if (node.front() != '/') {
    const std::string ns = this->get_effective_namespace();
    node = (ns == "/") ? "/" + node : ns + "/" + node;
  }

  const std::string & transition = request->transition;
  std::string message;
  bool success;

  if (transition == "configure") {
    success = changeNodeState(node, Transition::TRANSITION_CONFIGURE, message);
  } else if (transition == "activate") {
    success = changeNodeState(node, Transition::TRANSITION_ACTIVATE, message);
  } else if (transition == "deactivate") {
    success = changeNodeState(node, Transition::TRANSITION_DEACTIVATE, message);
  } else if (transition == "cleanup") {
    success = changeNodeState(node, Transition::TRANSITION_CLEANUP, message);
  } else if (transition == "shutdown") {
    success = shutdownNode(node, message);
  } else if (transition == "bringup") {
    success = bringUpNode(node, message);
  } else if (transition == "bringdown") {
    success = bringDownNode(node, message);
  } else {
    response->success = false;
    response->message = "Unknown transition '" + transition +
      "'. Use configure, activate, deactivate, cleanup, shutdown, bringup or bringdown.";
    return;
  }

  if (success) {
    RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
    // An explicit bring-up counts as handled, so the discovery sweep does not
    // touch this node again; an explicit take-down is left as the operator set
    // it, for the same reason.
    broughtUpNodes_.insert(node);
  } else {
    RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
  }

  response->success = success;
  response->message = message;
  response->node_names.push_back(node);
  response->states.push_back(nodeStateLabel(node));
}

void WmxEngineNode::getNodeStatesCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  const std::vector<std::string> nodes = discoverLifecycleNodes();

  std::string message;
  for (const std::string & node : nodes) {
    message += node + ": " + nodeStateLabel(node) + "; ";
  }

  response->success = true;
  response->message = message.empty() ? "No lifecycle nodes found." : message;
}

// A plain node name is resolved against this node's namespace; an absolute
// name is taken as given.
std::string WmxEngineNode::resolveNodeName(const std::string & name) const
{
  if (name.empty() || name.front() == '/') {
    return name;
  }

  const std::string ns = this->get_effective_namespace();
  return (ns == "/") ? "/" + name : ns + "/" + name;
}

const WmxEngineNode::LifecycleClients & WmxEngineNode::clientsFor(const std::string & node)
{
  const auto cached = lifecycleClients_.find(node);
  if (cached != lifecycleClients_.end()) {
    return cached->second;
  }

  LifecycleClients clients;
  clients.changeState = this->create_client<lifecycle_msgs::srv::ChangeState>(
    node + "/change_state", rclcpp::ServicesQoS(), lifecycleClientCbGroup_);
  clients.getState = this->create_client<lifecycle_msgs::srv::GetState>(
    node + "/get_state", rclcpp::ServicesQoS(), lifecycleClientCbGroup_);

  return lifecycleClients_.emplace(node, std::move(clients)).first->second;
}

std::vector<std::string> WmxEngineNode::discoverLifecycleNodes()
{
  std::vector<std::string> found;

  const auto nodes = this->get_node_graph_interface()->get_node_names_and_namespaces();

  for (const auto & entry : nodes) {
    const std::string & name = entry.first;
    const std::string & ns = entry.second;
    const std::string fullName = (ns == "/") ? "/" + name : ns + "/" + name;

    if (fullName == this->get_fully_qualified_name()) {
      continue;
    }

    std::map<std::string, std::vector<std::string>> services;
    try {
      services = this->get_service_names_and_types_by_node(name, ns);
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

std::string WmxEngineNode::nodeStateLabel(const std::string & node)
{
  const LifecycleClients & clients = clientsFor(node);

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

bool WmxEngineNode::changeNodeState(
  const std::string & node, uint8_t transition, std::string & message)
{
  const char * label = transitionLabel(transition);
  const LifecycleClients & clients = clientsFor(node);

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
      nodeStateLabel(node) + "')";
    return false;
  }

  message = node + ": " + label + " done";
  return true;
}

// configure (when unconfigured) then activate.

bool WmxEngineNode::bringUpNode(const std::string & node, std::string & message)
{
  std::string state = nodeStateLabel(node);

  if (state == "unconfigured") {
    if (!changeNodeState(node, Transition::TRANSITION_CONFIGURE, message)) {
      return false;
    }
    state = nodeStateLabel(node);
  }

  if (state == "inactive") {
    return changeNodeState(node, Transition::TRANSITION_ACTIVATE, message);
  }

  if (state == "active") {
    message = node + ": already active";
    return true;
  }

  message = node + ": cannot be brought up from state '" + state + "'";
  return false;
}

// deactivate, when the node is active.

bool WmxEngineNode::bringDownNode(const std::string & node, std::string & message)
{
  const std::string state = nodeStateLabel(node);

  if (state == "active") {
    return changeNodeState(node, Transition::TRANSITION_DEACTIVATE, message);
  }

  if (state == "inactive" || state == "unconfigured" || state == "finalized") {
    message = node + ": already down (state '" + state + "')";
    return true;
  }

  message = node + ": cannot be brought down from state '" + state + "'";
  return false;
}

// The shutdown transition id depends on the state the node is shut down from.

bool WmxEngineNode::shutdownNode(const std::string & node, std::string & message)
{
  const std::string state = nodeStateLabel(node);

  if (state == "finalized") {
    message = node + ": already finalized";
    return true;
  }

  uint8_t transition = Transition::TRANSITION_UNCONFIGURED_SHUTDOWN;
  if (state == "active") {
    transition = Transition::TRANSITION_ACTIVE_SHUTDOWN;
  } else if (state == "inactive") {
    transition = Transition::TRANSITION_INACTIVE_SHUTDOWN;
  } else if (state != "unconfigured") {
    message = node + ": cannot be shut down from state '" + state + "'";
    return false;
  }

  return changeNodeState(node, transition, message);
}

// Take the lifecycle nodes off the device before the engine pulls it away from
// under them. With cleanup they also release their device handle.
// Reverse of the bring-up order: the controllers stop before the device-level
// nodes whose axes they command.

void WmxEngineNode::bringDownDiscoveredNodes(bool cleanup)
{
  const std::vector<std::string> nodes = discoverLifecycleNodes();

  for (auto it = nodes.rbegin(); it != nodes.rend(); ++it) {
    const std::string & node = *it;
    std::string message;

    if (bringDownNode(node, message)) {
      RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
      continue;
    }

    if (cleanup && nodeStateLabel(node) == "inactive") {
      if (changeNodeState(node, Transition::TRANSITION_CLEANUP, message)) {
        RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
      }
    }
  }
}

// Bring up every lifecycle node the engine has not handled yet. Runs while the
// engine is communicating, so a node that joins late (or respawns) is picked up
// without anyone having to list it anywhere.

void WmxEngineNode::discoveryStep()
{
  if (!isEngineStarted_) {
    return;
  }

  if (!isCommStarted_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 30000,
      "Engine is not communicating; lifecycle nodes stay unconfigured. "
      "Start communication with wmx/engine/set_comm to bring them up.");
    return;
  }

  const std::vector<std::string> discovered = discoverLifecycleNodes();

  // Forget nodes that left the graph, so a respawned one is brought up again.
  for (auto it = broughtUpNodes_.begin(); it != broughtUpNodes_.end(); ) {
    if (std::find(discovered.begin(), discovered.end(), *it) == discovered.end()) {
      RCLCPP_INFO(this->get_logger(), "%s left the graph", it->c_str());
      it = broughtUpNodes_.erase(it);
    } else {
      ++it;
    }
  }

  for (const std::string & node : discovered) {
    if (broughtUpNodes_.count(node) > 0) {
      continue;
    }

    std::string message;
    if (bringUpNode(node, message)) {
      RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
      broughtUpNodes_.insert(node);
    } else {
      // Left out of broughtUpNodes_ so a transient failure (device lock, node
      // still starting) is retried on the next sweep.
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 10000, "%s", message.c_str());
    }
  }
}
