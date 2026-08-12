// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_engine_node.hpp"

#include <algorithm>
#include <cinttypes>

using std::placeholders::_1;
using std::placeholders::_2;
using lifecycle_msgs::msg::Transition;

namespace
{
// The managed nodes attach to the WMX3 device with a 10 s timeout of their own,
// so a configure transition needs plenty of headroom.
constexpr std::chrono::seconds kServiceWaitTimeout{5};
constexpr std::chrono::seconds kTransitionTimeout{30};
// A get_state server that exists answers immediately, so a state query only
// waits long enough to tell "not running yet" from "running but slow". Keeping
// this short bounds how long a states query takes when nodes are missing.
constexpr std::chrono::seconds kQueryServiceWaitTimeout{1};
constexpr std::chrono::seconds kQueryTimeout{5};

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

WmxEngineNode::WmxEngineNode()
: Node("wmx_engine_node")
{
  this->declare_parameter<int>("engine_core", -1);
  this->declare_parameter<int64_t>("engine_affinity_mask", 0);

  autoManageNodes_ = this->declare_parameter<bool>("auto_manage_nodes", true);
  const std::vector<std::string> managedNames =
    this->declare_parameter<std::vector<std::string>>(
    "managed_nodes",
    std::vector<std::string>{"wmx_core_motion_node", "wmx_io_node", "wmx_ethercat_node"});

  managerCbGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  lifecycleClientCbGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  for (const std::string & name : managedNames) {
    ManagedNode node;
    node.name = name;
    node.changeState = this->create_client<lifecycle_msgs::srv::ChangeState>(
      name + "/change_state", rclcpp::ServicesQoS(), lifecycleClientCbGroup_);
    node.getState = this->create_client<lifecycle_msgs::srv::GetState>(
      name + "/get_state", rclcpp::ServicesQoS(), lifecycleClientCbGroup_);
    managedNodes_.push_back(std::move(node));
  }

  setEngineService_ = this->create_service<wmx_r2_message::srv::SetEngine>(
    "wmx/engine/set_engine",
    std::bind(&WmxEngineNode::setEngine, this, _1, _2),
    rclcpp::ServicesQoS(), managerCbGroup_);

  setCommService_ = this->create_service<std_srvs::srv::SetBool>(
    "wmx/engine/set_comm",
    std::bind(&WmxEngineNode::setComm, this, _1, _2),
    rclcpp::ServicesQoS(), managerCbGroup_);

  getEngineStatusService_ = this->create_service<std_srvs::srv::Trigger>(
    "wmx/engine/get_status",
    std::bind(&WmxEngineNode::getEngineStatus, this, _1, _2));

  setNodeStateService_ = this->create_service<wmx_r2_message::srv::SetNodeState>(
    "wmx/engine/set_node_state",
    std::bind(&WmxEngineNode::setNodeState, this, _1, _2),
    rclcpp::ServicesQoS(), managerCbGroup_);

  getNodeStatesService_ = this->create_service<std_srvs::srv::Trigger>(
    "wmx/engine/get_node_states",
    std::bind(&WmxEngineNode::getNodeStates, this, _1, _2),
    rclcpp::ServicesQoS(), managerCbGroup_);

  if (autoManageNodes_ && !managedNodes_.empty()) {
    // Bringing the managed nodes up needs a spinning executor (their responses
    // are handled by it), so it cannot run from the constructor or from
    // startThread_. This timer does it on the first tick after the engine
    // startup attempt finishes, then cancels itself.
    bringUpTimer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&WmxEngineNode::bringUpStep, this),
      managerCbGroup_);
  }

  // Run startEngine on a background thread to avoid blocking the executor
  // during construction. The executor must be spinning so services and
  // timers can fire while the engine initialises.
  startThread_ = std::thread(&WmxEngineNode::startEngine, this);

  RCLCPP_INFO(this->get_logger(), "wmx_engine_node is ready");
}

WmxEngineNode::~WmxEngineNode()
{
  if (startThread_.joinable()) {
    startThread_.join();
  }
  stopCommunication();
  stopEngine();
  std::this_thread::sleep_for(std::chrono::seconds(3));
  RCLCPP_INFO(this->get_logger(), "wmx_engine_node stopped");
}

WmxEngineNode::ManagedNode * WmxEngineNode::findManagedNode(const std::string & name)
{
  for (ManagedNode & node : managedNodes_) {
    if (node.name == name) {
      return &node;
    }
  }
  return nullptr;
}

// Current lifecycle state of a managed node, or "unavailable"/"unknown" when it
// cannot be reached.
std::string WmxEngineNode::nodeStateLabel(const ManagedNode & node)
{
  if (!node.getState->wait_for_service(kQueryServiceWaitTimeout)) {
    return "unavailable";
  }

  auto future = node.getState->async_send_request(
    std::make_shared<lifecycle_msgs::srv::GetState::Request>());

  if (future.wait_for(kQueryTimeout) != std::future_status::ready) {
    node.getState->remove_pending_request(future);
    return "unknown";
  }

  return future.get()->current_state.label;
}

bool WmxEngineNode::changeNodeState(
  const ManagedNode & node, uint8_t transition, std::string & message)
{
  const char * label = transitionLabel(transition);

  if (!node.changeState->wait_for_service(kServiceWaitTimeout)) {
    message = node.name + ": " + node.changeState->get_service_name() + " is not available";
    return false;
  }

  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;

  auto future = node.changeState->async_send_request(request);
  if (future.wait_for(kTransitionTimeout) != std::future_status::ready) {
    node.changeState->remove_pending_request(future);
    message = node.name + ": " + label + " timed out";
    return false;
  }

  if (!future.get()->success) {
    message = node.name + ": " + label + " was rejected (state is now '" +
      nodeStateLabel(node) + "')";
    return false;
  }

  message = node.name + ": " + label + " done";
  return true;
}

// configure (when unconfigured) then activate.
bool WmxEngineNode::bringUpNode(const ManagedNode & node, std::string & message)
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
    message = node.name + ": already active";
    return true;
  }

  message = node.name + ": cannot be brought up from state '" + state + "'";
  return false;
}

// deactivate, when the node is active.
bool WmxEngineNode::bringDownNode(const ManagedNode & node, std::string & message)
{
  const std::string state = nodeStateLabel(node);

  if (state == "active") {
    return changeNodeState(node, Transition::TRANSITION_DEACTIVATE, message);
  }

  if (state == "inactive" || state == "unconfigured" || state == "finalized") {
    message = node.name + ": already down (state '" + state + "')";
    return true;
  }

  message = node.name + ": cannot be brought down from state '" + state + "'";
  return false;
}

// The shutdown transition id depends on the state the node is shut down from.
bool WmxEngineNode::shutdownNode(const ManagedNode & node, std::string & message)
{
  const std::string state = nodeStateLabel(node);

  if (state == "finalized") {
    message = node.name + ": already finalized";
    return true;
  }

  uint8_t transition = Transition::TRANSITION_UNCONFIGURED_SHUTDOWN;
  if (state == "active") {
    transition = Transition::TRANSITION_ACTIVE_SHUTDOWN;
  } else if (state == "inactive") {
    transition = Transition::TRANSITION_INACTIVE_SHUTDOWN;
  } else if (state != "unconfigured") {
    message = node.name + ": cannot be shut down from state '" + state + "'";
    return false;
  }

  return changeNodeState(node, transition, message);
}

void WmxEngineNode::bringUpManagedNodes()
{
  if (managedNodes_.empty()) {
    return;
  }

  RCLCPP_INFO(
    this->get_logger(), "Bringing up %zu managed node(s)...", managedNodes_.size());

  for (const ManagedNode & node : managedNodes_) {
    std::string message;
    if (bringUpNode(node, message)) {
      RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
    }
  }
}

// Take the managed nodes off the device before the engine pulls it away from
// under them. With cleanup they also release their device handle.
// Reverse order: the controllers at the end of the list command the axes that
// wmx_core_motion_node at the front of it owns, so they stop first.
void WmxEngineNode::bringDownManagedNodes(bool cleanup)
{
  for (auto it = managedNodes_.rbegin(); it != managedNodes_.rend(); ++it) {
    const ManagedNode & node = *it;
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

void WmxEngineNode::bringUpStep()
{
  if (!startComplete_) {
    return;
  }

  // One shot: from here on the managed nodes are driven by wmx/engine/set_comm,
  // wmx/engine/set_engine and wmx/engine/set_node_state.
  bringUpTimer_->cancel();

  if (!commStarted_) {
    RCLCPP_WARN(
      this->get_logger(),
      "Engine is not communicating; managed nodes stay unconfigured. "
      "Start communication with wmx/engine/set_comm to bring them up.");
    return;
  }

  bringUpManagedNodes();
}

void WmxEngineNode::startEngine()
{
  const int core = static_cast<int>(this->get_parameter("engine_core").as_int());
  const int64_t affinityMask = this->get_parameter("engine_affinity_mask").as_int();
  RCLCPP_INFO(
    this->get_logger(), "Starting engine... (core=%d, affinityMask=0x%" PRIx64 ")",
    core, affinityMask);
  unsigned int timeout = 10000;
  int maxRetries = 5;
  int retryDelay = 2000;
  const int CreateDeviceLockError = 297;
  int err;
  char errString[256];

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  for (int attempt = 0; attempt < maxRetries; attempt++) {
    if (attempt > 0) {
      RCLCPP_INFO(
        this->get_logger(), "Retrying device creation (attempt %d/%d)...",
        attempt + 1, maxRetries);
      std::this_thread::sleep_for(std::chrono::milliseconds(retryDelay));
    }

    err = wmx3Lib_.CreateDevice(
      WMX3_SDK_PATH, wmx3Api::DeviceType::DeviceTypeNormal, timeout, core, affinityMask);

    if (err == wmx3Api::ErrorCode::None) {
      wmx3Lib_.SetDeviceName("wmx_engine_node");
      RCLCPP_INFO(this->get_logger(), "Device created (attempt %d)", attempt + 1);

      err = wmx3Lib_.StartCommunication(timeout);
      if (err == wmx3Api::ErrorCode::None) {
        RCLCPP_INFO(this->get_logger(), "Communication started");
        commStarted_ = true;
        startComplete_ = true;
      } else {
        wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
        RCLCPP_ERROR(
          this->get_logger(),
          "Failed to start communication. Error=%d (%s)", err, errString);
      }
      startComplete_ = true;
      return;
    } else {
      wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
      if (err == CreateDeviceLockError) {
        RCLCPP_WARN(
          this->get_logger(),
          "Device lock error (attempt %d/%d). Waiting...",
          attempt + 1, maxRetries);
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Failed to create device (attempt %d/%d). Error=%d (%s)",
          attempt + 1, maxRetries, err, errString);
      }
    }
  }

  wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
  RCLCPP_ERROR(
    this->get_logger(),
    "Failed to create device after %d attempts. Error=%d (%s)",
    maxRetries, err, errString);
  startComplete_ = true;
}

void WmxEngineNode::stopCommunication()
{
  unsigned int timeout = 10000;
  int err;
  char errString[256];
  err = wmx3Lib_.StopCommunication(timeout);
  if (err != wmx3Api::ErrorCode::None) {
    wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
    RCLCPP_ERROR(this->get_logger(), "Failed to stop communication");
  } else {
    RCLCPP_INFO(this->get_logger(), "Communication stopped");
  }
  commStarted_ = false;
}

void WmxEngineNode::stopEngine()
{
  int err;
  char errString[256];

  unsigned int timeout = 10000;
  err = wmx3Lib_.StopEngine(timeout);
  if (err != wmx3Api::ErrorCode::None) {
    wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
    RCLCPP_ERROR(this->get_logger(), "Failed to stop engine");
  } else {
    RCLCPP_INFO(this->get_logger(), "Engine stopped");
  }

  err = wmx3Lib_.CloseDevice();
  if (err != wmx3Api::ErrorCode::None) {
    wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
    RCLCPP_ERROR(this->get_logger(), "Failed to close device");
  } else {
    RCLCPP_INFO(this->get_logger(), "Device closed");
  }
}

void WmxEngineNode::getEngineStatus(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!startComplete_) {
    response->success = false;
    response->message = "Engine startup in progress";
    return;
  }

  wmx3Api::EngineStatus status;
  wmx3Lib_.GetEngineStatus(&status);

  std::string status_str;
  switch (status.state) {
    case wmx3Api::EngineState::Idle:          status_str = "Idle"; break;
    case wmx3Api::EngineState::Running:       status_str = "Running"; break;
    case wmx3Api::EngineState::Communicating: status_str = "Communicating"; break;
    case wmx3Api::EngineState::Shutdown:      status_str = "Shutdown"; break;
    case wmx3Api::EngineState::Unknown:       status_str = "Unknown"; break;
    default:                                  status_str = "Invalid"; break;
  }

  response->success = true;
  response->message = status_str;
}

void WmxEngineNode::setComm(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (!startComplete_) {
    response->success = false;
    response->message = "Engine startup in progress";
    return;
  }

  unsigned int timeout = 10000;
  int err;
  char errString[256];
  char buffer[512];
  if (request->data) {
    err = wmx3Lib_.StartCommunication(timeout);
    if (err != wmx3Api::ErrorCode::None) {
      wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
      snprintf(
        buffer, sizeof(buffer),
        "Failed to start communication. Error=%d (%s)", err, errString);
      RCLCPP_ERROR(this->get_logger(), "%s", buffer);
      response->success = false;
      response->message = std::string(buffer);
    } else {
      commStarted_ = true;
      snprintf(buffer, sizeof(buffer), "Communication started");
      RCLCPP_INFO(this->get_logger(), "%s", buffer);
      response->success = true;
      response->message = std::string(buffer);

      if (autoManageNodes_) {
        bringUpManagedNodes();
      }
    }
  } else {
    // The managed nodes command axes over this link, so stop them first.
    if (autoManageNodes_) {
      bringDownManagedNodes(false);
    }

    err = wmx3Lib_.StopCommunication(timeout);
    if (err != wmx3Api::ErrorCode::None) {
      wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
      snprintf(
        buffer, sizeof(buffer),
        "Failed to stop communication. Error=%d (%s)", err, errString);
      RCLCPP_ERROR(this->get_logger(), "%s", buffer);
      response->success = false;
      response->message = std::string(buffer);
    } else {
      commStarted_ = false;
      snprintf(buffer, sizeof(buffer), "Communication stopped");
      RCLCPP_INFO(this->get_logger(), "%s", buffer);
      response->success = true;
      response->message = std::string(buffer);
    }
  }
}

void WmxEngineNode::setEngine(
  const std::shared_ptr<wmx_r2_message::srv::SetEngine::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetEngine::Response> response)
{
  if (!startComplete_) {
    response->success = false;
    response->message = "Engine startup in progress";
    return;
  }

  unsigned int timeout = 10000;
  int err;
  char errString[256];
  char buffer[512];
  if (request->data) {
    const int core = static_cast<int>(this->get_parameter("engine_core").as_int());
    const int64_t affinityMask = this->get_parameter("engine_affinity_mask").as_int();
    err = wmx3Lib_.CreateDevice(
      request->path.c_str(), wmx3Api::DeviceType::DeviceTypeNormal, timeout, core, affinityMask);
    if (err != wmx3Api::ErrorCode::None) {
      wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
      snprintf(
        buffer, sizeof(buffer),
        "Failed to create device. Error=%d (%s)", err, errString);
      RCLCPP_ERROR(this->get_logger(), "%s", buffer);
      response->success = false;
      response->message = std::string(buffer);
    } else {
      wmx3Lib_.SetDeviceName(request->name.c_str());
      snprintf(
        buffer, sizeof(buffer),
        "Created device with name: %s", request->name.c_str());
      RCLCPP_INFO(this->get_logger(), "%s", buffer);
      response->success = true;
      response->message = std::string(buffer);
    }
  } else {
    // Closing the device invalidates the handles the managed nodes hold, so
    // take them all the way down to unconfigured first.
    if (autoManageNodes_) {
      bringDownManagedNodes(true);
    }

    err = wmx3Lib_.CloseDevice();
    if (err != wmx3Api::ErrorCode::None) {
      wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
      snprintf(
        buffer, sizeof(buffer),
        "Failed to close device. Error=%d (%s)", err, errString);
      RCLCPP_ERROR(this->get_logger(), "%s", buffer);
      response->success = false;
      response->message = std::string(buffer);
    } else {
      commStarted_ = false;
      snprintf(buffer, sizeof(buffer), "Device closed");
      RCLCPP_INFO(this->get_logger(), "%s", buffer);
      response->success = true;
      response->message = std::string(buffer);
    }
  }
}

// ros2 service call /wmx/engine/set_node_state wmx_r2_message/srv/SetNodeState
//   "{node_name: 'wmx_io_node', transition: 'deactivate'}"
// An empty node_name targets every managed node.
void WmxEngineNode::setNodeState(
  const std::shared_ptr<wmx_r2_message::srv::SetNodeState::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetNodeState::Response> response)
{
  const std::string & transition = request->transition;

  // Same ordering rule as the automatic path: bring up front to back, take down
  // back to front, so a controller never outlives the axes it commands.
  const bool takingDown =
    transition == "deactivate" || transition == "cleanup" ||
    transition == "shutdown" || transition == "bringdown";

  std::vector<const ManagedNode *> targets;
  if (request->node_name.empty()) {
    for (const ManagedNode & node : managedNodes_) {
      targets.push_back(&node);
    }
    if (takingDown) {
      std::reverse(targets.begin(), targets.end());
    }
  } else {
    const ManagedNode * node = findManagedNode(request->node_name);
    if (node == nullptr) {
      response->success = false;
      response->message = "'" + request->node_name +
        "' is not a managed node. Check the managed_nodes parameter.";
      return;
    }
    targets.push_back(node);
  }

  if (targets.empty()) {
    response->success = false;
    response->message = "No managed nodes configured.";
    return;
  }

  bool all_success = true;
  std::string message;

  for (const ManagedNode * node : targets) {
    std::string node_message;
    bool ok;

    if (transition == "configure") {
      ok = changeNodeState(*node, Transition::TRANSITION_CONFIGURE, node_message);
    } else if (transition == "activate") {
      ok = changeNodeState(*node, Transition::TRANSITION_ACTIVATE, node_message);
    } else if (transition == "deactivate") {
      ok = changeNodeState(*node, Transition::TRANSITION_DEACTIVATE, node_message);
    } else if (transition == "cleanup") {
      ok = changeNodeState(*node, Transition::TRANSITION_CLEANUP, node_message);
    } else if (transition == "shutdown") {
      ok = shutdownNode(*node, node_message);
    } else if (transition == "bringup") {
      ok = bringUpNode(*node, node_message);
    } else if (transition == "bringdown") {
      ok = bringDownNode(*node, node_message);
    } else {
      response->success = false;
      response->message = "Unknown transition '" + transition +
        "'. Use configure, activate, deactivate, cleanup, shutdown, bringup or bringdown.";
      return;
    }

    if (ok) {
      RCLCPP_INFO(this->get_logger(), "%s", node_message.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "%s", node_message.c_str());
      all_success = false;
    }

    message += node_message + "; ";
    response->node_names.push_back(node->name);
    response->states.push_back(nodeStateLabel(*node));
  }

  response->success = all_success;
  response->message = message;
}

void WmxEngineNode::getNodeStates(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::string message;
  for (const ManagedNode & node : managedNodes_) {
    message += node.name + ": " + nodeStateLabel(node) + "; ";
  }

  response->success = true;
  response->message = message.empty() ? "No managed nodes configured." : message;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WmxEngineNode>();
  // Multi threaded: the manager callbacks block waiting for lifecycle
  // transitions while the executor handles the responses of those same calls.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
