// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_engine_node.hpp"

#include <algorithm>
#include <cinttypes>


using std::placeholders::_1;
using std::placeholders::_2;

/*
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
}
*/

WmxEngineNode::WmxEngineNode()
: Node("wmx_engine_node")
{
  devicePath_ = this->declare_parameter<std::string>("device_path", "/opt/wmx3/");
  deviceName_ = this->declare_parameter<std::string>("device_name", "wmx_r2");
  engineCore_ = this->declare_parameter<int>("engine_core", -1);
  engineAffinityMask_ = this->declare_parameter<int64_t>("engine_affinity_mask", 0);
  wmxParamFilePath_ = this->declare_parameter<std::string>("wmx_param_file_path", "");

  managerCbGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
//   lifecycleClientCbGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  setEngineService_ = this->create_service<std_srvs::srv::SetBool>(
    "wmx/engine/set_engine",
    std::bind(&WmxEngineNode::setEngineCallback, this, _1, _2),
    rclcpp::ServicesQoS(), managerCbGroup_);

  setCommService_ = this->create_service<std_srvs::srv::SetBool>(
    "wmx/engine/set_comm",
    std::bind(&WmxEngineNode::setCommCallback, this, _1, _2),
    rclcpp::ServicesQoS(), managerCbGroup_);

  getEngineStatusService_ = this->create_service<std_srvs::srv::Trigger>(
    "wmx/engine/get_status",
    std::bind(&WmxEngineNode::getEngineStatusCallback, this, _1, _2));

  loadParamsService_ = this->create_service<wmx_r2_message::srv::LoadWmxParams>(
    "wmx/params/load",
    std::bind(&WmxEngineNode::loadWmxParamsCallback, this, _1, _2),
    rclcpp::ServicesQoS(), managerCbGroup_);

  getParamsService_ = this->create_service<wmx_r2_message::srv::GetWmxParams>(
    "wmx/params/get",
    std::bind(&WmxEngineNode::getWmxParamsCallback, this, _1, _2),
    rclcpp::ServicesQoS(), managerCbGroup_);

//   setNodeStateService_ = this->create_service<wmx_r2_message::srv::SetNodeState>(
//     "wmx/engine/set_node_state",
//     std::bind(&WmxEngineNode::setNodeStateCallback, this, _1, _2),
//     rclcpp::ServicesQoS(), managerCbGroup_);
//
//   getNodeStatesService_ = this->create_service<std_srvs::srv::Trigger>(
//     "wmx/engine/get_node_states",
//     std::bind(&WmxEngineNode::getNodeStatesCallback, this, _1, _2),
//     rclcpp::ServicesQoS(), managerCbGroup_);

//   // Watches the graph for lifecycle nodes and brings up each new one. Runs on a
//   // timer, not from the constructor or startThread_, because the transition
//   // responses are handled by the executor, which has to be spinning.
//   discoveryTimer_ = this->create_wall_timer(
//     std::chrono::seconds(2),
//     std::bind(&WmxEngineNode::discoveryStep, this),
//     managerCbGroup_);

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
  std::string message;
  stopCommunication(message);
  stopEngine();
  std::this_thread::sleep_for(std::chrono::seconds(3));
  RCLCPP_INFO(this->get_logger(), "wmx_engine_node stopped");
}

// const WmxEngineNode::LifecycleClients & WmxEngineNode::clientsFor(const std::string & node)
// {
//   const auto cached = lifecycleClients_.find(node);
//   if (cached != lifecycleClients_.end()) {
//     return cached->second;
//   }
//
//   LifecycleClients clients;
//   clients.changeState = this->create_client<lifecycle_msgs::srv::ChangeState>(
//     node + "/change_state", rclcpp::ServicesQoS(), lifecycleClientCbGroup_);
//   clients.getState = this->create_client<lifecycle_msgs::srv::GetState>(
//     node + "/get_state", rclcpp::ServicesQoS(), lifecycleClientCbGroup_);
//
//   return lifecycleClients_.emplace(node, std::move(clients)).first->second;
// }
//
// // Every node on the graph that offers a lifecycle change_state service, whether
// // it belongs to this package or not, ordered for bring-up: the WMX device-level
// // nodes first, then everything that depends on them.
// std::vector<std::string> WmxEngineNode::discoverLifecycleNodes()
// {
//   std::vector<std::string> found;
//
//   const auto nodes = this->get_node_graph_interface()->get_node_names_and_namespaces();
//
//   for (const auto & entry : nodes) {
//     const std::string & name = entry.first;
//     const std::string & ns = entry.second;
//     const std::string fullName = (ns == "/") ? "/" + name : ns + "/" + name;
//
//     if (fullName == this->get_fully_qualified_name()) {
//       continue;
//     }
//
//     std::map<std::string, std::vector<std::string>> services;
//     try {
//       services = this->get_service_names_and_types_by_node(name, ns);
//     } catch (const std::runtime_error &) {
//       // The node left the graph between listing and querying it.
//       continue;
//     }
//
//     const auto service = services.find(fullName + "/change_state");
//     if (service == services.end()) {
//       continue;
//     }
//     if (std::find(
//         service->second.begin(), service->second.end(),
//         "lifecycle_msgs/srv/ChangeState") == service->second.end())
//     {
//       continue;
//     }
//
//     found.push_back(fullName);
//   }
//
//   std::sort(
//     found.begin(), found.end(),
//     [](const std::string & a, const std::string & b) {
//       const bool aIsDeviceLevel = isDeviceLevelNode(a);
//       const bool bIsDeviceLevel = isDeviceLevelNode(b);
//       if (aIsDeviceLevel != bIsDeviceLevel) {
//         return aIsDeviceLevel;
//       }
//       return a < b;
//     });
//
//   return found;
// }
//
// // Current lifecycle state of a node, or "unavailable"/"unknown" when it cannot
// // be reached.
// std::string WmxEngineNode::nodeStateLabel(const std::string & node)
// {
//   const LifecycleClients & clients = clientsFor(node);
//
//   if (!clients.getState->wait_for_service(kQueryServiceWaitTimeout)) {
//     return "unavailable";
//   }
//
//   auto future = clients.getState->async_send_request(
//     std::make_shared<lifecycle_msgs::srv::GetState::Request>());
//
//   if (future.wait_for(kQueryTimeout) != std::future_status::ready) {
//     clients.getState->remove_pending_request(future);
//     return "unknown";
//   }
//
//   return future.get()->current_state.label;
// }
//
// bool WmxEngineNode::changeNodeState(
//   const std::string & node, uint8_t transition, std::string & message)
// {
//   const char * label = transitionLabel(transition);
//   const LifecycleClients & clients = clientsFor(node);
//
//   if (!clients.changeState->wait_for_service(kServiceWaitTimeout)) {
//     message = node + ": " + clients.changeState->get_service_name() + " is not available";
//     return false;
//   }
//
//   auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
//   request->transition.id = transition;
//
//   auto future = clients.changeState->async_send_request(request);
//   if (future.wait_for(kTransitionTimeout) != std::future_status::ready) {
//     clients.changeState->remove_pending_request(future);
//     message = node + ": " + label + " timed out";
//     return false;
//   }
//
//   if (!future.get()->success) {
//     message = node + ": " + label + " was rejected (state is now '" +
//       nodeStateLabel(node) + "')";
//     return false;
//   }
//
//   message = node + ": " + label + " done";
//   return true;
// }
//
// // configure (when unconfigured) then activate.
// bool WmxEngineNode::bringUpNode(const std::string & node, std::string & message)
// {
//   std::string state = nodeStateLabel(node);
//
//   if (state == "unconfigured") {
//     if (!changeNodeState(node, Transition::TRANSITION_CONFIGURE, message)) {
//       return false;
//     }
//     state = nodeStateLabel(node);
//   }
//
//   if (state == "inactive") {
//     return changeNodeState(node, Transition::TRANSITION_ACTIVATE, message);
//   }
//
//   if (state == "active") {
//     message = node + ": already active";
//     return true;
//   }
//
//   message = node + ": cannot be brought up from state '" + state + "'";
//   return false;
// }
//
// // deactivate, when the node is active.
// bool WmxEngineNode::bringDownNode(const std::string & node, std::string & message)
// {
//   const std::string state = nodeStateLabel(node);
//
//   if (state == "active") {
//     return changeNodeState(node, Transition::TRANSITION_DEACTIVATE, message);
//   }
//
//   if (state == "inactive" || state == "unconfigured" || state == "finalized") {
//     message = node + ": already down (state '" + state + "')";
//     return true;
//   }
//
//   message = node + ": cannot be brought down from state '" + state + "'";
//   return false;
// }
//
// // The shutdown transition id depends on the state the node is shut down from.
// bool WmxEngineNode::shutdownNode(const std::string & node, std::string & message)
// {
//   const std::string state = nodeStateLabel(node);
//
//   if (state == "finalized") {
//     message = node + ": already finalized";
//     return true;
//   }
//
//   uint8_t transition = Transition::TRANSITION_UNCONFIGURED_SHUTDOWN;
//   if (state == "active") {
//     transition = Transition::TRANSITION_ACTIVE_SHUTDOWN;
//   } else if (state == "inactive") {
//     transition = Transition::TRANSITION_INACTIVE_SHUTDOWN;
//   } else if (state != "unconfigured") {
//     message = node + ": cannot be shut down from state '" + state + "'";
//     return false;
//   }
//
//   return changeNodeState(node, transition, message);
// }
//
// // Take the lifecycle nodes off the device before the engine pulls it away from
// // under them. With cleanup they also release their device handle.
// // Reverse of the bring-up order: the controllers stop before the device-level
// // nodes whose axes they command.
// void WmxEngineNode::bringDownDiscoveredNodes(bool cleanup)
// {
//   const std::vector<std::string> nodes = discoverLifecycleNodes();
//
//   for (auto it = nodes.rbegin(); it != nodes.rend(); ++it) {
//     const std::string & node = *it;
//     std::string message;
//
//     if (bringDownNode(node, message)) {
//       RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
//     } else {
//       RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
//       continue;
//     }
//
//     if (cleanup && nodeStateLabel(node) == "inactive") {
//       if (changeNodeState(node, Transition::TRANSITION_CLEANUP, message)) {
//         RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
//       } else {
//         RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
//       }
//     }
//   }
// }
//
// // Bring up every lifecycle node the engine has not handled yet. Runs while the
// // engine is communicating, so a node that joins late (or respawns) is picked up
// // without anyone having to list it anywhere.
// void WmxEngineNode::discoveryStep()
// {
//   if (!isEngineStarted_) {
//     return;
//   }
//
//   if (!isCommStarted_) {
//     RCLCPP_WARN_THROTTLE(
//       this->get_logger(), *this->get_clock(), 30000,
//       "Engine is not communicating; lifecycle nodes stay unconfigured. "
//       "Start communication with wmx/engine/set_comm to bring them up.");
//     return;
//   }
//
//   const std::vector<std::string> discovered = discoverLifecycleNodes();
//
//   // Forget nodes that left the graph, so a respawned one is brought up again.
//   for (auto it = broughtUpNodes_.begin(); it != broughtUpNodes_.end(); ) {
//     if (std::find(discovered.begin(), discovered.end(), *it) == discovered.end()) {
//       RCLCPP_INFO(this->get_logger(), "%s left the graph", it->c_str());
//       it = broughtUpNodes_.erase(it);
//     } else {
//       ++it;
//     }
//   }
//
//   for (const std::string & node : discovered) {
//     if (broughtUpNodes_.count(node) > 0) {
//       continue;
//     }
//
//     std::string message;
//     if (bringUpNode(node, message)) {
//       RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
//       broughtUpNodes_.insert(node);
//     } else {
//       // Left out of broughtUpNodes_ so a transient failure (device lock, node
//       // still starting) is retried on the next sweep.
//       RCLCPP_ERROR_THROTTLE(
//         this->get_logger(), *this->get_clock(), 10000, "%s", message.c_str());
//     }
//   }
// }

void WmxEngineNode::startEngine()
{
  RCLCPP_INFO(
    this->get_logger(), "Starting engine... (core=%d, affinityMask=0x%" PRIx64 ")",
    engineCore_, engineAffinityMask_);
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
      devicePath_.c_str(), wmx3Api::DeviceType::DeviceTypeNormal, timeout,
      engineCore_, engineAffinityMask_);

    if (err == wmx3Api::ErrorCode::None) {
      wmx3Lib_.SetDeviceName(deviceName_.c_str());
      wmx3LibCm_ = std::make_unique<wmx3Api::CoreMotion>(&wmx3Lib_);
      RCLCPP_INFO(this->get_logger(), "Device created (attempt %d)", attempt + 1);

      std::string message;
      if (startCommunication(message) == wmx3Api::ErrorCode::None &&
        !wmxParamFilePath_.empty())
      {
        setWmxParam(wmxParamFilePath_, message);
      }

      isEngineStarted_ = true;
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
  isEngineStarted_ = true;
}

// Returns the WMX3 error code; message is what to report back.
int WmxEngineNode::startCommunication(std::string & message)
{
  unsigned int timeout = 10000;
  char errString[256];
  char buffer[512];

  const int err = wmx3Lib_.StartCommunication(timeout);
  if (err != wmx3Api::ErrorCode::None) {
    wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
    snprintf(
      buffer, sizeof(buffer),
      "Failed to start communication. Error=%d (%s)", err, errString);
    RCLCPP_ERROR(this->get_logger(), "%s", buffer);
  } else {
    isCommStarted_ = true;
    snprintf(buffer, sizeof(buffer), "Communication started");
    RCLCPP_INFO(this->get_logger(), "%s", buffer);
  }

  message = buffer;
  return err;
}

int WmxEngineNode::stopCommunication(std::string & message)
{
  unsigned int timeout = 10000;
  char errString[256];
  char buffer[512];

  const int err = wmx3Lib_.StopCommunication(timeout);
  if (err != wmx3Api::ErrorCode::None) {
    wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
    snprintf(
      buffer, sizeof(buffer),
      "Failed to stop communication. Error=%d (%s)", err, errString);
    RCLCPP_ERROR(this->get_logger(), "%s", buffer);
  } else {
    snprintf(buffer, sizeof(buffer), "Communication stopped");
    RCLCPP_INFO(this->get_logger(), "%s", buffer);
  }

  isCommStarted_ = false;

  message = buffer;
  return err;
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

void WmxEngineNode::getEngineStatusCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!isEngineStarted_) {
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

void WmxEngineNode::setCommCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (!isEngineStarted_) {
    response->success = false;
    response->message = "Engine startup in progress";
    return;
  }

  std::string message;
  int err;

  if (request->data) {
    err = startCommunication(message);
    // if (err == wmx3Api::ErrorCode::None) {
    //   // Whatever is already waiting on the graph can attach now.
    //   discoveryStep();
    // }
  } else {
    // The lifecycle nodes command axes over this link, so stop them first.
    // bringDownDiscoveredNodes(false);
    err = stopCommunication(message);
  }

  response->success = (err == wmx3Api::ErrorCode::None);
  response->message = message;
}

void WmxEngineNode::setEngineCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (!isEngineStarted_) {
    response->success = false;
    response->message = "Engine startup in progress";
    return;
  }

  unsigned int timeout = 10000;
  int err;
  char errString[256];
  char buffer[512];

  if (request->data) {
    err = wmx3Lib_.CreateDevice(
      devicePath_.c_str(), wmx3Api::DeviceType::DeviceTypeNormal, timeout,
      engineCore_, engineAffinityMask_);
    if (err != wmx3Api::ErrorCode::None) {
      wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
      snprintf(
        buffer, sizeof(buffer),
        "Failed to create device. Error=%d (%s)", err, errString);
      RCLCPP_ERROR(this->get_logger(), "%s", buffer);
      response->success = false;
      response->message = std::string(buffer);
    } else {
      wmx3Lib_.SetDeviceName(deviceName_.c_str());
      wmx3LibCm_ = std::make_unique<wmx3Api::CoreMotion>(&wmx3Lib_);
      snprintf(
        buffer, sizeof(buffer),
        "Created device '%s' at %s", deviceName_.c_str(), devicePath_.c_str());
      RCLCPP_INFO(this->get_logger(), "%s", buffer);
      response->success = true;
      response->message = std::string(buffer);
    }
  } else {
    // Closing the device invalidates the handles the lifecycle nodes hold, so
    // take them all the way down to unconfigured first.
    // bringDownDiscoveredNodes(true);

    wmx3LibCm_.reset();
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
      isCommStarted_ = false;
      snprintf(buffer, sizeof(buffer), "Device closed");
      RCLCPP_INFO(this->get_logger(), "%s", buffer);
      response->success = true;
      response->message = std::string(buffer);
    }
  }
}

// // ros2 service call /wmx/engine/set_node_state wmx_r2_message/srv/SetNodeState
// //   "{node_name: 'wmx_io_node', transition: 'deactivate'}"
// void WmxEngineNode::setNodeStateCallback(
//   const std::shared_ptr<wmx_r2_message::srv::SetNodeState::Request> request,
//   std::shared_ptr<wmx_r2_message::srv::SetNodeState::Response> response)
// {
//   if (request->node_name.empty()) {
//     response->success = false;
//     response->message =
//       "node_name is required. Call wmx/engine/get_node_states to list the nodes.";
//     return;
//   }
//
//   // A plain node name is resolved against this node's namespace; an absolute
//   // name is taken as given.
//   std::string node = request->node_name;
//   if (node.front() != '/') {
//     const std::string ns = this->get_effective_namespace();
//     node = (ns == "/") ? "/" + node : ns + "/" + node;
//   }
//
//   const std::string & transition = request->transition;
//   std::string message;
//   bool success;
//
//   if (transition == "configure") {
//     success = changeNodeState(node, Transition::TRANSITION_CONFIGURE, message);
//   } else if (transition == "activate") {
//     success = changeNodeState(node, Transition::TRANSITION_ACTIVATE, message);
//   } else if (transition == "deactivate") {
//     success = changeNodeState(node, Transition::TRANSITION_DEACTIVATE, message);
//   } else if (transition == "cleanup") {
//     success = changeNodeState(node, Transition::TRANSITION_CLEANUP, message);
//   } else if (transition == "shutdown") {
//     success = shutdownNode(node, message);
//   } else if (transition == "bringup") {
//     success = bringUpNode(node, message);
//   } else if (transition == "bringdown") {
//     success = bringDownNode(node, message);
//   } else {
//     response->success = false;
//     response->message = "Unknown transition '" + transition +
//       "'. Use configure, activate, deactivate, cleanup, shutdown, bringup or bringdown.";
//     return;
//   }
//
//   if (success) {
//     RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
//     // An explicit bring-up counts as handled, so the discovery sweep does not
//     // touch this node again; an explicit take-down is left as the operator set
//     // it, for the same reason.
//     broughtUpNodes_.insert(node);
//   } else {
//     RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
//   }
//
//   response->success = success;
//   response->message = message;
//   response->node_names.push_back(node);
//   response->states.push_back(nodeStateLabel(node));
// }
//
// void WmxEngineNode::getNodeStatesCallback(
//   const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
//   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
// {
//   const std::vector<std::string> nodes = discoverLifecycleNodes();
//
//   std::string message;
//   for (const std::string & node : nodes) {
//     message += node + ": " + nodeStateLabel(node) + "; ";
//   }
//
//   response->success = true;
//   response->message = message.empty() ? "No lifecycle nodes found." : message;
// }

// Import a WMX parameter file onto the device.
bool WmxEngineNode::setWmxParam(const std::string & path, std::string & message)
{
  if (!wmx3LibCm_) {
    message = "No device: create it with wmx/engine/set_engine first.";
    return false;
  }

  char errString[256];
  char buffer[512];
  wmx3Api::Config::SystemParam sysParamErr;
  wmx3Api::Config::AxisParam axisParamErr;

  const int err = wmx3LibCm_->config->ImportAndSetAll(
    const_cast<char *>(path.c_str()), &sysParamErr, &axisParamErr);

  if (err != wmx3Api::ErrorCode::None) {
    wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
    snprintf(
      buffer, sizeof(buffer),
      "Failed to load params from %s. Error=%d (%s)", path.c_str(), err, errString);
    RCLCPP_ERROR(this->get_logger(), "%s", buffer);
    message = buffer;
    return false;
  }

  snprintf(buffer, sizeof(buffer), "Loaded params from: %s", path.c_str());
  RCLCPP_INFO(this->get_logger(), "%s", buffer);
  message = buffer;
  return true;
}

void WmxEngineNode::getWmxParam(
  const std::vector<int32_t> & axes, std::vector<std::string> & dump)
{
  wmx3Api::Config::SystemParam sysParam;
  wmx3Api::Config::AxisParam axisParam;

  wmx3LibCm_->config->GetParam(&sysParam);
  wmx3LibCm_->config->GetAxisParam(&axisParam);

  for (int32_t i : axes) {
    dump.push_back("=== Axis " + std::to_string(i) + " ===");

    dump.push_back("[AxisParam]");
    dump.push_back(
      "  GearRatio          = " + std::to_string(axisParam.gearRatioNumerator[i]) +
      " / " + std::to_string(axisParam.gearRatioDenominator[i]));
    dump.push_back("  AxisUnit           = " + std::to_string(axisParam.axisUnit[i]));
    dump.push_back(
      "  AxisPolarity       = " +
      std::to_string(static_cast<int>(axisParam.axisPolarity[i])));
    dump.push_back(
      "  CommandMode        = " +
      std::to_string(static_cast<int>(axisParam.axisCommandMode[i])));
    dump.push_back("  MaxTrqLimit        = " + std::to_string(axisParam.maxTrqLimit[i]));
    dump.push_back("  MaxMotorSpeed      = " + std::to_string(axisParam.maxMotorSpeed[i]));
    dump.push_back(
      "  VelFeedforwardGain = " +
      std::to_string(axisParam.velocityFeedforwardGain[i]));

    dump.push_back("[HomeParam]");
    dump.push_back(
      "  HomeType           = " +
      std::to_string(static_cast<int>(sysParam.homeParam[i].homeType)));
    dump.push_back(
      "  HomeDirection      = " +
      std::to_string(static_cast<int>(sysParam.homeParam[i].homeDirection)));
    dump.push_back(
      "  HomingVelSlow      = " +
      std::to_string(sysParam.homeParam[i].homingVelocitySlow));
    dump.push_back(
      "  HomingVelFast      = " +
      std::to_string(sysParam.homeParam[i].homingVelocityFast));
    dump.push_back("  HomePosition       = " + std::to_string(sysParam.homeParam[i].homePosition));

    dump.push_back("[FeedbackParam]");
    dump.push_back(
      "  InPosWidth         = " +
      std::to_string(sysParam.feedbackParam[i].inPosWidth));
    dump.push_back(
      "  PosSetWidth        = " + std::to_string(
        sysParam.feedbackParam[i].posSetWidth));
    dump.push_back(
      "  DelayedPosSetWidth = " +
      std::to_string(sysParam.feedbackParam[i].delayedPosSetWidth));

    dump.push_back("[AlarmParam]");
    dump.push_back(
      "  FollowErrStopped   = " +
      std::to_string(sysParam.alarmParam[i].followingErrorStopped));
    dump.push_back(
      "  FollowErrMoving    = " +
      std::to_string(sysParam.alarmParam[i].followingErrorMoving));

    dump.push_back("[LimitParam]");
    dump.push_back(
      "  SoftLimitPosPos    = " +
      std::to_string(sysParam.limitParam[i].softLimitPositivePos));
    dump.push_back(
      "  SoftLimitNegPos    = " +
      std::to_string(sysParam.limitParam[i].softLimitNegativePos));
    dump.push_back("");
  }
}

void WmxEngineNode::loadWmxParamsCallback(
  const std::shared_ptr<wmx_r2_message::srv::LoadWmxParams::Request> request,
  std::shared_ptr<wmx_r2_message::srv::LoadWmxParams::Response> response)
{
  if (!isEngineStarted_) {
    response->success = false;
    response->message = "Engine startup in progress";
    return;
  }

  std::string message;
  response->success = setWmxParam(request->file_path, message);
  response->message = message;
}

void WmxEngineNode::getWmxParamsCallback(
  const std::shared_ptr<wmx_r2_message::srv::GetWmxParams::Request> request,
  std::shared_ptr<wmx_r2_message::srv::GetWmxParams::Response> response)
{
  if (!isEngineStarted_ || !wmx3LibCm_) {
    response->success = false;
    response->message = "No device: create it with wmx/engine/set_engine first.";
    return;
  }

  getWmxParam(request->index, response->params_dump);

  response->success = true;
  response->message = "OK";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WmxEngineNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
