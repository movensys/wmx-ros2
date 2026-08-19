// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_engine_node.hpp"

#include <cinttypes>

using std::placeholders::_1;
using std::placeholders::_2;

namespace
{
const char * engineStateLabel(wmx3Api::EngineState::T state)
{
  switch (state) {
    case wmx3Api::EngineState::Idle:          return "Idle";
    case wmx3Api::EngineState::Running:       return "Running";
    case wmx3Api::EngineState::Communicating: return "Communicating";
    case wmx3Api::EngineState::Shutdown:      return "Shutdown";
    case wmx3Api::EngineState::Unknown:       return "Unknown";
    default:                                  return "Invalid";
  }
}
}  // namespace

WmxEngineNodeApi::WmxEngineNodeApi(const rclcpp::Logger & logger, const Config & config)
: logger_(logger), config_(config)
{
}

WmxEngineNodeApi::~WmxEngineNodeApi()
{
  std::string message;
  stopCommunication(message);
  stopEngine(message);
}

void WmxEngineNodeApi::setConfig(const Config & config)
{
  std::lock_guard<std::recursive_mutex> lock(deviceMutex_);
  config_ = config;
}

std::string WmxEngineNodeApi::errorText(int err)
{
  char errString[256] = {};
  wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
  return errString;
}

int WmxEngineNodeApi::startEngine(std::string & message)
{
  std::lock_guard<std::recursive_mutex> lock(deviceMutex_);

  RCLCPP_INFO(
    logger_, "Starting engine... (core=%d, affinityMask=0x%" PRIx64 ")",
    config_.core, config_.affinityMask);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  int err = wmx3Api::ErrorCode::None;

  for (int attempt = 0; attempt < maxRetries_; attempt++) {
    if (attempt > 0) {
      RCLCPP_INFO(
        logger_, "Retrying device creation (attempt %d/%d)...", attempt + 1, maxRetries_);
      std::this_thread::sleep_for(std::chrono::milliseconds(retryDelay_));
    }

    err = wmx3Lib_.CreateDevice(
      WMX3_SDK_PATH, wmx3Api::DeviceType::DeviceTypeNormal, timeout_,
      config_.core, config_.affinityMask);

    if (err == wmx3Api::ErrorCode::None) {
      wmx3Lib_.SetDeviceName(deviceName_);
      RCLCPP_INFO(logger_, "Device created (attempt %d)", attempt + 1);

      return startCommunication(message);
    }

    if (err == wmx3Api::ErrorCode::CreateDeviceLockError) {
      RCLCPP_WARN(
        logger_, "Device lock error (attempt %d/%d). Waiting...", attempt + 1, maxRetries_);
    } else {
      RCLCPP_WARN(
        logger_, "Failed to create device (attempt %d/%d). Error=%d (%s)",
        attempt + 1, maxRetries_, err, errorText(err).c_str());
    }
  }

  message = "Failed to create device after " + std::to_string(maxRetries_) +
    " attempts. Error=" + std::to_string(err) + " (" + errorText(err) + ")";
  RCLCPP_ERROR(logger_, "%s", message.c_str());
  return err;
}

int WmxEngineNodeApi::stopEngine(std::string & message)
{
  std::lock_guard<std::recursive_mutex> lock(deviceMutex_);

  int err = wmx3Lib_.StopEngine(timeout_);
  if (err != wmx3Api::ErrorCode::None) {
    message = "Failed to stop engine. Error=" + std::to_string(err) +
      " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
  } else {
    message = "Engine stopped";
    RCLCPP_INFO(logger_, "%s", message.c_str());
  }

  err = wmx3Lib_.CloseDevice();
  if (err != wmx3Api::ErrorCode::None) {
    message = "Failed to close device. Error=" + std::to_string(err) +
      " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
  } else {
    message = "Device closed";
    RCLCPP_INFO(logger_, "%s", message.c_str());
  }

  return err;
}

int WmxEngineNodeApi::createDevice(
  const std::string & path, const std::string & name, std::string & message)
{
  std::lock_guard<std::recursive_mutex> lock(deviceMutex_);

  const int err = wmx3Lib_.CreateDevice(
    path.c_str(), wmx3Api::DeviceType::DeviceTypeNormal, timeout_,
    config_.core, config_.affinityMask);

  if (err != wmx3Api::ErrorCode::None) {
    message = "Failed to create device. Error=" + std::to_string(err) +
      " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  wmx3Lib_.SetDeviceName(name.c_str());
  message = "Created device with name: " + name;
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return err;
}

int WmxEngineNodeApi::closeDevice(std::string & message)
{
  std::lock_guard<std::recursive_mutex> lock(deviceMutex_);

  const int err = wmx3Lib_.CloseDevice();
  if (err != wmx3Api::ErrorCode::None) {
    message = "Failed to close device. Error=" + std::to_string(err) +
      " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  isCommStarted_ = false;
  message = "Device closed";
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return err;
}

int WmxEngineNodeApi::startCommunication(std::string & message)
{
  std::lock_guard<std::recursive_mutex> lock(deviceMutex_);

  const int err = wmx3Lib_.StartCommunication(timeout_);
  if (err != wmx3Api::ErrorCode::None) {
    message = "Failed to start communication. Error=" + std::to_string(err) +
      " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
  } else {
    isCommStarted_ = true;
    message = "Communication started";
    RCLCPP_INFO(logger_, "%s", message.c_str());
  }

  return err;
}

int WmxEngineNodeApi::stopCommunication(std::string & message)
{
  std::lock_guard<std::recursive_mutex> lock(deviceMutex_);

  const int err = wmx3Lib_.StopCommunication(timeout_);
  if (err != wmx3Api::ErrorCode::None) {
    message = "Failed to stop communication. Error=" + std::to_string(err) +
      " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
  } else {
    isCommStarted_ = false;
    message = "Communication stopped";
    RCLCPP_INFO(logger_, "%s", message.c_str());
  }

  return err;
}

std::string WmxEngineNodeApi::getEngineStatus()
{
  std::lock_guard<std::recursive_mutex> lock(deviceMutex_);

  wmx3Api::EngineStatus status;
  wmx3Lib_.GetEngineStatus(&status);
  return engineStateLabel(status.state);
}

WmxEngineNode::WmxEngineNode()
: Node("wmx_engine_node")
{
  this->declare_parameter<int>("engine_core", -1);
  this->declare_parameter<int64_t>("engine_affinity_mask", 0);

  api_ = std::make_unique<WmxEngineNodeApi>(this->get_logger(), readConfig());

  auto ready_qos = rclcpp::QoS(1).reliable().transient_local();
  engineReadyPub_ = this->create_publisher<std_msgs::msg::Bool>("wmx/engine/ready", ready_qos);

  setEngineService_ = this->create_service<wmx_r2_message::srv::SetEngine>(
    "wmx/engine/set_engine",
    std::bind(&WmxEngineNode::setEngineCallback, this, _1, _2));

  setCommService_ = this->create_service<std_srvs::srv::SetBool>(
    "wmx/engine/set_comm",
    std::bind(&WmxEngineNode::setCommCallback, this, _1, _2));

  getEngineStatusService_ = this->create_service<std_srvs::srv::Trigger>(
    "wmx/engine/get_engine_status",
    std::bind(&WmxEngineNode::getEngineStatusCallback, this, _1, _2));

  readyTimer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000),
    std::bind(&WmxEngineNode::publishReady, this));

  startThread_ = std::thread(&WmxEngineNode::startEngine, this);

  RCLCPP_INFO(this->get_logger(), "wmx_engine_node is ready");
}

WmxEngineNode::~WmxEngineNode()
{
  if (startThread_.joinable()) {
    startThread_.join();
  }
  api_.reset();
  std::this_thread::sleep_for(std::chrono::seconds(3));
  RCLCPP_INFO(this->get_logger(), "wmx_engine_node stopped");
}

WmxEngineNodeApi::Config WmxEngineNode::readConfig()
{
  WmxEngineNodeApi::Config config;
  config.core = static_cast<int>(this->get_parameter("engine_core").as_int());
  config.affinityMask = this->get_parameter("engine_affinity_mask").as_int();
  return config;
}

void WmxEngineNode::startEngine()
{
  std::string message;
  api_->startEngine(message);
  startComplete_ = true;
}

void WmxEngineNode::publishReady()
{
  auto msg = std_msgs::msg::Bool();
  msg.data = api_->isCommStarted();
  engineReadyPub_->publish(msg);
}

void WmxEngineNode::getEngineStatusCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!startComplete_) {
    response->success = false;
    response->message = "Engine startup in progress";
    return;
  }

  response->success = true;
  response->message = api_->getEngineStatus();
}

void WmxEngineNode::setCommCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (!startComplete_) {
    response->success = false;
    response->message = "Engine startup in progress";
    return;
  }

  std::string message;
  const int err = request->data ?
    api_->startCommunication(message) :
    api_->stopCommunication(message);

  response->success = (err == wmx3Api::ErrorCode::None);
  response->message = message;

  if (response->success) {
    publishReady();
  }
}

void WmxEngineNode::setEngineCallback(
  const std::shared_ptr<wmx_r2_message::srv::SetEngine::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetEngine::Response> response)
{
  if (!startComplete_) {
    response->success = false;
    response->message = "Engine startup in progress";
    return;
  }

  std::string message;
  int err;

  if (request->data) {
    api_->setConfig(readConfig());
    err = api_->createDevice(request->path, request->name, message);
  } else {
    err = api_->closeDevice(message);
  }

  response->success = (err == wmx3Api::ErrorCode::None);
  response->message = message;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WmxEngineNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
