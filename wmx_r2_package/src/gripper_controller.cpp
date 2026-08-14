// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "gripper_controller.hpp"

#include <cstdlib>

using std::placeholders::_1;
using std::placeholders::_2;

using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;
using wmx3Api::IO;

namespace
{
constexpr int32_t kCr3aGripperPowerByte = 28;
constexpr int32_t kCr3aGripperPowerValue = 113;
constexpr int32_t kCr3aGripperSenseByte = 0;
constexpr int32_t kCr3aGripperSenseBit = 1;

std::string errorText(int err)
{
  char errString[256] = {};
  IO::ErrorToString(err, errString, sizeof(errString));
  return errString;
}
}  // namespace

GripperControllerApi::GripperControllerApi(const rclcpp::Logger & logger)
: logger_(logger)
{
}

GripperControllerApi::~GripperControllerApi()
{
  releaseDevice();
}

int GripperControllerApi::attachDevice(std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (isDeviceAttached_) {
    message = "Already attached to the WMX3 device";
    return ErrorCode::None;
  }

  int err = wmx3Lib_.CreateDevice(WMX3_SDK_PATH, DeviceType::DeviceTypeNormal, timeout_);
  if (err != ErrorCode::None) {
    if (err == ErrorCode::StartProcessLockError) {
      message = "Failed to attach to device (lock busy). Is the engine communicating?";
    } else {
      message = "Failed to attach to device. Error=" + std::to_string(err) +
        " (" + errorText(err) + ")";
    }
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  err = wmx3Lib_.SetDeviceName(deviceName_);
  if (err != ErrorCode::None) {
    message = "Failed to name the device '" + std::string(deviceName_) + "'. Error=" +
      std::to_string(err) + " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    wmx3Lib_.CloseDevice();
    return err;
  }

  io_ = IO(&wmx3Lib_);
  isDeviceAttached_ = true;

  message = "Attached to WMX3 device";
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

void GripperControllerApi::releaseDevice()
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  const int err = wmx3Lib_.CloseDevice();
  if (err != ErrorCode::None) {
    RCLCPP_ERROR(logger_, "Failed to close device. Error=%d (%s)", err, errorText(err).c_str());
  } else {
    isDeviceAttached_ = false;
    RCLCPP_INFO(logger_, "Device closed");
  }
}

int GripperControllerApi::setOutputBit(
  int32_t byte, int32_t bit, int32_t value, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  const int err = io_.SetOutBit(byte, bit, static_cast<unsigned char>(value));
  if (err != ErrorCode::None) {
    message = "SetOutBit failed. byte=" + std::to_string(byte) + " bit=" + std::to_string(bit) +
      " value=" + std::to_string(value) + ". Error=" + std::to_string(err) +
      " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "SetOutBit success. byte=" + std::to_string(byte) + " bit=" + std::to_string(bit) +
    " value=" + std::to_string(value);
  return ErrorCode::None;
}

int GripperControllerApi::setOutputByte(int32_t byte, int32_t value, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  const int err = io_.SetOutByte(byte, static_cast<unsigned char>(value));
  if (err != ErrorCode::None) {
    message = "SetOutByte failed. byte=" + std::to_string(byte) + " value=" +
      std::to_string(value) + ". Error=" + std::to_string(err) + " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "SetOutByte success. byte=" + std::to_string(byte) + " value=" + std::to_string(value);
  return ErrorCode::None;
}

int GripperControllerApi::getOutputByte(int32_t byte, int32_t & value, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  unsigned char raw = 0;
  const int err = io_.GetOutByte(byte, &raw);
  if (err != ErrorCode::None) {
    message = "GetOutByte failed. byte=" + std::to_string(byte) + ". Error=" +
      std::to_string(err) + " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  value = raw;
  message = "GetOutByte success. byte=" + std::to_string(byte) + " value=" + std::to_string(value);
  return ErrorCode::None;
}

int GripperControllerApi::getInputBit(
  int32_t byte, int32_t bit, int32_t & value, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  unsigned char raw = 0;
  const int err = io_.GetInBit(byte, bit, &raw);
  if (err != ErrorCode::None) {
    message = "GetInBit failed. byte=" + std::to_string(byte) + " bit=" + std::to_string(bit) +
      ". Error=" + std::to_string(err) + " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  value = raw;
  message = "GetInBit success. byte=" + std::to_string(byte) + " bit=" + std::to_string(bit) +
    " value=" + std::to_string(value);
  return ErrorCode::None;
}

GripperController::GripperController()
: LifecycleNode("gripper_controller")
{
  setRosParameter();

  api_ = std::make_unique<GripperControllerApi>(this->get_logger());

  RCLCPP_INFO(this->get_logger(), "gripper_controller is unconfigured, waiting for configure...");
}

GripperController::~GripperController()
{
  api_.reset();
  RCLCPP_INFO(this->get_logger(), "gripper_controller stopped");
}

bool GripperController::isNodeActive() const
{
  return isNodeActive_.load();
}

std::string GripperController::notActiveMessage() const
{
  return "gripper_controller is not active (state: " +
         this->get_current_state().label() + ").";
}

void GripperController::setRosParameter()
{
  wmxGripperTopic_ = this->declare_parameter<std::string>(
    "wmx_gripper_topic", "/wmx_gripper_topic/no_param");
  gripperAddress_ = this->declare_parameter<std::vector<int64_t>>(
    "gripper_address", std::vector<int64_t>{0, 0});

  if (gripperAddress_.size() < 2) {
    RCLCPP_WARN(
      this->get_logger(),
      "gripper_address needs [byte, bit], got %zu entries. Falling back to [0, 0].",
      gripperAddress_.size());
    gripperAddress_ = {0, 0};
  }

  RCLCPP_INFO(this->get_logger(), "===== ROS2 Parameters =====");
  RCLCPP_INFO(this->get_logger(), "wmx_gripper_topic: %s", wmxGripperTopic_.c_str());
  RCLCPP_INFO(
    this->get_logger(), "gripper_address: [%ld, %ld]",
    gripperAddress_[0], gripperAddress_[1]);
  RCLCPP_INFO(this->get_logger(), "===========================");
}

GripperController::CallbackReturn GripperController::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring gripper_controller...");

  std::string message;
  if (api_->attachDevice(message) != ErrorCode::None) {
    return CallbackReturn::FAILURE;
  }

  const char * manipulatorModel = std::getenv("MANIPULATOR_MODEL");
  if (manipulatorModel && std::string(manipulatorModel) == "dobot_cr3a") {
    dobotCR3AGripperSetup();
  } else {
    RCLCPP_INFO(
      this->get_logger(), "Skipping dobotCR3AGripperSetup (MANIPULATOR_MODEL=%s)",
      manipulatorModel ? manipulatorModel : "not set");
  }

  setGripperService_ = this->create_service<std_srvs::srv::SetBool>(
    wmxGripperTopic_,
    std::bind(&GripperController::setGripperCallback, this, _1, _2));

  RCLCPP_INFO(this->get_logger(), "gripper_controller is configured");
  return CallbackReturn::SUCCESS;
}

GripperController::CallbackReturn GripperController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  LifecycleNode::on_activate(previous_state);
  isNodeActive_ = true;

  RCLCPP_INFO(this->get_logger(), "gripper_controller is active");
  return CallbackReturn::SUCCESS;
}

GripperController::CallbackReturn GripperController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  isNodeActive_ = false;

  LifecycleNode::on_deactivate(previous_state);
  RCLCPP_INFO(this->get_logger(), "gripper_controller is inactive");
  return CallbackReturn::SUCCESS;
}

GripperController::CallbackReturn GripperController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  isNodeActive_ = false;

  setGripperService_.reset();

  api_->releaseDevice();

  RCLCPP_INFO(this->get_logger(), "gripper_controller is cleaned up");
  return CallbackReturn::SUCCESS;
}

GripperController::CallbackReturn GripperController::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

void GripperController::dobotCR3AGripperSetup()
{
  std::string message;

  if (api_->setOutputByte(
      kCr3aGripperPowerByte, kCr3aGripperPowerValue, message) != ErrorCode::None)
  {
    RCLCPP_ERROR(this->get_logger(), "[dobot_cr3a] gripper setup failed: %s", message.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "[dobot_cr3a] gripper power byte set");

  int32_t switchData = 0;
  if (api_->getOutputByte(kCr3aGripperPowerByte, switchData, message) != ErrorCode::None) {
    RCLCPP_ERROR(this->get_logger(), "[dobot_cr3a] gripper setup failed: %s", message.c_str());
    return;
  }

  int32_t powerData = 0;
  if (api_->getInputBit(
      kCr3aGripperSenseByte, kCr3aGripperSenseBit, powerData, message) != ErrorCode::None)
  {
    RCLCPP_ERROR(this->get_logger(), "[dobot_cr3a] gripper setup failed: %s", message.c_str());
    return;
  }

  if (switchData == kCr3aGripperPowerValue && powerData == 1) {
    RCLCPP_INFO(this->get_logger(), "[dobot_cr3a] gripper is on and ready");
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "[dobot_cr3a] gripper state unexpected: switchData=%d, powerData=%d",
      switchData, powerData);
  }
}

void GripperController::setGripperCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (!isNodeActive()) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  const int32_t value = request->data ? 1 : 0;
  const char * action = request->data ? "close" : "open";

  std::string message;
  if (api_->setOutputBit(
      static_cast<int32_t>(gripperAddress_[0]), static_cast<int32_t>(gripperAddress_[1]),
      value, message) != ErrorCode::None)
  {
    RCLCPP_ERROR(this->get_logger(), "Gripper fails to %s: %s", action, message.c_str());
    response->success = false;
    response->message = std::string("Failed to ") + action + " gripper";
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Gripper success to %s", action);
  response->success = true;
  response->message = std::string("Gripper ") + action + "ed successfully";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperController>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
