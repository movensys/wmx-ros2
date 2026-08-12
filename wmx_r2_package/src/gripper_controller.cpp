// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include <atomic>
#include <memory>
#include <thread>
#include <sstream>
#include <chrono>
#include <cstdlib>
#include <string>
#include <vector>

#include "WMX3Api.h"
#include "IOApi.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/bool.hpp"

using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;
using wmx3Api::IO;
using wmx3Api::WMX3Api;

// Managed node: wmx_engine_node drives the transitions once the engine is
// communicating (see wmx/engine/set_node_state).
class GripperController : public rclcpp_lifecycle::LifecycleNode {
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  GripperController();
  ~GripperController() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<int64_t> gripperAddress;
  std::string wmxGripperTopic_;

  unsigned char gripperSwitchData_;
  unsigned char gripperPowerData_;

  int err_;
  char errString_[256];

private:
  std::atomic<bool> active_{false};
  bool deviceAttached_ = false;

  WMX3Api wmx3Lib_;
  IO Wmx3Lib_Io_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr setGripperService_;

  // Service callback declaration
  void setGripper(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  void setRosParameter();
  bool attachDevice();
  void releaseDevice();
  void DobotCR3AGripperSetup();
};

GripperController::GripperController() : LifecycleNode("gripper_controller"){
  setRosParameter();

  RCLCPP_INFO(this->get_logger(), "gripper_controller is unconfigured, waiting for configure...");
}

GripperController::~GripperController(){
  releaseDevice();

  RCLCPP_INFO(this->get_logger(), "gripper_controller is stopped");
}

// Attach to the device the engine created. Returns false so the transition can
// fail loudly instead of leaving the node inactive with no device.
bool GripperController::attachDevice(){
  if (deviceAttached_) {
    return true;
  }

  unsigned int timeout = 10000;
  err_ = wmx3Lib_.CreateDevice("/opt/lmx/", DeviceType::DeviceTypeNormal, timeout);

  if (err_ != ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to attach to device. Error=%d (%s)", err_, errString_);
    return false;
  }

  wmx3Lib_.SetDeviceName("gripper_controller");
  deviceAttached_ = true;
  RCLCPP_INFO(this->get_logger(), "Attached to WMX3 device");
  return true;
}

void GripperController::releaseDevice(){
  if (!deviceAttached_) {
    return;
  }

  err_ = wmx3Lib_.CloseDevice();
  if (err_ != ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    RCLCPP_ERROR(this->get_logger(), "Failed to close device");
  } else {
    RCLCPP_INFO(this->get_logger(), "Device closed");
  }
  deviceAttached_ = false;
}

GripperController::CallbackReturn GripperController::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring gripper_controller...");

  if (!attachDevice()) {
    return CallbackReturn::FAILURE;
  }

  Wmx3Lib_Io_ = IO(&wmx3Lib_);

  const char* manipulatorModel = std::getenv("MANIPULATOR_MODEL");
  if (manipulatorModel && std::string(manipulatorModel) == "dobot_cr3a") {
    DobotCR3AGripperSetup();
  } else {
    RCLCPP_INFO(this->get_logger(),
      "Skipping DobotCR3AGripperSetup (MANIPULATOR_MODEL=%s)",
      manipulatorModel ? manipulatorModel : "not set");
  }

  setGripperService_ = this->create_service<std_srvs::srv::SetBool>(wmxGripperTopic_,
                    std::bind(&GripperController::setGripper, this,
                    std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "gripper_controller is configured");
  return CallbackReturn::SUCCESS;
}

GripperController::CallbackReturn GripperController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  LifecycleNode::on_activate(previous_state);
  active_ = true;
  RCLCPP_INFO(this->get_logger(), "gripper_controller is active");
  return CallbackReturn::SUCCESS;
}

GripperController::CallbackReturn GripperController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  active_ = false;
  LifecycleNode::on_deactivate(previous_state);
  RCLCPP_INFO(this->get_logger(), "gripper_controller is inactive");
  return CallbackReturn::SUCCESS;
}

GripperController::CallbackReturn GripperController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  setGripperService_.reset();
  releaseDevice();

  RCLCPP_INFO(this->get_logger(), "gripper_controller is cleaned up");
  return CallbackReturn::SUCCESS;
}

GripperController::CallbackReturn GripperController::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

void GripperController::DobotCR3AGripperSetup(){
  err_ = Wmx3Lib_Io_.SetOutByte(28, 113);
  if (err_ != ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    RCLCPP_ERROR(
      this->get_logger(),
      "[dobot_cr3a] gripper setup failed (SetOutByte): %s", errString_);
    return;
  }
  RCLCPP_INFO(this->get_logger(), "[dobot_cr3a] gripper power byte set");

  err_ = Wmx3Lib_Io_.GetOutByte(28, &gripperSwitchData_);
  if (err_ != ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    RCLCPP_ERROR(
      this->get_logger(),
      "[dobot_cr3a] gripper setup failed (GetOutByte): %s", errString_);
    return;
  }

  err_ = Wmx3Lib_Io_.GetInBit(0, 1, &gripperPowerData_);
  if (err_ != ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    RCLCPP_ERROR(
      this->get_logger(),
      "[dobot_cr3a] gripper setup failed (GetInBit): %s", errString_);
    return;
  }

  if (gripperSwitchData_ == 113 && gripperPowerData_ == 1) {
    RCLCPP_INFO(this->get_logger(), "[dobot_cr3a] gripper is on and ready");
  } else {
    RCLCPP_WARN(this->get_logger(),
      "[dobot_cr3a] gripper state unexpected: switchData=%d, powerData=%d",
      gripperSwitchData_, gripperPowerData_);
  }
}

void GripperController::setRosParameter(){
  this->declare_parameter<std::string>("wmx_gripper_topic", "/wmx_gripper_topic/no_param");
  this->declare_parameter<std::vector<int64_t>>("gripper_address", std::vector<int64_t>{0, 0});

  this->get_parameter("wmx_gripper_topic", wmxGripperTopic_);
  this->get_parameter("gripper_address", gripperAddress);

  // Print parameter values
  RCLCPP_INFO(this->get_logger(), "===== ROS2 Parameters =====");
  RCLCPP_INFO(this->get_logger(), "wmx_gripper_topic: %s", wmxGripperTopic_.c_str());
  RCLCPP_INFO(
    this->get_logger(), "gripper_address: [%ld, %ld]",
    gripperAddress[0], gripperAddress[1]);
  RCLCPP_INFO(this->get_logger(), "===========================");
}

void GripperController::setGripper(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                        std::shared_ptr<std_srvs::srv::SetBool::Response> response){
  if (!active_) {
    response->success = false;
    response->message = "gripper_controller is not active (state: " +
                        this->get_current_state().label() + ").";
    return;
  }

  if (request->data) {
    err_ = Wmx3Lib_Io_.SetOutBit(gripperAddress[0], gripperAddress[1], 1);
    if (err_ != ErrorCode::None) {
      wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
      RCLCPP_ERROR(this->get_logger(), "Gripper fails to Close: %s", errString_);
      response->success = false;
      response->message = "Failed to close gripper";
    } else {
      RCLCPP_INFO(this->get_logger(), "Gripper success to Close");
      response->success = true;
      response->message = "Gripper closed successfully";
    }
  } else {
    err_ = Wmx3Lib_Io_.SetOutBit(gripperAddress[0], gripperAddress[1], 0);
    if (err_ != ErrorCode::None) {
      wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
      RCLCPP_ERROR(this->get_logger(), "Gripper fails to Open: %s", errString_);
      response->success = false;
      response->message = "Failed to open gripper";
    } else {
      RCLCPP_INFO(this->get_logger(), "Gripper success to Open");
      response->success = true;
      response->message = "Gripper opened successfully";
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperController>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
