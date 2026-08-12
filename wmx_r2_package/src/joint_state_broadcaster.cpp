// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "wmx_r2_message/srv/set_axis.hpp"

#include "WMX3Api.h"
#include "CoreMotionApi.h"
#include "IOApi.h"

using std::placeholders::_1;
using wmx3Api::CoreMotion;
using wmx3Api::CoreMotionStatus;
using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;
using wmx3Api::IO;
using wmx3Api::WMX3Api;

class JointStateBroadcaster : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  JointStateBroadcaster();
  ~JointStateBroadcaster() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  int jointFeedbackRate_;
  float gripperCloseValue_;
  float gripperOpenValue_;
  std::vector<int64_t> jointAxes_;
  std::vector<std::string> jointNames_;
  std::vector<std::string> gripperJointNames_;
  std::vector<int64_t> gripperAddress_;
  std::string encoderJointTopic_;
  std::string isaacsimJointTopic_;
  std::string gazeboJointTopic_;

  unsigned char gripperData_;

  int err_;
  char errString_[256];

private:
  std::atomic<bool> isActive_{false};
  bool isDeviceAttached_ = false;

  WMX3Api wmx3Lib_;
  CoreMotionStatus cmStatus_;
  std::unique_ptr<CoreMotion> wmx3LibCm_;
  std::unique_ptr<IO> wmx3Lib_Io_;

  rclcpp::CallbackGroup::SharedPtr axisClientCbGroup_;
  rclcpp::Client<wmx_r2_message::srv::SetAxis>::SharedPtr clearAlarmClient_;
  rclcpp::Client<wmx_r2_message::srv::SetAxis>::SharedPtr setAxisOnClient_;

  rclcpp::TimerBase::SharedPtr encoderJointTimer_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr encoderJointPub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr isaacsimJointPub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr gazeboJointPub_;

  bool attachDevice();
  void releaseDevice();
  void servoOff();
  bool callSetAxisService(rclcpp::Client<wmx_r2_message::srv::SetAxis>::SharedPtr client,
                          const std::string & service_name,
                          const std::vector<int64_t> & index,
                          const std::vector<int64_t> & data);
  void publishJointState();
  void setRosParameter();
};

JointStateBroadcaster::JointStateBroadcaster()
: LifecycleNode("joint_state_broadcaster")
{
  RCLCPP_INFO(this->get_logger(), "start joint_state_broadcaster");

  setRosParameter();

  axisClientCbGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  clearAlarmClient_ = this->create_client<wmx_r2_message::srv::SetAxis>(
    "wmx/axis/clear_alarm", rclcpp::ServicesQoS(), axisClientCbGroup_);

  setAxisOnClient_ = this->create_client<wmx_r2_message::srv::SetAxis>(
    "wmx/axis/set_on", rclcpp::ServicesQoS(), axisClientCbGroup_);

  RCLCPP_INFO(
    this->get_logger(), "joint_state_broadcaster is unconfigured, waiting for configure...");
}

JointStateBroadcaster::~JointStateBroadcaster()
{
  RCLCPP_INFO(this->get_logger(), "Stop joint_state_broadcaster");

  servoOff();
  releaseDevice();

  RCLCPP_INFO(this->get_logger(), "joint_state_broadcaster is stopped");
}

bool JointStateBroadcaster::attachDevice()
{
  if (isDeviceAttached_) {
    return true;
  }

  unsigned int timeout = 10000;
  err_ = wmx3Lib_.CreateDevice(WMX3_SDK_PATH, DeviceType::DeviceTypeNormal, timeout);

  if (err_ != ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    if (err_ == ErrorCode::StartProcessLockError) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to attach to device (lock busy). Is the engine communicating?");
    } else {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to attach to device. Error=%d (%s)", err_, errString_);
    }
    return false;
  }

  wmx3Lib_.SetDeviceName("joint_state_broadcaster");
  isDeviceAttached_ = true;
  RCLCPP_INFO(this->get_logger(), "Attached to WMX3 device");
  return true;
}

void JointStateBroadcaster::releaseDevice()
{
  if (!isDeviceAttached_) {
    return;
  }

  wmx3Lib_Io_.reset();
  wmx3LibCm_.reset();

  err_ = wmx3Lib_.CloseDevice();
  if (err_ != ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    RCLCPP_ERROR(this->get_logger(), "Failed to close device. Error=%d (%s)", err_, errString_);
  } else {
    RCLCPP_INFO(this->get_logger(), "Device closed");
  }
  isDeviceAttached_ = false;
}

void JointStateBroadcaster::servoOff()
{
  if (!isActive_ || !wmx3LibCm_) {
    return;
  }

  for (int axis : jointAxes_) {
    err_ = wmx3LibCm_->axisControl->SetServoOn(axis, 0);
    if (err_ != ErrorCode::None) {
      wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
      RCLCPP_ERROR(
        this->get_logger(), "Servo %d error to off. Error=%d (%s)", axis, err_, errString_);
    } else {
      RCLCPP_INFO(this->get_logger(), "Servo %d off", axis);
    }
  }
}

JointStateBroadcaster::CallbackReturn JointStateBroadcaster::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring joint_state_broadcaster...");

  if (!attachDevice()) {
    return CallbackReturn::FAILURE;
  }

  wmx3LibCm_ = std::make_unique<CoreMotion>(&wmx3Lib_);
  wmx3Lib_Io_ = std::make_unique<IO>(&wmx3Lib_);

  encoderJointPub_ = this->create_publisher<sensor_msgs::msg::JointState>(encoderJointTopic_, 1);
  isaacsimJointPub_ = this->create_publisher<sensor_msgs::msg::JointState>(isaacsimJointTopic_, 1);
  gazeboJointPub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(gazeboJointTopic_, 1);

  encoderJointTimer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / jointFeedbackRate_),
    std::bind(&JointStateBroadcaster::publishJointState, this));
  encoderJointTimer_->cancel();

  RCLCPP_INFO(this->get_logger(), "joint_state_broadcaster is configured");
  return CallbackReturn::SUCCESS;
}

JointStateBroadcaster::CallbackReturn JointStateBroadcaster::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  std::vector<int64_t> zeroData(jointAxes_.size(), 0);
  std::vector<int64_t> onData(jointAxes_.size(), 1);

  if (!callSetAxisService(clearAlarmClient_, "wmx/axis/clear_alarm", jointAxes_, zeroData)) {
    RCLCPP_ERROR(this->get_logger(), "Activation failed at clear_alarm");
    return CallbackReturn::FAILURE;
  }

  if (!callSetAxisService(setAxisOnClient_, "wmx/axis/set_on", jointAxes_, onData)) {
    RCLCPP_ERROR(this->get_logger(), "Activation failed at set_on");
    return CallbackReturn::FAILURE;
  }

  LifecycleNode::on_activate(previous_state);
  isActive_ = true;
  encoderJointTimer_->reset();

  RCLCPP_INFO(this->get_logger(), "joint_state_broadcaster is active");
  return CallbackReturn::SUCCESS;
}

JointStateBroadcaster::CallbackReturn JointStateBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  encoderJointTimer_->cancel();
  servoOff();
  isActive_ = false;

  LifecycleNode::on_deactivate(previous_state);
  RCLCPP_INFO(this->get_logger(), "joint_state_broadcaster is inactive");
  return CallbackReturn::SUCCESS;
}

JointStateBroadcaster::CallbackReturn JointStateBroadcaster::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  isActive_ = false;

  encoderJointTimer_.reset();
  encoderJointPub_.reset();
  isaacsimJointPub_.reset();
  gazeboJointPub_.reset();

  releaseDevice();

  RCLCPP_INFO(this->get_logger(), "joint_state_broadcaster is cleaned up");
  return CallbackReturn::SUCCESS;
}

JointStateBroadcaster::CallbackReturn JointStateBroadcaster::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

bool JointStateBroadcaster::callSetAxisService(
              rclcpp::Client<wmx_r2_message::srv::SetAxis>::SharedPtr client,
              const std::string & service_name,
              const std::vector<int64_t> & index,
              const std::vector<int64_t> & data)
{
  const int max_retries = 5;
  const auto service_timeout = std::chrono::seconds(10);
  const auto call_timeout = std::chrono::seconds(15);

  if (!client->wait_for_service(service_timeout)) {
    RCLCPP_ERROR(this->get_logger(), "Service %s not available", service_name.c_str());
    return false;
  }

  for (int attempt = 1; attempt <= max_retries; attempt++) {
    auto request = std::make_shared<wmx_r2_message::srv::SetAxis::Request>();
    request->index.assign(index.begin(), index.end());
    request->data.assign(data.begin(), data.end());

    RCLCPP_INFO(
      this->get_logger(), "Calling %s (attempt %d/%d)",
      service_name.c_str(), attempt, max_retries);

    auto future = client->async_send_request(request);
    if (future.wait_for(call_timeout) != std::future_status::ready) {
      RCLCPP_WARN(
        this->get_logger(), "Service call %s timed out (attempt %d/%d)",
        service_name.c_str(), attempt, max_retries);
      continue;
    }

    auto response = future.get();
    if (!response->success) {
      // Server may not be active yet -- retry instead of aborting
      if (response->message.find("not active") != std::string::npos ||
        response->message.find("not initialized") != std::string::npos)
      {
        RCLCPP_WARN(
          this->get_logger(),
          "%s: server not ready yet (attempt %d/%d), retrying...",
          service_name.c_str(), attempt, max_retries);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        continue;
      }
      RCLCPP_ERROR(
        this->get_logger(), "%s failed: %s",
        service_name.c_str(), response->message.c_str());
      return false;
    }

    RCLCPP_INFO(
      this->get_logger(), "%s succeeded: %s",
      service_name.c_str(), response->message.c_str());
    return true;
  }

  RCLCPP_ERROR(
    this->get_logger(), "Service call %s failed after %d attempts",
    service_name.c_str(), max_retries);
  return false;
}

void JointStateBroadcaster::setRosParameter()
{
  this->declare_parameter<std::vector<int64_t>>("joint_axes", std::vector<int64_t>{});
  this->declare_parameter<int>("joint_feedback_rate", 0);
  this->declare_parameter<float>("gripper_open_value", 0);
  this->declare_parameter<float>("gripper_close_value", 0);
  this->declare_parameter<std::vector<std::string>>(
    "joint_name", {"j1", "j2", "j3", "j4", "j5", "j6"});
  this->declare_parameter<std::vector<std::string>>(
    "gripper_joint_name", std::vector<std::string>{});
  this->declare_parameter<std::vector<int64_t>>("gripper_address", std::vector<int64_t>{0, 0});
  this->declare_parameter<std::string>("encoder_joint_topic", "/encoder_joint_topic/no_param");
  this->declare_parameter<std::string>("isaacsim_joint_topic", "/isaacsim_joint_topic/no_param");
  this->declare_parameter<std::string>("gazebo_joint_topic", "/gazebo_joint_topic/no_param");

  this->get_parameter("joint_axes", jointAxes_);
  this->get_parameter("joint_feedback_rate", jointFeedbackRate_);
  this->get_parameter("gripper_open_value", gripperOpenValue_);
  this->get_parameter("gripper_close_value", gripperCloseValue_);
  this->get_parameter("joint_name", jointNames_);
  this->get_parameter("gripper_joint_name", gripperJointNames_);
  this->get_parameter("gripper_address", gripperAddress_);
  this->get_parameter("encoder_joint_topic", encoderJointTopic_);
  this->get_parameter("isaacsim_joint_topic", isaacsimJointTopic_);
  this->get_parameter("gazebo_joint_topic", gazeboJointTopic_);

  RCLCPP_INFO(this->get_logger(), "===== ROS2 Parameters =====");
  RCLCPP_INFO(this->get_logger(), "joint_feedback_rate: %d", jointFeedbackRate_);

  std::string joint_names_str;
  for (size_t i = 0; i < jointNames_.size(); ++i) {
    if (i > 0) {joint_names_str += ", ";}
    joint_names_str += jointNames_[i];
  }
  RCLCPP_INFO(this->get_logger(), "joint_name: [%s]", joint_names_str.c_str());

  std::string joint_axes_str;
  for (size_t i = 0; i < jointAxes_.size(); ++i) {
    if (i > 0) {joint_axes_str += ", ";}
    joint_axes_str += std::to_string(jointAxes_[i]);
  }
  RCLCPP_INFO(this->get_logger(), "joint_axes: [%s]", joint_axes_str.c_str());

  if (!gripperJointNames_.empty()) {
    std::string gripper_joint_names_str;
    for (size_t i = 0; i < gripperJointNames_.size(); ++i) {
      if (i > 0) {gripper_joint_names_str += ", ";}
      gripper_joint_names_str += gripperJointNames_[i];
    }
    RCLCPP_INFO(this->get_logger(), "gripper_joint_name: [%s]", gripper_joint_names_str.c_str());
    RCLCPP_INFO(this->get_logger(), "gripper_open_value: %f",  gripperOpenValue_);
    RCLCPP_INFO(this->get_logger(), "gripper_close_value: %f", gripperCloseValue_);
    if (gripperAddress_.size() >= 2) {
      RCLCPP_INFO(
        this->get_logger(), "gripper_address: [%ld, %ld]",
        gripperAddress_[0], gripperAddress_[1]);
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "gripper_joint_name: [] (no gripper)");
  }
  RCLCPP_INFO(this->get_logger(), "encoder_joint_topic: %s", encoderJointTopic_.c_str());
  RCLCPP_INFO(this->get_logger(), "isaacsim_joint_topic: %s", isaacsimJointTopic_.c_str());
  RCLCPP_INFO(this->get_logger(), "gazebo_joint_topic: %s", gazeboJointTopic_.c_str());
  RCLCPP_INFO(this->get_logger(), "===========================");
}

void JointStateBroadcaster::publishJointState()
{
  wmx3LibCm_->GetStatus(&cmStatus_);

  sensor_msgs::msg::JointState encoderJointMsg_;
  std_msgs::msg::Float64MultiArray gazeboJointMsg_;

  for (size_t i = 0; i < jointAxes_.size(); ++i) {
    encoderJointMsg_.name.push_back(jointNames_[i]);
    encoderJointMsg_.position.push_back(cmStatus_.axesStatus[jointAxes_[i]].actualPos);
    encoderJointMsg_.velocity.push_back(cmStatus_.axesStatus[jointAxes_[i]].actualVelocity);
  }

  for (size_t i = 0; i < gripperJointNames_.size() && gripperAddress_.size() >= 1; ++i) {
    encoderJointMsg_.name.push_back(gripperJointNames_[i]);
    wmx3Lib_Io_->GetOutBit(gripperAddress_[0], gripperAddress_[1], &gripperData_);
    if (gripperData_) {
      encoderJointMsg_.position.push_back(gripperCloseValue_);
      encoderJointMsg_.velocity.push_back(0.000);
    } else {
      encoderJointMsg_.position.push_back(gripperOpenValue_);
      encoderJointMsg_.velocity.push_back(0.000);
    }
  }

  isaacsimJointPub_->publish(encoderJointMsg_);
  encoderJointMsg_.header.stamp = this->get_clock()->now();
  encoderJointPub_->publish(encoderJointMsg_);

  gazeboJointMsg_.data = encoderJointMsg_.position;
  gazeboJointPub_->publish(gazeboJointMsg_);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointStateBroadcaster>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
