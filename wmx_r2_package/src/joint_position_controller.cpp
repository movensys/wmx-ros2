// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include <atomic>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/bool.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "WMX3Api.h"
#include "CoreMotionApi.h"

using std::placeholders::_1;
using wmx3Api::AxisSelection;
using wmx3Api::CoreMotion;
using wmx3Api::CoreMotionStatus;
using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;
using wmx3Api::Motion;
using wmx3Api::ProfileType;
using wmx3Api::WMX3Api;

class JointPositionController : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  JointPositionController();
  ~JointPositionController() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

private:
  std::atomic<bool> isActive_{false};
  bool isDeviceAttached_ = false;
  std::atomic<bool> in_execution_{false};

  WMX3Api wmx3Lib_;
  std::unique_ptr<CoreMotion> wmx3LibCm_;
  CoreMotionStatus cmStatus_;
  Motion::LinearIntplCommand intpl_;
  AxisSelection axisSel_;

  int err_ = 0;
  char errString_[256];

  std::vector<int64_t> jointAxes_;
  std::vector<std::string> jointNames_;
  std::string jointTrajectoryTopic_;
  double accelRatio_ = 0.0;
  double defaultVelocity_ = 0.0;
  double minStep_ = 0.0;

  std::map<std::string, int> axisByName_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr execActiveSub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr jointTrajectorySub_;

  void setRosParameter();
  bool attachDevice();
  void releaseDevice();
  void onExecActive(const std_msgs::msg::Bool::SharedPtr msg);
  void jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  bool buildCommand(const trajectory_msgs::msg::JointTrajectory & traj);
};

JointPositionController::JointPositionController()
: LifecycleNode("joint_position_controller")
{
  RCLCPP_INFO(this->get_logger(), "start joint_position_controller");

  setRosParameter();

  RCLCPP_INFO(
    this->get_logger(), "joint_position_controller is unconfigured, waiting for configure...");
}

JointPositionController::~JointPositionController()
{
  RCLCPP_INFO(this->get_logger(), "Stop joint_position_controller");

  releaseDevice();

  RCLCPP_INFO(this->get_logger(), "joint_position_controller is stopped");
}

bool JointPositionController::attachDevice()
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

  wmx3Lib_.SetDeviceName("joint_position_controller");
  isDeviceAttached_ = true;
  RCLCPP_INFO(this->get_logger(), "Attached to WMX3 device");
  return true;
}

void JointPositionController::releaseDevice()
{
  if (!isDeviceAttached_) {
    return;
  }

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

JointPositionController::CallbackReturn JointPositionController::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring joint_position_controller...");

  if (!attachDevice()) {
    return CallbackReturn::FAILURE;
  }

  wmx3LibCm_ = std::make_unique<CoreMotion>(&wmx3Lib_);

  execActiveSub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/moveit2_trajectory/execution_active", rclcpp::QoS(1).transient_local(),
    std::bind(&JointPositionController::onExecActive, this, _1));

  jointTrajectorySub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    jointTrajectoryTopic_, 1,
    std::bind(&JointPositionController::jointTrajectoryCallback, this, _1));

  RCLCPP_INFO(this->get_logger(), "joint_position_controller is configured");
  return CallbackReturn::SUCCESS;
}

JointPositionController::CallbackReturn JointPositionController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  LifecycleNode::on_activate(previous_state);
  isActive_ = true;
  RCLCPP_INFO(this->get_logger(), "joint_position_controller is active");
  return CallbackReturn::SUCCESS;
}

JointPositionController::CallbackReturn JointPositionController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  isActive_ = false;

  if (wmx3LibCm_) {
    wmx3LibCm_->motion->Stop(&axisSel_);
    wmx3LibCm_->motion->Wait(&axisSel_);
  }

  LifecycleNode::on_deactivate(previous_state);
  RCLCPP_INFO(this->get_logger(), "joint_position_controller is inactive");
  return CallbackReturn::SUCCESS;
}

JointPositionController::CallbackReturn JointPositionController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  isActive_ = false;

  jointTrajectorySub_.reset();
  execActiveSub_.reset();

  releaseDevice();

  RCLCPP_INFO(this->get_logger(), "joint_position_controller is cleaned up");
  return CallbackReturn::SUCCESS;
}

JointPositionController::CallbackReturn JointPositionController::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

void JointPositionController::setRosParameter()
{
  this->declare_parameter<std::vector<int64_t>>("joint_axes", std::vector<int64_t>{});
  this->declare_parameter<std::vector<std::string>>(
    "joint_name", {"j1", "j2", "j3", "j4", "j5", "j6"});
  this->declare_parameter<std::string>(
    "joint_trajectory_topic", "/joint_trajectory_topic/no_param");
  this->declare_parameter<double>("accel_ratio", 0.5);
  this->declare_parameter<double>("default_velocity", 0.1);
  this->declare_parameter<double>("min_step", 0.1);

  this->get_parameter("joint_axes", jointAxes_);
  this->get_parameter("joint_name", jointNames_);
  this->get_parameter("joint_trajectory_topic", jointTrajectoryTopic_);
  this->get_parameter("accel_ratio", accelRatio_);
  this->get_parameter("default_velocity", defaultVelocity_);
  this->get_parameter("min_step", minStep_);

  for (size_t i = 0; i < jointNames_.size() && i < jointAxes_.size(); ++i) {
    axisByName_[jointNames_[i]] = static_cast<int>(jointAxes_[i]);
  }

  axisSel_.axisCount = jointAxes_.size();
  for (size_t i = 0; i < jointAxes_.size(); ++i) {
    axisSel_.axis[i] = static_cast<int>(jointAxes_[i]);
  }

  std::string joint_axes_str;
  for (size_t i = 0; i < jointAxes_.size(); ++i) {
    if (i > 0) {joint_axes_str += ", ";}
    joint_axes_str += std::to_string(jointAxes_[i]);
  }

  RCLCPP_INFO(this->get_logger(), "===== ROS2 Parameters =====");
  RCLCPP_INFO(this->get_logger(), "joint_axes: [%s]", joint_axes_str.c_str());
  RCLCPP_INFO(this->get_logger(), "joint_trajectory_topic: %s", jointTrajectoryTopic_.c_str());
  RCLCPP_INFO(this->get_logger(), "accel_ratio: %f", accelRatio_);
  RCLCPP_INFO(this->get_logger(), "default_velocity: %f", defaultVelocity_);
  RCLCPP_INFO(this->get_logger(), "min_step: %f", minStep_);
  RCLCPP_INFO(this->get_logger(), "===========================");
}

void JointPositionController::onExecActive(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (in_execution_.exchange(msg->data) == msg->data) {
    return;
  }
  RCLCPP_INFO(
    this->get_logger(), "Servo commands %s (move_group execution %s)",
    msg->data ? "blocked" : "allowed", msg->data ? "started" : "finished");
}

bool JointPositionController::buildCommand(const trajectory_msgs::msg::JointTrajectory & traj)
{
  const auto & pt = traj.points.back();
  const size_t count = pt.positions.size();

  if (count != jointAxes_.size()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Dropped trajectory: %zu positions for %zu axes", count, jointAxes_.size());
    return false;
  }

  const double dt = rclcpp::Duration(pt.time_from_start).seconds();

  wmx3LibCm_->GetStatus(&cmStatus_);

  intpl_.axisCount = count;
  double largestStep = 0.0;

  for (size_t i = 0; i < count; ++i) {
    int axis;
    if (traj.joint_names.empty()) {
      axis = static_cast<int>(jointAxes_[i]);
    } else {
      const auto it = axisByName_.find(traj.joint_names[i]);
      if (it == axisByName_.end()) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Dropped trajectory: joint '%s' is not in joint_name", traj.joint_names[i].c_str());
        return false;
      }
      axis = it->second;
    }

    const double step = std::fabs(pt.positions[i] - cmStatus_.axesStatus[axis].posCmd);
    largestStep = std::fmax(largestStep, step);

    const double velocity = (dt > 0.0) ? step / dt : defaultVelocity_;
    const double accel = velocity / (accelRatio_ * ((dt > 0.0) ? dt : 1.0));

    intpl_.axis[i] = axis;
    intpl_.target[i] = pt.positions[i];
    intpl_.maxVelocity[i] = velocity;
    intpl_.maxAcc[i] = accel;
    intpl_.maxDec[i] = accel;
  }

  if (largestStep < minStep_) {
    return false;
  }

  intpl_.profile.type = ProfileType::Trapezoidal;
  intpl_.profile.velocity = 0.0;
  intpl_.profile.acc = 0.0;
  intpl_.profile.dec = 0.0;

  return true;
}

void JointPositionController::jointTrajectoryCallback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  if (!isActive_ || msg->points.empty() || in_execution_.load() || !buildCommand(*msg)) {
    return;
  }

  err_ = wmx3LibCm_->motion->StartLinearIntplPos(&intpl_);
  if (err_ != ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "StartLinearIntplPos failed. Error=%d (%s)", err_, errString_);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointPositionController>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
