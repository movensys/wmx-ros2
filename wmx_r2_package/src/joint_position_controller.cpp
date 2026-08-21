// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "joint_position_controller.hpp"

#include <thread>

#include <chrono>

#include <cmath>

using std::placeholders::_1;

using wmx3Api::CoreMotion;
using wmx3Api::CoreMotionStatus;
using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;
using wmx3Api::Motion;
using wmx3Api::ProfileType;

namespace
{
std::string errorText(int err)
{
  char errString[256] = {};
  CoreMotion::ErrorToString(err, errString, sizeof(errString));
  return errString;
}
}  // namespace

JointPositionControllerApi::JointPositionControllerApi(const rclcpp::Logger & logger)
: logger_(logger)
{
}

JointPositionControllerApi::~JointPositionControllerApi()
{
  releaseDevice();
}

int JointPositionControllerApi::attachDevice(std::string & message)
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

  cm_ = CoreMotion(&wmx3Lib_);
  isDeviceAttached_ = true;

  message = "Attached to WMX3 device";
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

void JointPositionControllerApi::releaseDevice()
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  const int err = wmx3Lib_.CloseDevice();
  if (err != ErrorCode::None) {
    RCLCPP_ERROR(logger_, "Failed to close device. Error=%d (%s)", err, errorText(err).c_str());
    return;
  }

  RCLCPP_INFO(logger_, "Device closed");
  isDeviceAttached_ = false;
}

int JointPositionControllerApi::setAxes(
  const std::vector<int64_t> & axes, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (axes.empty()) {
    message = "No axes configured: 'joint_axes' is empty.";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return ErrorCode::ArgumentOutOfRange;
  }

  if (axes.size() > static_cast<size_t>(wmx3Api::constants::maxAxes)) {
    message = "Too many axes: " + std::to_string(axes.size()) + " configured, the SDK allows " +
      std::to_string(wmx3Api::constants::maxAxes) + ".";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return ErrorCode::ArgumentOutOfRange;
  }

  for (const int64_t axis : axes) {
    if (axis < 0 || axis >= wmx3Api::constants::maxAxes) {
      message = "Invalid axis " + std::to_string(axis) + ": must be in [0, " +
        std::to_string(wmx3Api::constants::maxAxes) + ").";
      RCLCPP_ERROR(logger_, "%s", message.c_str());
      return ErrorCode::ArgumentOutOfRange;
    }
  }

  axisSel_.axisCount = static_cast<int>(axes.size());
  for (size_t i = 0; i < axes.size(); ++i) {
    axisSel_.axis[i] = static_cast<int>(axes[i]);
  }

  message = "Driving " + std::to_string(axes.size()) + " axes";
  return ErrorCode::None;
}

int JointPositionControllerApi::getPosCmd(
  const std::vector<int> & axes, std::vector<double> & posCmd, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!isDeviceAttached_) {
    message = "Cannot read the axis status. Device is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  CoreMotionStatus status;
  const int err = cm_.GetStatus(&status);
  if (err != ErrorCode::None) {
    message = "GetStatus failed. Error=" + std::to_string(err) + " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  posCmd.clear();
  posCmd.reserve(axes.size());
  for (const int axis : axes) {
    if (axis < 0 || axis >= wmx3Api::constants::maxAxes) {
      message = "Invalid axis " + std::to_string(axis) + ": must be in [0, " +
        std::to_string(wmx3Api::constants::maxAxes) + ").";
      return ErrorCode::ArgumentOutOfRange;
    }
    posCmd.push_back(status.axesStatus[axis].posCmd);
  }

  return ErrorCode::None;
}

int JointPositionControllerApi::startLinearIntplPos(
  const std::vector<int> & axes,
  const std::vector<double> & targets,
  const std::vector<double> & maxVelocity,
  const std::vector<double> & maxAcc,
  std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!isDeviceAttached_) {
    message = "Cannot start the interpolation. Device is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  if (axes.empty() || axes.size() > static_cast<size_t>(wmx3Api::constants::maxAxes)) {
    message = "Invalid axis count " + std::to_string(axes.size()) + ": must be in [1, " +
      std::to_string(wmx3Api::constants::maxAxes) + "].";
    return ErrorCode::ArgumentOutOfRange;
  }

  if (targets.size() != axes.size() || maxVelocity.size() != axes.size() ||
    maxAcc.size() != axes.size())
  {
    message = "Interpolation argument sizes do not match the axis count.";
    return ErrorCode::ArgumentOutOfRange;
  }

  Motion::LinearIntplCommand intpl;
  intpl.axisCount = static_cast<unsigned int>(axes.size());
  for (size_t i = 0; i < axes.size(); ++i) {
    intpl.axis[i] = axes[i];
    intpl.target[i] = targets[i];
    intpl.maxVelocity[i] = maxVelocity[i];
    intpl.maxAcc[i] = maxAcc[i];
    intpl.maxDec[i] = maxAcc[i];
  }

  intpl.profile.type = ProfileType::Trapezoidal;
  intpl.profile.velocity = 0.0;
  intpl.profile.acc = 0.0;
  intpl.profile.dec = 0.0;

  const int err = cm_.motion->StartLinearIntplPos(&intpl);
  if (err != ErrorCode::None) {
    message = "StartLinearIntplPos failed. Error=" + std::to_string(err) +
      " (" + errorText(err) + ")";
    return err;
  }

  message = "Interpolating " + std::to_string(axes.size()) + " axes";
  return ErrorCode::None;
}

int JointPositionControllerApi::stopAxes(std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!isDeviceAttached_) {
    message = "Cannot stop the axes. Device is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  int err = cm_.motion->Stop(&axisSel_);
  if (err != ErrorCode::None) {
    message = "Stop failed. Error=" + std::to_string(err) + " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  err = cm_.motion->Wait(&axisSel_);
  if (err != ErrorCode::None) {
    message = "Wait failed. Error=" + std::to_string(err) + " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Axes stopped";
  return ErrorCode::None;
}

JointPositionController::JointPositionController()
: LifecycleNode("joint_position_controller")
{
  setRosParameter();

  api_ = std::make_unique<JointPositionControllerApi>(this->get_logger());

  RCLCPP_INFO(
    this->get_logger(), "joint_position_controller is unconfigured, waiting for configure...");
}

JointPositionController::~JointPositionController()
{
  api_.reset();
  RCLCPP_INFO(this->get_logger(), "joint_position_controller stopped");
}

bool JointPositionController::isNodeActive() const
{
  return isNodeActive_.load();
}

std::string JointPositionController::notActiveMessage()
{
  return "joint_position_controller is not active (state: " +
         this->get_current_state().label() + ").";
}

void JointPositionController::setRosParameter()
{
  jointAxes_ = this->declare_parameter<std::vector<int64_t>>("joint_axes", std::vector<int64_t>{});
  jointNames_ = this->declare_parameter<std::vector<std::string>>(
    "joint_name", {"j1", "j2", "j3", "j4", "j5", "j6"});
  jointTrajectoryTopic_ = this->declare_parameter<std::string>(
    "joint_trajectory_topic", "/joint_trajectory_topic/no_param");
  accelRatio_ = this->declare_parameter<double>("accel_ratio", 0.5);
  defaultVelocity_ = this->declare_parameter<double>("default_velocity", 0.1);
  minStep_ = this->declare_parameter<double>("min_step", 0.1);

  for (size_t i = 0; i < jointNames_.size() && i < jointAxes_.size(); ++i) {
    axisByName_[jointNames_[i]] = static_cast<int>(jointAxes_[i]);
  }

  std::string jointAxesText;
  for (size_t i = 0; i < jointAxes_.size(); ++i) {
    if (i > 0) {jointAxesText += ", ";}
    jointAxesText += std::to_string(jointAxes_[i]);
  }

  if (!std::isfinite(accelRatio_) || accelRatio_ <= 0.0) {
    RCLCPP_WARN(
      this->get_logger(),
      "accel_ratio must be finite and > 0, got %f. Falling back to 0.5.", accelRatio_);
    accelRatio_ = 0.5;
  }

  RCLCPP_INFO(this->get_logger(), "===== ROS2 Parameters =====");
  RCLCPP_INFO(this->get_logger(), "joint_axes: [%s]", jointAxesText.c_str());
  RCLCPP_INFO(this->get_logger(), "joint_trajectory_topic: %s", jointTrajectoryTopic_.c_str());
  RCLCPP_INFO(this->get_logger(), "accel_ratio: %f", accelRatio_);
  RCLCPP_INFO(this->get_logger(), "default_velocity: %f", defaultVelocity_);
  RCLCPP_INFO(this->get_logger(), "min_step: %f", minStep_);
  RCLCPP_INFO(this->get_logger(), "===========================");
}

JointPositionController::CallbackReturn JointPositionController::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring joint_position_controller...");

  std::string message;
  if (api_->attachDevice(message) != ErrorCode::None) {
    return CallbackReturn::FAILURE;
  }

  if (api_->setAxes(jointAxes_, message) != ErrorCode::None) {
    api_->releaseDevice();
    return CallbackReturn::FAILURE;
  }

  execActiveSub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/moveit2_trajectory/execution_active", rclcpp::QoS(1).transient_local(),
    std::bind(&JointPositionController::execActiveCallback, this, _1));

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
  isNodeActive_ = true;

  RCLCPP_INFO(this->get_logger(), "joint_position_controller is active");
  return CallbackReturn::SUCCESS;
}

JointPositionController::CallbackReturn JointPositionController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  isNodeActive_ = false;

  std::string message;
  api_->stopAxes(message);

  LifecycleNode::on_deactivate(previous_state);
  RCLCPP_INFO(this->get_logger(), "joint_position_controller is inactive");
  return CallbackReturn::SUCCESS;
}

JointPositionController::CallbackReturn JointPositionController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  isNodeActive_ = false;

  jointTrajectorySub_.reset();
  execActiveSub_.reset();

  api_->releaseDevice();

  RCLCPP_INFO(this->get_logger(), "joint_position_controller is cleaned up");
  return CallbackReturn::SUCCESS;
}

JointPositionController::CallbackReturn JointPositionController::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

void JointPositionController::execActiveCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (inExecution_.exchange(msg->data) == msg->data) {
    return;
  }

  RCLCPP_INFO(
    this->get_logger(), "Servo commands %s (move_group execution %s)",
    msg->data ? "blocked" : "allowed", msg->data ? "started" : "finished");
}

bool JointPositionController::buildCommand(
  const trajectory_msgs::msg::JointTrajectory & traj,
  std::vector<int> & axes,
  std::vector<double> & targets,
  std::vector<double> & maxVelocity,
  std::vector<double> & maxAcc)
{
  const auto & pt = traj.points.back();
  const size_t count = pt.positions.size();

  if (count != jointAxes_.size()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Dropped trajectory: %zu positions for %zu axes", count, jointAxes_.size());
    return false;
  }

  if (!traj.joint_names.empty() && traj.joint_names.size() < count) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Dropped trajectory: %zu joint names for %zu positions",
      traj.joint_names.size(), count);
    return false;
  }

  axes.clear();
  axes.reserve(count);
  for (size_t i = 0; i < count; ++i) {
    if (traj.joint_names.empty()) {
      axes.push_back(static_cast<int>(jointAxes_[i]));
      continue;
    }

    const auto it = axisByName_.find(traj.joint_names[i]);
    if (it == axisByName_.end()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Dropped trajectory: joint '%s' is not in joint_name", traj.joint_names[i].c_str());
      return false;
    }
    axes.push_back(it->second);
  }

  std::string message;
  std::vector<double> posCmd;
  if (api_->getPosCmd(axes, posCmd, message) != ErrorCode::None) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Dropped trajectory: %s", message.c_str());
    return false;
  }

  const double dt = rclcpp::Duration(pt.time_from_start).seconds();

  targets.assign(count, 0.0);
  maxVelocity.assign(count, 0.0);
  maxAcc.assign(count, 0.0);

  double largestStep = 0.0;
  for (size_t i = 0; i < count; ++i) {
    const double step = std::fabs(pt.positions[i] - posCmd[i]);
    largestStep = std::fmax(largestStep, step);

    const double velocity = (dt > 0.0) ? step / dt : defaultVelocity_;

    targets[i] = pt.positions[i];
    maxVelocity[i] = velocity;
    maxAcc[i] = velocity / (accelRatio_ * ((dt > 0.0) ? dt : 1.0));
  }

  return largestStep >= minStep_;
}

void JointPositionController::jointTrajectoryCallback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  if (!isNodeActive() || msg->points.empty() || inExecution_.load()) {
    return;
  }

  std::vector<int> axes;
  std::vector<double> targets;
  std::vector<double> maxVelocity;
  std::vector<double> maxAcc;

  if (!buildCommand(*msg, axes, targets, maxVelocity, maxAcc)) {
    return;
  }

  std::string message;
  if (api_->startLinearIntplPos(axes, targets, maxVelocity, maxAcc, message) != ErrorCode::None) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "%s", message.c_str());
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
