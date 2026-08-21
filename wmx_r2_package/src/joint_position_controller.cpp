// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "joint_position_controller.hpp"

#include <chrono>
#include <cmath>

using std::placeholders::_1;

using wmx3Api::AxisSelection;
using wmx3Api::CoreMotion;
using wmx3Api::CoreMotionStatus;
using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;
using wmx3Api::Motion;
using wmx3Api::ProfileType;

JointPositionControllerApi::JointPositionControllerApi(const rclcpp::Logger & logger)
: logger_(logger)
{
  axisSel_.axisCount = 0;
}

JointPositionControllerApi::~JointPositionControllerApi()
{
  if (cm_) {
    releaseDevice();
  }
}

std::string JointPositionControllerApi::errorText(int err)
{
  char errString[256] = {};
  wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
  return errString;
}

int JointPositionControllerApi::attachDevice(std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (cm_) {
    message = "Already attached to the WMX3 device";
    return ErrorCode::None;
  }

  const int err = wmx3Lib_.CreateDevice(WMX3_SDK_PATH, DeviceType::DeviceTypeNormal, timeout_);
  if (err != ErrorCode::None) {
    message = "Failed to attach to device. Error=" + std::to_string(err) +
      " (" + errorText(err) + ")";
    return err;
  }

  wmx3Lib_.SetDeviceName(deviceName_);
  cm_ = std::make_unique<CoreMotion>(&wmx3Lib_);

  message = "Attached to WMX3 device";
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

void JointPositionControllerApi::releaseDevice()
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  cm_.reset();

  const int err = wmx3Lib_.CloseDevice();
  if (err != ErrorCode::None) {
    RCLCPP_ERROR(logger_, "Failed to close device. Error=%d (%s)", err, errorText(err).c_str());
  } else {
    RCLCPP_INFO(logger_, "Device closed");
  }
}

void JointPositionControllerApi::setAxes(const std::vector<int64_t> & axes)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  axisSel_.axisCount = static_cast<int>(axes.size());
  for (size_t i = 0; i < axes.size(); ++i) {
    axisSel_.axis[i] = static_cast<int>(axes[i]);
  }
}

int JointPositionControllerApi::getPosCmd(
  const std::vector<int> & axes, std::vector<double> & posCmd, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!cm_) {
    message = "Cannot read command positions. Core motion is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  CoreMotionStatus status;
  const int err = cm_->GetStatus(&status);

  posCmd.clear();
  posCmd.reserve(axes.size());
  for (const int axis : axes) {
    posCmd.push_back(status.axesStatus[axis].posCmd);
  }

  return err;
}

int JointPositionControllerApi::startLinearIntplPos(
  const std::vector<int> & axes,
  const std::vector<double> & targets,
  const std::vector<double> & maxVelocity,
  const std::vector<double> & maxAcc,
  std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!cm_) {
    message = "Cannot move axes. Core motion is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  Motion::LinearIntplCommand intpl;
  intpl.axisCount = static_cast<int>(axes.size());

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

  const int err = cm_->motion->StartLinearIntplPos(&intpl);
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

  if (!cm_) {
    message = "Cannot stop axes. Core motion is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  cm_->motion->Stop(&axisSel_);

  const int err = cm_->motion->Wait(&axisSel_);
  if (err != ErrorCode::None) {
    message = "Failed to stop axes. Error=" + std::to_string(err) +
      " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Axes stopped";
  return ErrorCode::None;
}

JointPositionController::JointPositionController()
: LifecycleNode("joint_position_controller")
{
  api_ = std::make_unique<JointPositionControllerApi>(this->get_logger());

  setRosParameter();

  execActiveSub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/moveit2_trajectory/execution_active", rclcpp::QoS(1).transient_local(),
    std::bind(&JointPositionController::onExecActive, this, _1));

  RCLCPP_INFO(
    this->get_logger(), "joint_position_controller is unconfigured, waiting for configure...");
}

JointPositionController::~JointPositionController()
{
  api_.reset();
  RCLCPP_INFO(this->get_logger(), "joint_position_controller stopped");
}

bool JointPositionController::isNodeActive()
{
  return this->get_current_state().id() ==
         lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
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

  api_->setAxes(jointAxes_);

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

JointPositionController::CallbackReturn JointPositionController::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring joint_position_controller...");

  std::string message;
  if (api_->attachDevice(message) != ErrorCode::None) {
    RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(this->get_logger(), "joint_position_controller is configured");
  return CallbackReturn::SUCCESS;
}

JointPositionController::CallbackReturn JointPositionController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  jointTrajectorySub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    jointTrajectoryTopic_, 1,
    std::bind(&JointPositionController::jointTrajectoryCallback, this, _1));

  LifecycleNode::on_activate(previous_state);
  RCLCPP_INFO(this->get_logger(), "joint_position_controller is active");
  return CallbackReturn::SUCCESS;
}

JointPositionController::CallbackReturn JointPositionController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  jointTrajectorySub_.reset();

  std::string message;
  api_->stopAxes(message);

  LifecycleNode::on_deactivate(previous_state);
  RCLCPP_INFO(this->get_logger(), "joint_position_controller is inactive");
  return CallbackReturn::SUCCESS;
}

JointPositionController::CallbackReturn JointPositionController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  jointTrajectorySub_.reset();
  if (api_->isDeviceOpen()) {
    api_->releaseDevice();
  }

  RCLCPP_INFO(this->get_logger(), "joint_position_controller is cleaned up");
  return CallbackReturn::SUCCESS;
}

JointPositionController::CallbackReturn JointPositionController::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

bool JointPositionController::buildCommand(
  const trajectory_msgs::msg::JointTrajectory & traj, IntplCommand & command)
{
  const auto & pt = traj.points.back();
  const size_t count = pt.positions.size();

  if (count != jointAxes_.size()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Dropped trajectory: %zu positions for %zu axes", count, jointAxes_.size());
    return false;
  }

  command.axes.clear();
  command.axes.reserve(count);

  for (size_t i = 0; i < count; ++i) {
    if (traj.joint_names.empty()) {
      command.axes.push_back(static_cast<int>(jointAxes_[i]));
      continue;
    }

    const auto it = axisByName_.find(traj.joint_names[i]);
    if (it == axisByName_.end()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Dropped trajectory: joint '%s' is not in joint_name", traj.joint_names[i].c_str());
      return false;
    }
    command.axes.push_back(it->second);
  }

  std::vector<double> posCmd;
  std::string message;
  api_->getPosCmd(command.axes, posCmd, message);

  const double dt = rclcpp::Duration(pt.time_from_start).seconds();

  command.targets.clear();
  command.maxVelocity.clear();
  command.maxAcc.clear();
  double largestStep = 0.0;

  for (size_t i = 0; i < count; ++i) {
    const double step = std::fabs(pt.positions[i] - posCmd[i]);
    largestStep = std::fmax(largestStep, step);

    const double velocity = (dt > 0.0) ? step / dt : defaultVelocity_;
    const double accel = velocity / (accelRatio_ * ((dt > 0.0) ? dt : 1.0));

    command.targets.push_back(pt.positions[i]);
    command.maxVelocity.push_back(velocity);
    command.maxAcc.push_back(accel);
  }

  return largestStep >= minStep_;
}

void JointPositionController::jointTrajectoryCallback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  IntplCommand command;

  if (!isNodeActive() || msg->points.empty() || in_execution_.load() ||
    !buildCommand(*msg, command))
  {
    return;
  }

  std::string message;
  const int err = api_->startLinearIntplPos(
    command.axes, command.targets, command.maxVelocity, command.maxAcc, message);

  if (err != ErrorCode::None) {
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
