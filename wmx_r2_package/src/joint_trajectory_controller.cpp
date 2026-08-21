// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "joint_trajectory_controller.hpp"

#include <chrono>
#include <functional>
#include <sstream>
#include <thread>

struct ScopeExit
{
  std::function<void()> fn;
  ~ScopeExit() {fn();}
};

using wmx3Api::AdvancedMotion;
using wmx3Api::AdvMotion;
using wmx3Api::AxisSelection;
using wmx3Api::CoreMotion;
using wmx3Api::CoreMotionStatus;
using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;

JointTrajectoryControllerApi::JointTrajectoryControllerApi(const rclcpp::Logger & logger)
: logger_(logger)
{
  axisSel_.axisCount = 0;
}

JointTrajectoryControllerApi::~JointTrajectoryControllerApi()
{
  if (cm_) {
    releaseDevice();
  }
}

std::string JointTrajectoryControllerApi::errorText(int err)
{
  char errString[256] = {};
  wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
  return errString;
}

int JointTrajectoryControllerApi::attachDevice(std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (cm_) {
    message = "Already attached to the WMX3 device";
    return ErrorCode::None;
  }

  const int err = wmx3Lib_.CreateDevice(WMX3_SDK_PATH, DeviceType::DeviceTypeNormal, timeout_);
  if (err != ErrorCode::None) {
    if (err == ErrorCode::StartProcessLockError) {
      message = "Failed to attach to device (lock busy, will retry on next signal).";
      RCLCPP_WARN(logger_, "%s", message.c_str());
    } else {
      message = "Failed to attach to device. Error=" + std::to_string(err) +
        " (" + errorText(err) + ")";
      RCLCPP_ERROR(logger_, "%s", message.c_str());
    }
    return err;
  }

  wmx3Lib_.SetDeviceName(deviceName_);

  cm_ = std::make_unique<CoreMotion>(&wmx3Lib_);
  am_ = std::make_unique<AdvancedMotion>(&wmx3Lib_);
  am_->advMotion->CreateSplineBuffer(kSplineChannel, kMaxTrajectoryPoints);

  message = "Attached to WMX3 device";
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

void JointTrajectoryControllerApi::releaseDevice()
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (am_) {
    am_->advMotion->FreeSplineBuffer(kSplineChannel);
  }
  am_.reset();
  cm_.reset();

  const int err = wmx3Lib_.CloseDevice();
  if (err != ErrorCode::None) {
    RCLCPP_ERROR(logger_, "Failed to close device. Error=%d (%s)", err, errorText(err).c_str());
  } else {
    RCLCPP_INFO(logger_, "Device closed");
  }
}

void JointTrajectoryControllerApi::setAxes(const std::vector<int64_t> & axes)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  axes_ = axes;

  axisSel_.axisCount = static_cast<int>(axes.size());
  for (size_t i = 0; i < axes.size(); ++i) {
    axisSel_.axis[i] = static_cast<int>(axes[i]);
  }
}

int JointTrajectoryControllerApi::importAndSetAll(
  const std::string & path, wmx3Api::Config::AxisParam & axisParamError, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!cm_) {
    message = "Cannot set WMX params. Core motion is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  wmx3Api::Config::SystemParam sysParamError;

  std::vector<char> pathBuffer(path.begin(), path.end());
  pathBuffer.push_back('\0');

  const int err = cm_->config->ImportAndSetAll(pathBuffer.data(), &sysParamError, &axisParamError);
  if (err != ErrorCode::None) {
    message = "Failed to set WMX params. Error=" + std::to_string(err) +
      " (" + errorText(err) + ")";
    return err;
  }

  message = "Success to set WMX params";
  return ErrorCode::None;
}

int JointTrajectoryControllerApi::getAxisParam(
  wmx3Api::Config::AxisParam & axisParam, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!cm_) {
    message = "Cannot read axis params. Core motion is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  const int err = cm_->config->GetAxisParam(&axisParam);
  if (err != ErrorCode::None) {
    message = "Failed to get axis params. Error=" + std::to_string(err) +
      " (" + errorText(err) + ")";
    return err;
  }

  message = "OK";
  return ErrorCode::None;
}

int JointTrajectoryControllerApi::startCSplinePos(
  const std::vector<std::vector<double>> & positions,
  const std::vector<double> & timesMs,
  std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!am_) {
    message = "Cannot start spline. Advanced motion is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  const int pointCount = static_cast<int>(positions.size());
  if (pointCount <= 0 || pointCount > kMaxTrajectoryPoints) {
    message = "Invalid trajectory point count: " + std::to_string(pointCount);
    return ErrorCode::ArgumentOutOfRange;
  }

  AdvMotion::PointTimeSplineCommand splineCommand;
  splineCommand.dimensionCount = static_cast<int>(axes_.size());
  for (size_t j = 0; j < axes_.size(); ++j) {
    splineCommand.axis[j] = static_cast<int>(axes_[j]);
  }

  std::vector<AdvMotion::SplinePoint> splinePoints(pointCount);
  std::vector<double> splineTimesMs(timesMs.begin(), timesMs.begin() + pointCount);

  for (int i = 0; i < pointCount; ++i) {
    for (size_t j = 0; j < axes_.size(); ++j) {
      splinePoints[i].pos[j] = positions[i][j];
    }
  }

  const int err = am_->advMotion->StartCSplinePos(
    kSplineChannel, &splineCommand, pointCount, splinePoints.data(), splineTimesMs.data());

  if (err != ErrorCode::None) {
    char errString[256] = {};
    AdvancedMotion::ErrorToString(err, errString, sizeof(errString));
    message = "StartCSplinePos Error: " + std::string(errString);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Spline started over " + std::to_string(pointCount) + " points";
  return ErrorCode::None;
}

int JointTrajectoryControllerApi::getInPos(bool & inPos, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!cm_) {
    message = "Cannot read status. Core motion is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  CoreMotionStatus status;
  const int err = cm_->GetStatus(&status);

  inPos = true;
  for (const int64_t axis : axes_) {
    if (!status.axesStatus[axis].inPos) {
      inPos = false;
      break;
    }
  }

  return err;
}

int JointTrajectoryControllerApi::stopAxes(std::string & message)
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

JointTrajectoryController::JointTrajectoryController()
: LifecycleNode("joint_trajectory_controller")
{
  api_ = std::make_unique<JointTrajectoryControllerApi>(this->get_logger());

  setRosParameter();

  RCLCPP_INFO(
    this->get_logger(), "joint_trajectory_controller is unconfigured, waiting for configure...");
}

JointTrajectoryController::~JointTrajectoryController()
{
  api_.reset();
  RCLCPP_INFO(this->get_logger(), "joint_trajectory_controller stopped");
}

bool JointTrajectoryController::isNodeActive()
{
  return this->get_current_state().id() ==
         lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

std::string JointTrajectoryController::notActiveMessage()
{
  return "joint_trajectory_controller is not active (state: " +
         this->get_current_state().label() + ").";
}

JointTrajectoryController::CallbackReturn JointTrajectoryController::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring joint_trajectory_controller...");

  std::string message;
  if (api_->attachDevice(message) != ErrorCode::None) {
    RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
    return CallbackReturn::FAILURE;
  }

  api_->setAxes(jointAxes_);

  setWmxParam(wmxParamFilePath_);
  getWmxParam();

  servoResetPub_ = this->create_publisher<control_msgs::msg::JointJog>(
    "/servo_node/delta_joint_cmds", 10);

  execActivePub_ = this->create_publisher<std_msgs::msg::Bool>(
    "/moveit2_trajectory/execution_active", rclcpp::QoS(1).transient_local());

  action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
    this,
    jointTrajectoryAction_,
    std::bind(
      &JointTrajectoryController::handle_goal, this, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&JointTrajectoryController::handle_cancel, this, std::placeholders::_1),
    std::bind(&JointTrajectoryController::handle_accepted, this, std::placeholders::_1)
  );

  RCLCPP_INFO(this->get_logger(), "joint_trajectory_controller is configured");
  return CallbackReturn::SUCCESS;
}

JointTrajectoryController::CallbackReturn JointTrajectoryController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  LifecycleNode::on_activate(previous_state);

  publishExecActive(false);

  RCLCPP_INFO(this->get_logger(), "joint_trajectory_controller is active");
  return CallbackReturn::SUCCESS;
}

JointTrajectoryController::CallbackReturn JointTrajectoryController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  publishExecActive(false);

  std::string message;
  api_->stopAxes(message);

  LifecycleNode::on_deactivate(previous_state);
  RCLCPP_INFO(this->get_logger(), "joint_trajectory_controller is inactive");
  return CallbackReturn::SUCCESS;
}

JointTrajectoryController::CallbackReturn JointTrajectoryController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  action_server_.reset();
  execActivePub_.reset();
  servoResetPub_.reset();

  if (api_->isDeviceOpen()) {
    api_->releaseDevice();
  }

  RCLCPP_INFO(this->get_logger(), "joint_trajectory_controller is cleaned up");
  return CallbackReturn::SUCCESS;
}

JointTrajectoryController::CallbackReturn JointTrajectoryController::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

void JointTrajectoryController::setWmxParam(const std::string & path)
{
  wmx3Api::Config::AxisParam axisParamError;
  std::string message;

  if (api_->importAndSetAll(path, axisParamError, message) == ErrorCode::None) {
    RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
    return;
  }

  RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
  for (const int64_t axis : jointAxes_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "  [axis %ld] AxisParam error flags: gearNum=%.0f gearDen=%.6f polarity=%d "
      "absEnc=%d maxSpd=%.0f maxSpdUnitNum=%.6f maxSpdUnitDen=%.6f singleTurnMode=%d "
      "singleTurnCnt=%u maxTrq=%.1f posTrq=%.1f negTrq=%.1f axisUnit=%.6f cmdMode=%d",
      axis,
      axisParamError.gearRatioNumerator[axis], axisParamError.gearRatioDenominator[axis],
      static_cast<int>(axisParamError.axisPolarity[axis]),
      static_cast<int>(axisParamError.absoluteEncoderMode[axis]),
      axisParamError.maxMotorSpeed[axis],
      axisParamError.maxMotorSpeedUnitNumerator[axis],
      axisParamError.maxMotorSpeedUnitDenominator[axis],
      static_cast<int>(axisParamError.singleTurnMode[axis]),
      axisParamError.singleTurnEncoderCount[axis],
      axisParamError.maxTrqLimit[axis], axisParamError.positiveTrqLimit[axis],
      axisParamError.negativeTrqLimit[axis], axisParamError.axisUnit[axis],
      static_cast<int>(axisParamError.axisCommandMode[axis]));
  }
}

void JointTrajectoryController::getWmxParam()
{
  wmx3Api::Config::AxisParam axisParam;
  std::string message;

  if (api_->getAxisParam(axisParam, message) != ErrorCode::None) {
    RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
    return;
  }

  for (const int64_t axis : jointAxes_) {
    RCLCPP_INFO(
      this->get_logger(), "axis: %ld, numerator: %f", axis,
      axisParam.gearRatioNumerator[axis]);
    RCLCPP_INFO(
      this->get_logger(), "axis: %ld, denominator: %f", axis,
      axisParam.gearRatioDenominator[axis]);
    RCLCPP_INFO(
      this->get_logger(), "axis: %ld, polarity: %d", axis,
      static_cast<int>(axisParam.axisPolarity[axis]));
    RCLCPP_INFO(
      this->get_logger(), "axis: %ld, abs encoder: %d", axis,
      axisParam.absoluteEncoderMode[axis]);
    RCLCPP_INFO(
      this->get_logger(), "axis: %ld, mode: %d", axis,
      axisParam.axisCommandMode[axis]);
  }
}

void JointTrajectoryController::setRosParameter()
{
  this->declare_parameter<std::vector<int64_t>>("joint_axes", std::vector<int64_t>{});
  this->declare_parameter<std::string>(
    "joint_trajectory_action", "/joint_trajectory_action/no_param");
  this->declare_parameter<std::string>("wmx_param_file_path", "/joint_trajectory/no_param");

  this->get_parameter("joint_axes", jointAxes_);
  this->get_parameter("joint_trajectory_action", jointTrajectoryAction_);
  this->get_parameter("wmx_param_file_path", wmxParamFilePath_);

  std::string joint_axes_str;
  for (size_t i = 0; i < jointAxes_.size(); ++i) {
    if (i > 0) {joint_axes_str += ", ";}
    joint_axes_str += std::to_string(jointAxes_[i]);
  }

  RCLCPP_INFO(this->get_logger(), "===== ROS2 Parameters =====");
  RCLCPP_INFO(this->get_logger(), "joint_axes: [%s]", joint_axes_str.c_str());
  RCLCPP_INFO(this->get_logger(), "joint_trajectory_action: %s", jointTrajectoryAction_.c_str());
  RCLCPP_INFO(this->get_logger(), "wmx_param_file_path: %s", wmxParamFilePath_.c_str());
  RCLCPP_INFO(this->get_logger(), "===========================");
}

rclcpp_action::GoalResponse JointTrajectoryController::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const FollowJointTrajectory::Goal> goal)
{
  (void)uuid;
  (void)goal;

  if (!isNodeActive()) {
    RCLCPP_WARN(this->get_logger(), "Goal rejected: %s", notActiveMessage().c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(this->get_logger(), "Received goal request");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse JointTrajectoryController::handle_cancel(
  std::shared_ptr<GoalHandleFJT> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void JointTrajectoryController::handle_accepted(std::shared_ptr<GoalHandleFJT> goal_handle)
{
  std::thread{std::bind(&JointTrajectoryController::execute, this, std::placeholders::_1),
    goal_handle}.detach();
}

void JointTrajectoryController::publishExecActive(bool active)
{
  if (!execActivePub_) {
    return;
  }

  std_msgs::msg::Bool msg;
  msg.data = active;
  execActivePub_->publish(msg);
}

void JointTrajectoryController::resetServo(const std::vector<std::string> & joint_names)
{
  if (!servoResetPub_) {
    return;
  }

  control_msgs::msg::JointJog jog;
  jog.header.stamp = this->get_clock()->now();
  jog.joint_names = joint_names;
  jog.velocities.assign(joint_names.size(), 0.0);
  servoResetPub_->publish(jog);
}

void JointTrajectoryController::execute(std::shared_ptr<GoalHandleFJT> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  const auto & trajectory = goal->trajectory;

  publishExecActive(true);
  const auto servo_joint_names = trajectory.joint_names;
  ScopeExit on_exit{[this, servo_joint_names]() {
      publishExecActive(false);
      resetServo(servo_joint_names);
    }};

  int num_points = static_cast<int>(trajectory.points.size());

  RCLCPP_INFO(this->get_logger(), "Received a new trajectory goal! Point number: [%d]", num_points);

  auto result = std::make_shared<FollowJointTrajectory::Result>();

  if (num_points > JointTrajectoryControllerApi::kMaxTrajectoryPoints) {
    RCLCPP_WARN(
      this->get_logger(),
      "Too many trajectory point size! "
      "current points:%d / max traj points:%d \nAborting current goal.",
      num_points, JointTrajectoryControllerApi::kMaxTrajectoryPoints);
    goal_handle->abort(result);
    return;
  }

  logTrajectory(trajectory);

  std::vector<std::vector<double>> positions;
  std::vector<double> timesMs;
  positions.reserve(trajectory.points.size());
  timesMs.reserve(trajectory.points.size());

  for (const auto & pt : trajectory.points) {
    timesMs.push_back(rclcpp::Duration(pt.time_from_start).seconds() * 1000);

    std::vector<double> axisTargets;
    axisTargets.reserve(jointAxes_.size());
    for (size_t j = 0; j < jointAxes_.size(); ++j) {
      axisTargets.push_back(pt.positions.at(j));
    }
    positions.push_back(std::move(axisTargets));
  }

  if (!timesMs.empty() && timesMs[0] != 0.0) {
    timesMs[0] = 0.0;
  }

  if (trajectory.points.size() >= 2) {
    const size_t last = trajectory.points.size() - 1;
    if (rclcpp::Duration(trajectory.points[last].time_from_start).seconds() -
      rclcpp::Duration(trajectory.points[last - 1].time_from_start).seconds() < 1e-3)
    {
      num_points -= 1;
    }
  }

  if (num_points == 0) {
    RCLCPP_INFO(this->get_logger(), "Point count is zero. It is already in the targeted position");
  } else {
    RCLCPP_INFO(this->get_logger(), "Command Start!!!");

    positions.resize(num_points);

    std::string message;
    const int err = api_->startCSplinePos(positions, timesMs, message);
    if (err != ErrorCode::None) {
      result->error_code = err;
      goal_handle->abort(result);
      return;
    }

    while (true) {
      if (goal_handle->is_canceling()) {
        api_->stopAxes(message);
        result->error_code = 0;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled, axes stopped");
        return;
      }

      bool all_done = false;
      api_->getInPos(all_done, message);
      if (all_done) {break;}

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  result->error_code = 0;
  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_logger(), "Trajectory execution completed successfully");
}

void JointTrajectoryController::logTrajectory(
  const trajectory_msgs::msg::JointTrajectory & trajectory)
{
  std::ostringstream jn;
  for (size_t i = 0; i < trajectory.joint_names.size(); ++i) {
    if (i) {jn << ", ";}
    jn << trajectory.joint_names[i];
  }
  RCLCPP_INFO(this->get_logger(), "Joint Names: [%s]", jn.str().c_str());

  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    const auto & pt = trajectory.points[i];
    std::ostringstream pos, vel, acc;
    for (size_t k = 0; k < pt.positions.size(); ++k) {
      if (k) {pos << ", ";}
      pos << pt.positions[k];
    }
    for (size_t k = 0; k < pt.velocities.size(); ++k) {
      if (k) {vel << ", ";}
      vel << pt.velocities[k];
    }
    for (size_t k = 0; k < pt.accelerations.size(); ++k) {
      if (k) {acc << ", ";}
      acc << pt.accelerations[k];
    }
    RCLCPP_INFO(
      this->get_logger(),
      "Point %zu: Positions: [%s], Velocities: [%s], "
      "Accelerations: [%s], TimeFromStart: %d s %u ns",
      i, pos.str().c_str(), vel.str().c_str(), acc.str().c_str(),
      pt.time_from_start.sec, pt.time_from_start.nanosec);

    if (i != 0) {
      rclcpp::Duration duration_cur(trajectory.points[i].time_from_start);
      rclcpp::Duration duration_pre(trajectory.points[i - 1].time_from_start);
      RCLCPP_INFO(
        this->get_logger(), "Time interval: %f",
        (duration_cur - duration_pre).seconds());
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointTrajectoryController>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
