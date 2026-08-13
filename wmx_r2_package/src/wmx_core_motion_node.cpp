// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_core_motion_node.hpp"

#include <cmath>
#include <sstream>

using std::placeholders::_1;
using std::placeholders::_2;

using wmx3Api::AxisCommandMode;
using wmx3Api::Config;
using wmx3Api::CoreMotion;
using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;
using wmx3Api::ProfileType;

WmxCoreMotionNodeApi::WmxCoreMotionNodeApi(const rclcpp::Logger & logger)
: logger_(logger)
{
}

WmxCoreMotionNodeApi::~WmxCoreMotionNodeApi()
{
  releaseDevice();
}

std::string WmxCoreMotionNodeApi::errorText(int err)
{
  char errString[256] = {};
  wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
  return errString;
}

std::shared_ptr<CoreMotion> WmxCoreMotionNodeApi::device()
{
  std::lock_guard<std::mutex> lock(deviceMutex_);
  return cm_;
}

int WmxCoreMotionNodeApi::attachDevice(std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (cm_) {
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
    // The device exists but is unnamed: close it so the next attempt starts clean.
    wmx3Lib_.CloseDevice();
    return err;
  }

  cm_ = std::make_shared<CoreMotion>(&wmx3Lib_);
  deviceOpen_ = true;

  message = "Attached to WMX3 device";
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

void WmxCoreMotionNodeApi::releaseDevice()
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!cm_) {
    return;
  }

  cm_.reset();
  deviceOpen_ = false;

  const int err = wmx3Lib_.CloseDevice();
  if (err != ErrorCode::None) {
    RCLCPP_ERROR(logger_, "Failed to close device. Error=%d (%s)", err, errorText(err).c_str());
  } else {
    RCLCPP_INFO(logger_, "Device closed");
  }
}

// numOfInterrupts is the number of cyclic handlers (max 2); numOfAxes is the
// number of axes on that handler, not the number of slaves.
int WmxCoreMotionNodeApi::readAxisCount()
{
  wmx3Api::EngineStatus engineStatus;
  wmx3Lib_.GetEngineStatus(&engineStatus);

  int axisCount = 0;
  for (int i = 0; i < engineStatus.numOfInterrupts; ++i) {
    axisCount += engineStatus.interrupts[i].numOfAxes;
  }
  return axisCount;
}

int WmxCoreMotionNodeApi::getStatus(wmx3Api::CoreMotionStatus & status)
{
  const std::shared_ptr<CoreMotion> cm = device();
  if (!cm) {
    return ErrorCode::DeviceIsNull;
  }
  return cm->GetStatus(&status);
}

int WmxCoreMotionNodeApi::startPos(
  int axis, double target, double velocity, double acc, double dec, std::string & message)
{
  const std::shared_ptr<CoreMotion> cm = device();
  if (!cm) {
    message = "Cannot move axis " + std::to_string(axis) + ". Core motion is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  wmx3Api::Motion::PosCommand position;
  position.axis = axis;
  position.target = target;
  position.profile.velocity = velocity;
  position.profile.type = ProfileType::T::Trapezoidal;
  position.profile.acc = acc;
  position.profile.dec = dec;

  const int err = cm->motion->StartPos(&position);
  if (err != ErrorCode::None) {
    message = "Failed to move position motor " + std::to_string(axis) + ". Error=" +
      std::to_string(err) + " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Moving axis " + std::to_string(axis) + " to " + std::to_string(target);
  return ErrorCode::None;
}

int WmxCoreMotionNodeApi::startMov(
  int axis, double target, double velocity, double acc, double dec, std::string & message)
{
  const std::shared_ptr<CoreMotion> cm = device();
  if (!cm) {
    message = "Cannot move axis " + std::to_string(axis) + ". Core motion is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  wmx3Api::Motion::PosCommand position;
  position.axis = axis;
  position.target = target;
  position.profile.velocity = velocity;
  position.profile.type = ProfileType::T::Trapezoidal;
  position.profile.acc = acc;
  position.profile.dec = dec;

  const int err = cm->motion->StartMov(&position);
  if (err != ErrorCode::None) {
    message = "Failed to move relative motor " + std::to_string(axis) + ". Error=" +
      std::to_string(err) + " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Moving axis " + std::to_string(axis) + " by " + std::to_string(target);
  return ErrorCode::None;
}

int WmxCoreMotionNodeApi::startVel(
  int axis, double velocity, double acc, double dec, std::string & message)
{
  const std::shared_ptr<CoreMotion> cm = device();
  if (!cm) {
    message = "Cannot move axis " + std::to_string(axis) + ". Core motion is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  wmx3Api::Velocity::VelCommand command;
  command.axis = axis;
  command.profile.velocity = velocity;
  command.profile.type = ProfileType::T::Trapezoidal;
  command.profile.acc = acc;
  command.profile.dec = dec;

  const int err = cm->velocity->StartVel(&command);
  if (err != ErrorCode::None) {
    message = "Failed to move velocity motor " + std::to_string(axis) + ". Error=" +
      std::to_string(err) + " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Axis " + std::to_string(axis) + " running at " + std::to_string(velocity);
  return ErrorCode::None;
}

// Same profile WOS uses for its jog buttons. A run time requires a time-based
// profile: Trapezoidal is rejected with ProfileTypeNotSupported.
int WmxCoreMotionNodeApi::startJog(
  int axis, double velocity, double accTimeMs, double decTimeMs, double jerkRatio,
  double runTimeMs, std::string & message)
{
  const std::shared_ptr<CoreMotion> cm = device();
  if (!cm) {
    message = "Cannot jog axis " + std::to_string(axis) + ". Core motion is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  wmx3Api::Motion::JogCommand jogCommand = wmx3Api::Motion::JogCommand();
  jogCommand.axis = axis;
  jogCommand.profile.type = ProfileType::T::TimeAccJerkRatio;
  jogCommand.profile.velocity = velocity;
  jogCommand.profile.acc = 0;
  jogCommand.profile.dec = 0;
  jogCommand.profile.jerkAcc = 0;
  jogCommand.profile.jerkDec = 0;
  jogCommand.profile.jerkAccRatio = jerkRatio;
  jogCommand.profile.jerkDecRatio = jerkRatio;
  jogCommand.profile.accTimeMilliseconds = accTimeMs;
  jogCommand.profile.decTimeMilliseconds = decTimeMs;
  jogCommand.profile.startingVelocity = 0;
  jogCommand.profile.endVelocity = 0;
  jogCommand.profile.secondVelocity = 0;
  jogCommand.profile.movingAverageTimeMilliseconds = 0;
  // Backstop: if this node dies mid-jog the engine still decelerates the axis.
  // Once it elapses the axis stays stopped until the operator releases and
  // presses again, because a held key only refreshes the dead-man deadline.
  jogCommand.SetRunTime(runTimeMs);

  const int err = cm->motion->StartJog(&jogCommand);
  if (err != ErrorCode::None) {
    message = "Failed to jog motor " + std::to_string(axis) + ". Error=" +
      std::to_string(err) + " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Jogging axis " + std::to_string(axis);
  return ErrorCode::None;
}

int WmxCoreMotionNodeApi::stopAxis(int axis)
{
  const std::shared_ptr<CoreMotion> cm = device();
  if (!cm) {
    return ErrorCode::DeviceIsNull;
  }

  const int err = cm->motion->Stop(axis);
  if (err != ErrorCode::None) {
    RCLCPP_DEBUG(
      logger_, "Stop on axis %d returned %d (%s)", axis, err, errorText(err).c_str());
  }
  return err;
}

int WmxCoreMotionNodeApi::setServoOn(int axis, int on, std::string & message)
{
  const std::shared_ptr<CoreMotion> cm = device();
  if (!cm) {
    message = "Cannot set axis " + std::to_string(axis) + ". Core motion is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  const int err = cm->axisControl->SetServoOn(axis, on, servoOnTimeout_);
  const std::string onOff = on ? "on" : "off";

  if (err != ErrorCode::None) {
    message = "Failed to set axis " + std::to_string(axis) + " " + onOff + ". Error=" +
      std::to_string(err) + " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Set axis " + std::to_string(axis) + " " + onOff;
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxCoreMotionNodeApi::setAxisCommandMode(int axis, int mode, std::string & message)
{
  const std::shared_ptr<CoreMotion> cm = device();
  if (!cm) {
    message = "Cannot set axis " + std::to_string(axis) + ". Core motion is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  if (mode != 0 && mode != 1) {
    message = "Invalid mode " + std::to_string(mode) + " for axis " + std::to_string(axis);
    RCLCPP_WARN(logger_, "%s", message.c_str());
    return ErrorCode::ArgumentOutOfRange;
  }

  const AxisCommandMode::T commandMode =
    (mode == 0) ? AxisCommandMode::Position : AxisCommandMode::Velocity;

  const int err = cm->axisControl->SetAxisCommandMode(axis, commandMode);
  if (err != ErrorCode::None) {
    message = "Failed to set axis " + std::to_string(axis) + " mode " + std::to_string(mode) +
      ". Error=" + std::to_string(err) + " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Set axis " + std::to_string(axis) + " in " +
    ((mode == 0) ? "Position" : "Velocity") + " mode";
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxCoreMotionNodeApi::clearAmpAlarm(int axis, std::string & message)
{
  const std::shared_ptr<CoreMotion> cm = device();
  if (!cm) {
    message = "Cannot clear alarm on axis " + std::to_string(axis) +
      ". Core motion is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  const int err = cm->axisControl->ClearAmpAlarm(axis);
  if (err != ErrorCode::None) {
    message = "Failed to clear alarm axis " + std::to_string(axis) + ". Error=" +
      std::to_string(err) + " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Cleared alarm axis " + std::to_string(axis);
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxCoreMotionNodeApi::setAxisPolarity(int axis, int polarity, std::string & message)
{
  const std::shared_ptr<CoreMotion> cm = device();
  if (!cm) {
    message = "Cannot set polarity on axis " + std::to_string(axis) +
      ". Core motion is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  if (polarity != 1 && polarity != -1) {
    message = "Invalid polarity value for axis " + std::to_string(axis) + ": " +
      std::to_string(polarity);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return ErrorCode::ArgumentOutOfRange;
  }

  const int err = cm->config->SetAxisPolarity(axis, polarity);
  if (err != ErrorCode::None) {
    message = "Failed to set polarity on axis " + std::to_string(axis) + ". Error=" +
      std::to_string(err) + " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Set polarity on axis " + std::to_string(axis);
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxCoreMotionNodeApi::setGearRatio(
  int axis, int numerator, int denominator, std::string & message)
{
  const std::shared_ptr<CoreMotion> cm = device();
  if (!cm) {
    message = "Cannot set gear ratio on axis " + std::to_string(axis) +
      ". Core motion is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  const int err = cm->config->SetGearRatio(axis, numerator, denominator);
  if (err != ErrorCode::None) {
    message = "Failed to set gear ratio on axis " + std::to_string(axis) + ". Error=" +
      std::to_string(err) + " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Set gear ratio on axis " + std::to_string(axis);
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

// Blocks in motion->Wait() until the axis finishes homing. Runs without
// deviceMutex_ held, so the state timer and the jog watchdog keep running.
int WmxCoreMotionNodeApi::homeAxis(int axis, std::string & message)
{
  const std::shared_ptr<CoreMotion> cm = device();
  if (!cm) {
    message = "Cannot home axis " + std::to_string(axis) + ". Core motion is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  Config::HomeParam homeParam;

  int err = cm->config->GetHomeParam(axis, &homeParam);
  if (err != ErrorCode::None) {
    message = "Failed to read home param for axis " + std::to_string(axis) + ". Error=" +
      std::to_string(err) + " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  homeParam.homeType = Config::HomeType::CurrentPos;

  err = cm->config->SetHomeParam(axis, &homeParam);
  if (err != ErrorCode::None) {
    message = "Failed to set home param for axis " + std::to_string(axis) + ". Error=" +
      std::to_string(err) + " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  err = cm->home->StartHome(axis);
  if (err != ErrorCode::None) {
    message = "Failed to start homing axis " + std::to_string(axis) + ". Error=" +
      std::to_string(err) + " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  err = cm->motion->Wait(axis);
  if (err != ErrorCode::None) {
    message = "Failed to home axis " + std::to_string(axis) + ". Error=" +
      std::to_string(err) + " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Homed axis " + std::to_string(axis);
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

WmxCoreMotionNode::WmxCoreMotionNode()
: LifecycleNode("wmx_core_motion_node")
{
  api_ = std::make_unique<WmxCoreMotionNodeApi>(this->get_logger());

  jogTimeoutMs_ = this->declare_parameter("jog_timeout_ms", 200.0);
  jogRunTimeMs_ = this->declare_parameter("jog_run_time_ms", 2000.0);
  jogJerkRatio_ = this->declare_parameter("jog_jerk_ratio", 0.75);

  homing_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  RCLCPP_INFO(
    this->get_logger(), "wmx_core_motion_node is unconfigured, waiting for configure...");
}

WmxCoreMotionNode::~WmxCoreMotionNode()
{
  api_.reset();
  RCLCPP_INFO(this->get_logger(), "wmx_core_motion_node stopped");
}

bool WmxCoreMotionNode::isNodeActive() const
{
  return this->get_current_state().id() ==
         lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

std::string WmxCoreMotionNode::notActiveMessage() const
{
  return "wmx_core_motion_node is not active (state: " +
         this->get_current_state().label() + ").";
}

void WmxCoreMotionNode::stopAllJogs()
{
  std::vector<int> jogging;
  {
    std::lock_guard<std::mutex> lock(jogMutex_);
    for (const auto & entry : jogState_) {
      jogging.push_back(entry.first);
    }
    jogState_.clear();
  }

  for (const int axis : jogging) {
    api_->stopAxis(axis);
  }
}

WmxCoreMotionNode::CallbackReturn WmxCoreMotionNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring wmx_core_motion_node...");

  std::string message;
  if (api_->attachDevice(message) != ErrorCode::None) {
    return CallbackReturn::FAILURE;
  }

  axisCount_ = api_->readAxisCount();
  if (axisCount_ <= 0) {
    RCLCPP_WARN(this->get_logger(), "Engine reported 0 axes; axis state will be empty.");
  }

  setAxisOnService_ = this->create_service<wmx_r2_message::srv::SetAxis>(
    "wmx/axis/set_on",
    std::bind(&WmxCoreMotionNode::setAxisOnCallback, this, _1, _2));

  clearAlarmService_ = this->create_service<wmx_r2_message::srv::SetAxis>(
    "wmx/axis/clear_alarm",
    std::bind(&WmxCoreMotionNode::clearAlarmCallback, this, _1, _2));

  setAxisModeService_ = this->create_service<wmx_r2_message::srv::SetAxis>(
    "wmx/axis/set_mode",
    std::bind(&WmxCoreMotionNode::setAxisModeCallback, this, _1, _2));

  setAxisPolarityService_ = this->create_service<wmx_r2_message::srv::SetAxis>(
    "wmx/axis/set_polarity",
    std::bind(&WmxCoreMotionNode::setAxisPolarityCallback, this, _1, _2));

  setAxisGearRatioService_ = this->create_service<wmx_r2_message::srv::SetAxisGearRatio>(
    "wmx/axis/set_gear_ratio",
    std::bind(&WmxCoreMotionNode::setAxisGearRatioCallback, this, _1, _2));

  setHomingService_ = this->create_service<wmx_r2_message::srv::SetAxis>(
    "wmx/axis/homing",
    std::bind(&WmxCoreMotionNode::setHomingCallback, this, _1, _2),
    rclcpp::ServicesQoS(), homing_cb_group_);

  stopAxisService_ = this->create_service<wmx_r2_message::srv::SetAxis>(
    "wmx/axis/stop",
    std::bind(&WmxCoreMotionNode::stopAxesCallback, this, _1, _2));

  axisStatePub_ = this->create_publisher<wmx_r2_message::msg::AxisState>(
    "wmx/axis/state", 1);

  axisVelSub_ = this->create_subscription<wmx_r2_message::msg::AxisVelocity>(
    "wmx/axis/velocity", 1,
    std::bind(&WmxCoreMotionNode::axisVelCallback, this, _1));

  axisPoseSub_ = this->create_subscription<wmx_r2_message::msg::AxisPose>(
    "wmx/axis/position", 1,
    std::bind(&WmxCoreMotionNode::axisPoseCallback, this, _1));

  axisPoseRelativeSub_ = this->create_subscription<wmx_r2_message::msg::AxisPose>(
    "wmx/axis/position/relative", 1,
    std::bind(&WmxCoreMotionNode::axisPoseRelativeCallback, this, _1));

  axisJogSub_ = this->create_subscription<wmx_r2_message::msg::AxisVelocity>(
    "wmx/axis/jog", 1,
    std::bind(&WmxCoreMotionNode::axisJogCallback, this, _1));

  axisStateTimer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / rate_),
    std::bind(&WmxCoreMotionNode::axisStateStep, this));
  axisStateTimer_->cancel();

  jogWatchdogTimer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&WmxCoreMotionNode::jogWatchdogStep, this));
  jogWatchdogTimer_->cancel();

  RCLCPP_INFO(
    this->get_logger(), "wmx_core_motion_node is configured (%d axes, %d Hz)",
    axisCount_, rate_);
  return CallbackReturn::SUCCESS;
}

WmxCoreMotionNode::CallbackReturn WmxCoreMotionNode::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  LifecycleNode::on_activate(previous_state);

  axisStateTimer_->reset();
  jogWatchdogTimer_->reset();

  RCLCPP_INFO(this->get_logger(), "wmx_core_motion_node is active");
  return CallbackReturn::SUCCESS;
}

WmxCoreMotionNode::CallbackReturn WmxCoreMotionNode::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{

  axisStateTimer_->cancel();
  jogWatchdogTimer_->cancel();
  stopAllJogs();

  LifecycleNode::on_deactivate(previous_state);

  RCLCPP_INFO(this->get_logger(), "wmx_core_motion_node is inactive");
  return CallbackReturn::SUCCESS;
}

WmxCoreMotionNode::CallbackReturn WmxCoreMotionNode::on_cleanup(const rclcpp_lifecycle::State &)
{

  axisStateTimer_.reset();
  jogWatchdogTimer_.reset();
  stopAllJogs();

  axisVelSub_.reset();
  axisPoseSub_.reset();
  axisPoseRelativeSub_.reset();
  axisJogSub_.reset();

  setAxisOnService_.reset();
  clearAlarmService_.reset();
  setAxisModeService_.reset();
  setAxisPolarityService_.reset();
  setAxisGearRatioService_.reset();
  setHomingService_.reset();
  stopAxisService_.reset();

  axisStatePub_.reset();

  api_->releaseDevice();

  RCLCPP_INFO(this->get_logger(), "wmx_core_motion_node is cleaned up");
  return CallbackReturn::SUCCESS;
}

WmxCoreMotionNode::CallbackReturn WmxCoreMotionNode::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

void WmxCoreMotionNode::axisStateStep()
{
  wmx3Api::CoreMotionStatus cmStatus;
  if (api_->getStatus(cmStatus) != ErrorCode::None) {
    return;
  }

  axisStateMsg_.amp_alarm.clear();
  axisStateMsg_.servo_on.clear();
  axisStateMsg_.home_done.clear();
  axisStateMsg_.motion_complete.clear();
  axisStateMsg_.negative_ls.clear();
  axisStateMsg_.positive_ls.clear();
  axisStateMsg_.home_switch.clear();
  axisStateMsg_.pos_cmd.clear();
  axisStateMsg_.velocity_cmd.clear();
  axisStateMsg_.actual_pos.clear();
  axisStateMsg_.actual_velocity.clear();
  axisStateMsg_.actual_torque.clear();

  axisStateMsg_.header.stamp = this->now();
  axisStateMsg_.header.frame_id = "base_link";

  for (int i = 0; i < axisCount_; ++i) {
    axisStateMsg_.amp_alarm.push_back(cmStatus.axesStatus[i].ampAlarm);
    axisStateMsg_.servo_on.push_back(cmStatus.axesStatus[i].servoOn);
    axisStateMsg_.home_done.push_back(cmStatus.axesStatus[i].homeDone);
    axisStateMsg_.motion_complete.push_back(cmStatus.axesStatus[i].motionComplete);
    axisStateMsg_.negative_ls.push_back(cmStatus.axesStatus[i].negativeLS);
    axisStateMsg_.positive_ls.push_back(cmStatus.axesStatus[i].positiveLS);
    axisStateMsg_.home_switch.push_back(cmStatus.axesStatus[i].homeSwitch);
    axisStateMsg_.pos_cmd.push_back(cmStatus.axesStatus[i].posCmd);
    axisStateMsg_.velocity_cmd.push_back(cmStatus.axesStatus[i].velocityCmd);
    axisStateMsg_.actual_pos.push_back(cmStatus.axesStatus[i].actualPos);
    axisStateMsg_.actual_velocity.push_back(cmStatus.axesStatus[i].actualVelocity);
    axisStateMsg_.actual_torque.push_back(cmStatus.axesStatus[i].actualTorque);
  }
  axisStatePub_->publish(axisStateMsg_);
}

void WmxCoreMotionNode::axisPoseCallback(const wmx_r2_message::msg::AxisPose::SharedPtr msg)
{
  if (!isNodeActive()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Position command ignored: %s", notActiveMessage().c_str());
    return;
  }

  std::string message;
  for (size_t i = 0; i < msg->index.size(); i++) {
    api_->startPos(
      msg->index[i], msg->target[i], msg->velocity[i], msg->acc[i], msg->dec[i], message);
  }
}

void WmxCoreMotionNode::axisPoseRelativeCallback(
  const wmx_r2_message::msg::AxisPose::SharedPtr msg)
{
  if (!isNodeActive()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Relative position command ignored: %s", notActiveMessage().c_str());
    return;
  }

  std::string message;
  for (size_t i = 0; i < msg->index.size(); i++) {
    api_->startMov(
      msg->index[i], msg->target[i], msg->velocity[i], msg->acc[i], msg->dec[i], message);
  }
}

void WmxCoreMotionNode::axisVelCallback(const wmx_r2_message::msg::AxisVelocity::SharedPtr msg)
{
  if (!isNodeActive()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Velocity command ignored: %s", notActiveMessage().c_str());
    return;
  }

  std::string message;
  for (size_t i = 0; i < msg->index.size(); i++) {
    api_->startVel(msg->index[i], msg->velocity[i], msg->acc[i], msg->dec[i], message);
  }
}

// Jog command. The publisher (keyboard/joystick teleop, CLI, ...) must keep
// republishing while the operator holds the control; jogWatchdogStep() stops
// the axis once refreshes stop arriving. Velocity sign selects the direction.
void WmxCoreMotionNode::axisJogCallback(const wmx_r2_message::msg::AxisVelocity::SharedPtr msg)
{
  if (!isNodeActive()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Jog ignored: %s", notActiveMessage().c_str());
    return;
  }

  const size_t axis_count = msg->index.size();
  if (msg->velocity.size() != axis_count ||
    msg->acc.size() != axis_count ||
    msg->dec.size() != axis_count)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Jog ignored: index/velocity/acc/dec must be the same length.");
    return;
  }

  const rclcpp::Time deadline =
    this->now() + rclcpp::Duration::from_nanoseconds(
    static_cast<int64_t>(jogTimeoutMs_ * 1e6));

  for (size_t i = 0; i < axis_count; ++i) {
    const int axis = msg->index[i];
    const double velocity = msg->velocity[i];

    if (velocity == 0.0) {
      std::lock_guard<std::mutex> lock(jogMutex_);
      if (jogState_.erase(axis) > 0) {
        api_->stopAxis(axis);
      }
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(jogMutex_);
      auto it = jogState_.find(axis);
      if (it != jogState_.end() && it->second.velocity == velocity) {
        // Same direction and speed: only refresh the dead-man deadline.
        // Re-issuing StartJog here would override a jog with a jog, which the
        // WMX3 manual does not define. It also means that once jog_run_time_ms
        // elapses the axis stays stopped until the operator releases and
        // presses again, which is the intended behavior.
        it->second.deadline = deadline;
        continue;
      }
    }

    // acc/dec stay accelerations on the wire and are converted to ramp times
    // here, so AxisVelocity keeps the same meaning as on wmx/axis/velocity.
    const double accTimeMs = (msg->acc[i] > 0.0) ?
      std::abs(velocity) / msg->acc[i] * 1000.0 : 0.0;
    const double decTimeMs = (msg->dec[i] > 0.0) ?
      std::abs(velocity) / msg->dec[i] * 1000.0 : 0.0;

    std::string message;
    if (api_->startJog(
        axis, velocity, accTimeMs, decTimeMs, jogJerkRatio_, jogRunTimeMs_, message) !=
      ErrorCode::None)
    {
      continue;
    }

    std::lock_guard<std::mutex> lock(jogMutex_);
    jogState_.insert_or_assign(axis, JogState{velocity, deadline});
  }
}

// Dead-man: stop every axis whose jog refresh has expired.
void WmxCoreMotionNode::jogWatchdogStep()
{
  if (!isNodeActive()) {
    return;
  }

  const rclcpp::Time now = this->now();
  std::vector<int> expired;

  {
    std::lock_guard<std::mutex> lock(jogMutex_);
    for (auto it = jogState_.begin(); it != jogState_.end(); ) {
      if (now >= it->second.deadline) {
        expired.push_back(it->first);
        it = jogState_.erase(it);
      } else {
        ++it;
      }
    }
  }

  for (const int axis : expired) {
    api_->stopAxis(axis);
  }
}

void WmxCoreMotionNode::stopAxesCallback(
  const std::shared_ptr<wmx_r2_message::srv::SetAxis::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxis::Response> response)
{
  if (!isNodeActive()) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < request->index.size(); ++i) {
    const int axis = request->index[i];

    {
      std::lock_guard<std::mutex> lock(jogMutex_);
      jogState_.erase(axis);
    }

    const int err = api_->stopAxis(axis);
    if (err != ErrorCode::None) {
      msg_stream << "Failed to stop axis " << axis << ". Error=" << err << "; ";
      all_success = false;
    } else {
      msg_stream << "Stopped axis " << axis << "; ";
    }
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

void WmxCoreMotionNode::setAxisOnCallback(
  const std::shared_ptr<wmx_r2_message::srv::SetAxis::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxis::Response> response)
{
  if (!isNodeActive()) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < request->index.size(); ++i) {
    std::string message;
    if (api_->setServoOn(request->index[i], request->data[i], message) != ErrorCode::None) {
      all_success = false;
    }
    msg_stream << message << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

void WmxCoreMotionNode::setAxisModeCallback(
  const std::shared_ptr<wmx_r2_message::srv::SetAxis::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxis::Response> response)
{
  if (!isNodeActive()) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < request->index.size(); ++i) {
    std::string message;
    if (api_->setAxisCommandMode(request->index[i], request->data[i], message) !=
      ErrorCode::None)
    {
      all_success = false;
    }
    msg_stream << message << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

void WmxCoreMotionNode::clearAlarmCallback(
  const std::shared_ptr<wmx_r2_message::srv::SetAxis::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxis::Response> response)
{
  if (!isNodeActive()) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < request->index.size(); ++i) {
    std::string message;
    if (api_->clearAmpAlarm(request->index[i], message) != ErrorCode::None) {
      all_success = false;
    }
    msg_stream << message << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

void WmxCoreMotionNode::setAxisPolarityCallback(
  const std::shared_ptr<wmx_r2_message::srv::SetAxis::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxis::Response> response)
{
  if (!isNodeActive()) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < request->index.size(); ++i) {
    std::string message;
    if (api_->setAxisPolarity(request->index[i], request->data[i], message) != ErrorCode::None) {
      all_success = false;
    }
    msg_stream << message << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

void WmxCoreMotionNode::setAxisGearRatioCallback(
  const std::shared_ptr<wmx_r2_message::srv::SetAxisGearRatio::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxisGearRatio::Response> response)
{
  if (!isNodeActive()) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < request->index.size(); ++i) {
    std::string message;
    if (api_->setGearRatio(
        request->index[i], request->numerator[i], request->denominator[i], message) !=
      ErrorCode::None)
    {
      all_success = false;
    }
    msg_stream << message << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

/* ros2 service call /wmx/axis/homing wmx_r2_message/srv/SetAxis \
     "{index: [0, 1], data: [0, 0]}" */
void WmxCoreMotionNode::setHomingCallback(
  const std::shared_ptr<wmx_r2_message::srv::SetAxis::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxis::Response> response)
{
  if (!isNodeActive()) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < request->index.size(); ++i) {
    std::string message;
    if (api_->homeAxis(request->index[i], message) != ErrorCode::None) {
      all_success = false;
    }
    msg_stream << message << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WmxCoreMotionNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
