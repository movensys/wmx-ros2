// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_core_motion_node.hpp"

#include <cmath>
#include <sstream>

using std::placeholders::_1;
using std::placeholders::_2;

using wmx3Api::AxisCommandMode;
using wmx3Api::CoreMotion;
using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;
using wmx3Api::ProfileType;

WmxCoreMotionNodeApi::WmxCoreMotionNodeApi(
  const rclcpp::Logger & logger, const Config & config)
: logger_(logger), config_(config)
{
}

WmxCoreMotionNodeApi::~WmxCoreMotionNodeApi()
{
  if (cm_) {
    stopAllJogs();
    releaseDevice();
  }
}

std::string WmxCoreMotionNodeApi::errorText(int err)
{
  char errString[256] = {};
  wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
  return errString;
}

int WmxCoreMotionNodeApi::attachDevice(std::string & message)
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

  message = "Attached to WMX3 device";
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

void WmxCoreMotionNodeApi::releaseDevice()
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
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!cm_) {
    return ErrorCode::DeviceIsNull;
  }
  return cm_->GetStatus(&status);
}

int WmxCoreMotionNodeApi::startPos(
  int axis, double target, double velocity, double acc, double dec, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!cm_) {
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

  const int err = cm_->motion->StartPos(&position);
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
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!cm_) {
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

  const int err = cm_->motion->StartMov(&position);
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
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!cm_) {
    message = "Cannot move axis " + std::to_string(axis) + ". Core motion is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  wmx3Api::Velocity::VelCommand command;
  command.axis = axis;
  command.profile.velocity = velocity;
  command.profile.type = ProfileType::T::Trapezoidal;
  command.profile.acc = acc;
  command.profile.dec = dec;

  const int err = cm_->velocity->StartVel(&command);
  if (err != ErrorCode::None) {
    message = "Failed to move velocity motor " + std::to_string(axis) + ". Error=" +
      std::to_string(err) + " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Axis " + std::to_string(axis) + " running at " + std::to_string(velocity);
  return ErrorCode::None;
}

int WmxCoreMotionNodeApi::startJog(
  int axis, double velocity, double acc, double dec, const rclcpp::Time & now,
  std::string & message)
{
  const rclcpp::Time deadline =
    now + rclcpp::Duration::from_nanoseconds(
    static_cast<int64_t>(config_.jogTimeoutMs * 1e6));

  if (velocity == 0.0) {
    bool wasJogging = false;
    {
      std::lock_guard<std::mutex> jogLock(jogMutex_);
      wasJogging = jogState_.erase(axis) > 0;
    }
    if (wasJogging) {
      stopAxis(axis);
    }
    message = "Stopped jog on axis " + std::to_string(axis);
    return ErrorCode::None;
  }

  {
    std::lock_guard<std::mutex> jogLock(jogMutex_);
    auto it = jogState_.find(axis);
    if (it != jogState_.end() && it->second.velocity == velocity) {
      it->second.deadline = deadline;
      message = "Refreshed jog on axis " + std::to_string(axis);
      return ErrorCode::None;
    }
  }

  const double accTimeMs = (acc > 0.0) ? std::abs(velocity) / acc * 1000.0 : 0.0;
  const double decTimeMs = (dec > 0.0) ? std::abs(velocity) / dec * 1000.0 : 0.0;

  {
    std::lock_guard<std::mutex> lock(deviceMutex_);

    if (!cm_) {
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
    jogCommand.profile.jerkAccRatio = config_.jogJerkRatio;
    jogCommand.profile.jerkDecRatio = config_.jogJerkRatio;
    jogCommand.profile.accTimeMilliseconds = accTimeMs;
    jogCommand.profile.decTimeMilliseconds = decTimeMs;
    jogCommand.profile.startingVelocity = 0;
    jogCommand.profile.endVelocity = 0;
    jogCommand.profile.secondVelocity = 0;
    jogCommand.profile.movingAverageTimeMilliseconds = 0;
    jogCommand.SetRunTime(config_.jogRunTimeMs);

    const int err = cm_->motion->StartJog(&jogCommand);
    if (err != ErrorCode::None) {
      message = "Failed to jog motor " + std::to_string(axis) + ". Error=" +
        std::to_string(err) + " (" + errorText(err) + ")";
      RCLCPP_ERROR(logger_, "%s", message.c_str());
      return err;
    }
  }

  std::lock_guard<std::mutex> jogLock(jogMutex_);
  jogState_.insert_or_assign(axis, JogState{velocity, deadline});
  message = "Jogging axis " + std::to_string(axis);
  return ErrorCode::None;
}

void WmxCoreMotionNodeApi::stopExpiredJogs(const rclcpp::Time & now)
{
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
    stopAxis(axis);
  }
}

void WmxCoreMotionNodeApi::stopAllJogs()
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
    stopAxis(axis);
  }
}

void WmxCoreMotionNodeApi::clearJog(int axis)
{
  std::lock_guard<std::mutex> lock(jogMutex_);
  jogState_.erase(axis);
}

int WmxCoreMotionNodeApi::stopAxis(int axis)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!cm_) {
    return ErrorCode::DeviceIsNull;
  }

  const int err = cm_->motion->Stop(axis);
  if (err != ErrorCode::None) {
    RCLCPP_DEBUG(
      logger_, "Stop on axis %d returned %d (%s)", axis, err, errorText(err).c_str());
  }
  return err;
}

int WmxCoreMotionNodeApi::setServoOn(int axis, int on, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!cm_) {
    message = "Cannot set axis " + std::to_string(axis) + ". Core motion is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  const int err = cm_->axisControl->SetServoOn(axis, on, servoOnTimeout_);
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
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!cm_) {
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

  const int err = cm_->axisControl->SetAxisCommandMode(axis, commandMode);
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
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!cm_) {
    message = "Cannot clear alarm on axis " + std::to_string(axis) +
      ". Core motion is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  const int err = cm_->axisControl->ClearAmpAlarm(axis);
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
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!cm_) {
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

  const int err = cm_->config->SetAxisPolarity(axis, polarity);
  if (err != ErrorCode::None) {
    message = "Failed to set polarity on axis " + std::to_string(axis);
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
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!cm_) {
    message = "Cannot set gear ratio on axis " + std::to_string(axis) +
      ". Core motion is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  const int err = cm_->config->SetGearRatio(axis, numerator, denominator);
  if (err != ErrorCode::None) {
    message = "Failed to set gear ratio on axis " + std::to_string(axis);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Set gear ratio on axis " + std::to_string(axis);
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxCoreMotionNodeApi::startHome(int axis, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!cm_) {
    message = "Cannot home axis " + std::to_string(axis) + ". Core motion is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  wmx3Api::Config::HomeParam homeParam;
  cm_->config->GetHomeParam(axis, &homeParam);
  homeParam.homeType = wmx3Api::Config::HomeType::CurrentPos;
  cm_->config->SetHomeParam(axis, &homeParam);
  cm_->home->StartHome(axis);

  const int err = cm_->motion->Wait(axis);
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

int WmxCoreMotionNodeApi::loadWmxParams(const std::string & path, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!cm_) {
    message = "Cannot load params. Core motion is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  wmx3Api::Config::SystemParam sysParamErr;
  wmx3Api::Config::AxisParam axisParamErr;

  std::vector<char> pathBuffer(path.begin(), path.end());
  pathBuffer.push_back('\0');

  const int err = cm_->config->ImportAndSetAll(pathBuffer.data(), &sysParamErr, &axisParamErr);
  if (err != ErrorCode::None) {
    message = "Failed to load params: " + errorText(err);
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Loaded params from: " + path;
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxCoreMotionNodeApi::getWmxParams(
  const std::vector<int32_t> & axes, std::vector<std::string> & dump, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!cm_) {
    message = "Cannot get params. Core motion is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  for (const int32_t i : axes) {
    if (i < 0 || i >= wmx3Api::constants::maxAxes) {
      message = "Axis " + std::to_string(i) + " is out of range (0.." +
        std::to_string(wmx3Api::constants::maxAxes - 1) + ")";
      RCLCPP_ERROR(logger_, "%s", message.c_str());
      return ErrorCode::AxisOutOfRange;
    }
  }

  wmx3Api::Config::SystemParam sysParam;
  wmx3Api::Config::AxisParam axisParam;

  cm_->config->GetParam(&sysParam);
  cm_->config->GetAxisParam(&axisParam);

  for (const int32_t i : axes) {
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

  message = "OK";
  return ErrorCode::None;
}

WmxCoreMotionNode::WmxCoreMotionNode()
: Node("wmx_core_motion_node")
{
  WmxCoreMotionNodeApi::Config config;
  config.jogTimeoutMs = this->declare_parameter("jog_timeout_ms", 200.0);
  config.jogRunTimeMs = this->declare_parameter("jog_run_time_ms", 2000.0);
  config.jogJerkRatio = this->declare_parameter("jog_jerk_ratio", 0.75);

  api_ = std::make_unique<WmxCoreMotionNodeApi>(this->get_logger(), config);

  init_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  homing_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = init_cb_group_;

  auto ready_qos = rclcpp::QoS(1).reliable().transient_local();
  engineReadySub_ = this->create_subscription<std_msgs::msg::Bool>(
    "wmx/engine/ready", ready_qos,
    std::bind(&WmxCoreMotionNode::onEngineReady, this, _1), sub_opts);

  coreMotionReadyPub_ = this->create_publisher<std_msgs::msg::Bool>(
    "wmx/core_motion/ready", ready_qos);

  setServoOnService_ = this->create_service<wmx_r2_message::srv::SetAxes>(
    "wmx/axes/set_servo_on",
    std::bind(&WmxCoreMotionNode::setServoOnCallback, this, _1, _2));

  clearAmpAlarmService_ = this->create_service<wmx_r2_message::srv::SetAxes>(
    "wmx/axes/clear_amp_alarm",
    std::bind(&WmxCoreMotionNode::clearAmpAlarmCallback, this, _1, _2));

  setAxisCommandModeService_ = this->create_service<wmx_r2_message::srv::SetAxes>(
    "wmx/axes/set_axis_command_mode",
    std::bind(&WmxCoreMotionNode::setAxisCommandModeCallback, this, _1, _2));

  setAxisPolarityService_ = this->create_service<wmx_r2_message::srv::SetAxes>(
    "wmx/axes/set_axis_polarity",
    std::bind(&WmxCoreMotionNode::setAxisPolarityCallback, this, _1, _2));

  setGearRatioService_ = this->create_service<wmx_r2_message::srv::SetAxesGearRatio>(
    "wmx/axes/set_gear_ratio",
    std::bind(&WmxCoreMotionNode::setGearRatioCallback, this, _1, _2));

  startHomeService_ = this->create_service<wmx_r2_message::srv::SetAxes>(
    "wmx/axes/start_home",
    std::bind(&WmxCoreMotionNode::startHomeCallback, this, _1, _2),
    rclcpp::ServicesQoS(), homing_cb_group_);

  stopAxisService_ = this->create_service<wmx_r2_message::srv::SetAxes>(
    "wmx/axes/stop",
    std::bind(&WmxCoreMotionNode::stopAxisCallback, this, _1, _2));

  loadWmxParamsService_ = this->create_service<wmx_r2_message::srv::LoadWmxParams>(
    "wmx/core_motion/load_wmx_params",
    std::bind(&WmxCoreMotionNode::loadWmxParamsCallback, this, _1, _2));

  getWmxParamsService_ = this->create_service<wmx_r2_message::srv::GetWmxParams>(
    "wmx/core_motion/get_wmx_params",
    std::bind(&WmxCoreMotionNode::getWmxParamsCallback, this, _1, _2));

  RCLCPP_INFO(this->get_logger(), "wmx_core_motion_node waiting for engine...");
}

WmxCoreMotionNode::~WmxCoreMotionNode()
{
  if (axesStatusTimer_) {
    axesStatusTimer_->cancel();
  }
  if (jogWatchdogTimer_) {
    jogWatchdogTimer_->cancel();
  }
  api_.reset();
  RCLCPP_INFO(this->get_logger(), "wmx_core_motion_node stopped");
}

bool WmxCoreMotionNode::isReady()
{
  return initialized_.load();
}

std::string WmxCoreMotionNode::notReadyMessage()
{
  return "CoreMotion not initialized. Engine not ready.";
}

void WmxCoreMotionNode::onEngineReady(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data || initialized_) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Engine ready — initializing CoreMotion...");

  std::string message;
  if (api_->attachDevice(message) != ErrorCode::None) {
    return;
  }

  axisCount_ = api_->readAxisCount();
  if (axisCount_ <= 0) {
    RCLCPP_WARN(this->get_logger(), "Engine reported 0 axes; axes status will be empty.");
  }

  axesStatusPub_ = this->create_publisher<wmx_r2_message::msg::AxesStatus>(
    "wmx/axes/status", 1);

  startVelSub_ = this->create_subscription<wmx_r2_message::msg::AxesVelocity>(
    "wmx/axes/start_vel", 1,
    std::bind(&WmxCoreMotionNode::startVelCallback, this, _1));

  startPosSub_ = this->create_subscription<wmx_r2_message::msg::AxesPose>(
    "wmx/axes/start_pos", 1,
    std::bind(&WmxCoreMotionNode::startPosCallback, this, _1));

  startMovSub_ = this->create_subscription<wmx_r2_message::msg::AxesPose>(
    "wmx/axes/start_mov", 1,
    std::bind(&WmxCoreMotionNode::startMovCallback, this, _1));

  startJogSub_ = this->create_subscription<wmx_r2_message::msg::AxesVelocity>(
    "wmx/axes/start_jog", 1,
    std::bind(&WmxCoreMotionNode::startJogCallback, this, _1));

  axesStatusTimer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / rate_),
    std::bind(&WmxCoreMotionNode::axesStatusStep, this));

  jogWatchdogTimer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&WmxCoreMotionNode::jogWatchdogStep, this));

  initialized_ = true;

  auto ready_msg = std_msgs::msg::Bool();
  ready_msg.data = true;
  coreMotionReadyPub_->publish(ready_msg);

  engineReadySub_.reset();

  RCLCPP_INFO(
    this->get_logger(), "wmx_core_motion_node is ready (%d axes, %d Hz)", axisCount_, rate_);
}

void WmxCoreMotionNode::axesStatusStep()
{
  axesStatusMsg_.amp_alarms.clear();
  axesStatusMsg_.servo_on.clear();
  axesStatusMsg_.home_done.clear();
  axesStatusMsg_.motion_complete.clear();
  axesStatusMsg_.negative_ls.clear();
  axesStatusMsg_.positive_ls.clear();
  axesStatusMsg_.home_switch.clear();
  axesStatusMsg_.position_commands.clear();
  axesStatusMsg_.velocity_commands.clear();
  axesStatusMsg_.actual_positions.clear();
  axesStatusMsg_.actual_velocities.clear();
  axesStatusMsg_.actual_torques.clear();

  axesStatusMsg_.header.stamp = this->now();
  axesStatusMsg_.header.frame_id = "base_link";

  wmx3Api::CoreMotionStatus status;
  api_->getStatus(status);

  for (int i = 0; i < axisCount_; ++i) {
    axesStatusMsg_.amp_alarms.push_back(status.axesStatus[i].ampAlarm);
    axesStatusMsg_.servo_on.push_back(status.axesStatus[i].servoOn);
    axesStatusMsg_.home_done.push_back(status.axesStatus[i].homeDone);
    axesStatusMsg_.motion_complete.push_back(status.axesStatus[i].motionComplete);
    axesStatusMsg_.negative_ls.push_back(status.axesStatus[i].negativeLS);
    axesStatusMsg_.positive_ls.push_back(status.axesStatus[i].positiveLS);
    axesStatusMsg_.home_switch.push_back(status.axesStatus[i].homeSwitch);
    axesStatusMsg_.position_commands.push_back(status.axesStatus[i].posCmd);
    axesStatusMsg_.velocity_commands.push_back(status.axesStatus[i].velocityCmd);
    axesStatusMsg_.actual_positions.push_back(status.axesStatus[i].actualPos);
    axesStatusMsg_.actual_velocities.push_back(status.axesStatus[i].actualVelocity);
    axesStatusMsg_.actual_torques.push_back(status.axesStatus[i].actualTorque);
  }
  axesStatusPub_->publish(axesStatusMsg_);
}

void WmxCoreMotionNode::startPosCallback(const wmx_r2_message::msg::AxesPose::SharedPtr msg)
{
  std::string message;
  for (size_t i = 0; i < msg->indices.size(); ++i) {
    api_->startPos(
      msg->indices[i], msg->positions[i], msg->velocities[i],
      msg->accelerations[i], msg->decelerations[i], message);
  }
}

void WmxCoreMotionNode::startMovCallback(const wmx_r2_message::msg::AxesPose::SharedPtr msg)
{
  std::string message;
  for (size_t i = 0; i < msg->indices.size(); ++i) {
    api_->startMov(
      msg->indices[i], msg->positions[i], msg->velocities[i],
      msg->accelerations[i], msg->decelerations[i], message);
  }
}

void WmxCoreMotionNode::startVelCallback(const wmx_r2_message::msg::AxesVelocity::SharedPtr msg)
{
  std::string message;
  for (size_t i = 0; i < msg->indices.size(); ++i) {
    api_->startVel(
      msg->indices[i], msg->velocities[i],
      msg->accelerations[i], msg->decelerations[i], message);
  }
}

void WmxCoreMotionNode::startJogCallback(const wmx_r2_message::msg::AxesVelocity::SharedPtr msg)
{
  if (!isReady()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Jog ignored: CoreMotion not initialized.");
    return;
  }

  const size_t axis_count = msg->indices.size();
  if (msg->velocities.size() != axis_count ||
    msg->accelerations.size() != axis_count ||
    msg->decelerations.size() != axis_count)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Jog ignored: index/velocity/acc/dec must be the same length.");
    return;
  }

  const rclcpp::Time now = this->now();
  std::string message;

  for (size_t i = 0; i < axis_count; ++i) {
    api_->startJog(
      msg->indices[i], msg->velocities[i], msg->accelerations[i],
      msg->decelerations[i], now, message);
  }
}

void WmxCoreMotionNode::jogWatchdogStep()
{
  if (!isReady()) {
    return;
  }
  api_->stopExpiredJogs(this->now());
}

void WmxCoreMotionNode::stopAxisCallback(
  const std::shared_ptr<wmx_r2_message::srv::SetAxes::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxes::Response> response)
{
  if (!isReady()) {
    response->success = false;
    response->message = notReadyMessage();
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (const int32_t axis : request->indices) {
    api_->clearJog(axis);

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

void WmxCoreMotionNode::setServoOnCallback(
  const std::shared_ptr<wmx_r2_message::srv::SetAxes::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxes::Response> response)
{
  if (!isReady()) {
    response->success = false;
    response->message = notReadyMessage();
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < request->indices.size(); ++i) {
    std::string message;
    if (api_->setServoOn(request->indices[i], request->data[i], message) != ErrorCode::None) {
      all_success = false;
    }
    msg_stream << message << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

void WmxCoreMotionNode::setAxisCommandModeCallback(
  const std::shared_ptr<wmx_r2_message::srv::SetAxes::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxes::Response> response)
{
  if (!isReady()) {
    response->success = false;
    response->message = notReadyMessage();
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < request->indices.size(); ++i) {
    std::string message;
    if (api_->setAxisCommandMode(
        request->indices[i], request->data[i], message) != ErrorCode::None)
    {
      all_success = false;
    }
    msg_stream << message << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

void WmxCoreMotionNode::clearAmpAlarmCallback(
  const std::shared_ptr<wmx_r2_message::srv::SetAxes::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxes::Response> response)
{
  if (!isReady()) {
    response->success = false;
    response->message = notReadyMessage();
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (const int32_t axis : request->indices) {
    std::string message;
    if (api_->clearAmpAlarm(axis, message) != ErrorCode::None) {
      all_success = false;
    }
    msg_stream << message << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

void WmxCoreMotionNode::setAxisPolarityCallback(
  const std::shared_ptr<wmx_r2_message::srv::SetAxes::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxes::Response> response)
{
  if (!isReady()) {
    response->success = false;
    response->message = notReadyMessage();
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < request->indices.size(); ++i) {
    std::string message;
    if (api_->setAxisPolarity(request->indices[i], request->data[i], message) != ErrorCode::None) {
      all_success = false;
    }
    msg_stream << message << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

void WmxCoreMotionNode::setGearRatioCallback(
  const std::shared_ptr<wmx_r2_message::srv::SetAxesGearRatio::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxesGearRatio::Response> response)
{
  if (!isReady()) {
    response->success = false;
    response->message = notReadyMessage();
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < request->indices.size(); ++i) {
    std::string message;
    if (api_->setGearRatio(
        request->indices[i], request->numerators[i], request->denominators[i],
        message) != ErrorCode::None)
    {
      all_success = false;
    }
    msg_stream << message << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

void WmxCoreMotionNode::startHomeCallback(
  const std::shared_ptr<wmx_r2_message::srv::SetAxes::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxes::Response> response)
{
  if (!isReady()) {
    response->success = false;
    response->message = notReadyMessage();
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (const int32_t axis : request->indices) {
    std::string message;
    if (api_->startHome(axis, message) != ErrorCode::None) {
      all_success = false;
    }
    msg_stream << message << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

void WmxCoreMotionNode::loadWmxParamsCallback(
  const std::shared_ptr<wmx_r2_message::srv::LoadWmxParams::Request> request,
  std::shared_ptr<wmx_r2_message::srv::LoadWmxParams::Response> response)
{
  if (!isReady()) {
    response->success = false;
    response->message = notReadyMessage();
    return;
  }

  std::string message;
  const int err = api_->loadWmxParams(request->file_path, message);

  response->success = (err == ErrorCode::None);
  response->message = message;
}

void WmxCoreMotionNode::getWmxParamsCallback(
  const std::shared_ptr<wmx_r2_message::srv::GetWmxParams::Request> request,
  std::shared_ptr<wmx_r2_message::srv::GetWmxParams::Response> response)
{
  if (!isReady()) {
    response->success = false;
    response->message = notReadyMessage();
    return;
  }

  std::string message;
  const int err = api_->getWmxParams(request->indices, response->params_dump, message);

  response->success = (err == ErrorCode::None);
  response->message = message;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WmxCoreMotionNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
