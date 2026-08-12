// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_core_motion_node.hpp"

using wmx3Api::AxisCommandMode;
using wmx3Api::Config;
using wmx3Api::CoreMotion;
using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;
using wmx3Api::ProfileType;

void WmxCoreMotionNode::axisStateStep()
{
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

  wmx3LibCm_->GetStatus(&cmStatus_);

  for (int i = 0; i < axisCount_; ++i) {
    axisStateMsg_.amp_alarm.push_back(cmStatus_.axesStatus[i].ampAlarm);
    axisStateMsg_.servo_on.push_back(cmStatus_.axesStatus[i].servoOn);
    axisStateMsg_.home_done.push_back(cmStatus_.axesStatus[i].homeDone);
    axisStateMsg_.motion_complete.push_back(cmStatus_.axesStatus[i].motionComplete);
    axisStateMsg_.negative_ls.push_back(cmStatus_.axesStatus[i].negativeLS);
    axisStateMsg_.positive_ls.push_back(cmStatus_.axesStatus[i].positiveLS);
    axisStateMsg_.home_switch.push_back(cmStatus_.axesStatus[i].homeSwitch);
    axisStateMsg_.pos_cmd.push_back(cmStatus_.axesStatus[i].posCmd);
    axisStateMsg_.velocity_cmd.push_back(cmStatus_.axesStatus[i].velocityCmd);
    axisStateMsg_.actual_pos.push_back(cmStatus_.axesStatus[i].actualPos);
    axisStateMsg_.actual_velocity.push_back(cmStatus_.axesStatus[i].actualVelocity);
    axisStateMsg_.actual_torque.push_back(cmStatus_.axesStatus[i].actualTorque);
  }
  axisStatePub_->publish(axisStateMsg_);
}

void WmxCoreMotionNode::axisPoseCallback(const wmx_r2_message::msg::AxisPose::SharedPtr msg)
{
  if (!isActive_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Position command ignored: %s", notActiveMessage().c_str());
    return;
  }

  size_t axis_count = msg->index.size();
  for (size_t i = 0; i < axis_count; i++) {
    position_.axis = msg->index[i];
    position_.target = msg->target[i];
    position_.profile.velocity = msg->velocity[i];
    position_.profile.type = ProfileType::T::Trapezoidal;
    position_.profile.acc = msg->acc[i];
    position_.profile.dec = msg->dec[i];

    err_ = wmx3LibCm_->motion->StartPos(&position_);
    if (err_ != ErrorCode::None) {
      wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to move position motor %d. Error=%d (%s)",
        msg->index[i], err_, errString_);
    }
  }
}

void WmxCoreMotionNode::axisPoseRelativeCallback(
  const wmx_r2_message::msg::AxisPose::SharedPtr msg)
{
  if (!isActive_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Relative position command ignored: %s", notActiveMessage().c_str());
    return;
  }

  size_t axis_count = msg->index.size();
  for (size_t i = 0; i < axis_count; i++) {
    position_.axis = msg->index[i];
    position_.target = msg->target[i];
    position_.profile.velocity = msg->velocity[i];
    position_.profile.type = ProfileType::T::Trapezoidal;
    position_.profile.acc = msg->acc[i];
    position_.profile.dec = msg->dec[i];

    err_ = wmx3LibCm_->motion->StartMov(&position_);
    if (err_ != ErrorCode::None) {
      wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to move relative motor %d. Error=%d (%s)",
        msg->index[i], err_, errString_);
    }
  }
}

void WmxCoreMotionNode::axisVelCallback(const wmx_r2_message::msg::AxisVelocity::SharedPtr msg)
{
  if (!isActive_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Velocity command ignored: %s", notActiveMessage().c_str());
    return;
  }

  size_t axis_count = msg->index.size();
  for (size_t i = 0; i < axis_count; i++) {
    velocity_.axis = msg->index[i];
    velocity_.profile.velocity = msg->velocity[i];
    velocity_.profile.type = ProfileType::T::Trapezoidal;
    velocity_.profile.acc = msg->acc[i];
    velocity_.profile.dec = msg->dec[i];

    err_ = wmx3LibCm_->velocity->StartVel(&velocity_);
    if (err_ != ErrorCode::None) {
      wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to move velocity motor %d. Error=%d (%s)",
        msg->index[i], err_, errString_);
    }
  }
}

// Jog command. The publisher (keyboard/joystick teleop, CLI, ...) must keep
// republishing while the operator holds the control; jogWatchdogStep() stops
// the axis once refreshes stop arriving. Velocity sign selects the direction.
void WmxCoreMotionNode::axisJogCallback(const wmx_r2_message::msg::AxisVelocity::SharedPtr msg)
{
  if (!isActive_) {
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
        stopAxis(axis);
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

    // Same profile WOS uses for its jog buttons. A run time requires a
    // time-based profile: Trapezoidal is rejected with ProfileTypeNotSupported.
    // acc/dec stay accelerations on the wire and are converted to ramp times
    // here, so AxisVelocity keeps the same meaning as on wmx/axis/velocity.
    const double accTimeMs = (msg->acc[i] > 0.0) ?
      std::abs(velocity) / msg->acc[i] * 1000.0 : 0.0;
    const double decTimeMs = (msg->dec[i] > 0.0) ?
      std::abs(velocity) / msg->dec[i] * 1000.0 : 0.0;

    wmx3Api::Motion::JogCommand jogCommand = wmx3Api::Motion::JogCommand();
    jogCommand.axis = axis;
    jogCommand.profile.type = ProfileType::T::TimeAccJerkRatio;
    jogCommand.profile.velocity = velocity;
    jogCommand.profile.acc = 0;
    jogCommand.profile.dec = 0;
    jogCommand.profile.jerkAcc = 0;
    jogCommand.profile.jerkDec = 0;
    jogCommand.profile.jerkAccRatio = jogJerkRatio_;
    jogCommand.profile.jerkDecRatio = jogJerkRatio_;
    jogCommand.profile.accTimeMilliseconds = accTimeMs;
    jogCommand.profile.decTimeMilliseconds = decTimeMs;
    jogCommand.profile.startingVelocity = 0;
    jogCommand.profile.endVelocity = 0;
    jogCommand.profile.secondVelocity = 0;
    jogCommand.profile.movingAverageTimeMilliseconds = 0;
    // Backstop: if this node dies mid-jog the engine still decelerates the axis.
    // Once it elapses the axis stays stopped until the operator releases and
    // presses again, because a held key only refreshes the dead-man deadline.
    jogCommand.SetRunTime(jogRunTimeMs_);

    const int err = wmx3LibCm_->motion->StartJog(&jogCommand);
    if (err != ErrorCode::None) {
      char errString[256];
      wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to jog motor %d. Error=%d (%s)", axis, err, errString);
      continue;
    }

    std::lock_guard<std::mutex> lock(jogMutex_);
    jogState_.insert_or_assign(axis, JogState{velocity, deadline});
  }
}

// Dead-man: stop every axis whose jog refresh has expired.
void WmxCoreMotionNode::jogWatchdogStep()
{
  if (!isActive_) {
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
    stopAxis(axis);
  }
}

void WmxCoreMotionNode::stopAxes(
  const std::shared_ptr<wmx_r2_message::srv::SetAxis::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxis::Response> response)
{
  if (!isActive_) {
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

    const int err = stopAxis(axis);
    if (err != ErrorCode::None) {
      char errString[256];
      wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
      msg_stream << "Failed to stop axis " << axis
                 << ". Error=" << err << " (" << errString << "); ";
      all_success = false;
    } else {
      msg_stream << "Stopped axis " << axis << "; ";
    }
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

void WmxCoreMotionNode::setAxisOn(
  const std::shared_ptr<wmx_r2_message::srv::SetAxis::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxis::Response> response)
{
  if (!isActive_) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < request->index.size(); ++i) {
    int axis_index = request->index[i];
    int on_off = request->data[i];

    err_ = wmx3LibCm_->axisControl->SetServoOn(axis_index, on_off, 1000);
    if (err_ != ErrorCode::None) {
      wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
      snprintf(
        buffer_, sizeof(buffer_),
        "Failed to set axis %d %s. Error=%d (%s)",
        axis_index, on_off ? "on" : "off", err_, errString_);
      RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
      all_success = false;
    } else {
      snprintf(
        buffer_, sizeof(buffer_), "Set axis %d %s",
        axis_index, on_off ? "on" : "off");
      RCLCPP_INFO(this->get_logger(), "%s", buffer_);
    }
    msg_stream << buffer_ << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

void WmxCoreMotionNode::setAxisMode(
  const std::shared_ptr<wmx_r2_message::srv::SetAxis::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxis::Response> response)
{
  if (!isActive_) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < request->index.size(); ++i) {
    int axis_index = request->index[i];
    int mode_value = request->data[i];

    if (mode_value == 0) {
      err_ = wmx3LibCm_->axisControl->SetAxisCommandMode(
        axis_index, AxisCommandMode::Position);
    } else if (mode_value == 1) {
      err_ = wmx3LibCm_->axisControl->SetAxisCommandMode(
        axis_index, AxisCommandMode::Velocity);
    } else {
      snprintf(
        buffer_, sizeof(buffer_),
        "Invalid mode %d for axis %d", mode_value, axis_index);
      RCLCPP_WARN(this->get_logger(), "%s", buffer_);
      msg_stream << buffer_ << "; ";
      all_success = false;
      continue;
    }

    if (err_ != ErrorCode::None) {
      wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
      snprintf(
        buffer_, sizeof(buffer_),
        "Failed to set axis %d mode %d. Error=%d (%s)",
        axis_index, mode_value, err_, errString_);
      RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
      all_success = false;
    } else {
      const char * mode_str = (mode_value == 0) ? "Position" : "Velocity";
      snprintf(buffer_, sizeof(buffer_), "Set axis %d in %s mode", axis_index, mode_str);
      RCLCPP_INFO(this->get_logger(), "%s", buffer_);
    }
    msg_stream << buffer_ << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

void WmxCoreMotionNode::clearAlarm(
  const std::shared_ptr<wmx_r2_message::srv::SetAxis::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxis::Response> response)
{
  if (!isActive_) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < request->index.size(); ++i) {
    err_ = wmx3LibCm_->axisControl->ClearAmpAlarm(request->index[i]);
    if (err_ != ErrorCode::None) {
      wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
      snprintf(
        buffer_, sizeof(buffer_),
        "Failed to clear alarm axis %d. Error=%d (%s)",
        request->index[i], err_, errString_);
      RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
      all_success = false;
    } else {
      snprintf(buffer_, sizeof(buffer_), "Cleared alarm axis %d", request->index[i]);
      RCLCPP_INFO(this->get_logger(), "%s", buffer_);
    }
    msg_stream << buffer_ << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

void WmxCoreMotionNode::setAxisPolarity(
  const std::shared_ptr<wmx_r2_message::srv::SetAxis::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxis::Response> response)
{
  if (!isActive_) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < request->index.size(); ++i) {
    if (request->data[i] != 1 && request->data[i] != -1) {
      snprintf(
        buffer_, sizeof(buffer_),
        "Invalid polarity value for axis %d: %d",
        request->index[i], request->data[i]);
      RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
      msg_stream << buffer_ << "; ";
      all_success = false;
      continue;
    }

    err_ = wmx3LibCm_->config->SetAxisPolarity(request->index[i], request->data[i]);
    if (err_ != ErrorCode::None) {
      snprintf(
        buffer_, sizeof(buffer_),
        "Failed to set polarity on axis %d", request->index[i]);
      RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
      all_success = false;
    } else {
      snprintf(buffer_, sizeof(buffer_), "Set polarity on axis %d", request->index[i]);
      RCLCPP_INFO(this->get_logger(), "%s", buffer_);
    }
    msg_stream << buffer_ << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

void WmxCoreMotionNode::setAxisGearRatio(
  const std::shared_ptr<wmx_r2_message::srv::SetAxisGearRatio::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxisGearRatio::Response> response)
{
  if (!isActive_) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < request->index.size(); ++i) {
    err_ = wmx3LibCm_->config->SetGearRatio(
      request->index[i], request->numerator[i], request->denominator[i]);
    if (err_ != ErrorCode::None) {
      all_success = false;
      snprintf(
        buffer_, sizeof(buffer_),
        "Failed to set gear ratio on axis %d", request->index[i]);
      RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
    } else {
      snprintf(buffer_, sizeof(buffer_), "Set gear ratio on axis %d", request->index[i]);
      RCLCPP_INFO(this->get_logger(), "%s", buffer_);
    }
    msg_stream << buffer_ << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}

// ros2 service call /wmx/axis/homing wmx_r2_message/srv/SetAxis \
//   "{index: [0, 1], data: [0, 0]}"
void WmxCoreMotionNode::setHoming(
  const std::shared_ptr<wmx_r2_message::srv::SetAxis::Request> request,
  std::shared_ptr<wmx_r2_message::srv::SetAxis::Response> response)
{
  if (!isActive_) {
    response->success = false;
    response->message = notActiveMessage();
    return;
  }

  bool all_success = true;
  std::stringstream msg_stream;

  for (size_t i = 0; i < request->index.size(); ++i) {
    wmx3LibCm_->config->GetHomeParam(request->index[i], &homeParam_);
    homeParam_.homeType = Config::HomeType::CurrentPos;
    wmx3LibCm_->config->SetHomeParam(request->index[i], &homeParam_);
    wmx3LibCm_->home->StartHome(request->index[i]);
    err_ = wmx3LibCm_->motion->Wait(request->index[i]);

    if (err_ != ErrorCode::None) {
      wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
      snprintf(
        buffer_, sizeof(buffer_),
        "Failed to home axis %d. Error=%d (%s)",
        request->index[i], err_, errString_);
      RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
      all_success = false;
    } else {
      snprintf(buffer_, sizeof(buffer_), "Homed axis %d", request->index[i]);
      RCLCPP_INFO(this->get_logger(), "%s", buffer_);
    }
    msg_stream << buffer_ << "; ";
  }

  response->success = all_success;
  response->message = msg_stream.str();
}
